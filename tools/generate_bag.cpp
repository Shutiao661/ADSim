// =============================================================================
//  generate_bag.cpp — 合成路测数据集生成器
//
//  用途：
//    1. 在没有实车数据的环境下提供可复现的测试输入；
//    2. 按倍数放大生成 GB 级数据，用于验证管道的吞吐与内存表现。
//
//  生成内容模拟一段真实路测：车辆沿弯曲道路行驶，同步录制
//  激光雷达（含噪声与孤立离群点）、GPS（含多路径跳点）、车辆状态。
// =============================================================================
#include "adsim/datapipeline/BagCompression.h"
#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/RosBagWriter.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

struct Options {
  std::string output{"synthetic.bag"};
  int lidar_frames{200};
  int lidar_points{20000};
  int gps_rate_hz{10};
  int odom_rate_hz{50};
  int lidar_rate_hz{10};
  double duration_seconds{20.0};
  double gps_noise_meters{0.8};
  double gps_outlier_rate{0.02};
  double gps_outlier_meters{25.0};
  double lidar_noise_meters{0.02};
  double lidar_outlier_rate{0.005};
  std::uint32_t seed{20240924u};
  std::string compression{"none"};  ///< none / bz2 / lz4
  bool quiet{false};
};

void printUsage() {
  std::cout << R"(用法: adsim_gen_bag [选项]

选项:
  --output=<路径>          输出 bag 路径           (默认 synthetic.bag)
  --duration=<秒>          录制时长                (默认 20)
  --lidar-rate=<Hz>        点云频率                (默认 10)
  --gps-rate=<Hz>          GPS 频率                (默认 10)
  --odom-rate=<Hz>         车辆状态频率            (默认 50)
  --lidar-points=<N>       每帧点数                (默认 20000)
  --gps-noise=<米>         GPS 噪声标准差          (默认 0.8)
  --gps-outlier-rate=<p>   GPS 跳点比例            (默认 0.02)
  --lidar-outlier-rate=<p> 点云离群点比例          (默认 0.005)
  --seed=<N>               随机种子                (默认 20240924)
  --compression=<方式>     Chunk 压缩: none|bz2|lz4 (默认 none)
  --quiet                  安静模式
  --help, -h               显示帮助
)" << std::endl;
}

/// 解析 --key=value 形式的参数
bool parseArg(const std::string& arg, const std::string& key, std::string& value) {
  const std::string prefix = "--" + key + "=";
  if (arg.rfind(prefix, 0) == 0) {
    value = arg.substr(prefix.size());
    return true;
  }
  return false;
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string value;

    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else if (arg == "--quiet") {
      options.quiet = true;
    } else if (parseArg(arg, "output", value)) {
      options.output = value;
    } else if (parseArg(arg, "duration", value)) {
      options.duration_seconds = std::stod(value);
    } else if (parseArg(arg, "lidar-rate", value)) {
      options.lidar_rate_hz = std::stoi(value);
    } else if (parseArg(arg, "gps-rate", value)) {
      options.gps_rate_hz = std::stoi(value);
    } else if (parseArg(arg, "odom-rate", value)) {
      options.odom_rate_hz = std::stoi(value);
    } else if (parseArg(arg, "lidar-points", value)) {
      options.lidar_points = std::stoi(value);
    } else if (parseArg(arg, "gps-noise", value)) {
      options.gps_noise_meters = std::stod(value);
    } else if (parseArg(arg, "gps-outlier-rate", value)) {
      options.gps_outlier_rate = std::stod(value);
    } else if (parseArg(arg, "lidar-outlier-rate", value)) {
      options.lidar_outlier_rate = std::stod(value);
    } else if (parseArg(arg, "seed", value)) {
      options.seed = static_cast<std::uint32_t>(std::stoul(value));
    } else if (parseArg(arg, "compression", value)) {
      options.compression = value;
    } else {
      std::cerr << "未知参数: " << arg << std::endl;
      printUsage();
      std::exit(1);
    }
  }

  // 以点云帧数为准推算时长，保证三路数据覆盖同一时间区间
  options.lidar_frames =
      static_cast<int>(options.duration_seconds * options.lidar_rate_hz);
  return options;
}

/// 车辆行驶轨迹：一条带缓和曲线的 S 形道路
struct GroundTruth {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double speed{0.0};

  static GroundTruth at(double t, double speed) {
    GroundTruth g;
    const double distance = speed * t;

    // 曲率随里程正弦变化，模拟连续弯道
    const double kappa = 0.02 * std::sin(distance * 0.05);
    g.theta = 0.02 * (1.0 - std::cos(distance * 0.05)) / 0.05;

    // 沿航向积分得到位置
    g.x = distance * std::cos(g.theta * 0.5);
    g.y = 12.0 * std::sin(distance * 0.03);
    g.speed = speed;
    (void)kappa;
    return g;
  }
};

}  // namespace

int main(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);

  constexpr double kOriginLat = 31.2304;  // 上海
  constexpr double kOriginLon = 121.4737;
  constexpr double kOriginAlt = 10.0;
  constexpr adsim::Timestamp kStartTime = 1700000000000000000LL;

  const adsim::GeoProjector projector(kOriginLat, kOriginLon, kOriginAlt);
  const double nominal_speed = 11.0;  // m/s ≈ 40 km/h

  std::mt19937 rng(options.seed);
  std::normal_distribution<double> gps_noise(0.0, options.gps_noise_meters);
  std::normal_distribution<double> lidar_noise(0.0, options.lidar_noise_meters);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * adsim::kPi);
  std::uniform_real_distribution<double> range_dist(3.0, 60.0);
  std::uniform_real_distribution<double> outlier_jump(-options.gps_outlier_meters,
                                                      options.gps_outlier_meters);

  if (!options.quiet) {
    std::cout << "生成合成路测数据..." << std::endl;
    std::cout << "  输出      : " << options.output << std::endl;
    std::cout << "  时长      : " << options.duration_seconds << " s" << std::endl;
    std::cout << "  点云      : " << options.lidar_frames << " 帧 × "
              << options.lidar_points << " 点" << std::endl;
    std::cout << "  GPS       : " << static_cast<int>(options.duration_seconds *
                                                      options.gps_rate_hz) << " 帧"
              << " (噪声 σ=" << options.gps_noise_meters << "m, 跳点率 "
              << options.gps_outlier_rate * 100.0 << "%)" << std::endl;
  }

  const auto wall_start = std::chrono::steady_clock::now();

  // 压缩方式：写出的 Chunk 是否压缩。读取端需要本机构建支持对应算法。
  adsim::bag::Compression compression = adsim::bag::Compression::kNone;
  if (options.compression == "bz2") compression = adsim::bag::Compression::kBz2;
  else if (options.compression == "lz4") compression = adsim::bag::Compression::kLz4;
  else if (options.compression != "none") {
    std::cerr << "未知压缩方式: " << options.compression << "（可选 none / bz2 / lz4）"
              << std::endl;
    return 1;
  }

  if (compression != adsim::bag::Compression::kNone &&
      !adsim::bag::isCompressionSupported(compression)) {
    std::cerr << "本构建不支持 " << options.compression << " 压缩（支持: "
              << adsim::bag::supportedCompressions() << "）" << std::endl;
    return 1;
  }

  adsim::RosBagWriter::Config writer_config;
  writer_config.compression = compression;
  adsim::RosBagWriter writer(options.output, writer_config);

  for (const char* type : {"sensor_msgs/PointCloud2", "sensor_msgs/NavSatFix",
                           "nav_msgs/Odometry"}) {
    const adsim::ros::MessageTypeInfo info = adsim::ros::lookupMessageType(type);
    writer.addConnection(std::string("/") + (type == std::string("sensor_msgs/PointCloud2")
                                                 ? "lidar"
                                                 : (type == std::string("sensor_msgs/NavSatFix")
                                                        ? "gps"
                                                        : "odom")),
                         info.type, info.md5sum, info.message_definition);
  }

  std::size_t gps_written = 0;
  std::size_t odom_written = 0;
  std::size_t lidar_written = 0;
  std::size_t gps_outliers = 0;

  const int gps_count = static_cast<int>(options.duration_seconds * options.gps_rate_hz);
  const int odom_count = static_cast<int>(options.duration_seconds * options.odom_rate_hz);

  const auto gps_period = adsim::fromSeconds(1.0 / options.gps_rate_hz);
  const auto odom_period = adsim::fromSeconds(1.0 / options.odom_rate_hz);
  const auto lidar_period = adsim::fromSeconds(1.0 / options.lidar_rate_hz);

  // ---- 三路数据按时间序交织写出 ----
  //
  // 真实 rosbag 中不同话题的消息是按时间交错排列的，而不是按话题分块。
  // 早期版本按流分块写入（先全部 GPS、再全部 odom、最后全部点云），
  // 结果是任何按消息条数截断的消费者（例如 `adsim_cli tag --limit=N`）
  // 都只能看到排在最前面的那一路，其余话题被完全饿死。
  //
  // 这里改用 k 路归并：每轮挑出时间最早的一路写出。既保持了与实车录制
  // 一致的时序特性，也是流式实现——内存占用与录制时长无关。
  int gps_index = 0;
  int odom_index = 0;
  int lidar_index = 0;

  // 已耗尽的一路用极大时间戳占位，从而自然地从归并中退出
  constexpr adsim::Timestamp kExhausted = std::numeric_limits<adsim::Timestamp>::max();

  while (gps_index < gps_count || odom_index < odom_count ||
         lidar_index < options.lidar_frames) {
    const adsim::Timestamp gps_time =
        gps_index < gps_count ? kStartTime + gps_index * gps_period : kExhausted;
    const adsim::Timestamp odom_time =
        odom_index < odom_count ? kStartTime + odom_index * odom_period : kExhausted;
    const adsim::Timestamp lidar_time =
        lidar_index < options.lidar_frames ? kStartTime + lidar_index * lidar_period
                                           : kExhausted;

    // ---- GPS ----
    if (gps_time <= odom_time && gps_time <= lidar_time) {
      const double t = static_cast<double>(gps_index) / options.gps_rate_hz;
      const GroundTruth truth = GroundTruth::at(t, nominal_speed);

      double lat = 0.0, lon = 0.0, alt = 0.0;
      projector.toGeodetic({truth.x, truth.y, 0.0}, lat, lon, alt);

      adsim::GpsFrame frame;
      frame.stamp = gps_time;
      frame.latitude = lat + gps_noise(rng) * 1e-5;  // 约 1e-5 度 ≈ 1.1 m
      frame.longitude = lon + gps_noise(rng) * 1e-5;
      frame.altitude = kOriginAlt + alt;
      frame.heading = truth.theta;
      frame.speed = truth.speed;
      frame.hdop = 0.8;
      frame.fix_type = 3;

      // 按概率注入多路径跳点
      if (uniform(rng) < options.gps_outlier_rate) {
        frame.latitude += outlier_jump(rng) * 1e-5;
        frame.longitude += outlier_jump(rng) * 1e-5;
        ++gps_outliers;
      }

      const std::vector<std::uint8_t> payload = adsim::ros::encodeNavSatFix(frame);
      writer.writeMessage("/gps", frame.stamp, payload.data(), payload.size());
      ++gps_written;
      ++gps_index;
      continue;
    }

    // ---- 车辆状态 ----
    if (odom_time <= lidar_time) {
      const double t = static_cast<double>(odom_index) / options.odom_rate_hz;
      const GroundTruth truth = GroundTruth::at(t, nominal_speed);

      adsim::VehicleState state;
      state.stamp = odom_time;
      state.x = truth.x;
      state.y = truth.y;
      state.theta = truth.theta;
      state.speed = truth.speed;
      state.yaw_rate = 0.0;
      state.gear = 1;

      const std::vector<std::uint8_t> payload = adsim::ros::encodeOdometry(state);
      writer.writeMessage("/odom", state.stamp, payload.data(), payload.size());
      ++odom_written;
      ++odom_index;
      continue;
    }

    // ---- 点云：模拟两侧墙体 + 噪声 + 孤立离群点 ----
    {
      const double t = static_cast<double>(lidar_index) / options.lidar_rate_hz;
      const GroundTruth truth = GroundTruth::at(t, nominal_speed);

      adsim::LidarFrame frame;
      frame.stamp = lidar_time;
      frame.frame_id = "velodyne";
      frame.points.reserve(static_cast<std::size_t>(options.lidar_points));

      const int wall_points = options.lidar_points / 2;
      for (int i = 0; i < wall_points; ++i) {
        // 左右两侧墙体，局部坐标 y = ±5m
        const double along = range_dist(rng);
        const double side = (i % 2 == 0) ? 5.0 : -5.0;
        const double local_x = along * std::cos(angle_dist(rng) * 0.1);
        const double local_y = side + lidar_noise(rng);

        const double c = std::cos(truth.theta);
        const double s = std::sin(truth.theta);

        adsim::LidarPoint p;
        p.x = static_cast<float>(truth.x + local_x * c - local_y * s + lidar_noise(rng));
        p.y = static_cast<float>(truth.y + local_x * s + local_y * c + lidar_noise(rng));
        p.z = static_cast<float>(lidar_noise(rng) * 10.0 - 1.8);
        p.intensity = static_cast<float>(uniform(rng) * 255.0);
        frame.points.push_back(p);
      }

      // 孤立离群点：雨雾/扬尘产生的虚假回波，用于检验统计离群点去除
      const int outlier_count =
          static_cast<int>(options.lidar_points * options.lidar_outlier_rate);
      for (int i = 0; i < outlier_count; ++i) {
        adsim::LidarPoint p;
        p.x = static_cast<float>(truth.x + outlier_jump(rng) * 3.0);
        p.y = static_cast<float>(truth.y + outlier_jump(rng) * 3.0);
        p.z = static_cast<float>(outlier_jump(rng));
        p.intensity = 1.0f;
        frame.points.push_back(p);
      }

      const std::vector<std::uint8_t> payload = adsim::ros::encodePointCloud2(frame);
      writer.writeMessage("/lidar", frame.stamp, payload.data(), payload.size());
      ++lidar_written;
      ++lidar_index;
    }
  }

  writer.close();

  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();

  if (!options.quiet) {
    std::cout << "\n生成完成:" << std::endl;
    std::cout << "  GPS   : " << gps_written << " 帧 (含 " << gps_outliers << " 个跳点)"
              << std::endl;
    std::cout << "  状态  : " << odom_written << " 帧" << std::endl;
    std::cout << "  点云  : " << options.lidar_frames << " 帧" << std::endl;
    std::cout << "  Chunk : " << writer.chunkCount() << " (压缩: " << options.compression
              << ")" << std::endl;
    std::cout << "  耗时  : " << elapsed << " s" << std::endl;
  }
  return 0;
}
