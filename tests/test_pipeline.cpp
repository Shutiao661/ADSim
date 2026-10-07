// =============================================================================
//  test_pipeline.cpp — 数据处理管道单元测试
//  覆盖：时间对齐 / 点云降噪 / GPS 滤波 / 端到端管道
// =============================================================================
#include "TestFramework.h"

#include "adsim/datapipeline/DataPipeline.h"
#include "adsim/datapipeline/GpsFilter.h"
#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/PointCloudFilter.h"
#include "adsim/datapipeline/RosBagReader.h"
#include "adsim/datapipeline/RosBagWriter.h"
#include "adsim/datapipeline/TimeAligner.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace adsim;

namespace {

constexpr Timestamp kSec = 1000000000LL;

class TempFile {
 public:
  explicit TempFile(const std::string& name) : path_("/tmp/adsim_pipe_" + name) {}
  ~TempFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::vector<VehicleState> makeOdomTrack(int count, double rate_hz, Timestamp start) {
  std::vector<VehicleState> track;
  track.reserve(static_cast<std::size_t>(count));
  const double period = 1.0 / rate_hz;

  for (int i = 0; i < count; ++i) {
    const double t = i * period;
    VehicleState s;
    s.stamp = start + fromSeconds(t);
    s.x = 10.0 * t;
    s.y = 2.0 * std::sin(t);
    s.theta = normalizeAngle(0.2 * std::cos(t));
    s.speed = 10.0;
    track.push_back(s);
  }
  return track;
}

std::vector<GpsFrame> makeGpsTrack(const GeoProjector& projector, int count, double rate_hz,
                                   Timestamp start, double noise_sigma, unsigned seed) {
  std::vector<GpsFrame> track;
  track.reserve(static_cast<std::size_t>(count));
  const double period = 1.0 / rate_hz;

  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_sigma * 1e-5);  // 度

  for (int i = 0; i < count; ++i) {
    const double t = i * period;
    const double x = 10.0 * t;
    const double y = 2.0 * std::sin(t);

    double lat = 0.0, lon = 0.0, alt = 0.0;
    projector.toGeodetic({x, y, 0.0}, lat, lon, alt);

    GpsFrame g;
    g.stamp = start + fromSeconds(t);
    g.latitude = lat + noise(rng);
    g.longitude = lon + noise(rng);
    g.altitude = 10.0;
    g.hdop = 0.8;
    g.fix_type = 3;
    track.push_back(g);
  }
  return track;
}

}  // namespace

// ===========================================================================
//  时间对齐
// ===========================================================================

ADSIM_TEST(TimeAligner, 最近邻查找与容差) {
  const std::vector<Timestamp> stamps = {0, 100, 200, 300, 400};
  std::size_t index = 999;

  ADSIM_CHECK(TimeAligner::nearestIndex(stamps, 210, 50, index));
  ADSIM_CHECK_EQ(index, std::size_t(2));

  // 恰好等距时取更早的样本（实现约定：不窥探未来）
  ADSIM_CHECK(TimeAligner::nearestIndex(stamps, 150, 50, index));
  ADSIM_CHECK_EQ(index, std::size_t(1));

  // 超出容差应失败
  ADSIM_CHECK(!TimeAligner::nearestIndex(stamps, 1000, 50, index));

  // 空序列不得崩溃
  ADSIM_CHECK(!TimeAligner::nearestIndex({}, 0, 50, index));
}

ADSIM_TEST(TimeAligner, 区间定位) {
  const std::vector<Timestamp> stamps = {0, 100, 200, 300};
  std::size_t lo = 0, hi = 0;

  ADSIM_CHECK(TimeAligner::bracket(stamps, 150, lo, hi));
  ADSIM_CHECK_EQ(lo, std::size_t(1));
  ADSIM_CHECK_EQ(hi, std::size_t(2));

  // 落在首末样本之间即可构成区间
  ADSIM_CHECK(TimeAligner::bracket(stamps, 50, lo, hi));
  ADSIM_CHECK_EQ(lo, std::size_t(0));
  ADSIM_CHECK_EQ(hi, std::size_t(1));

  // 恰好落在首个样本上时其前方无样本，不构成区间
  ADSIM_CHECK(!TimeAligner::bracket(stamps, 0, lo, hi));
  // 超出末样本，同样不构成区间
  ADSIM_CHECK(!TimeAligner::bracket(stamps, 350, lo, hi));
  // 样本数不足 2 无法构成区间
  ADSIM_CHECK(!TimeAligner::bracket({1}, 1, lo, hi));
}

ADSIM_TEST(TimeAligner, 车辆状态线性插值) {
  std::vector<VehicleState> states(2);
  states[0].stamp = 0;
  states[0].x = 0.0;
  states[0].y = 0.0;
  states[0].speed = 10.0;
  states[1].stamp = 100;
  states[1].x = 100.0;
  states[1].y = 50.0;
  states[1].speed = 20.0;

  VehicleState out;
  ADSIM_CHECK(TimeAligner::interpolate(states, 25, out));
  ADSIM_CHECK_NEAR(out.x, 25.0, 1e-9);
  ADSIM_CHECK_NEAR(out.y, 12.5, 1e-9);
  ADSIM_CHECK_NEAR(out.speed, 12.5, 1e-9);
  ADSIM_CHECK_EQ(out.stamp, Timestamp(25));
}

ADSIM_TEST(TimeAligner, 航向插值走最短路径) {
  std::vector<VehicleState> states(2);
  states[0].stamp = 0;
  states[0].theta = deg2rad(170.0);
  states[1].stamp = 100;
  states[1].theta = deg2rad(-170.0);

  VehicleState out;
  ADSIM_CHECK(TimeAligner::interpolate(states, 50, out));

  // 跨越 ±180° 时必须走 20° 的短弧，而不是 340° 的长弧。
  // 中点应为 ±180°，其归一化结果绝对值接近 π。
  ADSIM_CHECK_NEAR(std::abs(out.theta), kPi, deg2rad(1.0));
}

ADSIM_TEST(TimeAligner, 序列外退化为最近邻) {
  std::vector<VehicleState> states(2);
  states[0].stamp = 100;
  states[0].x = 5.0;
  states[1].stamp = 200;
  states[1].x = 9.0;

  VehicleState out;
  ADSIM_CHECK(TimeAligner::interpolate(states, 0, out));   // 早于首帧
  ADSIM_CHECK_NEAR(out.x, 5.0, 1e-9);

  ADSIM_CHECK(TimeAligner::interpolate(states, 500, out));  // 晚于末帧
  ADSIM_CHECK_NEAR(out.x, 9.0, 1e-9);

  ADSIM_CHECK(!TimeAligner::interpolate({}, 0, out));       // 空输入
}

ADSIM_TEST(TimeAligner, 批量对齐以点云为节拍) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);

  constexpr int kLidarFrames = 10;
  constexpr double kLidarHz = 1.0;  // 点云覆盖 0 ~ 9 s
  const double duration = kLidarFrames / kLidarHz;

  // GPS 与车辆状态必须覆盖与点云相同的时间跨度，否则末端帧无从配对
  const std::vector<GpsFrame> gps =
      makeGpsTrack(projector, static_cast<int>(duration * 10.0), 10.0, 0, 0.0, 1u);
  const std::vector<VehicleState> odom =
      makeOdomTrack(static_cast<int>(duration * 50.0), 50.0, 0);

  std::vector<LidarFrame> lidar(kLidarFrames);
  for (int i = 0; i < kLidarFrames; ++i) {
    lidar[i].stamp = fromSeconds(i / kLidarHz);
    lidar[i].frame_id = "lidar";
    lidar[i].points.push_back({1.0f, 2.0f, 0.0f, 1.0f});
  }

  TimeAligner::Config config;
  config.match_tolerance = 200 * 1000000LL;  // 200ms
  TimeAligner aligner(config);

  const std::vector<AlignedFrame> aligned = aligner.align(gps, odom, lidar);

  ADSIM_CHECK_EQ(aligned.size(), std::size_t(kLidarFrames));
  for (const AlignedFrame& frame : aligned) {
    ADSIM_CHECK(frame.has_lidar);
    ADSIM_CHECK(frame.has_odom);
    ADSIM_CHECK(frame.has_gps);
  }

  const TimeAligner::Report report = TimeAligner::analyze(aligned);
  ADSIM_CHECK_EQ(report.total_frames, std::size_t(kLidarFrames));
  ADSIM_CHECK_NEAR(report.gps_match_rate, 1.0, 1e-9);
  ADSIM_CHECK_NEAR(report.odom_match_rate, 1.0, 1e-9);
  ADSIM_CHECK(!report.toString().empty());
}

ADSIM_TEST(TimeAligner, 容差外不匹配) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);
  const std::vector<GpsFrame> gps = makeGpsTrack(projector, 5, 1.0, 0, 0.0, 1u);

  std::vector<LidarFrame> lidar(1);
  lidar[0].stamp = 100 * kSec;  // 远晚于所有 GPS 帧

  TimeAligner::Config config;
  config.match_tolerance = 10 * 1000000LL;
  config.interpolate = false;  // 禁用插值以检验纯最近邻路径
  TimeAligner aligner(config);

  const std::vector<AlignedFrame> aligned = aligner.align(gps, {}, lidar);
  ADSIM_CHECK_EQ(aligned.size(), std::size_t(1));
  ADSIM_CHECK(!aligned[0].has_gps);
}

ADSIM_TEST(StreamingTimeAligner, 流式推送与取用) {
  StreamingTimeAligner aligner(100 * 1000000LL);  // 容差 100ms

  constexpr Timestamp kOdomPeriod = 20 * 1000000LL;   // 50Hz
  constexpr Timestamp kLidarPeriod = 200 * 1000000LL;  // 5Hz
  constexpr int kLidarFrames = 5;

  // 必须严格按时间顺序推送：流式对齐器会淘汰超出容差窗口的旧样本，
  // 若先灌完整段低速数据再补点云，早期样本早已被淘汰
  int odom_pushed = 0;
  const Timestamp end = kLidarPeriod * (kLidarFrames - 1);

  for (Timestamp t = 0; t <= end; t += kOdomPeriod) {
    VehicleState s;
    s.stamp = t;
    s.x = static_cast<double>(t) * 1e-6;
    aligner.pushVehicleState(s);
    ++odom_pushed;

    GpsFrame g;
    g.stamp = t;
    g.latitude = 31.23 + static_cast<double>(t) * 1e-12;
    aligner.pushGps(g);

    if (t % kLidarPeriod == 0) {
      LidarFrame l;
      l.stamp = t;
      aligner.pushLidar(l);
    }
  }

  const std::vector<AlignedFrame> frames = aligner.drain();
  ADSIM_CHECK_EQ(frames.size(), std::size_t(kLidarFrames));

  for (const AlignedFrame& frame : frames) {
    ADSIM_CHECK(frame.has_lidar);
    ADSIM_CHECK(frame.has_odom);
    ADSIM_CHECK(frame.has_gps);
  }

  const StreamingTimeAligner::Stats stats = aligner.stats();
  ADSIM_CHECK_EQ(stats.lidar_received, std::size_t(kLidarFrames));
  ADSIM_CHECK_EQ(stats.odom_received, std::size_t(odom_pushed));
  ADSIM_CHECK_EQ(stats.frames_emitted, std::size_t(kLidarFrames));

  // 取用后缓冲应清空
  ADSIM_CHECK_EQ(aligner.drain().size(), std::size_t(0));
}

ADSIM_TEST(StreamingTimeAligner, 缓冲区有界不随时长增长) {
  StreamingTimeAligner aligner(50 * 1000000LL);

  for (int i = 0; i < 5000; ++i) {
    VehicleState s;
    s.stamp = i * 1000000LL;
    aligner.pushVehicleState(s);
  }

  // 容差为 50ms，缓冲水位应与窗口成正比而非与总时长成正比
  ADSIM_CHECK_LT(aligner.stats().buffer_high_water, std::size_t(1000));
}

// ===========================================================================
//  点云降噪
// ===========================================================================

namespace {

/// 构造一个规整的栅格点云，便于精确验证各处理环节。
/// origin 用于把栅格整体推离原点，从而绕开距离裁剪环节的干扰。
std::vector<LidarPoint> makeGridPoints(int nx, int ny, double spacing,
                                       double origin_x = 0.0, double origin_y = 0.0) {
  std::vector<LidarPoint> points;
  points.reserve(static_cast<std::size_t>(nx) * ny);
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      LidarPoint p;
      p.x = static_cast<float>(origin_x + i * spacing);
      p.y = static_cast<float>(origin_y + j * spacing);
      p.z = 0.0f;
      p.intensity = 100.0f;
      points.push_back(p);
    }
  }
  return points;
}

/// 判断点是否落在给定矩形内（闭区间，带微小容差）
bool insideBox(const LidarPoint& p, double x0, double x1, double y0, double y1) {
  constexpr float eps = 1e-4f;
  return p.x >= x0 - eps && p.x <= x1 + eps && p.y >= y0 - eps && p.y <= y1 + eps;
}

}  // namespace

ADSIM_TEST(PointCloudFilter, 距离与高度裁剪) {
  std::vector<LidarPoint> points = {
      {0.5f, 0.0f, 0.0f, 0.0f},    // 过近，应剔除
      {5.0f, 0.0f, 0.0f, 0.0f},    // 保留
      {50.0f, 0.0f, 0.0f, 0.0f},   // 保留
      {100.0f, 0.0f, 0.0f, 0.0f},  // 过远，应剔除
      {10.0f, 0.0f, 10.0f, 0.0f},  // 过高，应剔除
      {10.0f, 0.0f, -10.0f, 0.0f}, // 过低，应剔除
  };

  const auto out = PointCloudFilter::cropByRange(points, 1.5, 80.0, -3.0, 3.0);
  ADSIM_CHECK_EQ(out.size(), std::size_t(2));
  ADSIM_CHECK_NEAR(out[0].x, 5.0, 1e-6);
  ADSIM_CHECK_NEAR(out[1].x, 50.0, 1e-6);
}

ADSIM_TEST(PointCloudFilter, 自车车体剔除) {
  std::vector<LidarPoint> points = {
      {0.0f, 0.0f, 0.0f, 0.0f},   // 车体中心，应剔除
      {2.0f, 0.5f, 0.0f, 0.0f},   // 车体内部，应剔除
      {10.0f, 0.0f, 0.0f, 0.0f},  // 远处，保留
      {0.0f, 5.0f, 0.0f, 0.0f},   // 侧向，保留
  };

  const auto out = PointCloudFilter::removeEgoBox(points, 5.0, 2.6);
  ADSIM_CHECK_EQ(out.size(), std::size_t(2));
}

ADSIM_TEST(PointCloudFilter, 体素降采样压缩点数并保持位置) {
  // 10×10 栅格，间距 0.1m，体素取 0.5m 应合并为 2×2 个体素
  const std::vector<LidarPoint> points = makeGridPoints(10, 10, 0.1);

  std::size_t cells = 0;
  const auto out = PointCloudFilter::voxelDownsample(points, 0.5, &cells);

  ADSIM_CHECK_EQ(out.size(), std::size_t(4));
  ADSIM_CHECK_EQ(cells, std::size_t(4));
  // 90×90 个点分入 4 个体素，每个约 25 个点
  for (const LidarPoint& p : out) {
    ADSIM_CHECK(p.x >= 0.0f && p.x <= 0.95f);
    ADSIM_CHECK(p.y >= 0.0f && p.y <= 0.95f);
  }

  // 非法体素尺寸应原样返回
  ADSIM_CHECK_EQ(PointCloudFilter::voxelDownsample(points, 0.0, nullptr).size(),
                 points.size());
}

ADSIM_TEST(PointCloudFilter, 统计离群点去除剔除注入的噪声点) {
  // 主体：密集栅格
  std::vector<LidarPoint> points = makeGridPoints(40, 40, 0.1);  // 1600 个点
  const std::size_t clean_count = points.size();

  // 注入 20 个远离主体的孤立点
  std::mt19937 rng(7u);
  std::uniform_real_distribution<double> far(50.0, 80.0);
  for (int i = 0; i < 20; ++i) {
    LidarPoint p;
    p.x = static_cast<float>(far(rng));
    p.y = static_cast<float>(far(rng));
    p.z = 0.0f;
    p.intensity = 1.0f;
    points.push_back(p);
  }

  const auto out = PointCloudFilter::statisticalOutlierRemoval(points, 10, 1.0);

  // 孤立点必须被全部剔除
  ADSIM_CHECK_LT(out.size(), points.size());
  ADSIM_CHECK_GT(out.size(), clean_count * 90 / 100);  // 主体应基本保留

  std::size_t far_points = 0;
  for (const LidarPoint& p : out) {
    if (p.x > 40.0f || p.y > 40.0f) ++far_points;
  }
  ADSIM_CHECK_EQ(far_points, std::size_t(0));
}

ADSIM_TEST(PointCloudFilter, 统计离群点去除保留内部点) {
  // 在均匀栅格上，边界点的平均近邻距离天然大于内部点，被 SOR 剔除属于
  // 该算法的固有特性而非缺陷。这里检验真正该保证的语义：内部点不得被误伤。
  constexpr double kSpacing = 0.2;
  constexpr int kGrid = 30;

  const std::vector<LidarPoint> points = makeGridPoints(kGrid, kGrid, kSpacing);
  const auto out = PointCloudFilter::statisticalOutlierRemoval(points, 8, 1.5);

  ADSIM_CHECK_GT(out.size(), std::size_t(0));

  // 距边界至少 5 格子的区域应被完整保留
  const double margin = 5 * kSpacing;
  const double hi = (kGrid - 1) * kSpacing - margin;

  std::size_t interior_in = 0;
  std::size_t interior_out = 0;
  for (const LidarPoint& p : points) {
    if (insideBox(p, margin, hi, margin, hi)) ++interior_in;
  }
  for (const LidarPoint& p : out) {
    if (insideBox(p, margin, hi, margin, hi)) ++interior_out;
  }

  ADSIM_CHECK_GT(interior_in, std::size_t(0));
  ADSIM_CHECK_EQ(interior_out, interior_in);
}

ADSIM_TEST(PointCloudFilter, 点数过少时安全返回) {
  const std::vector<LidarPoint> tiny = {{0.0f, 0.0f, 0.0f, 0.0f},
                                        {1.0f, 0.0f, 0.0f, 0.0f}};
  ADSIM_CHECK_EQ(PointCloudFilter::statisticalOutlierRemoval(tiny, 10, 1.0).size(),
                 tiny.size());
  ADSIM_CHECK_EQ(PointCloudFilter::statisticalOutlierRemoval({}, 10, 1.0).size(),
                 std::size_t(0));
}

ADSIM_TEST(PointCloudFilter, RANSAC地面分离) {
  std::vector<LidarPoint> points;

  // 地面：z ≈ 0 的平面
  for (int i = 0; i < 300; ++i) {
    LidarPoint p;
    p.x = static_cast<float>(i % 20) * 0.5f;
    p.y = static_cast<float>(i / 20) * 0.5f;
    p.z = 0.0f;
    p.intensity = 50.0f;
    points.push_back(p);
  }

  // 障碍物：z = 1.5m 处的点
  for (int i = 0; i < 50; ++i) {
    LidarPoint p;
    p.x = static_cast<float>(i) * 0.3f;
    p.y = 5.0f;
    p.z = 1.5f;
    p.intensity = 200.0f;
    points.push_back(p);
  }

  std::vector<LidarPoint> ground;
  std::vector<LidarPoint> non_ground;
  PointCloudFilter::separateGround(points, 200, 0.1, 0.5, ground, non_ground);

  ADSIM_CHECK_GT(ground.size(), std::size_t(250));
  ADSIM_CHECK_LT(ground.size(), points.size());
  ADSIM_CHECK_GT(non_ground.size(), std::size_t(30));

  // 分离出的地面点高度应接近 0
  for (const LidarPoint& p : ground) {
    ADSIM_CHECK_LT(std::abs(p.z), 0.3f);
  }
}

ADSIM_TEST(PointCloudFilter, 完整处理链统计正确) {
  // 栅格整体偏移到 x∈[5, 9.9]、y∈[-2.5, 2.4]：全部点都落在距离裁剪区间内、
  // 也不与自车轮廓相交，从而能单独检验体素降采样与离群点去除两个环节
  std::vector<LidarPoint> points = makeGridPoints(50, 50, 0.1, 5.0, -2.5);

  // 加入少量位于量程内、但远离主体的孤立点
  for (int i = 0; i < 10; ++i) {
    points.push_back({30.0f, 30.0f, 0.0f, 1.0f});
  }

  LidarFrame frame;
  frame.stamp = 12345;
  frame.frame_id = "lidar";
  frame.points = points;

  PointCloudFilter::Config config;
  config.enable_ground_removal = false;
  PointCloudFilter filter(config);

  PointCloudFilter::Statistics stats;
  const LidarFrame out = filter.filter(frame, &stats);

  ADSIM_CHECK_EQ(stats.input_points, points.size());
  ADSIM_CHECK_EQ(stats.after_range_crop, points.size());      // 无点被距离裁剪
  ADSIM_CHECK_EQ(stats.after_ego_removal, points.size());     // 无点落在自车轮廓内
  ADSIM_CHECK_LT(stats.after_voxel, stats.after_ego_removal); // 体素降采样生效
  ADSIM_CHECK_LT(stats.after_outlier, stats.after_voxel);     // 孤立点被剔除
  ADSIM_CHECK_EQ(stats.output_points, out.points.size());
  ADSIM_CHECK_GT(stats.reductionRatio(), 0.5);
  ADSIM_CHECK(!stats.toString().empty());

  // 帧元信息必须被保留
  ADSIM_CHECK_EQ(out.stamp, Timestamp(12345));
  ADSIM_CHECK_EQ(out.frame_id, frame.frame_id);
}

// ===========================================================================
//  GPS 滤波
// ===========================================================================

ADSIM_TEST(GpsFilter, 正常轨迹被接受) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);
  const std::vector<GpsFrame> track = makeGpsTrack(projector, 100, 10.0, 0, 0.5, 42u);

  GpsFilter filter;
  const std::vector<FilteredGpsSample> result = filter.filterAll(track);

  ADSIM_CHECK_EQ(result.size(), std::size_t(100));

  // 门控阈值取 99% 分位数，意味着约 1% 的正常观测会被固有地判为异常，
  // 叠加收敛暂态期更大的新息，这里要求误拒率控制在 5% 以内即可。
  ADSIM_CHECK_LT(filter.statistics().rejected, std::size_t(5));
  ADSIM_CHECK_GT(filter.statistics().acceptanceRate(), 0.95);

  // 首帧位于投影原点
  ADSIM_CHECK_NEAR(result[0].position.norm(), 0.0, 1e-6);
}

ADSIM_TEST(GpsFilter, 离群跳点被剔除) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);
  std::vector<GpsFrame> track = makeGpsTrack(projector, 200, 10.0, 0, 0.3, 7u);

  // 在若干位置注入数十米级的跳点
  const std::vector<int> outlier_indices = {30, 80, 81, 150};
  for (int index : outlier_indices) {
    track[index].latitude += 0.001;   // ≈ 111 m
    track[index].longitude -= 0.001;
  }

  GpsFilter filter;
  filter.filterAll(track);

  ADSIM_CHECK_EQ(filter.statistics().total, std::size_t(200));
  ADSIM_CHECK_GT(filter.statistics().rejected, std::size_t(0));
  ADSIM_CHECK_LT(filter.statistics().rejected, std::size_t(15));  // 不应误杀正常帧
  ADSIM_CHECK_GT(filter.statistics().max_rejected_jump, 50.0);
}

ADSIM_TEST(GpsFilter, 滤波后位置误差小于原始观测) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);

  // 真值轨迹
  std::vector<Vec2> truth;
  for (int i = 0; i < 300; ++i) {
    const double t = i * 0.1;
    truth.push_back({10.0 * t, 2.0 * std::sin(t)});
  }

  std::vector<GpsFrame> noisy;
  std::mt19937 rng(99u);
  std::normal_distribution<double> noise(0.0, 1.5 * 1e-5);  // ≈ 1.7m

  for (std::size_t i = 0; i < truth.size(); ++i) {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    projector.toGeodetic({truth[i].x, truth[i].y, 0.0}, lat, lon, alt);

    GpsFrame g;
    g.stamp = static_cast<Timestamp>(i) * 100000000LL;
    g.latitude = lat + noise(rng);
    g.longitude = lon + noise(rng);
    g.altitude = 10.0;
    g.hdop = 0.8;
    g.fix_type = 3;
    noisy.push_back(g);
  }

  GpsFilter filter;
  filter.reset();
  const std::vector<FilteredGpsSample> filtered = filter.filterAll(noisy);

  std::vector<Vec2> raw_positions;
  std::vector<Vec2> filtered_positions;
  raw_positions.reserve(filtered.size());
  filtered_positions.reserve(filtered.size());
  for (const FilteredGpsSample& s : filtered) {
    raw_positions.push_back(s.raw_position);
    filtered_positions.push_back(s.position);
  }

  const double raw_rmse = GpsFilter::rootMeanSquareError(raw_positions, truth);
  const double filtered_rmse = GpsFilter::rootMeanSquareError(filtered_positions, truth);

  ADSIM_CHECK_GT(raw_rmse, 0.5);          // 原始观测确实含明显噪声
  ADSIM_CHECK_LT(filtered_rmse, raw_rmse);  // 滤波必须带来改善
}

ADSIM_TEST(GpsFilter, 滑动平均对照组同样可用) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);
  const std::vector<GpsFrame> track = makeGpsTrack(projector, 50, 10.0, 0, 2.0, 3u);

  const std::vector<GpsFrame> smoothed = GpsFilter::movingAverage(track, 5);
  ADSIM_CHECK_EQ(smoothed.size(), track.size());

  // 时间戳保持不变
  for (std::size_t i = 0; i < track.size(); ++i) {
    ADSIM_CHECK_EQ(smoothed[i].stamp, track[i].stamp);
  }

  // 窗口为 1 时应原样返回
  const std::vector<GpsFrame> identity = GpsFilter::movingAverage(track, 1);
  for (std::size_t i = 0; i < track.size(); ++i) {
    ADSIM_CHECK_NEAR(identity[i].latitude, track[i].latitude, 1e-12);
  }
}

ADSIM_TEST(GpsFilter, 重置后状态清空) {
  const GeoProjector projector(31.2304, 121.4737, 10.0);
  const std::vector<GpsFrame> track = makeGpsTrack(projector, 20, 10.0, 0, 0.3, 5u);

  GpsFilter filter;
  filter.filterAll(track);
  ADSIM_CHECK(filter.initialized());

  filter.reset();
  ADSIM_CHECK(!filter.initialized());
  ADSIM_CHECK_EQ(filter.statistics().total, std::size_t(0));
}

ADSIM_TEST(HeadingFilter, 跨越正负180度不跳变) {
  HeadingFilter filter(0.5);

  // 航向从 +179° 连续转到 -179°，实际只转动了 2°
  const double first = filter.update(deg2rad(179.0));
  const double second = filter.update(deg2rad(-179.0));

  ADSIM_CHECK_NEAR(first, deg2rad(179.0), 1e-9);
  // 平滑结果必须落在 179° 与 -179° 之间的短弧上（即接近 ±180°），
  // 而不是被拉到 0° 附近
  ADSIM_CHECK_GT(std::abs(second), deg2rad(170.0));

  filter.reset();
  ADSIM_CHECK_EQ(filter.sampleCount(), std::size_t(0));
}

// ===========================================================================
//  端到端管道
// ===========================================================================

namespace {

/// 生成一个小规模测试 bag
void writeTestBag(const std::string& path, int lidar_frames, int points_per_frame) {
  RosBagWriter writer(path);

  const ros::MessageTypeInfo lidar_type = ros::lookupMessageType("sensor_msgs/PointCloud2");
  const ros::MessageTypeInfo gps_type = ros::lookupMessageType("sensor_msgs/NavSatFix");
  const ros::MessageTypeInfo odom_type = ros::lookupMessageType("nav_msgs/Odometry");

  writer.addConnection("/lidar", lidar_type.type, lidar_type.md5sum,
                       lidar_type.message_definition);
  writer.addConnection("/gps", gps_type.type, gps_type.md5sum, gps_type.message_definition);
  writer.addConnection("/odom", odom_type.type, odom_type.md5sum,
                       odom_type.message_definition);

  const GeoProjector projector(31.2304, 121.4737, 10.0);
  constexpr Timestamp start = 1700000000000000000LL;

  for (int f = 0; f < lidar_frames; ++f) {
    const Timestamp stamp = start + f * 100000000LL;  // 10Hz
    const double t = f * 0.1;

    // 点云：一片栅格 + 若干离群点
    LidarFrame lidar;
    lidar.stamp = stamp;
    lidar.frame_id = "velodyne";
    for (int i = 0; i < points_per_frame; ++i) {
      LidarPoint p;
      p.x = static_cast<float>(t * 10.0 + (i % 20) * 0.1);
      p.y = static_cast<float>((i / 20) * 0.1);
      p.z = 0.0f;
      p.intensity = 100.0f;
      lidar.points.push_back(p);
    }
    lidar.points.push_back({200.0f, 200.0f, 0.0f, 1.0f});  // 孤立点
    const std::vector<std::uint8_t> lidar_payload = ros::encodePointCloud2(lidar);
    writer.writeMessage("/lidar", stamp, lidar_payload.data(), lidar_payload.size());

    // GPS
    double lat = 0.0, lon = 0.0, alt = 0.0;
    projector.toGeodetic({t * 10.0, 0.0, 0.0}, lat, lon, alt);
    GpsFrame gps;
    gps.stamp = stamp;
    gps.latitude = lat;
    gps.longitude = lon;
    gps.altitude = 10.0;
    gps.hdop = 0.8;
    gps.fix_type = 3;
    const std::vector<std::uint8_t> gps_payload = ros::encodeNavSatFix(gps);
    writer.writeMessage("/gps", stamp, gps_payload.data(), gps_payload.size());

    // 车辆状态
    VehicleState odom;
    odom.stamp = stamp;
    odom.x = t * 10.0;
    odom.y = 0.0;
    odom.theta = 0.0;
    odom.speed = 10.0;
    const std::vector<std::uint8_t> odom_payload = ros::encodeOdometry(odom);
    writer.writeMessage("/odom", stamp, odom_payload.data(), odom_payload.size());
  }

  writer.close();
}

}  // namespace

ADSIM_TEST(DataPipeline, 端到端处理并输出bag) {
  TempFile input("e2e_in.bag");
  TempFile output("e2e_out.bag");
  writeTestBag(input.path(), 30, 400);

  // 先确认输入 bag 自身可读
  const BagInfo probe = DataPipeline::probe(input.path());
  ADSIM_CHECK_EQ(probe.message_count, std::size_t(90));  // 30 帧 × 3 话题

  DataPipeline::Config config;
  config.thread_count = 4;
  config.enable_lidar_filter = true;
  config.enable_gps_filter = true;
  config.keep_lidar = true;

  DataPipeline pipeline(config);
  const DataPipeline::Report report = pipeline.run(input.path(), output.path());

  ADSIM_CHECK_EQ(report.messages_read, std::size_t(90));
  ADSIM_CHECK_EQ(report.lidar_frames, std::size_t(30));
  ADSIM_CHECK_EQ(report.gps_frames, std::size_t(30));
  ADSIM_CHECK_EQ(report.odometry_frames, std::size_t(30));
  ADSIM_CHECK_EQ(report.decode_failures, std::size_t(0));
  ADSIM_CHECK_EQ(report.aligned_frames, std::size_t(30));
  ADSIM_CHECK_GT(report.lidar_points_in, std::size_t(0));
  ADSIM_CHECK_GT(report.elapsed_seconds, 0.0);
  ADSIM_CHECK_GT(report.bytes_processed, 0.0);
  ADSIM_CHECK_GT(report.messages_per_second, 0.0);
  ADSIM_CHECK(!report.toString().empty());

  // 降噪后点数应显著减少（体素降采样生效）
  ADSIM_CHECK_LT(report.lidar_points_out, report.lidar_points_in);

  // 输出 bag 必须可被重新读取
  RosBagReader reader(output.path());
  reader.open();
  ADSIM_CHECK_GT(reader.info().message_count, std::size_t(0));
  ADSIM_CHECK(reader.hasTopic("/lidar/clean"));
  ADSIM_CHECK(reader.hasTopic("/gps/clean"));
  ADSIM_CHECK(reader.hasTopic("/odom/clean"));
}

ADSIM_TEST(DataPipeline, 不落盘模式仅做统计) {
  TempFile input("nostore_in.bag");
  writeTestBag(input.path(), 10, 200);

  DataPipeline::Config config;
  config.thread_count = 2;
  config.keep_lidar = false;

  DataPipeline pipeline(config);
  const DataPipeline::Report report = pipeline.run(input.path());

  ADSIM_CHECK_EQ(report.lidar_frames, std::size_t(10));
  ADSIM_CHECK_EQ(report.output_messages, std::size_t(0));
  ADSIM_CHECK(report.output_path.empty());
}

ADSIM_TEST(DataPipeline, 单线程与多线程结果一致) {
  TempFile input("consistency.bag");
  writeTestBag(input.path(), 40, 300);

  DataPipeline::Config single;
  single.thread_count = 1;
  DataPipeline pipeline_single(single);
  const DataPipeline::Report report_single = pipeline_single.run(input.path());

  DataPipeline::Config multi;
  multi.thread_count = 8;
  DataPipeline pipeline_multi(multi);
  const DataPipeline::Report report_multi = pipeline_multi.run(input.path());

  // 并行化不得改变处理结果
  ADSIM_CHECK_EQ(report_single.lidar_frames, report_multi.lidar_frames);
  ADSIM_CHECK_EQ(report_single.aligned_frames, report_multi.aligned_frames);
  ADSIM_CHECK_EQ(report_single.lidar_points_in, report_multi.lidar_points_in);
  ADSIM_CHECK_EQ(report_single.lidar_points_out, report_multi.lidar_points_out);
  ADSIM_CHECK_EQ(report_single.gps_rejected, report_multi.gps_rejected);
}

ADSIM_TEST(DataPipeline, 小容量缓冲仍能正确处理背压) {
  TempFile input("backpressure.bag");
  writeTestBag(input.path(), 60, 300);

  DataPipeline::Config config;
  config.thread_count = 2;
  config.queue_capacity = 4;  // 故意设置极小，强制触发背压等待

  DataPipeline pipeline(config);
  const DataPipeline::Report report = pipeline.run(input.path());

  // 背压只会拖慢速度，绝不能丢数据
  ADSIM_CHECK_EQ(report.lidar_frames, std::size_t(60));
  ADSIM_CHECK_EQ(report.aligned_frames, std::size_t(60));
}

ADSIM_TEST(DataPipeline, 不存在的文件应报错) {
  DataPipeline pipeline;
  ADSIM_CHECK_THROWS(pipeline.run("/tmp/adsim_definitely_missing_file.bag"),
                     std::runtime_error);
}
