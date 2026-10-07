// =============================================================================
//  test_compression.cpp — Bag Chunk 压缩与解压
//
//  实车 bag 经常是压缩存储的，不支持解压的解析器可用性有限。
//
//  格式正确性已用**独立实现交叉验证**过（见 docs/algorithms.md）：
//    * LZ4 —— 与 ROS 官方 roslz4（ros_comm/utilities/roslz4）双向对撞，
//             压缩产物逐字节一致
//    * bz2 —— 与 Python 标准库 bz2 模块双向对撞
//  本文件把这些行为固化下来，防止后续改动破坏兼容性。
// =============================================================================
#include "TestFramework.h"

#include "adsim/datapipeline/BagCompression.h"
#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/RosBagReader.h"
#include "adsim/datapipeline/DataPipeline.h"
#include "adsim/datapipeline/RosBagWriter.h"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace adsim;

namespace {

class TempFile {
 public:
  explicit TempFile(const std::string& name) : path_("/tmp/adsim_comp_" + name) {}
  ~TempFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

/// 构造一段有结构的测试数据（模拟 chunk 内 Records 的布局）
std::vector<std::uint8_t> makePayload(std::size_t size, unsigned seed) {
  std::mt19937 rng(seed);
  std::vector<std::uint8_t> data;
  data.reserve(size);

  while (data.size() < size) {
    // 模拟记录头：长度字段 + 字段名
    const std::uint32_t header_len = 8;
    for (int i = 0; i < 4; ++i) data.push_back(static_cast<std::uint8_t>(header_len >> (8 * i)));
    for (char c : std::string("op\x02" "conn")) data.push_back(static_cast<std::uint8_t>(c));
    // 模拟载荷
    for (int i = 0; i < 16; ++i) {
      data.push_back(static_cast<std::uint8_t>(rng() & 0xFF));
    }
  }
  data.resize(size);
  return data;
}

/// 本构建支持的压缩方式列表
std::vector<bag::Compression> testableCompressions() {
  std::vector<bag::Compression> result{bag::Compression::kNone};
  if (bag::isCompressionSupported(bag::Compression::kBz2)) {
    result.push_back(bag::Compression::kBz2);
  }
  if (bag::isCompressionSupported(bag::Compression::kLz4)) {
    result.push_back(bag::Compression::kLz4);
  }
  return result;
}

/// 写一个指定压缩方式的 bag
void writeTestBag(const std::string& path, bag::Compression compression,
                  int frames, int points_per_frame) {
  RosBagWriter::Config config;
  config.compression = compression;

  RosBagWriter writer(path, config);

  const ros::MessageTypeInfo lidar_type = ros::lookupMessageType("sensor_msgs/PointCloud2");
  const ros::MessageTypeInfo gps_type = ros::lookupMessageType("sensor_msgs/NavSatFix");
  writer.addConnection("/lidar", lidar_type.type, lidar_type.md5sum,
                       lidar_type.message_definition);
  writer.addConnection("/gps", gps_type.type, gps_type.md5sum,
                       gps_type.message_definition);

  constexpr Timestamp start = 1700000000000000000LL;

  for (int f = 0; f < frames; ++f) {
    const Timestamp stamp = start + f * 100000000LL;

    LidarFrame lidar;
    lidar.stamp = stamp;
    lidar.frame_id = "velodyne";
    for (int i = 0; i < points_per_frame; ++i) {
      LidarPoint p;
      p.x = static_cast<float>(f * 0.5 + i * 0.1);
      p.y = static_cast<float>(i * 0.05);
      p.z = 0.0f;
      p.intensity = static_cast<float>(i % 200);
      lidar.points.push_back(p);
    }
    const std::vector<std::uint8_t> payload = ros::encodePointCloud2(lidar);
    writer.writeMessage("/lidar", stamp, payload.data(), payload.size());

    GpsFrame gps;
    gps.stamp = stamp;
    gps.latitude = 31.23 + f * 1e-5;
    gps.longitude = 121.47 + f * 1e-5;
    gps.altitude = 10.0;
    gps.hdop = 0.8;
    gps.fix_type = 3;
    const std::vector<std::uint8_t> gps_payload = ros::encodeNavSatFix(gps);
    writer.writeMessage("/gps", stamp, gps_payload.data(), gps_payload.size());
  }

  writer.close();
}

}  // namespace

// ===========================================================================
//  压缩/解压往返
// ===========================================================================

ADSIM_TEST(Compression, 各算法压缩解压往返一致) {
  const std::vector<std::uint8_t> original = makePayload(200000, 12345u);

  for (bag::Compression algo : testableCompressions()) {
    std::vector<std::uint8_t> compressed;
    std::string error;

    ADSIM_CHECK_MSG(bag::compressChunk(algo, original.data(), original.size(), compressed,
                                       error),
                    "压缩失败 (" << bag::toString(algo) << "): " << error);
    ADSIM_CHECK_MSG(!compressed.empty(), "压缩产物为空 " << bag::toString(algo));

    std::vector<std::uint8_t> restored;
    ADSIM_CHECK_MSG(bag::decompressChunk(algo, compressed.data(), compressed.size(),
                                         original.size(), restored, error),
                    "解压失败 (" << bag::toString(algo) << "): " << error);
    ADSIM_CHECK_EQ(restored.size(), original.size());
    ADSIM_CHECK_MSG(restored == original,
                    "往返后内容不一致 (" << bag::toString(algo) << ")");
  }
}

ADSIM_TEST(Compression, 压缩确实减小了体积) {
  // 有结构的重复数据应当被显著压缩
  const std::vector<std::uint8_t> original = makePayload(200000, 999u);

  for (bag::Compression algo : testableCompressions()) {
    if (algo == bag::Compression::kNone) continue;

    std::vector<std::uint8_t> compressed;
    std::string error;
    ADSIM_CHECK(bag::compressChunk(algo, original.data(), original.size(), compressed, error));

    ADSIM_CHECK_MSG(compressed.size() < original.size(),
                    bag::toString(algo) << " 未产生压缩效果: " << compressed.size() << " vs "
                                        << original.size());
  }
}

ADSIM_TEST(Compression, 空数据往返) {
  for (bag::Compression algo : testableCompressions()) {
    std::vector<std::uint8_t> compressed;
    std::string error;
    ADSIM_CHECK(bag::compressChunk(algo, nullptr, 0, compressed, error));

    std::vector<std::uint8_t> restored;
    ADSIM_CHECK(bag::decompressChunk(algo, compressed.empty() ? nullptr : compressed.data(),
                                     compressed.size(), 0, restored, error));
    ADSIM_CHECK_EQ(restored.size(), std::size_t(0));
  }
}

ADSIM_TEST(Compression, 随机数据不崩溃) {
  // 不可压缩的数据也应当正确处理（压缩后反而变大）
  std::mt19937 rng(7u);
  std::vector<std::uint8_t> random_data(50000);
  for (auto& byte : random_data) byte = static_cast<std::uint8_t>(rng() & 0xFF);

  for (bag::Compression algo : testableCompressions()) {
    std::vector<std::uint8_t> compressed;
    std::string error;
    ADSIM_CHECK(bag::compressChunk(algo, random_data.data(), random_data.size(), compressed,
                                   error));

    std::vector<std::uint8_t> restored;
    ADSIM_CHECK(bag::decompressChunk(algo, compressed.data(), compressed.size(),
                                     random_data.size(), restored, error));
    ADSIM_CHECK(restored == random_data);
  }
}

// ===========================================================================
//  损坏数据的处理
// ===========================================================================

ADSIM_TEST(Compression, 长度不符被检出) {
  const std::vector<std::uint8_t> original = makePayload(50000, 3u);

  for (bag::Compression algo : testableCompressions()) {
    if (algo == bag::Compression::kNone) continue;

    std::vector<std::uint8_t> compressed;
    std::string error;
    ADSIM_CHECK(bag::compressChunk(algo, original.data(), original.size(), compressed, error));

    // 声明一个错误的解压长度：必须被检出，而不是静默接受
    std::vector<std::uint8_t> restored;
    const bool ok = bag::decompressChunk(algo, compressed.data(), compressed.size(),
                                         original.size() + 100, restored, error);
    ADSIM_CHECK_MSG(!ok, bag::toString(algo) << " 未检出长度不符");
    ADSIM_CHECK(!error.empty());
  }
}

ADSIM_TEST(Compression, 截断数据被检出) {
  const std::vector<std::uint8_t> original = makePayload(50000, 5u);

  for (bag::Compression algo : testableCompressions()) {
    if (algo == bag::Compression::kNone) continue;

    std::vector<std::uint8_t> compressed;
    std::string error;
    ADSIM_CHECK(bag::compressChunk(algo, original.data(), original.size(), compressed, error));

    // 截掉后半段：应当报错而不是返回垃圾数据
    std::vector<std::uint8_t> restored;
    const bool ok = bag::decompressChunk(algo, compressed.data(), compressed.size() / 2,
                                         original.size(), restored, error);
    ADSIM_CHECK_MSG(!ok, bag::toString(algo) << " 未检出数据截断");
  }
}

ADSIM_TEST(Compression, 垃圾数据被检出且不崩溃) {
  std::vector<std::uint8_t> garbage(1024, 0xAB);

  for (bag::Compression algo : testableCompressions()) {
    if (algo == bag::Compression::kNone) continue;

    std::vector<std::uint8_t> restored;
    std::string error;
    const bool ok = bag::decompressChunk(algo, garbage.data(), garbage.size(), 4096,
                                         restored, error);
    ADSIM_CHECK_MSG(!ok, bag::toString(algo) << " 未检出垃圾数据");
    ADSIM_CHECK(!error.empty());
  }
}

ADSIM_TEST(Compression, 未压缩模式的长度校验) {
  const std::vector<std::uint8_t> data = makePayload(1000, 1u);
  std::vector<std::uint8_t> restored;
  std::string error;

  // 长度相符
  ADSIM_CHECK(bag::decompressChunk(bag::Compression::kNone, data.data(), data.size(),
                                   data.size(), restored, error));
  ADSIM_CHECK(restored == data);

  // 长度不符
  ADSIM_CHECK(!bag::decompressChunk(bag::Compression::kNone, data.data(), data.size(),
                                    data.size() + 1, restored, error));
  ADSIM_CHECK(!error.empty());
}

// ===========================================================================
//  压缩 bag 的完整读写
// ===========================================================================

ADSIM_TEST(Compression, 压缩bag可正确写入与读回) {
  for (bag::Compression algo : testableCompressions()) {
    const std::string tag = std::string("bag_") + bag::toString(algo);
    TempFile file(tag + ".bag");

    writeTestBag(file.path(), algo, 20, 300);

    RosBagReader reader(file.path());
    reader.open();

    ADSIM_CHECK_MSG(reader.info().message_count == 40,
                    bag::toString(algo) << " 消息数错误: " << reader.info().message_count);
    ADSIM_CHECK_MSG(reader.info().compression == algo,
                    bag::toString(algo) << " 压缩方式识别错误");
    ADSIM_CHECK_EQ(reader.info().connections.size(), std::size_t(2));

    // 连接信息必须能从压缩 Chunk 中解出——它存在 Chunk 内部，
    // 若解压环节漏掉了连接扫描，这里会是 0
    ADSIM_CHECK_MSG(reader.info().findConnection("/lidar") != nullptr,
                    bag::toString(algo) << " 未能从压缩 Chunk 中解析出连接信息");

    std::size_t lidar_count = 0;
    std::size_t points_total = 0;
    reader.forEachMessage([&](const bag::MessageView& view) {
      if (view.type() == "sensor_msgs/PointCloud2") {
        const LidarFrame frame = ros::decodePointCloud2(view.data, view.size);
        if (frame.stamp == kInvalidTimestamp) return false;
        ++lidar_count;
        points_total += frame.points.size();
      }
      return true;
    });

    ADSIM_CHECK_MSG(lidar_count == 20,
                    bag::toString(algo) << " 点云帧数错误: " << lidar_count);
    ADSIM_CHECK_MSG(points_total == 20 * 300,
                    bag::toString(algo) << " 点云总点数错误: " << points_total);
  }
}

ADSIM_TEST(Compression, 各压缩方式解出的内容一致) {
  // 同一批数据用不同压缩方式写入，解出来的内容必须逐字节一致
  std::vector<std::pair<bag::Compression, std::string>> files;

  for (bag::Compression algo : testableCompressions()) {
    const std::string tag = std::string("same_") + bag::toString(algo);
    files.emplace_back(algo, "/tmp/adsim_comp_" + tag + ".bag");
    writeTestBag(files.back().second, algo, 10, 200);
  }

  std::vector<std::vector<float>> signatures;
  for (const auto& entry : files) {
    RosBagReader reader(entry.second);
    reader.open();

    std::vector<float> signature;
    reader.forEachMessage([&](const bag::MessageView& view) {
      if (view.type() == "sensor_msgs/PointCloud2") {
        const LidarFrame frame = ros::decodePointCloud2(view.data, view.size);
        for (const LidarPoint& p : frame.points) {
          signature.push_back(p.x);
          signature.push_back(p.y);
        }
      }
      return true;
    });
    signatures.push_back(std::move(signature));
  }

  for (std::size_t i = 1; i < signatures.size(); ++i) {
    ADSIM_CHECK_MSG(signatures[i] == signatures[0],
                    "压缩方式 " << bag::toString(files[i].first) << " 与 "
                                << bag::toString(files[0].first) << " 解出的内容不同");
  }

  for (const auto& entry : files) {
    std::remove(entry.second.c_str());
  }
}

ADSIM_TEST(Compression, 数据管道可处理压缩bag) {
  // 遍历本构建实际支持的压缩方式——未启用压缩库时本用例自动跳过，
  // 而不是因为"造不出测试数据"而失败
  for (bag::Compression algo : testableCompressions()) {
    if (algo == bag::Compression::kNone) continue;

    const std::string tag = std::string("pipeline_") + bag::toString(algo);
    TempFile file(tag + ".bag");
    writeTestBag(file.path(), algo, 15, 250);

    DataPipeline::Config config;
    config.thread_count = 4;

    DataPipeline pipeline(config);
    const DataPipeline::Report report = pipeline.run(file.path());

    ADSIM_CHECK_MSG(report.lidar_frames == 15,
                    bag::toString(algo) << " 点云帧数错误: " << report.lidar_frames);
    ADSIM_CHECK_MSG(report.gps_frames == 15,
                    bag::toString(algo) << " GPS 帧数错误: " << report.gps_frames);
    ADSIM_CHECK_MSG(report.decode_failures == 0,
                    bag::toString(algo) << " 出现解码失败");
    ADSIM_CHECK_MSG(report.aligned_frames == 15,
                    bag::toString(algo) << " 对齐帧数错误: " << report.aligned_frames);
  }
}

// ===========================================================================
//  能力查询
// ===========================================================================

ADSIM_TEST(Compression, 能力查询与编译期支持一致) {
  // 未压缩永远支持
  ADSIM_CHECK(bag::isCompressionSupported(bag::Compression::kNone));

#if defined(ADSIM_HAS_BZ2)
  ADSIM_CHECK(bag::isCompressionSupported(bag::Compression::kBz2));
#else
  ADSIM_CHECK(!bag::isCompressionSupported(bag::Compression::kBz2));
#endif

#if defined(ADSIM_HAS_LZ4)
  ADSIM_CHECK(bag::isCompressionSupported(bag::Compression::kLz4));
#else
  ADSIM_CHECK(!bag::isCompressionSupported(bag::Compression::kLz4));
#endif

  ADSIM_CHECK(!bag::isCompressionSupported(bag::Compression::kUnknown));

  const std::string list = bag::supportedCompressions();
  ADSIM_CHECK(list.find("none") != std::string::npos);
  ADSIM_CHECK(!list.empty());
}
