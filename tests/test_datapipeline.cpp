// =============================================================================
//  test_datapipeline.cpp — 数据管道单元测试
//  覆盖：ROS 消息编解码 / bag 格式读写往返 / 元信息统计
// =============================================================================
#include "TestFramework.h"

#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/RosBagFormat.h"
#include "adsim/datapipeline/RosBagReader.h"
#include "adsim/datapipeline/RosBagWriter.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace adsim;

namespace {

/// 生成一段确定性的合成点云，便于逐点比对
LidarFrame makeLidarFrame(Timestamp stamp, int point_count, float seed) {
  LidarFrame frame;
  frame.stamp = stamp;
  frame.frame_id = "velodyne";
  frame.points.reserve(static_cast<std::size_t>(point_count));
  for (int i = 0; i < point_count; ++i) {
    LidarPoint p;
    p.x = seed + static_cast<float>(i) * 0.25f;
    p.y = -seed + static_cast<float>(i) * 0.5f;
    p.z = static_cast<float>(i % 7) * 0.1f;
    p.intensity = static_cast<float>(i % 255);
    frame.points.push_back(p);
  }
  return frame;
}

GpsFrame makeGpsFrame(Timestamp stamp, double lat, double lon) {
  GpsFrame frame;
  frame.stamp = stamp;
  frame.latitude = lat;
  frame.longitude = lon;
  frame.altitude = 12.5;
  frame.hdop = 0.8;
  frame.fix_type = 3;
  return frame;
}

VehicleState makeVehicleState(Timestamp stamp, double x, double y, double theta) {
  VehicleState state;
  state.stamp = stamp;
  state.x = x;
  state.y = y;
  state.theta = theta;
  state.speed = 8.5;
  state.yaw_rate = 0.03;
  return state;
}

/// 测试用临时文件路径
class TempFile {
 public:
  explicit TempFile(const std::string& name) : path_("/tmp/adsim_test_" + name) {}
  ~TempFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

}  // namespace

// ===========================================================================
//  ROS 消息编解码
// ===========================================================================

ADSIM_TEST(MessageCodec, 基础类型序列化规则) {
  ros::MessageWriter writer;
  writer.writeU8(0xAB);
  writer.writeU32(0xDEADBEEF);
  writer.writeF64(3.14159);
  writer.writeString("hello");

  const auto& data = writer.data();
  // u8(1) + u32(4) + f64(8) + [u32 长度前缀(4) + 5 字节]
  ADSIM_CHECK_EQ(data.size(), std::size_t(1 + 4 + 8 + 4 + 5));

  // 小端序校验：0xDEADBEEF 应存为 EF BE AD DE
  ADSIM_CHECK_EQ(data[1], std::uint8_t(0xEF));
  ADSIM_CHECK_EQ(data[2], std::uint8_t(0xBE));
  ADSIM_CHECK_EQ(data[3], std::uint8_t(0xAD));
  ADSIM_CHECK_EQ(data[4], std::uint8_t(0xDE));

  ros::MessageReader reader(data.data(), data.size());
  ADSIM_CHECK_EQ(reader.readU8(), std::uint8_t(0xAB));
  ADSIM_CHECK_EQ(reader.readU32(), std::uint32_t(0xDEADBEEF));
  ADSIM_CHECK_NEAR(reader.readF64(), 3.14159, 1e-12);
  ADSIM_CHECK_EQ(reader.readString(), std::string("hello"));
  ADSIM_CHECK(reader.ok());
  ADSIM_CHECK_EQ(reader.remaining(), std::size_t(0));
}

ADSIM_TEST(MessageCodec, 读取越界不崩溃且标记错误) {
  const std::uint8_t data[3] = {1, 2, 3};
  ros::MessageReader reader(data, sizeof(data));

  ADSIM_CHECK(reader.ok());
  reader.readU64();          // 越界读取
  ADSIM_CHECK(!reader.ok()); // 必须标记为错误
  ADSIM_CHECK_EQ(reader.readU32(), std::uint32_t(0));  // 后续读取安全返回默认值
}

ADSIM_TEST(MessageCodec, NavSatFix往返一致) {
  const GpsFrame original = makeGpsFrame(1700000000123456789LL, 31.2304, 121.4737);

  const std::vector<std::uint8_t> encoded = ros::encodeNavSatFix(original);
  const GpsFrame decoded = ros::decodeNavSatFix(encoded.data(), encoded.size());

  ADSIM_CHECK_EQ(decoded.stamp, original.stamp);
  ADSIM_CHECK_NEAR(decoded.latitude, original.latitude, 1e-12);
  ADSIM_CHECK_NEAR(decoded.longitude, original.longitude, 1e-12);
  ADSIM_CHECK_NEAR(decoded.altitude, original.altitude, 1e-12);
  ADSIM_CHECK_NEAR(decoded.hdop, original.hdop, 1e-9);
  ADSIM_CHECK_EQ(decoded.fix_type, 3);
}

ADSIM_TEST(MessageCodec, Odometry往返一致) {
  const VehicleState original = makeVehicleState(1700000000987654321LL, 123.75, -45.5, 0.7853);

  const std::vector<std::uint8_t> encoded = ros::encodeOdometry(original);
  const VehicleState decoded = ros::decodeOdometry(encoded.data(), encoded.size());

  ADSIM_CHECK_EQ(decoded.stamp, original.stamp);
  ADSIM_CHECK_NEAR(decoded.x, original.x, 1e-9);
  ADSIM_CHECK_NEAR(decoded.y, original.y, 1e-9);
  // 航向经四元数往返，应保持等价角度
  ADSIM_CHECK_NEAR(normalizeAngle(decoded.theta - original.theta), 0.0, 1e-9);
  ADSIM_CHECK_NEAR(decoded.speed, original.speed, 1e-9);
  ADSIM_CHECK_NEAR(decoded.yaw_rate, original.yaw_rate, 1e-9);
}

ADSIM_TEST(MessageCodec, PointCloud2往返逐点一致) {
  const LidarFrame original = makeLidarFrame(1700000000111111111LL, 500, 1.5f);

  const std::vector<std::uint8_t> encoded = ros::encodePointCloud2(original);
  const LidarFrame decoded = ros::decodePointCloud2(encoded.data(), encoded.size());

  ADSIM_CHECK_EQ(decoded.stamp, original.stamp);
  ADSIM_CHECK_EQ(decoded.frame_id, std::string("velodyne"));
  ADSIM_CHECK_EQ(decoded.points.size(), original.points.size());

  bool all_equal = true;
  for (std::size_t i = 0; i < original.points.size(); ++i) {
    const LidarPoint& a = original.points[i];
    const LidarPoint& b = decoded.points[i];
    if (a.x != b.x || a.y != b.y || a.z != b.z || a.intensity != b.intensity) {
      all_equal = false;
      break;
    }
  }
  ADSIM_CHECK(all_equal);
}

ADSIM_TEST(MessageCodec, 空点云不崩溃) {
  LidarFrame empty;
  empty.stamp = 1000;
  empty.frame_id = "lidar";

  const std::vector<std::uint8_t> encoded = ros::encodePointCloud2(empty);
  const LidarFrame decoded = ros::decodePointCloud2(encoded.data(), encoded.size());

  ADSIM_CHECK_EQ(decoded.points.size(), std::size_t(0));
  ADSIM_CHECK_EQ(decoded.stamp, Timestamp(1000));
}

ADSIM_TEST(MessageCodec, 损坏数据不崩溃) {
  const std::vector<std::uint8_t> garbage = {0x01, 0x02, 0x03};
  const LidarFrame frame = ros::decodePointCloud2(garbage.data(), garbage.size());
  ADSIM_CHECK_EQ(frame.stamp, kInvalidTimestamp);  // 应被标记为无效

  const GpsFrame gps = ros::decodeNavSatFix(garbage.data(), garbage.size());
  ADSIM_CHECK_EQ(gps.stamp, kInvalidTimestamp);
}

ADSIM_TEST(MessageCodec, 类型元信息查询) {
  const ros::MessageTypeInfo lidar = ros::lookupMessageType("sensor_msgs/PointCloud2");
  ADSIM_CHECK_EQ(lidar.type, std::string("sensor_msgs/PointCloud2"));
  ADSIM_CHECK_EQ(lidar.md5sum, std::string("1158d486dd51d683ce2f1be655c3c181"));
  ADSIM_CHECK(!lidar.message_definition.empty());

  ADSIM_CHECK(ros::isSupportedType("sensor_msgs/NavSatFix"));
  ADSIM_CHECK(ros::isSupportedType("nav_msgs/Odometry"));
  ADSIM_CHECK(!ros::isSupportedType("unknown/Type"));
}

// ===========================================================================
//  bag 头部字段
// ===========================================================================

ADSIM_TEST(RosBagFormat, 头部字段读写往返) {
  bag::HeaderWriter writer;
  writer.addU8("op", 0x03);
  writer.addU32("conn_count", 7);
  writer.addU64("index_pos", 1234567890123ull);
  writer.addString("topic", "/lidar/top");

  bag::Header header;
  header.parse(writer.bytes().data(), writer.bytes().size());

  ADSIM_CHECK_EQ(header.getU8("op"), std::uint8_t(0x03));
  ADSIM_CHECK_EQ(header.getU32("conn_count"), std::uint32_t(7));
  ADSIM_CHECK_EQ(header.getU64("index_pos"), std::uint64_t(1234567890123ull));
  ADSIM_CHECK_EQ(header.getString("topic"), std::string("/lidar/top"));

  // 缺失字段应返回默认值
  ADSIM_CHECK(!header.has("nonexistent"));
  ADSIM_CHECK_EQ(header.getU32("nonexistent", 42), std::uint32_t(42));
}

ADSIM_TEST(RosBagFormat, 时间编解码往返) {
  const Timestamp stamps[] = {
      0LL, 1LL, 999999999LL, 1000000000LL, 1700000000123456789LL};

  for (Timestamp stamp : stamps) {
    ADSIM_CHECK_EQ(bag::decodeTime(bag::encodeTime(stamp)), stamp);
  }
}

ADSIM_TEST(RosBagFormat, 合法头部不得误报损坏) {
  bag::HeaderWriter writer;
  writer.addString("compression", "none");
  writer.addU32("size", 4096);

  bag::Header header;
  header.parse(writer.bytes().data(), writer.bytes().size());
  ADSIM_CHECK_EQ(header.getString("compression"), std::string("none"));
  ADSIM_CHECK_EQ(header.getU32("size"), std::uint32_t(4096));
}

// ===========================================================================
//  bag 读写往返
// ===========================================================================

ADSIM_TEST(RosBag, 写入后读取消息数与话题正确) {
  TempFile file("roundtrip.bag");
  constexpr int kGpsCount = 50;
  constexpr int kOdomCount = 100;
  constexpr int kLidarCount = 20;

  {
    RosBagWriter writer(file.path());
    for (const char* type : {"sensor_msgs/NavSatFix", "nav_msgs/Odometry",
                             "sensor_msgs/PointCloud2"}) {
      const ros::MessageTypeInfo info = ros::lookupMessageType(type);
      const std::string topic = std::string("/") + type;
      writer.addConnection(topic, info.type, info.md5sum, info.message_definition);
    }

    for (int i = 0; i < kGpsCount; ++i) {
      const GpsFrame frame = makeGpsFrame(1000000000LL + i * 100000000LL, 31.23 + i * 1e-5,
                                          121.47 + i * 1e-5);
      const std::vector<std::uint8_t> payload = ros::encodeNavSatFix(frame);
      writer.writeMessage("/sensor_msgs/NavSatFix", frame.stamp, payload.data(), payload.size());
    }
    for (int i = 0; i < kOdomCount; ++i) {
      const VehicleState state = makeVehicleState(1000000000LL + i * 50000000LL, i * 0.5,
                                                  i * 0.1, 0.01 * i);
      const std::vector<std::uint8_t> payload = ros::encodeOdometry(state);
      writer.writeMessage("/nav_msgs/Odometry", state.stamp, payload.data(), payload.size());
    }
    for (int i = 0; i < kLidarCount; ++i) {
      const LidarFrame frame = makeLidarFrame(1000000000LL + i * 100000000LL, 100, 1.0f * i);
      const std::vector<std::uint8_t> payload = ros::encodePointCloud2(frame);
      writer.writeMessage("/sensor_msgs/PointCloud2", frame.stamp, payload.data(), payload.size());
    }
    writer.close();
  }

  RosBagReader reader(file.path());
  reader.open();

  const BagInfo& info = reader.info();
  ADSIM_CHECK_EQ(info.message_count, std::size_t(kGpsCount + kOdomCount + kLidarCount));
  ADSIM_CHECK_EQ(info.connection_count, std::size_t(3));
  ADSIM_CHECK_EQ(info.connections.size(), std::size_t(3));
  ADSIM_CHECK_EQ(info.messageCountFor("/sensor_msgs/NavSatFix"), std::size_t(kGpsCount));
  ADSIM_CHECK_EQ(info.messageCountFor("/nav_msgs/Odometry"), std::size_t(kOdomCount));
  ADSIM_CHECK_EQ(info.messageCountFor("/sensor_msgs/PointCloud2"), std::size_t(kLidarCount));

  ADSIM_CHECK(info.start_time != kInvalidTimestamp);
  ADSIM_CHECK(info.end_time > info.start_time);
  ADSIM_CHECK(info.file_size > 0);

  // 连接元信息必须被正确还原
  const bag::ConnectionInfo* lidar_conn = info.findConnection("/sensor_msgs/PointCloud2");
  ADSIM_CHECK(lidar_conn != nullptr);
  if (lidar_conn != nullptr) {
    ADSIM_CHECK_EQ(lidar_conn->type, std::string("sensor_msgs/PointCloud2"));
    ADSIM_CHECK_EQ(lidar_conn->md5sum, std::string("1158d486dd51d683ce2f1be655c3c181"));
  }
  ADSIM_CHECK(!info.summary().empty());
}

ADSIM_TEST(RosBag, 遍历消息内容与原值一致) {
  TempFile file("content.bag");
  constexpr int kCount = 30;

  std::vector<VehicleState> written;
  {
    RosBagWriter writer(file.path());
    const ros::MessageTypeInfo info = ros::lookupMessageType("nav_msgs/Odometry");
    writer.addConnection("/odom", info.type, info.md5sum, info.message_definition);

    for (int i = 0; i < kCount; ++i) {
      const VehicleState state =
          makeVehicleState(2000000000LL + i * 10000000LL, i * 1.25, -i * 0.75, 0.02 * i);
      written.push_back(state);
      const std::vector<std::uint8_t> payload = ros::encodeOdometry(state);
      writer.writeMessage("/odom", state.stamp, payload.data(), payload.size());
    }
    writer.close();
  }

  RosBagReader reader(file.path());
  reader.open();

  std::vector<VehicleState> read_back;
  reader.forEachMessage([&read_back](const bag::MessageView& view) {
    ADSIM_CHECK_EQ(view.topic(), std::string("/odom"));
    ADSIM_CHECK_EQ(view.type(), std::string("nav_msgs/Odometry"));
    read_back.push_back(ros::decodeOdometry(view.data, view.size));
    return true;
  });

  ADSIM_CHECK_EQ(read_back.size(), written.size());
  ADSIM_CHECK_EQ(read_back.size(), std::size_t(kCount));

  bool all_match = true;
  for (std::size_t i = 0; i < written.size(); ++i) {
    if (read_back[i].stamp != written[i].stamp) { all_match = false; break; }
    if (std::abs(read_back[i].x - written[i].x) > 1e-9) { all_match = false; break; }
    if (std::abs(read_back[i].y - written[i].y) > 1e-9) { all_match = false; break; }
  }
  ADSIM_CHECK(all_match);
}

ADSIM_TEST(RosBag, 按话题过滤遍历) {
  TempFile file("filter.bag");

  {
    RosBagWriter writer(file.path());
    for (const char* type : {"sensor_msgs/NavSatFix", "nav_msgs/Odometry"}) {
      const ros::MessageTypeInfo info = ros::lookupMessageType(type);
      writer.addConnection(std::string("/") + type, info.type, info.md5sum,
                           info.message_definition);
    }
    for (int i = 0; i < 10; ++i) {
      const GpsFrame gps = makeGpsFrame(1000 + i, 31.0, 121.0);
      const std::vector<std::uint8_t> gps_payload = ros::encodeNavSatFix(gps);
      writer.writeMessage("/sensor_msgs/NavSatFix", gps.stamp, gps_payload.data(),
                          gps_payload.size());

      const VehicleState odom = makeVehicleState(1000 + i, 0, 0, 0);
      const std::vector<std::uint8_t> odom_payload = ros::encodeOdometry(odom);
      writer.writeMessage("/nav_msgs/Odometry", odom.stamp, odom_payload.data(),
                          odom_payload.size());
    }
    writer.close();
  }

  RosBagReader reader(file.path());
  reader.open();

  std::size_t odom_only = 0;
  reader.forEachMessage({"/nav_msgs/Odometry"}, [&odom_only](const bag::MessageView& view) {
    ADSIM_CHECK_EQ(view.topic(), std::string("/nav_msgs/Odometry"));
    ++odom_only;
    return true;
  });
  ADSIM_CHECK_EQ(odom_only, std::size_t(10));

  ADSIM_CHECK(reader.hasTopic("/sensor_msgs/NavSatFix"));
  ADSIM_CHECK(!reader.hasTopic("/nonexistent"));
}

ADSIM_TEST(RosBag, 回调返回false可提前终止) {
  TempFile file("early_stop.bag");

  {
    RosBagWriter writer(file.path());
    const ros::MessageTypeInfo info = ros::lookupMessageType("nav_msgs/Odometry");
    writer.addConnection("/odom", info.type, info.md5sum, info.message_definition);
    for (int i = 0; i < 100; ++i) {
      const VehicleState state = makeVehicleState(1000 + i, i, 0, 0);
      const std::vector<std::uint8_t> payload = ros::encodeOdometry(state);
      writer.writeMessage("/odom", state.stamp, payload.data(), payload.size());
    }
    writer.close();
  }

  RosBagReader reader(file.path());
  reader.open();

  std::size_t visited = 0;
  reader.forEachMessage([&visited](const bag::MessageView&) {
    ++visited;
    return visited < 5;  // 第 5 条后终止
  });
  ADSIM_CHECK_EQ(visited, std::size_t(5));
}

ADSIM_TEST(RosBag, 多Chunk文件完整读取) {
  TempFile file("multichunk.bag");
  constexpr int kCount = 2000;

  {
    // 极小的 Chunk 阈值，强制产生大量 Chunk，验证跨 Chunk 的连接记录逻辑
    RosBagWriter::Config config;
    config.chunk_size_bytes = 4096;

    RosBagWriter writer(file.path(), config);
    const ros::MessageTypeInfo info = ros::lookupMessageType("sensor_msgs/PointCloud2");
    writer.addConnection("/lidar", info.type, info.md5sum, info.message_definition);

    for (int i = 0; i < kCount; ++i) {
      const LidarFrame frame = makeLidarFrame(1000 + i, 20, 0.5f * i);
      const std::vector<std::uint8_t> payload = ros::encodePointCloud2(frame);
      writer.writeMessage("/lidar", frame.stamp, payload.data(), payload.size());
    }
    writer.close();
    ADSIM_CHECK_GT(writer.chunkCount(), std::size_t(1));
  }

  RosBagReader reader(file.path());
  reader.open();

  std::size_t count = 0;
  bool content_ok = true;
  reader.forEachMessage([&count, &content_ok](const bag::MessageView& view) {
    const LidarFrame frame = ros::decodePointCloud2(view.data, view.size);
    if (frame.points.size() != 20) { content_ok = false; }
    if (frame.stamp != view.stamp) { content_ok = false; }
    ++count;
    return true;
  });

  ADSIM_CHECK_EQ(count, std::size_t(kCount));
  ADSIM_CHECK(content_ok);
  ADSIM_CHECK_GT(reader.info().chunk_count, std::size_t(1));
}

ADSIM_TEST(RosBag, 空bag可正常读写) {
  TempFile file("empty.bag");
  {
    RosBagWriter writer(file.path());
    writer.close();
  }

  RosBagReader reader(file.path());
  reader.open();

  ADSIM_CHECK_EQ(reader.info().message_count, std::size_t(0));

  std::size_t count = 0;
  reader.forEachMessage([&count](const bag::MessageView&) {
    ++count;
    return true;
  });
  ADSIM_CHECK_EQ(count, std::size_t(0));
}

ADSIM_TEST(RosBag, 未注册话题写入应报错) {
  TempFile file("unregistered.bag");
  RosBagWriter writer(file.path());

  const std::uint8_t payload[4] = {0, 0, 0, 0};
  ADSIM_CHECK_THROWS(writer.writeMessage("/unknown", 1000, payload, sizeof(payload)),
                     std::runtime_error);
}

ADSIM_TEST(RosBag, 非bag文件应被拒绝) {
  TempFile file("notabag.bag");
  {
    std::FILE* fp = std::fopen(file.path().c_str(), "wb");
    ADSIM_CHECK(fp != nullptr);
    if (fp != nullptr) {
      const char* junk = "this is definitely not a rosbag file";
      std::fwrite(junk, 1, 36, fp);
      std::fclose(fp);
    }
  }

  RosBagReader reader(file.path());
  ADSIM_CHECK_THROWS(reader.open(), std::runtime_error);
}
