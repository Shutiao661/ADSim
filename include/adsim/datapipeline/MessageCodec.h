// =============================================================================
//  MessageCodec.h — ROS 消息序列化 / 反序列化
//
//  按 ROS 标准序列化规则实现（小端、字符串为 u32 长度前缀、数组为 u32 计数前缀），
//  覆盖路测数据中最关键的三类消息：
//      sensor_msgs/NavSatFix     — GPS/RTK 定位
//      nav_msgs/Odometry         — 车辆位姿与速度
//      sensor_msgs/PointCloud2   — 激光雷达点云
//
//  不依赖 ROS 运行时，可独立解析实车录制的 bag。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace adsim {
namespace ros {

// ---------------------------------------------------------------------------
// 序列化写入器
// ---------------------------------------------------------------------------
class MessageWriter {
 public:
  void writeU8(std::uint8_t value);
  void writeI8(std::int8_t value);
  void writeU16(std::uint16_t value);
  void writeI16(std::int16_t value);
  void writeU32(std::uint32_t value);
  void writeI32(std::int32_t value);
  void writeU64(std::uint64_t value);
  void writeF32(float value);
  void writeF64(double value);

  /// ROS string：u32 长度 + 原始字节（无终止符）
  void writeString(const std::string& value);
  void writeBool(bool value) { writeU8(value ? 1 : 0); }

  /// ROS time：u32 秒 + u32 纳秒
  void writeTime(Timestamp stamp);
  void writeDuration(double seconds);

  void writeBytes(const void* data, std::size_t size);

  const std::vector<std::uint8_t>& data() const { return buffer_; }
  std::size_t size() const { return buffer_.size(); }
  void clear() { buffer_.clear(); }

  std::vector<std::uint8_t> take() { return std::move(buffer_); }

 private:
  std::vector<std::uint8_t> buffer_;
};

// ---------------------------------------------------------------------------
// 反序列化读取器
//
//  越界读取不抛异常，而是把内部状态置为错误并返回 0/空值，
//  便于对损坏数据做容错处理；调用方通过 ok() 判断是否可信。
// ---------------------------------------------------------------------------
class MessageReader {
 public:
  MessageReader(const std::uint8_t* data, std::size_t size)
      : data_(data), size_(size) {}

  std::uint8_t readU8();
  std::int8_t readI8();
  std::uint16_t readU16();
  std::int16_t readI16();
  std::uint32_t readU32();
  std::int32_t readI32();
  std::uint64_t readU64();
  float readF32();
  double readF64();
  std::string readString();
  bool readBool() { return readU8() != 0; }

  Timestamp readTime();
  double readDuration();

  /// 跳过 n 字节
  void skip(std::size_t n);

  bool ok() const { return ok_; }
  std::size_t remaining() const { return ok_ ? size_ - offset_ : 0; }
  std::size_t offset() const { return offset_; }

 private:
  bool require(std::size_t n);

  const std::uint8_t* data_{nullptr};
  std::size_t size_{0};
  std::size_t offset_{0};
  bool ok_{true};
};

// ---------------------------------------------------------------------------
// std_msgs/Header
// ---------------------------------------------------------------------------
struct StdHeader {
  std::uint32_t seq{0};
  Timestamp stamp{kInvalidTimestamp};
  std::string frame_id;
};

void writeStdHeader(MessageWriter& writer, const StdHeader& header);
StdHeader readStdHeader(MessageReader& reader);

// ---------------------------------------------------------------------------
// 具体消息类型编解码
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> encodeNavSatFix(const GpsFrame& frame);
GpsFrame decodeNavSatFix(const std::uint8_t* data, std::size_t size);

std::vector<std::uint8_t> encodeOdometry(const VehicleState& state);
VehicleState decodeOdometry(const std::uint8_t* data, std::size_t size);

std::vector<std::uint8_t> encodePointCloud2(const LidarFrame& frame);
LidarFrame decodePointCloud2(const std::uint8_t* data, std::size_t size);

// ---------------------------------------------------------------------------
// 消息类型元信息（用于填充 bag 的连接记录）
// ---------------------------------------------------------------------------

/// 已知消息类型的元信息
struct MessageTypeInfo {
  std::string type;                ///< 如 sensor_msgs/PointCloud2
  std::string md5sum;              ///< ROS 官方 MD5，用于接收端类型校验
  std::string message_definition;  ///< 完整 .msg 定义（含依赖类型的分隔段）
};

/// 查询已知类型的元信息；未知类型返回空 type 字段
MessageTypeInfo lookupMessageType(const std::string& type);

/// 判断某类型是否受支持
bool isSupportedType(const std::string& type);

}  // namespace ros
}  // namespace adsim
