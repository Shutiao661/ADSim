// =============================================================================
//  RosBagFormat.h — ROS Bag v2.0 文件格式原语
//
//  rosbag 是实车路测数据的标准载体，格式规范见
//  http://wiki.ros.org/Bags/Format/2.0
//
//  文件结构：
//      #ROSBAG V2.0\n                     文件魔数
//      <BagHeader 记录>                    索引区偏移、连接数、Chunk 数
//      <Chunk 记录> +                      每个 Chunk 内含若干 Connection/Message 记录
//      <Chunk 记录> + ...
//      <IndexData 记录> +                  索引区：每个连接的消息时间与偏移
//      <ChunkInfo 记录> +                  索引区：每个 Chunk 的时间范围与统计
//
//  记录通用结构：
//      <header_len:u32><header:bytes><data_len:u32><data:bytes>
//  头部由若干字段串联而成：
//      <field_len:u32><name_len:u8><name>='='<value>
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace adsim {
namespace bag {

// ---------------------------------------------------------------------------
// 记录类型
// ---------------------------------------------------------------------------
constexpr std::uint8_t kOpMessageData = 0x02;
constexpr std::uint8_t kOpBagHeader = 0x03;
constexpr std::uint8_t kOpIndexData = 0x04;
constexpr std::uint8_t kOpChunk = 0x05;
constexpr std::uint8_t kOpChunkInfo = 0x06;
constexpr std::uint8_t kOpConnection = 0x07;

/// 文件魔数，长度固定 13 字节
constexpr std::size_t kMagicLen = 13;
extern const char kMagic[kMagicLen];

/// ROS 时间编码：高 32 位为秒，低 32 位为纳秒
inline std::uint64_t encodeTime(Timestamp ns) {
  const std::uint64_t secs = static_cast<std::uint64_t>(ns / 1000000000LL);
  const std::uint64_t nsecs = static_cast<std::uint64_t>(ns % 1000000000LL);
  return (secs << 32) | nsecs;
}

inline Timestamp decodeTime(std::uint64_t encoded) {
  const std::uint64_t secs = encoded >> 32;
  const std::uint64_t nsecs = encoded & 0xFFFFFFFFull;
  return static_cast<Timestamp>(secs) * 1000000000LL + static_cast<Timestamp>(nsecs);
}

// ---------------------------------------------------------------------------
// 压缩方式
// ---------------------------------------------------------------------------
enum class Compression { kNone, kBz2, kLz4, kUnknown };

Compression compressionFromString(const std::string& name);
const char* toString(Compression compression);

// ---------------------------------------------------------------------------
// 头部字段表
// ---------------------------------------------------------------------------

/// 解析后的记录头部（字段名 → 原始字节值）
class Header {
 public:
  /// 从原始字节解析；格式非法时抛出 std::runtime_error
  void parse(const std::uint8_t* data, std::size_t size);

  bool has(const std::string& key) const;
  std::uint8_t getU8(const std::string& key, std::uint8_t default_value = 0) const;
  std::uint32_t getU32(const std::string& key, std::uint32_t default_value = 0) const;
  std::uint64_t getU64(const std::string& key, std::uint64_t default_value = 0) const;
  std::string getString(const std::string& key, const std::string& default_value = "") const;

  const std::vector<std::pair<std::string, std::string>>& fields() const { return fields_; }

 private:
  const std::string* find(const std::string& key) const;

  std::vector<std::pair<std::string, std::string>> fields_;
};

/// 头部字段表构造器
class HeaderWriter {
 public:
  void addU8(const std::string& key, std::uint8_t value);
  void addU32(const std::string& key, std::uint32_t value);
  void addU64(const std::string& key, std::uint64_t value);
  void addString(const std::string& key, const std::string& value);
  void addBytes(const std::string& key, const void* data, std::size_t size);

  const std::vector<std::uint8_t>& bytes() const { return buffer_; }
  std::size_t size() const { return buffer_.size(); }
  void clear() { buffer_.clear(); }

 private:
  std::vector<std::uint8_t> buffer_;
};

// ---------------------------------------------------------------------------
// 连接信息（一个 topic 对应一条连接）
// ---------------------------------------------------------------------------
struct ConnectionInfo {
  std::uint32_t id{0};
  std::string topic;
  std::string type;                 ///< 消息类型，如 sensor_msgs/PointCloud2
  std::string md5sum;
  std::string message_definition;   ///< 完整的 .msg 文本定义

  std::size_t message_count{0};     ///< 该连接累计消息数（索引区统计）
};

// ---------------------------------------------------------------------------
// 消息视图
// ---------------------------------------------------------------------------

/// 消息的零拷贝视图。
///
/// data 指向读取器内部的 Chunk 缓冲区，**仅在回调执行期间有效**；
/// 若需在回调返回后继续使用数据，必须自行拷贝。
/// 这是为吞吐量做的取舍：100GB 级数据下逐条拷贝不可接受。
struct MessageView {
  std::uint32_t connection_id{0};
  Timestamp stamp{kInvalidTimestamp};
  const ConnectionInfo* connection{nullptr};
  const std::uint8_t* data{nullptr};
  std::size_t size{0};

  const std::string& topic() const;
  const std::string& type() const;
};

}  // namespace bag
}  // namespace adsim
