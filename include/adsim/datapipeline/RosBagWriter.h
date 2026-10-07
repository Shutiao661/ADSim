// =============================================================================
//  RosBagWriter.h — ROS Bag 写入器
//
//  用途：
//    1. 将清洗/降噪后的数据落盘为新的 bag，供 CARLA 数字回放使用；
//    2. 生成合成路测数据集，用于管道性能压测与回归测试。
//
//  写入策略：消息先累积在内存 Chunk 中，达到阈值后整体落盘为一条 Chunk 记录；
//  文件关闭时补写索引区（IndexData + ChunkInfo）并回填 BagHeader，
//  产出的 bag 可被 ROS 官方工具直接读取。
// =============================================================================
#pragma once

#include "adsim/datapipeline/RosBagFormat.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace adsim {

class RosBagWriter {
 public:
  struct Config {
    /// 单个 Chunk 的字节阈值，达到后落盘
    std::size_t chunk_size_bytes{4 * 1024 * 1024};
    /// Chunk 压缩方式。需要本构建支持该算法（见 bag::isCompressionSupported）。
    bag::Compression compression{bag::Compression::kNone};
  };

  explicit RosBagWriter(const std::string& path);
  RosBagWriter(const std::string& path, const Config& config);
  ~RosBagWriter();

  RosBagWriter(const RosBagWriter&) = delete;
  RosBagWriter& operator=(const RosBagWriter&) = delete;

  /// 注册一个 topic 连接。同一 topic 重复注册会被忽略。
  /// @param md5sum 消息定义的 MD5，用于 ROS 端类型校验
  void addConnection(const std::string& topic,
                     const std::string& type,
                     const std::string& md5sum,
                     const std::string& message_definition);

  /// 写入一条消息。topic 必须已注册，否则抛出 std::runtime_error。
  void writeMessage(const std::string& topic, Timestamp stamp,
                    const void* data, std::size_t size);

  /// 落盘剩余数据并补写索引区；可重复调用，第二次起为空操作
  void close();

  std::size_t messageCount() const { return message_count_; }
  std::size_t chunkCount() const { return chunks_.size(); }
  bool isOpen() const { return stream_.is_open(); }

 private:
  struct ChunkIndexEntry {
    std::uint32_t connection_id{0};
    std::uint64_t time{0};
    std::uint32_t offset{0};
  };

  struct ChunkRecord {
    std::uint64_t file_position{0};
    std::uint64_t start_time{0};
    std::uint64_t end_time{0};
    std::size_t message_count{0};
    std::vector<ChunkIndexEntry> index;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> per_connection_counts;
  };

  struct Connection {
    std::uint32_t id{0};
    std::string topic;
    std::string type;
    std::string md5sum;
    std::string message_definition;
    bool written_in_current_chunk{false};
  };

  void writeBagHeader(std::uint64_t index_pos, std::uint32_t connection_count,
                      std::uint32_t chunk_count);
  void appendConnectionRecord(Connection& connection);
  void flushChunk();

  static void appendRecord(std::vector<std::uint8_t>& out,
                           const bag::HeaderWriter& header,
                           const void* data, std::size_t size);

  Config config_;
  std::string path_;
  std::ofstream stream_;

  std::vector<Connection> connections_;
  std::unordered_map<std::string, std::size_t> topic_to_index_;

  std::vector<std::uint8_t> chunk_buffer_;
  std::vector<ChunkIndexEntry> current_index_;
  std::unordered_map<std::uint32_t, std::uint32_t> current_counts_;
  std::uint64_t current_start_time_{0};
  std::uint64_t current_end_time_{0};
  std::size_t current_message_count_{0};

  std::vector<ChunkRecord> chunks_;
  std::size_t message_count_{0};

  std::uint64_t bag_header_position_{0};
  bool closed_{false};
};

}  // namespace adsim
