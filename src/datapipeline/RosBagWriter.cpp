#include "adsim/datapipeline/RosBagWriter.h"

#include "adsim/datapipeline/BagCompression.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace adsim {

namespace {

/// ROS 的 BagHeader 记录用于在关闭时原地回填索引偏移，因此必须定长。
///
/// 注意这里的长度语义与直觉不同（已对照 rosbag_storage/src/bag.cpp 的
/// writeFileHeaderRecord 核实）：ROS 取的是 **data_len = 4096 − header_len**，
/// 于是整条记录的实际字节数是
///     4 (header_len 字段) + header_len + 4 (data_len 字段) + (4096 − header_len)
///   = 4104
/// 而不是 4096。早期版本按"整条记录 4096"实现，读起来没问题（格式是自描述的，
/// 读取方只按长度字段跳过），但与 ROS 产出的文件逐字节不一致。
///
/// 填充字节是**空格**而不是零，这一点也要对齐。
constexpr std::size_t kFileHeaderLength = 4 * 1024;

}  // namespace

RosBagWriter::RosBagWriter(const std::string& path) : RosBagWriter(path, Config()) {}

RosBagWriter::RosBagWriter(const std::string& path, const Config& config)
    : config_(config), path_(path) {
  stream_.open(path_, std::ios::binary | std::ios::trunc);
  if (!stream_.is_open()) {
    throw std::runtime_error("无法创建 bag 文件: " + path_);
  }

  stream_.write(bag::kMagic, static_cast<std::streamsize>(bag::kMagicLen));
  bag_header_position_ = static_cast<std::uint64_t>(stream_.tellp());

  // 先写入占位头部，close() 时回填真实的索引偏移与计数
  writeBagHeader(0, 0, 0);

  if (config_.chunk_size_bytes == 0) {
    config_.chunk_size_bytes = 4 * 1024 * 1024;
  }
}

RosBagWriter::~RosBagWriter() {
  try {
    close();
  } catch (...) {
    // 析构函数中不抛出异常
  }
}

void RosBagWriter::appendRecord(std::vector<std::uint8_t>& out,
                                const bag::HeaderWriter& header,
                                const void* data, std::size_t size) {
  const auto header_len = static_cast<std::uint32_t>(header.size());
  const auto data_len = static_cast<std::uint32_t>(size);

  const std::uint8_t* header_len_bytes =
      reinterpret_cast<const std::uint8_t*>(&header_len);
  out.insert(out.end(), header_len_bytes, header_len_bytes + sizeof(header_len));

  const std::vector<std::uint8_t>& header_bytes = header.bytes();
  out.insert(out.end(), header_bytes.begin(), header_bytes.end());

  const std::uint8_t* data_len_bytes = reinterpret_cast<const std::uint8_t*>(&data_len);
  out.insert(out.end(), data_len_bytes, data_len_bytes + sizeof(data_len));

  if (size > 0 && data != nullptr) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), bytes, bytes + size);
  }
}

void RosBagWriter::writeBagHeader(std::uint64_t index_pos,
                                  std::uint32_t connection_count,
                                  std::uint32_t chunk_count) {
  bag::HeaderWriter header;
  header.addU8("op", bag::kOpBagHeader);
  header.addU64("index_pos", index_pos);
  header.addU32("conn_count", connection_count);
  header.addU32("chunk_count", chunk_count);

  const auto header_len = static_cast<std::uint32_t>(header.size());
  // 与 ROS 一致：填充量按 header_len 计算，而非整条记录长度
  const std::uint32_t data_len =
      header_len < kFileHeaderLength
          ? static_cast<std::uint32_t>(kFileHeaderLength - header_len)
          : 0u;

  const std::vector<std::uint8_t>& header_bytes = header.bytes();

  std::vector<std::uint8_t> record;
  record.reserve(4 + header_bytes.size() + 4 + data_len);

  const auto* header_len_bytes = reinterpret_cast<const std::uint8_t*>(&header_len);
  record.insert(record.end(), header_len_bytes, header_len_bytes + sizeof(header_len));
  record.insert(record.end(), header_bytes.begin(), header_bytes.end());

  const auto* data_len_bytes = reinterpret_cast<const std::uint8_t*>(&data_len);
  record.insert(record.end(), data_len_bytes, data_len_bytes + sizeof(data_len));
  // 填充用空格，与 ROS 的 writeFileHeaderRecord 保持一致
  record.insert(record.end(), data_len, static_cast<std::uint8_t>(' '));

  stream_.seekp(static_cast<std::streamoff>(bag_header_position_), std::ios::beg);
  stream_.write(reinterpret_cast<const char*>(record.data()),
                static_cast<std::streamsize>(record.size()));
}

void RosBagWriter::addConnection(const std::string& topic,
                                 const std::string& type,
                                 const std::string& md5sum,
                                 const std::string& message_definition) {
  if (topic_to_index_.count(topic) > 0) {
    return;
  }

  Connection conn;
  conn.id = static_cast<std::uint32_t>(connections_.size());
  conn.topic = topic;
  conn.type = type;
  conn.md5sum = md5sum;
  conn.message_definition = message_definition;

  topic_to_index_[topic] = connections_.size();
  connections_.push_back(std::move(conn));
}

void RosBagWriter::appendConnectionRecord(Connection& connection) {
  // 连接记录本身是一个"头部格式"的数据块，承载 topic/类型等元信息
  bag::HeaderWriter conn_data;
  conn_data.addString("topic", connection.topic);
  conn_data.addString("type", connection.type);
  conn_data.addString("md5sum", connection.md5sum);
  conn_data.addString("message_definition", connection.message_definition);

  bag::HeaderWriter record_header;
  record_header.addU8("op", bag::kOpConnection);
  record_header.addU32("conn", connection.id);
  record_header.addString("topic", connection.topic);

  appendRecord(chunk_buffer_, record_header, conn_data.bytes().data(), conn_data.size());
  connection.written_in_current_chunk = true;
}

void RosBagWriter::writeMessage(const std::string& topic, Timestamp stamp,
                                const void* data, std::size_t size) {
  const auto it = topic_to_index_.find(topic);
  if (it == topic_to_index_.end()) {
    throw std::runtime_error("写入前必须先注册 topic: " + topic);
  }

  Connection& connection = connections_[it->second];

  // 每个 Chunk 起始处必须包含其内消息用到的连接记录，保证可独立解码
  if (!connection.written_in_current_chunk) {
    appendConnectionRecord(connection);
  }

  bag::HeaderWriter header;
  header.addU8("op", bag::kOpMessageData);
  header.addU32("conn", connection.id);

  const std::uint64_t encoded_time = bag::encodeTime(stamp);
  header.addU64("time", encoded_time);

  // 索引记录的是消息记录在"未压缩 Chunk 数据"中的起始偏移
  const auto offset = static_cast<std::uint32_t>(chunk_buffer_.size());
  appendRecord(chunk_buffer_, header, data, size);

  ChunkIndexEntry entry;
  entry.connection_id = connection.id;
  entry.time = encoded_time;
  entry.offset = offset;
  current_index_.push_back(entry);

  ++current_counts_[connection.id];
  ++current_message_count_;
  ++message_count_;

  if (current_message_count_ == 1) {
    current_start_time_ = encoded_time;
    current_end_time_ = encoded_time;
  } else {
    current_start_time_ = std::min(current_start_time_, encoded_time);
    current_end_time_ = std::max(current_end_time_, encoded_time);
  }

  if (chunk_buffer_.size() >= config_.chunk_size_bytes) {
    flushChunk();
  }
}

void RosBagWriter::flushChunk() {
  if (current_message_count_ == 0) {
    return;
  }

  // ---- 按配置压缩 Chunk 数据 ----
  std::vector<std::uint8_t> payload;
  if (config_.compression != bag::Compression::kNone) {
    if (!bag::isCompressionSupported(config_.compression)) {
      throw std::runtime_error(
          std::string("本构建不支持 ") + bag::toString(config_.compression) +
          " 压缩（支持的压缩方式: " + bag::supportedCompressions() + "）");
    }

    std::string error;
    if (!bag::compressChunk(config_.compression, chunk_buffer_.data(),
                            chunk_buffer_.size(), payload, error)) {
      throw std::runtime_error("压缩 Chunk 失败: " + error);
    }
  }

  const std::uint8_t* payload_data =
      payload.empty() ? chunk_buffer_.data() : payload.data();
  const std::size_t payload_size = payload.empty() ? chunk_buffer_.size() : payload.size();

  bag::HeaderWriter header;
  header.addU8("op", bag::kOpChunk);
  header.addString("compression", bag::toString(config_.compression));
  // size 字段始终记录**解压后**的长度——这是 ROS 的约定，
  // 读取方据此分配解压缓冲区并校验解压结果
  header.addU32("size", static_cast<std::uint32_t>(chunk_buffer_.size()));

  ChunkRecord record;
  record.file_position = static_cast<std::uint64_t>(stream_.tellp());
  record.start_time = current_start_time_;
  record.end_time = current_end_time_;
  record.message_count = current_message_count_;
  record.index = current_index_;
  // 注意：索引里的 offset 是相对**解压后**数据的偏移（ROS 约定），
  // 因此这里记录的是压缩前的 chunk_buffer_ 布局，与 payload 无关
  record.per_connection_counts.assign(current_counts_.begin(), current_counts_.end());

  std::vector<std::uint8_t> bytes;
  bytes.reserve(payload_size + 64);
  appendRecord(bytes, header, payload_data, payload_size);

  stream_.write(reinterpret_cast<const char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size()));

  chunks_.push_back(std::move(record));

  // 重置 Chunk 状态
  chunk_buffer_.clear();
  current_index_.clear();
  current_counts_.clear();
  current_message_count_ = 0;
  current_start_time_ = 0;
  current_end_time_ = 0;
  for (Connection& conn : connections_) {
    conn.written_in_current_chunk = false;
  }
}

void RosBagWriter::close() {
  if (closed_ || !stream_.is_open()) {
    return;
  }
  closed_ = true;

  flushChunk();

  const std::uint64_t index_pos = static_cast<std::uint64_t>(stream_.tellp());

  // ---- 索引区：IndexData 记录（每条连接 × 每个 Chunk 一组）----
  for (const Connection& conn : connections_) {
    for (const ChunkRecord& chunk : chunks_) {
      std::vector<ChunkIndexEntry> entries;
      for (const ChunkIndexEntry& entry : chunk.index) {
        if (entry.connection_id == conn.id) entries.push_back(entry);
      }
      if (entries.empty()) continue;

      bag::HeaderWriter header;
      header.addU8("op", bag::kOpIndexData);
      header.addU32("ver", 1);
      header.addU32("conn", conn.id);
      header.addU32("count", static_cast<std::uint32_t>(entries.size()));

      // 索引项布局：time(u64) + offset(u32)，共 12 字节
      std::vector<std::uint8_t> data(entries.size() * 12);
      std::size_t offset = 0;
      for (const ChunkIndexEntry& entry : entries) {
        std::memcpy(data.data() + offset, &entry.time, sizeof(entry.time));
        offset += sizeof(entry.time);
        std::memcpy(data.data() + offset, &entry.offset, sizeof(entry.offset));
        offset += sizeof(entry.offset);
      }

      std::vector<std::uint8_t> bytes;
      appendRecord(bytes, header, data.data(), data.size());
      stream_.write(reinterpret_cast<const char*>(bytes.data()),
                    static_cast<std::streamsize>(bytes.size()));
    }
  }

  // ---- 索引区：ChunkInfo 记录（每个 Chunk 一条）----
  for (const ChunkRecord& chunk : chunks_) {
    bag::HeaderWriter header;
    header.addU8("op", bag::kOpChunkInfo);
    header.addU32("ver", 1);
    header.addU64("chunk_pos", chunk.file_position);
    header.addU64("start_time", chunk.start_time);
    header.addU64("end_time", chunk.end_time);
    header.addU32("count", static_cast<std::uint32_t>(chunk.per_connection_counts.size()));

    std::vector<std::uint8_t> data(chunk.per_connection_counts.size() * 8);
    std::size_t offset = 0;
    for (const auto& pair : chunk.per_connection_counts) {
      std::memcpy(data.data() + offset, &pair.first, sizeof(pair.first));
      offset += sizeof(pair.first);
      std::memcpy(data.data() + offset, &pair.second, sizeof(pair.second));
      offset += sizeof(pair.second);
    }

    std::vector<std::uint8_t> bytes;
    appendRecord(bytes, header, data.data(), data.size());
    stream_.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
  }

  stream_.flush();

  // ---- 回填 BagHeader ----
  writeBagHeader(index_pos, static_cast<std::uint32_t>(connections_.size()),
                 static_cast<std::uint32_t>(chunks_.size()));

  stream_.flush();
  stream_.close();
}

}  // namespace adsim
