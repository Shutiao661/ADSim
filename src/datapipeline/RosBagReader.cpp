#include "adsim/datapipeline/RosBagReader.h"

#include "adsim/datapipeline/BagCompression.h"

#include "adsim/common/Logger.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace adsim {

namespace {

/// 从记录头部解析出 topic 连接的全部元信息
bag::ConnectionInfo parseConnectionRecord(const bag::Header& header,
                                          const std::uint8_t* data,
                                          std::size_t size) {
  bag::ConnectionInfo info;
  info.id = header.getU32("conn");
  info.topic = header.getString("topic");

  // 连接记录的 data 部分本身又是一个头部格式的块，承载类型信息
  bag::Header conn_header;
  conn_header.parse(data, size);

  info.topic = conn_header.getString("topic", info.topic);
  info.type = conn_header.getString("type");
  info.md5sum = conn_header.getString("md5sum");
  info.message_definition = conn_header.getString("message_definition");
  return info;
}

/// 登记一条连接记录；同一 id 重复出现时忽略（Chunk 之间会重复写连接记录）
void registerConnection(bag::ConnectionInfo connection,
                        std::unordered_map<std::uint32_t, std::size_t>& index,
                        std::vector<bag::ConnectionInfo>& connections) {
  const auto it = index.find(connection.id);
  if (it != index.end()) return;

  index[connection.id] = connections.size();
  connections.push_back(std::move(connection));
}

/// 在一块**已解压**的 Chunk 数据中扫描连接记录。
///
/// 与流式扫描的区别在于数据已在内存中：无需 seek，只需顺序走记录头。
/// 压缩 Chunk 必须先解压才能走到这里——连接记录存在 Chunk 内部，
/// 跳过它就等于整包的 topic / 消息类型全部丢失。
void scanConnectionsInChunk(const std::uint8_t* data, std::size_t size,
                            std::unordered_map<std::uint32_t, std::size_t>& index,
                            std::vector<bag::ConnectionInfo>& connections) {
  const std::uint8_t* cursor = data;
  const std::uint8_t* end = data + size;

  while (cursor + 8 <= end) {
    std::uint32_t header_len = 0;
    std::memcpy(&header_len, cursor, sizeof(header_len));
    cursor += 4;
    if (cursor + header_len + 4 > end) break;

    bag::Header header;
    header.parse(cursor, header_len);
    cursor += header_len;

    std::uint32_t data_len = 0;
    std::memcpy(&data_len, cursor, sizeof(data_len));
    cursor += 4;
    if (cursor + data_len > end) break;

    if (header.getU8("op") == bag::kOpConnection) {
      registerConnection(parseConnectionRecord(header, cursor, data_len), index,
                         connections);
    }

    cursor += data_len;
  }
}

/// 读取一个记录：头部 + 数据长度。数据本体不读入，由调用方按需跳过或读取。
/// @return 是否成功读到一条记录（false 表示到达文件末尾）
bool readRecordHeader(std::ifstream& stream, bag::Header& header, std::uint32_t& data_len) {
  std::uint32_t header_len = 0;
  stream.read(reinterpret_cast<char*>(&header_len), sizeof(header_len));
  if (stream.gcount() != static_cast<std::streamsize>(sizeof(header_len))) {
    return false;
  }

  // 防御性检查：损坏文件可能给出天文数字的头部长度
  constexpr std::uint32_t kMaxHeaderLen = 1u << 20;  // 1MB
  if (header_len == 0 || header_len > kMaxHeaderLen) {
    throw std::runtime_error("bag 记录头部长度非法: " + std::to_string(header_len));
  }

  std::vector<std::uint8_t> header_bytes(header_len);
  stream.read(reinterpret_cast<char*>(header_bytes.data()),
              static_cast<std::streamsize>(header_len));
  if (stream.gcount() != static_cast<std::streamsize>(header_len)) {
    throw std::runtime_error("bag 文件在记录头部处意外结束");
  }

  stream.read(reinterpret_cast<char*>(&data_len), sizeof(data_len));
  if (stream.gcount() != static_cast<std::streamsize>(sizeof(data_len))) {
    throw std::runtime_error("bag 文件在记录数据长度处意外结束");
  }

  header.parse(header_bytes.data(), header_bytes.size());
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct RosBagReader::Impl {
  std::string path;
  std::ifstream stream;
  BagInfo info;

  /// 连接 id → info.connections 下标
  std::unordered_map<std::uint32_t, std::size_t> connection_index;

  /// Chunk 缓冲区，跨 Chunk 复用，避免反复分配。
  /// 压缩包场景下存放的是**解压后**的数据。
  std::vector<std::uint8_t> chunk_buffer;

  /// 压缩 Chunk 的原始数据缓冲区，同样跨 Chunk 复用
  std::vector<std::uint8_t> compressed_buffer;

  /// 第一个 Chunk 的文件偏移（紧随 bag header 记录之后）
  std::uint64_t first_chunk_pos{0};

  void readExactly(void* destination, std::size_t size);
  void skip(std::uint64_t bytes);
};

void RosBagReader::Impl::readExactly(void* destination, std::size_t size) {
  stream.read(static_cast<char*>(destination), static_cast<std::streamsize>(size));
  if (stream.gcount() != static_cast<std::streamsize>(size)) {
    throw std::runtime_error("bag 文件读取不足，可能在传输中被截断");
  }
}

void RosBagReader::Impl::skip(std::uint64_t bytes) {
  stream.seekg(static_cast<std::streamoff>(bytes), std::ios::cur);
  if (!stream) {
    throw std::runtime_error("bag 文件定位失败，文件可能已损坏");
  }
}

// ---------------------------------------------------------------------------
// BagInfo
// ---------------------------------------------------------------------------

std::size_t BagInfo::messageCountFor(const std::string& topic) const {
  for (const auto& entry : messages_per_topic) {
    if (entry.first == topic) return entry.second;
  }
  return 0;
}

const bag::ConnectionInfo* BagInfo::findConnection(const std::string& topic) const {
  for (const bag::ConnectionInfo& conn : connections) {
    if (conn.topic == topic) return &conn;
  }
  return nullptr;
}

std::string BagInfo::summary() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);

  oss << "bag 文件: " << path << "\n";
  oss.precision(1);
  oss << "  文件大小  : " << static_cast<double>(file_size) / (1024.0 * 1024.0) << " MB\n";
  oss << "  消息总数  : " << message_count << "\n";
  oss << "  Chunk 数  : " << chunk_count << "\n";
  oss << "  连接数    : " << connection_count << "\n";
  oss << "  压缩方式  : " << bag::toString(compression) << "\n";
  oss.precision(3);
  if (start_time != kInvalidTimestamp && end_time != kInvalidTimestamp) {
    oss << "  时间跨度  : " << duration_seconds << " s\n";
  }
  oss << "  话题分布  :\n";
  for (const auto& entry : messages_per_topic) {
    oss.precision(1);
    const double pct = message_count == 0
                           ? 0.0
                           : 100.0 * static_cast<double>(entry.second) /
                                 static_cast<double>(message_count);
    oss << "    - " << entry.first << "  " << entry.second << " 条 (" << pct << "%)\n";
  }
  return oss.str();
}

// ---------------------------------------------------------------------------
// RosBagReader
// ---------------------------------------------------------------------------

RosBagReader::RosBagReader(const std::string& path) : impl_(new Impl()) {
  impl_->path = path;
  impl_->info.path = path;
}

RosBagReader::~RosBagReader() = default;

const BagInfo& RosBagReader::info() const { return impl_->info; }

void RosBagReader::open(bool resolve_connections) {
  Impl& impl = *impl_;
  impl.info = BagInfo();
  impl.info.path = impl.path;
  impl.connection_index.clear();

  impl.stream.open(impl.path, std::ios::binary);
  if (!impl.stream.is_open()) {
    throw std::runtime_error("无法打开 bag 文件: " + impl.path);
  }

  // 文件大小
  impl.stream.seekg(0, std::ios::end);
  impl.info.file_size = static_cast<std::uint64_t>(impl.stream.tellg());
  impl.stream.seekg(0, std::ios::beg);

  // 校验魔数
  char magic[bag::kMagicLen] = {0};
  impl.readExactly(magic, bag::kMagicLen);
  if (std::memcmp(magic, bag::kMagic, bag::kMagicLen) != 0) {
    throw std::runtime_error("不是合法的 ROS bag v2.0 文件: " + impl.path);
  }

  // bag header 记录
  bag::Header header;
  std::uint32_t data_len = 0;
  if (!readRecordHeader(impl.stream, header, data_len)) {
    throw std::runtime_error("bag 文件缺少头部记录: " + impl.path);
  }
  if (header.getU8("op") != bag::kOpBagHeader) {
    throw std::runtime_error("bag 文件首个记录不是 BagHeader");
  }

  const std::uint64_t index_pos = header.getU64("index_pos");
  impl.info.connection_count = header.getU32("conn_count");
  impl.info.chunk_count = header.getU32("chunk_count");

  // 跳过 bag header 的数据区（其中为对齐填充），定位到第一个 Chunk
  impl.skip(data_len);
  impl.first_chunk_pos = static_cast<std::uint64_t>(impl.stream.tellg());

  if (index_pos > 0 && index_pos < impl.info.file_size) {
    // ---- 从索引区读取统计信息（无需扫描消息体）----
    impl.stream.seekg(static_cast<std::streamoff>(index_pos), std::ios::beg);

    std::unordered_map<std::uint32_t, std::size_t> counts;
    std::vector<std::uint8_t> data;

    while (true) {
      bag::Header rec_header;
      std::uint32_t rec_data_len = 0;
      const std::streampos before = impl.stream.tellg();
      if (!readRecordHeader(impl.stream, rec_header, rec_data_len)) break;
      (void)before;

      const std::uint8_t op = rec_header.getU8("op");

      if (op == bag::kOpIndexData) {
        // 索引项：每条 (time u64, offset u32)
        const std::uint32_t conn_id = rec_header.getU32("conn");
        const std::uint32_t count = rec_header.getU32("count");
        counts[conn_id] += count;
        impl.info.message_count += count;
        impl.skip(rec_data_len);
      } else if (op == bag::kOpChunkInfo) {
        const std::uint64_t start_time = rec_header.getU64("start_time");
        const std::uint64_t end_time = rec_header.getU64("end_time");

        const Timestamp start_ns = bag::decodeTime(start_time);
        const Timestamp end_ns = bag::decodeTime(end_time);
        if (impl.info.start_time == kInvalidTimestamp || start_ns < impl.info.start_time) {
          impl.info.start_time = start_ns;
        }
        if (impl.info.end_time == kInvalidTimestamp || end_ns > impl.info.end_time) {
          impl.info.end_time = end_ns;
        }
        impl.skip(rec_data_len);
      } else {
        impl.skip(rec_data_len);
      }
    }

    // ---- 解析 Chunk 内的连接记录以获取 topic 与类型 ----
    if (resolve_connections) {
      impl.stream.clear();
      impl.stream.seekg(static_cast<std::streamoff>(impl.first_chunk_pos), std::ios::beg);

      // 逐个 Chunk 只读记录头并跳过数据体；连接记录通常出现在 Chunk 起始处，
      // 因此大多数文件只需扫描极少量数据即可完成解析。
      std::size_t remaining_chunks = impl.info.chunk_count;
      while (remaining_chunks > 0) {
        bag::Header chunk_header;
        std::uint32_t chunk_data_len = 0;
        if (!readRecordHeader(impl.stream, chunk_header, chunk_data_len)) break;

        if (chunk_header.getU8("op") != bag::kOpChunk) break;

        const auto compression =
            bag::compressionFromString(chunk_header.getString("compression", "none"));
        if (impl.info.compression == bag::Compression::kNone) {
          impl.info.compression = compression;
        }

        // 压缩 Chunk：若本构建支持该算法则解压后扫描连接记录，否则跳过。
        //
        // 这一步不能省。连接记录（topic / 消息类型 / MD5）存在 Chunk 内部，
        // 若整包都是压缩的而我们直接跳过，就会一个连接都解析不出来——
        // 表现为 bag 能打开、消息数对，但话题列表为空。
        if (compression != bag::Compression::kNone) {
          if (!bag::isCompressionSupported(compression)) {
            // 不支持解压：跳过连接解析，但仍记录压缩方式供上层给出提示
            impl.skip(chunk_data_len);
            --remaining_chunks;
            continue;
          }

          const std::size_t uncompressed_size = chunk_header.getU32("size", 0);
          std::vector<std::uint8_t> compressed(chunk_data_len);
          if (chunk_data_len > 0) {
            impl.readExactly(compressed.data(), chunk_data_len);
          }

          std::string error;
          std::vector<std::uint8_t> plain;
          if (!bag::decompressChunk(compression, compressed.data(), chunk_data_len,
                                    uncompressed_size, plain, error)) {
            throw std::runtime_error("解析 Chunk 连接记录时解压失败: " + error);
          }

          scanConnectionsInChunk(plain.data(), plain.size(), impl.connection_index,
                                 impl.info.connections);

          if (impl.info.connections.size() >= impl.info.connection_count) break;
          --remaining_chunks;
          continue;
        }

        const std::uint64_t chunk_start = static_cast<std::uint64_t>(impl.stream.tellg());
        const std::uint64_t chunk_end = chunk_start + chunk_data_len;

        while (static_cast<std::uint64_t>(impl.stream.tellg()) < chunk_end) {
          bag::Header rec_header;
          std::uint32_t rec_data_len = 0;
          if (!readRecordHeader(impl.stream, rec_header, rec_data_len)) break;

          if (rec_header.getU8("op") == bag::kOpConnection) {
            std::vector<std::uint8_t> conn_bytes(rec_data_len);
            if (rec_data_len > 0) {
              impl.readExactly(conn_bytes.data(), rec_data_len);
            }
            registerConnection(
                parseConnectionRecord(rec_header, conn_bytes.data(), conn_bytes.size()),
                impl.connection_index, impl.info.connections);
          } else {
            impl.skip(rec_data_len);
          }
        }

        // 若连接已全部解析出来则提前结束扫描
        if (impl.info.connections.size() >= impl.info.connection_count) break;
        --remaining_chunks;
      }
    }

    // 按 topic 汇总消息数
    for (const auto& entry : counts) {
      const auto it = impl.connection_index.find(entry.first);
      if (it != impl.connection_index.end()) {
        impl.info.connections[it->second].message_count = entry.second;
        impl.info.messages_per_topic.emplace_back(
            impl.info.connections[it->second].topic, entry.second);
      }
    }
  } else {
    // 无索引区（例如边写边读的 bag）：退化为全量扫描统计
    ADSIM_LOG_WARN("bag 缺少索引区，将全量扫描统计: ", impl.path);
    impl.stream.clear();
    impl.stream.seekg(static_cast<std::streamoff>(impl.first_chunk_pos), std::ios::beg);
    // 全量扫描交由 forEachMessage 完成；此处仅标记
    impl.info.message_count = 0;
  }

  if (impl.info.start_time != kInvalidTimestamp) {
    impl.info.duration_seconds =
        toSeconds(impl.info.end_time - impl.info.start_time);
  }

  std::sort(impl.info.messages_per_topic.begin(), impl.info.messages_per_topic.end(),
            [](const std::pair<std::string, std::size_t>& a,
               const std::pair<std::string, std::size_t>& b) { return a.second > b.second; });
}

bool RosBagReader::hasTopic(const std::string& topic) const {
  for (const bag::ConnectionInfo& conn : impl_->info.connections) {
    if (conn.topic == topic) return true;
  }
  return false;
}

void RosBagReader::forEachMessage(const MessageCallback& callback) {
  forEachMessage({}, callback);
}

void RosBagReader::forEachMessage(const std::vector<std::string>& topics,
                                  const MessageCallback& callback) {
  Impl& impl = *impl_;
  if (!impl.stream.is_open()) {
    throw std::runtime_error("遍历前必须先调用 open()");
  }

  std::unordered_set<std::string> topic_filter(topics.begin(), topics.end());

  impl.stream.clear();
  impl.stream.seekg(static_cast<std::streamoff>(impl.first_chunk_pos), std::ios::beg);

  // 局部连接表：优先使用 open() 已解析的结果，并补充扫描中发现的连接
  std::unordered_map<std::uint32_t, bag::ConnectionInfo> local_connections;
  for (const bag::ConnectionInfo& conn : impl.info.connections) {
    local_connections[conn.id] = conn;
  }

  bool keep_going = true;

  while (keep_going) {
    bag::Header record_header;
    std::uint32_t record_data_len = 0;
    if (!readRecordHeader(impl.stream, record_header, record_data_len)) break;

    const std::uint8_t op = record_header.getU8("op");

    if (op == bag::kOpChunk) {
      const auto compression =
          bag::compressionFromString(record_header.getString("compression", "none"));

      const std::uint8_t* chunk_data = nullptr;
      std::size_t chunk_size = 0;

      if (compression == bag::Compression::kNone) {
        // 未压缩：直接载入复用缓冲区（内存占用与文件大小无关）
        impl.chunk_buffer.resize(record_data_len);
        if (record_data_len > 0) {
          impl.readExactly(impl.chunk_buffer.data(), record_data_len);
        }
        chunk_data = impl.chunk_buffer.data();
        chunk_size = record_data_len;
      } else {
        if (!bag::isCompressionSupported(compression)) {
          throw std::runtime_error(
              std::string("本构建不支持 ") + bag::toString(compression) +
              " 压缩的 Chunk（支持的压缩方式: " + bag::supportedCompressions() +
              "）。请安装对应的库后重新构建，"
              "或使用 compression:=none 重新录制。");
        }

        // 压缩数据读进独立缓冲区，解压结果写入 chunk_buffer。
        // 两个缓冲区都跨 Chunk 复用，内存占用仍与文件大小无关。
        impl.compressed_buffer.resize(record_data_len);
        if (record_data_len > 0) {
          impl.readExactly(impl.compressed_buffer.data(), record_data_len);
        }

        const std::size_t uncompressed_size = record_header.getU32("size", 0);
        std::string error;
        if (!bag::decompressChunk(compression, impl.compressed_buffer.data(),
                                  record_data_len, uncompressed_size,
                                  impl.chunk_buffer, error)) {
          throw std::runtime_error("解压 " + std::string(bag::toString(compression)) +
                                   " Chunk 失败: " + error);
        }
        chunk_data = impl.chunk_buffer.data();
        chunk_size = impl.chunk_buffer.size();
      }

      const std::uint8_t* cursor = chunk_data;
      const std::uint8_t* chunk_end = cursor + chunk_size;

      while (cursor + 8 <= chunk_end) {
        std::uint32_t header_len = 0;
        std::memcpy(&header_len, cursor, sizeof(header_len));
        cursor += 4;
        if (cursor + header_len + 4 > chunk_end) break;

        bag::Header rec_header;
        rec_header.parse(cursor, header_len);
        cursor += header_len;

        std::uint32_t rec_data_len = 0;
        std::memcpy(&rec_data_len, cursor, sizeof(rec_data_len));
        cursor += 4;
        if (cursor + rec_data_len > chunk_end) break;

        const std::uint8_t rec_op = rec_header.getU8("op");

        if (rec_op == bag::kOpConnection) {
          bag::ConnectionInfo conn =
              parseConnectionRecord(rec_header, cursor, rec_data_len);
          local_connections[conn.id] = std::move(conn);
        } else if (rec_op == bag::kOpMessageData) {
          bag::MessageView view;
          view.connection_id = rec_header.getU32("conn");
          view.stamp = bag::decodeTime(rec_header.getU64("time"));
          view.data = cursor;
          view.size = rec_data_len;

          const auto it = local_connections.find(view.connection_id);
          if (it != local_connections.end()) {
            view.connection = &it->second;
          }

          if (topic_filter.empty() ||
              (view.connection != nullptr && topic_filter.count(view.topic()) > 0)) {
            if (!callback(view)) {
              keep_going = false;
              break;
            }
          }
        }

        cursor += rec_data_len;
      }
    } else if (op == bag::kOpIndexData || op == bag::kOpChunkInfo) {
      // 进入索引区，消息已全部遍历完毕
      break;
    } else {
      impl.skip(record_data_len);
    }
  }
}

}  // namespace adsim
