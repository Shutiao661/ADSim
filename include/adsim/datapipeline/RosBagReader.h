// =============================================================================
//  RosBagReader.h — ROS Bag 流式读取器
//
//  设计要点：
//    * **流式读取**：按 Chunk 载入内存并复用同一块缓冲区，内存占用与文件大小
//      无关。这是处理 100GB 级路测数据的前提——一次性载入不可行。
//    * **零拷贝访问**：回调拿到的是指向 Chunk 缓冲区的视图，不复制消息体。
//    * **索引驱动**：优先从索引区读取元信息，避免为了统计信息而全量扫描。
// =============================================================================
#pragma once

#include "adsim/datapipeline/RosBagFormat.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace adsim {

/// bag 文件元信息
struct BagInfo {
  std::string path;
  std::uint64_t file_size{0};
  std::size_t message_count{0};
  std::size_t chunk_count{0};
  std::size_t connection_count{0};
  Timestamp start_time{kInvalidTimestamp};
  Timestamp end_time{kInvalidTimestamp};
  double duration_seconds{0.0};
  bag::Compression compression{bag::Compression::kNone};

  std::vector<bag::ConnectionInfo> connections;
  std::vector<std::pair<std::string, std::size_t>> messages_per_topic;

  std::size_t messageCountFor(const std::string& topic) const;
  const bag::ConnectionInfo* findConnection(const std::string& topic) const;

  /// 生成可读的元信息摘要
  std::string summary() const;
};

class RosBagReader {
 public:
  /// 消息回调。返回 false 可提前终止遍历。
  ///
  /// 注意：回调收到的 MessageView::data 指向读取器内部缓冲区，
  /// **仅在本次回调执行期间有效**，需要留存时必须自行拷贝。
  using MessageCallback = std::function<bool(const bag::MessageView&)>;

  explicit RosBagReader(const std::string& path);
  ~RosBagReader();

  RosBagReader(const RosBagReader&) = delete;
  RosBagReader& operator=(const RosBagReader&) = delete;

  /// 打开文件并读取元信息。
  /// @param resolve_connections 是否扫描 Chunk 内的连接记录以获取 topic/类型。
  ///        仅需消息计数等统计信息时可置 false，避免额外 I/O。
  void open(bool resolve_connections = true);

  const BagInfo& info() const;

  /// 遍历全部消息
  void forEachMessage(const MessageCallback& callback);

  /// 仅遍历指定 topic 的消息
  void forEachMessage(const std::vector<std::string>& topics,
                      const MessageCallback& callback);

  /// 判断某 topic 是否存在于该 bag
  bool hasTopic(const std::string& topic) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace adsim
