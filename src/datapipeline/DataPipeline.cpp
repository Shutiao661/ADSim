#include "adsim/datapipeline/DataPipeline.h"

#include "adsim/common/Logger.h"
#include "adsim/common/Profiler.h"
#include "adsim/datapipeline/MessageCodec.h"
#include "adsim/datapipeline/RosBagWriter.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <sstream>
#include <thread>

namespace adsim {

namespace {

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

/// 流水线中的工作项：载荷由内存池承载，可安全跨线程传递
struct WorkItem {
  PooledBlock block;
  std::size_t size{0};
  std::uint32_t connection_id{0};
  Timestamp stamp{kInvalidTimestamp};
  MessageKind kind{MessageKind::kUnknown};

  WorkItem() = default;
  WorkItem(WorkItem&&) = default;
  WorkItem& operator=(WorkItem&&) = default;
  WorkItem(const WorkItem&) = delete;
  WorkItem& operator=(const WorkItem&) = delete;
};

MessageKind classifyMessage(const std::string& type) {
  if (type == "sensor_msgs/PointCloud2") return MessageKind::kLidar;
  if (type == "sensor_msgs/NavSatFix") return MessageKind::kGps;
  if (type == "nav_msgs/Odometry") return MessageKind::kOdometry;
  return MessageKind::kUnknown;
}

struct DecodedLidar {
  Timestamp stamp{kInvalidTimestamp};
  LidarFrame frame;             ///< 若开启降噪，此处已是降噪后的帧
  std::size_t points_in{0};     ///< 降噪前的点数，用于量化压缩比
  double decode_ms{0.0};
  double filter_ms{0.0};
};

}  // namespace

const char* toString(MessageKind kind) {
  switch (kind) {
    case MessageKind::kLidar: return "点云";
    case MessageKind::kGps: return "GPS";
    case MessageKind::kOdometry: return "车辆状态";
    case MessageKind::kUnknown: return "未知";
  }
  return "未知";
}

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

DataPipeline::DataPipeline() = default;
DataPipeline::DataPipeline(const Config& config) : config_(config) {}

// ---------------------------------------------------------------------------
// 元信息探查
// ---------------------------------------------------------------------------

BagInfo DataPipeline::probe(const std::string& bag_path) {
  RosBagReader reader(bag_path);
  reader.open(true);
  return reader.info();
}

// ---------------------------------------------------------------------------
// 主管道
// ---------------------------------------------------------------------------

DataPipeline::Report DataPipeline::run(const std::string& bag_path,
                                       const std::string& output_bag_path) {
  Report report;
  report.input_path = bag_path;
  report.output_path = output_bag_path;

  const auto wall_start = Clock::now();

  RosBagReader reader(bag_path);
  {
    ADSIM_PROFILE_SCOPE("阶段1_读取bag元信息");
    reader.open(true);
  }
  report.bytes_processed = static_cast<double>(reader.info().file_size);

  ThreadPool pool(config_.thread_count);
  ADSIM_LOG_INFO("数据管道启动: 工作线程 ", pool.size(), " 个, 缓冲槽位 ",
                 config_.queue_capacity);

  // ---- 跨线程收集区 ----
  std::mutex collector_mutex;
  std::vector<DecodedLidar> lidars;
  std::vector<GpsFrame> gps_frames;
  std::vector<VehicleState> odometry;
  std::size_t decode_failures = 0;
  double lidar_decode_ms = 0.0;
  double lidar_filter_ms = 0.0;

  const PointCloudFilter lidar_filter(config_.lidar_filter);

  // 每个工作项的处理逻辑：解码 → 降噪
  auto process = [&](WorkItem& item) {
    switch (item.kind) {
      case MessageKind::kLidar: {
        const auto t0 = Clock::now();
        LidarFrame frame = ros::decodePointCloud2(
            static_cast<const std::uint8_t*>(item.block.data()), item.size);
        const double decode_ms = millisecondsSince(t0);

        if (frame.stamp == kInvalidTimestamp) {
          std::lock_guard<std::mutex> lock(collector_mutex);
          ++decode_failures;
          break;
        }

        const std::size_t points_in = frame.points.size();

        const auto t1 = Clock::now();
        PointCloudFilter::Statistics filter_stats;
        if (config_.enable_lidar_filter) {
          frame = lidar_filter.filter(frame, &filter_stats);
        }
        const double filter_ms = millisecondsSince(t1);

        {
          std::lock_guard<std::mutex> lock(collector_mutex);
          lidar_decode_ms += decode_ms;
          lidar_filter_ms += filter_ms;
          lidars.push_back(
              DecodedLidar{item.stamp, std::move(frame), points_in, decode_ms, filter_ms});
        }
        break;
      }

      case MessageKind::kGps: {
        const GpsFrame frame = ros::decodeNavSatFix(
            static_cast<const std::uint8_t*>(item.block.data()), item.size);
        if (frame.stamp == kInvalidTimestamp) {
          std::lock_guard<std::mutex> lock(collector_mutex);
          ++decode_failures;
          break;
        }
        std::lock_guard<std::mutex> lock(collector_mutex);
        gps_frames.push_back(frame);
        break;
      }

      case MessageKind::kOdometry: {
        const VehicleState state = ros::decodeOdometry(
            static_cast<const std::uint8_t*>(item.block.data()), item.size);
        if (state.stamp == kInvalidTimestamp) {
          std::lock_guard<std::mutex> lock(collector_mutex);
          ++decode_failures;
          break;
        }
        std::lock_guard<std::mutex> lock(collector_mutex);
        odometry.push_back(state);
        break;
      }

      case MessageKind::kUnknown:
        break;
    }
  };

  // ---- 读取线程：解包并拷贝入内存池 ----
  SpscRingBuffer<WorkItem> queue(config_.queue_capacity);
  std::atomic<bool> reader_finished{false};
  std::atomic<std::size_t> messages_read{0};

  std::thread reader_thread([&]() {
    ADSIM_PROFILE_SCOPE("阶段2_解包");

    reader.forEachMessage([&](const bag::MessageView& view) {
      messages_read.fetch_add(1, std::memory_order_relaxed);

      const MessageKind kind = classifyMessage(view.type());
      if (kind == MessageKind::kUnknown || view.size == 0) {
        return true;  // 跳过不支持的消息类型
      }

      WorkItem item;
      item.size = view.size;
      item.connection_id = view.connection_id;
      item.stamp = view.stamp;
      item.kind = kind;

      // 消息视图指向读取器内部 Chunk 缓冲区，无法跨线程持有，
      // 必须拷贝一份；这里从内存池取块，避免每条消息一次 malloc
      PooledBlock block(MemoryPoolRegistry::get(view.size));
      if (block.capacity() < view.size) {
        return true;  // 超出池块上限的异常大消息，跳过
      }
      std::memcpy(block.data(), view.data, view.size);
      item.block = std::move(block);

      // 背压：缓冲满则等待消费，防止内存无界增长
      while (!queue.push(std::move(item))) {
        std::this_thread::yield();
      }
      return true;
    });

    reader_finished.store(true, std::memory_order_release);
  });

  // ---- 派发线程（本线程）：从环形缓冲取任务投递到线程池 ----
  {
    ADSIM_PROFILE_SCOPE("阶段3_派发与并行处理");

    while (!reader_finished.load(std::memory_order_acquire) || !queue.empty()) {
      WorkItem item;
      if (queue.pop(item)) {
        // 管道资源不能阻塞派发，异常由工作线程内部兜底
        try {
          pool.enqueue([&process, item = std::move(item)]() mutable { process(item); });
        } catch (const std::exception& e) {
          ADSIM_LOG_ERROR("任务投递失败: ", e.what());
        }
      } else {
        std::this_thread::yield();
      }
    }
  }

  reader_thread.join();
  {
    ADSIM_PROFILE_SCOPE("阶段4_等待工作线程结束");
    pool.waitIdle();
  }

  report.messages_read = messages_read.load();
  report.decode_failures = decode_failures;

  // ---- 按时间排序，保证后续对齐与落盘有序 ----
  const auto byStamp = [](const auto& a, const auto& b) { return a.stamp < b.stamp; };
  std::sort(lidars.begin(), lidars.end(), byStamp);
  std::sort(gps_frames.begin(), gps_frames.end(), byStamp);
  std::sort(odometry.begin(), odometry.end(), byStamp);

  report.lidar_frames = lidars.size();
  report.gps_frames = gps_frames.size();
  report.odometry_frames = odometry.size();

  for (const DecodedLidar& item : lidars) {
    report.lidar_points_in += item.points_in;
    report.lidar_points_out += item.frame.points.size();
  }

  // ---- GPS 滤波：剔除跳点并按滤波结果重建经纬度 ----
  std::vector<GpsFrame> cleaned_gps;
  if (config_.enable_gps_filter && !gps_frames.empty()) {
    ADSIM_PROFILE_SCOPE("阶段5_GPS滤波");

    GpsFilter gps_filter(config_.gps_filter);
    cleaned_gps.reserve(gps_frames.size());

    for (const GpsFrame& frame : gps_frames) {
      Vec2 filtered_position;
      const bool accepted = gps_filter.update(frame, filtered_position);

      if (!accepted) {
        ++report.gps_rejected;
        continue;  // 跳点直接丢弃，不进入下游
      }

      GpsFrame cleaned = frame;
      double latitude = 0.0;
      double longitude = 0.0;
      double altitude = 0.0;
      gps_filter.projector().toGeodetic(
          Vec3{filtered_position.x, filtered_position.y, 0.0}, latitude, longitude, altitude);
      cleaned.latitude = latitude;
      cleaned.longitude = longitude;
      cleaned.altitude = frame.altitude;  // 高程未参与滤波，保持原值
      cleaned_gps.push_back(cleaned);
    }

    ADSIM_LOG_INFO(gps_filter.statistics().toString());
  } else {
    cleaned_gps = gps_frames;
  }

  report.gps_frames = cleaned_gps.size();

  // ---- 时间对齐 ----
  std::vector<AlignedFrame> aligned;
  if (config_.keep_lidar || !cleaned_gps.empty()) {
    ADSIM_PROFILE_SCOPE("阶段6_时间对齐");

    std::vector<LidarFrame> lidar_frames_only;
    if (config_.keep_lidar) {
      lidar_frames_only.reserve(lidars.size());
      for (DecodedLidar& item : lidars) {
        lidar_frames_only.push_back(std::move(item.frame));
      }
    }

    TimeAligner aligner(config_.time_aligner);
    aligned = aligner.align(cleaned_gps, odometry, lidar_frames_only);
    report.alignment = TimeAligner::analyze(aligned);
    report.aligned_frames = aligned.size();

    ADSIM_LOG_INFO(report.alignment.toString());
  }

  // ---- 落盘 ----
  if (!output_bag_path.empty()) {
    ADSIM_PROFILE_SCOPE("阶段7_写出bag");

    RosBagWriter writer(output_bag_path);
    const ros::MessageTypeInfo lidar_type = ros::lookupMessageType("sensor_msgs/PointCloud2");
    const ros::MessageTypeInfo gps_type = ros::lookupMessageType("sensor_msgs/NavSatFix");
    const ros::MessageTypeInfo odom_type = ros::lookupMessageType("nav_msgs/Odometry");

    writer.addConnection("/lidar/clean", lidar_type.type, lidar_type.md5sum,
                         lidar_type.message_definition);
    writer.addConnection("/gps/clean", gps_type.type, gps_type.md5sum,
                         gps_type.message_definition);
    writer.addConnection("/odom/clean", odom_type.type, odom_type.md5sum,
                         odom_type.message_definition);

    for (const AlignedFrame& frame : aligned) {
      if (frame.has_lidar) {
        const std::vector<std::uint8_t> payload = ros::encodePointCloud2(frame.lidar);
        writer.writeMessage("/lidar/clean", frame.lidar.stamp, payload.data(), payload.size());
        ++report.output_messages;
      }
      if (frame.has_gps) {
        const std::vector<std::uint8_t> payload = ros::encodeNavSatFix(frame.gps);
        writer.writeMessage("/gps/clean", frame.gps.stamp, payload.data(), payload.size());
        ++report.output_messages;
      }
      if (frame.has_odom) {
        const std::vector<std::uint8_t> payload = ros::encodeOdometry(frame.odom);
        writer.writeMessage("/odom/clean", frame.odom.stamp, payload.data(), payload.size());
        ++report.output_messages;
      }
    }

    writer.close();
  }

  // ---- 统计 ----
  report.elapsed_seconds =
      std::chrono::duration<double>(Clock::now() - wall_start).count();
  if (report.elapsed_seconds > 0.0) {
    report.throughput_mb_per_second =
        report.bytes_processed / (1024.0 * 1024.0) / report.elapsed_seconds;
    report.messages_per_second =
        static_cast<double>(report.messages_read) / report.elapsed_seconds;
  }
  if (report.lidar_frames > 0) {
    report.avg_lidar_ms = lidar_filter_ms / static_cast<double>(report.lidar_frames);
    report.avg_decode_ms = lidar_decode_ms / static_cast<double>(report.lidar_frames);
  }

  std::size_t peak_blocks = 0;
  for (const auto& entry : MemoryPoolRegistry::allStats()) {
    peak_blocks += static_cast<std::size_t>(entry.second.peak_blocks_in_use);
  }
  report.peak_pool_blocks = peak_blocks;

  return report;
}

// ---------------------------------------------------------------------------
// 报告
// ---------------------------------------------------------------------------

std::string DataPipeline::Report::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);

  oss << "\n==================== 数据管道报告 ====================\n";
  oss << "输入文件    : " << input_path << "\n";
  oss.precision(2);
  oss << "数据量      : " << bytes_processed / (1024.0 * 1024.0) << " MB\n";
  oss << "总耗时      : " << elapsed_seconds << " s\n";
  oss << "吞吐        : " << throughput_mb_per_second << " MB/s"
      << "   (" << messages_per_second << " 消息/s)\n";

  oss << "\n-- 消息统计 --\n";
  oss << "读取消息    : " << messages_read << "\n";
  oss << "  点云帧    : " << lidar_frames << "\n";
  oss << "  GPS 帧    : " << gps_frames << "\n";
  oss << "  状态帧    : " << odometry_frames << "\n";
  oss << "解码失败    : " << decode_failures << "\n";

  oss << "\n-- 处理结果 --\n";
  oss << "对齐帧数    : " << aligned_frames << "\n";
  oss << "GPS 剔除跳点: " << gps_rejected << "\n";
  oss << "点云输入点数: " << lidar_points_in << "\n";
  oss << "点云输出点数: " << lidar_points_out << "\n";
  oss.precision(3);
  oss << "点云平均解码: " << avg_decode_ms << " ms/帧\n";
  oss << "点云平均降噪: " << avg_lidar_ms << " ms/帧\n";
  oss << "内存池峰值  : " << peak_pool_blocks << " 块\n";

  if (!output_path.empty()) {
    oss << "\n输出文件    : " << output_path << "\n";
    oss << "输出消息数  : " << output_messages << "\n";
  }
  oss << "======================================================\n";
  return oss.str();
}

}  // namespace adsim
