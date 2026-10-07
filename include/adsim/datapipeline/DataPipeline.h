// =============================================================================
//  DataPipeline.h — 路测数据处理管道
//
//  将"解包 → 解码 → 时间对齐 → 降噪 → 落盘"串成一条流水线，并让各阶段重叠执行。
//
//  线程模型：
//
//     [读取线程] --SpscRingBuffer<WorkItem>--> [派发线程] --enqueue--> [线程池 worker]
//         解包+拷贝入内存池                     背压感知派发            解码 + 降噪
//
//  三处基础设施各司其职，不是为用而用：
//    * MemoryPool    —— 消息载荷按 Chunk 复用的缓冲区无法跨线程持有，
//                       必须拷贝；逐条 malloc 会成为瓶颈，故走内存池。
//    * SpscRingBuffer—— 读取与消费速度不匹配时提供有界缓冲；
//                       满时读取线程直接等待，天然形成背压，避免内存失控。
//    * ThreadPool    —— 点云降噪是计算密集环节，多核并行是主要加速来源。
// =============================================================================
#pragma once

#include "adsim/common/MemoryPool.h"
#include "adsim/common/RingBuffer.h"
#include "adsim/common/ThreadPool.h"
#include "adsim/datapipeline/GpsFilter.h"
#include "adsim/datapipeline/PointCloudFilter.h"
#include "adsim/datapipeline/RosBagFormat.h"
#include "adsim/datapipeline/RosBagReader.h"
#include "adsim/datapipeline/TimeAligner.h"

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 消息类别，决定解码与后续处理方式
enum class MessageKind { kLidar, kGps, kOdometry, kUnknown };

const char* toString(MessageKind kind);

class DataPipeline {
 public:
  struct Config {
    /// 工作线程数；0 表示使用硬件并发度
    std::size_t thread_count{0};
    /// 流水线缓冲槽位数，决定背压阈值
    std::size_t queue_capacity{256};
    /// 是否对点云执行降噪
    bool enable_lidar_filter{true};
    /// 是否对 GPS 执行卡尔曼滤波
    bool enable_gps_filter{true};
    /// 是否保留点云（关闭时仅统计，内存占用极低）
    bool keep_lidar{true};
    /// 是否输出性能剖析报告
    bool enable_profiling{true};

    PointCloudFilter::Config lidar_filter;
    GpsFilter::Config gps_filter;
    TimeAligner::Config time_aligner;
  };

  struct Report {
    std::string input_path;

    std::size_t messages_read{0};
    std::size_t lidar_frames{0};
    std::size_t gps_frames{0};
    std::size_t odometry_frames{0};
    std::size_t decode_failures{0};

    std::size_t aligned_frames{0};
    std::size_t gps_rejected{0};
    std::size_t lidar_points_in{0};
    std::size_t lidar_points_out{0};

    std::size_t output_messages{0};
    std::string output_path;

    double elapsed_seconds{0.0};
    double bytes_processed{0};
    double throughput_mb_per_second{0.0};
    double messages_per_second{0.0};
    double avg_lidar_ms{0.0};
    double avg_decode_ms{0.0};

    std::size_t peak_pool_blocks{0};

    /// 对齐全过程的详细统计（匹配率、时间偏差分布）
    TimeAligner::Report alignment;

    std::string toString() const;
  };

  DataPipeline();
  explicit DataPipeline(const Config& config);

  /// 执行完整管道。
  /// @param bag_path        输入 bag 路径
  /// @param output_bag_path 输出 bag 路径；为空则不落盘
  Report run(const std::string& bag_path, const std::string& output_bag_path = "");

  /// 仅读取元信息，不做任何解码——探查超大文件时用
  static BagInfo probe(const std::string& bag_path);

  const Config& config() const { return config_; }

 private:
  Config config_;
};

}  // namespace adsim
