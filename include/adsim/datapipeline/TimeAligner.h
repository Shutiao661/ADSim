// =============================================================================
//  TimeAligner.h — 多传感器时间对齐
//
//  实车各传感器独立打时间戳（Lidar 10~20Hz、GPS 10Hz、底盘 CAN 50~100Hz），
//  硬件触发延迟与传输抖动使它们无法直接按序号配对，必须先做时间对齐。
//
//  提供两套接口：
//    * TimeAligner          — 离线批量对齐，用于数据清洗阶段
//    * StreamingTimeAligner — 流式对齐，供在线管道边解析边对齐
//
//  连续量（位姿、速度、经纬度）采用线性插值；位姿中的航向角按最短路径插值，
//  避免 ±π 附近出现跳变。离散量（点云）只能取最近邻，不插值。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace adsim {

/// 一帧对齐结果。以参考流（通常是激光雷达）的时刻为基准。
struct AlignedFrame {
  Timestamp stamp{kInvalidTimestamp};

  bool has_gps{false};
  bool has_odom{false};
  bool has_lidar{false};

  GpsFrame gps;
  VehicleState odom;                 ///< 可能来自插值
  LidarFrame lidar;

  bool odom_interpolated{false};
  Timestamp gps_offset{0};           ///< GPS 样本时刻与参考时刻之差
  Timestamp odom_offset{0};          ///< 车辆状态样本时刻与参考时刻之差
};

class TimeAligner {
 public:
  struct Config {
    /// 配对容差：样本与参考时刻相差超过该值即视为无匹配
    Timestamp match_tolerance{50 * 1000000LL};  ///< 50 ms
    /// 是否对连续量做线性插值（false 则退化为最近邻）
    bool interpolate{true};
  };

  TimeAligner();
  explicit TimeAligner(const Config& config);

  /// 在有序时间戳序列中查找最接近 query 的样本。
  /// @return 是否在容差范围内找到
  static bool nearestIndex(const std::vector<Timestamp>& stamps,
                           Timestamp query,
                           Timestamp tolerance,
                           std::size_t& index);

  /// 定位 query 落在的相邻区间 [lo, hi]，满足 stamps[lo] <= query <= stamps[hi]
  static bool bracket(const std::vector<Timestamp>& stamps,
                      Timestamp query,
                      std::size_t& lo,
                      std::size_t& hi);

  /// 线性插值车辆状态。位置与速度按时间比例插值，
  /// 航向角按最短路径插值（跨越 ±π 时不绕远路）。
  static bool interpolate(const std::vector<VehicleState>& states,
                          Timestamp query,
                          VehicleState& out);

  /// 线性插值 GPS 定位
  static bool interpolate(const std::vector<GpsFrame>& frames,
                          Timestamp query,
                          GpsFrame& out);

  /// 批量对齐。以点云流为参考节拍；
  /// 点云为空时退化为以 GPS 与车辆状态时间戳的并集为参考。
  std::vector<AlignedFrame> align(const std::vector<GpsFrame>& gps,
                                  const std::vector<VehicleState>& odom,
                                  const std::vector<LidarFrame>& lidar) const;

  struct Report {
    std::size_t total_frames{0};
    std::size_t gps_matched{0};
    std::size_t odom_matched{0};
    std::size_t interpolated{0};
    double mean_gps_offset_ms{0.0};
    double max_gps_offset_ms{0.0};
    double mean_odom_offset_ms{0.0};
    double max_odom_offset_ms{0.0};
    double gps_match_rate{0.0};
    double odom_match_rate{0.0};

    std::string toString() const;
  };

  /// 统计对齐质量，用于评估录制数据的时间同步状况
  static Report analyze(const std::vector<AlignedFrame>& frames);

  const Config& config() const { return config_; }

 private:
  Config config_;
};

// ---------------------------------------------------------------------------
//  流式对齐器
//
//  解析线程调用 push* 投递样本，消费线程调用 drain 取出已可对齐的帧。
//  内部仅保留容差窗口内的样本，内存占用与录制时长无关。
// ---------------------------------------------------------------------------
class StreamingTimeAligner {
 public:
  explicit StreamingTimeAligner(Timestamp tolerance_ns);

  void pushGps(const GpsFrame& frame);
  void pushVehicleState(const VehicleState& state);
  /// 点云作为参考节拍：每收到一帧点云即尝试产出一帧对齐结果
  void pushLidar(const LidarFrame& frame);

  /// 取出当前所有已就绪的对齐帧
  std::vector<AlignedFrame> drain();

  struct Stats {
    std::size_t gps_received{0};
    std::size_t odom_received{0};
    std::size_t lidar_received{0};
    std::size_t frames_emitted{0};
    std::size_t gps_matched{0};
    std::size_t odom_matched{0};
    std::size_t interpolated{0};
    std::size_t buffer_high_water{0};
  };

  Stats stats() const;
  void clear();

  Timestamp tolerance() const { return tolerance_; }

 private:
  void evictOldLocked(Timestamp newest);

  const Timestamp tolerance_;
  mutable std::mutex mutex_;

  std::deque<GpsFrame> gps_;
  std::deque<VehicleState> odom_;
  std::vector<AlignedFrame> ready_;
  Stats stats_;
};

}  // namespace adsim
