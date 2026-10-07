#include "adsim/datapipeline/TimeAligner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace adsim {

namespace {

/// 按时间比例 t ∈ [0,1] 在两个值之间线性插值
double lerp(double a, double b, double t) { return a + (b - a) * t; }

/// 航向角插值：先归一化角差到 (-π, π]，再插值，避免绕远路
double lerpAngle(double a, double b, double t) {
  return normalizeAngle(a + normalizeAngle(b - a) * t);
}

template <typename Sample>
std::vector<Timestamp> collectStamps(const std::vector<Sample>& samples) {
  std::vector<Timestamp> stamps;
  stamps.reserve(samples.size());
  for (const Sample& s : samples) stamps.push_back(s.stamp);
  return stamps;
}

/// 在有序时间戳中定位最接近 query 的下标（二分查找）
std::size_t lowerBoundIndex(const std::vector<Timestamp>& stamps, Timestamp query) {
  return static_cast<std::size_t>(
      std::lower_bound(stamps.begin(), stamps.end(), query) - stamps.begin());
}

}  // namespace

// ---------------------------------------------------------------------------
// TimeAligner
// ---------------------------------------------------------------------------

TimeAligner::TimeAligner() = default;
TimeAligner::TimeAligner(const Config& config) : config_(config) {}

bool TimeAligner::nearestIndex(const std::vector<Timestamp>& stamps,
                               Timestamp query,
                               Timestamp tolerance,
                               std::size_t& index) {
  if (stamps.empty()) return false;

  const std::size_t pos = lowerBoundIndex(stamps, query);

  // 候选为 pos（第一个 >= query）与其前一个
  std::size_t best = stamps.size();
  Timestamp best_diff = std::numeric_limits<Timestamp>::max();

  if (pos < stamps.size()) {
    best = pos;
    best_diff = std::abs(stamps[pos] - query);
  }
  if (pos > 0) {
    const Timestamp diff = std::abs(stamps[pos - 1] - query);
    // 恰好等距时取更早的样本（<= 而非 <）：
    // 在线场景下"不多看未来的数据"比"少滞后一点"更重要，且让结果可确定复现
    if (diff <= best_diff) {
      best = pos - 1;
      best_diff = diff;
    }
  }

  if (best == stamps.size() || best_diff > tolerance) {
    return false;
  }
  index = best;
  return true;
}

bool TimeAligner::bracket(const std::vector<Timestamp>& stamps,
                          Timestamp query,
                          std::size_t& lo,
                          std::size_t& hi) {
  if (stamps.size() < 2) return false;

  const std::size_t pos = lowerBoundIndex(stamps, query);

  // query 落在序列之外时无法构成有效区间
  if (pos == 0 || pos >= stamps.size()) return false;

  lo = pos - 1;
  hi = pos;
  return true;
}

bool TimeAligner::interpolate(const std::vector<VehicleState>& states,
                              Timestamp query,
                              VehicleState& out) {
  if (states.empty()) return false;

  // 序列外的查询退化为最近邻——外推没有物理依据
  if (query <= states.front().stamp) {
    out = states.front();
    return true;
  }
  if (query >= states.back().stamp) {
    out = states.back();
    return true;
  }

  std::size_t lo = 0;
  std::size_t hi = 0;
  if (!bracket(collectStamps(states), query, lo, hi)) {
    return false;
  }

  const VehicleState& a = states[lo];
  const VehicleState& b = states[hi];
  const Timestamp span = b.stamp - a.stamp;
  if (span <= 0) {
    out = a;
    return true;
  }

  const double t = static_cast<double>(query - a.stamp) / static_cast<double>(span);

  out = a;
  out.stamp = query;
  out.x = lerp(a.x, b.x, t);
  out.y = lerp(a.y, b.y, t);
  out.theta = lerpAngle(a.theta, b.theta, t);
  out.speed = lerp(a.speed, b.speed, t);
  out.acceleration = lerp(a.acceleration, b.acceleration, t);
  out.steering = lerp(a.steering, b.steering, t);
  out.yaw_rate = lerp(a.yaw_rate, b.yaw_rate, t);
  out.throttle = lerp(a.throttle, b.throttle, t);
  out.brake = lerp(a.brake, b.brake, t);
  return true;
}

bool TimeAligner::interpolate(const std::vector<GpsFrame>& frames,
                              Timestamp query,
                              GpsFrame& out) {
  if (frames.empty()) return false;

  if (query <= frames.front().stamp) {
    out = frames.front();
    return true;
  }
  if (query >= frames.back().stamp) {
    out = frames.back();
    return true;
  }

  std::size_t lo = 0;
  std::size_t hi = 0;
  if (!bracket(collectStamps(frames), query, lo, hi)) {
    return false;
  }

  const GpsFrame& a = frames[lo];
  const GpsFrame& b = frames[hi];
  const Timestamp span = b.stamp - a.stamp;
  if (span <= 0) {
    out = a;
    return true;
  }

  const double t = static_cast<double>(query - a.stamp) / static_cast<double>(span);

  out = a;
  out.stamp = query;
  out.latitude = lerp(a.latitude, b.latitude, t);
  out.longitude = lerp(a.longitude, b.longitude, t);
  out.altitude = lerp(a.altitude, b.altitude, t);
  out.heading = lerpAngle(a.heading, b.heading, t);
  out.speed = lerp(a.speed, b.speed, t);
  return true;
}

std::vector<AlignedFrame> TimeAligner::align(const std::vector<GpsFrame>& gps,
                                             const std::vector<VehicleState>& odom,
                                             const std::vector<LidarFrame>& lidar) const {
  std::vector<AlignedFrame> result;

  const std::vector<Timestamp> gps_stamps = collectStamps(gps);
  const std::vector<Timestamp> odom_stamps = collectStamps(odom);

  // 参考节拍：优先点云，其次二者的时间戳并集
  std::vector<Timestamp> reference;
  if (!lidar.empty()) {
    reference = collectStamps(lidar);
  } else {
    reference = gps_stamps;
    reference.insert(reference.end(), odom_stamps.begin(), odom_stamps.end());
    std::sort(reference.begin(), reference.end());
    reference.erase(std::unique(reference.begin(), reference.end()), reference.end());
  }

  result.reserve(reference.size());

  for (std::size_t i = 0; i < reference.size(); ++i) {
    AlignedFrame frame;
    frame.stamp = reference[i];

    if (!lidar.empty()) {
      frame.lidar = lidar[i];
      frame.has_lidar = true;
    }

    // ---- GPS ----
    if (!gps.empty()) {
      if (config_.interpolate) {
        std::size_t lo = 0;
        std::size_t hi = 0;
        const bool inside = bracket(gps_stamps, frame.stamp, lo, hi);
        const bool within_tolerance =
            inside && (frame.stamp - gps_stamps[lo] <= config_.match_tolerance) &&
            (gps_stamps[hi] - frame.stamp <= config_.match_tolerance);
        if (within_tolerance) {
          interpolate(gps, frame.stamp, frame.gps);
          frame.has_gps = true;
          frame.gps_offset = 0;
        }
      }

      if (!frame.has_gps) {
        std::size_t index = 0;
        if (nearestIndex(gps_stamps, frame.stamp, config_.match_tolerance, index)) {
          frame.gps = gps[index];
          frame.has_gps = true;
          frame.gps_offset = gps_stamps[index] - frame.stamp;
        }
      }
    }

    // ---- 车辆状态 ----
    if (!odom.empty()) {
      if (config_.interpolate) {
        std::size_t lo = 0;
        std::size_t hi = 0;
        const bool inside = bracket(odom_stamps, frame.stamp, lo, hi);
        const bool within_tolerance =
            inside && (frame.stamp - odom_stamps[lo] <= config_.match_tolerance) &&
            (odom_stamps[hi] - frame.stamp <= config_.match_tolerance);
        if (within_tolerance) {
          interpolate(odom, frame.stamp, frame.odom);
          frame.has_odom = true;
          frame.odom_interpolated = true;
          frame.odom_offset = 0;
        }
      }

      if (!frame.has_odom) {
        std::size_t index = 0;
        if (nearestIndex(odom_stamps, frame.stamp, config_.match_tolerance, index)) {
          frame.odom = odom[index];
          frame.has_odom = true;
          frame.odom_interpolated = false;
          frame.odom_offset = odom_stamps[index] - frame.stamp;
        }
      }
    }

    result.push_back(std::move(frame));
  }

  return result;
}

std::string TimeAligner::Report::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);

  oss << "时间对齐报告:\n";
  oss << "  对齐帧数      : " << total_frames << "\n";
  oss.precision(1);
  oss << "  GPS 匹配率    : " << gps_match_rate * 100.0 << "%  (" << gps_matched << ")\n";
  oss << "  车辆状态匹配率: " << odom_match_rate * 100.0 << "%  (" << odom_matched << ")\n";
  oss << "  插值帧数      : " << interpolated << "\n";
  oss.precision(3);
  oss << "  GPS 时间偏差  : 均值 " << mean_gps_offset_ms << " ms / 最大 "
      << max_gps_offset_ms << " ms\n";
  oss << "  状态时间偏差  : 均值 " << mean_odom_offset_ms << " ms / 最大 "
      << max_odom_offset_ms << " ms\n";
  return oss.str();
}

TimeAligner::Report TimeAligner::analyze(const std::vector<AlignedFrame>& frames) {
  Report report;
  report.total_frames = frames.size();
  if (frames.empty()) return report;

  double gps_offset_sum = 0.0;
  double odom_offset_sum = 0.0;

  for (const AlignedFrame& frame : frames) {
    if (frame.has_gps) {
      ++report.gps_matched;
      const double offset_ms = std::abs(toSeconds(frame.gps_offset)) * 1000.0;
      gps_offset_sum += offset_ms;
      report.max_gps_offset_ms = std::max(report.max_gps_offset_ms, offset_ms);
    }
    if (frame.has_odom) {
      ++report.odom_matched;
      if (frame.odom_interpolated) ++report.interpolated;
      const double offset_ms = std::abs(toSeconds(frame.odom_offset)) * 1000.0;
      odom_offset_sum += offset_ms;
      report.max_odom_offset_ms = std::max(report.max_odom_offset_ms, offset_ms);
    }
  }

  const double total = static_cast<double>(report.total_frames);
  report.gps_match_rate = static_cast<double>(report.gps_matched) / total;
  report.odom_match_rate = static_cast<double>(report.odom_matched) / total;
  report.mean_gps_offset_ms =
      report.gps_matched == 0 ? 0.0 : gps_offset_sum / static_cast<double>(report.gps_matched);
  report.mean_odom_offset_ms =
      report.odom_matched == 0 ? 0.0 : odom_offset_sum / static_cast<double>(report.odom_matched);

  return report;
}

// ---------------------------------------------------------------------------
// StreamingTimeAligner
// ---------------------------------------------------------------------------

StreamingTimeAligner::StreamingTimeAligner(Timestamp tolerance_ns)
    : tolerance_(tolerance_ns > 0 ? tolerance_ns : 50 * 1000000LL) {}

void StreamingTimeAligner::pushGps(const GpsFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  gps_.push_back(frame);
  ++stats_.gps_received;
  evictOldLocked(frame.stamp);
}

void StreamingTimeAligner::pushVehicleState(const VehicleState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_.push_back(state);
  ++stats_.odom_received;
  evictOldLocked(state.stamp);
}

void StreamingTimeAligner::pushLidar(const LidarFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.lidar_received;

  // 点云作为参考节拍，立即尝试配对
  AlignedFrame aligned;
  aligned.stamp = frame.stamp;
  aligned.lidar = frame;
  aligned.has_lidar = true;

  // ---- GPS 配对：优先插值，失败则最近邻 ----
  if (!gps_.empty()) {
    std::vector<Timestamp> stamps;
    stamps.reserve(gps_.size());
    for (const GpsFrame& f : gps_) stamps.push_back(f.stamp);

    std::size_t lo = 0;
    std::size_t hi = 0;
    if (TimeAligner::bracket(stamps, frame.stamp, lo, hi) &&
        frame.stamp - stamps[lo] <= tolerance_ && stamps[hi] - frame.stamp <= tolerance_) {
      std::vector<GpsFrame> buffer(gps_.begin(), gps_.end());
      if (TimeAligner::interpolate(buffer, frame.stamp, aligned.gps)) {
        aligned.has_gps = true;
        aligned.gps_offset = 0;
        ++stats_.interpolated;
      }
    }

    if (!aligned.has_gps) {
      std::size_t index = 0;
      if (TimeAligner::nearestIndex(stamps, frame.stamp, tolerance_, index)) {
        aligned.gps = gps_[index];
        aligned.has_gps = true;
        aligned.gps_offset = stamps[index] - frame.stamp;
      }
    }
  }

  // ---- 车辆状态配对 ----
  if (!odom_.empty()) {
    std::vector<Timestamp> stamps;
    stamps.reserve(odom_.size());
    for (const VehicleState& s : odom_) stamps.push_back(s.stamp);

    std::size_t lo = 0;
    std::size_t hi = 0;
    if (TimeAligner::bracket(stamps, frame.stamp, lo, hi) &&
        frame.stamp - stamps[lo] <= tolerance_ && stamps[hi] - frame.stamp <= tolerance_) {
      std::vector<VehicleState> buffer(odom_.begin(), odom_.end());
      if (TimeAligner::interpolate(buffer, frame.stamp, aligned.odom)) {
        aligned.has_odom = true;
        aligned.odom_interpolated = true;
        aligned.odom_offset = 0;
      }
    }

    if (!aligned.has_odom) {
      std::size_t index = 0;
      if (TimeAligner::nearestIndex(stamps, frame.stamp, tolerance_, index)) {
        aligned.odom = odom_[index];
        aligned.has_odom = true;
        aligned.odom_interpolated = false;
        aligned.odom_offset = stamps[index] - frame.stamp;
      }
    }
  }

  if (aligned.has_gps) ++stats_.gps_matched;
  if (aligned.has_odom) ++stats_.odom_matched;
  ++stats_.frames_emitted;

  ready_.push_back(std::move(aligned));

  stats_.buffer_high_water = std::max(stats_.buffer_high_water, gps_.size() + odom_.size());
  evictOldLocked(frame.stamp);
}

void StreamingTimeAligner::evictOldLocked(Timestamp newest) {
  // 只保留容差窗口内的样本：更早的数据不会再被任何后续帧匹配到
  const Timestamp cutoff = newest - tolerance_ * 4;

  while (!gps_.empty() && gps_.front().stamp < cutoff) {
    gps_.pop_front();
  }
  while (!odom_.empty() && odom_.front().stamp < cutoff) {
    odom_.pop_front();
  }
}

std::vector<AlignedFrame> StreamingTimeAligner::drain() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<AlignedFrame> out;
  out.swap(ready_);
  return out;
}

StreamingTimeAligner::Stats StreamingTimeAligner::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void StreamingTimeAligner::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  gps_.clear();
  odom_.clear();
  ready_.clear();
  stats_ = Stats();
}

}  // namespace adsim
