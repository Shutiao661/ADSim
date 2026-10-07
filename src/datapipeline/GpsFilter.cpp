#include "adsim/datapipeline/GpsFilter.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace adsim {

namespace {

/// 每次拒绝观测时位置协方差的放大倍数
constexpr double kRejectionInflation = 2.0;

/// 对 2×2 对称矩阵求逆；行列式过小时返回 false
bool invert2x2(const double m[2][2], double out[2][2]) {
  const double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  if (std::abs(det) < 1e-12) {
    return false;
  }
  const double inv_det = 1.0 / det;
  out[0][0] = m[1][1] * inv_det;
  out[0][1] = -m[0][1] * inv_det;
  out[1][0] = -m[1][0] * inv_det;
  out[1][1] = m[0][0] * inv_det;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// GpsFilter
// ---------------------------------------------------------------------------

GpsFilter::GpsFilter() = default;
GpsFilter::GpsFilter(const Config& config) : config_(config) {}

void GpsFilter::reset() {
  initialized_ = false;
  last_stamp_ = kInvalidTimestamp;
  for (double& v : state_) v = 0.0;
  for (auto& row : covariance_) {
    for (double& v : row) v = 0.0;
  }
  innovation_sq_ema_ = -1.0;
  stats_ = Statistics();
}

void GpsFilter::predict(double dt) {
  // 状态转移：匀速模型
  state_[0] += state_[2] * dt;
  state_[1] += state_[3] * dt;

  // P = F P Fᵀ + Q
  double next[4][4]{};

  // F P Fᵀ，F 为 [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        // F[i][k]
        double f_ik = 0.0;
        if (i == k) f_ik = 1.0;
        if (i == 0 && k == 2) f_ik = dt;
        if (i == 1 && k == 3) f_ik = dt;

        for (int l = 0; l < 4; ++l) {
          // F[j][l]
          double f_jl = 0.0;
          if (j == l) f_jl = 1.0;
          if (j == 0 && l == 2) f_jl = dt;
          if (j == 1 && l == 3) f_jl = dt;

          sum += f_ik * covariance_[k][l] * f_jl;
        }
      }
      next[i][j] = sum;
    }
  }

  // 过程噪声：连续时间常速度模型的白噪声加速度积分形式
  const double q = config_.process_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  next[0][0] += q * dt4 / 4.0;
  next[1][1] += q * dt4 / 4.0;
  next[2][2] += q * dt2;
  next[3][3] += q * dt2;
  next[0][2] += q * dt3 / 2.0;
  next[2][0] += q * dt3 / 2.0;
  next[1][3] += q * dt3 / 2.0;
  next[3][1] += q * dt3 / 2.0;

  std::copy(&next[0][0], &next[0][0] + 16, &covariance_[0][0]);
}

void GpsFilter::correct(const Vec2& measurement, double measurement_variance) {
  // ---- 自适应观测噪声：由新息二阶矩直接反推 R ----
  double variance = measurement_variance;
  if (config_.enable_adaptive_noise && innovation_sq_ema_ > 0.0) {
    const double estimated =
        (innovation_sq_ema_ - covariance_[0][0] - covariance_[1][1]) * 0.5;
    const double ceiling = std::max(config_.max_measurement_variance, measurement_variance);
    variance = clamp(estimated, measurement_variance, ceiling);
  }

  // 观测矩阵 H = [[1,0,0,0],[0,1,0,0]]，故 HPHᵀ 即 P 的左上 2×2 块
  const double S[2][2] = {
      {covariance_[0][0] + variance, covariance_[0][1]},
      {covariance_[1][0], covariance_[1][1] + variance}};

  double S_inv[2][2];
  if (!invert2x2(S, S_inv)) {
    return;  // 协方差退化，跳过本次更新
  }

  // 新息 y = z - Hx
  const double y[2] = {measurement.x - state_[0], measurement.y - state_[1]};

  // ---- 离群点门控：新息的马氏距离平方服从 2 自由度卡方分布 ----
  const double mahalanobis =
      y[0] * (S_inv[0][0] * y[0] + S_inv[0][1] * y[1]) +
      y[1] * (S_inv[1][0] * y[0] + S_inv[1][1] * y[1]);

  if (config_.enable_outlier_rejection && mahalanobis > config_.outlier_gate) {
    ++stats_.rejected;
    stats_.max_rejected_jump =
        std::max(stats_.max_rejected_jump, std::sqrt(y[0] * y[0] + y[1] * y[1]));

    // 门控的前提是滤波器已处于稳态。连续拒绝说明预测与观测系统性不符，
    // 此时适度放大位置协方差，让门控重新打开——否则滤波器会"锁死"在
    // 错误的预测上，再也无法重新捕获真实轨迹。
    // 上限保证长时间异常后门控不会宽松到把真实跳点也放进来。
    if (config_.enable_rejection_recovery) {
      const double cap = config_.max_position_variance;
      covariance_[0][0] = std::min(covariance_[0][0] * kRejectionInflation, cap);
      covariance_[1][1] = std::min(covariance_[1][1] * kRejectionInflation, cap);
    }
    return;  // 仅保留预测值，不采信本次观测
  }

  // 卡尔曼增益 K = P Hᵀ S⁻¹，PHᵀ 即 P 的前两列
  double K[4][2];
  for (int i = 0; i < 4; ++i) {
    K[i][0] = covariance_[i][0] * S_inv[0][0] + covariance_[i][1] * S_inv[1][0];
    K[i][1] = covariance_[i][0] * S_inv[0][1] + covariance_[i][1] * S_inv[1][1];
  }

  for (int i = 0; i < 4; ++i) {
    state_[i] += K[i][0] * y[0] + K[i][1] * y[1];
  }

  // P = (I - K H) P
  double next[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      next[i][j] = covariance_[i][j] -
                   (K[i][0] * covariance_[0][j] + K[i][1] * covariance_[1][j]);
    }
  }
  std::copy(&next[0][0], &next[0][0] + 16, &covariance_[0][0]);

  // 更新噪声估计。只统计被接受的观测——离群点若计入会把 R 抬高，
  // 反而让门控对后续跳点越来越宽松，形成"越脏越松"的退化。
  const double y_squared = y[0] * y[0] + y[1] * y[1];
  if (innovation_sq_ema_ <= 0.0) {
    innovation_sq_ema_ = y_squared;
  } else {
    innovation_sq_ema_ += config_.adaptation_rate * (y_squared - innovation_sq_ema_);
  }

  ++stats_.accepted;
}

bool GpsFilter::update(const GpsFrame& frame, Vec2& position_out) {
  ++stats_.total;

  if (frame.fix_type >= 3) ++stats_.rtk_fixed;
  else if (frame.fix_type <= 1) ++stats_.single_point;

  if (!initialized_) {
    // 首帧：以该点建立局部 ENU 投影原点，并用观测初始化滤波器状态
    projector_ = GeoProjector(frame.latitude, frame.longitude, frame.altitude);

    state_[0] = 0.0;
    state_[1] = 0.0;
    state_[2] = 0.0;
    state_[3] = 0.0;

    for (auto& row : covariance_) {
      for (double& v : row) v = 0.0;
    }
    const double initial_variance = std::max(config_.measurement_noise, 1.0);
    covariance_[0][0] = initial_variance;
    covariance_[1][1] = initial_variance;
    // 初速度完全未知：给一个诚实的宽先验，让收敛期的暂态滞后能够体现在
    // 协方差里，从而被门控正确容忍，而不是被误判为离群点
    covariance_[2][2] = std::max(config_.initial_velocity_variance, 1.0);
    covariance_[3][3] = std::max(config_.initial_velocity_variance, 1.0);

    last_stamp_ = frame.stamp;
    initialized_ = true;
    ++stats_.accepted;

    position_out = Vec2{0.0, 0.0};
    return true;
  }

  // 投影原点已在首帧确定，此处直接用其换算观测值
  const Vec3 local = projector_.toLocal(frame.latitude, frame.longitude, frame.altitude);
  const Vec2 measurement{local.x, local.y};

  // 时间步长：异常值（重复时间戳、时间倒流）退化到 0.1s
  double dt = toSeconds(frame.stamp - last_stamp_);
  if (dt <= 1e-6 || dt > 1.0) {
    dt = 0.1;
  }
  last_stamp_ = frame.stamp;

  predict(dt);

  // 观测噪声：RTK 固定解精度远高于单点解
  double variance = config_.measurement_noise;
  if (config_.use_quality_weighting) {
    if (frame.fix_type >= 3) {
      variance *= 0.01;  // 厘米级
    } else if (frame.fix_type == 2) {
      variance *= 0.25;  // 分米级
    } else {
      variance *= 4.0;  // 单点解，米级
    }
    // hdop 越大精度越差，按平方关系放大方差
    if (frame.hdop > 0.0 && frame.hdop < 50.0) {
      variance *= std::max(frame.hdop * frame.hdop, 0.25);
    }
  }

  const std::size_t rejected_before = stats_.rejected;
  correct(measurement, variance);

  const bool accepted = stats_.rejected == rejected_before;
  position_out = Vec2{state_[0], state_[1]};
  return accepted;
}

std::vector<FilteredGpsSample> GpsFilter::filterAll(const std::vector<GpsFrame>& frames) {
  std::vector<FilteredGpsSample> result;
  result.reserve(frames.size());

  for (const GpsFrame& frame : frames) {
    FilteredGpsSample sample;
    sample.stamp = frame.stamp;
    sample.fix_type = frame.fix_type;
    sample.initialized = initialized_;

    // 记录滤波前的局部坐标，便于量化滤波增益。
    // 首帧之前投影原点尚未确定，此时 raw 无意义，留作原点 (0,0)。
    if (initialized_) {
      const Vec3 raw = projector_.toLocal(frame.latitude, frame.longitude, frame.altitude);
      sample.raw_position = Vec2{raw.x, raw.y};
    }

    sample.accepted = update(frame, sample.position);
    sample.velocity = velocity();
    result.push_back(sample);
  }

  return result;
}

std::string GpsFilter::Statistics::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(4);

  oss << "GPS 滤波统计:";
  oss.precision(0);
  oss << " 总计 " << total << " 帧";
  oss << " 接受 " << accepted << " 拒绝 " << rejected;
  oss.precision(1);
  oss << " 接受率 " << acceptanceRate() * 100.0 << "%";
  oss.precision(0);
  oss << " (RTK固定 " << rtk_fixed << " / 单点 " << single_point << ")";
  oss.precision(2);
  oss << " 最大剔除跳变 " << max_rejected_jump << " m";
  return oss.str();
}

// ---------------------------------------------------------------------------
// 对照组：滑动平均
// ---------------------------------------------------------------------------

std::vector<GpsFrame> GpsFilter::movingAverage(const std::vector<GpsFrame>& frames,
                                               std::size_t window) {
  if (frames.empty() || window == 0) return frames;

  std::vector<GpsFrame> out = frames;
  const std::size_t half = window / 2;

  for (std::size_t i = 0; i < frames.size(); ++i) {
    const std::size_t lo = i > half ? i - half : 0;
    const std::size_t hi = std::min(i + half, frames.size() - 1);

    double lat = 0.0, lon = 0.0, alt = 0.0;
    std::size_t count = 0;
    for (std::size_t j = lo; j <= hi; ++j) {
      lat += frames[j].latitude;
      lon += frames[j].longitude;
      alt += frames[j].altitude;
      ++count;
    }
    const double inv = 1.0 / static_cast<double>(count);
    out[i].latitude = lat * inv;
    out[i].longitude = lon * inv;
    out[i].altitude = alt * inv;
  }
  return out;
}

double GpsFilter::rootMeanSquareError(const std::vector<Vec2>& estimate,
                                      const std::vector<Vec2>& reference) {
  if (estimate.size() != reference.size() || estimate.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (std::size_t i = 0; i < estimate.size(); ++i) {
    const Vec2 d = estimate[i] - reference[i];
    sum += d.squaredNorm();
  }
  return std::sqrt(sum / static_cast<double>(estimate.size()));
}

// ---------------------------------------------------------------------------
// HeadingFilter
// ---------------------------------------------------------------------------

HeadingFilter::HeadingFilter(double alpha) : alpha_(clamp(alpha, 0.01, 1.0)) {}

double HeadingFilter::update(double heading) {
  const double normalized = normalizeAngle(heading);

  if (!initialized_) {
    unwrapped_ = normalized;
    initialized_ = true;
    count_ = 1;
    return normalized;
  }

  // 先把新观测展开到与累积角同一圈，再平滑——
  // 直接在 (-π, π] 上做平均会在跨越 ±π 时产生剧烈抖动
  const double delta = normalizeAngle(normalized - unwrapped_);
  unwrapped_ += alpha_ * delta;
  ++count_;

  return normalizeAngle(unwrapped_);
}

void HeadingFilter::reset() {
  unwrapped_ = 0.0;
  initialized_ = false;
  count_ = 0;
}

}  // namespace adsim
