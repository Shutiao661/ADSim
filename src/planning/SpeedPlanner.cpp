#include "adsim/planning/SpeedPlanner.h"

#include "adsim/planning/optimizer/PathOptimizer.h"

#include <algorithm>
#include <cmath>

namespace adsim {

namespace {

/// 相邻点间距
std::vector<double> computeSpacings(const std::vector<Vec2>& path) {
  std::vector<double> spacings;
  spacings.reserve(path.size());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    spacings.push_back(std::max((path[i + 1] - path[i]).norm(), 1e-6));
  }
  if (!path.empty()) spacings.push_back(spacings.empty() ? 1e-6 : spacings.back());
  return spacings;
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

SpeedPlanner::SpeedPlanner() = default;
SpeedPlanner::SpeedPlanner(const Config& config) : config_(config) {}

// ---------------------------------------------------------------------------
// 静态约束计算
// ---------------------------------------------------------------------------

double SpeedPlanner::curvatureSpeedLimit(double curvature,
                                         double max_lateral_acceleration) {
  const double kappa = std::abs(curvature);

  // 直道上曲率趋零，速度上限趋于无穷——此时返回极大值而非 inf，
  // 避免后续求最小值时出现 inf * 0 之类的未定义结果
  if (kappa < 1e-9) {
    return 1e9;
  }
  return std::sqrt(std::max(max_lateral_acceleration, 0.0) / kappa);
}

double SpeedPlanner::followSpeed(double ego_speed, double lead_speed, double gap) const {
  if (gap > 200.0) return 1e9;  // 前车已远，不受跟车约束

  // IDM 中的期望间距项
  const double desired_gap =
      config_.min_gap + config_.time_headway * std::max(ego_speed, 0.0);

  const double safe_gap = std::max(gap, 0.1);
  const double ratio = desired_gap / safe_gap;

  // IDM 加速度公式：a = a_max · [1 − (v/v₀)⁴ − (s*/s)²]
  // 这里只关心它对速度的约束，因此把 v₀ 取当前允许的上限，
  // 反解出"使加速度不小于舒适减速度"的速度
  const double interaction = ratio * ratio;
  const double free_term = 1.0 - interaction;

  if (free_term <= 0.0) {
    // 间距已小于期望值，必须减速
    const double allowed = lead_speed + config_.comfortable_deceleration *
                                           (safe_gap - desired_gap) / std::max(ego_speed, 1.0);
    return std::max(0.0, allowed);
  }

  // 间距充裕，允许比前车快，但不超过一个合理倍数
  return lead_speed + std::sqrt(free_term) * std::max(ego_speed, 1.0);
}

// ---------------------------------------------------------------------------
// 可行性修正
// ---------------------------------------------------------------------------

void SpeedPlanner::enforceFeasibility(std::vector<double>& speeds,
                                      const std::vector<double>& spacings,
                                      const Config& config) {
  if (speeds.size() < 2 || speeds.size() != spacings.size()) return;

  const double acceleration = std::max(config.max_acceleration, 0.0);
  const double deceleration = std::max(config.max_deceleration, 0.0);

  // ---- 后向传播：保证在每个点之前都能减下来 ----
  // 必须先从后往前做，否则先做前向传播会把速度抬到"刹不住"的水平
  for (std::size_t i = speeds.size() - 1; i-- > 0;) {
    const double ds = spacings[i];
    const double limit = std::sqrt(speeds[i + 1] * speeds[i + 1] + 2.0 * deceleration * ds);
    speeds[i] = std::min(speeds[i], limit);
  }

  // ---- 前向传播：保证加速度不超限 ----
  for (std::size_t i = 0; i + 1 < speeds.size(); ++i) {
    const double ds = spacings[i];
    const double limit = std::sqrt(speeds[i] * speeds[i] + 2.0 * acceleration * ds);
    speeds[i + 1] = std::min(speeds[i + 1], limit);
  }

  // ---- jerk 限制：加速度的变化率不能过大 ----
  // 用相邻两点的速度差与距离反推加速度，再限制其变化幅度
  const double max_jerk = std::max(config.max_jerk, 0.0);
  if (max_jerk <= 0.0 || speeds.size() < 3) {
    for (double& v : speeds) v = std::max(v, 0.0);
    return;
  }

  double previous_acceleration = 0.0;
  for (std::size_t i = 1; i < speeds.size(); ++i) {
    const double ds = std::max(spacings[i - 1], 1e-6);
    const double v0 = speeds[i - 1];
    const double v1 = speeds[i];

    // 由 v₁² − v₀² = 2·a·ds 反解加速度
    double acc = (v1 * v1 - v0 * v0) / (2.0 * ds);

    // 加速度变化量受 jerk × 时间 约束；时间用平均速度估计
    const double mean_speed = std::max(0.5 * (v0 + v1), 0.1);
    const double dt = ds / mean_speed;
    const double max_delta = max_jerk * dt;

    if (acc > previous_acceleration + max_delta) {
      acc = previous_acceleration + max_delta;
      // 由受限后的加速度反算速度
      const double limited = std::sqrt(std::max(v0 * v0 + 2.0 * acc * ds, 0.0));
      speeds[i] = std::min(speeds[i], limited);
    } else if (acc < previous_acceleration - max_delta) {
      acc = previous_acceleration - max_delta;
      const double limited = std::sqrt(std::max(v0 * v0 + 2.0 * acc * ds, 0.0));
      speeds[i] = std::min(speeds[i], limited);
    }

    previous_acceleration = acc;
  }

  for (double& v : speeds) v = std::max(v, 0.0);
}

// ---------------------------------------------------------------------------
// 速度剖面生成
// ---------------------------------------------------------------------------

Trajectory SpeedPlanner::plan(const std::vector<Vec2>& path, double target_speed,
                              double current_speed, double dt) const {
  Trajectory trajectory;
  if (path.size() < 2) return trajectory;

  const std::size_t count = path.size();
  const std::vector<double> spacings = computeSpacings(path);
  const std::vector<double> curvatures = PathOptimizer::computeCurvatures(path);

  // ---- 1. 逐点速度上限 ----
  std::vector<double> speeds(count, std::max(target_speed, 0.0));
  for (std::size_t i = 0; i < count; ++i) {
    const double limit =
        curvatureSpeedLimit(curvatures[i], config_.max_lateral_acceleration);
    speeds[i] = std::min(speeds[i], limit);
  }

  // ---- 2. 首点速度与当前速度衔接 ----
  // 直接截断会产生速度阶跃，等价于无穷大加速度；
  // 这里改用"在一个控制周期内能达到的速度"作为首点上限
  const double reachable =
      std::max(current_speed, 0.0) +
      std::max(config_.max_acceleration, 0.0) * std::max(dt, 1e-3);
  speeds.front() = std::min(speeds.front(), reachable);

  // 首点也不应超过当前速度太多——否则第一步就会产生正向冲击
  if (current_speed > 0.0) {
    speeds.front() = std::min(speeds.front(), std::max(current_speed * 1.2, current_speed));
  }

  // ---- 3. 可行性修正 ----
  enforceFeasibility(speeds, spacings, config_);

  // ---- 4. 组装轨迹 ----
  trajectory.resize(count);

  double time = 0.0;

  for (std::size_t i = 0; i < count; ++i) {
    TrajectoryPoint& point = trajectory[i];
    point.x = path[i].x;
    point.y = path[i].y;
    point.kappa = curvatures[i];
    point.v = speeds[i];

    // 航向由相邻点方向确定
    if (i + 1 < count) {
      point.theta = (path[i + 1] - path[i]).heading();
    } else if (i > 0) {
      point.theta = (path[i] - path[i - 1]).heading();
    }

    point.t = time;

    // 用 v₁² − v₀² = 2·a·ds 反解加速度
    if (i > 0) {
      const double ds = spacings[i - 1];
      point.a = (speeds[i] * speeds[i] - speeds[i - 1] * speeds[i - 1]) / (2.0 * ds);
    }

    // 时间推进用平均速度，比用单点速度更接近真实
    if (i + 1 < count) {
      const double mean_speed = std::max(0.5 * (speeds[i] + speeds[i + 1]), 0.1);
      time += spacings[i] / mean_speed;
    }
  }

  return trajectory;
}

}  // namespace adsim
