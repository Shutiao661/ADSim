// =============================================================================
//  VehicleModel.cpp — 运动学自行车模型实现
//
//  参考点选在后轴中心：后轴中心的速度方向始终与车身纵轴一致，因此位姿积分
//  只需要"曲率 + 位移"两个量，不必反解前后轮的几何关系。这也是规划模块里
//  路径曲率可以直接映射为前轮转角的原因（κ = tanδ / L）。
//
//  积分方式为圆弧精确解而非欧拉法：定步长欧拉在 κ·ds 较大时会引入 O(κ·ds²)
//  的系统性位置漂移，做长距离回归时误差会累积到米级，无法用于轨迹比对。
// =============================================================================
#include "adsim/sim/VehicleModel.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace adsim {

namespace {

/// 曲率绝对值小于该阈值时按直线处理。
/// 取 1e-8 是因为 1/κ 放在 double 里仍完全可表示（1e8），
/// 而 sin(κds)/κ 与 ds 的相对误差已小于机器精度。
constexpr double kStraightCurvatureEps = 1e-8;

/// "直行"判定阈值：用于 currentTurningRadius 这类对外接口，
/// 保证返回值不会出现 1e6 量级的"半大不小"的半径。
constexpr double kStraightKappaEps = 1e-6;

/// 直行时返回的极大转弯半径：用有限大数而不是 inf，
/// 避免下游 inf*0、inf-inf 产生 NaN。
constexpr double kInfiniteRadius = 1e9;

/// 约束边界比较容差：恰好取到边界的状态必须判为可行，
/// 否则控制器输出的饱和值会被误判为越界。
constexpr double kFeasibilityEps = 1e-9;

/// 舒适性侧向加速度上限（约 0.4g）：超过该值乘员明显不适，
/// 也是轮胎进入非线性区的经验边界。
constexpr double kComfortLateralAcceleration = 4.0;

}  // namespace

// ---------------------------------------------------------------------------
// 构造与参数
// ---------------------------------------------------------------------------

VehicleModel::VehicleModel() = default;

VehicleModel::VehicleModel(const VehicleParams& params) : params_(params) {}

// ---------------------------------------------------------------------------
// 运动学关系
// ---------------------------------------------------------------------------

double VehicleModel::curvatureFromSteering(double steering) const {
  if (params_.wheelbase < kEpsilon) return 0.0;  // 参数非法时退化为直行，避免除零
  return std::tan(steering) / params_.wheelbase;
}

double VehicleModel::steeringFromCurvature(double curvature) const {
  return std::atan(curvature * params_.wheelbase);
}

double VehicleModel::minTurningRadius() const {
  const double kappa = std::fabs(curvatureFromSteering(params_.max_steering));
  if (kappa < kStraightCurvatureEps) return kInfiniteRadius;
  return 1.0 / kappa;
}

Pose2 VehicleModel::advanceOnArc(const Pose2& pose, double curvature, double ds) const {
  if (std::fabs(curvature) < kStraightCurvatureEps) {
    // 直线退化：沿当前航向平移，等价于下面公式在 κ→0 时的极限
    return Pose2(pose.x + ds * std::cos(pose.theta),
                 pose.y + ds * std::sin(pose.theta), normalizeAngle(pose.theta));
  }

  const double theta_next = pose.theta + curvature * ds;
  // 圆弧精确解：把 dθ=κ·ds 的旋转写成解析积分，误差只有浮点舍入量级。
  //   x' = x + (sin(θ+dθ) - sinθ)/κ
  //   y' = y - (cos(θ+dθ) - cosθ)/κ
  const double x = pose.x + (std::sin(theta_next) - std::sin(pose.theta)) / curvature;
  const double y = pose.y - (std::cos(theta_next) - std::cos(pose.theta)) / curvature;
  return Pose2(x, y, normalizeAngle(theta_next));
}

// ---------------------------------------------------------------------------
// 仿真推进
// ---------------------------------------------------------------------------

VehicleState VehicleModel::step(const VehicleState& state, double throttle, double brake,
                                double steering, double dt) const {
  VehicleState next = state;
  if (!(dt > 0.0) || !std::isfinite(dt)) return next;  // 非法步长不推进，避免除零

  // ---- 1. 纵向：踏板 → 目标加速度 ----
  const double t = clamp(throttle, 0.0, 1.0);
  const double b = clamp(brake, 0.0, 1.0);
  // 油门与制动按线性叠加（简化建模）：max_deceleration 为负值，
  // 因此 b 越大目标加速度越负。同时踩下时净效果仍由物理上限裁剪。
  const double a_cmd =
      clamp(t * params_.max_acceleration + b * params_.max_deceleration,
            params_.max_deceleration, params_.max_acceleration);

  // 加加速度限制：加速度本身按 max_jerk 一阶限幅。
  // 这一步是"乘员舒适性"与"执行器响应"的共同约束，
  // 缺了它，策略给出的阶跃加速度会让仿真结果远比实车乐观。
  const double max_delta_a = params_.max_jerk * dt;
  double a = clamp(a_cmd, state.acceleration - max_delta_a, state.acceleration + max_delta_a);
  // 再用物理上限兜底：若传入状态的加速度本身越界，优先保证输出可行
  a = clamp(a, params_.max_deceleration, params_.max_acceleration);

  const double v = clamp(state.speed + a * dt, params_.min_speed, params_.max_speed);
  // 速度被裁剪时实际加速度随之变小：用 (v' - v)/dt 反算，
  // 保证 speed 与 acceleration 自洽（否则下游 jerk 计算会出现虚假尖峰）。
  // 对可行输入该反算值必定仍在加速度区间内。
  const double a_achieved = (v - state.speed) / dt;
  next.speed = v;
  next.acceleration = clamp(a_achieved, params_.max_deceleration, params_.max_acceleration);

  // ---- 2. 横向：转角向目标值靠拢（执行器速率限制）----
  //
  // 建模取舍：VehicleState 只有一个 steering 字段，没有独立的"目标转角"通道。
  // 这里约定 state.steering 是**本步开始时的实际转角**，参数 steering 是
  // **目标转角**，每个步长最多变化 max_steering_rate·dt，即把前轮执行器
  // 建模为一阶速率受限环节。由此带来的结果是：第一拍的可用转角取决于调用方
  // 给出的初始 state.steering（默认 0，相当于方向盘回正起步），这与实车从
  // 中位开始打方向的物理过程一致。要消除该依赖，调用方应在初始状态里显式
  // 给出转向初值，而不是期望模型去"猜"。
  const double steering_target =
      clamp(steering, -params_.max_steering, params_.max_steering);
  const double steering_current =
      clamp(state.steering, -params_.max_steering, params_.max_steering);
  const double max_delta_s = params_.max_steering_rate * dt;
  next.steering = clamp(
      steering_current + clamp(steering_target - steering_current, -max_delta_s, max_delta_s),
      -params_.max_steering, params_.max_steering);

  // ---- 3. 位姿：沿曲率 κ 的圆弧精确积分 ----
  const double kappa = curvatureFromSteering(next.steering);
  const double ds = next.speed * dt;
  const Pose2 pose = advanceOnArc(state.pose(), kappa, ds);
  next.x = pose.x;
  next.y = pose.y;
  next.theta = pose.theta;

  // 横摆角速度：运动学模型下 θ̇ = v·κ（后轴参考点无侧偏）
  next.yaw_rate = next.speed * kappa;
  if (next.speed < 0.0) next.gear = -1;
  else if (next.speed > 0.0) next.gear = 1;
  else next.gear = 0;

  return next;
}

VehicleState VehicleModel::stepByAcceleration(const VehicleState& state, double acceleration,
                                              double steering, double dt) const {
  // 由目标加速度反推踏板归一化量；模型内部会再做一次裁剪，
  // 因此超出车辆能力的请求会被自动饱和，不会产生不可行状态。
  double throttle = 0.0;
  double brake = 0.0;
  if (acceleration >= 0.0) {
    throttle = (params_.max_acceleration > kEpsilon)
                   ? clamp(acceleration / params_.max_acceleration, 0.0, 1.0)
                   : 0.0;
  } else {
    brake = (params_.max_deceleration < -kEpsilon)
                ? clamp(acceleration / params_.max_deceleration, 0.0, 1.0)
                : 0.0;
  }
  return step(state, throttle, brake, steering, dt);
}

VehicleState VehicleModel::stepAlongPath(const VehicleState& state, double curvature,
                                         double speed, double dt) const {
  if (!(dt > 0.0) || !std::isfinite(dt)) return state;
  const double steering = steeringFromCurvature(curvature);
  // 速度跟踪用一阶（P）控制律，时间常数 0.5s：加速度指令随速度误差线性衰减，
  // 接近目标速度时平滑趋零，实际可达值再由 step 的加速度/jerk 约束裁剪。
  // 注意不能用"一拍到位"（a = Δv/dt）：那样加速度会在目标速度两侧 bang-bang
  // 切换，受 jerk 限制后又来不及切换回来，形成 ±(max_jerk·dt) 的极限环，
  // 回放出来的轨迹会在目标速度附近持续抖动，不是一条平滑轨迹。
  constexpr double kSpeedTrackingGain = 2.0;  // 1/s，即 τ = 0.5s
  const double a = kSpeedTrackingGain * (speed - state.speed);
  return stepByAcceleration(state, a, steering, dt);
}

// ---------------------------------------------------------------------------
// 几何与安全检查
// ---------------------------------------------------------------------------

Obb2 VehicleModel::boundingBox(const VehicleState& state) const {
  return Obb2(state.pose(), params_.length(), params_.width);
}

double VehicleModel::currentTurningRadius(const VehicleState& state) const {
  const double kappa = curvatureFromSteering(state.steering);
  if (!std::isfinite(kappa) || std::fabs(kappa) < kStraightKappaEps) return kInfiniteRadius;
  const double radius = 1.0 / std::fabs(kappa);
  return std::isfinite(radius) ? radius : kInfiniteRadius;
}

bool VehicleModel::isStateFeasible(const VehicleState& state) const {
  // 先查 NaN/Inf：越界比较对 NaN 恒为 false，必须先排除
  if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
      !std::isfinite(state.theta) || !std::isfinite(state.speed) ||
      !std::isfinite(state.acceleration) || !std::isfinite(state.steering)) {
    return false;
  }
  if (state.speed > params_.max_speed + kFeasibilityEps) return false;
  if (state.speed < params_.min_speed - kFeasibilityEps) return false;
  if (state.acceleration > params_.max_acceleration + kFeasibilityEps) return false;
  if (state.acceleration < params_.max_deceleration - kFeasibilityEps) return false;
  if (std::fabs(state.steering) > params_.max_steering + kFeasibilityEps) return false;
  return true;
}

double VehicleModel::maxCurvatureAtSpeed(double speed) const {
  // 运动学上限：前轮转角打到头
  const double kinematic = std::fabs(curvatureFromSteering(params_.max_steering));
  // 动力学（安全）上限：侧向加速度 a_lat = v²·κ 不超过舒适/附着边界。
  // 高速下后者才是真正的瓶颈，这也是"弯道减速"的物理来源。
  const double v = std::fabs(speed);
  if (v < 1e-3) return kinematic;
  const double dynamic = kComfortLateralAcceleration / (v * v);
  return std::min(kinematic, dynamic);
}

std::string VehicleModel::validate() const {
  std::ostringstream oss;
  if (!(params_.wheelbase > 0.0)) oss << "轴距必须为正; ";
  if (!(params_.width > 0.0)) oss << "车宽必须为正; ";
  if (params_.front_overhang < 0.0 || params_.rear_overhang < 0.0)
    oss << "前/后悬不能为负; ";
  if (!(params_.max_steering > 0.0) || params_.max_steering >= kPi * 0.5)
    oss << "最大前轮转角必须落在 (0, pi/2); ";
  if (!(params_.max_steering_rate > 0.0)) oss << "转角速率上限必须为正; ";
  if (!(params_.max_speed > 0.0)) oss << "最高车速必须为正; ";
  if (!(params_.min_speed < params_.max_speed)) oss << "最低车速必须小于最高车速; ";
  if (!(params_.max_acceleration > 0.0)) oss << "最大加速度必须为正; ";
  if (!(params_.max_deceleration < 0.0)) oss << "最大减速度必须为负; ";
  if (!(params_.max_jerk > 0.0)) oss << "最大加加速度必须为正; ";
  return oss.str();
}

}  // namespace adsim
