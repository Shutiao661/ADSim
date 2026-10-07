#include "adsim/planning/PlanningPolicy.h"

#include <algorithm>
#include <cmath>

namespace adsim {

namespace {

/// 前视点搜索的最小间距，避免路径点极密时反复采样
constexpr double kMinLookaheadStep = 0.5;

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

PlanningPolicy::PlanningPolicy() : PlanningPolicy(Config()) {}

PlanningPolicy::PlanningPolicy(const Config& config)
    : config_(config),
      decision_maker_(config.decision),
      path_planner_(config.path),
      speed_planner_(config.speed) {
  // 让路径规划器的优化开关跟随策略配置
  path_planner_.config().enable_optimization = config_.path.enable_optimization;
}

std::string PlanningPolicy::name() const {
  std::string name = "planning";
  name += config_.enable_decision ? "+fsm+bt" : "+no-decision";
  name += config_.enable_path_optimization ? "+opt" : "+raw";
  return name;
}

void PlanningPolicy::reset() {
  decision_maker_.reset();
  last_perception_ = PerceptionResult();
  last_decision_ = Decision();
  last_path_result_ = PathPlanningResult();
  last_trajectory_.clear();
  last_path_.clear();
  ego_lane_id_ = -1;
  cycle_count_ = 0;
}

// ---------------------------------------------------------------------------
// 控制
// ---------------------------------------------------------------------------

Vec2 PlanningPolicy::findLookaheadPoint(const VehicleState& ego,
                                        const std::vector<Vec2>& path) const {
  if (path.empty()) return ego.pose().position();

  const Vec2 ego_position = ego.pose().position();
  const double lookahead =
      config_.min_lookahead + config_.lookahead_gain * std::max(ego.speed, 0.0);

  // 从自车位置沿路径向前找第一个距离不小于前视距离的点
  Vec2 previous = ego_position;
  for (const Vec2& point : path) {
    const double distance = (point - ego_position).norm();
    if (distance >= lookahead) {
      return point;
    }
    if ((point - previous).norm() < kMinLookaheadStep && distance < lookahead * 0.5) {
      continue;  // 路径点过密且仍很近，跳过以降低计算量
    }
    previous = point;
  }

  // 路径不足前视距离时取末端点——这会自然产生"提前转向"的效果，
  // 比停在原地等待路径延长更安全
  return path.back();
}

double PlanningPolicy::purePursuitSteering(const VehicleState& ego,
                                           const std::vector<Vec2>& path) const {
  if (path.size() < 2) return 0.0;

  const Vec2 ego_position = ego.pose().position();
  const Vec2 target = findLookaheadPoint(ego, path);

  const Vec2 to_target = target - ego_position;
  double lookahead = to_target.norm();
  if (lookahead < 1e-3) {
    return 0.0;  // 目标点与自车重合，无从计算转向
  }

  // 纯跟踪公式：δ = atan(2·L·sin(α) / L_d)
  // α 为目标点方向与本车航向的夹角，L 为轴距
  const double alpha = normalizeAngle(to_target.heading() - ego.theta);
  const double steering =
      std::atan2(2.0 * config_.wheelbase * std::sin(alpha), lookahead);

  return clamp(steering, -config_.max_steering, config_.max_steering);
}

// ---------------------------------------------------------------------------
// 主循环
// ---------------------------------------------------------------------------

IPolicy::Command PlanningPolicy::computeCommand(const VehicleState& ego, const World& world,
                                                double dt) {
  Command command;

  // ---- 1. 定位自车所在车道 ----
  const LaneProjection projection = world.project(ego.pose().position());
  if (projection.valid) {
    ego_lane_id_ = projection.lane_id;
  }

  // ---- 2. 感知 ----
  last_perception_ = DecisionMaker::perceive(ego, world, ego_lane_id_,
                                             config_.desired_speed,
                                             config_.decision.ego_length);

  // ---- 3. 决策 ----
  if (config_.enable_decision) {
    last_decision_ = decision_maker_.decide(last_perception_, world);
  } else {
    // 对照模式：不做决策，直接按期望速度沿当前车道行驶
    last_decision_ = Decision();
    last_decision_.maneuver = Maneuver::kLaneKeep;
    last_decision_.target_speed = config_.desired_speed;
    last_decision_.target_lane_id = -1;
    last_decision_.rationale = "决策模块已禁用";
    last_decision_.confidence = 1.0;
  }

  // ---- 4. 路径规划 ----
  PathPlanningRequest request;
  request.ego = ego;
  request.world = &world;
  request.ego_lane_id = ego_lane_id_;
  request.target_lane_id = last_decision_.target_lane_id;
  request.target_speed = last_decision_.target_speed;
  request.lateral_offset = last_decision_.target_lateral_offset;
  request.horizon = 60.0;
  request.sample_step = config_.path.sample_step;

  // 紧急制动时不做换道，保持在当前车道内减速——
  // 紧急工况下大幅转向的风险高于追尾
  if (last_decision_.emergency) {
    request.target_lane_id = -1;
  }

  last_path_result_ = path_planner_.plan(request);

  if (last_path_result_.success && last_path_result_.path.size() >= 2) {
    last_path_ = last_path_result_.path;
  } else if (last_path_.size() >= 2) {
    // 本周期规划失败时沿用上周期路径，避免控制量突变为零
    // （"规划失败就急打方向"比"短暂沿用旧路径"危险得多）
  } else {
    last_path_.clear();
  }

  // ---- 5. 速度规划 ----
  if (last_path_.size() >= 2) {
    const Trajectory planned = speed_planner_.plan(last_path_, last_decision_.target_speed,
                                                   ego.speed, dt);
    if (!planned.empty()) {
      last_trajectory_ = planned;
    }
  }

  // ---- 6. 控制 ----
  double steering = 0.0;
  if (last_path_.size() >= 2) {
    steering = purePursuitSteering(ego, last_path_) * config_.steering_gain;
    steering = clamp(steering, -config_.max_steering, config_.max_steering);
  }

  // ---- 纵向 ----
  double acceleration = last_decision_.target_acceleration;

  if (last_decision_.emergency) {
    acceleration = std::min(acceleration, -6.0);
  } else {
    acceleration = clamp(acceleration, -config_.speed.max_deceleration,
                         config_.speed.max_acceleration);
  }

  // ---- 零速保护 ----
  //
  // 本策略不产生倒车意图。若不显式阻止，停车点附近的持续负加速度会把车推入
  // 倒车（车辆模型允许 -2 m/s 以满足挪车工况），随后车辆会带着残余转角
  // 倒退绕圈——这正是实测中"驶出路面"的真正来源。
  const double predicted_speed = ego.speed + acceleration * dt;
  if (predicted_speed < 0.0) {
    acceleration = -ego.speed / std::max(dt, 1e-6);  // 恰好减到零，而非减过头
  }

  // 已停稳且期望停车时不再输出纵向指令，避免残余加速度导致车辆蠕动
  if (ego.speed <= 1e-3 && last_decision_.target_speed <= 1e-3) {
    acceleration = std::max(acceleration, 0.0);
  }

  // 关于加加速度（jerk）限制：
  //
  // 这里**刻意不做**指令级的 jerk 限幅，而是交由车辆模型统一处理
  // （见 VehicleParams::max_jerk 与 VehicleModel::step）。原因是职责划分：
  // 策略只负责"期望多大的加速度"，执行器响应特性属于车辆模型。
  //
  // 早期版本在两层各加了一次限幅，且策略层的阈值（2.5 m/s³）比模型层
  // （5.0 m/s³）更严。结果是系统的实际 jerk 上限由策略层决定，从加速
  // 切换到制动需要 2.6 秒才能完成——实测中车辆因此完全无法响应前车急刹，
  // 一路加速到最高速冲了过去。冗余限制比没有限制更危险，因为它不会被
  // 任何单层的行为暴露出来。

  command.acceleration = acceleration;
  command.steering = steering;
  command.valid = true;

  ++cycle_count_;
  return command;
}

}  // namespace adsim
