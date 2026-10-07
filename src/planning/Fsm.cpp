#include "adsim/planning/Fsm.h"

#include <algorithm>
#include <sstream>

namespace adsim {

namespace {

constexpr std::size_t kMaxHistory = 64;

/// 触发紧急制动的 TTC 阈值。取 1.5s 是工程折中：
/// 低于该值时常规制动已难以避免碰撞，必须立即让位于安全逻辑。
constexpr double kEmergencyTtc = 1.5;

/// 判定"前车过慢"的相对速度阈值 (m/s)
constexpr double kSlowLeadThreshold = 1.0;

}  // namespace

const char* toString(DrivingState state) {
  switch (state) {
    case DrivingState::kLaneKeep: return "车道保持";
    case DrivingState::kFollow: return "跟车";
    case DrivingState::kLaneChangeLeft: return "向左换道";
    case DrivingState::kLaneChangeRight: return "向右换道";
    case DrivingState::kOvertake: return "超车";
    case DrivingState::kYield: return "让行";
    case DrivingState::kTurnLeft: return "左转";
    case DrivingState::kTurnRight: return "右转";
    case DrivingState::kStop: return "停车";
    case DrivingState::kEmergencyBrake: return "紧急制动";
  }
  return "未知";
}

const char* toString(DrivingEvent event) {
  switch (event) {
    case DrivingEvent::kNone: return "无";
    case DrivingEvent::kObstacleDetected: return "检测到障碍物";
    case DrivingEvent::kObstacleCleared: return "障碍物消失";
    case DrivingEvent::kLeadVehicleSlow: return "前车减速";
    case DrivingEvent::kLeadVehicleGone: return "前车驶离";
    case DrivingEvent::kLaneChangeRequested: return "请求换道";
    case DrivingEvent::kLaneChangeCompleted: return "换道完成";
    case DrivingEvent::kLaneChangeAborted: return "换道中止";
    case DrivingEvent::kOvertakeRequested: return "请求超车";
    case DrivingEvent::kOvertakeCompleted: return "超车完成";
    case DrivingEvent::kIntersectionAhead: return "前方路口";
    case DrivingEvent::kTurnRequested: return "请求转向";
    case DrivingEvent::kYieldRequired: return "需要让行";
    case DrivingEvent::kYieldCompleted: return "让行结束";
    case DrivingEvent::kEmergencyDetected: return "紧急情况";
    case DrivingEvent::kEmergencyCleared: return "紧急解除";
    case DrivingEvent::kStopRequested: return "请求停车";
    case DrivingEvent::kGoRequested: return "请求起步";
  }
  return "未知";
}

// ---------------------------------------------------------------------------
// DrivingFsm
// ---------------------------------------------------------------------------

DrivingFsm::DrivingFsm(DrivingState initial) : state_(initial), previous_(initial) {
  history_.reserve(kMaxHistory);
}

void DrivingFsm::addTransition(const Transition& transition) {
  transitions_.push_back(transition);
}

const Transition* DrivingFsm::findTransition(DrivingEvent event,
                                             const FsmContext& context) const {
  for (const Transition& transition : transitions_) {
    if (transition.from != state_ || transition.event != event) continue;
    // 守卫为空表示无条件转移
    if (transition.guard && !transition.guard(context)) continue;
    return &transition;
  }
  return nullptr;
}

bool DrivingFsm::handleEvent(DrivingEvent event, FsmContext& context) {
  if (event == DrivingEvent::kNone) return false;

  const Transition* transition = findTransition(event, context);
  if (transition == nullptr) {
    return false;  // 当前状态下该事件无对应转移，安全忽略
  }

  previous_ = state_;
  state_ = transition->to;

  if (transition->action) {
    transition->action(context);
  }

  context.last_reason = transition->description;
  ++transition_count_;
  time_in_state_ = 0.0;

  TransitionRecord record;
  record.time = context.time;
  record.from = previous_;
  record.to = state_;
  record.event = event;
  record.description = transition->description;

  history_.push_back(std::move(record));
  if (history_.size() > kMaxHistory) {
    history_.erase(history_.begin());
  }

  return true;
}

DrivingEvent DrivingFsm::deriveEvent(const FsmContext& context) const {
  // 按优先级从高到低判定：安全 > 避障 > 跟车 > 让行 > 路口 > 常规
  if (context.collision_imminent || context.min_ttc < kEmergencyTtc) {
    return DrivingEvent::kEmergencyDetected;
  }

  // 紧急状态的解除需要**显式**事件。
  //
  // 走到这里已确认当前无碰撞风险，若仍处于紧急制动状态，必须发出解除事件。
  // 早期版本漏掉了这一步：转移表里 kEmergencyBrake 只有 kEmergencyCleared
  // 一个出口，而该事件从不被派生，状态机就此闩死在紧急制动上再也不会恢复——
  // 实测中车辆会一直以 -8 m/s² 制动并最终停在路上。
  if (state_ == DrivingState::kEmergencyBrake) {
    return DrivingEvent::kEmergencyCleared;
  }

  if (context.obstacle_ahead) {
    return DrivingEvent::kObstacleDetected;
  }

  if (context.lead_detected &&
      context.lead_speed + kSlowLeadThreshold < context.desired_speed) {
    return DrivingEvent::kLeadVehicleSlow;
  }

  if (!context.lead_detected && state_ == DrivingState::kFollow) {
    return DrivingEvent::kLeadVehicleGone;
  }

  if (context.yield_required) {
    return DrivingEvent::kYieldRequired;
  }

  if (context.intersection_ahead && context.turn_left_requested) {
    return DrivingEvent::kTurnRequested;
  }

  if (!context.intersection_ahead && !context.yield_required &&
      !context.obstacle_ahead && (state_ != DrivingState::kLaneKeep) &&
      state_ != DrivingState::kStop) {
    // 前方已清空，回到车道保持
    return DrivingEvent::kObstacleCleared;
  }

  return DrivingEvent::kNone;
}

std::string DrivingFsm::update(FsmContext& context) {
  time_ = context.time;
  time_in_state_ += 0.0;  // 实际累加由调用方通过时间差驱动，见下方说明

  const DrivingState before = state_;
  const DrivingEvent event = deriveEvent(context);

  if (event != DrivingEvent::kNone && handleEvent(event, context)) {
    std::ostringstream oss;
    oss << toString(before) << " → " << toString(state_) << " (" << toString(event)
        << ": " << context.last_reason << ")";
    return oss.str();
  }
  return {};
}

void DrivingFsm::reset(DrivingState initial) {
  state_ = initial;
  previous_ = initial;
  time_ = 0.0;
  time_in_state_ = 0.0;
  transition_count_ = 0;
  history_.clear();
}

// ---------------------------------------------------------------------------
// 内置规则集
// ---------------------------------------------------------------------------

void DrivingFsm::addDefaultTransitions() {
  auto guard_always = [](const FsmContext&) { return true; };

  // ---- 紧急制动：最高优先级，任何状态都应能进入 ----
  for (DrivingState from :
       {DrivingState::kLaneKeep, DrivingState::kFollow, DrivingState::kLaneChangeLeft,
        DrivingState::kLaneChangeRight, DrivingState::kOvertake, DrivingState::kYield,
        DrivingState::kTurnLeft, DrivingState::kTurnRight}) {
    addTransition({from, DrivingEvent::kEmergencyDetected, DrivingState::kEmergencyBrake,
                   guard_always,
                   [](FsmContext& ctx) { ctx.speed_limit_override = 0.0; },
                   "检测到碰撞风险，立即接管为紧急制动"});
  }

  // 紧急解除后回到车道保持
  addTransition({DrivingState::kEmergencyBrake, DrivingEvent::kEmergencyCleared,
                 DrivingState::kLaneKeep, guard_always,
                 [](FsmContext& ctx) { ctx.speed_limit_override = -1.0; },
                 "风险解除，恢复车道保持"});

  // ---- 车道保持 ----
  addTransition({DrivingState::kLaneKeep, DrivingEvent::kObstacleDetected,
                 DrivingState::kFollow, guard_always, nullptr,
                 "前方有障碍物，转入跟车"});

  addTransition({DrivingState::kLaneKeep, DrivingEvent::kLeadVehicleSlow,
                 DrivingState::kFollow, guard_always, nullptr,
                 "前车速度低于期望，转入跟车"});

  addTransition({DrivingState::kLaneKeep, DrivingEvent::kYieldRequired,
                 DrivingState::kYield, guard_always, nullptr, "存在让行义务"});

  addTransition({DrivingState::kLaneKeep, DrivingEvent::kTurnRequested,
                 DrivingState::kTurnLeft, guard_always, nullptr, "按目标路径左转"});
  addTransition({DrivingState::kLaneKeep, DrivingEvent::kTurnRequested,
                 DrivingState::kTurnRight,
                 [](const FsmContext& ctx) { return !ctx.turn_left_requested; }, nullptr,
                 "按目标路径右转"});

  // ---- 跟车 ----
  // 换道请求只有在目标车道确实存在且空闲时才被接受——
  // 这正是"事件发生 ≠ 一定转移"的典型场景
  addTransition({DrivingState::kFollow, DrivingEvent::kObstacleDetected,
                 DrivingState::kLaneChangeLeft,
                 [](const FsmContext& ctx) {
                   return ctx.allow_lane_change && ctx.left_lane_exists &&
                          ctx.left_lane_clear;
                 },
                 nullptr, "左侧车道空闲，换道绕行"});

  addTransition({DrivingState::kFollow, DrivingEvent::kObstacleDetected,
                 DrivingState::kLaneChangeRight,
                 [](const FsmContext& ctx) {
                   return ctx.allow_lane_change && ctx.right_lane_exists &&
                          ctx.right_lane_clear;
                 },
                 nullptr, "右侧车道空闲，换道绕行"});

  // 两侧都不具备换道条件时只能继续跟车
  addTransition({DrivingState::kFollow, DrivingEvent::kObstacleDetected,
                 DrivingState::kFollow,
                 [](const FsmContext& ctx) {
                   return !(ctx.left_lane_exists && ctx.left_lane_clear) &&
                          !(ctx.right_lane_exists && ctx.right_lane_clear);
                 },
                 nullptr, "两侧均不具备换道条件，保持跟车"});

  addTransition({DrivingState::kFollow, DrivingEvent::kObstacleCleared,
                 DrivingState::kLaneKeep, guard_always, nullptr, "前方畅通，恢复巡航"});

  addTransition({DrivingState::kFollow, DrivingEvent::kLeadVehicleGone,
                 DrivingState::kLaneKeep, guard_always, nullptr, "前车驶离，恢复巡航"});

  addTransition({DrivingState::kFollow, DrivingEvent::kYieldRequired,
                 DrivingState::kYield, guard_always, nullptr, "存在让行义务"});

  // ---- 换道 ----
  for (DrivingState change : {DrivingState::kLaneChangeLeft, DrivingState::kLaneChangeRight}) {
    addTransition({change, DrivingEvent::kLaneChangeCompleted, DrivingState::kLaneKeep,
                   guard_always, nullptr, "换道完成"});

    // 换道途中若目标车道出现来车，必须中止并退回跟车
    addTransition({change, DrivingEvent::kObstacleDetected, DrivingState::kFollow,
                   [](const FsmContext& ctx) {
                     return !(ctx.left_lane_clear || ctx.right_lane_clear);
                   },
                   nullptr, "目标车道被占用，中止换道"});

    addTransition({change, DrivingEvent::kLaneChangeAborted, DrivingState::kFollow,
                   guard_always, nullptr, "换道中止，退回跟车"});
  }

  // ---- 超车 ----
  addTransition({DrivingState::kOvertake, DrivingEvent::kOvertakeCompleted,
                 DrivingState::kLaneKeep, guard_always, nullptr, "超车完成"});
  addTransition({DrivingState::kOvertake, DrivingEvent::kObstacleDetected,
                 DrivingState::kFollow, guard_always, nullptr, "超车受阻，退回跟车"});

  // ---- 让行 ----
  addTransition({DrivingState::kYield, DrivingEvent::kYieldCompleted,
                 DrivingState::kLaneKeep, guard_always, nullptr, "让行结束"});
  addTransition({DrivingState::kYield, DrivingEvent::kObstacleCleared,
                 DrivingState::kLaneKeep, guard_always, nullptr, "通行权已获得"});

  // ---- 路口转向 ----
  for (DrivingState turn : {DrivingState::kTurnLeft, DrivingState::kTurnRight}) {
    addTransition({turn, DrivingEvent::kObstacleCleared, DrivingState::kLaneKeep,
                   guard_always, nullptr, "转向完成"});
  }

  // ---- 停车与起步 ----
  for (DrivingState from :
       {DrivingState::kLaneKeep, DrivingState::kFollow, DrivingState::kYield}) {
    addTransition({from, DrivingEvent::kStopRequested, DrivingState::kStop,
                   guard_always,
                   [](FsmContext& ctx) { ctx.speed_limit_override = 0.0; },
                   "收到停车指令"});
  }

  addTransition({DrivingState::kStop, DrivingEvent::kGoRequested,
                 DrivingState::kLaneKeep, guard_always,
                 [](FsmContext& ctx) { ctx.speed_limit_override = -1.0; },
                 "收到起步指令"});
}

// ---------------------------------------------------------------------------
// 导出
// ---------------------------------------------------------------------------

std::string DrivingFsm::toMermaid() const {
  std::ostringstream oss;
  oss << "stateDiagram-v2\n";
  oss << "    [*] --> " << toString(state_) << "\n";

  for (const Transition& transition : transitions_) {
    oss << "    " << toString(transition.from) << " --> " << toString(transition.to)
        << " : " << toString(transition.event) << "\n";
  }
  return oss.str();
}

std::string DrivingFsm::toDot() const {
  std::ostringstream oss;
  oss << "digraph DrivingFsm {\n";
  oss << "  rankdir=LR;\n";
  oss << "  node [shape=box];\n";

  // 只输出实际出现过的状态，避免把全部枚举都画出来
  std::vector<DrivingState> states;
  states.push_back(state_);
  for (const Transition& transition : transitions_) {
    if (std::find(states.begin(), states.end(), transition.from) == states.end()) {
      states.push_back(transition.from);
    }
    if (std::find(states.begin(), states.end(), transition.to) == states.end()) {
      states.push_back(transition.to);
    }
  }

  for (DrivingState state : states) {
    oss << "  \"" << toString(state) << "\";\n";
  }

  for (const Transition& transition : transitions_) {
    oss << "  \"" << toString(transition.from) << "\" -> \"" << toString(transition.to)
        << "\" [label=\"" << toString(transition.event) << "\"];\n";
  }

  oss << "}\n";
  return oss.str();
}

}  // namespace adsim
