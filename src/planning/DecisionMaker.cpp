#include "adsim/planning/DecisionMaker.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace adsim {

namespace {

constexpr std::size_t kMaxHistory = 128;

/// 换道意图需要连续多少帧确认才真正执行。
/// 单帧的瞬时判断在感知抖动下会产生"换道-放弃-再换道"的反复，
/// 加一个确认窗口是最简单有效的抑制手段。
constexpr int kLaneChangeConfirmFrames = 3;

/// 计算 TTC 所需的最低自车速度 (m/s)。
///
/// 低于该速度时，按恒定速度外推得到的 TTC 会失去物理意义：
/// 自车以 0.1 m/s 蠕动、间距 2.4m 会算出约 1.5s 的"碰撞时间"，
/// 但实际上一脚刹车就能停住。用这种 TTC 驱动紧急制动会让车辆在
/// 停车点附近反复触发紧急状态，表现为决策抖动。
/// 低速下应改用绝对间距判据（见 decide 中的 collision_imminent）。
constexpr double kMinTtcSpeed = 1.0;

}  // namespace

const char* toString(Maneuver maneuver) {
  switch (maneuver) {
    case Maneuver::kLaneKeep: return "车道保持";
    case Maneuver::kFollow: return "跟车";
    case Maneuver::kLaneChangeLeft: return "向左换道";
    case Maneuver::kLaneChangeRight: return "向右换道";
    case Maneuver::kOvertake: return "超车";
    case Maneuver::kYield: return "让行";
    case Maneuver::kTurnLeft: return "左转";
    case Maneuver::kTurnRight: return "右转";
    case Maneuver::kStop: return "停车";
    case Maneuver::kEmergencyBrake: return "紧急制动";
  }
  return "未知";
}

std::string Decision::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);

  // 限定命名空间：成员函数 toString() 会遮蔽同名的自由函数
  oss << "[" << adsim::toString(maneuver) << "]";
  oss << " 速度 " << target_speed << " m/s";
  oss << ", 加速度 " << target_acceleration << " m/s²";
  if (target_lane_id >= 0) oss << ", 目标车道 " << target_lane_id;
  if (emergency) oss << "  【紧急】";
  oss.precision(0);
  oss << "  置信度 " << confidence * 100.0 << "%";
  if (!rationale.empty()) oss << "  —— " << rationale;
  return oss.str();
}

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

DecisionMaker::DecisionMaker() : DecisionMaker(Config()) {}

DecisionMaker::DecisionMaker(const Config& config)
    : config_(config), behavior_tree_(&blackboard_) {
  fsm_.addDefaultTransitions();
  history_.reserve(kMaxHistory);

  // ---- 行为树：模式内部的具体执行策略 ----
  // 与状态机互补：状态机回答"现在处于什么模式"，
  // 行为树回答"这个模式下具体该怎么做"。
  Blackboard* bb = &blackboard_;

  auto root = std::unique_ptr<Selector>(new Selector("决策"));

  // 1) 紧急制动优先级最高，任何情况下都先判定
  {
    auto sequence = std::unique_ptr<Sequence>(new Sequence("紧急制动判断"));
    sequence->addChild(std::unique_ptr<ConditionNode>(new ConditionNode(
        "存在碰撞风险",
        [](const Blackboard& b) { return b.getBool("collision_imminent"); }, bb)));
    sequence->addChild(std::unique_ptr<ActionNode>(new ActionNode(
        "输出紧急制动",
        [](Blackboard& b) {
          b.setString("maneuver", "emergency_brake");
          b.setString("rationale", "TTC 低于安全阈值，接管为紧急制动");
          return NodeStatus::kSuccess;
        },
        bb)));
    root->addChild(std::move(sequence));
  }

  // 2) 让行
  {
    auto sequence = std::unique_ptr<Sequence>(new Sequence("让行判断"));
    sequence->addChild(std::unique_ptr<ConditionNode>(new ConditionNode(
        "存在让行义务", [](const Blackboard& b) { return b.getBool("yield_required"); },
        bb)));
    sequence->addChild(std::unique_ptr<ActionNode>(new ActionNode(
        "输出让行",
        [](Blackboard& b) {
          b.setString("maneuver", "yield");
          b.setString("rationale", "存在通行权冲突，减速让行");
          return NodeStatus::kSuccess;
        },
        bb)));
    root->addChild(std::move(sequence));
  }

  // 3) 换道：需要"有障碍"与"目标车道可用"同时成立
  {
    auto sequence = std::unique_ptr<Sequence>(new Sequence("向左换道判断"));
    sequence->addChild(std::unique_ptr<ConditionNode>(new ConditionNode(
        "左侧可换道",
        [](const Blackboard& b) {
          return b.getBool("obstacle_ahead") && b.getBool("left_lane_exists") &&
                 b.getBool("left_lane_clear");
        },
        bb)));
    sequence->addChild(std::unique_ptr<ActionNode>(new ActionNode(
        "输出向左换道",
        [](Blackboard& b) {
          b.setString("maneuver", "lane_change_left");
          b.setString("rationale", "左侧车道空闲，换道绕开前方障碍");
          return NodeStatus::kSuccess;
        },
        bb)));
    root->addChild(std::move(sequence));
  }

  {
    auto sequence = std::unique_ptr<Sequence>(new Sequence("向右换道判断"));
    sequence->addChild(std::unique_ptr<ConditionNode>(new ConditionNode(
        "右侧可换道",
        [](const Blackboard& b) {
          return b.getBool("obstacle_ahead") && b.getBool("right_lane_exists") &&
                 b.getBool("right_lane_clear");
        },
        bb)));
    sequence->addChild(std::unique_ptr<ActionNode>(new ActionNode(
        "输出向右换道",
        [](Blackboard& b) {
          b.setString("maneuver", "lane_change_right");
          b.setString("rationale", "右侧车道空闲，换道绕开前方障碍");
          return NodeStatus::kSuccess;
        },
        bb)));
    root->addChild(std::move(sequence));
  }

  // 4) 跟车
  {
    auto sequence = std::unique_ptr<Sequence>(new Sequence("跟车判断"));
    sequence->addChild(std::unique_ptr<ConditionNode>(new ConditionNode(
        "前方有车", [](const Blackboard& b) { return b.getBool("lead_detected"); }, bb)));
    sequence->addChild(std::unique_ptr<ActionNode>(new ActionNode(
        "输出跟车",
        [](Blackboard& b) {
          b.setString("maneuver", "follow");
          b.setString("rationale", "前方有车，按跟车时距调整速度");
          return NodeStatus::kSuccess;
        },
        bb)));
    root->addChild(std::move(sequence));
  }

  // 5) 兜底：车道保持
  root->addChild(std::unique_ptr<ActionNode>(new ActionNode(
      "输出车道保持",
      [](Blackboard& b) {
        b.setString("maneuver", "lane_keep");
        b.setString("rationale", "前方畅通，保持车道巡航");
        return NodeStatus::kSuccess;
      },
      bb)));

  behavior_tree_.setRoot(std::move(root));
}

// ---------------------------------------------------------------------------
// 感知
// ---------------------------------------------------------------------------

PerceptionResult DecisionMaker::perceive(const VehicleState& ego, const World& world,
                                         int ego_lane_id, double desired_speed,
                                         double ego_length) {
  PerceptionResult perception;
  perception.ego = ego;
  perception.ego_lane_id = ego_lane_id;
  perception.desired_speed = desired_speed;
  perception.speed_limit = desired_speed > 0.0 ? desired_speed : 13.9;

  const Lane* ego_lane = world.findLane(ego_lane_id);
  if (ego_lane != nullptr) {
    perception.speed_limit = ego_lane->speed_limit;
    perception.left_lane_exists = ego_lane->left_lane_id >= 0;
    perception.right_lane_exists = ego_lane->right_lane_id >= 0;
  }

  // 默认认为邻道可用，发现车辆后再否决——
  // 这样在没有感知数据的空旷路段也能正常换道
  perception.left_lane_clear = perception.left_lane_exists;
  perception.right_lane_clear = perception.right_lane_exists;

  const Vec2 ego_position = ego.pose().position();
  const Vec2 forward{std::cos(ego.theta), std::sin(ego.theta)};

  for (const RoadObject& object : world.objects()) {
    const Vec2 delta = object.position() - ego_position;
    const double distance = delta.norm();
    if (distance > 120.0) continue;  // 超出关注范围

    const double longitudinal = delta.dot(forward);
    const double lateral = delta.cross(forward);  // 左正右负

    if (object.type == RoadObject::Type::kPedestrian) {
      perception.pedestrian_detected = true;
      perception.pedestrian_distance = std::min(perception.pedestrian_distance, distance);
      continue;
    }

    // 同车道且在自车前方 → 前车
    const bool same_lane = std::abs(lateral) < 2.0;
    if (same_lane && longitudinal > 0.0 && longitudinal < perception.lead_distance) {
      // 换算成净间距：中心距减去两车各半个车长。
      //
      // 这一步不能省。交通语境下的"车距"一律指车头到前车车尾的净间距，
      // 而传感器给出的是中心距。对 4.6m 长的车来说，5m 中心距只剩 0.4m
      // 净距——车实际上已经贴上去了。用中心距驱动跟车与制动决策，
      // 会系统性地高估安全裕度，实测中表现为"看着保持了 5m 车距，
      // 实际上追尾了"。
      const double net_gap = longitudinal - object.length * 0.5 - ego_length * 0.5;

      perception.lead_detected = true;
      perception.lead_id = object.id;
      perception.lead_distance = net_gap;
      perception.lead_speed = object.speed;

      const double relative_speed = ego.speed - object.speed;
      // 只在"确实在接近且速度足够高"时才算 TTC；否则不存在有意义的碰撞时间
      perception.lead_ttc = (relative_speed > 0.1 && ego.speed > kMinTtcSpeed)
                                ? net_gap / relative_speed
                                : 1e9;
    }

    // 相邻车道 → 判断该车道是否可换
    if (std::abs(lateral) > 2.0 && std::abs(lateral) < 6.0 && longitudinal > -10.0) {
      const bool left_side = lateral > 0.0;
      if (left_side) {
        perception.left_gap = perception.left_gap == 0.0
                                  ? distance
                                  : std::min(perception.left_gap, distance);
        if (distance < 25.0) perception.left_lane_clear = false;
      } else {
        perception.right_gap = perception.right_gap == 0.0
                                   ? distance
                                   : std::min(perception.right_gap, distance);
        if (distance < 25.0) perception.right_lane_clear = false;
      }
    }

    perception.min_gap = std::min(perception.min_gap, distance);
  }

  perception.min_ttc = perception.lead_ttc;

  // 行人进入本车道前方较近处时，视为需要让行
  if (perception.pedestrian_detected && perception.pedestrian_distance < 30.0) {
    perception.min_ttc = std::min(perception.min_ttc,
                                  perception.pedestrian_distance /
                                      std::max(ego.speed, 0.1));
  }

  return perception;
}

// ---------------------------------------------------------------------------
// 决策
// ---------------------------------------------------------------------------

double DecisionMaker::followSpeed(const PerceptionResult& perception) const {
  if (!perception.lead_detected) return perception.desired_speed;

  // 期望间距 = 最小间距 + 车头时距 × 自车速度
  const double desired_gap =
      config_.min_follow_gap + config_.time_headway * std::max(perception.ego.speed, 0.0);

  const double gap_error = perception.lead_distance - desired_gap;

  // 简化 IDM：间距误差为正时允许加速，为负时按比例减速
  const double speed =
      perception.lead_speed + 0.5 * gap_error;

  return clamp(speed, 0.0, perception.desired_speed);
}

double DecisionMaker::safetySpeedLimit(const PerceptionResult& perception) const {
  if (!perception.lead_detected) return perception.desired_speed;

  // ---- 间距约束（与速度无关，低速下同样有效）----
  //
  // 这是最关键的一条：由 v² − v_lead² = 2·a·d 反解出"在剩余间距内能把速度
  // 降到前车速度"的上限。它不依赖 TTC，因此在低速蠕行时依然生效。
  //
  // 早期版本只依赖 TTC，而 TTC 在低速下被主动抑制（因为恒定速度外推在
  // 低速时失去意义），两者叠加的后果是低速时安全约束完全失效：决策层在
  // 距停止前车仅 4.8m 时仍以 11 m/s 为目标加速。任何单一判据都不足以
  // 覆盖全速度域，必须并存。
  constexpr double kConstraintDeceleration = 4.0;
  const double available = perception.lead_distance - config_.min_follow_gap;
  if (available <= 0.0) {
    return 0.0;  // 已进入最小间距，不允许再有任何速度
  }

  const double stopping_limit = std::sqrt(
      std::max(perception.lead_speed * perception.lead_speed +
                   2.0 * kConstraintDeceleration * available,
               0.0));

  double allowed = std::min(perception.desired_speed, stopping_limit);

  // ---- TTC 约束（高速下提供更激进的减速）----
  if (perception.lead_ttc <= config_.braking_ttc) {
    const double ratio = clamp(perception.lead_ttc / config_.braking_ttc, 0.0, 1.0);
    allowed = std::min(
        allowed, perception.lead_speed + ratio * (perception.ego.speed - perception.lead_speed));
  }

  return clamp(allowed, 0.0, perception.desired_speed);
}

Maneuver DecisionMaker::selectManeuver(const PerceptionResult& perception,
                                       const FsmContext& context) {
  // 状态机给出的是"模式级"结论，行为树细化到"动作级"。
  // 这里先看状态机是否已经强制了某个模式（例如紧急制动），
  // 若没有则交给行为树决定。
  if (fsm_.state() == DrivingState::kEmergencyBrake || context.collision_imminent) {
    return Maneuver::kEmergencyBrake;
  }

  const std::string maneuver = blackboard_.getString("maneuver", "lane_keep");

  Maneuver selected = Maneuver::kLaneKeep;
  if (maneuver == "emergency_brake") selected = Maneuver::kEmergencyBrake;
  else if (maneuver == "yield") selected = Maneuver::kYield;
  else if (maneuver == "lane_change_left") selected = Maneuver::kLaneChangeLeft;
  else if (maneuver == "lane_change_right") selected = Maneuver::kLaneChangeRight;
  else if (maneuver == "follow") selected = Maneuver::kFollow;

  // 换道需要连续多帧确认，抑制感知抖动导致的反复
  if (selected == Maneuver::kLaneChangeLeft || selected == Maneuver::kLaneChangeRight) {
    if (selected == pending_lane_change_) {
      ++lane_change_intent_;
    } else {
      pending_lane_change_ = selected;
      lane_change_intent_ = 1;
    }

    if (lane_change_intent_ < kLaneChangeConfirmFrames) {
      // 尚未确认，本帧先按跟车处理
      return Maneuver::kFollow;
    }
  } else {
    pending_lane_change_ = Maneuver::kLaneKeep;
    lane_change_intent_ = 0;
  }

  // 状态机处于让行模式时不允许换道
  if (fsm_.state() == DrivingState::kYield &&
      (selected == Maneuver::kLaneChangeLeft || selected == Maneuver::kLaneChangeRight)) {
    return Maneuver::kYield;
  }

  (void)perception;
  return selected;
}

Decision DecisionMaker::decide(const PerceptionResult& perception, const World& world) {
  // ---- 1. 组装状态机上下文 ----
  FsmContext context;
  context.time = time_;
  context.obstacle_ahead = perception.lead_detected && perception.lead_distance < 60.0 &&
                           perception.lead_speed < perception.desired_speed - 1.0;
  context.obstacle_distance = perception.lead_distance;
  context.lead_detected = perception.lead_detected;
  context.lead_distance = perception.lead_distance;
  context.lead_speed = perception.lead_speed;
  context.lead_ttc = perception.lead_ttc;
  context.left_lane_exists = perception.left_lane_exists;
  context.left_lane_clear = perception.left_lane_clear;
  context.right_lane_exists = perception.right_lane_exists;
  context.right_lane_clear = perception.right_lane_clear;
  context.intersection_ahead = perception.intersection_ahead;
  context.distance_to_intersection = perception.distance_to_intersection;
  context.turn_left_requested = perception.turn_left_requested;
  context.yield_required = perception.pedestrian_detected &&
                           perception.pedestrian_distance < 25.0;
  context.min_ttc = perception.min_ttc;
  context.min_gap = perception.min_gap;

  // 紧急判定同时看 TTC 与绝对间距：
  // 低速下 TTC 不可用（见 kMinTtcSpeed 的说明），但"已经贴到前车脸上"
  // 这件事无论如何都必须触发紧急制动，否则会漏掉真实的危险工况。
  const bool gap_too_close = perception.lead_detected &&
                             perception.lead_distance < config_.min_follow_gap * 0.4;
  context.collision_imminent =
      perception.collision_imminent || perception.min_ttc < config_.emergency_ttc ||
      gap_too_close;
  context.ego_speed = perception.ego.speed;
  context.desired_speed = perception.desired_speed;

  fsm_.update(context);

  // ---- 2. 写入黑板并 tick 行为树 ----
  blackboard_.setBool("collision_imminent", context.collision_imminent);
  blackboard_.setBool("yield_required", context.yield_required);
  blackboard_.setBool("obstacle_ahead", context.obstacle_ahead);
  blackboard_.setBool("lead_detected", perception.lead_detected);
  blackboard_.setBool("left_lane_exists", perception.left_lane_exists);
  blackboard_.setBool("left_lane_clear", perception.left_lane_clear);
  blackboard_.setBool("right_lane_exists", perception.right_lane_exists);
  blackboard_.setBool("right_lane_clear", perception.right_lane_clear);
  blackboard_.setDouble("lead_distance", perception.lead_distance);
  blackboard_.setDouble("min_ttc", perception.min_ttc);

  behavior_tree_.tick();

  // ---- 3. 得到动作 ----
  Decision decision;
  decision.maneuver = selectManeuver(perception, context);
  decision.fsm_state = fsm_.state();
  decision.rationale = blackboard_.getString("rationale", "");

  // ---- 4. 目标速度：取"期望速度 / 跟车速度 / 安全速度"的最小值 ----
  const double speed_limit =
      perception.speed_limit * (1.0 + std::max(config_.speed_limit_margin, 0.0));
  const double desired = std::min(perception.desired_speed, speed_limit);

  double target = desired;
  switch (decision.maneuver) {
    case Maneuver::kEmergencyBrake:
      target = 0.0;
      decision.emergency = true;
      break;
    case Maneuver::kStop:
      target = 0.0;
      break;
    case Maneuver::kYield:
      target = std::min(desired, std::max(perception.ego.speed * 0.5, 0.0));
      break;
    case Maneuver::kFollow:
      target = std::max(0.0, followSpeed(perception));
      break;
    default:
      target = desired;
      break;
  }

  // 安全速度上限对任何动作都生效——这是最后一道防线
  target = std::min(target, safetySpeedLimit(perception));
  decision.target_speed = std::max(target, 0.0);

  // ---- 5. 目标加速度 ----
  const double speed_error = decision.target_speed - perception.ego.speed;
  constexpr double kSpeedGain = 0.8;
  constexpr double kMaxAcceleration = 2.5;
  constexpr double kMaxDeceleration = -5.0;

  double acceleration = kSpeedGain * speed_error;

  // 间距约束：若在剩余可用间距内无法把速度降到目标值，就必须按运动学
  // 反解出的减速度制动，而不能只靠速度误差的比例项。
  //
  // 比例项在低速下给出的减速度过小，车辆会一路"蹭"到前车跟前才停住——
  // 实测中会蠕行到距前车 1.7m 处，虽未碰撞但已失去安全裕度。
  // 由 v² − v_t² = 2·a·d 反解 a，是最直接的修正。
  if (!decision.emergency && perception.lead_detected &&
      perception.ego.speed > decision.target_speed) {
    const double available =
        std::max(perception.lead_distance - config_.min_follow_gap, 0.1);
    const double required = (perception.ego.speed * perception.ego.speed -
                             decision.target_speed * decision.target_speed) /
                            (2.0 * available);
    acceleration = std::min(acceleration, -required);
  }

  if (decision.emergency) {
    acceleration = -8.0;  // 紧急制动不受舒适性约束
  } else {
    acceleration = clamp(acceleration, kMaxDeceleration, kMaxAcceleration);
  }
  decision.target_acceleration = acceleration;

  // ---- 6. 目标车道与置信度 ----
  if (decision.maneuver == Maneuver::kLaneChangeLeft ||
      decision.maneuver == Maneuver::kLaneChangeRight) {
    // 目标车道必须由地图拓扑解算，绝不能凭方向猜一个 id
    const Lane* ego_lane = world.findLane(perception.ego_lane_id);
    if (ego_lane != nullptr) {
      decision.target_lane_id = decision.maneuver == Maneuver::kLaneChangeLeft
                                    ? ego_lane->left_lane_id
                                    : ego_lane->right_lane_id;
    }

    // 地图上没有对应邻道，换道决策不成立，退回跟车
    if (decision.target_lane_id < 0) {
      decision.maneuver = Maneuver::kFollow;
      decision.rationale += "（地图中无对应相邻车道，放弃换道）";
      decision.target_speed = std::max(0.0, followSpeed(perception));
      decision.target_speed = std::min(decision.target_speed,
                                       safetySpeedLimit(perception));
    }
  }

  decision.confidence = 0.7;
  if (decision.emergency) {
    decision.confidence = perception.lead_detected ? 0.95 : 0.6;
  } else if (decision.maneuver == Maneuver::kFollow) {
    decision.confidence = perception.lead_detected ? 0.9 : 0.4;
  } else if (decision.maneuver == Maneuver::kLaneChangeLeft ||
             decision.maneuver == Maneuver::kLaneChangeRight) {
    decision.confidence = 0.85;
  }

  // 置信度过低时保守处理：退回车道保持
  if (decision.confidence < config_.min_confidence && !decision.emergency) {
    decision.maneuver = Maneuver::kLaneKeep;
    decision.target_speed = std::min(desired, perception.ego.speed + 1.0);
    decision.target_acceleration = clamp(decision.target_acceleration, -2.0, 1.0);
    decision.rationale += "（置信度不足，保持车道）";
  }

  // ---- 7. 记录 ----
  history_.push_back(decision);
  if (history_.size() > kMaxHistory) {
    history_.erase(history_.begin());
  }

  time_ += 0.05;  // 决策周期与仿真步长一致
  last_lead_detected_ = perception.lead_detected;

  return decision;
}

void DecisionMaker::reset() {
  fsm_.reset();
  behavior_tree_.reset();
  blackboard_.clear();
  history_.clear();
  lane_change_intent_ = 0;
  pending_lane_change_ = Maneuver::kLaneKeep;
  last_lead_detected_ = false;
  time_ = 0.0;
}

}  // namespace adsim
