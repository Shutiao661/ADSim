// =============================================================================
//  DecisionMaker.h — 行为决策
//
//  把感知结果翻译成"做什么"：本周期应当执行哪种驾驶动作、目标速度多少、
//  目标车道是哪条。输出是抽象的驾驶意图，不含具体的路径点——那是
//  路径规划模块的职责。
//
//  决策采用"状态机 + 行为树"的两层结构：
//
//    ┌─────────────────────────────────────────────┐
//    │  有限状态机：决定当前的驾驶模式              │
//    │  互斥、有驻留时间、切换被严格约束            │
//    └────────────────────┬────────────────────────┘
//                         │ 当前模式作为行为树的前置条件
//    ┌────────────────────▼────────────────────────┐
//    │  行为树：决定模式内部的具体执行策略          │
//    │  可组合、可重试、可在运行中被中断            │
//    └─────────────────────────────────────────────┘
//
//  这样划分解决了一个实际问题：纯状态机在"换道"这类带多个子步骤的行为上
//  会产生状态爆炸（换道判断/换道中/换道失败/重试…），而纯行为树又缺少
//  对"车辆当前处在什么模式"这一全局约束的显式表达。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/planning/BehaviorTree.h"
#include "adsim/planning/Fsm.h"
#include "adsim/sim/World.h"

#include <cstddef>
#include <string>
#include <vector>

namespace adsim {

/// 驾驶动作（决策的输出）
enum class Maneuver {
  kLaneKeep,
  kFollow,
  kLaneChangeLeft,
  kLaneChangeRight,
  kOvertake,
  kYield,
  kTurnLeft,
  kTurnRight,
  kStop,
  kEmergencyBrake,
};

const char* toString(Maneuver maneuver);

/// 感知结果：决策模块的全部输入
struct PerceptionResult {
  VehicleState ego;
  /// 自车当前所在车道 id；-1 表示未知（未知时不做换道决策）
  int ego_lane_id{-1};

  // ---- 前车 ----
  bool lead_detected{false};
  int lead_id{-1};
  /// 净间距 (m)：自车车头到前车车尾的距离。
  /// 交通语境下的"车距"一律指净间距；用中心距会系统性高估安全裕度。
  double lead_distance{1e9};
  double lead_speed{0.0};
  double lead_ttc{1e9};

  // ---- 相邻车道 ----
  bool left_lane_exists{false};
  bool right_lane_exists{false};
  bool left_lane_clear{false};
  bool right_lane_clear{false};
  double left_gap{0.0};             ///< 目标车道上最近车辆的距离
  double right_gap{0.0};

  // ---- 路口 ----
  bool intersection_ahead{false};
  double distance_to_intersection{1e9};
  bool turn_left_requested{false};

  // ---- 弱势交通参与者 ----
  bool pedestrian_detected{false};
  double pedestrian_distance{1e9};

  // ---- 全局安全指标 ----
  double min_ttc{1e9};
  double min_gap{1e9};
  bool collision_imminent{false};

  // ---- 期望 ----
  double desired_speed{0.0};
  double speed_limit{13.9};
};

/// 决策结果
struct Decision {
  Maneuver maneuver{Maneuver::kLaneKeep};
  DrivingState fsm_state{DrivingState::kLaneKeep};

  double target_speed{0.0};          ///< 目标速度 (m/s)
  double target_acceleration{0.0};   ///< 目标加速度 (m/s²)
  int target_lane_id{-1};            ///< -1 表示保持当前车道
  double target_lateral_offset{0.0}; ///< 相对目标车道中心的横向偏移 (m)

  bool emergency{false};
  double confidence{0.0};            ///< 决策置信度 [0,1]
  std::string rationale;             ///< 决策依据（可读，用于复盘）

  std::string toString() const;
};

class DecisionMaker {
 public:
  struct Config {
    /// 自车车长 (m)，用于把传感器给出的中心距换算成净间距
    double ego_length{4.6};

    /// 跟车时距 (s)：车头时距低于该值开始减速
    double time_headway{1.8};
    /// 跟车最小间距 (m)，指**车头到前车车尾的净间距**而非中心距
    double min_follow_gap{5.0};
    /// 换道所需的最小目标车道间距 (m)
    double min_lane_change_gap{25.0};
    /// 触发紧急制动的 TTC 阈值 (s)
    double emergency_ttc{1.5};
    /// 触发制动的 TTC 阈值 (s)
    double braking_ttc{4.0};
    /// 期望速度超出限速的比例上限
    double speed_limit_margin{0.1};
    /// 决策置信度低于该值时输出保持车道的保守决策
    double min_confidence{0.3};
  };

  DecisionMaker();
  explicit DecisionMaker(const Config& config);

  /// 执行一次决策
  Decision decide(const PerceptionResult& perception, const World& world);

  /// 直接从仿真世界构造感知结果（把世界中的动态物体归类为前车/邻道车）。
  ///
  /// @param ego_length 自车车长，用于把传感器给出的中心距换算成净间距
  static PerceptionResult perceive(const VehicleState& ego, const World& world,
                                   int ego_lane_id, double desired_speed,
                                   double ego_length = 4.6);

  const DrivingFsm& fsm() const { return fsm_; }
  BehaviorTree& behaviorTree() { return behavior_tree_; }

  /// 决策历史（最多保留 128 条），用于复盘
  const std::vector<Decision>& history() const { return history_; }

  void reset();

  const Config& config() const { return config_; }

 private:
  /// 由感知结果推导当前应当输出的动作
  Maneuver selectManeuver(const PerceptionResult& perception, const FsmContext& context);

  /// 计算跟车目标速度（IDM 简化形式）
  double followSpeed(const PerceptionResult& perception) const;

  /// 计算安全速度上限（由 TTC 与最小间距共同约束）
  double safetySpeedLimit(const PerceptionResult& perception) const;

  Config config_;
  DrivingFsm fsm_;
  Blackboard blackboard_;
  BehaviorTree behavior_tree_;
  std::vector<Decision> history_;

  /// 换道意图的保持计数：换道是有惯性的动作，
  /// 单帧的瞬时判断不足以支撑，需要连续多帧确认
  int lane_change_intent_{0};
  Maneuver pending_lane_change_{Maneuver::kLaneKeep};
  bool last_lead_detected_{false};
  double time_{0.0};
};

}  // namespace adsim
