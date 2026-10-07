// =============================================================================
//  Fsm.h — 驾驶行为有限状态机
//
//  上层行为决策的两大支柱之一（另一半是行为树，见 BehaviorTree.h）。
//
//  两者的分工：
//    * 状态机负责"车辆当前处于哪种驾驶模式"——模式是互斥的、有明确驻留时间的，
//      且模式切换本身就是需要被记录和审计的事件。
//    * 行为树负责模式内部的"任务分解与重试"——同一模式下可能有多种执行路径。
//
//  这样划分的好处是：状态的合法性由转移表严格约束（不可能从"车道保持"直接
//  跳到"左转完成"），而任务级的灵活性交给行为树，两者互不干扰。
//
//  转移表支持带守卫（guard）与动作（action）：
//    guard  —— 事件发生不代表一定转移，还要看条件是否满足
//               （例如"请求换道"在目标车道有车时不应生效）
//    action —— 转移发生时执行的副作用（重置计数器、记录日志等）
// =============================================================================
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace adsim {

/// 驾驶模式
enum class DrivingState {
  kLaneKeep,           ///< 车道保持
  kFollow,             ///< 跟车
  kLaneChangeLeft,     ///< 向左换道
  kLaneChangeRight,    ///< 向右换道
  kOvertake,           ///< 超车
  kYield,              ///< 让行
  kTurnLeft,           ///< 左转
  kTurnRight,          ///< 右转
  kStop,               ///< 停车等待
  kEmergencyBrake,     ///< 紧急制动
};

const char* toString(DrivingState state);

/// 触发状态转移的事件
enum class DrivingEvent {
  kNone,
  kObstacleDetected,     ///< 前方出现障碍物
  kObstacleCleared,      ///< 前方障碍物消失
  kLeadVehicleSlow,      ///< 前车减速
  kLeadVehicleGone,      ///< 前车驶离
  kLaneChangeRequested,  ///< 请求换道
  kLaneChangeCompleted,  ///< 换道完成
  kLaneChangeAborted,    ///< 换道中止
  kOvertakeRequested,    ///< 请求超车
  kOvertakeCompleted,    ///< 超车完成
  kIntersectionAhead,    ///< 前方路口
  kTurnRequested,        ///< 请求转向（由目标路径决定方向）
  kYieldRequired,        ///< 需要让行
  kYieldCompleted,       ///< 让行结束
  kEmergencyDetected,    ///< 检测到紧急情况
  kEmergencyCleared,     ///< 紧急情况解除
  kStopRequested,        ///< 请求停车
  kGoRequested,          ///< 请求起步
};

const char* toString(DrivingEvent event);

/// 状态机运行时上下文：守卫与动作通过它读写决策所需的全部数据。
///
/// 刻意用一个具体的结构体而非模板参数：状态机的输入本身就应该被显式定义，
/// 否则守卫函数会退化成"能访问一切的上帝对象"。
struct FsmContext {
  double time{0.0};

  // ---- 环境感知 ----
  bool obstacle_ahead{false};
  double obstacle_distance{1e9};
  bool lead_detected{false};
  double lead_distance{1e9};
  double lead_speed{0.0};
  double lead_ttc{1e9};

  // ---- 相邻车道 ----
  bool left_lane_exists{false};
  bool left_lane_clear{false};
  bool right_lane_exists{false};
  bool right_lane_clear{false};

  // ---- 路口 ----
  bool intersection_ahead{false};
  double distance_to_intersection{1e9};
  bool turn_left_requested{false};

  // ---- 让行对象 ----
  bool yield_required{false};
  double yield_ttc{1e9};

  // ---- 安全 ----
  double min_ttc{1e9};
  double min_gap{1e9};
  bool collision_imminent{false};

  // ---- 自车 ----
  double ego_speed{0.0};
  double desired_speed{0.0};

  // ---- 状态机输出（动作写入） ----
  double speed_limit_override{-1.0};  ///< <0 表示不覆盖
  bool allow_lane_change{true};
  std::string last_reason;
};

/// 一条状态转移规则
struct Transition {
  DrivingState from{DrivingState::kLaneKeep};
  DrivingEvent event{DrivingEvent::kNone};
  DrivingState to{DrivingState::kLaneKeep};
  /// 守卫：返回 false 则本次转移不生效（事件被忽略）
  std::function<bool(const FsmContext&)> guard;
  /// 转移发生时执行的副作用
  std::function<void(FsmContext&)> action;
  std::string description;
};

class DrivingFsm {
 public:
  explicit DrivingFsm(DrivingState initial = DrivingState::kLaneKeep);

  /// 注册转移规则。同一 (from, event) 可注册多条，
  /// 按注册顺序取第一条守卫通过者——这天然表达了优先级。
  void addTransition(const Transition& transition);

  /// 处理事件。返回是否发生了状态转移。
  bool handleEvent(DrivingEvent event, FsmContext& context);

  /// 每一帧调用：根据上下文自动派生事件并尝试转移。
  /// 返回本次发生的转移描述；无转移时返回空字符串。
  std::string update(FsmContext& context);

  DrivingState state() const { return state_; }
  DrivingState previousState() const { return previous_; }

  /// 在当前状态的驻留时长
  double timeInState() const { return time_in_state_; }

  /// 状态切换次数
  std::size_t transitionCount() const { return transition_count_; }

  /// 最近的转移记录（最多保留 64 条）
  struct TransitionRecord {
    double time{0.0};
    DrivingState from{DrivingState::kLaneKeep};
    DrivingState to{DrivingState::kLaneKeep};
    DrivingEvent event{DrivingEvent::kNone};
    std::string description;
  };
  const std::vector<TransitionRecord>& history() const { return history_; }

  void reset(DrivingState initial = DrivingState::kLaneKeep);

  /// 导出状态转移图，便于评审与文档化
  std::string toMermaid() const;
  std::string toDot() const;

  /// 注册内置的驾驶规则集（覆盖跟车、换道、让行、紧急制动等）
  void addDefaultTransitions();

 private:
  /// 由上下文推导当前应触发的事件（按优先级）
  DrivingEvent deriveEvent(const FsmContext& context) const;

  /// 在当前状态下查找可用的转移
  const Transition* findTransition(DrivingEvent event, const FsmContext& context) const;

  DrivingState state_{DrivingState::kLaneKeep};
  DrivingState previous_{DrivingState::kLaneKeep};
  double time_{0.0};
  double time_in_state_{0.0};
  std::size_t transition_count_{0};

  std::vector<Transition> transitions_;
  std::vector<TransitionRecord> history_;
};

}  // namespace adsim
