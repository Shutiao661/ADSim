// =============================================================================
//  SimEngine.h — 仿真内核
//
//  按固定步长推进世界状态，采集自车轨迹与安全指标。设计要点：
//    * 固定步长（默认 50ms）：保证规划模块的时序假设稳定，且结果可复现
//    * 确定性：不使用随机数，相同输入必得相同输出，便于回归测试比对
//    * 与规划解耦：通过 IPolicy 接口注入决策/规划算法，引擎本身不做决策
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/sim/VehicleModel.h"
#include "adsim/sim/World.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 安全事件
enum class SafetyEvent {
  kNone,
  kCollision,          ///< 发生碰撞
  kNearMiss,           ///< 距离低于阈值
  kLowTtc,             ///< 碰撞时间低于阈值
  kOffRoad,            ///< 驶出可行驶区域
  kOverSpeed,          ///< 超速
  kHarshBraking,       ///< 急刹
};

const char* toString(SafetyEvent event);

/// 单次安全事件记录
struct SafetyEventRecord {
  SafetyEvent event{SafetyEvent::kNone};
  double time{0.0};
  double value{0.0};       ///< 事件相关量（TTC / 距离 / 速度等）
  int object_id{-1};       ///< 相关物体
  std::string description;
};

/// 仿真结果
struct SimulationResult {
  std::string scenario_name;

  std::vector<double> time;                  ///< 各步时间
  std::vector<TrajectoryPoint> ego_states;   ///< 自车实际轨迹
  std::vector<TrajectoryPoint> planned_paths;///< 规划模块当步输出的路径（可能为空）
  std::vector<std::vector<RoadObject>> object_history;  ///< 各步其他物体状态

  std::vector<SafetyEventRecord> events;

  double min_ttc{1e9};            ///< 全程最小碰撞时间
  double min_distance{1e9};       ///< 全程与最近物体的最小距离
  double max_lateral_acceleration{0.0};
  double max_curvature{0.0};
  double max_jerk{0.0};
  double total_distance{0.0};
  double average_speed{0.0};

  int collision_count{0};
  int near_miss_count{0};
  int off_road_count{0};
  bool completed{false};          ///< 是否跑完全程未发生碰撞

  /// 是否为"高价值"（危险）场景，用于筛选回归测试用例
  bool isCritical() const {
    return collision_count > 0 || near_miss_count > 0 || min_ttc < 2.0;
  }

  std::string toString() const;
};

/// 决策/规划策略接口。仿真内核通过该接口调用被测算法，
/// 从而可以对不同算法做同一场景下的横向对比。
class IPolicy {
 public:
  virtual ~IPolicy() = default;

  /// 每步被调用一次，返回本步的执行指令
  struct Command {
    double acceleration{0.0};  ///< 目标加速度 (m/s²)
    double steering{0.0};      ///< 目标前轮转角 (rad)
    bool valid{false};         ///< 为 false 时引擎保持上一步指令
  };

  virtual Command computeCommand(const VehicleState& ego, const World& world,
                                 double dt) = 0;

  /// 策略名称，用于报告
  virtual std::string name() const = 0;

  /// 重置内部状态（每次仿真开始前调用）
  virtual void reset() {}

  /// 最近一次规划出的路径，供结果记录（无则返回空）
  virtual std::vector<Vec2> lastPlannedPath() const { return {}; }
};

class SimEngine {
 public:
  struct Config {
    double step_size{0.05};        ///< 仿真步长 (s)
    double max_duration{30.0};     ///< 最长仿真时长 (s)
    double near_miss_distance{1.5};///< 判定"接近事故"的距离阈值 (m)
    double low_ttc_threshold{2.0}; ///< 判定"低 TTC"的阈值 (s)
    double harsh_braking_threshold{-4.0};  ///< 急刹加速度阈值
    bool record_object_history{true};
    bool stop_on_collision{false}; ///< 碰撞后是否立即终止
  };

  SimEngine(const World& world, const VehicleModel& model);
  SimEngine(const World& world, const VehicleModel& model, const Config& config);

  /// 设置自车初始状态
  void setEgo(const VehicleState& state);

  /// 加入其他物体（会在仿真中被策略或场景脚本更新）
  void addObject(const RoadObject& object);

  /// 设置被测策略
  void setPolicy(std::shared_ptr<IPolicy> policy);
  IPolicy* policy() const { return policy_.get(); }

  /// 终止条件：返回 true 则提前结束仿真
  using TerminationCondition = std::function<bool(const SimEngine&)>;
  void setTerminationCondition(TerminationCondition condition);

  /// 运行仿真直至超时、碰撞或满足终止条件
  SimulationResult run();

  /// 推进一个步长。可用外部脚本每步更新其他物体状态。
  /// @param object_updater 在步进前更新其他物体，可为空
  void step(const std::function<void(World&, double)>& object_updater);

  // ---- 状态访问 ----
  const VehicleState& ego() const { return ego_; }
  const World& world() const { return world_; }
  World& mutableWorld() { return world_; }
  double time() const { return time_; }
  double stepSize() const { return config_.step_size; }
  const Config& config() const { return config_; }
  const SimulationResult& result() const { return result_; }

  // ---- 安全指标 ----
  /// 自车与指定物体的最近距离（OBB 间距）
  double distanceTo(const RoadObject& object) const;

  /// 与指定物体的碰撞时间。沿当前速度外推，无法在合理时间内相交时返回极大值。
  double timeToCollision(const RoadObject& object) const;

  /// 当步是否存在碰撞
  bool hasCollision() const;

  /// 自车是否已驶出可行驶区域
  bool isOffRoad() const;

  /// 记录一次安全事件
  void recordEvent(SafetyEvent event, double value, int object_id,
                   const std::string& description);

 private:
  void collectMetrics();
  void finalize();

  World world_;
  VehicleModel model_;
  Config config_;

  VehicleState ego_;
  std::shared_ptr<IPolicy> policy_;
  TerminationCondition termination_;

  SimulationResult result_;
  double time_{0.0};
  double previous_acceleration_{0.0};
  double previous_speed_{0.0};
  TrajectoryPoint previous_point_;
  bool has_previous_{false};

  std::vector<RoadObject> dynamic_objects_;
};

// ---------------------------------------------------------------------------
// OBB 相交检测（分离轴定理），供碰撞判定使用
// ---------------------------------------------------------------------------

/// 两个有向包围盒是否相交
bool obbIntersect(const Obb2& a, const Obb2& b);

/// 两个有向包围盒之间的最小间距；相交时返回 0
double obbDistance(const Obb2& a, const Obb2& b);

/// 点是否在 OBB 内部
bool obbContains(const Obb2& box, const Vec2& point);

}  // namespace adsim
