// =============================================================================
//  PlanningPolicy.h — 决策规划控制闭环
//
//  把四个模块串成一条完整的自动驾驶流水线，并作为 IPolicy 接入仿真内核：
//
//    感知  ──►  决策  ──►  路径规划  ──►  速度规划  ──►  控制
//   (perceive) (decide)  (PathPlanner) (SpeedPlanner) (pure pursuit + 纵向)
//
//  之所以做成 IPolicy 实现而不是把逻辑塞进仿真内核，是为了让"同一场景、
//  不同算法"的横向对比成为可能：仿真只负责推进物理与判定安全事件，
//  算法完全可替换。这正是仿真平台作为回归测试基础设施的价值所在。
//
//  横向控制采用纯跟踪（pure pursuit）：几何直观、参数少、对路径噪声不敏感。
//  纵向控制采用带加速度约束的比例控制器，与速度规划给出的剖面配合。
// =============================================================================
#pragma once

#include "adsim/planning/DecisionMaker.h"
#include "adsim/planning/PathPlanner.h"
#include "adsim/planning/SpeedPlanner.h"
#include "adsim/sim/SimEngine.h"

#include <string>
#include <vector>

namespace adsim {

class PlanningPolicy : public IPolicy {
 public:
  struct Config {
    DecisionMaker::Config decision;
    PathPlanner::Config path;
    SpeedPlanner::Config speed;

    /// 纯跟踪前视距离 = min_lookahead + lookahead_gain × 速度
    double min_lookahead{5.0};
    double lookahead_gain{0.8};
    /// 轴距 (m)，用于由曲率反算前轮转角
    double wheelbase{2.7};
    /// 前轮最大转角 (rad)
    double max_steering{0.6};
    /// 横向控制的比例增益
    double steering_gain{0.8};
    /// 期望巡航速度 (m/s)
    double desired_speed{11.0};

    /// 是否启用决策模块（关闭时退化为纯路径跟踪，用于对照实验）
    bool enable_decision{true};
    /// 是否启用路径优化（关闭时直接用参考路径，用于量化优化的增益）
    bool enable_path_optimization{true};
  };

  PlanningPolicy();
  explicit PlanningPolicy(const Config& config);

  Command computeCommand(const VehicleState& ego, const World& world, double dt) override;

  std::string name() const override;

  void reset() override;

  std::vector<Vec2> lastPlannedPath() const override { return last_path_; }

  // ---- 调试与复盘接口 ----

  const Decision& lastDecision() const { return last_decision_; }
  const PathPlanningResult& lastPathResult() const { return last_path_result_; }
  const Trajectory& lastTrajectory() const { return last_trajectory_; }
  const PerceptionResult& lastPerception() const { return last_perception_; }

  DecisionMaker& decisionMaker() { return decision_maker_; }
  PathPlanner& pathPlanner() { return path_planner_; }
  SpeedPlanner& speedPlanner() { return speed_planner_; }

  /// 决策周期的累计次数
  std::size_t cycleCount() const { return cycle_count_; }

  const Config& config() const { return config_; }
  Config& config() { return config_; }

  /// 由前视点计算前轮转角（公开以便单独测试）
  double purePursuitSteering(const VehicleState& ego,
                             const std::vector<Vec2>& path) const;

  /// 在路径上寻找距自车前视距离处的目标点；路径过短时返回最后一个点
  Vec2 findLookaheadPoint(const VehicleState& ego,
                          const std::vector<Vec2>& path) const;

 private:
  Config config_;

  DecisionMaker decision_maker_;
  PathPlanner path_planner_;
  SpeedPlanner speed_planner_;

  PerceptionResult last_perception_;
  Decision last_decision_;
  PathPlanningResult last_path_result_;
  Trajectory last_trajectory_;
  std::vector<Vec2> last_path_;

  int ego_lane_id_{-1};
  std::size_t cycle_count_{0};
};

}  // namespace adsim
