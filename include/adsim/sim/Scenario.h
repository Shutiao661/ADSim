// =============================================================================
//  Scenario.h — 场景定义与场景库
//
//  场景 = 初始世界 + 自车初始状态 + 其他交通参与者的行为脚本 + 终止条件。
//
//  内置场景聚焦于实车路测中难以复现、却最容易暴露决策缺陷的几类工况：
//    * 前车急刹        —— 考验纵向控制与制动时机
//    * 旁车加塞        —— 考验博弈与让行决策
//    * 无保护左转      —— 考验对向车流间隙判断
//    * 行人横穿        —— 考验对弱势交通参与者的保守程度
//    * 弯道超速        —— 考验曲率-速度协调
//
//  这些正是"Corner Case 难复现"的典型代表，也正是仿真平台的价值所在。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/sim/World.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 场景元信息
struct ScenarioInfo {
  std::string name;
  std::string description;
  std::string category;      ///< 如"纵向"/"横向"/"路口"/"弱势参与者"
  bool is_critical{false};   ///< 是否属于危险工况
  double duration{20.0};
};

class Scenario {
 public:
  virtual ~Scenario() = default;

  virtual ScenarioInfo info() const = 0;

  /// 构造初始世界
  virtual World buildWorld() const = 0;

  /// 自车初始状态
  virtual VehicleState initialEgoState() const = 0;

  /// 返回自车的目标终点（用于判断是否完成），为空表示不设终点
  virtual std::vector<Vec2> goalPath() const { return {}; }

  /// 每步更新其他交通参与者的状态
  /// @param world   可变世界
  /// @param time    当前仿真时刻
  /// @param dt      步长
  virtual void update(World& world, double time, double dt) const = 0;

  /// 判断仿真是否应当提前结束
  virtual bool shouldTerminate(const SimEngine& engine) const;

  /// 便捷方法：按给定配置跑完整场景
  SimulationResult run(const SimEngine::Config& config) const;

  /// 配置被测策略后再运行
  SimulationResult run(const SimEngine::Config& config,
                       std::shared_ptr<IPolicy> policy) const;
};

// ---------------------------------------------------------------------------
// 具体场景
// ---------------------------------------------------------------------------

/// 前车急刹：自车跟随前车，前车在指定时刻以指定减速度制动
class LeadBrakeScenario : public Scenario {
 public:
  struct Params {
    double ego_speed{13.0};
    double lead_speed{11.0};
    double initial_gap{20.0};
    double brake_time{3.0};        ///< 前车开始制动的时刻
    double brake_deceleration{-5.0};
  };

  LeadBrakeScenario();
  explicit LeadBrakeScenario(const Params& params);

  ScenarioInfo info() const override;
  World buildWorld() const override;
  VehicleState initialEgoState() const override;
  void update(World& world, double time, double dt) const override;
  std::vector<Vec2> goalPath() const override;

 private:
  Params params_;
};

/// 旁车加塞：相邻车道车辆在自车前方强行并入
class CutInScenario : public Scenario {
 public:
  struct Params {
    double ego_speed{13.0};
    double cut_in_speed{12.0};
    double initial_gap{15.0};       ///< 加塞车与自车的纵向距离
    double cut_in_time{2.5};        ///< 开始加塞的时刻
    double cut_in_duration{2.0};    ///< 完成加塞所需时间
    bool aggressive{false};         ///< 激进模式下加塞距离更近
  };

  CutInScenario();
  explicit CutInScenario(const Params& params);

  ScenarioInfo info() const override;
  World buildWorld() const override;
  VehicleState initialEgoState() const override;
  void update(World& world, double time, double dt) const override;
  std::vector<Vec2> goalPath() const override;

 private:
  Params params_;
};

/// 无保护左转：路口内与对向直行车流博弈
class UnprotectedLeftTurnScenario : public Scenario {
 public:
  struct Params {
    double ego_speed{8.0};
    double oncoming_speed{12.0};
    double oncoming_distance{60.0};
    double intersection_distance{40.0};
  };

  UnprotectedLeftTurnScenario();
  explicit UnprotectedLeftTurnScenario(const Params& params);

  ScenarioInfo info() const override;
  World buildWorld() const override;
  VehicleState initialEgoState() const override;
  void update(World& world, double time, double dt) const override;
  std::vector<Vec2> goalPath() const override;

 private:
  Params params_;
};

/// 行人横穿：行人从路侧突然进入车道
class PedestrianCrossingScenario : public Scenario {
 public:
  struct Params {
    double ego_speed{11.0};
    double pedestrian_start_time{2.0};
    double pedestrian_speed{1.4};   ///< 行人步速 (m/s)
    double pedestrian_start_lateral{6.0};  ///< 起始横向距离
  };

  PedestrianCrossingScenario();
  explicit PedestrianCrossingScenario(const Params& params);

  ScenarioInfo info() const override;
  World buildWorld() const override;
  VehicleState initialEgoState() const override;
  void update(World& world, double time, double dt) const override;

 private:
  Params params_;
};

/// 弯道行驶：考验曲率-速度协调与路径平滑度
class CurvedRoadScenario : public Scenario {
 public:
  struct Params {
    double ego_speed{15.0};
    double curvature{0.02};   ///< 1/m，对应约 50m 转弯半径
    double road_length{200.0};
  };

  CurvedRoadScenario();
  explicit CurvedRoadScenario(const Params& params);

  ScenarioInfo info() const override;
  World buildWorld() const override;
  VehicleState initialEgoState() const override;
  void update(World& world, double time, double dt) const override;

 private:
  Params params_;
};

// ---------------------------------------------------------------------------
// 场景注册表
// ---------------------------------------------------------------------------

class ScenarioRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Scenario>()>;

  static ScenarioRegistry& instance();

  void registerScenario(const std::string& name, Factory factory);
  std::unique_ptr<Scenario> create(const std::string& name) const;

  std::vector<std::string> names() const;
  std::vector<ScenarioInfo> allInfo() const;

 private:
  ScenarioRegistry();

  std::vector<std::pair<std::string, Factory>> entries_;
};

}  // namespace adsim
