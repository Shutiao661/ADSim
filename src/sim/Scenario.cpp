// =============================================================================
//  Scenario.cpp — 内置场景库实现
//
//  场景脚本一律用**解析式**描述其他交通参与者的运动（正弦、线性、smoothstep），
//  不写状态机、不用随机数。理由有两条：
//    1) 解析式是时间的纯函数：任意时刻的状态都能直接算出，不受步长、调用次数
//       影响，仿真中断重跑也不会漂移；
//    2) 回归测试要求"同一场景两次运行逐点一致"，纯函数天然满足。
//  真实交通参与者的博弈行为由被测策略侧引入，场景只负责给出确定性的激励。
// =============================================================================
#include "adsim/sim/Scenario.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace adsim {

namespace {

// ---- 场景公共几何常量 ----
constexpr double kLaneWidth = 3.5;
constexpr double kRoadLength = 250.0;
constexpr double kEgoLaneY = -1.75;   ///< 右车道中心线（2 车道道路，右侧通行）
constexpr double kOtherLaneY = 1.75;  ///< 左车道中心线
/// 行人横穿位置：取在该处自车到达时刻与行人进入自车车道的时刻接近，
/// 使"行人突然进入车道"这一激励真正作用在自车前方
constexpr double kPedestrianCrossX = 70.0;

// ---- 物体 id ----
constexpr int kLeadId = 1;
constexpr int kCutInId = 2;
constexpr int kOncomingId = 3;
constexpr int kPedestrianId = 4;

// ---- 终止判据常量 ----
constexpr double kGoalRadius = 2.0;            ///< 与终点距离小于该值视为到达
constexpr double kGoalLateralTolerance = 5.0;  ///< 判定"越过终点"时的横向容差 (m)
constexpr double kStandstillSeconds = 5.0;     ///< 持续静止多久判定为任务停滞
constexpr double kStandstillSpeed = 0.1;       ///< 静止判定的速度阈值 (m/s)

/// 车辆初始状态（默认参数：静止起步）
VehicleState makeEgo(double x, double y, double theta, double speed) {
  VehicleState ego;
  ego.stamp = 0;
  ego.x = x;
  ego.y = y;
  ego.theta = theta;
  ego.speed = speed;
  ego.acceleration = 0.0;
  ego.steering = 0.0;
  ego.yaw_rate = 0.0;
  ego.throttle = 0.0;
  ego.brake = 0.0;
  ego.gear = 1;
  return ego;
}

/// 按时间解析推算"匀减速到停"的纵向运动：返回 (位移, 速度)
void brakingMotion(double t, double t_brake, double v0, double a, double& displacement,
                   double& speed) {
  displacement = v0 * t;
  speed = v0;
  if (t <= t_brake || v0 <= 0.0 || a >= 0.0) return;

  const double tau = t - t_brake;
  const double t_stop = -v0 / a;  // 减速到零所需时间
  const double travelled_before_brake = v0 * t_brake;
  if (tau < t_stop) {
    displacement = travelled_before_brake + v0 * tau + 0.5 * a * tau * tau;
    speed = v0 + a * tau;
  } else {
    // 已停稳：保持静止，绝不因解析式外推而"倒车"
    displacement = travelled_before_brake + v0 * t_stop + 0.5 * a * t_stop * t_stop;
    speed = 0.0;
  }
}

/// 生成一条等间距直行参考线（用于 goalPath）
std::vector<Vec2> straightGoal(double x0, double x1, double y, int segments) {
  std::vector<Vec2> path;
  if (segments < 1) segments = 1;
  path.reserve(static_cast<std::size_t>(segments) + 1);
  for (int i = 0; i <= segments; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(segments);
    path.push_back({x0 + (x1 - x0) * t, y});
  }
  return path;
}

/// 左转目标路径：一段直行 + 一段 90° 圆弧 + 一段直行（弧与两段直线相切）
std::vector<Vec2> leftTurnGoal(double start_x, double approach_y, double exit_x,
                               double turn_radius, double exit_y) {
  std::vector<Vec2> path;
  // 圆弧圆心：到两条切线等距的点（左转 => 圆心在左前方）
  const double center_x = exit_x - turn_radius;
  const double center_y = approach_y + turn_radius;

  path.push_back({start_x, approach_y});
  path.push_back({center_x, approach_y});  // 切点

  const int arc_steps = std::max(4, static_cast<int>(std::ceil(turn_radius * kPi * 0.5)));
  for (int i = 0; i <= arc_steps; ++i) {
    // 从 -90°（相对圆心指向切点）逆时针转到 0°
    const double phi = -kPi * 0.5 + kPi * 0.5 * static_cast<double>(i) /
                                        static_cast<double>(arc_steps);
    path.push_back({center_x + turn_radius * std::cos(phi),
                    center_y + turn_radius * std::sin(phi)});
  }

  path.push_back({exit_x, exit_y});
  return path;
}

/// 名称归一化：忽略大小写与分隔符，便于 create("lead_brake") 这类写法也能命中
std::string normalizeName(const std::string& name) {
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    if (c == '_' || c == '-' || c == ' ') continue;
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    out.push_back(c);
  }
  return out;
}

/// smoothstep：3p²-2p³。两端一阶导为 0，因此横向速度连续、无突变。
double smoothstep(double p) { return p * p * (3.0 - 2.0 * p); }

/// smoothstep 对时间的导数（用于给出横向速度，保证速度场与位移场自洽）
double smoothstepDerivative(double p, double duration) {
  if (duration < kEpsilon) return 0.0;
  return 6.0 * p * (1.0 - p) / duration;
}

}  // namespace

// ---------------------------------------------------------------------------
// Scenario 基类
// ---------------------------------------------------------------------------

bool Scenario::shouldTerminate(const SimEngine& engine) const {
  // 判据一：已到达目标路径终点附近（任务完成）
  const std::vector<Vec2> goal = goalPath();
  if (goal.size() >= 2) {
    const std::vector<double> arc = laneArcLength(goal);
    const Vec2 end = goal.back();
    const double end_heading = headingAlong(goal, arc, arc.back());
    const Vec2 direction{std::cos(end_heading), std::sin(end_heading)};
    const Vec2 offset = engine.ego().pose().position() - end;
    const double longitudinal = offset.dot(direction);  // >0 表示已越过终点
    const double lateral = std::fabs(offset.cross(direction));
    if (offset.norm() < kGoalRadius) return true;
    if (longitudinal > 0.0 && lateral < kGoalLateralTolerance) return true;
  }

  // 判据二：长时间静止。任务型场景里"停着不动"意味着已经没有推进余地
  // （例如被前车彻底堵死），继续跑只会白耗时间；显式终止让结果更可读。
  const double dt = engine.stepSize();
  if (dt > 0.0 && std::isfinite(dt)) {
    const std::size_t window =
        static_cast<std::size_t>(std::ceil(kStandstillSeconds / dt)) + 1;
    const std::vector<TrajectoryPoint>& history = engine.result().ego_states;
    if (history.size() >= window && window > 0) {
      bool all_stopped = true;
      for (std::size_t i = history.size() - window; i < history.size(); ++i) {
        if (std::fabs(history[i].v) > kStandstillSpeed) {
          all_stopped = false;
          break;
        }
      }
      if (all_stopped) return true;
    }
  }
  return false;
}

SimulationResult Scenario::run(const SimEngine::Config& config) const {
  return run(config, nullptr);
}

SimulationResult Scenario::run(const SimEngine::Config& config,
                               std::shared_ptr<IPolicy> policy) const {
  SimEngine engine(buildWorld(), VehicleModel(), config);
  engine.setEgo(initialEgoState());
  if (policy) {
    policy->reset();
    engine.setPolicy(std::move(policy));
  }
  engine.setTerminationCondition([this](const SimEngine& e) { return shouldTerminate(e); });

  const double dt = config.step_size;
  const std::function<void(World&, double)> updater = [this, dt](World& world, double t) {
    update(world, t, dt);
  };

  // 主循环：超时 / 终止条件 / （配置要求时）碰撞即停。
  // 每步先让场景脚本按解析式更新其他交通参与者，再推进自车。
  //
  // ⚠ 不变量：这里的三个退出条件必须与 SimEngine::run() 内部使用的完全一致
  //   （超时用同一个 max_duration，终止条件用同一个 shouldTerminate，碰撞即停
  //   用同一个开关）。因为循环退出后是交给 run() 收尾的，而 run() 内部以
  //   step(nullptr) 推进 —— 一旦某个退出条件没有被 run() 识别，它就会在循环
  //   退出后继续空跑：场景脚本不再被调用，其他交通参与者被冻结在原地，自车
  //   却照常行驶，于是产生"追尾静止前车"这类假碰撞。当前三个条件都已对齐
  //   （engine.setTerminationCondition 用的就是 shouldTerminate），
  //   回归测试 Sim.场景_提前终止_不空跑 对此做了固定。
  while (engine.time() < config.max_duration - 1e-9 && !shouldTerminate(engine) &&
         !(config.stop_on_collision && engine.hasCollision())) {
    engine.step(updater);
  }

  // 交回引擎收尾：至此退出条件已不满足，run() 会立即退出并做汇总
  // （平均速度、总里程、completed），结果与直接调用 run() 完全一致。
  SimulationResult result = engine.run();
  result.scenario_name = info().name;
  return result;
}

// ---------------------------------------------------------------------------
// 前车急刹
// ---------------------------------------------------------------------------

LeadBrakeScenario::LeadBrakeScenario() = default;

LeadBrakeScenario::LeadBrakeScenario(const Params& params) : params_(params) {}

ScenarioInfo LeadBrakeScenario::info() const {
  ScenarioInfo info;
  info.name = "LeadBrake";
  info.description = "自车跟随前车，前车在指定时刻以恒定减速度制动，考验纵向控制与制动时机";
  info.category = "纵向";
  info.is_critical = true;
  info.duration = 15.0;
  return info;
}

World LeadBrakeScenario::buildWorld() const {
  World world = World::straightRoad(2, kLaneWidth, kRoadLength);

  RoadObject lead;
  lead.id = kLeadId;
  lead.type = RoadObject::Type::kVehicle;
  lead.pose = Pose2(params_.initial_gap, kEgoLaneY, 0.0);
  lead.velocity = Vec2{params_.lead_speed, 0.0};
  lead.speed = params_.lead_speed;
  lead.length = 4.5;
  lead.width = 1.8;
  world.objects().push_back(lead);
  return world;
}

VehicleState LeadBrakeScenario::initialEgoState() const {
  return makeEgo(0.0, kEgoLaneY, 0.0, params_.ego_speed);
}

void LeadBrakeScenario::update(World& world, double time, double dt) const {
  (void)dt;  // 解析式与步长无关
  for (RoadObject& object : world.objects()) {
    if (object.id != kLeadId) continue;
    double displacement = 0.0;
    double speed = 0.0;
    brakingMotion(time, params_.brake_time, params_.lead_speed, params_.brake_deceleration,
                  displacement, speed);
    object.pose.x = params_.initial_gap + displacement;
    object.pose.y = kEgoLaneY;
    object.pose.theta = 0.0;
    object.velocity = Vec2{speed, 0.0};
    object.speed = speed;
  }
}

std::vector<Vec2> LeadBrakeScenario::goalPath() const {
  return straightGoal(0.0, kRoadLength, kEgoLaneY, 10);
}

// ---------------------------------------------------------------------------
// 旁车加塞
// ---------------------------------------------------------------------------

CutInScenario::CutInScenario() = default;

CutInScenario::CutInScenario(const Params& params) : params_(params) {}

ScenarioInfo CutInScenario::info() const {
  ScenarioInfo info;
  info.name = "CutIn";
  info.description = "相邻车道车辆以平滑横向轨迹并入自车前方，考验博弈与让行决策";
  info.category = "横向";
  info.is_critical = true;
  info.duration = 15.0;
  return info;
}

World CutInScenario::buildWorld() const {
  World world = World::straightRoad(2, kLaneWidth, kRoadLength);

  RoadObject cut_in;
  cut_in.id = kCutInId;
  cut_in.type = RoadObject::Type::kVehicle;
  // 初始位于左车道（相邻车道），纵向领先自车 initial_gap
  cut_in.pose = Pose2(params_.initial_gap, kOtherLaneY, 0.0);
  cut_in.velocity = Vec2{params_.cut_in_speed, 0.0};
  cut_in.speed = params_.cut_in_speed;
  cut_in.length = 4.5;
  cut_in.width = 1.8;
  world.objects().push_back(cut_in);
  return world;
}

VehicleState CutInScenario::initialEgoState() const {
  return makeEgo(0.0, kEgoLaneY, 0.0, params_.ego_speed);
}

void CutInScenario::update(World& world, double time, double dt) const {
  (void)dt;
  // 激进模式：加塞更早、更快、距离更近
  const double start = params_.aggressive ? params_.cut_in_time * 0.6 : params_.cut_in_time;
  const double duration =
      params_.aggressive ? params_.cut_in_duration * 0.7 : params_.cut_in_duration;
  const double gap = params_.aggressive ? params_.initial_gap * 0.6 : params_.initial_gap;

  double progress = 0.0;
  if (time > start && duration > kEpsilon) {
    progress = clamp((time - start) / duration, 0.0, 1.0);
  }
  const double lateral_done = smoothstep(progress);              // [0,1]
  const double lateral_rate = smoothstepDerivative(progress, duration);  // 1/s

  for (RoadObject& object : world.objects()) {
    if (object.id != kCutInId) continue;
    // 横向：从相邻车道中心线平移到自车车道中心线（共一个车道宽）
    const double y = kOtherLaneY - kLaneWidth * lateral_done;
    const double vy = -kLaneWidth * lateral_rate;
    object.pose.x = gap + params_.cut_in_speed * time;
    object.pose.y = y;
    // 车头始终指向实际运动方向：横向速度连续 => 航向连续，不会出现"斜着平移"
    object.pose.theta = std::atan2(vy, params_.cut_in_speed);
    object.velocity = Vec2{params_.cut_in_speed, vy};
    object.speed = std::sqrt(params_.cut_in_speed * params_.cut_in_speed + vy * vy);
  }
}

std::vector<Vec2> CutInScenario::goalPath() const {
  return straightGoal(0.0, kRoadLength, kEgoLaneY, 10);
}

// ---------------------------------------------------------------------------
// 无保护左转
// ---------------------------------------------------------------------------

UnprotectedLeftTurnScenario::UnprotectedLeftTurnScenario() = default;

UnprotectedLeftTurnScenario::UnprotectedLeftTurnScenario(const Params& params)
    : params_(params) {}

ScenarioInfo UnprotectedLeftTurnScenario::info() const {
  ScenarioInfo info;
  info.name = "UnprotectedLeftTurn";
  info.description = "路口内左转，与对向直行车流争夺间隙，考验交互博弈与间隙判断";
  info.category = "路口";
  info.is_critical = true;
  info.duration = 20.0;
  return info;
}

World UnprotectedLeftTurnScenario::buildWorld() const {
  // 路段长度必须覆盖自车与对向车的初始距离，否则它们的初始位置会落在路口外
  const double arm = std::max(params_.intersection_distance, params_.oncoming_distance) + 60.0;
  World world = World::intersection(arm, kLaneWidth);

  RoadObject oncoming;
  oncoming.id = kOncomingId;
  oncoming.type = RoadObject::Type::kVehicle;
  // 对向车在东侧路段的西行车道（y=+1.75），车头朝 -x
  oncoming.pose = Pose2(params_.oncoming_distance, kOtherLaneY, kPi);
  oncoming.velocity = Vec2{-params_.oncoming_speed, 0.0};
  oncoming.speed = params_.oncoming_speed;
  oncoming.length = 4.5;
  oncoming.width = 1.8;
  world.objects().push_back(oncoming);
  return world;
}

VehicleState UnprotectedLeftTurnScenario::initialEgoState() const {
  // 自车在路口西侧进口道：西侧路段东行车道（y=-1.75）
  return makeEgo(-params_.intersection_distance, kEgoLaneY, 0.0, params_.ego_speed);
}

void UnprotectedLeftTurnScenario::update(World& world, double time, double dt) const {
  (void)dt;
  for (RoadObject& object : world.objects()) {
    if (object.id != kOncomingId) continue;
    // 对向车沿直线匀速驶来（真实车流的主行为，不做随机扰动）
    object.pose.x = params_.oncoming_distance - params_.oncoming_speed * time;
    object.pose.y = kOtherLaneY;
    object.pose.theta = kPi;
    object.velocity = Vec2{-params_.oncoming_speed, 0.0};
    object.speed = params_.oncoming_speed;
  }
}

std::vector<Vec2> UnprotectedLeftTurnScenario::goalPath() const {
  // 左转路线：进口道直行 → 半径 5m 的 90° 左转（> 最小转弯半径 3.95m）→ 北向出口道
  const double exit_x = kLaneWidth * 0.5;  // 北向车道中心线 x = +1.75
  return leftTurnGoal(-params_.intersection_distance, kEgoLaneY, exit_x, 5.0, 40.0);
}

// ---------------------------------------------------------------------------
// 行人横穿
// ---------------------------------------------------------------------------

PedestrianCrossingScenario::PedestrianCrossingScenario() = default;

PedestrianCrossingScenario::PedestrianCrossingScenario(const Params& params)
    : params_(params) {}

ScenarioInfo PedestrianCrossingScenario::info() const {
  ScenarioInfo info;
  info.name = "PedestrianCrossing";
  info.description = "行人从路侧以恒定步速横穿车道，考验对弱势交通参与者的保守程度";
  info.category = "弱势参与者";
  info.is_critical = true;
  info.duration = 15.0;
  return info;
}

World PedestrianCrossingScenario::buildWorld() const {
  World world = World::straightRoad(2, kLaneWidth, kRoadLength);

  RoadObject pedestrian;
  pedestrian.id = kPedestrianId;
  pedestrian.type = RoadObject::Type::kPedestrian;
  // 行人轮廓按 0.5m × 0.5m 处理（肩宽量级），远小于车辆
  pedestrian.length = 0.5;
  pedestrian.width = 0.5;
  const double start_y = kEgoLaneY + params_.pedestrian_start_lateral;
  pedestrian.pose = Pose2(kPedestrianCrossX, start_y, -kPi * 0.5);  // 朝 -y 横穿
  pedestrian.velocity = Vec2{0.0, 0.0};
  pedestrian.speed = 0.0;
  world.objects().push_back(pedestrian);
  return world;
}

VehicleState PedestrianCrossingScenario::initialEgoState() const {
  return makeEgo(0.0, kEgoLaneY, 0.0, params_.ego_speed);
}

void PedestrianCrossingScenario::update(World& world, double time, double dt) const {
  (void)dt;
  const double start_y = kEgoLaneY + params_.pedestrian_start_lateral;
  const double end_y = kEgoLaneY - params_.pedestrian_start_lateral;

  for (RoadObject& object : world.objects()) {
    if (object.id != kPedestrianId) continue;
    if (time <= params_.pedestrian_start_time) {
      // 到达起始时刻前原地等待（"突然进入车道"的静止段）
      object.pose.y = start_y;
      object.velocity = Vec2{0.0, 0.0};
      object.speed = 0.0;
      continue;
    }
    const double travelled = params_.pedestrian_speed * (time - params_.pedestrian_start_time);
    const double y = std::max(start_y - travelled, end_y);  // 穿过车道后在对面停下
    object.pose.x = kPedestrianCrossX;
    object.pose.y = y;
    object.pose.theta = -kPi * 0.5;
    const bool moving = (start_y - travelled) > end_y;
    object.velocity = moving ? Vec2{0.0, -params_.pedestrian_speed} : Vec2{0.0, 0.0};
    object.speed = moving ? params_.pedestrian_speed : 0.0;
  }
}

// ---------------------------------------------------------------------------
// 弯道行驶
// ---------------------------------------------------------------------------

CurvedRoadScenario::CurvedRoadScenario() = default;

CurvedRoadScenario::CurvedRoadScenario(const Params& params) : params_(params) {}

ScenarioInfo CurvedRoadScenario::info() const {
  ScenarioInfo info;
  info.name = "CurvedRoad";
  info.description = "定曲率弯道，考验曲率-速度协调、路径平滑度与横向加速度控制";
  info.category = "横向";
  info.is_critical = false;
  info.duration = 20.0;
  return info;
}

World CurvedRoadScenario::buildWorld() const {
  return World::curvedRoad(2, kLaneWidth, params_.road_length, params_.curvature);
}

VehicleState CurvedRoadScenario::initialEgoState() const {
  // 右车道中心线相对参考圆弧横向偏移 -1.75m；起点处航向与参考线一致（θ=0）
  return makeEgo(0.0, kEgoLaneY, 0.0, params_.ego_speed);
}

void CurvedRoadScenario::update(World&, double, double) const {
  // 弯道场景没有其他交通参与者
}

// ---------------------------------------------------------------------------
// 场景注册表
// ---------------------------------------------------------------------------

ScenarioRegistry::ScenarioRegistry() {
  registerScenario("LeadBrake", []() { return std::make_unique<LeadBrakeScenario>(); });
  registerScenario("CutIn", []() { return std::make_unique<CutInScenario>(); });
  registerScenario("UnprotectedLeftTurn",
                   []() { return std::make_unique<UnprotectedLeftTurnScenario>(); });
  registerScenario("PedestrianCrossing",
                   []() { return std::make_unique<PedestrianCrossingScenario>(); });
  registerScenario("CurvedRoad", []() { return std::make_unique<CurvedRoadScenario>(); });
}

ScenarioRegistry& ScenarioRegistry::instance() {
  // 函数内静态变量：C++11 起保证线程安全的惰性初始化，无需显式加锁
  static ScenarioRegistry registry;
  return registry;
}

void ScenarioRegistry::registerScenario(const std::string& name, Factory factory) {
  entries_.emplace_back(name, std::move(factory));
}

std::unique_ptr<Scenario> ScenarioRegistry::create(const std::string& name) const {
  const std::string key = normalizeName(name);
  for (const auto& entry : entries_) {
    if (normalizeName(entry.first) == key) return entry.second();
  }
  return nullptr;  // 未知场景：返回空指针，由调用方决定如何报错
}

std::vector<std::string> ScenarioRegistry::names() const {
  std::vector<std::string> result;
  result.reserve(entries_.size());
  for (const auto& entry : entries_) result.push_back(entry.first);
  return result;
}

std::vector<ScenarioInfo> ScenarioRegistry::allInfo() const {
  std::vector<ScenarioInfo> result;
  result.reserve(entries_.size());
  for (const auto& entry : entries_) {
    std::unique_ptr<Scenario> scenario = entry.second();
    if (scenario) result.push_back(scenario->info());
  }
  return result;
}

}  // namespace adsim
