// =============================================================================
//  test_sim.cpp — 仿真内核单元测试
//
//  覆盖四层：
//    1) 车辆运动学模型：几何关系、精确圆弧积分、执行器/物理约束裁剪；
//    2) 世界模型：折线弧长、车道投影、道路构造、可行驶区域、物体查询；
//    3) OBB 几何：SAT 相交、精确间距、包含判定（碰撞判定的数学地基）；
//    4) 仿真引擎与场景：定步长、确定性、指标采集、安全事件、TTC 与内置场景。
//
//  断言中的数值均来自解析推导（而非"跑出来是多少写多少"），
//  例如 TTC 的 1.545s = (20 - (4.6+4.5)/2) / 10，容差只用于吸收浮点误差。
// =============================================================================
#include "TestFramework.h"

#include "adsim/sim/Scenario.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/sim/VehicleModel.h"
#include "adsim/sim/World.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace adsim;

namespace {

// ---------------------------------------------------------------------------
//  通用小工具
// ---------------------------------------------------------------------------

/// 统计结果中某类安全事件的条数
int countEvents(const SimulationResult& result, SafetyEvent event) {
  int count = 0;
  for (const SafetyEventRecord& record : result.events) {
    if (record.event == event) ++count;
  }
  return count;
}

/// 结果中第一条某类安全事件（无则返回 nullptr）
const SafetyEventRecord* firstEvent(const SimulationResult& result, SafetyEvent event) {
  for (const SafetyEventRecord& record : result.events) {
    if (record.event == event) return &record;
  }
  return nullptr;
}

/// 自车轨迹上相对于指定车道中心线的最大横向偏移绝对值
double maxLateralDeviation(const World& world, const SimulationResult& result, int lane_id) {
  double worst = 0.0;
  for (const TrajectoryPoint& point : result.ego_states) {
    const LaneProjection projection = world.projectOnLane(lane_id, point.position());
    if (projection.valid) worst = std::max(worst, std::fabs(projection.lateral));
  }
  return worst;
}

/// 自车轨迹的最大纵向位移
double maxLongitudinal(const SimulationResult& result) {
  double best = 0.0;
  for (const TrajectoryPoint& point : result.ego_states) {
    best = std::max(best, point.x);
  }
  return best;
}

/// 构造自车初始状态（静止起步的默认值 + 指定位置与速度）
VehicleState makeEgo(double x, double y, double theta, double speed) {
  VehicleState state;
  state.x = x;
  state.y = y;
  state.theta = theta;
  state.speed = speed;
  return state;
}

/// 沿折线长度取两点的距离（用于校验 Lane::length 等折线量）
double polylineLength(const std::vector<Vec2>& line) {
  double total = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    total += (line[i] - line[i - 1]).norm();
  }
  return total;
}

// ---------------------------------------------------------------------------
//  被测策略：纵向 AEB（TTC 分级制动 + 巡航）
// ---------------------------------------------------------------------------

class TestAebPolicy : public IPolicy {
 public:
  explicit TestAebPolicy(double cruise_speed) : cruise_speed_(cruise_speed) {}

  Command computeCommand(const VehicleState& ego, const World& world, double) override {
    Command command;
    command.valid = true;

    const Vec2 forward{std::cos(ego.theta), std::sin(ego.theta)};
    const Vec2 self = ego.pose().position();
    double gap = 1e9;  // 与前车保险杠间距
    for (const RoadObject& object : world.objects()) {
      const Vec2 relative = object.position() - self;
      const double longitudinal = relative.dot(forward);
      const double lateral = std::fabs(relative.cross(forward));
      if (longitudinal <= 0.0 || lateral > 2.5) continue;  // 只关心正前方同车道目标
      const double clearance = longitudinal - (4.6 + object.length) * 0.5;
      gap = std::min(gap, clearance);
    }

    const double ttc = gap < 1e8 ? gap / std::max(ego.speed, 0.1) : 1e9;
    if (ttc < 3.0) {
      command.acceleration = -6.0;  // 紧急制动
    } else if (ttc < 6.0) {
      command.acceleration = -2.0;  // 舒适制动
    } else {
      command.acceleration = cruise_speed_ - ego.speed;  // 巡航
    }
    if (ego.speed < 0.3 && command.acceleration < 0.0) command.acceleration = 0.0;
    return command;
  }

  std::string name() const override { return "TestAEB"; }

 private:
  double cruise_speed_;
};

/// 停稳并保持：直接把车刹停，之后不再给纵向指令。
/// 用途是构造"提前终止"路径 —— 自车停稳后触发 shouldTerminate 的持续静止判据。
class TestStopAndHoldPolicy : public IPolicy {
 public:
  Command computeCommand(const VehicleState& ego, const World&, double) override {
    Command command;
    command.valid = true;
    // 低速段改用一阶衰减（a = -2v，dt=0.05 下每步衰减 10%），避免"−6 m/s²
    // 一拍跨过零点"把车推到负速度：负速度会让 |v| 重新超过静止阈值，
    // 持续静止判据永远不成立（也就测不到提前终止这条路径）。
    command.acceleration = ego.speed > 1.0 ? -6.0 : -2.0 * ego.speed;
    return command;
  }

  std::string name() const override { return "TestStopAndHold"; }
};

// ---------------------------------------------------------------------------
//  被测策略：纯跟踪横向控制 + 曲率限速
// ---------------------------------------------------------------------------

class TestPurePursuitPolicy : public IPolicy {
 public:
  TestPurePursuitPolicy(double cruise_speed, double lookahead)
      : cruise_speed_(cruise_speed), lookahead_(lookahead) {}

  Command computeCommand(const VehicleState& ego, const World& world, double) override {
    Command command;
    const LaneProjection projection = world.project(ego.pose().position());
    if (!projection.valid) return command;  // valid=false：引擎保持上一步指令
    command.valid = true;

    // 前视点 → 前视偏差角 α → 曲率 κ = 2·sinα / Ld（纯跟踪公式）
    const Vec2 target = world.pointAt(projection.lane_id, projection.s + lookahead_);
    const Vec2 delta = target - ego.pose().position();
    if (delta.squaredNorm() < 1e-12) return command;
    const double alpha = normalizeAngle(std::atan2(delta.y, delta.x) - ego.theta);
    const double kappa = 2.0 * std::sin(alpha) / lookahead_;

    const VehicleModel model;
    command.steering = model.steeringFromCurvature(kappa);

    // 曲率越大允许的速度越低：a_lat = v²κ ≤ 4 m/s²
    const double curvature = std::max(std::fabs(kappa), 1e-6);
    const double speed_limit = std::sqrt(4.0 / curvature);
    const double target_speed = std::min(cruise_speed_, speed_limit);
    command.acceleration = clamp(1.2 * (target_speed - ego.speed), -4.0, 2.0);
    return command;
  }

  std::string name() const override { return "TestPurePursuit"; }

  /// 记录前视路径（供仿真结果回放规划输出）
  std::vector<Vec2> lastPlannedPath() const override { return last_path_; }
  void reset() override { last_path_.clear(); }

  /// 由外部在每步后调用，登记"本步规划路径"
  void setPlannedPath(std::vector<Vec2> path) { last_path_ = std::move(path); }

 private:
  double cruise_speed_;
  double lookahead_;
  std::vector<Vec2> last_path_;
};

/// 恒定曲率/速度回放策略：直接按纯跟踪的逆解输出指令，用于测试 stepAlongPath 语义
class TestConstantSteerPolicy : public IPolicy {
 public:
  TestConstantSteerPolicy(double curvature, double speed)
      : curvature_(curvature), speed_(speed) {}

  Command computeCommand(const VehicleState& ego, const World&, double) override {
    Command command;
    command.valid = true;
    const VehicleModel model;
    command.steering = model.steeringFromCurvature(curvature_);
    command.acceleration = clamp(speed_ - ego.speed, -3.0, 3.0);
    return command;
  }

  std::string name() const override { return "TestConstantSteer"; }

 private:
  double curvature_;
  double speed_;
};

}  // namespace

// ===========================================================================
//  一、车辆运动学模型
// ===========================================================================

ADSIM_TEST(Sim, 模型_几何关系与曲率逆解) {
  const VehicleParams params;
  const VehicleModel model(params);

  // 轴距 = 前悬 + 轴距 + 后悬
  ADSIM_CHECK_NEAR(params.length(), 0.9 + 2.7 + 1.0, 1e-12);

  // κ = tanδ / L
  for (double delta = -0.6; delta <= 0.6; delta += 0.1) {
    const double kappa = model.curvatureFromSteering(delta);
    ADSIM_CHECK_NEAR(kappa, std::tan(delta) / params.wheelbase, 1e-12);
    // 正反解互为逆运算
    ADSIM_CHECK_NEAR(model.steeringFromCurvature(kappa), delta, 1e-9);
  }
  ADSIM_CHECK_NEAR(model.curvatureFromSteering(0.0), 0.0, 1e-15);

  // 最小转弯半径 = 轴距 / tan(δ_max) ≈ 3.95 m
  // （后轴参考点的运动学自行车模型：转弯半径由轴距 L 决定，
  //   车身长度里的前/后悬不参与转向几何，否则会低估操控能力）
  const double expected = params.wheelbase / std::tan(params.max_steering);
  ADSIM_CHECK_NEAR(model.minTurningRadius(), expected, 1e-9);
  ADSIM_CHECK(model.minTurningRadius() > 3.5 && model.minTurningRadius() < 4.5);
  ADSIM_CHECK_LT(model.minTurningRadius(), params.length());

  // 速度-曲率协调上限：高速时由侧向加速度约束（v²κ ≤ 4），低速时由转角极限约束
  const double kappa_low = model.maxCurvatureAtSpeed(0.0);
  const double kappa_high = model.maxCurvatureAtSpeed(20.0);
  ADSIM_CHECK_NEAR(kappa_low, model.curvatureFromSteering(params.max_steering), 1e-12);
  ADSIM_CHECK_LT(kappa_high, kappa_low);
  ADSIM_CHECK_NEAR(20.0 * 20.0 * kappa_high, 4.0, 1e-9);
}

ADSIM_TEST(Sim, 模型_圆弧精确积分) {
  const VehicleModel model;
  const double radius = 50.0;
  const double curvature = 1.0 / radius;

  // 单步 10m 圆弧：与解析解逐项比对
  const Pose2 start(3.0, -2.0, 0.35);
  const Pose2 one = model.advanceOnArc(start, curvature, 10.0);
  ADSIM_CHECK_NEAR(one.theta, start.theta + curvature * 10.0, 1e-12);
  ADSIM_CHECK_NEAR(one.x, start.x + (std::sin(one.theta) - std::sin(start.theta)) / curvature,
                   1e-9);
  ADSIM_CHECK_NEAR(one.y, start.y - (std::cos(one.theta) - std::cos(start.theta)) / curvature,
                   1e-9);

  // 圆心到新旧位姿的距离都等于 R（圆弧上的点）
  const Vec2 center = start.position() +
                      Vec2{-std::sin(start.theta), std::cos(start.theta)} * radius;
  ADSIM_CHECK_NEAR((one.position() - center).norm(), radius, 1e-9);

  // 精确积分的可加性：1 步 10m 与 100 步 0.1m 结果一致。
  // 欧拉法在这里会累积 O(κ·ds²·n) ≈ 1e-2 m 的系统漂移，本断言正是为它设的。
  Pose2 many = start;
  for (int i = 0; i < 100; ++i) many = model.advanceOnArc(many, curvature, 0.1);
  ADSIM_CHECK_NEAR(many.x, one.x, 1e-9);
  ADSIM_CHECK_NEAR(many.y, one.y, 1e-9);
  ADSIM_CHECK_NEAR(many.theta, one.theta, 1e-9);

  // 绕整圈回到原点（闭合性）：总弧长 2πR
  Pose2 loop = start;
  const int steps = 360;
  for (int i = 0; i < steps; ++i) loop = model.advanceOnArc(loop, curvature, 2.0 * kPi * radius /
                                                                              steps);
  ADSIM_CHECK_NEAR(loop.x, start.x, 1e-9);
  ADSIM_CHECK_NEAR(loop.y, start.y, 1e-9);
}

ADSIM_TEST(Sim, 模型_积分退化与数值安全) {
  const VehicleModel model;

  // κ = 0 退化为直线
  const Pose2 straight_pose(1.0, 2.0, kPi * 0.5);
  const Pose2 moved = model.advanceOnArc(straight_pose, 0.0, 3.0);
  ADSIM_CHECK_NEAR(moved.x, 1.0, 1e-12);
  ADSIM_CHECK_NEAR(moved.y, 5.0, 1e-12);
  ADSIM_CHECK_NEAR(moved.theta, kPi * 0.5, 1e-12);

  // 极小曲率不产生 NaN/Inf，且与直线结果几乎相同
  const Pose2 tiny = model.advanceOnArc(straight_pose, 1e-12, 3.0);
  ADSIM_CHECK(std::isfinite(tiny.x) && std::isfinite(tiny.y) && std::isfinite(tiny.theta));
  ADSIM_CHECK_NEAR(tiny.y, 5.0, 1e-6);

  // 位移为 0 时位姿不变
  const Pose2 zero = model.advanceOnArc(straight_pose, 0.05, 0.0);
  ADSIM_CHECK_NEAR(zero.x, straight_pose.x, 1e-12);
  ADSIM_CHECK_NEAR(zero.y, straight_pose.y, 1e-12);
  ADSIM_CHECK_NEAR(zero.theta, straight_pose.theta, 1e-12);

  // 大曲率（不切实际的输入）也不产生 NaN
  const Pose2 sharp = model.advanceOnArc(straight_pose, 5.0, 1.0);
  ADSIM_CHECK(std::isfinite(sharp.x) && std::isfinite(sharp.y) &&
              std::isfinite(sharp.theta));
}

ADSIM_TEST(Sim, 模型_转弯半径与直行极大值) {
  const VehicleModel model;
  const VehicleParams& params = model.params();

  // 直行（转角为 0）返回有限极大值，绝不能是 inf/NaN：
  // 下游会做 radius*0、radius-radius 之类的运算，inf 会立刻污染成 NaN。
  VehicleState state = makeEgo(0.0, 0.0, 0.0, 10.0);
  const double straight_radius = model.currentTurningRadius(state);
  ADSIM_CHECK(std::isfinite(straight_radius));
  ADSIM_CHECK_GT(straight_radius, 1e6);

  // 带转角时 R = 1 / |κ|
  state.steering = 0.3;
  const double expected = 1.0 / std::fabs(std::tan(0.3) / params.wheelbase);
  ADSIM_CHECK_NEAR(model.currentTurningRadius(state), expected, 1e-9);
  ADSIM_CHECK_GT(model.currentTurningRadius(state), model.minTurningRadius());

  // 转角达到上限时等于最小转弯半径
  state.steering = params.max_steering;
  ADSIM_CHECK_NEAR(model.currentTurningRadius(state), model.minTurningRadius(), 1e-9);
}

ADSIM_TEST(Sim, 模型_加速度与加加速度限制) {
  const VehicleModel model;
  const VehicleParams& params = model.params();
  const double dt = 0.05;

  // 全油门起步：加速度按 max_jerk 逐步爬升，且始终不越界
  VehicleState state = makeEgo(0.0, 0.0, 0.0, 0.0);
  double previous_acceleration = state.acceleration;
  for (int i = 0; i < 200; ++i) {
    const VehicleState next = model.step(state, 1.0, 0.0, 0.0, dt);
    ADSIM_CHECK(next.acceleration <= params.max_acceleration + 1e-9);
    ADSIM_CHECK(next.acceleration >= params.max_deceleration - 1e-9);
    // 单步加速度变化受 max_jerk 限制（这正是"舒适性"约束）
    ADSIM_CHECK(std::fabs(next.acceleration - previous_acceleration) <=
                params.max_jerk * dt + 1e-9);
    previous_acceleration = next.acceleration;
    state = next;
  }
  ADSIM_CHECK_NEAR(state.acceleration, params.max_acceleration, 1e-6);
  // 速度 = 加速度积分：先按 max_jerk 斜坡爬升 12 步，之后保持满加速度。
  // 不能简单写成 a·t —— 忽略 jerk 爬坡阶段会高估约 0.8 m/s。
  double expected_speed = 0.0;
  for (int i = 0; i < 200; ++i) {
    expected_speed += std::min(params.max_acceleration, (i + 1) * params.max_jerk * dt) * dt;
  }
  ADSIM_CHECK_NEAR(state.speed, expected_speed, 1e-6);

  // 全制动：减速度不超过物理上限，加速度变化率受 max_jerk 限制，
  // 直到速度触到倒车下限（min_speed）后被硬限幅。
  bool reached_full_braking = false;
  bool speed_saturated = false;
  for (int i = 0; i < 400; ++i) {
    const bool already_saturated = state.speed <= params.min_speed + 1e-12;
    const VehicleState next = model.step(state, 0.0, 1.0, 0.0, dt);
    ADSIM_CHECK(next.acceleration >= params.max_deceleration - 1e-9);
    // 速度被限幅的步：实际速度变化小于请求值，实测加速度随之变小
    const bool speed_limited = next.speed <= params.min_speed + 1e-12 ||
                               next.speed >= params.max_speed - 1e-12;
    if (!speed_limited) {
      ADSIM_CHECK(std::fabs(next.acceleration - previous_acceleration) <=
                  params.max_jerk * dt + 1e-9);
    } else if (already_saturated) {
      // 限幅器边界：速度已被夹住、不再变化，实测加速度归零（状态仍然自洽）。
      // 加速度在此处会一次性从 -max_deceleration 跳到 0 —— 这是硬限幅的边界
      // 行为而非模型缺陷，因此该步不再要求 jerk 连续。
      ADSIM_CHECK_NEAR(next.acceleration, 0.0, 1e-12);
      ADSIM_CHECK_NEAR(next.speed, params.min_speed, 1e-12);
      speed_saturated = true;
      break;
    }
    if (next.acceleration <= params.max_deceleration + 1e-9) reached_full_braking = true;
    previous_acceleration = next.acceleration;
    state = next;
  }
  ADSIM_CHECK(reached_full_braking);
  ADSIM_CHECK(speed_saturated);
  ADSIM_CHECK_NEAR(state.speed, params.min_speed, 1e-12);  // 倒车下限

  // 目标加速度远超能力时自动饱和（stepByAcceleration 的裁剪）
  VehicleState from_rest = makeEgo(0.0, 0.0, 0.0, 0.0);
  const VehicleState saturated = model.stepByAcceleration(from_rest, 100.0, 0.0, dt);
  ADSIM_CHECK_NEAR(saturated.acceleration, params.max_jerk * dt, 1e-9);
  const VehicleState hard_brake = model.stepByAcceleration(from_rest, -100.0, 0.0, dt);
  ADSIM_CHECK_NEAR(hard_brake.acceleration, -params.max_jerk * dt, 1e-9);
}

ADSIM_TEST(Sim, 模型_转角速率限制) {
  const VehicleModel model;
  const VehicleParams& params = model.params();
  const double dt = 0.05;

  // 建模约定：state.steering 是"当前实际转角"，入参 steering 是"目标转角"，
  // 每步最多变化 max_steering_rate·dt（把前轮执行器建模为一阶速率受限环节）。
  VehicleState state = makeEgo(0.0, 0.0, 0.0, 5.0);
  const VehicleState first = model.step(state, 0.0, 0.0, params.max_steering, dt);
  ADSIM_CHECK_NEAR(first.steering, params.max_steering_rate * dt, 1e-12);

  // 连续给满目标：转角单调逼近目标值
  double previous = first.steering;
  state = first;
  for (int i = 0; i < 100; ++i) {
    const VehicleState next = model.step(state, 0.0, 0.0, params.max_steering, dt);
    ADSIM_CHECK(next.steering >= previous - 1e-12);
    ADSIM_CHECK(next.steering <= params.max_steering + 1e-12);
    previous = next.steering;
    state = next;
  }
  ADSIM_CHECK_NEAR(state.steering, params.max_steering, 1e-9);

  // 回正：从满转角回到 0
  state.steering = params.max_steering;
  const VehicleState straighten = model.step(state, 0.0, 0.0, 0.0, dt);
  ADSIM_CHECK_NEAR(straighten.steering, params.max_steering - params.max_steering_rate * dt,
                   1e-12);

  // 目标超限时按物理上限裁剪
  state = makeEgo(0.0, 0.0, 0.0, 5.0);
  state.steering = params.max_steering;
  const VehicleState clipped = model.step(state, 0.0, 0.0, 100.0, dt);
  ADSIM_CHECK_NEAR(clipped.steering, params.max_steering, 1e-12);
}

ADSIM_TEST(Sim, 模型_输出状态可行性) {
  const VehicleModel model;
  const double throttles[] = {-1.0, 0.0, 0.5, 1.0, 5.0};
  const double brakes[] = {-1.0, 0.0, 0.5, 1.0, 5.0};
  const double steerings[] = {-10.0, -0.6, -0.1, 0.0, 0.1, 0.6, 10.0};

  VehicleState state = makeEgo(0.0, 0.0, 0.0, 7.0);
  for (double throttle : throttles) {
    for (double brake : brakes) {
      for (double steering : steerings) {
        const VehicleState next = model.step(state, throttle, brake, steering, 0.05);
        // 引擎/规划都依赖这一条：模型输出必须落在车辆约束内
        ADSIM_CHECK(model.isStateFeasible(next));
        state = next;
      }
    }
  }

  // 非法步长不推进（也不产生 NaN）
  const VehicleState frozen = model.step(state, 1.0, 1.0, 0.3, 0.0);
  ADSIM_CHECK_NEAR(frozen.x, state.x, 1e-12);
  ADSIM_CHECK_NEAR(frozen.speed, state.speed, 1e-12);
  const VehicleState negative = model.step(state, 1.0, 1.0, 0.3, -0.05);
  ADSIM_CHECK_NEAR(negative.x, state.x, 1e-12);

  // 输入本身含 NaN/Inf 时，状态必须被判定为不可行（而不是静默传播）
  VehicleState broken = makeEgo(0.0, 0.0, 0.0, 1.0);
  broken.speed = std::nan("");
  ADSIM_CHECK(!model.isStateFeasible(broken));
  broken = makeEgo(0.0, 0.0, 0.0, 1.0);
  broken.x = std::numeric_limits<double>::infinity();
  ADSIM_CHECK(!model.isStateFeasible(broken));
}

ADSIM_TEST(Sim, 模型_速度与加速度自洽) {
  const VehicleModel model;
  const double dt = 0.05;

  // 报告里的 acceleration 必须等于 (v' - v)/dt：
  // 否则下游由"速度差/步长"反算的 jerk 会出现虚假尖峰。
  VehicleState state = makeEgo(0.0, 0.0, 0.0, 4.0);
  for (int i = 0; i < 120; ++i) {
    const double previous_speed = state.speed;
    const VehicleState next = model.stepByAcceleration(state, (i % 40 < 20) ? 2.0 : -3.0, 0.05,
                                                       dt);
    ADSIM_CHECK_NEAR(next.acceleration, (next.speed - previous_speed) / dt, 1e-9);
    state = next;
  }
}

ADSIM_TEST(Sim, 模型_轨迹回放) {
  const VehicleModel model;
  const double dt = 0.05;
  const double curvature = 0.02;  // R = 50m

  const double target_steering = model.steeringFromCurvature(curvature);
  VehicleState state = makeEgo(0.0, 0.0, 0.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    const double previous_steering = state.steering;
    // 目标速度 12 m/s：一阶跟踪 + 加速度/jerk 约束，逐步逼近且不超调
    state = model.stepAlongPath(state, curvature, 12.0, dt);
    // 转角以执行器速率上限逼近目标值（state.steering 是实际值，不是目标值）
    ADSIM_CHECK(state.steering <= target_steering + 1e-9);
    ADSIM_CHECK(std::fabs(state.steering - previous_steering) <=
                model.params().max_steering_rate * dt + 1e-9);
    ADSIM_CHECK(state.speed <= 12.0 + 1e-9);  // 绝不过冲（回放轨迹的平滑性前提）
  }
  ADSIM_CHECK_NEAR(state.steering, target_steering, 1e-9);
  // 5s 后已非常接近目标速度；再跑 5s 收敛到 1e-3 以内
  ADSIM_CHECK_NEAR(state.speed, 12.0, 0.15);
  for (int i = 0; i < 100; ++i) {
    state = model.stepAlongPath(state, curvature, 12.0, dt);
  }
  ADSIM_CHECK_NEAR(state.speed, 12.0, 1e-3);
  // 定曲率回放：角速度与曲率、速度严格自洽
  ADSIM_CHECK_NEAR(state.yaw_rate, state.speed * curvature, 1e-9);

  // 轨迹点落在半径 50m 的圆上（左转弯心在起点左侧法向 R 处）。
  // 容差 1mm：前几步转角还在 0.5rad/s 的速率限制下爬升，曲率尚未到位，
  // 会让整段圆弧（绕行 2.4 圈）相对理想圆心平移约 5e-4 m。
  const Vec2 center = Vec2{-std::sin(0.0), std::cos(0.0)} * (1.0 / curvature);
  ADSIM_CHECK_NEAR((state.pose().position() - center).norm(), 1.0 / curvature, 1e-3);

  // 目标速度低于当前速度时平滑减速（不出现阶跃）：
  // 单步加速度变化受 max_jerk 限制，因此首步只会从 ~0 降到 -jerk·dt
  const VehicleState slow = model.stepAlongPath(state, curvature, 6.0, dt);
  ADSIM_CHECK_LT(slow.acceleration, 0.0);
  ADSIM_CHECK_NEAR(slow.acceleration, -model.params().max_jerk * dt, 1e-3);
  ADSIM_CHECK_GT(slow.speed, state.speed - model.params().max_jerk * dt * dt);
}

ADSIM_TEST(Sim, 模型_参数校验) {
  ADSIM_CHECK(VehicleModel(VehicleParams{}).validate().empty());

  VehicleParams bad_wheelbase;
  bad_wheelbase.wheelbase = 0.0;
  ADSIM_CHECK(!VehicleModel(bad_wheelbase).validate().empty());

  VehicleParams bad_deceleration;
  bad_deceleration.max_deceleration = 2.0;  // 减速度必须为负
  ADSIM_CHECK(!VehicleModel(bad_deceleration).validate().empty());

  VehicleParams bad_steering;
  bad_steering.max_steering = kPi * 0.5;  // 必须落在 (0, pi/2)
  ADSIM_CHECK(!VehicleModel(bad_steering).validate().empty());

  VehicleParams bad_jerk;
  bad_jerk.max_jerk = 0.0;
  ADSIM_CHECK(!VehicleModel(bad_jerk).validate().empty());
}

// ===========================================================================
//  二、世界模型
// ===========================================================================

ADSIM_TEST(Sim, 世界_折线弧长与插值) {
  // 折线 (0,0) -> (3,0) -> (3,4)，总长 7
  const std::vector<Vec2> line{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  const std::vector<double> arc = laneArcLength(line);
  ADSIM_CHECK_EQ(arc.size(), line.size());
  ADSIM_CHECK_NEAR(arc[0], 0.0, 1e-12);
  ADSIM_CHECK_NEAR(arc[1], 3.0, 1e-12);
  ADSIM_CHECK_NEAR(arc[2], 7.0, 1e-12);

  // 弧长插值
  const Vec2 at_1_5 = interpolateAlong(line, arc, 1.5);
  ADSIM_CHECK_NEAR(at_1_5.x, 1.5, 1e-12);
  ADSIM_CHECK_NEAR(at_1_5.y, 0.0, 1e-12);
  const Vec2 at_5 = interpolateAlong(line, arc, 5.0);
  ADSIM_CHECK_NEAR(at_5.x, 3.0, 1e-12);
  ADSIM_CHECK_NEAR(at_5.y, 2.0, 1e-12);
  const Vec2 at_0 = interpolateAlong(line, arc, 0.0);
  ADSIM_CHECK_NEAR(at_0.x, 0.0, 1e-12);
  const Vec2 clamped = interpolateAlong(line, arc, 100.0);  // 越界裁剪到端点
  ADSIM_CHECK_NEAR(clamped.x, 3.0, 1e-12);
  ADSIM_CHECK_NEAR(clamped.y, 4.0, 1e-12);

  // 航向插值
  ADSIM_CHECK_NEAR(headingAlong(line, arc, 1.5), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(headingAlong(line, arc, 5.0), kPi * 0.5, 1e-12);

  // 退化输入：空折线、单点、长度不匹配
  ADSIM_CHECK_NEAR(interpolateAlong({}, {}, 1.0).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(interpolateAlong({Vec2{1.0, 1.0}}, {0.0}, 3.0).x, 1.0, 1e-12);
  ADSIM_CHECK_NEAR(headingAlong({Vec2{1.0, 1.0}}, {0.0}, 0.0), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(headingAlong(line, laneArcLength(line), 1.5), 0.0, 1e-12);
  // 所有点重合的退化折线
  const std::vector<Vec2> degenerate{{2.0, 2.0}, {2.0, 2.0}};
  const std::vector<double> degenerate_arc = laneArcLength(degenerate);
  const Vec2 degenerate_point = interpolateAlong(degenerate, degenerate_arc, 1.0);
  ADSIM_CHECK_NEAR(degenerate_point.x, 2.0, 1e-12);
}

ADSIM_TEST(Sim, 世界_车道长度与拓扑) {
  const World world = World::straightRoad(3, 3.5, 120.0);
  ADSIM_CHECK_EQ(static_cast<int>(world.laneCount()), 3);

  // 车道自左向右编号 0..2；沿 +x 行驶时左侧为 +y，故 0 号车道 y 最大
  const Lane* left = world.findLane(0);
  const Lane* middle = world.findLane(1);
  const Lane* right = world.findLane(2);
  ADSIM_CHECK(left != nullptr && middle != nullptr && right != nullptr);
  ADSIM_CHECK_GT(left->centerline.front().y, middle->centerline.front().y);
  ADSIM_CHECK_GT(middle->centerline.front().y, right->centerline.front().y);
  ADSIM_CHECK_NEAR(middle->centerline.front().y, 0.0, 1e-12);

  // 车道长度 = 折线长度 = 道路长度
  for (const Lane& lane : world.lanes()) {
    ADSIM_CHECK_NEAR(lane.length(), 120.0, 1e-9);
    ADSIM_CHECK_NEAR(lane.length(), polylineLength(lane.centerline), 1e-9);
    ADSIM_CHECK(!lane.is_junction);
    ADSIM_CHECK_NEAR(lane.width, 3.5, 1e-12);
    ADSIM_CHECK_NEAR(lane.speed_limit, 13.9, 1e-12);
  }

  // 相邻车道互指：0 号左侧无车道、右侧是 1 号，依此类推
  ADSIM_CHECK_EQ(left->left_lane_id, -1);
  ADSIM_CHECK_EQ(left->right_lane_id, 1);
  ADSIM_CHECK_EQ(middle->left_lane_id, 0);
  ADSIM_CHECK_EQ(middle->right_lane_id, 2);
  ADSIM_CHECK_EQ(right->left_lane_id, 1);
  ADSIM_CHECK_EQ(right->right_lane_id, -1);

  // 不存在的车道
  ADSIM_CHECK(world.findLane(99) == nullptr);

  // 非法参数返回空世界，不抛异常
  ADSIM_CHECK_EQ(static_cast<int>(World::straightRoad(0, 3.5, 100.0).laneCount()), 0);
  ADSIM_CHECK_EQ(static_cast<int>(World::straightRoad(2, -1.0, 100.0).laneCount()), 0);
  ADSIM_CHECK_EQ(static_cast<int>(World::straightRoad(2, 3.5, 0.0).laneCount()), 0);
}

ADSIM_TEST(Sim, 世界_投影_弧长与横向偏移) {
  const World world = World::straightRoad(2, 3.5, 200.0);

  // 车道中心线上：横向偏移为 0，弧长等于纵向坐标
  const LaneProjection on_line = world.project(Vec2(50.0, -1.75));
  ADSIM_CHECK(on_line.valid);
  ADSIM_CHECK_EQ(on_line.lane_id, 1);
  ADSIM_CHECK_NEAR(on_line.s, 50.0, 1e-9);
  ADSIM_CHECK_NEAR(on_line.lateral, 0.0, 1e-9);
  ADSIM_CHECK_NEAR(on_line.heading, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(on_line.curvature, 0.0, 1e-9);

  // 左侧为正：目标点在车道中心线左侧（+y 方向）
  const LaneProjection left = world.project(Vec2(50.0, -1.0));
  ADSIM_CHECK_EQ(left.lane_id, 1);
  ADSIM_CHECK_NEAR(left.lateral, 0.75, 1e-9);
  // 右侧为负
  const LaneProjection right = world.project(Vec2(50.0, -2.5));
  ADSIM_CHECK_EQ(right.lane_id, 1);
  ADSIM_CHECK_NEAR(right.lateral, -0.75, 1e-9);

  // 投影到最近车道：y=+2.5 距 0 号车道中心线 0.75m，距 1 号车道 4.25m
  const LaneProjection nearest = world.project(Vec2(50.0, 2.5));
  ADSIM_CHECK_EQ(nearest.lane_id, 0);
  ADSIM_CHECK_NEAR(nearest.lateral, 0.75, 1e-9);

  // projectOnLane 只投影到指定车道
  const LaneProjection forced = world.projectOnLane(0, Vec2(50.0, 2.5));
  ADSIM_CHECK_EQ(forced.lane_id, 0);
  ADSIM_CHECK_NEAR(forced.lateral, 0.75, 1e-9);
  ADSIM_CHECK(!world.projectOnLane(42, Vec2(50.0, 2.5)).valid);

  // 空世界：无效投影
  ADSIM_CHECK(!World{}.project(Vec2(1.0, 1.0)).valid);
}

ADSIM_TEST(Sim, 世界_投影_最近车道与端点边界) {
  const World world = World::straightRoad(2, 3.5, 100.0);

  // 纵向越过车道终点后，必须切换到真正的后继车道（而不是把 s 卡在端点、
  // 横向偏移退化为 0 后仍判为原车道）——路口场景里这会直接导致限速、
  // 可行驶区域判断出错。
  const LaneProjection before_end = world.project(Vec2(99.0, -1.75));
  ADSIM_CHECK_EQ(before_end.lane_id, 1);
  ADSIM_CHECK_NEAR(before_end.s, 99.0, 1e-9);

  // 越过终点：仍应选最近的（本世界只有这一条路，因此横向偏移退化为 0，
  // 但 isOnRoad 必须借助"到折线的距离"判为驶出路面）
  const Vec2 beyond(140.0, -1.75);
  ADSIM_CHECK(!world.isOnRoad(beyond));
  ADSIM_CHECK_NEAR(world.distanceToRoadEdge(beyond), 40.0 - 1.75, 1e-6);

  // 起点之前同理
  const Vec2 before_start(-30.0, -1.75);
  ADSIM_CHECK(!world.isOnRoad(before_start));
  ADSIM_CHECK_NEAR(world.distanceToRoadEdge(before_start), 30.0 - 1.75, 1e-6);

  // 十字路口：西侧进口道之后的点应投影到路口内的贯通车道，再之后是东侧出口道
  const World junction = World::intersection(60.0, 3.5);
  const LaneProjection in_junction = junction.project(Vec2(0.0, -1.75));
  const Lane* junction_lane = junction.findLane(in_junction.lane_id);
  ADSIM_CHECK(junction_lane != nullptr);
  ADSIM_CHECK(junction_lane->is_junction);
  const LaneProjection after_junction = junction.project(Vec2(20.0, -1.75));
  const Lane* exit_lane = junction.findLane(after_junction.lane_id);
  ADSIM_CHECK(exit_lane != nullptr);
  ADSIM_CHECK(!exit_lane->is_junction);
  ADSIM_CHECK_NEAR(exit_lane->centerline.front().x, 3.5, 1e-9);
}

ADSIM_TEST(Sim, 世界_点位姿与端点裁剪) {
  const World world = World::straightRoad(2, 3.5, 100.0);

  // s 被裁剪到 [0, length]
  const Vec2 start = world.pointAt(0, -10.0);
  ADSIM_CHECK_NEAR(start.x, 0.0, 1e-9);
  const Vec2 end = world.pointAt(0, 500.0);
  ADSIM_CHECK_NEAR(end.x, 100.0, 1e-9);
  ADSIM_CHECK_NEAR(end.y, 1.75, 1e-9);

  const Vec2 mid = world.pointAt(1, 25.0);
  ADSIM_CHECK_NEAR(mid.x, 25.0, 1e-9);
  ADSIM_CHECK_NEAR(mid.y, -1.75, 1e-9);

  const Pose2 pose = world.poseAt(1, 25.0);
  ADSIM_CHECK_NEAR(pose.x, 25.0, 1e-9);
  ADSIM_CHECK_NEAR(pose.y, -1.75, 1e-9);
  ADSIM_CHECK_NEAR(pose.theta, 0.0, 1e-12);

  // 未知车道返回默认值，不崩溃
  ADSIM_CHECK_NEAR(world.pointAt(42, 1.0).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(world.poseAt(42, 1.0).x, 0.0, 1e-12);
}

ADSIM_TEST(Sim, 世界_车道坐标系_前视航向误差) {
  const World world = World::straightRoad(2, 3.5, 100.0);
  double s = 0.0;
  double lateral = 0.0;
  double heading_error = 0.0;

  // 点在中心线上：前视连线与该处切线重合 ⇒ 航向误差为 0
  ADSIM_CHECK(world.laneFrame(1, Vec2(30.0, -1.75), s, lateral, heading_error));
  ADSIM_CHECK_NEAR(s, 30.0, 1e-9);
  ADSIM_CHECK_NEAR(lateral, 0.0, 1e-9);
  ADSIM_CHECK_NEAR(heading_error, 0.0, 1e-9);

  // 点偏左（+y）：前视连线向右下方指 ⇒ 航向误差为负（提示向右修正），
  // 与纯跟踪控制器的转向修正方向一致
  ADSIM_CHECK(world.laneFrame(1, Vec2(30.0, -0.75), s, lateral, heading_error));
  ADSIM_CHECK_NEAR(lateral, 1.0, 1e-9);
  ADSIM_CHECK_LT(heading_error, 0.0);
  // 点偏右：符号相反
  ADSIM_CHECK(world.laneFrame(1, Vec2(30.0, -2.75), s, lateral, heading_error));
  ADSIM_CHECK_GT(heading_error, 0.0);

  // 未知车道
  ADSIM_CHECK(!world.laneFrame(42, Vec2(30.0, 0.0), s, lateral, heading_error));
}

ADSIM_TEST(Sim, 世界_弯道几何与曲率) {
  const double curvature = 0.02;  // R = 50m
  const World world = World::curvedRoad(2, 3.5, 150.0, curvature);

  // 0 号车道在左（+1.75 偏移），1 号车道在右（-1.75 偏移）；
  // 平行曲线的曲率 κ' = κ/(1 - κ·offset)
  const Lane* left = world.findLane(0);
  const Lane* right = world.findLane(1);
  ADSIM_CHECK(left != nullptr && right != nullptr);
  const double left_curvature = curvature / (1.0 - curvature * 1.75);
  const double right_curvature = curvature / (1.0 + curvature * 1.75);

  const LaneProjection left_projection = world.projectOnLane(0, left->centerline.back());
  ADSIM_CHECK_NEAR(left_projection.curvature, left_curvature, 1e-4);
  const LaneProjection right_projection = world.projectOnLane(1, right->centerline.back());
  ADSIM_CHECK_NEAR(right_projection.curvature, right_curvature, 1e-4);
  // 左转弯时左车道在弯道内侧：半径更小 ⇒ 曲率更大（这条差异决定了
  // "按车道限速"与"按曲率限速"必须分开算，差约 7%）
  ADSIM_CHECK_GT(left_projection.curvature, right_projection.curvature);
  ADSIM_CHECK_GT(left_projection.curvature, curvature);
  ADSIM_CHECK_LT(right_projection.curvature, curvature);

  // 直线道路上曲率估计为 0
  const World straight = World::curvedRoad(1, 3.5, 100.0, 0.0);
  ADSIM_CHECK_NEAR(straight.project(Vec2(50.0, 0.0)).curvature, 0.0, 1e-9);

  // 弯道确实是弯的：终点航向 ≈ κ·s，且横向明显偏离起点切线
  const Pose2 end_pose = world.poseAt(0, 100.0);
  ADSIM_CHECK_GT(std::fabs(end_pose.theta), 1.5);  // 100m × 0.02 = 2 rad
  ADSIM_CHECK_GT(std::fabs(end_pose.y), 20.0);

  // 弯道上的点能正确投影（横向偏移 ≈ 0）
  const Vec2 on_curve = world.pointAt(1, 40.0);
  const LaneProjection projection = world.project(on_curve);
  ADSIM_CHECK(projection.valid);
  ADSIM_CHECK_NEAR(projection.lateral, 0.0, 1e-6);
  ADSIM_CHECK_NEAR(projection.s, 40.0, 1e-6);
}

ADSIM_TEST(Sim, 世界_十字路口结构) {
  const double lane_width = 3.5;
  const World world = World::intersection(60.0, lane_width);

  ADSIM_CHECK_EQ(static_cast<int>(world.laneCount()), 12);

  // 四条路 × 两个方向 = 8 条路段车道 + 4 条路口贯通车道
  int junction_count = 0;
  int arm_count = 0;
  for (const Lane& lane : world.lanes()) {
    if (lane.is_junction) {
      ++junction_count;
      // 路口内限速更低；路口贯通车道长度 = 路口边长 = 2 × 车道宽
      // （对向各一条车道，路口横向宽度即两条车道）
      ADSIM_CHECK_NEAR(lane.speed_limit, 8.3, 1e-9);
      ADSIM_CHECK_NEAR(lane.length(), 2.0 * lane_width, 1e-9);
      ADSIM_CHECK_GT(lane.predecessor_id, -1);
      ADSIM_CHECK_GT(lane.successor_id, -1);
    } else {
      ++arm_count;
      ADSIM_CHECK_NEAR(lane.speed_limit, 13.9, 1e-9);
    }
  }
  ADSIM_CHECK_EQ(junction_count, 4);
  ADSIM_CHECK_EQ(arm_count, 8);

  // 路口车道与前后路段车道首尾相接（前驱的终点 == 路口的起点）
  for (const Lane& lane : world.lanes()) {
    if (!lane.is_junction) continue;
    const Lane* predecessor = world.findLane(lane.predecessor_id);
    const Lane* successor = world.findLane(lane.successor_id);
    ADSIM_CHECK(predecessor != nullptr && successor != nullptr);
    ADSIM_CHECK_NEAR((predecessor->centerline.back() - lane.centerline.front()).norm(), 0.0,
                     1e-9);
    ADSIM_CHECK_NEAR((successor->centerline.front() - lane.centerline.back()).norm(), 0.0,
                     1e-9);
  }

  // 东西向贯通车道的中心线在 y = ∓1.75 上
  const Lane* eastbound = world.findLane(1);
  ADSIM_CHECK(eastbound != nullptr);
  ADSIM_CHECK_NEAR(eastbound->centerline.front().y, -1.75, 1e-9);
  ADSIM_CHECK_NEAR(eastbound->centerline.front().x, -lane_width, 1e-9);
  ADSIM_CHECK_NEAR(eastbound->centerline.back().x, lane_width, 1e-9);

  // 非法参数
  ADSIM_CHECK_EQ(static_cast<int>(World::intersection(-1.0, 3.5).laneCount()), 0);
  ADSIM_CHECK_EQ(static_cast<int>(World::intersection(60.0, 0.0).laneCount()), 0);
}

ADSIM_TEST(Sim, 世界_可行驶区域判定) {
  const World world = World::straightRoad(2, 3.5, 100.0);

  // 车道中心线：在路内
  ADSIM_CHECK(world.isOnRoad(Vec2(50.0, -1.75)));
  // 恰好在车道边界上：算在路内（容差 1e-9）
  ADSIM_CHECK(world.isOnRoad(Vec2(50.0, -1.75 + 1.75)));
  // 略微压线：仍在相邻车道内 ⇒ 在路内
  ADSIM_CHECK(world.isOnRoad(Vec2(50.0, 0.5)));
  // 超出路肩：在路外
  ADSIM_CHECK(!world.isOnRoad(Vec2(50.0, 4.0)));
  ADSIM_CHECK(!world.isOnRoad(Vec2(50.0, -4.0)));

  // distanceToRoadEdge：路内为负，路外为正
  ADSIM_CHECK_LT(world.distanceToRoadEdge(Vec2(50.0, -1.75)), 0.0);
  ADSIM_CHECK_NEAR(world.distanceToRoadEdge(Vec2(50.0, -3.6)), 0.1, 1e-9);
  ADSIM_CHECK_NEAR(world.distanceToRoadEdge(Vec2(50.0, 3.6)), 0.1, 1e-9);

  // 空世界：不在任何路上
  ADSIM_CHECK(!World{}.isOnRoad(Vec2(0.0, 0.0)));
  ADSIM_CHECK_GT(World{}.distanceToRoadEdge(Vec2(0.0, 0.0)), 1e9);
}

ADSIM_TEST(Sim, 世界_物体与障碍查询) {
  World world = World::straightRoad(2, 3.5, 200.0);

  RoadObject near_object;
  near_object.id = 7;
  near_object.pose = Pose2(50.0, -1.75, 0.0);
  RoadObject far_object;
  far_object.id = 8;
  far_object.pose = Pose2(60.0, -1.75, 0.0);
  world.objects().push_back(near_object);
  world.objects().push_back(far_object);

  // 索引按距离升序返回
  const std::vector<std::size_t> all = world.queryObjects(Vec2(50.0, -1.75), 100.0);
  ADSIM_CHECK_EQ(static_cast<int>(all.size()), 2);
  ADSIM_CHECK_EQ(static_cast<int>(all[0]), 0);  // 7 号更近
  ADSIM_CHECK_EQ(static_cast<int>(all[1]), 1);

  // 半径过滤：只返回半径内的物体
  const std::vector<std::size_t> close = world.queryObjects(Vec2(50.0, -1.75), 5.0);
  ADSIM_CHECK_EQ(static_cast<int>(close.size()), 1);
  ADSIM_CHECK_EQ(static_cast<int>(close[0]), 0);

  // 半径外的物体不返回
  ADSIM_CHECK_EQ(static_cast<int>(world.queryObjects(Vec2(0.0, 0.0), 1.0).size()), 0);

  // 静态障碍查询
  world.addObstacle(Obb2(Pose2(52.0, 2.0, 0.0), 2.0, 2.0));
  world.addObstacle(Obb2(Pose2(150.0, 2.0, 0.0), 2.0, 2.0));
  const std::vector<std::size_t> obstacles = world.queryObstacles(Vec2(52.0, 2.0), 3.0);
  ADSIM_CHECK_EQ(static_cast<int>(obstacles.size()), 1);
  ADSIM_CHECK_EQ(static_cast<int>(obstacles[0]), 0);
  // 查询点落入障碍内部时距离为 0
  const std::vector<std::size_t> inside = world.queryObstacles(Vec2(52.0, 2.0), 0.0);
  ADSIM_CHECK_EQ(static_cast<int>(inside.size()), 1);
}

ADSIM_TEST(Sim, 世界_物体碰撞) {
  RoadObject a;
  a.pose = Pose2(0.0, 0.0, 0.0);

  RoadObject b;
  b.pose = Pose2(1.0, 0.0, 0.0);  // 车长 4.5，必然重叠
  ADSIM_CHECK(a.collidesWith(b));
  ADSIM_CHECK(b.collidesWith(a));

  b.pose = Pose2(20.0, 0.0, 0.0);
  ADSIM_CHECK(!a.collidesWith(b));

  // 纵向贴边（车心距 = 车长）：算碰撞
  b.pose = Pose2(4.5, 0.0, 0.0);
  ADSIM_CHECK(a.collidesWith(b));

  b.pose = Pose2(4.6, 0.0, 0.0);
  ADSIM_CHECK(!a.collidesWith(b));

  // 横向错开一个车身：不算碰撞
  b.pose = Pose2(0.0, 7.0, kPi * 0.5);
  ADSIM_CHECK(!a.collidesWith(b));
}

// ===========================================================================
//  三、OBB 几何（SAT）
// ===========================================================================

ADSIM_TEST(Sim, OBB_分离轴_基本情形) {
  const Obb2 box = Obb2(Pose2(0.0, 0.0, 0.0), 4.0, 2.0);

  // 完全分离
  ADSIM_CHECK(!obbIntersect(box, Obb2(Pose2(10.0, 0.0, 0.0), 4.0, 2.0)));
  ADSIM_CHECK(!obbIntersect(box, Obb2(Pose2(0.0, 5.0, 0.0), 4.0, 2.0)));
  // 重叠
  ADSIM_CHECK(obbIntersect(box, Obb2(Pose2(3.0, 0.0, 0.0), 4.0, 2.0)));
  // 恰好贴边（车心距 4.0 = 半长之和）：约定为相交
  ADSIM_CHECK(obbIntersect(box, Obb2(Pose2(4.0, 0.0, 0.0), 4.0, 2.0)));
  ADSIM_CHECK(obbIntersect(box, Obb2(Pose2(0.0, 2.0, 0.0), 4.0, 2.0)));
  // 差一点点分离
  ADSIM_CHECK(!obbIntersect(box, Obb2(Pose2(4.0001, 0.0, 0.0), 4.0, 2.0)));

  // 正交十字（中心重合）：相交
  ADSIM_CHECK(obbIntersect(box, Obb2(Pose2(0.0, 0.0, kPi * 0.5), 4.0, 2.0)));
  // 正交后错开：旋转 90° 的盒子长边沿 y 轴（±2），与 box 的 y 半宽 1 相加为 3，
  // 因此沿 y 错开 3.0 恰好相切（相交）、3.5 才真正分离。
  // 只看 AABB 会算成 2+1=3 —— 这里两者的结论恰好一致，因为分离轴正是 y 轴。
  ADSIM_CHECK(obbIntersect(box, Obb2(Pose2(0.0, 3.0, kPi * 0.5), 4.0, 2.0)));
  ADSIM_CHECK(!obbIntersect(box, Obb2(Pose2(0.0, 3.5, kPi * 0.5), 4.0, 2.0)));
}

ADSIM_TEST(Sim, OBB_分离轴_斜置矩形) {
  // 两个平行的细长矩形，沿自身横向错开 3m：
  // 它们的 AABB 明显重叠（朴素 AABB 判定会误报碰撞），
  // 但沿宽度方向（分离轴）间隙为 3 - 0.5 - 0.5 = 2m，必须判为不相交。
  const double diagonal = kPi * 0.25;
  const Vec2 width_axis{-std::sin(diagonal), std::cos(diagonal)};
  const Vec2 offset = width_axis * 3.0;

  const Obb2 a(Pose2(0.0, 0.0, diagonal), 10.0, 1.0);
  const Obb2 b(Pose2(offset.x, offset.y, diagonal), 10.0, 1.0);

  // 前置条件：AABB 重叠（否则这个用例证明不了 SAT 的必要性）
  ADSIM_CHECK(a.aabb().overlaps(b.aabb()));
  ADSIM_CHECK(!obbIntersect(a, b));

  // 错开到 1.0m：横向间隙为 0，恰好相切 ⇒ 相交
  const Vec2 touching = width_axis * 1.0;
  const Obb2 c(Pose2(touching.x, touching.y, diagonal), 10.0, 1.0);
  ADSIM_CHECK(obbIntersect(a, c));

  // 错开到 0.9m：重叠 ⇒ 相交
  const Vec2 overlapping = width_axis * 0.9;
  const Obb2 d(Pose2(overlapping.x, overlapping.y, diagonal), 10.0, 1.0);
  ADSIM_CHECK(obbIntersect(a, d));

  // 45° 斜置矩形 vs 轴对齐正方形：斜置后沿 x 轴的"投影半宽"变成
  // 1·cos45 + 0.25·sin45 = 0.884（而不是半长 1.0），分离阈值随之变成 2.884。
  // 只按"车心距 vs 半长之和"估算必然判错。
  const Obb2 axis_aligned(Pose2(0.0, 0.0, 0.0), 4.0, 4.0);
  ADSIM_CHECK(obbIntersect(axis_aligned, Obb2(Pose2(3.0, 0.0, diagonal), 4.0, 4.0)));
  ADSIM_CHECK(obbIntersect(axis_aligned, Obb2(Pose2(2.5, 0.0, diagonal), 2.0, 0.5)));
  ADSIM_CHECK(!obbIntersect(axis_aligned, Obb2(Pose2(3.0, 0.0, diagonal), 2.0, 0.5)));
  ADSIM_CHECK(!obbIntersect(axis_aligned, Obb2(Pose2(5.0, 0.0, diagonal), 2.0, 0.5)));
}

ADSIM_TEST(Sim, OBB_精确间距) {
  // 纵向正对：间隙 = 5 - 2 - 2 = 1
  const Obb2 a(Pose2(0.0, 0.0, 0.0), 4.0, 2.0);
  ADSIM_CHECK_NEAR(obbDistance(a, Obb2(Pose2(5.0, 0.0, 0.0), 4.0, 2.0)), 1.0, 1e-9);

  // 横向正对：间隙 = 3 - 1 - 1 = 1
  ADSIM_CHECK_NEAR(obbDistance(a, Obb2(Pose2(0.0, 3.0, 0.0), 4.0, 2.0)), 1.0, 1e-9);

  // 斜向（角对角）：最近点是两个角点 (1,1)-(2,2)，距离 √2
  const Obb2 small(Pose2(0.0, 0.0, 0.0), 2.0, 2.0);
  ADSIM_CHECK_NEAR(obbDistance(small, Obb2(Pose2(3.0, 3.0, 0.0), 2.0, 2.0)), std::sqrt(2.0),
                   1e-9);
  // 角对角（非对称尺寸）：长盒半长 3、半宽 1，与另一个错开 (4,3)。
  // 最近点是 (3,1)-(4,3) 两个角点，距离 √5；
  // 而任何"车心距 - 半径"近似都给不出这个值（车心距 5 减去任何常数≠√5）。
  const Obb2 long_box(Pose2(0.0, 0.0, 0.0), 6.0, 2.0);   // x∈[-3,3], y∈[-1,1]
  const Obb2 shifted(Pose2(7.0, 4.0, 0.0), 6.0, 2.0);     // x∈[4,10], y∈[3,5]
  ADSIM_CHECK_NEAR(obbDistance(long_box, shifted), std::sqrt(1.0 * 1.0 + 2.0 * 2.0), 1e-9);

  // 面—面正对：错开 (5,4) 时 x 区间重叠，最近的是上下两条边，间隙 2
  ADSIM_CHECK_NEAR(obbDistance(long_box, Obb2(Pose2(5.0, 4.0, 0.0), 6.0, 2.0)), 2.0, 1e-9);

  // 顶点—边最近的斜置情形：一个矩形的顶点正对另一个的边
  const Obb2 edge_case(Pose2(3.5, 3.0, 0.0), 2.0, 2.0);  // 左下角 (2.5,2)
  const Obb2 base(Pose2(0.0, 0.0, 0.0), 4.0, 2.0);       // 右上角 (2,1)
  ADSIM_CHECK_NEAR(obbDistance(base, edge_case), std::sqrt(0.5 * 0.5 + 1.0 * 1.0), 1e-9);

  // 相交时返回 0（内部/包含也算 0）
  ADSIM_CHECK_NEAR(obbDistance(a, Obb2(Pose2(1.0, 0.0, 0.0), 4.0, 2.0)), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(obbDistance(a, Obb2(Pose2(0.0, 0.0, 0.0), 1.0, 1.0)), 0.0, 1e-12);
  // 相切时也是 0
  ADSIM_CHECK_NEAR(obbDistance(a, Obb2(Pose2(4.0, 0.0, 0.0), 4.0, 2.0)), 0.0, 1e-12);

  // 对称性
  ADSIM_CHECK_NEAR(obbDistance(small, long_box), obbDistance(long_box, small), 1e-12);
}

ADSIM_TEST(Sim, OBB_包含判定) {
  // 旋转 45° 的矩形：局部坐标系才是正确判据，用 AABB 判会多收 41% 面积
  const Obb2 box(Pose2(1.0, 1.0, kPi * 0.25), 4.0, 2.0);

  ADSIM_CHECK(obbContains(box, Vec2(1.0, 1.0)));  // 中心
  // 沿局部 x 轴 1.5m（局部 (1.5, 0) ⇒ 全局 (1+1.06, 1+1.06)）
  const Vec2 along_x = box.pose.toGlobal(Vec2(1.5, 0.0));
  ADSIM_CHECK(obbContains(box, along_x));
  // 沿局部 y 轴 0.9m
  const Vec2 along_y = box.pose.toGlobal(Vec2(0.0, 0.9));
  ADSIM_CHECK(obbContains(box, along_y));
  // 局部 x 超界
  ADSIM_CHECK(!obbContains(box, box.pose.toGlobal(Vec2(2.1, 0.0))));
  ADSIM_CHECK(!obbContains(box, box.pose.toGlobal(Vec2(0.0, 1.1))));
  // AABB 内但 OBB 外（45° 矩形的角外区域）
  const Vec2 aabb_corner = box.aabb().max;
  ADSIM_CHECK(box.aabb().contains(aabb_corner));
  ADSIM_CHECK(!obbContains(box, aabb_corner));

  // 边界上的点算包含（容差内）
  ADSIM_CHECK(obbContains(box, box.pose.toGlobal(Vec2(2.0, 0.0))));
}

// ===========================================================================
//  四、仿真引擎
// ===========================================================================

ADSIM_TEST(Sim, 引擎_固定步长与轨迹记录) {
  SimEngine::Config config;
  config.max_duration = 2.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));

  const SimulationResult result = engine.run();

  // 2s / 0.05s = 40 步，外加 t=0 的初始点
  ADSIM_CHECK_EQ(static_cast<int>(result.time.size()), 41);
  ADSIM_CHECK_EQ(static_cast<int>(result.ego_states.size()), 41);
  ADSIM_CHECK_NEAR(result.time.front(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(result.time.back(), 2.0, 1e-12);
  for (std::size_t i = 1; i < result.time.size(); ++i) {
    ADSIM_CHECK_NEAR(result.time[i] - result.time[i - 1], 0.05, 1e-9);
  }

  // 无策略时默认滑行：速度不变，里程 = v·t
  ADSIM_CHECK_NEAR(result.ego_states.back().v, 10.0, 1e-9);
  ADSIM_CHECK_NEAR(result.total_distance, 20.0, 1e-9);
  ADSIM_CHECK_NEAR(result.average_speed, 10.0, 1e-9);
  ADSIM_CHECK(result.completed);
  ADSIM_CHECK(!result.isCritical());

  // 位姿与轨迹点一致
  ADSIM_CHECK_NEAR(engine.ego().x, result.ego_states.back().x, 1e-12);
  ADSIM_CHECK_NEAR(engine.time(), 2.0, 1e-12);
}

ADSIM_TEST(Sim, 引擎_确定性) {
  SimEngine::Config config;
  config.max_duration = 3.0;

  const auto run_once = [&config]() {
    SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
    engine.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));
    RoadObject lead;
    lead.id = 1;
    lead.pose = Pose2(30.0, -1.75, 0.0);
    lead.speed = 8.0;
    lead.velocity = Vec2{8.0, 0.0};
    engine.addObject(lead);
    engine.setPolicy(std::make_shared<TestAebPolicy>(12.0));
    return engine.run();
  };

  const SimulationResult first = run_once();
  const SimulationResult second = run_once();

  // 不使用随机数：相同输入必须逐位一致
  ADSIM_CHECK_EQ(static_cast<int>(first.time.size()), static_cast<int>(second.time.size()));
  ADSIM_CHECK_EQ(static_cast<int>(first.events.size()), static_cast<int>(second.events.size()));
  ADSIM_CHECK_NEAR(first.min_ttc, second.min_ttc, 0.0);
  ADSIM_CHECK_NEAR(first.min_distance, second.min_distance, 0.0);
  ADSIM_CHECK_NEAR(first.max_jerk, second.max_jerk, 0.0);
  ADSIM_CHECK_NEAR(first.total_distance, second.total_distance, 0.0);
  for (std::size_t i = 0; i < first.ego_states.size(); ++i) {
    ADSIM_CHECK(first.ego_states[i].x == second.ego_states[i].x);
    ADSIM_CHECK(first.ego_states[i].y == second.ego_states[i].y);
    ADSIM_CHECK(first.ego_states[i].v == second.ego_states[i].v);
    ADSIM_CHECK(first.time[i] == second.time[i]);
  }
}

ADSIM_TEST(Sim, 引擎_终止条件) {
  SimEngine::Config config;
  config.max_duration = 10.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));
  engine.setTerminationCondition(
      [](const SimEngine& state) { return state.time() >= 1.0; });

  const SimulationResult result = engine.run();

  // 终止条件在每步之前检查：时长落在 [1.0, 1.0 + dt]
  ADSIM_CHECK_GT(result.time.back(), 0.95);
  ADSIM_CHECK_LT(result.time.back(), 1.05);
  ADSIM_CHECK_LT(result.total_distance, 11.0);
}

ADSIM_TEST(Sim, 引擎_碰撞即停与事件折叠) {
  // 自车 10m/s 追尾静止前车（车心距 12m）：0.745s 后接触
  SimEngine::Config config;
  config.max_duration = 4.0;
  config.stop_on_collision = true;

  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));
  RoadObject obstacle;
  obstacle.id = 3;
  obstacle.pose = Pose2(12.0, -1.75, 0.0);
  engine.addObject(obstacle);

  const SimulationResult result = engine.run();

  ADSIM_CHECK_EQ(result.collision_count, 1);
  ADSIM_CHECK_EQ(countEvents(result, SafetyEvent::kCollision), 1);
  ADSIM_CHECK(result.isCritical());
  ADSIM_CHECK(!result.completed);
  ADSIM_CHECK(engine.hasCollision());
  ADSIM_CHECK_LT(result.time.back(), 1.0);  // 碰撞后立即退出

  const SafetyEventRecord* collision = firstEvent(result, SafetyEvent::kCollision);
  ADSIM_CHECK(collision != nullptr);
  if (collision != nullptr) {
    ADSIM_CHECK_NEAR(collision->time, 0.745, 0.06);
    ADSIM_CHECK_EQ(collision->object_id, 3);
  }

  // 持续接触只算一次事件（而不是每一步一条）：
  // 关掉 stop_on_collision 后自车会长时间压在障碍物上
  SimEngine::Config ongoing = config;
  ongoing.stop_on_collision = false;
  SimEngine sustained(World::straightRoad(2, 3.5, 200.0), VehicleModel(), ongoing);
  sustained.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));
  sustained.addObject(obstacle);
  const SimulationResult long_run = sustained.run();

  ADSIM_CHECK_EQ(long_run.collision_count, 1);
  ADSIM_CHECK_EQ(countEvents(long_run, SafetyEvent::kCollision), 1);
  ADSIM_CHECK_EQ(long_run.near_miss_count, 1);
  ADSIM_CHECK_EQ(countEvents(long_run, SafetyEvent::kNearMiss), 1);
  ADSIM_CHECK_NEAR(long_run.time.back(), 4.0, 1e-9);
}

ADSIM_TEST(Sim, 引擎_指标_侧向加速度_曲率_jerk) {
  SimEngine::Config config;
  config.max_duration = 4.0;
  SimEngine engine(World::curvedRoad(2, 3.5, 200.0, 0.02), VehicleModel(), config);
  // 初速 5 m/s、目标 12 m/s：加速过程会留下非零 jerk，才能验证 jerk 指标
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 5.0));
  engine.setPolicy(std::make_shared<TestConstantSteerPolicy>(0.02, 12.0));

  const SimulationResult result = engine.run();

  // 报告里的极值必须与轨迹点自洽：a_lat = v²κ、|κ|、jerk 逐点复核
  double expected_lateral = 0.0;
  double expected_curvature = 0.0;
  for (const TrajectoryPoint& point : result.ego_states) {
    expected_lateral = std::max(expected_lateral, point.v * point.v * std::fabs(point.kappa));
    expected_curvature = std::max(expected_curvature, std::fabs(point.kappa));
  }
  ADSIM_CHECK_NEAR(result.max_lateral_acceleration, expected_lateral, 1e-9);
  ADSIM_CHECK_NEAR(result.max_curvature, expected_curvature, 1e-9);
  ADSIM_CHECK_NEAR(result.max_curvature, 0.02, 1e-6);
  // v 收敛到 12 m/s（±0.5），横向加速度 ≈ v²·κ ≈ 2.9 m/s²
  ADSIM_CHECK_GT(result.max_lateral_acceleration, 2.5);
  ADSIM_CHECK_LT(result.max_lateral_acceleration, 3.5);
  ADSIM_CHECK_NEAR(result.ego_states.back().v, 12.0, 0.5);

  // jerk 受模型 max_jerk 约束：绝不出现超过物理限制的"虚假尖峰"
  ADSIM_CHECK_GT(result.max_jerk, 4.0);
  ADSIM_CHECK_LT(result.max_jerk, VehicleParams{}.max_jerk + 1e-9);
}

ADSIM_TEST(Sim, 引擎_指标_最小间距与物体历史) {
  SimEngine::Config config;
  config.max_duration = 3.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 6.0));
  RoadObject lead;
  lead.id = 9;
  lead.pose = Pose2(15.0, -1.75, 0.0);
  lead.speed = 4.0;
  lead.velocity = Vec2{4.0, 0.0};
  engine.addObject(lead);

  const SimulationResult result = engine.run();

  // 用记录下来的轨迹与物体历史复算最小间距，必须与报告一致
  ADSIM_CHECK_EQ(static_cast<int>(result.object_history.size()),
                 static_cast<int>(result.ego_states.size()));
  const VehicleModel model;
  double min_distance = 1e9;
  for (std::size_t i = 0; i < result.ego_states.size(); ++i) {
    const Obb2 ego_box(result.ego_states[i].pose(), model.params().length(),
                       model.params().width);
    for (const RoadObject& object : result.object_history[i]) {
      min_distance = std::min(min_distance, obbDistance(ego_box, object.obb()));
    }
  }
  ADSIM_CHECK_NEAR(result.min_distance, min_distance, 1e-9);
  ADSIM_CHECK_LT(result.min_distance, 1e9);

  // 未开启历史记录时不再累积（内存友好）
  SimEngine::Config lean = config;
  lean.record_object_history = false;
  SimEngine lean_engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), lean);
  lean_engine.setEgo(makeEgo(0.0, -1.75, 0.0, 6.0));
  lean_engine.addObject(lead);
  ADSIM_CHECK_EQ(static_cast<int>(lean_engine.run().object_history.size()), 0);
}

ADSIM_TEST(Sim, 引擎_碰撞时间TTC) {
  SimEngine::Config config;
  config.max_duration = 5.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 10.0));

  RoadObject lead;
  lead.id = 1;
  lead.pose = Pose2(20.0, -1.75, 0.0);
  lead.speed = 0.0;
  lead.velocity = Vec2{0.0, 0.0};
  engine.addObject(lead);

  // 车心距 20m、车长 4.6/4.5 ⇒ 接触时车心距 4.55m，需行驶 15.45m / 10m/s = 1.545s
  const double ttc = engine.timeToCollision(lead);
  ADSIM_CHECK(std::isfinite(ttc));
  ADSIM_CHECK_NEAR(ttc, 1.545, 0.05);

  // 车心距 12m ⇒ 0.745s
  lead.pose = Pose2(12.0, -1.75, 0.0);
  ADSIM_CHECK_NEAR(engine.timeToCollision(lead), 0.745, 0.02);

  // 前车同速/更快驶离：不可能追上
  lead.pose = Pose2(20.0, -1.75, 0.0);
  lead.velocity = Vec2{10.0, 0.0};
  lead.speed = 10.0;
  const double same_speed = engine.timeToCollision(lead);
  ADSIM_CHECK(std::isfinite(same_speed));
  ADSIM_CHECK_GT(same_speed, 1e8);
  lead.velocity = Vec2{15.0, 0.0};
  lead.speed = 15.0;
  ADSIM_CHECK_GT(engine.timeToCollision(lead), 1e8);

  // 目标在后方：不构成追尾风险
  lead.pose = Pose2(-20.0, -1.75, 0.0);
  lead.velocity = Vec2{0.0, 0.0};
  lead.speed = 0.0;
  ADSIM_CHECK_GT(engine.timeToCollision(lead), 1e8);

  // 横向错开一个车道：直线外推永不接触
  lead.pose = Pose2(20.0, 1.75, 0.0);
  ADSIM_CHECK_GT(engine.timeToCollision(lead), 1e8);

  // 斜向横穿：目标航向 -90°，其纵向 4.5m/横向 1.8m 因此分别落在 y/x 轴上。
  // 自车（车宽 ±0.9）与目标（x 向 ±0.9）接触要求车头 2.3+10t ≥ 12-0.9
  // ⇒ t ≥ 0.88s；而此时目标已横向压到 -1.01m，与自车纵向区间重叠 ⇒ 0.88s 相撞。
  // 注意 0.88s != 纵向间距/速度(0.745s)：横向运动让目标"提前挡住"了自车。
  lead.pose = Pose2(12.0, 3.0, -kPi * 0.5);
  lead.velocity = Vec2{0.0, -2.0};
  lead.speed = 2.0;
  ADSIM_CHECK_NEAR(engine.timeToCollision(lead), 0.88, 0.05);

  // 同样斜向但横向远离自车车道：永不接触
  lead.pose = Pose2(25.0, 6.0, -kPi * 0.5);
  lead.velocity = Vec2{0.0, 2.0};
  lead.speed = 2.0;
  ADSIM_CHECK_GT(engine.timeToCollision(lead), 1e8);

  // 自车静止：不会主动撞上静止目标
  SimEngine stopped_engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  stopped_engine.setEgo(makeEgo(0.0, -1.75, 0.0, 0.0));
  lead.pose = Pose2(10.0, -1.75, 0.0);
  lead.velocity = Vec2{0.0, 0.0};
  stopped_engine.addObject(lead);
  ADSIM_CHECK_GT(stopped_engine.timeToCollision(lead), 1e8);
}

ADSIM_TEST(Sim, 引擎_安全事件_接近事故与驶出路面) {
  SimEngine::Config config;
  config.max_duration = 2.0;

  World world = World::straightRoad(2, 3.5, 200.0);
  // 静态障碍在自车车道外侧 1.0m（< near_miss_distance 1.5m）：接近事故但不碰撞
  // 自车 0.9 半宽 + 障碍 0.5 半宽 = 1.4，因此把障碍中心放在 3.15m 外
  world.addObstacle(Obb2(Pose2(10.0, -1.75 + 1.4 + 1.0, 0.0), 1.0, 1.0));

  SimEngine engine(world, VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 5.0));
  const SimulationResult result = engine.run();

  ADSIM_CHECK_EQ(result.collision_count, 0);
  ADSIM_CHECK_EQ(result.near_miss_count, 1);
  ADSIM_CHECK_EQ(countEvents(result, SafetyEvent::kNearMiss), 1);
  ADSIM_CHECK_NEAR(result.min_distance, 1.0, 1e-9);
  const SafetyEventRecord* near_miss = firstEvent(result, SafetyEvent::kNearMiss);
  ADSIM_CHECK(near_miss != nullptr);
  if (near_miss != nullptr) {
    ADSIM_CHECK_NEAR(near_miss->value, 1.0, 1e-9);
    ADSIM_CHECK_EQ(near_miss->object_id, -1);  // 静态障碍
  }

  // 驶出路面：自车初始在路外（y = 10m）
  SimEngine off_road_engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  off_road_engine.setEgo(makeEgo(0.0, 10.0, 0.0, 5.0));
  const SimulationResult off = off_road_engine.run();

  ADSIM_CHECK(off_road_engine.isOffRoad());
  ADSIM_CHECK_EQ(off.off_road_count, 1);
  ADSIM_CHECK_EQ(countEvents(off, SafetyEvent::kOffRoad), 1);
  ADSIM_CHECK(!off.completed);
  const SafetyEventRecord* off_event = firstEvent(off, SafetyEvent::kOffRoad);
  ADSIM_CHECK(off_event != nullptr);
  if (off_event != nullptr) {
    // 距中心线 8.25m，扣掉半车道宽 1.75m ⇒ 超出 6.5m
    ADSIM_CHECK_NEAR(off_event->value, 6.5, 0.1);
  }
}

ADSIM_TEST(Sim, 引擎_结果报告与危险性判定) {
  // 安全场景：报告可读，且不判为危险
  SimEngine::Config config;
  config.max_duration = 2.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 8.0));
  const SimulationResult clean = engine.run();

  const std::string clean_report = clean.toString();
  ADSIM_CHECK(!clean_report.empty());
  ADSIM_CHECK(clean_report.find("(未命名)") != std::string::npos);
  ADSIM_CHECK(clean_report.find("平均速度") != std::string::npos);
  ADSIM_CHECK(clean_report.find("最大 jerk") != std::string::npos);
  ADSIM_CHECK(clean_report.find("无其他物体") != std::string::npos);
  ADSIM_CHECK(clean_report.find("是否危险工况: 否") != std::string::npos);
  ADSIM_CHECK(!clean.isCritical());

  // 危险场景：报告点名场景与事件
  LeadBrakeScenario scenario;
  SimEngine::Config brake_config = config;
  brake_config.max_duration = 12.0;
  const SimulationResult crash = scenario.run(brake_config);

  const std::string crash_report = crash.toString();
  ADSIM_CHECK(crash_report.find("LeadBrake") != std::string::npos);
  ADSIM_CHECK(crash_report.find("碰撞") != std::string::npos);
  ADSIM_CHECK(crash_report.find("是否危险工况: 是") != std::string::npos);
  ADSIM_CHECK(crash_report.find("是否完成    : 否") != std::string::npos);
  ADSIM_CHECK(crash.isCritical());
}

ADSIM_TEST(Sim, 引擎_规划路径记录) {
  SimEngine::Config config;
  config.max_duration = 1.0;
  SimEngine engine(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  engine.setEgo(makeEgo(0.0, -1.75, 0.0, 8.0));

  auto policy = std::make_shared<TestPurePursuitPolicy>(10.0, 8.0);
  engine.setPolicy(policy);

  // 手动步进：每步让策略给出 5 个前视点，验证数组按步累积（扁平结构）
  const World& reference = engine.world();
  int steps = 0;
  while (engine.time() < config.max_duration - 1e-9) {
    std::vector<Vec2> path;
    const LaneProjection projection = reference.project(engine.ego().pose().position());
    for (int k = 1; k <= 5; ++k) {
      path.push_back(reference.pointAt(projection.lane_id, projection.s + 2.0 * k));
    }
    policy->setPlannedPath(path);
    engine.step(nullptr);
    ++steps;
  }
  const SimulationResult result = engine.run();

  ADSIM_CHECK_EQ(steps, 20);
  ADSIM_CHECK_EQ(static_cast<int>(result.planned_paths.size()), 5 * steps);
  // 规划点落在自车前方，且时间戳与所在步一致
  for (std::size_t i = 0; i < result.planned_paths.size(); ++i) {
    ADSIM_CHECK_GT(result.planned_paths[i].x, -1.0);
  }
  ADSIM_CHECK_NEAR(result.planned_paths.front().t, 0.05, 1e-9);

  // 无策略时不记录规划输出
  SimEngine plain(World::straightRoad(2, 3.5, 200.0), VehicleModel(), config);
  plain.setEgo(makeEgo(0.0, -1.75, 0.0, 8.0));
  ADSIM_CHECK_EQ(static_cast<int>(plain.run().planned_paths.size()), 0);
}

// ===========================================================================
//  五、场景库
// ===========================================================================

ADSIM_TEST(Sim, 场景_注册表) {
  ScenarioRegistry& registry = ScenarioRegistry::instance();

  const std::vector<std::string> names = registry.names();
  ADSIM_CHECK_GT(names.size(), std::size_t{4});
  ADSIM_CHECK(std::find(names.begin(), names.end(), "LeadBrake") != names.end());
  ADSIM_CHECK(std::find(names.begin(), names.end(), "PedestrianCrossing") != names.end());

  // 五个内置场景都能创建，且元信息完整
  const std::vector<ScenarioInfo> infos = registry.allInfo();
  ADSIM_CHECK_EQ(infos.size(), names.size());
  for (const ScenarioInfo& info : infos) {
    ADSIM_CHECK(!info.name.empty());
    ADSIM_CHECK(!info.description.empty());
    ADSIM_CHECK(!info.category.empty());
    ADSIM_CHECK_GT(info.duration, 0.0);
    ADSIM_CHECK(registry.create(info.name) != nullptr);
  }

  // 名称归一化：忽略大小写与分隔符
  ADSIM_CHECK(registry.create("lead_brake") != nullptr);
  ADSIM_CHECK(registry.create("LEAD-BRAKE") != nullptr);
  ADSIM_CHECK(registry.create("  CutIn  ") != nullptr);
  // 未知场景返回空指针，由调用方决定如何报错
  ADSIM_CHECK(registry.create("不存在的场景") == nullptr);
  ADSIM_CHECK(registry.create("") == nullptr);

  // 创建出的场景可独立构造世界与自车状态
  std::unique_ptr<Scenario> curved = registry.create("CurvedRoad");
  ADSIM_CHECK(curved != nullptr);
  if (curved != nullptr) {
    ADSIM_CHECK_GT(curved->buildWorld().laneCount(), std::size_t{0});
    ADSIM_CHECK_NEAR(curved->initialEgoState().speed, 15.0, 1e-9);
  }
  // 有明确终点的场景提供目标路径，供"是否跑完"与规划模块使用
  std::unique_ptr<Scenario> lead_brake = registry.create("LeadBrake");
  ADSIM_CHECK(lead_brake != nullptr);
  if (lead_brake != nullptr) {
    ADSIM_CHECK(!lead_brake->goalPath().empty());
    ADSIM_CHECK(!lead_brake->info().category.empty());
    ADSIM_CHECK(lead_brake->info().is_critical);
  }
}

ADSIM_TEST(Sim, 场景_前车急刹_无策略) {
  LeadBrakeScenario scenario;
  SimEngine::Config config;
  config.max_duration = 12.0;

  const World world = scenario.buildWorld();
  ADSIM_CHECK_EQ(static_cast<int>(world.objects().size()), 1);
  const int lead_id = world.objects().front().id;

  const SimulationResult result = scenario.run(config);

  ADSIM_CHECK_EQ(result.scenario_name, "LeadBrake");
  ADSIM_CHECK(result.collision_count >= 1);
  ADSIM_CHECK(result.isCritical());
  ADSIM_CHECK(!result.completed);
  ADSIM_CHECK_LT(result.min_ttc, 2.0);
  ADSIM_CHECK_NEAR(result.min_distance, 0.0, 1e-9);

  // 碰撞事件指向被追尾的目标，且发生在开始制动之后
  const SafetyEventRecord* collision = firstEvent(result, SafetyEvent::kCollision);
  ADSIM_CHECK(collision != nullptr);
  if (collision != nullptr) {
    ADSIM_CHECK_EQ(collision->object_id, lead_id);
    ADSIM_CHECK_GT(collision->time, 3.0);
  }
}

ADSIM_TEST(Sim, 场景_前车急刹_AEB策略) {
  LeadBrakeScenario scenario;
  SimEngine::Config config;
  config.max_duration = 20.0;

  const SimulationResult result =
      scenario.run(config, std::make_shared<TestAebPolicy>(13.0));

  ADSIM_CHECK_EQ(result.collision_count, 0);
  ADSIM_CHECK_EQ(result.off_road_count, 0);
  ADSIM_CHECK(result.completed);
  ADSIM_CHECK(!result.isCritical());
  ADSIM_CHECK_GT(result.min_distance, 1.5);
  ADSIM_CHECK_GT(result.min_ttc, 1.0);
  // 制动过程记录为急刹事件，且是"一次事件"而不是每步一条
  ADSIM_CHECK(countEvents(result, SafetyEvent::kHarshBraking) >= 1);
  ADSIM_CHECK_LT(result.max_jerk, VehicleParams{}.max_jerk + 1e-9);
}

// 提前终止（shouldTerminate）是会漏测的一条路径：场景脚本在自车停稳后判定任务
// 结束，循环提前退出，此时**时间并未走满** max_duration。而 Scenario::run 是
// "自建循环 + 交回 engine.run() 收尾"的两段式结构，收尾那一步以 step(nullptr)
// 推进 —— 一旦退出条件没有被 run() 识别，它就会继续空跑：场景脚本不再被调用，
// 其他交通参与者被冻结在原地，自车照常行驶，于是报出"追尾静止前车"这类假碰撞。
// 本用例把这条路径固定下来：提前终止时步数不得再增长、物体必须与解析式一致
// 到最后一步、且不得出现碰撞。
ADSIM_TEST(Sim, 场景_提前终止_不空跑) {
  const double dt = 0.05;

  // ---- (1) 前车急刹 + 刹停保持：自车停稳后由"持续静止"判据提前终止 ----
  {
    LeadBrakeScenario scenario;
    SimEngine::Config config;
    config.max_duration = 30.0;  // 远大于实际结束时刻，便于暴露"空跑到超时"
    const SimulationResult result =
        scenario.run(config, std::make_shared<TestStopAndHoldPolicy>());

    ADSIM_CHECK_LT(result.time.back(), config.max_duration - 1.0);
    ADSIM_CHECK_EQ(result.collision_count, 0);
    ADSIM_CHECK(result.completed);
    // 记录的时间跨度就是真实仿真时长：步数 = 时长/步长 + 1（首拍种子）
    const int expected_steps = static_cast<int>(std::lround(result.time.back() / dt)) + 1;
    ADSIM_CHECK_EQ(static_cast<int>(result.time.size()), expected_steps);
    // 自车停稳后不再被推着走：末端若干步速度为零。
    //
    // 容差取 1 mm/s 而非浮点量级的 1e-6：测试策略用 a = −2v 做指数衰减，
    // 速度渐近趋于零但永远不等于零，残余量在 1e-5 m/s 量级（约 18 µm/s）。
    // 用 1e-6 去断言等于在检验浮点噪声而不是行为——这里真正要固定的是
    // "车已经停住、没有被推着走"这一事实。
    ADSIM_CHECK(result.ego_states.size() >= 20);
    for (std::size_t i = result.ego_states.size() - 20; i < result.ego_states.size(); ++i) {
      ADSIM_CHECK_NEAR(result.ego_states[i].v, 0.0, 1e-3);
    }
  }

  // ---- (2) 加塞：结束时被加塞车辆仍在行驶，物体必须一直被更新到最后一步 ----
  // 用会自己行驶的物体做判据，才能区分"脚本仍在更新"与"物体被冻结"。
  {
    CutInScenario scenario;
    SimEngine::Config config;
    config.max_duration = 30.0;

    SimEngine engine(scenario.buildWorld(), VehicleModel(), config);
    engine.setEgo(scenario.initialEgoState());
    auto policy = std::make_shared<TestPurePursuitPolicy>(10.0, 8.0);
    policy->reset();
    engine.setPolicy(policy);
    engine.setTerminationCondition(
        [&scenario](const SimEngine& e) { return scenario.shouldTerminate(e); });

    const std::function<void(World&, double)> updater = [&scenario, dt](World& world, double t) {
      scenario.update(world, t, dt);
    };
    std::size_t guard = 0;
    while (engine.time() < config.max_duration - 1e-9 && !scenario.shouldTerminate(engine) &&
           guard++ < 100000) {
      engine.step(updater);
    }
    ADSIM_CHECK_LT(engine.time(), config.max_duration - 1.0);  // 确实是提前终止

    const std::size_t steps_before = engine.result().time.size();
    const SimulationResult result = engine.run();  // 交回引擎收尾
    ADSIM_CHECK_EQ(static_cast<int>(result.time.size()), static_cast<int>(steps_before));

    // 末端物体位置 = 场景解析式在该时刻的值（容差取一个步长的行进量：
    // 物体在步内取零阶保持，记录与统计里的物体时刻为步长起点）
    ADSIM_CHECK(!result.object_history.empty());
    ADSIM_CHECK(!result.object_history.back().empty());
    World probe = scenario.buildWorld();
    scenario.update(probe, result.time.back(), dt);
    const double recorded_x = result.object_history.back().front().pose.x;
    const double analytic_x = probe.objects().front().pose.x;
    const double object_speed =
        std::max(probe.objects().front().speed, result.object_history.back().front().speed);
    ADSIM_CHECK_NEAR(recorded_x, analytic_x, object_speed * dt + 1e-6);

    // 记录末尾即最后一个时刻：自车状态与时间轴对齐，没有被空跑到超时
    ADSIM_CHECK_NEAR(result.ego_states.back().t, result.time.back(), 1e-9);
  }
}

ADSIM_TEST(Sim, 场景_加塞_横向轨迹与碰撞) {
  CutInScenario scenario;
  World world = scenario.buildWorld();
  ADSIM_CHECK_EQ(static_cast<int>(world.objects().size()), 1);
  const RoadObject& cut_in = world.objects().front();
  const int cut_in_id = cut_in.id;
  const double start_y = cut_in.pose.y;
  const double lane_width = world.lanes().front().width;
  ADSIM_CHECK_NEAR(start_y, 1.75, 1e-9);
  ADSIM_CHECK_NEAR(lane_width, 3.5, 1e-9);

  // smoothstep 横向轨迹：起点保持相邻车道，中点正好压线，终点完全并入自车车道
  World probe_world = scenario.buildWorld();
  scenario.update(probe_world, 2.5, 0.05);
  ADSIM_CHECK_NEAR(probe_world.objects().front().pose.y, start_y, 1e-9);
  probe_world = scenario.buildWorld();
  scenario.update(probe_world, 3.5, 0.05);
  ADSIM_CHECK_NEAR(probe_world.objects().front().pose.y, start_y - lane_width * 0.5, 1e-6);
  probe_world = scenario.buildWorld();
  scenario.update(probe_world, 4.5, 0.05);
  ADSIM_CHECK_NEAR(probe_world.objects().front().pose.y, start_y - lane_width, 1e-6);
  // 横向运动连续：车头指向实际运动方向，不会"斜着平移"
  scenario.update(probe_world, 4.6, 0.05);
  ADSIM_CHECK_NEAR(probe_world.objects().front().pose.theta, 0.0, 1e-3);

  SimEngine::Config config;
  config.max_duration = 12.0;
  const SimulationResult result = scenario.run(config);
  ADSIM_CHECK_EQ(result.scenario_name, "CutIn");
  ADSIM_CHECK(result.collision_count >= 1);
  ADSIM_CHECK(!result.completed);
  const SafetyEventRecord* collision = firstEvent(result, SafetyEvent::kCollision);
  ADSIM_CHECK(collision != nullptr);
  if (collision != nullptr) ADSIM_CHECK_EQ(collision->object_id, cut_in_id);
}

ADSIM_TEST(Sim, 场景_行人横穿_解析运动与碰撞) {
  PedestrianCrossingScenario scenario;
  World world = scenario.buildWorld();
  ADSIM_CHECK_EQ(static_cast<int>(world.objects().size()), 1);

  const RoadObject& pedestrian = world.objects().front();
  ADSIM_CHECK(pedestrian.type == RoadObject::Type::kPedestrian);
  // 行人轮廓 0.5m × 0.5m（肩宽量级），远小于车辆
  ADSIM_CHECK_NEAR(pedestrian.length, 0.5, 1e-12);
  ADSIM_CHECK_NEAR(pedestrian.width, 0.5, 1e-12);
  const double start_y = pedestrian.pose.y;
  const int pedestrian_id = pedestrian.id;

  // 解析运动：起始时刻前原地等待；之后以恒定步速 1.4m/s 横穿
  World probe = scenario.buildWorld();
  scenario.update(probe, 1.0, 0.05);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.y, start_y, 1e-12);
  ADSIM_CHECK_NEAR(probe.objects().front().speed, 0.0, 1e-12);

  scenario.update(probe, 3.0, 0.05);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.y, start_y - 1.4, 1e-9);
  ADSIM_CHECK_NEAR(probe.objects().front().speed, 1.4, 1e-9);
  ADSIM_CHECK_NEAR(probe.objects().front().velocity.y, -1.4, 1e-9);

  scenario.update(probe, 4.0, 0.05);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.y, start_y - 2.8, 1e-9);

  // 横穿结束后在道路另一侧停下，不会继续"漂移"
  scenario.update(probe, 100.0, 0.05);
  ADSIM_CHECK_NEAR(probe.objects().front().speed, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.y, start_y - 12.0, 1e-9);

  SimEngine::Config config;
  config.max_duration = 12.0;
  const SimulationResult result = scenario.run(config);
  ADSIM_CHECK_EQ(result.scenario_name, "PedestrianCrossing");
  ADSIM_CHECK(result.collision_count >= 1);  // 无策略时与横穿行人相撞
  ADSIM_CHECK(!result.completed);
  const SafetyEventRecord* collision = firstEvent(result, SafetyEvent::kCollision);
  ADSIM_CHECK(collision != nullptr);
  if (collision != nullptr) ADSIM_CHECK_EQ(collision->object_id, pedestrian_id);
}

ADSIM_TEST(Sim, 场景_无保护左转) {
  UnprotectedLeftTurnScenario scenario;
  const World world = scenario.buildWorld();

  // 路口世界：12 条车道，其中 4 条为路口贯通车道
  ADSIM_CHECK_EQ(static_cast<int>(world.laneCount()), 12);
  int junction_lanes = 0;
  for (const Lane& lane : world.lanes()) {
    if (lane.is_junction) ++junction_lanes;
  }
  ADSIM_CHECK_EQ(junction_lanes, 4);

  // 自车与对向车分处相邻车道、方向相反
  const VehicleState ego = scenario.initialEgoState();
  ADSIM_CHECK_NEAR(ego.speed, 8.0, 1e-9);
  const LaneProjection ego_projection = world.project(ego.pose().position());
  ADSIM_CHECK(ego_projection.valid);
  const Lane* ego_lane = world.findLane(ego_projection.lane_id);
  ADSIM_CHECK(ego_lane != nullptr);
  ADSIM_CHECK(!ego_lane->is_junction);
  ADSIM_CHECK_NEAR(ego_lane->centerline.front().y, -1.75, 1e-9);

  ADSIM_CHECK_EQ(static_cast<int>(world.objects().size()), 1);
  const RoadObject& oncoming = world.objects().front();
  ADSIM_CHECK_NEAR(oncoming.velocity.x, -12.0, 1e-9);
  ADSIM_CHECK_GT(std::fabs(oncoming.position().y - ego.pose().position().y), 3.0);

  // 目标路径是"直线 → 90° 圆弧 → 直线"的左转轨迹
  const std::vector<Vec2> goal = scenario.goalPath();
  ADSIM_CHECK_GT(goal.size(), std::size_t{5});
  const std::vector<double> goal_arc = laneArcLength(goal);
  const double start_heading = headingAlong(goal, goal_arc, 0.0);
  const double end_heading = headingAlong(goal, goal_arc, goal_arc.back());
  ADSIM_CHECK_NEAR(std::fabs(normalizeAngle(end_heading - start_heading)), kPi * 0.5, 0.2);

  SimEngine::Config config;
  config.max_duration = 12.0;
  const SimulationResult result = scenario.run(config);

  ADSIM_CHECK_EQ(result.scenario_name, "UnprotectedLeftTurn");
  ADSIM_CHECK_EQ(result.collision_count, 0);  // 直行通过与对向车侧向错开
  ADSIM_CHECK_EQ(result.off_road_count, 0);
  ADSIM_CHECK(result.completed);
  ADSIM_CHECK_GT(result.min_distance, 1.0);
  // 自车确实穿过了路口向东驶去
  ADSIM_CHECK_GT(maxLongitudinal(result), 40.0);
}

ADSIM_TEST(Sim, 场景_弯道_无策略驶出路面) {
  CurvedRoadScenario scenario;
  SimEngine::Config config;
  config.max_duration = 10.0;

  const SimulationResult result = scenario.run(config);

  ADSIM_CHECK_EQ(result.scenario_name, "CurvedRoad");
  ADSIM_CHECK_EQ(result.collision_count, 0);
  ADSIM_CHECK(!result.completed);
  // 不转向就会冲出弯道：驶出路面记一次事件
  ADSIM_CHECK_EQ(result.off_road_count, 1);
  ADSIM_CHECK_EQ(countEvents(result, SafetyEvent::kOffRoad), 1);
  // 15 m/s 超过 13.9 m/s 限速（含 0.5 容差）
  ADSIM_CHECK_GT(countEvents(result, SafetyEvent::kOverSpeed), 0);
  // 全程曲率为 0（方向盘不动的直行）
  ADSIM_CHECK_NEAR(result.max_curvature, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(result.max_lateral_acceleration, 0.0, 1e-12);
}

ADSIM_TEST(Sim, 场景_弯道_纯跟踪跟随) {
  CurvedRoadScenario scenario;
  const World world = scenario.buildWorld();
  ADSIM_CHECK_NEAR(scenario.initialEgoState().speed, 15.0, 1e-9);

  SimEngine::Config config;
  config.max_duration = 10.0;
  const SimulationResult result =
      scenario.run(config, std::make_shared<TestPurePursuitPolicy>(12.0, 8.0));

  ADSIM_CHECK_EQ(result.collision_count, 0);
  ADSIM_CHECK_EQ(result.off_road_count, 0);
  ADSIM_CHECK(result.completed);

  // 右车道（1 号）中心线曲率 ≈ 0.02/1.035：跟随误差应在 0.3m 以内
  const Lane* right = world.findLane(1);
  ADSIM_CHECK(right != nullptr);
  ADSIM_CHECK_LT(maxLateralDeviation(world, result, 1), 0.3);

  // 侧向加速度 = v²κ 与弯道曲率协调（≈ 12² × 0.0193 ≈ 2.8 m/s²）
  ADSIM_CHECK_GT(result.max_lateral_acceleration, 2.0);
  ADSIM_CHECK_LT(result.max_lateral_acceleration, 5.0);
  ADSIM_CHECK_NEAR(result.max_curvature, 0.0193, 0.003);
  // 报告里的 jerk 不会突破车辆物理限制
  ADSIM_CHECK_LT(result.max_jerk, VehicleParams{}.max_jerk + 1e-9);
  // 平均速度合理（弯道限速下仍保持通行效率）
  ADSIM_CHECK_GT(result.average_speed, 8.0);
}

ADSIM_TEST(Sim, 场景_直线_纯跟踪收敛) {
  SimEngine::Config config;
  config.max_duration = 5.0;
  World world = World::straightRoad(2, 3.5, 250.0);
  SimEngine engine(world, VehicleModel(), config);
  // 自车在右车道中心线左侧 0.75m 处、以 6m/s 行驶
  engine.setEgo(makeEgo(0.0, -1.0, 0.0, 6.0));
  engine.setPolicy(std::make_shared<TestPurePursuitPolicy>(10.0, 8.0));

  const SimulationResult result = engine.run();

  const LaneProjection initial = world.project(engine.result().ego_states.front().position());
  ADSIM_CHECK_NEAR(initial.lateral, 0.75, 1e-9);

  // 横向误差单调收敛（无超调）：5s 内从 0.75m 收敛到 1cm 以内
  const LaneProjection final_projection = world.project(result.ego_states.back().position());
  ADSIM_CHECK_NEAR(final_projection.lateral, 0.0, 0.01);
  ADSIM_CHECK_LT(maxLateralDeviation(world, result, 1), 0.76);
  ADSIM_CHECK_EQ(result.off_road_count, 0);
  ADSIM_CHECK(result.completed);
  // 车速收敛到巡航速度
  ADSIM_CHECK_NEAR(result.ego_states.back().v, 10.0, 0.1);
}

// ===========================================================================
//  六、场景参数化与 CARLA 桥接的离线部分
// ===========================================================================

ADSIM_TEST(Sim, 场景_运行时参数与自定义脚本) {
  // 自定义参数：前车更近、制动更早，碰撞时间应更早
  LeadBrakeScenario::Params params;
  params.ego_speed = 10.0;
  params.lead_speed = 8.0;
  params.initial_gap = 12.0;
  params.brake_time = 1.0;
  params.brake_deceleration = -6.0;
  const LeadBrakeScenario scenario(params);

  ADSIM_CHECK_NEAR(scenario.initialEgoState().speed, 10.0, 1e-12);
  const World world = scenario.buildWorld();
  ADSIM_CHECK_NEAR(world.objects().front().pose.x, 12.0, 1e-12);
  ADSIM_CHECK_NEAR(world.objects().front().speed, 8.0, 1e-12);

  // 前车位移脚本：制动前匀速，制动后匀减速到停且不倒车
  World probe = scenario.buildWorld();
  scenario.update(probe, 0.5, 0.05);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.x, 12.0 + 8.0 * 0.5, 1e-9);
  scenario.update(probe, 2.0, 0.05);
  // 1s 匀速 + 1s 匀减速：8 + 8*1 + 0.5*(-6)*1 = 13m
  ADSIM_CHECK_NEAR(probe.objects().front().pose.x, 12.0 + 8.0 + 8.0 + 0.5 * -6.0, 1e-9);
  scenario.update(probe, 10.0, 0.05);
  const double stop_x = 12.0 + 8.0 * 1.0 + 8.0 * (8.0 / 6.0) + 0.5 * -6.0 * (8.0 / 6.0) *
                                                                                  (8.0 / 6.0);
  ADSIM_CHECK_NEAR(probe.objects().front().pose.x, stop_x, 1e-9);
  ADSIM_CHECK_NEAR(probe.objects().front().speed, 0.0, 1e-12);

  SimEngine::Config config;
  config.max_duration = 8.0;
  const SimulationResult result = scenario.run(config);
  ADSIM_CHECK(result.collision_count >= 1);
  ADSIM_CHECK(result.scenario_name == "LeadBrake");
}
