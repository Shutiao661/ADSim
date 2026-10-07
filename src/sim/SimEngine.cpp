// =============================================================================
//  SimEngine.cpp — 仿真内核与 OBB 几何谓词实现
//
//  两条贯穿全文件的设计原则：
//    1) 确定性：不使用任何随机源，容器遍历顺序固定（只用 vector），
//       相同输入必得逐比特相同的输出，回归测试才能做"轨迹逐点比对"。
//    2) 显式积分顺序：物体更新 → 策略 → 自车推进 → 指标 → 事件。
//       物体状态与自车状态在同一拍内存在一个步长的相位差（物体在步首更新、
//       自车推进到步末），这是定步长显式仿真的固有取舍，在 50ms 步长下
//       对间距/TTC 的影响远小于传感器噪声。
// =============================================================================
#include "adsim/sim/SimEngine.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace adsim {

namespace {

/// 不可能发生碰撞时返回的"极大值"：用有限值而不是 inf，
/// 便于下游直接参与 min()/比较而不产生 NaN。
constexpr double kNoCollisionTime = 1e9;
constexpr double kNoCollisionDistance = 1e9;

/// OBB 判定的接触容差：恰好相切（间隙为 0）算相交。
/// 取 1e-9 而不是更大的值，是为了不给"擦身而过"留虚假余量。
constexpr double kObbTolerance = 1e-9;

/// 前向搜索 TTC 的步长与时长上限
constexpr double kTtcSearchStep = 0.05;
constexpr double kTtcSearchHorizon = 10.0;

/// 时间比较容差（用于循环终止判定）
constexpr double kTimeEps = 1e-9;

/// 点到线段的最短距离
double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < kEpsilon) return (p - a).norm();
  const double t = clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return (p - (a + ab * t)).norm();
}

/// 把车辆状态转成轨迹点（曲率由转角反解，速度/加速度直接取自状态）
TrajectoryPoint toTrajectoryPoint(const VehicleState& state, double curvature, double t) {
  TrajectoryPoint p;
  p.x = state.x;
  p.y = state.y;
  p.theta = state.theta;
  p.kappa = curvature;
  p.v = state.speed;
  p.a = state.acceleration;
  p.t = t;
  return p;
}

/// 上一拍是否已经记录过同类事件：用于把"持续存在"的危险状态折叠成一次事件，
/// 否则 near_miss_count 记的就变成"处于接近事故的步数"而不是"接近事故次数"。
/// 前提：处于危险状态时每步都会写记录（见 collectMetrics），
/// 因此"最近一条同类记录的时间戳 == 上一步"就等价于"上一步也处于该危险中"。
bool recordedInPreviousStep(const SimulationResult& result, SafetyEvent event, double time,
                            double dt) {
  // events 按时间递增追加，从后往前找到最近一次同类事件即可
  for (auto it = result.events.rbegin(); it != result.events.rend(); ++it) {
    if (it->event != event) continue;
    return std::fabs(it->time - (time - dt)) < dt * 0.5 + kTimeEps;
  }
  return false;
}

/// 同类事件的严重程度合并：取"更严重"的那一个。
/// 距离 / TTC / 减速度越小越严重，越界距离与车速越大越严重。
double worseValue(SafetyEvent event, double a, double b) {
  switch (event) {
    case SafetyEvent::kOffRoad:
    case SafetyEvent::kOverSpeed:
      return std::max(a, b);
    default:
      return std::min(a, b);
  }
}

/// 把"持续危险"的逐步记录折叠成一次事件记录：同一类事件若上一条记录恰好
/// 早一个步长（即危险未中断），则本步记录只是上一拍的延续，丢弃；
/// 同时把延续期间的取值合并成整个危险区间内最严重的那一瞬。
///
/// 合并取值是必须的：事件在区间起点写入，若不合并，kNearMiss 的 value
/// 恒等于 near_miss_distance 阈值、kOffRoad 的 value 恒等于刚压线时的
/// 微小越界量，字段完全失去信息量；合并后它才是"最危险的那一瞬"。
///
/// 判断用的是**原始记录序列**（含被丢弃的延续记录）的时间戳，
/// 与 collectMetrics 的计数规则完全一致，因此 events 与 *_count 永远自洽。
/// 该函数是幂等的：折叠后的同类记录间隔必然大于 1.5 个步长，
/// 因此 finalize() 被多次调用（场景层自建循环后再调用 run()）不会二次折叠。
void foldContinuousEvents(SimulationResult& result, double dt) {
  constexpr int kEventTypeCount = 7;  // SafetyEvent 的取值个数（kNone..kHarshBraking）
  double last_time[kEventTypeCount];
  bool seen[kEventTypeCount] = {false};
  int kept_index[kEventTypeCount];  // 该类型当前危险区间在 folded 中的记录下标
  for (int i = 0; i < kEventTypeCount; ++i) {
    last_time[i] = 0.0;
    kept_index[i] = -1;
  }

  std::vector<SafetyEventRecord> folded;
  folded.reserve(result.events.size());
  for (const SafetyEventRecord& record : result.events) {
    const int index = static_cast<int>(record.event);
    const bool known = index >= 0 && index < kEventTypeCount;
    const bool continuation =
        known && seen[index] &&
        std::fabs(record.time - last_time[index] - dt) < dt * 0.5 + kTimeEps;
    if (known) {
      seen[index] = true;
      last_time[index] = record.time;  // 记录原始时间戳（含被丢弃的中间拍）
    }
    if (!continuation) {
      if (known) kept_index[index] = static_cast<int>(folded.size());
      folded.push_back(record);
    } else if (kept_index[index] >= 0) {
      SafetyEventRecord& kept = folded[static_cast<std::size_t>(kept_index[index])];
      kept.value = worseValue(record.event, kept.value, record.value);
    }
  }
  result.events.swap(folded);
}

}  // namespace

// ---------------------------------------------------------------------------
// 枚举与结果报告
// ---------------------------------------------------------------------------

const char* toString(SafetyEvent event) {
  switch (event) {
    case SafetyEvent::kNone: return "无";
    case SafetyEvent::kCollision: return "碰撞";
    case SafetyEvent::kNearMiss: return "接近事故";
    case SafetyEvent::kLowTtc: return "低TTC";
    case SafetyEvent::kOffRoad: return "驶出路面";
    case SafetyEvent::kOverSpeed: return "超速";
    case SafetyEvent::kHarshBraking: return "急刹";
  }
  return "未知";
}

std::string SimulationResult::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss << "=== 仿真结果: " << (scenario_name.empty() ? "(未命名)" : scenario_name) << " ===\n";
  oss << std::setprecision(2);
  const double duration = time.empty() ? 0.0 : time.back();
  oss << "  步数        : " << time.size() << " 步, 时长 " << duration << " s\n";
  oss << "  总里程      : " << total_distance << " m\n";
  oss << "  平均速度    : " << average_speed << " m/s\n";
  oss << std::setprecision(3);
  oss << "  最小 TTC    : " << (min_ttc >= kNoCollisionTime ? 0.0 : min_ttc);
  if (min_ttc >= kNoCollisionTime) oss << " (全程无接近风险)";
  oss << "\n";
  oss << "  最小间距    : " << (min_distance >= kNoCollisionDistance ? 0.0 : min_distance);
  if (min_distance >= kNoCollisionDistance) oss << " (无其他物体)";
  oss << "\n";
  oss << std::setprecision(3);
  oss << "  最大侧向加速度: " << max_lateral_acceleration << " m/s^2\n";
  oss << "  最大曲率    : " << max_curvature << " 1/m\n";
  oss << "  最大 jerk   : " << max_jerk << " m/s^3\n";
  oss << "  碰撞/接近/驶出: " << collision_count << " / " << near_miss_count << " / "
      << off_road_count << "\n";
  oss << "  是否完成    : " << (completed ? "是" : "否") << "\n";
  oss << "  是否危险工况: " << (isCritical() ? "是" : "否") << "\n";

  oss << "  安全事件    : ";
  if (events.empty()) {
    oss << "无\n";
  } else {
    oss << events.size() << " 条\n";
    for (const SafetyEventRecord& e : events) {
      oss << "    [t=" << std::setprecision(2) << std::setw(6) << e.time << "s] "
          << adsim::toString(e.event) << " value=" << std::setprecision(3) << e.value;
      if (e.object_id >= 0) oss << " obj=" << e.object_id;
      if (!e.description.empty()) oss << " " << e.description;
      oss << "\n";
    }
  }
  return oss.str();
}

// ---------------------------------------------------------------------------
// OBB 几何谓词
// ---------------------------------------------------------------------------

bool obbIntersect(const Obb2& a, const Obb2& b) {
  // 分离轴定理（SAT）：两个凸多面体不相交 ⇔ 存在一条轴，使两者的投影区间不重叠。
  // 对二维 OBB，候选轴只需取各自两条边的法向（共 4 条）。
  const Vec2 axes[4] = {
      {std::cos(a.pose.theta), std::sin(a.pose.theta)},
      {-std::sin(a.pose.theta), std::cos(a.pose.theta)},
      {std::cos(b.pose.theta), std::sin(b.pose.theta)},
      {-std::sin(b.pose.theta), std::cos(b.pose.theta)},
  };

  for (const Vec2& axis : axes) {
    // 把盒子投影到轴上：中心投影 ±(半长、半宽在轴上的投影之和)。
    // 这样避免了展开四个角点，数值上也更稳定。
    const Vec2 ca = a.pose.position();
    const Vec2 ua(std::cos(a.pose.theta), std::sin(a.pose.theta));
    const Vec2 va(-ua.y, ua.x);
    const double ra = std::fabs(axis.dot(ua)) * a.length * 0.5 +
                      std::fabs(axis.dot(va)) * a.width * 0.5;
    const double center_a = axis.dot(ca);

    const Vec2 cb = b.pose.position();
    const Vec2 ub(std::cos(b.pose.theta), std::sin(b.pose.theta));
    const Vec2 vb(-ub.y, ub.x);
    const double rb = std::fabs(axis.dot(ub)) * b.length * 0.5 +
                      std::fabs(axis.dot(vb)) * b.width * 0.5;
    const double center_b = axis.dot(cb);

    // 区间 [c-r, c+r] 不重叠 => 分离轴成立 => 不相交。
    // 相切（间隙恰为 0）不算分离，故留 kObbTolerance 的容差。
    if (center_a + ra < center_b - rb - kObbTolerance) return false;
    if (center_b + rb < center_a - ra - kObbTolerance) return false;
  }
  return true;
}

double obbDistance(const Obb2& a, const Obb2& b) {
  if (obbIntersect(a, b)) return 0.0;

  // 两个凸多边形的最小距离必在"顶点—边"对上取到（顶点—顶点是边退化的特例），
  // 因此对所有顶点到对边线段取距离，其最小值就是精确间距。
  //
  // 为什么不用 SAT 各轴的分离量：分离量只是真实距离的**下界**
  // （|a-b| >= |(a-b)·u| >= gap(u)），取它会把斜向靠近的两车判得比实际更远，
  // 而 TTC/最小间距这类安全指标必须偏保守，不能用下界糊弄。
  const std::vector<Vec2> ca = a.corners();
  const std::vector<Vec2> cb = b.corners();
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < ca.size(); ++i) {
    for (std::size_t j = 0; j < cb.size(); ++j) {
      best = std::min(best, pointSegmentDistance(ca[i], cb[j], cb[(j + 1) % cb.size()]));
      best = std::min(best, pointSegmentDistance(cb[j], ca[i], ca[(i + 1) % ca.size()]));
    }
  }
  return best == std::numeric_limits<double>::max() ? 0.0 : best;
}

bool obbContains(const Obb2& box, const Vec2& point) {
  const Vec2 local = box.pose.toLocal(point);
  return std::fabs(local.x) <= box.length * 0.5 + kObbTolerance &&
         std::fabs(local.y) <= box.width * 0.5 + kObbTolerance;
}

// ---------------------------------------------------------------------------
// SimEngine
// ---------------------------------------------------------------------------

SimEngine::SimEngine(const World& world, const VehicleModel& model)
    : world_(world), model_(model), config_() {}

SimEngine::SimEngine(const World& world, const VehicleModel& model, const Config& config)
    : world_(world), model_(model), config_(config) {}

void SimEngine::setEgo(const VehicleState& state) { ego_ = state; }

void SimEngine::addObject(const RoadObject& object) {
  // 同时放进 world_.objects()：策略只能通过 World 看到其他交通参与者，
  // 引擎持有的 dynamic_objects_ 与之一致，避免"策略看到的世界"和
  // "指标计算的世界"出现两套数据。
  dynamic_objects_.push_back(object);
  world_.objects().push_back(object);
}

void SimEngine::setPolicy(std::shared_ptr<IPolicy> policy) { policy_ = std::move(policy); }

void SimEngine::setTerminationCondition(TerminationCondition condition) {
  termination_ = std::move(condition);
}

double SimEngine::distanceTo(const RoadObject& object) const {
  return obbDistance(model_.boundingBox(ego_), object.obb());
}

double SimEngine::timeToCollision(const RoadObject& object) const {
  const Obb2 ego_box = model_.boundingBox(ego_);
  const Obb2 object_box = object.obb();

  if (obbIntersect(ego_box, object_box)) return 0.0;  // 已经接触，碰撞时间为 0

  // 自车速度沿航向（后轴参考点无侧偏）；其他物体优先用世界速度，
  // 缺省时退化为"沿航向的标量速度"。
  const Vec2 v_ego{std::cos(ego_.theta) * ego_.speed, std::sin(ego_.theta) * ego_.speed};
  Vec2 v_obj = object.velocity;
  if (v_obj.squaredNorm() < kEpsilon && std::fabs(object.speed) > kEpsilon) {
    v_obj = Vec2{std::cos(object.pose.theta), std::sin(object.pose.theta)} * object.speed;
  }

  // 在自车随动坐标系中，等价于"自车静止、物体以相对速度平移"。
  // 双方朝向在预测时域内视为不变（匀速直线外推），这是 TTC 的常规假设：
  // 它给出的是"若双方都不改变当前运动，多久会撞上"。
  const Vec2 relative_velocity = v_obj - v_ego;
  if (relative_velocity.squaredNorm() < kEpsilon) return kNoCollisionTime;  // 相对静止

  const Vec2 relative_position = object.position() - ego_.pose().position();
  // 中心距不缩短 => 相对位置与相对速度夹角 >= 90° => 永不相交。
  // 朝向固定时相交条件只取决于相对位置，故该快速判据是充分必要的。
  if (relative_position.dot(relative_velocity) >= 0.0) return kNoCollisionTime;

  const auto box_at = [&](double t) {
    return Obb2(Pose2(object.pose.x + relative_velocity.x * t,
                      object.pose.y + relative_velocity.y * t, object.pose.theta),
                object.length, object.width);
  };

  // 小步长前向搜索定位首个相交时刻，再用二分法细化到 1e-4 s 量级。
  // 碰撞集合在时间上是"前向封闭"的（一旦相交，在剩余接近过程中保持相交），
  // 因此二分法在这里是合法的。
  double t_hit = -1.0;
  for (double t = kTtcSearchStep; t <= kTtcSearchHorizon + kTimeEps; t += kTtcSearchStep) {
    if (obbIntersect(ego_box, box_at(t))) {
      t_hit = t;
      break;
    }
  }
  if (t_hit < 0.0) return kNoCollisionTime;

  double lo = std::max(0.0, t_hit - kTtcSearchStep);
  double hi = t_hit;
  for (int i = 0; i < 40; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (obbIntersect(ego_box, box_at(mid))) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

bool SimEngine::hasCollision() const {
  const Obb2 ego_box = model_.boundingBox(ego_);
  for (const RoadObject& object : dynamic_objects_) {
    if (obbIntersect(ego_box, object.obb())) return true;
  }
  for (const Obb2& obstacle : world_.obstacles()) {
    if (obbIntersect(ego_box, obstacle)) return true;
  }
  return false;
}

bool SimEngine::isOffRoad() const { return !world_.isOnRoad(ego_.pose().position()); }

void SimEngine::recordEvent(SafetyEvent event, double value, int object_id,
                            const std::string& description) {
  SafetyEventRecord record;
  record.event = event;
  record.time = time_;
  record.value = value;
  record.object_id = object_id;
  record.description = description;
  result_.events.push_back(record);
}

void SimEngine::collectMetrics() {
  const Obb2 ego_box = model_.boundingBox(ego_);

  double nearest_distance = kNoCollisionDistance;
  double nearest_ttc = kNoCollisionTime;
  int nearest_id = -1;
  int collision_id = -1;
  int ttc_id = -1;
  bool collided = false;

  for (const RoadObject& object : dynamic_objects_) {
    const Obb2& object_box = object.obb();
    if (obbIntersect(ego_box, object_box)) {
      collided = true;
      if (collision_id < 0) collision_id = object.id;
    }
    // 相交时 obbDistance 返回 0，无需单独处理
    const double distance = obbDistance(ego_box, object_box);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_id = object.id;
    }
    const double ttc = timeToCollision(object);
    if (ttc < nearest_ttc) {
      nearest_ttc = ttc;
      ttc_id = object.id;
    }
  }

  // 静态障碍也参与间距/碰撞统计（它们同样是碰撞风险的来源），但没有速度，
  // 因此不参与 TTC 计算。
  for (const Obb2& obstacle : world_.obstacles()) {
    if (obbIntersect(ego_box, obstacle)) {
      collided = true;
      if (collision_id < 0) collision_id = -1;
    }
    const double distance = obbDistance(ego_box, obstacle);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_id = -1;
    }
  }

  result_.min_distance = std::min(result_.min_distance, nearest_distance);
  result_.min_ttc = std::min(result_.min_ttc, nearest_ttc);

  const double dt = config_.step_size;

  // ---- 事件判定 ----
  // 语义约定：*_count 与 events 记录的是**危险事件（episode）次数**，不是
  // "处于危险状态的步数"。为了判断危险是否仍在持续，需要知道上一步是否也
  // 处于同类危险中，而唯一的依据就是 events 日志本身；因此危险存在时
  // **每一步都落一条记录**（保证"上一步的记录"始终存在、判断准确），
  // 持续的重复记录由 finalize() 里的 foldContinuousEvents 折叠掉。
  // 若这里改成"只在区间起点写记录"，被折叠掉的步会让上一步的记录变陈旧，
  // 判断会退化成"每两步算一次事件"，计数就会翻好几倍。
  if (collided) {
    if (!recordedInPreviousStep(result_, SafetyEvent::kCollision, time_, dt)) {
      ++result_.collision_count;  // 只在危险区间起点计数
    }
    recordEvent(SafetyEvent::kCollision, nearest_distance, collision_id,
                "自车与其他物体发生碰撞");
  }
  if (nearest_distance < config_.near_miss_distance) {
    if (!recordedInPreviousStep(result_, SafetyEvent::kNearMiss, time_, dt)) {
      ++result_.near_miss_count;
    }
    recordEvent(SafetyEvent::kNearMiss, nearest_distance, nearest_id, "最近距离低于阈值");
  }
  if (nearest_ttc < config_.low_ttc_threshold) {
    recordEvent(SafetyEvent::kLowTtc, nearest_ttc, ttc_id, "碰撞时间低于阈值");
  }
  if (isOffRoad()) {
    if (!recordedInPreviousStep(result_, SafetyEvent::kOffRoad, time_, dt)) {
      ++result_.off_road_count;
    }
    recordEvent(SafetyEvent::kOffRoad, world_.distanceToRoadEdge(ego_.pose().position()), -1,
                "自车驶出可行驶区域");
  }

  const LaneProjection projection = world_.project(ego_.pose().position());
  if (projection.valid) {
    const Lane* lane = world_.findLane(projection.lane_id);
    // 0.5 m/s 容差：限速本身是理想值，传感器与控制误差不应触发告警
    if (lane != nullptr && ego_.speed > lane->speed_limit + 0.5) {
      recordEvent(SafetyEvent::kOverSpeed, ego_.speed, -1, "车速超过所在车道限速");
    }
  }

  if (ego_.acceleration < config_.harsh_braking_threshold) {
    recordEvent(SafetyEvent::kHarshBraking, ego_.acceleration, -1, "减速度超过急刹阈值");
  }

  // completed 的判据与 finalize 保持一致：未碰撞且未驶出路面
  result_.completed = (result_.collision_count == 0 && result_.off_road_count == 0);
}

void SimEngine::step(const std::function<void(World&, double)>& object_updater) {
  const double dt = config_.step_size;
  if (!(dt > 0.0) || !std::isfinite(dt)) return;  // 非法步长：不推进也不记录

  // ---- 1. 其他交通参与者：由场景脚本按当前时刻更新 ----
  //
  // 时刻约定（做回归对比时务必留意）：object_updater 在步进**前**调用，所以
  // 本步记录与统计中的物体位置对应"步长起点时刻 t"，而同一条记录里的自车
  // 状态对应"步长终点时刻 t+dt"（自车在步内连续推进）。两者相差一个步长的
  // 相对运动量（相对速度 12m/s、dt=50ms 时约 0.6m）。这是"物体在步内取零阶
  // 保持"的离散化约定：策略看到的 ego 与物体仍同处 t 时刻（上一步结束时已把
  // 物体推进到该时刻）。改成同一时刻需要调整更新时机与指标采集，属于会影响
  // 全部下游标定的接口级变更，故此处只把约定写明。
  if (object_updater) object_updater(world_, time_);
  // 场景脚本往往直接改写 world_.objects()，这里同步到引擎内部物体表，
  // 保证策略看到的物体、指标统计的物体、被记录的历史是同一份数据。
  dynamic_objects_ = world_.objects();

  // 首拍先把初始状态记入轨迹，使轨迹从 t=0 开始（里程积分/可视化都需要）
  if (!has_previous_) {
    const TrajectoryPoint start =
        toTrajectoryPoint(ego_, model_.curvatureFromSteering(ego_.steering), time_);
    result_.time.push_back(time_);
    result_.ego_states.push_back(start);
    if (config_.record_object_history) result_.object_history.push_back(dynamic_objects_);
    previous_point_ = start;
    previous_acceleration_ = ego_.acceleration;
    previous_speed_ = ego_.speed;
    has_previous_ = true;
  }

  // ---- 2. 决策/规划：拿到指令 ----
  IPolicy::Command command;
  command.acceleration = 0.0;  // 无策略时的默认动作：滑行（保持当前速度）
  command.steering = ego_.steering;
  command.valid = true;
  if (policy_) {
    const IPolicy::Command raw = policy_->computeCommand(ego_, world_, dt);
    if (raw.valid) {
      command = raw;
    } else {
      // 接口约定 valid=false 表示"保持上一步指令"。状态里已经保存了上一步的
      // 执行结果，因此直接用当前加速度/转角作为保持值即可，无需额外历史成员。
      command.acceleration = ego_.acceleration;
      command.steering = ego_.steering;
      command.valid = true;
    }
  }

  // ---- 3. 推进自车 ----
  ego_ = model_.stepByAcceleration(ego_, command.acceleration, command.steering, dt);
  time_ += dt;

  // ---- 4. 采集指标 ----
  const double curvature = model_.curvatureFromSteering(ego_.steering);
  const TrajectoryPoint point = toTrajectoryPoint(ego_, curvature, time_);

  const double ds = (point.position() - previous_point_.position()).norm();
  result_.total_distance += ds;
  result_.average_speed = time_ > 0.0 ? result_.total_distance / time_ : 0.0;

  // 侧向加速度 a_lat = v²·κ：曲率-速度协调性的直接度量
  const double lateral_acceleration = ego_.speed * ego_.speed * curvature;
  result_.max_lateral_acceleration =
      std::max(result_.max_lateral_acceleration, std::fabs(lateral_acceleration));
  result_.max_curvature = std::max(result_.max_curvature, std::fabs(curvature));

  // jerk 用相邻两步加速度之差 / dt。这里的加速度取"实测值"（速度差 / dt）
  // 而不是模型内部记账值：实测值把速度被限幅、执行器饱和等情形自动包含进来，
  // 保证报告里的 jerk 就是乘员真实感受到的量。
  // 注意 previous_speed_ 记录的是**上一步结束**（即本步开始）的速度，
  // 因此 (v_now - previous_speed_)/dt 恰好是本步的实测加速度，
  // 与 previous_acceleration_（上一步的实测加速度）构成相邻两步之差。
  const double achieved_acceleration = (ego_.speed - previous_speed_) / dt;
  const double jerk = (achieved_acceleration - previous_acceleration_) / dt;
  result_.max_jerk = std::max(result_.max_jerk, std::fabs(jerk));

  result_.time.push_back(time_);
  result_.ego_states.push_back(point);
  if (policy_) {
    // planned_paths 是扁平的轨迹点序列（头文件定义），逐步追加即可；
    // 无规划输出时该步不追加任何点，因此"路径为空"由点的时间戳跳跃体现。
    const std::vector<Vec2> planned = policy_->lastPlannedPath();
    for (const Vec2& p : planned) {
      TrajectoryPoint tp;
      tp.x = p.x;
      tp.y = p.y;
      tp.t = time_;
      result_.planned_paths.push_back(tp);
    }
  }
  if (config_.record_object_history) result_.object_history.push_back(dynamic_objects_);

  previous_point_ = point;
  previous_speed_ = ego_.speed;              // 本步结束速度 = 下一步的起始速度
  previous_acceleration_ = achieved_acceleration;

  // ---- 5. 安全指标与事件 ----
  collectMetrics();
}

void SimEngine::finalize() {
  // 用轨迹折线长度重算总里程（与逐步累加等价，但以记录为准，
  // 保证"结果报告里的里程"和"回放出来的轨迹"严格自洽）
  double distance = 0.0;
  for (std::size_t i = 1; i < result_.ego_states.size(); ++i) {
    distance += (result_.ego_states[i].position() - result_.ego_states[i - 1].position()).norm();
  }
  result_.total_distance = distance;
  result_.average_speed = time_ > 0.0 ? distance / time_ : 0.0;
  result_.completed = (result_.collision_count == 0 && result_.off_road_count == 0);

  // 结果定稿：把持续危险折叠成语义上的"单次事件"，
  // 使 events 与已计数的 *_count 一致（详见 foldContinuousEvents 注释）。
  foldContinuousEvents(result_, config_.step_size);
}

SimulationResult SimEngine::run() {
  // 循环退出条件：超时 / 策略或场景给出的终止条件 / 配置要求碰撞即停。
  // 注意 run() 不会清空已有结果：引擎是"可续跑"的，连续两次 run() 不会
  // 把已记录的轨迹抹掉（场景层正是靠这一点在自建循环后调用 run() 收尾汇总）。
  while (time_ < config_.max_duration - kTimeEps) {
    if (termination_ && termination_(*this)) break;
    if (config_.stop_on_collision && hasCollision()) break;
    step(nullptr);
  }
  finalize();
  return result_;
}

}  // namespace adsim
