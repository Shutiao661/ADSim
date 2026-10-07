#include "adsim/planning/PathPlanner.h"

#include "adsim/planning/geometry/CurveFit.h"
#include "adsim/planning/geometry/Delaunay.h"
#include "adsim/planning/geometry/Spline.h"
#include "adsim/planning/optimizer/PathOptimizer.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace adsim {

namespace {

/// 五次多项式横向过渡：在 [0,1] 上满足 s(0)=0, s(1)=1，且两端一阶、二阶导均为 0。
///
/// 用五次而非三次是必要的：三次多项式只能保证首末位移与速度连续，
/// 二阶导（横向加速度）在端点会突变，乘客能明显感到"换道开始时被甩一下"。
double quinticSmoothstep(double t) {
  const double u = clamp(t, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

/// 计算路径相对参考折线的最大偏移。
/// 两侧点数往往不同（B 样条会重采样），因此必须用点到折线的距离，
/// 按索引逐点比会得出毫无意义的巨大数值。
double maxDeviationFrom(const std::vector<Vec2>& path,
                        const std::vector<Vec2>& reference) {
  double worst = 0.0;
  for (const Vec2& point : path) {
    worst = std::max(worst, geometry::distanceToPolyline(reference, point));
  }
  return worst;
}

/// 用 B 样条拟合参考路径
std::vector<Vec2> smoothWithBSpline(const std::vector<Vec2>& waypoints,
                                    const PathPlanner::Config& config) {
  std::size_t samples = config.bspline_samples;
  if (samples == 0) {
    // 按采样步长推算输出点数：B 样条是连续曲线，采样过疏会丢掉形状细节
    const double length = geometry::polylineLength(waypoints);
    const double step = std::max(config.sample_step, 0.1);
    samples = std::max<std::size_t>(32, static_cast<std::size_t>(length / step) * 4);
  }

  const Trajectory trajectory = geometry::smoothPathWithCurvatureLimit(
      waypoints, config.max_curvature, samples);

  std::vector<Vec2> path;
  path.reserve(trajectory.size());
  for (const TrajectoryPoint& point : trajectory) {
    path.push_back({point.x, point.y});
  }
  return path;
}

/// 应用配置指定的平滑后端，写入 result.path 与 result.smoothing_backend
void applySmoothing(const PathPlanner::Config& config,
                    const std::vector<Vec2>& resampled,
                    PathPlanningResult& result) {
  const auto use_optimization = [&]() {
    PathOptimizerOptions options = config.optimizer;
    options.max_curvature = config.max_curvature;
    PathOptimizer optimizer(options);
    result.path = optimizer.optimize(resampled, &result.optimization);
    result.smoothing_backend = "optimization";
  };

  switch (config.smoothing) {
    case SmoothingBackend::kNone:
      result.path = resampled;
      result.smoothing_backend = "none";
      return;

    case SmoothingBackend::kBSpline: {
      std::vector<Vec2> path = smoothWithBSpline(resampled, config);
      if (path.size() >= 2) {
        result.path = std::move(path);
        result.smoothing_backend = "bspline";
      } else {
        // 拟合失败（点数不足或数值退化）时退回参考路径，而不是给出空路径
        result.path = resampled;
        result.smoothing_backend = "none(bspline退化)";
      }
      return;
    }

    case SmoothingBackend::kAuto: {
      std::vector<Vec2> path = smoothWithBSpline(resampled, config);
      if (path.size() >= 2) {
        const double deviation = maxDeviationFrom(path, resampled);
        if (deviation <= config.bspline_max_deviation) {
          result.path = std::move(path);
          result.smoothing_backend = "bspline(auto)";
          return;
        }

        // 偏移超限则退回优化。这里判据用偏移而非曲率，是因为实测发现
        // B 样条的曲率本来就容易满足约束，真正会出问题的是它贴不住原始路径——
        // 2.4m 的偏移对 3.5m 宽的车道足以导致压线。
        //
        // 原因写进 result.message（规划器摘要）而不是 optimization.message：
        // 后者会被 use_optimization() 内部用求解器自己的消息覆盖掉，
        // 早期版本正是因为这一点把退回原因丢了，排查时无从判断为什么没用 B 样条。
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(2);
        oss << "；自动模式：B 样条偏移 " << deviation << " m 超过阈值 "
            << config.bspline_max_deviation << " m，已退回数值优化";

        use_optimization();
        result.message += oss.str();
        return;
      }
      use_optimization();
      return;
    }

    case SmoothingBackend::kOptimization:
    default:
      use_optimization();
      return;
  }
}

/// 计算折线上各点的航向
std::vector<double> computeHeadings(const std::vector<Vec2>& path) {
  std::vector<double> headings(path.size(), 0.0);
  if (path.size() < 2) return headings;

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    headings[i] = (path[i + 1] - path[i]).heading();
  }
  headings.back() = headings[path.size() - 2];

  // 对内部点取前后段的平均方向，减小离散化带来的抖动
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    headings[i] = normalizeAngle(0.5 * (headings[i - 1] + headings[i]));
  }
  return headings;
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

const char* toString(SmoothingBackend backend) {
  switch (backend) {
    case SmoothingBackend::kNone: return "none";
    case SmoothingBackend::kOptimization: return "optimization";
    case SmoothingBackend::kBSpline: return "bspline";
    case SmoothingBackend::kAuto: return "auto";
  }
  return "unknown";
}

PathPlanner::PathPlanner() = default;
PathPlanner::PathPlanner(const Config& config) : config_(config) {}

// ---------------------------------------------------------------------------
// 路径生成原语
// ---------------------------------------------------------------------------

std::vector<Vec2> PathPlanner::pathAlongLane(const World& world, int lane_id,
                                             double s_start, double horizon, double step,
                                             double lateral_offset) {
  std::vector<Vec2> path;
  const Lane* lane = world.findLane(lane_id);
  if (lane == nullptr || step <= 0.0) {
    return path;
  }

  const double lane_length = lane->length();
  const double s_begin = clamp(s_start, 0.0, lane_length);
  const double s_end = std::min(s_begin + std::max(horizon, step), lane_length);

  for (double s = s_begin; s <= s_end + 1e-9; s += step) {
    const Pose2 pose = world.poseAt(lane_id, s);
    // 横向偏移沿法向施加：左正右负
    const Vec2 normal{-std::sin(pose.theta), std::cos(pose.theta)};
    path.push_back(pose.position() + normal * lateral_offset);
  }

  if (path.size() < 2 && lane_length > 0.0) {
    path.push_back(world.poseAt(lane_id, s_begin).position());
    path.push_back(world.poseAt(lane_id, std::min(s_begin + step, lane_length)).position());
  }
  return path;
}

std::vector<Vec2> PathPlanner::laneChangePath(const World& world, int from_lane_id,
                                              int to_lane_id, double s_start, double length,
                                              double step) {
  std::vector<Vec2> path;
  const Lane* from_lane = world.findLane(from_lane_id);
  const Lane* to_lane = world.findLane(to_lane_id);
  if (from_lane == nullptr || to_lane == nullptr || step <= 0.0 || length <= 0.0) {
    return path;
  }

  const auto count = static_cast<std::size_t>(length / step) + 1;
  path.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const double progress = static_cast<double>(i) / static_cast<double>(count - 1);
    const double s = s_start + progress * length;

    const Pose2 from_pose = world.poseAt(from_lane_id, s);
    const Pose2 to_pose = world.poseAt(to_lane_id, s);

    // 横向按五次多项式过渡，纵向直接取两车道中心线的加权位置
    const double blend = quinticSmoothstep(progress);
    const Vec2 position = from_pose.position() * (1.0 - blend) + to_pose.position() * blend;

    // 航向同样做插值，避免位置过渡而航向仍指向原车道
    const double heading =
        normalizeAngle(from_pose.theta + normalizeAngle(to_pose.theta - from_pose.theta) * blend);

    (void)heading;  // 航向由后续的相邻点差分统一计算，这里不单独使用
    path.push_back(position);
  }

  return path;
}

std::vector<Vec2> PathPlanner::trimBehindEgo(const std::vector<Vec2>& path,
                                             const VehicleState& ego) {
  if (path.size() < 2) return path;

  const Vec2 ego_position = ego.pose().position();
  const Vec2 forward{std::cos(ego.theta), std::sin(ego.theta)};

  // 找到第一个位于自车前方的点作为起点
  std::size_t start = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if ((path[i] - ego_position).dot(forward) > 0.0) {
      start = i;
      break;
    }
    start = i;
  }

  std::vector<Vec2> trimmed;
  trimmed.reserve(path.size() - start + 1);

  // 自车当前位置作为路径起点，保证路径与车辆状态连续
  trimmed.push_back(ego_position);
  for (std::size_t i = start; i < path.size(); ++i) {
    trimmed.push_back(path[i]);
  }
  return trimmed;
}

// ---------------------------------------------------------------------------
// 规划主流程
// ---------------------------------------------------------------------------

PathPlanningResult PathPlanner::plan(const PathPlanningRequest& request) {
  PathPlanningResult result;

  if (request.world == nullptr) {
    result.message = "未提供地图，无法规划";
    return result;
  }

  const World& world = *request.world;

  // ---- 1. 确定自车在当前车道上的位置 ----
  int reference_lane = request.ego_lane_id;
  if (reference_lane < 0) {
    const LaneProjection projection = world.project(request.ego.pose().position());
    if (!projection.valid) {
      result.message = "自车不在任何车道附近，无法规划";
      return result;
    }
    reference_lane = projection.lane_id;
  }

  const LaneProjection ego_projection =
      world.projectOnLane(reference_lane, request.ego.pose().position());
  const double s_start = ego_projection.valid ? ego_projection.s : 0.0;

  // ---- 2. 生成参考路径 ----
  const bool changing_lane = request.target_lane_id >= 0 &&
                             request.target_lane_id != reference_lane;

  if (changing_lane) {
    const Lane* target_lane = world.findLane(request.target_lane_id);
    if (target_lane == nullptr) {
      result.message = "目标车道不存在";
      return result;
    }
    result.reference_path =
        laneChangePath(world, reference_lane, request.target_lane_id, s_start,
                       config_.lane_change_length, request.sample_step);
  } else {
    result.reference_path =
        pathAlongLane(world, reference_lane, s_start, request.horizon,
                      request.sample_step, request.lateral_offset);
  }

  if (result.reference_path.size() < 3) {
    result.message = "参考路径点数不足，无法规划";
    return result;
  }

  // ---- 3. 可选：绕障修正 ----
  // 仅在确实存在障碍且提供了足够多的障碍点时启用。
  // 障碍点太少时 Voronoi 图会退化，生成的骨架反而不如直接沿车道行驶。
  if (request.avoid_obstacles &&
      request.obstacle_points.size() >= config_.min_obstacle_points &&
      config_.enable_voronoi_detour) {
    geometry::VoronoiRoadmap roadmap(request.obstacle_points);

    const Vec2 start = request.ego.pose().position();
    const Vec2 goal = result.reference_path.back();

    const std::vector<Vec2> detour = roadmap.search(start, goal);

    // 只在绕行结果确实可通行时才采纳：路径过短或间隙过小说明
    // 骨架被障碍物挤死，此时强行绕行会得到一条贴着障碍物的危险路径
    if (detour.size() >= 3 && roadmap.minClearanceAlong(detour) > 0.5) {
      result.reference_path = detour;
      result.used_detour = true;
    }
  }

  // ---- 4. 平滑 ----
  result.path = result.reference_path;
  result.smoothing_backend = "none";

  if (config_.enable_optimization && result.reference_path.size() >= 5) {
    // 先按弧长重采样，使点间距均匀——两个后端的长度项与曲率项都依赖均匀间距
    const std::vector<Vec2> resampled =
        geometry::resampleByArcLength(result.reference_path, config_.sample_step);

    if (resampled.size() >= 5) {
      applySmoothing(config_, resampled, result);
      result.max_deviation = maxDeviationFrom(result.path, resampled);
    } else {
      result.optimization.message = "重采样后点数不足，跳过平滑";
    }
  } else {
    result.optimization.message = "已禁用平滑";
  }

  if (result.path.size() < 2) {
    result.message = "平滑后路径点数不足";
    return result;
  }

  // ---- 5. 计算几何量 ----
  result.max_curvature = PathOptimizer::maxAbsCurvature(result.path);
  result.length = geometry::polylineLength(result.path);

  const std::vector<double> headings = computeHeadings(result.path);
  const std::vector<double> curvatures = PathOptimizer::computeCurvatures(result.path);

  result.trajectory.resize(result.path.size());
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    TrajectoryPoint& point = result.trajectory[i];
    point.x = result.path[i].x;
    point.y = result.path[i].y;
    point.theta = headings[i];
    point.kappa = curvatures[i];
    point.v = request.target_speed;  // 速度剖面由 SpeedPlanner 单独负责
    point.a = 0.0;
    point.t = 0.0;
  }

  result.success = true;
  // 追加而非赋值：平滑阶段可能已经写入了后端选择的原因（如自动模式退回），
  // 直接覆盖会把这些诊断信息丢掉
  result.message = (changing_lane ? "换道路径规划完成" : "车道内路径规划完成") + result.message;
  if (result.used_detour) {
    result.message += "（含绕障）";
  }

  return result;
}

std::string PathPlanningResult::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);

  oss << "路径规划" << (success ? "成功" : "失败") << ": " << message << "\n";
  if (!success) return oss.str();

  oss << "  平滑后端    : " << smoothing_backend << "\n";
  oss << "  路径点数    : " << path.size() << "\n";
  oss.precision(2);
  oss << "  路径长度    : " << length << " m\n";
  oss.precision(4);
  oss << "  最大曲率    : " << max_curvature << " 1/m";
  if (max_curvature > 1e-6) {
    oss.precision(2);
    oss << "  (最小转弯半径 " << 1.0 / max_curvature << " m)";
  }
  oss << "\n";
  oss.precision(3);
  oss << "  相对参考偏移: " << max_deviation << " m\n";
  if (used_detour) {
    oss << "  已启用绕障\n";
  }
  oss.precision(4);
  oss << "  参考路径曲率: " << PathOptimizer::maxAbsCurvature(reference_path) << " 1/m\n";
  return oss.str();
}

}  // namespace adsim
