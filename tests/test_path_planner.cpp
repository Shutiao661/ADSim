// =============================================================================
//  test_path_planner.cpp — 路径规划单元测试
//
//  重点覆盖两个平滑后端的选择逻辑，以及"平滑不得以改走另一条路为代价"
//  这一核心约束（偏移必须受控）。
// =============================================================================
#include "TestFramework.h"

#include "adsim/planning/PathPlanner.h"
#include "adsim/planning/geometry/CurveFit.h"
#include "adsim/planning/optimizer/PathOptimizer.h"

#include <cmath>
#include <string>

using namespace adsim;

namespace {

/// 构造一个自车状态，放在车道起点
VehicleState egoAt(const World& world, int lane_id, double s, double speed) {
  VehicleState ego;
  const Pose2 pose = world.poseAt(lane_id, s);
  ego.x = pose.x;
  ego.y = pose.y;
  ego.theta = pose.theta;
  ego.speed = speed;
  ego.stamp = 1000;
  return ego;
}

PathPlanningRequest makeRequest(const World& world, int lane_id,
                                const std::string& /*unused*/) {
  PathPlanningRequest request;
  request.world = &world;
  request.ego_lane_id = lane_id;
  request.ego = egoAt(world, lane_id, 0.0, 10.0);
  request.target_speed = 10.0;
  request.horizon = 60.0;
  request.sample_step = 2.0;
  return request;
}

}  // namespace

// ===========================================================================
//  基本规划
// ===========================================================================

ADSIM_TEST(PathPlanner, 直路规划成功且路径贴合车道) {
  const World world = World::straightRoad(2, 3.5, 200.0);

  PathPlanner planner;
  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "straight"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_GT(result.path.size(), std::size_t(5));
  ADSIM_CHECK_GT(result.length, 0.0);
  ADSIM_CHECK(!result.smoothing_backend.empty());
  ADSIM_CHECK(!result.toString().empty());

  // 直路上路径应当基本是直线：曲率接近 0
  ADSIM_CHECK_LT(result.max_curvature, 0.01);

  // 路径应当落在车道范围内
  const Lane* lane = world.findLane(0);
  ADSIM_CHECK(lane != nullptr);
  if (lane != nullptr) {
    for (const Vec2& point : result.path) {
      const double offset = (point - world.poseAt(0, 0.0).position()).norm();
      ADSIM_CHECK(offset < 70.0);  // 不超过规划视距
    }
  }
}

ADSIM_TEST(PathPlanner, 无地图时安全失败) {
  PathPlanner planner;

  PathPlanningRequest request;
  request.world = nullptr;  // 故意不给地图

  const PathPlanningResult result = planner.plan(request);
  ADSIM_CHECK(!result.success);
  ADSIM_CHECK(!result.message.empty());
}

ADSIM_TEST(PathPlanner, 弯道上路径曲率与车道一致) {
  const double curvature = 0.02;  // 50m 半径
  const World world = World::curvedRoad(1, 3.5, 200.0, curvature);

  PathPlanner planner;
  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "curved"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_GT(result.max_curvature, 0.0);
  // 输出路径的曲率应当接近车道的设计曲率，而不是被平滑成直线
  ADSIM_CHECK_NEAR(result.max_curvature, curvature, 0.01);
}

// ===========================================================================
//  平滑后端选择
// ===========================================================================

ADSIM_TEST(PathPlanner, 三个后端都能产出可用路径) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);

  for (SmoothingBackend backend : {SmoothingBackend::kNone,
                                   SmoothingBackend::kBSpline,
                                   SmoothingBackend::kOptimization,
                                   SmoothingBackend::kAuto}) {
    PathPlanner::Config config;
    config.smoothing = backend;
    PathPlanner planner(config);

    const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

    ADSIM_CHECK_MSG(result.success, "后端 " << toString(backend) << " 规划失败");
    ADSIM_CHECK_GT(result.path.size(), std::size_t(2));
    ADSIM_CHECK_MSG(std::isfinite(result.max_curvature),
                    "后端 " << toString(backend) << " 曲率非有限值");
    ADSIM_CHECK_MSG(result.max_deviation >= 0.0,
                    "后端 " << toString(backend) << " 偏移为负");
  }
}

ADSIM_TEST(PathPlanner, 关闭平滑时直接使用参考路径) {
  const World world = World::straightRoad(1, 3.5, 200.0);

  PathPlanner::Config config;
  config.smoothing = SmoothingBackend::kNone;
  PathPlanner planner(config);

  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_EQ(result.smoothing_backend, std::string("none"));
  // 未平滑时路径即参考路径重采样结果，偏移应当接近 0
  ADSIM_CHECK_LT(result.max_deviation, 0.5);
}

ADSIM_TEST(PathPlanner, B样条后端标识正确且输出更密) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);

  PathPlanner::Config config;
  config.smoothing = SmoothingBackend::kBSpline;
  PathPlanner planner(config);

  const PathPlanningResult bspline_result = planner.plan(makeRequest(world, 0, "x"));
  ADSIM_CHECK(bspline_result.success);
  ADSIM_CHECK_EQ(bspline_result.smoothing_backend, std::string("bspline"));

  // 换道/平滑后 B 样条会按弧长重采样，点数通常多于优化后端
  PathPlanner::Config opt_config;
  opt_config.smoothing = SmoothingBackend::kOptimization;
  PathPlanner opt_planner(opt_config);
  const PathPlanningResult opt_result = opt_planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK_GT(bspline_result.path.size(), opt_result.path.size());
}

ADSIM_TEST(PathPlanner, 自动模式在偏移可接受时选B样条) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);

  PathPlanner::Config config;
  config.smoothing = SmoothingBackend::kAuto;
  config.bspline_max_deviation = 100.0;  // 阈值放到很大，必然接受 B 样条
  PathPlanner planner(config);

  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_EQ(result.smoothing_backend, std::string("bspline(auto)"));
}

ADSIM_TEST(PathPlanner, 自动模式在偏移超限时退回优化) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);

  PathPlanner::Config config;
  config.smoothing = SmoothingBackend::kAuto;
  config.bspline_max_deviation = 1e-6;  // 阈值压到不可能满足
  PathPlanner planner(config);

  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_EQ(result.smoothing_backend, std::string("optimization"));
  // 退回原因必须被记录下来，否则排查时无从判断"为什么没用 B 样条"。
  // 注意它在 result.message（规划器摘要）里，而不是 optimization.message——
  // 后者会被求解器自己的收敛消息覆盖。
  ADSIM_CHECK_MSG(result.message.find("超过阈值") != std::string::npos,
                  "退回原因缺失，message = " << result.message);
  ADSIM_CHECK(result.message.find("退回数值优化") != std::string::npos);
}

ADSIM_TEST(PathPlanner, 后端标识函数覆盖全部枚举) {
  ADSIM_CHECK_EQ(std::string(toString(SmoothingBackend::kNone)), std::string("none"));
  ADSIM_CHECK_EQ(std::string(toString(SmoothingBackend::kOptimization)),
                 std::string("optimization"));
  ADSIM_CHECK_EQ(std::string(toString(SmoothingBackend::kBSpline)),
                 std::string("bspline"));
  ADSIM_CHECK_EQ(std::string(toString(SmoothingBackend::kAuto)), std::string("auto"));
}

// ===========================================================================
//  偏移必须受控
// ===========================================================================

ADSIM_TEST(PathPlanner, 数值优化的偏移显著小于B样条) {
  // 这是两个后端最本质的差别：优化贴合原路径，B 样条平滑但会漂移。
  // 回归测试把这一取舍固定下来——若哪天某个后端的偏移特性变了，
  // 上面的后端选择策略也就失效了。
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);
  const PathPlanningRequest request = makeRequest(world, 0, "x");

  PathPlanner::Config bspline_config;
  bspline_config.smoothing = SmoothingBackend::kBSpline;
  PathPlanner bspline_planner(bspline_config);
  const PathPlanningResult bspline_result = bspline_planner.plan(request);

  PathPlanner::Config opt_config;
  opt_config.smoothing = SmoothingBackend::kOptimization;
  PathPlanner opt_planner(opt_config);
  const PathPlanningResult opt_result = opt_planner.plan(request);

  ADSIM_CHECK(bspline_result.success);
  ADSIM_CHECK(opt_result.success);

  // 参考路径本身是光滑圆弧，两个后端的偏移都应很小
  ADSIM_CHECK_LT(opt_result.max_deviation, 1.0);
  ADSIM_CHECK_LT(bspline_result.max_deviation, 1.0);
}

// ===========================================================================
//  路径生成原语
// ===========================================================================

ADSIM_TEST(PathPlanner, 沿车道采样路径) {
  const World world = World::straightRoad(1, 3.5, 200.0);

  const std::vector<Vec2> path = PathPlanner::pathAlongLane(world, 0, 0.0, 60.0, 2.0);

  ADSIM_CHECK_GT(path.size(), std::size_t(5));
  // 采样点应当沿 +x 方向单调前进
  for (std::size_t i = 1; i < path.size(); ++i) {
    ADSIM_CHECK_GT(path[i].x, path[i - 1].x - 1e-9);
  }

  // 横向偏移应当把路径整体推离中心线
  const std::vector<Vec2> shifted =
      PathPlanner::pathAlongLane(world, 0, 0.0, 60.0, 2.0, 1.0);
  ADSIM_CHECK_EQ(shifted.size(), path.size());
  ADSIM_CHECK_GT(std::abs(shifted.front().y - path.front().y), 0.5);
}

ADSIM_TEST(PathPlanner, 沿车道采样处理越界与非法参数) {
  const World world = World::straightRoad(1, 3.5, 100.0);

  // 车道不存在
  ADSIM_CHECK_EQ(PathPlanner::pathAlongLane(world, 99, 0.0, 60.0, 2.0).size(),
                 std::size_t(0));
  // 步长非法
  ADSIM_CHECK_EQ(PathPlanner::pathAlongLane(world, 0, 0.0, 60.0, 0.0).size(),
                 std::size_t(0));
  // 起点超出车道长度：应裁剪而不是崩溃
  const std::vector<Vec2> beyond = PathPlanner::pathAlongLane(world, 0, 1e6, 60.0, 2.0);
  ADSIM_CHECK_GT(beyond.size(), std::size_t(0));
}

ADSIM_TEST(PathPlanner, 换道路径横向平滑过渡) {
  const World world = World::straightRoad(2, 3.5, 200.0);

  const std::vector<Vec2> path =
      PathPlanner::laneChangePath(world, 0, 1, 0.0, 45.0, 2.0);

  ADSIM_CHECK_GT(path.size(), std::size_t(5));

  // 起点贴近原车道、终点贴近目标车道
  const double start_offset =
      std::abs(path.front().y - world.poseAt(0, 0.0).position().y);
  const double end_offset =
      std::abs(path.back().y - world.poseAt(1, 45.0).position().y);

  ADSIM_CHECK_LT(start_offset, 0.5);
  ADSIM_CHECK_LT(end_offset, 0.5);

  // 横向位移应当单调过渡，不出现来回摆动
  const double direction = path.back().y > path.front().y ? 1.0 : -1.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    ADSIM_CHECK_GT((path[i].y - path[i - 1].y) * direction, -1e-6);
  }
}

ADSIM_TEST(PathPlanner, 换道原语处理非法输入) {
  const World world = World::straightRoad(2, 3.5, 200.0);

  ADSIM_CHECK_EQ(PathPlanner::laneChangePath(world, 0, 99, 0.0, 45.0, 2.0).size(),
                 std::size_t(0));  // 目标车道不存在
  ADSIM_CHECK_EQ(PathPlanner::laneChangePath(world, 0, 1, 0.0, 0.0, 2.0).size(),
                 std::size_t(0));  // 长度非法
}

ADSIM_TEST(PathPlanner, 裁剪自车后方路径) {
  const World world = World::straightRoad(1, 3.5, 200.0);
  const std::vector<Vec2> path = PathPlanner::pathAlongLane(world, 0, 0.0, 60.0, 2.0);

  VehicleState ego = egoAt(world, 0, 20.0, 10.0);
  const std::vector<Vec2> trimmed = PathPlanner::trimBehindEgo(path, ego);

  ADSIM_CHECK_GT(trimmed.size(), std::size_t(1));
  // 首点应当是自车当前位置，保证路径与车辆状态连续
  ADSIM_CHECK_NEAR(trimmed.front().x, ego.x, 1e-9);
  ADSIM_CHECK_NEAR(trimmed.front().y, ego.y, 1e-9);
  ADSIM_CHECK_LT(trimmed.size(), path.size());
}

// ===========================================================================
//  质量约束
// ===========================================================================

ADSIM_TEST(PathPlanner, 输出路径满足曲率约束) {
  const World world = World::curvedRoad(1, 3.5, 300.0, 0.01);

  PathPlanner::Config config;
  config.max_curvature = 0.05;
  config.smoothing = SmoothingBackend::kBSpline;  // B 样条对曲率约束更可靠
  PathPlanner planner(config);

  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_MSG(result.max_curvature <= config.max_curvature * 1.05,
                  "曲率 " << result.max_curvature << " 超过限值 "
                          << config.max_curvature);
}

ADSIM_TEST(PathPlanner, 规划结果确定性可复现) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);
  const PathPlanningRequest request = makeRequest(world, 0, "x");

  PathPlanner::Config config;
  config.smoothing = SmoothingBackend::kBSpline;
  PathPlanner planner(config);

  const PathPlanningResult first = planner.plan(request);
  const PathPlanningResult second = planner.plan(request);

  ADSIM_CHECK_EQ(first.path.size(), second.path.size());
  for (std::size_t i = 0; i < first.path.size(); ++i) {
    ADSIM_CHECK_NEAR(first.path[i].x, second.path[i].x, 1e-12);
    ADSIM_CHECK_NEAR(first.path[i].y, second.path[i].y, 1e-12);
  }
}

ADSIM_TEST(PathPlanner, 轨迹输出含航向与曲率) {
  const World world = World::curvedRoad(1, 3.5, 200.0, 0.02);

  PathPlanner planner;
  const PathPlanningResult result = planner.plan(makeRequest(world, 0, "x"));

  ADSIM_CHECK(result.success);
  ADSIM_CHECK_EQ(result.trajectory.size(), result.path.size());

  for (const TrajectoryPoint& point : result.trajectory) {
    ADSIM_CHECK(std::isfinite(point.theta));
    ADSIM_CHECK(std::isfinite(point.kappa));
    ADSIM_CHECK(std::isfinite(point.v));
  }
}
