// =============================================================================
//  PathPlanner.h — 路径规划
//
//  把决策模块给出的"驾驶意图"（保持车道 / 换道 / 绕障）展开成一条具体的、
//  满足车辆运动学约束的路径。流程：
//
//      车道中心线采样  ──►  绕障修正  ──►  曲线拟合  ──►  数值优化  ──►  轨迹
//         (几何)          (Voronoi)     (B 样条)      (LM 求解器)
//
//  三个环节各自解决一类问题：
//    * 车道采样   —— 保证路径整体贴合道路拓扑
//    * Voronoi    —— 在有障碍时给出远离障碍物的绕行骨架
//    * 拟合+优化  —— 消除离散采样带来的曲率突变，把曲率压进车辆可行范围
//
//  最后一步是简历中"解决弯道处路径不平滑、曲率突变"的直接落点。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/planning/optimizer/PathOptimizer.h"
#include "adsim/sim/World.h"

#include <string>
#include <vector>

namespace adsim {

struct PathPlanningRequest {
  VehicleState ego;
  const World* world{nullptr};

  /// 目标车道；<0 表示保持当前车道
  int target_lane_id{-1};
  /// 自车当前所在车道
  int ego_lane_id{-1};

  double target_speed{10.0};
  /// 相对目标车道中心线的横向偏移（用于绕障时的横向避让）
  double lateral_offset{0.0};
  /// 规划视距 (m)
  double horizon{60.0};
  /// 沿路径的采样步长 (m)
  double sample_step{2.0};

  /// 障碍物离散点集；avoid_obstacles 为 true 时用于生成绕行骨架
  std::vector<Vec2> obstacle_points;
  bool avoid_obstacles{false};
};

/// 路径平滑后端
enum class SmoothingBackend {
  kNone,          ///< 不做平滑，直接使用参考路径
  kOptimization,  ///< 数值优化（LM）：贴合优先，偏移小但慢
  kBSpline,       ///< B 样条拟合：平滑优先，快且曲率好，但偏移大
  kAuto,          ///< 自动：先试 B 样条，偏移超限则退回数值优化
};

const char* toString(SmoothingBackend backend);

struct PathPlanningResult {
  bool success{false};

  std::vector<Vec2> reference_path;  ///< 未经平滑的参考路径
  std::vector<Vec2> path;            ///< 平滑后的路径
  Trajectory trajectory;             ///< 含航向、曲率、速度

  PathOptimizationReport optimization;

  double max_curvature{0.0};
  double length{0.0};
  /// 平滑结果相对参考路径的最大偏移 (m)。
  /// 这是选后端时最关键的指标——平滑不能以"改走另一条路"为代价。
  double max_deviation{0.0};
  bool used_detour{false};
  std::string smoothing_backend;  ///< 实际生效的后端标识
  std::string message;

  std::string toString() const;
};

class PathPlanner {
 public:
  struct Config {
    double sample_step{2.0};
    /// 路径允许的最大曲率 (1/m)
    double max_curvature{0.2};
    /// 换道过程的纵向长度 (m)
    double lane_change_length{45.0};
    /// 是否启用路径平滑
    bool enable_optimization{true};
    /// 是否在有障碍时启用 Voronoi 绕行
    bool enable_voronoi_detour{true};
    /// 触发绕障的障碍点数量下限
    std::size_t min_obstacle_points{8};

    // -------------------------------------------------------------------------
    // 平滑后端选择
    //
    // 两个后端是**真实的工程取舍**，不是谁替代谁。实测（41 点带噪圆弧）：
    //
    //              最大曲率   曲率变化率   最大偏移    耗时
    //   B 样条      0.040      0.0097      2.431 m    2.1 ms
    //   数值优化    0.057      0.0707      0.313 m  106.5 ms
    //
    // B 样条在平滑度与速度上压倒性占优，但偏移大 8 倍——2.4m 的偏移
    // 对 3.5m 宽的车道意味着会压线。因此：
    //   * 车道内行驶、需要精确贴合 → 用数值优化
    //   * 生成一条全新的平滑路径（如弯曲道路的全局路径） → 用 B 样条
    //   * 不确定 → 用 kAuto，由偏移阈值自动裁决
    // -------------------------------------------------------------------------
    SmoothingBackend smoothing{SmoothingBackend::kOptimization};

    /// B 样条拟合的控制点数。太少贴不住形状，太多会引入局部抖动。
    std::size_t bspline_control_points{12};
    /// B 样条输出采样点数；0 表示按路径长度与 sample_step 自动推算
    std::size_t bspline_samples{0};
    /// kAuto 模式下允许的 B 样条最大偏移 (m)，超过则退回数值优化
    double bspline_max_deviation{0.8};

    PathOptimizerOptions optimizer;
  };

  PathPlanner();
  explicit PathPlanner(const Config& config);

  PathPlanningResult plan(const PathPlanningRequest& request);

  // -------------------------------------------------------------------------
  // 路径生成原语（公开以便单独测试与复用）
  // -------------------------------------------------------------------------

  /// 沿车道中心线采样一条路径，可附加横向偏移。
  /// s_start 为起始弧长，向前延伸 horizon 米。
  static std::vector<Vec2> pathAlongLane(const World& world, int lane_id, double s_start,
                                         double horizon, double step,
                                         double lateral_offset = 0.0);

  /// 生成换道路径：横向位置按平滑曲线从当前车道过渡到目标车道。
  /// 用五次多项式保证首末的横向位移、速度、加速度都连续——
  /// 直线插值会让换道起止处出现横向加速度突变。
  static std::vector<Vec2> laneChangePath(const World& world, int from_lane_id,
                                          int to_lane_id, double s_start,
                                          double length, double step);

  /// 把参考路径投影到自车前方，去掉已经驶过的部分
  static std::vector<Vec2> trimBehindEgo(const std::vector<Vec2>& path,
                                         const VehicleState& ego);

  const Config& config() const { return config_; }
  Config& config() { return config_; }

 private:
  Config config_;
};

}  // namespace adsim
