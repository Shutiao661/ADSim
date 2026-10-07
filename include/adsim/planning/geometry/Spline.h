// =============================================================================
//  Spline.h — 三次 B 样条曲线与路径平滑
//
//  对应简历中"利用曲线拟合解决弯道处路径不平滑、曲率突变"的问题：
//  以离散路点为输入，通过最小二乘拟合出 C2 连续的三次 B 样条，再迭代调整
//  控制点使最大曲率收敛到约束范围内，从而消除曲率突变。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <vector>

namespace adsim {
namespace geometry {

/// 三次（k=3）B 样条曲线。默认采用 clamped 节点向量使曲线通过首末控制点。
class CubicBSpline {
 public:
  /// @param control_points 控制点，至少 4 个
  /// @param clamped true 时首末控制点被曲线穿过（节点向量端点重复度为 4）
  explicit CubicBSpline(const std::vector<Vec2>& control_points, bool clamped = true);

  /// 曲线求值，u ∈ [0, 1]
  Vec2 evaluate(double u) const;

  /// 一阶导数 dP/du
  Vec2 derivative(double u) const;

  /// 二阶导数 d²P/du²
  Vec2 secondDerivative(double u) const;

  /// 曲线在 u 处的曲率 κ = |x'y'' - y'x''| / (x'² + y'²)^1.5
  double curvature(double u) const;

  /// 弧长数值近似（复合 Simpson 法）
  double approximateLength(std::size_t samples = 200) const;

  /// 等参数采样为轨迹，含航向与曲率
  Trajectory sample(std::size_t count) const;

  /// 按弧长均匀采样为轨迹
  Trajectory sampleByArcLength(std::size_t count) const;

  const std::vector<Vec2>& controlPoints() const { return control_points_; }
  const std::vector<double>& knots() const { return knots_; }
  bool clamped() const { return clamped_; }

  /// 在参数 u 处求值（指定阶数，0 为位置）
  Vec2 evaluateDerivative(double u, int order) const;

 private:

  std::vector<Vec2> control_points_;
  std::vector<double> knots_;
  bool clamped_{true};
};

// ---------------------------------------------------------------------------
//  拟合
// ---------------------------------------------------------------------------

struct BSplineFitResult {
  std::vector<Vec2> control_points;
  double rms_error{0.0};       ///< 拟合曲线到原始路点的均方根误差 (m)
  double max_error{0.0};       ///< 最大偏差 (m)
  std::size_t iterations{0};   ///< 实际迭代次数
};

/// 最小二乘拟合三次 B 样条。
///
/// 采用"参数化 → 最小二乘求解控制点 → 重参数化"的迭代策略（类似 Piegl & Tiller
/// 的全局曲线插值），iterations 为最大迭代轮数。
///
/// @param waypoints     原始离散路点，至少 2 个
/// @param control_count 期望的控制点数量，至少 4；不足时自动提升
/// @param iterations    重参数化迭代轮数
BSplineFitResult fitCubicBSpline(const std::vector<Vec2>& waypoints,
                                 std::size_t control_count,
                                 std::size_t iterations = 3);

// ---------------------------------------------------------------------------
//  平滑
// ---------------------------------------------------------------------------

/// 由离散路点生成满足曲率约束的平滑轨迹。
///
/// 流程：路点抽稀 → B 样条拟合 → 检查最大曲率 → 超限则增加控制点重拟合 →
/// 仍超限则对控制点做局部平滑，直至满足约束或达到迭代上限。
///
/// @param waypoints     原始路点
/// @param max_kappa     允许的最大曲率 (1/m)，如 0.2 对应约 5m 转弯半径
/// @param sample_count  输出轨迹采样点数
/// @return 平滑后的轨迹；输入不足 2 点时返回空
Trajectory smoothPathWithCurvatureLimit(const std::vector<Vec2>& waypoints,
                                        double max_kappa,
                                        std::size_t sample_count = 200);

/// 由折线路径计算每个点的航向与曲率，生成轨迹（速度为标称值）
Trajectory pathToTrajectory(const std::vector<Vec2>& path, double nominal_speed = 0.0);

/// 计算轨迹的最大曲率绝对值
double maxCurvature(const Trajectory& trajectory);

}  // namespace geometry
}  // namespace adsim
