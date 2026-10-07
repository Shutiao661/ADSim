// =============================================================================
//  PathOptimizer.h — 基于数值优化的路径平滑
//
//  对应"弯道处路径不平滑、曲率突变"这一问题：上游模块（搜索、采样、拟合）
//  给出的路径往往存在相邻点方向跳变，直接跟踪会导致方向盘抖动、横向加速度
//  不连续。
//
//  本模块把它建成一个带约束的非线性最小二乘问题：
//
//      min_{P₀..Pₙ}   Σ w_smooth ·‖P_{i-1} − 2P_i + P_{i+1}‖²   平滑项
//                   + Σ w_dev    ·‖P_i − P_i⁰‖²                  贴合项
//                   + Σ w_len    ·‖P_{i+1} − P_i‖²               长度项
//                   + Σ w_curv   · max(0, |κ_i| − κ_max)²        曲率约束（铰链惩罚）
//
//  其中曲率 κ_i 用 Menger 曲率（三点外接圆）离散估计。
//  三项权重互相牵制：平滑项单独作用会把路径缩成一条直线，贴合项拉住它，
//  长度项防止路径点向两端聚集。曲率项是铰链式的——只有超限才产生惩罚，
//  因此不会无谓地改变本就平缓的路径。
//
//  求解后端默认为工程自带的 LM 求解器；以 ADSIM_WITH_CERES 构建时
//  自动切换到 Ceres Solver，代价函数定义完全不变。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <string>
#include <vector>

namespace adsim {

struct PathOptimizerOptions {
  /// 允许的最大曲率 (1/m)。0.2 对应约 5m 转弯半径，是城市道路的典型上限。
  double max_curvature{0.2};

  /// 平滑项权重：越大路径越平直（也越偏离原始路径）
  double smoothness_weight{1.0};
  /// 贴合项权重：越大越贴近原始路径
  double deviation_weight{5.0};
  /// 长度项权重：抑制路径点在局部聚集
  double length_weight{0.5};
  /// 曲率约束权重：越大越快把超限处压回阈值内
  double curvature_weight{20.0};

  /// 参考点间距，长度项以此为期望值 (m)
  double reference_spacing{2.0};

  int max_iterations{200};
  /// 固定首末点（通常起点是当前位姿、终点是目标点，不应被优化移动）
  bool fix_endpoints{true};
  /// 输出详细求解过程
  bool verbose{false};

  /// 约束满足的最大外层次数。
  ///
  /// 惩罚法只能渐进逼近约束：单轮固定的曲率权重未必足以把路径压进限值内
  /// （权重过大会让问题病态、收敛变慢）。因此在外层逐轮增大曲率权重，
  /// 直到满足约束或达到轮数上限——这是罚函数法处理约束的标准做法。
  int max_constraint_rounds{5};
  /// 每轮曲率权重的放大倍数
  double constraint_weight_growth{6.0};
  /// 判定"已满足约束"的相对容差
  double constraint_tolerance{0.02};
};

struct PathOptimizationReport {
  bool converged{false};
  int iterations{0};
  std::string message;
  std::string backend;          ///< "builtin-lm" 或 "ceres"

  double initial_cost{0.0};
  double final_cost{0.0};

  double initial_max_curvature{0.0};
  double final_max_curvature{0.0};
  /// 排除首末各 2 点后的最大曲率——固定端点附近不可优化，需单独度量
  double final_interior_max_curvature{0.0};
  double initial_max_curvature_rate{0.0};
  double final_max_curvature_rate{0.0};
  double max_deviation{0.0};    ///< 优化后相对原路径的最大偏移 (m)
  int constraint_rounds{0};     ///< 外层约束迭代的实际轮数
  double curvature_limit{0.0};  ///< 本次优化设定的曲率上限

  double elapsed_ms{0.0};

  /// 曲率是否已满足约束
  bool satisfiesCurvatureLimit() const;

  std::string toString() const;
};

class PathOptimizer {
 public:
  PathOptimizer();
  explicit PathOptimizer(const PathOptimizerOptions& options);

  /// 优化路径。
  /// @param initial_path 输入路径，至少 3 个点
  /// @param report       非空时填充优化过程统计
  /// @return 优化后的路径；输入不足 3 点时原样返回
  std::vector<Vec2> optimize(const std::vector<Vec2>& initial_path,
                             PathOptimizationReport* report = nullptr) const;

  const PathOptimizerOptions& options() const { return options_; }
  PathOptimizerOptions& options() { return options_; }

  /// 当前构建是否使用 Ceres 后端
  static bool usingCeres();

  // -------------------------------------------------------------------------
  // 离散几何量（供外部复用与测试）
  // -------------------------------------------------------------------------

  /// Menger 曲率：三点确定的外接圆曲率半径的倒数。
  /// 三点共线或重合时返回 0。
  static double mengerCurvature(const Vec2& a, const Vec2& b, const Vec2& c);

  /// 路径各点曲率（首末点用相邻段复制填充）
  static std::vector<double> computeCurvatures(const std::vector<Vec2>& path);

  /// 最大曲率绝对值
  static double maxAbsCurvature(const std::vector<Vec2>& path);

  /// 排除首末各 margin 个点后的最大曲率。
  ///
  /// 首末点通常被固定（起点是当前位姿、终点是目标点），其邻域的曲率由固定点
  /// 与本就被噪声污染的原始路径共同决定，再多权重也压不下去。因此"整体最大
  /// 曲率"会把这段无法优化的部分算进来，无法反映优化器的真实效果——
  /// 需要单独看这条指标。
  static double maxAbsCurvature(const std::vector<Vec2>& path, std::size_t margin);

  /// 最大曲率变化率 |dκ/ds|，衡量曲率突变的剧烈程度
  static double maxCurvatureRate(const std::vector<Vec2>& path);

 private:
  PathOptimizerOptions options_;
};

}  // namespace adsim
