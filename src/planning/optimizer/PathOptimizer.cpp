#include "adsim/planning/optimizer/PathOptimizer.h"

#include "adsim/planning/optimizer/NonlinearSolver.h"

#if defined(ADSIM_HAS_CERES)
#include "adsim/planning/optimizer/CeresPathOptimizer.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

namespace adsim {

namespace {

/// 曲率铰链惩罚相对 κ_max 的软化带宽，避免阈值处的梯度突变引起求解器震荡
constexpr double kHingeSoftening = 1e-3;

// ---------------------------------------------------------------------------
// 平滑残差：‖P_{i-1} − 2P_i + P_{i+1}‖
//
// 二阶差分是曲率的离散近似。雅可比是常数矩阵，直接给出即可。
// ---------------------------------------------------------------------------
class SmoothnessResidual : public ResidualBlock {
 public:
  explicit SmoothnessResidual(double weight) : weight_(std::sqrt(std::max(weight, 0.0))) {}

  int residualDimension() const override { return 2; }
  std::vector<int> parameterBlockSizes() const override { return {2, 2, 2}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    const double* prev = params[0];
    const double* curr = params[1];
    const double* next = params[2];

    residuals[0] = weight_ * (prev[0] - 2.0 * curr[0] + next[0]);
    residuals[1] = weight_ * (prev[1] - 2.0 * curr[1] + next[1]);

    if (jacobian == nullptr) return;

    // 行主序 2×6：对 (prev.x, prev.y, curr.x, curr.y, next.x, next.y) 求导
    std::fill(jacobian, jacobian + 12, 0.0);
    jacobian[0] = weight_;    // ∂r0/∂prev.x
    jacobian[2] = -2.0 * weight_;  // ∂r0/∂curr.x
    jacobian[4] = weight_;    // ∂r0/∂next.x

    jacobian[7] = weight_;    // ∂r1/∂prev.y
    jacobian[9] = -2.0 * weight_;  // ∂r1/∂curr.y
    jacobian[11] = weight_;   // ∂r1/∂next.y
  }

 private:
  double weight_;
};

// ---------------------------------------------------------------------------
// 贴合残差：(P_i − P_i⁰)，把路径拉回原始形状
// ---------------------------------------------------------------------------
class DeviationResidual : public ResidualBlock {
 public:
  DeviationResidual(double weight, double reference_x, double reference_y)
      : weight_(std::sqrt(std::max(weight, 0.0))),
        reference_{reference_x, reference_y} {}

  int residualDimension() const override { return 2; }
  std::vector<int> parameterBlockSizes() const override { return {2}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    const double* point = params[0];

    residuals[0] = weight_ * (point[0] - reference_[0]);
    residuals[1] = weight_ * (point[1] - reference_[1]);

    if (jacobian == nullptr) return;

    std::fill(jacobian, jacobian + 4, 0.0);
    jacobian[0] = weight_;  // ∂r0/∂x
    jacobian[3] = weight_;  // ∂r1/∂y
  }

 private:
  double weight_;
  double reference_[2];
};

// ---------------------------------------------------------------------------
// 长度残差：惩罚相邻点间距偏离参考间距
//
//  没有这一项时，平滑项会把相邻点越拉越近（间距趋零不增加二阶差分代价），
//  结果是一串挤在一起的重复点，参数化随之退化。
// ---------------------------------------------------------------------------
class LengthResidual : public ResidualBlock {
 public:
  LengthResidual(double weight, double reference_spacing)
      : weight_(std::sqrt(std::max(weight, 0.0))),
        reference_spacing_(reference_spacing) {}

  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {2, 2}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    const double* a = params[0];
    const double* b = params[1];

    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double length = std::sqrt(dx * dx + dy * dy);

    residuals[0] = weight_ * (length - reference_spacing_);

    if (jacobian == nullptr) return;

    if (length < 1e-9) {
      // 两点重合时长度对坐标不可导，给一个零梯度让其他项先把它们推开
      std::fill(jacobian, jacobian + 4, 0.0);
      return;
    }

    const double inv = weight_ / length;
    jacobian[0] = -inv * dx;  // ∂r/∂a.x
    jacobian[1] = -inv * dy;  // ∂r/∂a.y
    jacobian[2] = inv * dx;   // ∂r/∂b.x
    jacobian[3] = inv * dy;   // ∂r/∂b.y
  }

 private:
  double weight_;
  double reference_spacing_;
};

// ---------------------------------------------------------------------------
// 曲率残差：max(0, |κ| − κ_max) 的铰链惩罚
//
//  κ 是三点坐标的有理函数，解析雅可比冗长且极易出错；这里在块内用中心差分。
//  代价是每次迭代多出若干次残差求值，但对于数十个点的路径完全可以接受，
//  换来的是实现正确性与可读性——这个权衡在数值优化里通常是划算的。
// ---------------------------------------------------------------------------
class CurvatureResidual : public ResidualBlock {
 public:
  CurvatureResidual(double weight, double max_curvature, double softening)
      : weight_(std::sqrt(std::max(weight, 0.0))),
        max_curvature_(max_curvature),
        softening_(softening) {}

  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {2, 2, 2}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    // 把三个参数块拼成连续坐标，便于差分扰动
    double coords[6];
    for (int b = 0; b < 3; ++b) {
      coords[2 * b] = params[b][0];
      coords[2 * b + 1] = params[b][1];
    }

    residuals[0] = weight_ * hingeExcess(coords);

    if (jacobian == nullptr) return;

    // 中心差分；步长随坐标量级缩放
    std::fill(jacobian, jacobian + 6, 0.0);
    for (int j = 0; j < 6; ++j) {
      const double h = 1e-6 * std::max(1.0, std::abs(coords[j]));

      coords[j] += h;
      const double plus = hingeExcess(coords);
      coords[j] -= 2.0 * h;
      const double minus = hingeExcess(coords);
      coords[j] += h;  // 复原

      jacobian[j] = weight_ * (plus - minus) / (2.0 * h);
    }
  }

 private:
  /// 返回超出曲率上限的量；未超限时为 0。
  ///
  /// 采用平滑铰链 0.5·(e + √(e²+ε²)) − ε/2 而非硬 max(0, e)：
  /// 硬铰链在 e=0 处梯度不连续，求解器会在阈值附近来回震荡；
  /// 平滑版本处处可导，且 e≫ε 时与硬铰链的差可忽略。
  double hingeExcess(const double* coords) const {
    const Vec2 a{coords[0], coords[1]};
    const Vec2 b{coords[2], coords[3]};
    const Vec2 c{coords[4], coords[5]};

    const double curvature = std::abs(PathOptimizer::mengerCurvature(a, b, c));
    const double excess = curvature - max_curvature_;
    return 0.5 * (excess + std::sqrt(excess * excess + softening_ * softening_)) -
           0.5 * softening_;
  }

  double weight_;
  double max_curvature_;
  double softening_;
};

}  // namespace

// ---------------------------------------------------------------------------
// 离散几何量
// ---------------------------------------------------------------------------

double PathOptimizer::mengerCurvature(const Vec2& a, const Vec2& b, const Vec2& c) {
  // Menger 曲率：κ = 4·Area / (|ab|·|bc|·|ca|)，用叉积表示面积
  const Vec2 ab = b - a;
  const Vec2 bc = c - b;
  const Vec2 ca = a - c;

  const double cross = ab.cross(bc);  // 2 × 三角形面积
  const double la = ab.norm();
  const double lb = bc.norm();
  const double lc = ca.norm();

  const double denominator = la * lb * lc;
  if (denominator < 1e-12) {
    return 0.0;  // 存在重合点，曲率无从定义
  }

  return 2.0 * cross / denominator;
}

std::vector<double> PathOptimizer::computeCurvatures(const std::vector<Vec2>& path) {
  std::vector<double> curvatures(path.size(), 0.0);
  if (path.size() < 3) return curvatures;

  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    curvatures[i] = mengerCurvature(path[i - 1], path[i], path[i + 1]);
  }

  // 端点没有完整邻域，用相邻内点的值复制，避免出现虚假的零曲率
  curvatures.front() = curvatures[1];
  curvatures.back() = curvatures[path.size() - 2];
  return curvatures;
}

double PathOptimizer::maxAbsCurvature(const std::vector<Vec2>& path) {
  const std::vector<double> curvatures = computeCurvatures(path);
  double maximum = 0.0;
  for (double k : curvatures) {
    maximum = std::max(maximum, std::abs(k));
  }
  return maximum;
}

double PathOptimizer::maxAbsCurvature(const std::vector<Vec2>& path, std::size_t margin) {
  const std::vector<double> curvatures = computeCurvatures(path);

  // 路径太短时全部点都会被排除，此时退回整体指标
  if (curvatures.size() <= 2 * margin + 1) {
    double maximum = 0.0;
    for (double k : curvatures) maximum = std::max(maximum, std::abs(k));
    return maximum;
  }

  double maximum = 0.0;
  for (std::size_t i = margin; i + margin < curvatures.size(); ++i) {
    maximum = std::max(maximum, std::abs(curvatures[i]));
  }
  return maximum;
}

double PathOptimizer::maxCurvatureRate(const std::vector<Vec2>& path) {
  if (path.size() < 4) return 0.0;

  const std::vector<double> curvatures = computeCurvatures(path);

  double maximum = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = (path[i] - path[i - 1]).norm();
    if (ds < 1e-6) continue;  // 重合点无法定义弧长变化率
    const double rate = std::abs(curvatures[i] - curvatures[i - 1]) / ds;
    maximum = std::max(maximum, rate);
  }
  return maximum;
}

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

PathOptimizer::PathOptimizer() = default;
PathOptimizer::PathOptimizer(const PathOptimizerOptions& options) : options_(options) {}

// ---------------------------------------------------------------------------
// 优化
// ---------------------------------------------------------------------------

std::vector<Vec2> PathOptimizer::optimize(const std::vector<Vec2>& initial_path,
                                          PathOptimizationReport* report) const {
  const auto wall_start = std::chrono::steady_clock::now();

  PathOptimizationReport local_report;
  local_report.backend = usingCeres() ? "ceres" : "builtin-lm";
  local_report.initial_max_curvature = maxAbsCurvature(initial_path);
  local_report.initial_max_curvature_rate = maxCurvatureRate(initial_path);

  if (initial_path.size() < 3) {
    local_report.message = "路径点少于 3 个，无需优化";
    local_report.converged = true;
    local_report.final_max_curvature = local_report.initial_max_curvature;
    local_report.final_max_curvature_rate = local_report.initial_max_curvature_rate;
    if (report != nullptr) *report = local_report;
    return initial_path;
  }

  const std::size_t count = initial_path.size();

  // 决策变量：展平的坐标数组 [x0,y0, x1,y1, ...]
  std::vector<double> coords(count * 2);
  for (std::size_t i = 0; i < count; ++i) {
    coords[2 * i] = initial_path[i].x;
    coords[2 * i + 1] = initial_path[i].y;
  }

#if !defined(ADSIM_HAS_CERES)
  // 给定曲率权重，装配并求解一次（仅内置后端使用）
  auto buildAndSolve = [&](double curvature_weight) -> SolverSummary {
    Problem problem;

    // ---- 平滑项 ----
    for (std::size_t i = 1; i + 1 < count; ++i) {
      problem.addResidualBlock(
          std::make_shared<SmoothnessResidual>(options_.smoothness_weight),
          {&coords[2 * (i - 1)], &coords[2 * i], &coords[2 * (i + 1)]});
    }

    // ---- 贴合项 ----
    for (std::size_t i = 0; i < count; ++i) {
      problem.addResidualBlock(
          std::make_shared<DeviationResidual>(options_.deviation_weight, initial_path[i].x,
                                              initial_path[i].y),
          {&coords[2 * i]});
    }

    // ---- 长度项 ----
    for (std::size_t i = 0; i + 1 < count; ++i) {
      problem.addResidualBlock(
          std::make_shared<LengthResidual>(options_.length_weight,
                                           options_.reference_spacing),
          {&coords[2 * i], &coords[2 * (i + 1)]});
    }

    // ---- 曲率约束项（平滑铰链惩罚）----
    for (std::size_t i = 1; i + 1 < count; ++i) {
      problem.addResidualBlock(
          std::make_shared<CurvatureResidual>(curvature_weight, options_.max_curvature,
                                              kHingeSoftening),
          {&coords[2 * (i - 1)], &coords[2 * i], &coords[2 * (i + 1)]});
    }

    // ---- 固定首末点 ----
    // 用上下界把首末点钉死，而不是把它们排除在问题外：
    // 这样残差结构保持规整，代价函数也无需为零点特判。
    if (options_.fix_endpoints) {
      for (std::size_t index : {std::size_t(0), count - 1}) {
        const std::vector<double> fixed = {initial_path[index].x, initial_path[index].y};
        problem.setParameterBounds(&coords[2 * index], fixed, fixed);
      }
    }

    SolverOptions solver_options;
    solver_options.algorithm = SolverOptions::Algorithm::kLevenbergMarquardt;
    solver_options.max_iterations = options_.max_iterations;
    solver_options.verbose = options_.verbose;

    return NonlinearSolver::solve(problem, solver_options);
  };
#endif  // !ADSIM_HAS_CERES

  auto extract = [&]() {
    std::vector<Vec2> path(count);
    for (std::size_t i = 0; i < count; ++i) {
      path[i] = Vec2{coords[2 * i], coords[2 * i + 1]};
    }
    return path;
  };

  // ---- 求解 ----
  // 两个后端求解的是同一个数学问题，只是求解器实现不同。
  // 代价函数定义（平滑/贴合/长度/曲率铰链）完全一致，因此切换后端
  // 不会改变问题的语义，只影响收敛速度与数值精度。
  local_report.curvature_limit = options_.max_curvature;

  std::vector<Vec2> optimized = initial_path;
  double achieved = local_report.initial_max_curvature;

#if defined(ADSIM_HAS_CERES)
  // ---- Ceres 后端 ----
  // 自动微分 + 成熟的 LM 实现；外层约束迭代在 ceresOptimizePath 内部完成
  if (ceresOptimizePath(coords, initial_path, options_, &local_report)) {
    optimized = extract();
    achieved = maxAbsCurvature(optimized);
  } else {
    local_report.message = "Ceres 求解失败，结果保持原始路径";
    local_report.converged = false;
    if (report != nullptr) *report = local_report;
    return initial_path;
  }
#else
  // ---- 内置 LM 后端 ----
  // 外层约束迭代：惩罚法单轮未必能把曲率压进限值，逐轮加大曲率权重；
  // 每轮以上一轮的解为初值（热启动），比每轮从头开始收敛更快、偏移更小。
  double curvature_weight = options_.curvature_weight;
  const double allowed =
      options_.max_curvature * (1.0 + std::max(options_.constraint_tolerance, 0.0));

  SolverSummary summary;
  const int max_rounds = std::max(options_.max_constraint_rounds, 1);

  for (int round = 0; round < max_rounds; ++round) {
    summary = buildAndSolve(curvature_weight);
    optimized = extract();
    achieved = maxAbsCurvature(optimized);
    local_report.constraint_rounds = round + 1;

    if (achieved <= allowed) break;
    if (round + 1 >= max_rounds) break;  // 已是最后一轮，无需再加大权重

    curvature_weight *= std::max(options_.constraint_weight_growth, 1.0);
  }

  local_report.converged = summary.converged;
  local_report.iterations = summary.iterations;
  local_report.message = summary.message;
  local_report.initial_cost = summary.initial_cost;
  local_report.final_cost = summary.final_cost;
#endif  // ADSIM_HAS_CERES

  double max_deviation = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    max_deviation = std::max(max_deviation, (optimized[i] - initial_path[i]).norm());
  }

  local_report.final_max_curvature = achieved;
  local_report.final_interior_max_curvature = maxAbsCurvature(optimized, 2);
  local_report.final_max_curvature_rate = maxCurvatureRate(optimized);
  local_report.max_deviation = max_deviation;
  local_report.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
          .count();

  if (report != nullptr) *report = local_report;
  return optimized;
}

bool PathOptimizationReport::satisfiesCurvatureLimit() const {
  if (curvature_limit <= 0.0) return true;
  return final_max_curvature <= curvature_limit * 1.02;
}

bool PathOptimizer::usingCeres() {
#if defined(ADSIM_HAS_CERES)
  return true;
#else
  return false;
#endif
}

std::string PathOptimizationReport::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);

  oss << "路径优化 (" << backend << ") " << (converged ? "收敛" : "未收敛") << ":\n";
  oss << "  迭代次数      : " << iterations << "\n";
  oss.precision(3);
  oss << "  最大曲率      : " << initial_max_curvature << " → " << final_max_curvature
      << " (1/m)\n";
  oss << "  ├ 内部点数  : " << final_interior_max_curvature
      << " (1/m, 排除被固定的首末点邻域)\n";
  oss << "  最大曲率变化率: " << initial_max_curvature_rate << " → "
      << final_max_curvature_rate << " (1/m²)\n";
  oss << "  最大偏移      : " << max_deviation << " m\n";
  oss.precision(2);
  oss << "  代价          : " << initial_cost << " → " << final_cost << " (下降 "
      << (initial_cost > 0.0 ? (1.0 - final_cost / initial_cost) * 100.0 : 0.0) << "%)\n";
  oss << "  耗时          : " << elapsed_ms << " ms\n";
  if (!message.empty()) {
    oss << "  说明          : " << message << "\n";
  }
  return oss.str();
}

}  // namespace adsim
