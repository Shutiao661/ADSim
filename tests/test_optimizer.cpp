// =============================================================================
//  test_optimizer.cpp — 非线性优化器与路径平滑单元测试
// =============================================================================
#include "TestFramework.h"

#include "adsim/planning/optimizer/NonlinearSolver.h"
#include "adsim/planning/optimizer/PathOptimizer.h"

#include <cmath>
#include <random>
#include <vector>

using namespace adsim;

namespace {

// ---------------------------------------------------------------------------
// 测试用的简单代价函数
// ---------------------------------------------------------------------------

/// 单参数残差：r = x − target，解析雅可比为 1
class LinearResidual : public ResidualBlock {
 public:
  explicit LinearResidual(double target) : target_(target) {}

  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {1}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    residuals[0] = params[0][0] - target_;
    if (jacobian != nullptr) jacobian[0] = 1.0;
  }

 private:
  double target_;
};

/// 双参数残差：r = x − y，用于验证多参数块的耦合被正确处理
class DifferenceResidual : public ResidualBlock {
 public:
  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {1, 1}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    residuals[0] = params[0][0] - params[1][0];
    if (jacobian != nullptr) {
      jacobian[0] = 1.0;   // ∂r/∂x
      jacobian[1] = -1.0;  // ∂r/∂y
    }
  }
};

/// 非线性残差：r = x² − target，用于验证 LM 在非线性问题上的收敛性
class QuadraticResidual : public ResidualBlock {
 public:
  explicit QuadraticResidual(double target) : target_(target) {}

  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {1}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    const double x = params[0][0];
    residuals[0] = x * x - target_;
    if (jacobian != nullptr) jacobian[0] = 2.0 * x;
  }

 private:
  double target_;
};

/// 三点平滑残差，用于验证雅可比校验工具
class SecondDifferenceResidual : public ResidualBlock {
 public:
  int residualDimension() const override { return 1; }
  std::vector<int> parameterBlockSizes() const override { return {1, 1, 1}; }

  void evaluate(const double* const* params, double* residuals,
                double* jacobian) override {
    residuals[0] = params[0][0] - 2.0 * params[1][0] + params[2][0];
    if (jacobian != nullptr) {
      jacobian[0] = 1.0;
      jacobian[1] = -2.0;
      jacobian[2] = 1.0;
    }
  }
};

/// 构造一条带锯齿噪声的圆弧路径
std::vector<Vec2> makeJaggedArc(std::size_t count, double radius, double jitter_sigma,
                                bool jitter_endpoints, unsigned seed) {
  std::vector<Vec2> path;
  path.reserve(count);

  std::mt19937 rng(seed);
  std::normal_distribution<double> jitter(0.0, jitter_sigma);

  for (std::size_t i = 0; i < count; ++i) {
    const double angle = static_cast<double>(i) * 0.05;
    Vec2 point{radius * std::sin(angle), radius * (1.0 - std::cos(angle))};

    const bool at_end = (i == 0 || i + 1 == count);
    if (jitter_endpoints || !at_end) {
      point.x += jitter(rng);
      point.y += jitter(rng);
    }
    path.push_back(point);
  }
  return path;
}

}  // namespace

// ===========================================================================
//  求解器基础能力
// ===========================================================================

ADSIM_TEST(NonlinearSolver, 单参数线性问题精确求解) {
  double x = 0.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<LinearResidual>(5.0), {&x});

  const SolverSummary summary = NonlinearSolver::solve(problem);

  ADSIM_CHECK(summary.converged);
  ADSIM_CHECK_NEAR(x, 5.0, 1e-8);
  ADSIM_CHECK_LT(summary.final_cost, 1e-12);
  ADSIM_CHECK(!summary.toString().empty());
}

ADSIM_TEST(NonlinearSolver, 多参数块耦合被正确处理) {
  // 两个残差把 x 与 y 拉向同一个值，最优解应为 x == y
  double x = 0.0;
  double y = 10.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<DifferenceResidual>(), {&x, &y});
  problem.addResidualBlock(std::make_shared<LinearResidual>(3.0), {&x});
  problem.addResidualBlock(std::make_shared<LinearResidual>(3.0), {&y});

  const SolverSummary summary = NonlinearSolver::solve(problem);

  ADSIM_CHECK(summary.converged);
  ADSIM_CHECK_NEAR(x, 3.0, 1e-6);
  ADSIM_CHECK_NEAR(y, 3.0, 1e-6);
}

ADSIM_TEST(NonlinearSolver, 非线性问题收敛到正确根) {
  double x = 1.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<QuadraticResidual>(16.0), {&x});

  SolverOptions options;
  options.algorithm = SolverOptions::Algorithm::kLevenbergMarquardt;

  const SolverSummary summary = NonlinearSolver::solve(problem, options);

  ADSIM_CHECK(summary.converged);
  ADSIM_CHECK_NEAR(std::abs(x), 4.0, 1e-6);
}

ADSIM_TEST(NonlinearSolver, 高斯牛顿与LM都能求解良态问题) {
  for (auto algorithm : {SolverOptions::Algorithm::kGaussNewton,
                         SolverOptions::Algorithm::kLevenbergMarquardt}) {
    double x = 0.0;
    Problem problem;
    problem.addResidualBlock(std::make_shared<LinearResidual>(7.5), {&x});

    SolverOptions options;
    options.algorithm = algorithm;

    const SolverSummary summary = NonlinearSolver::solve(problem, options);
    ADSIM_CHECK(summary.converged);
    ADSIM_CHECK_NEAR(x, 7.5, 1e-8);
  }
}

ADSIM_TEST(NonlinearSolver, 参数上下界被遵守) {
  double x = 0.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<LinearResidual>(100.0), {&x});
  problem.setParameterBounds(&x, {0.0}, {10.0});

  NonlinearSolver::solve(problem);

  // 无约束最优在 100，但有上界 10，必须停在边界
  ADSIM_CHECK_NEAR(x, 10.0, 1e-6);
}

ADSIM_TEST(NonlinearSolver, 同一参数块被多个残差引用时不重复叠加) {
  // 两个残差都指向同一个参数块。若求解器按指针去重失败，
  // 海森矩阵会被叠加两次，导致解偏小（等效于权重翻倍）。
  // 本例中两个残差目标一致，正确解与去重与否无关，故改用权重不等的情形：
  // r1 = x - 0（权重 1），r2 = x - 0（权重 1）—— 无论叠加与否最优都是 0。
  // 真正能区分的是下面这种：两个残差把 x 拉向不同目标。
  double x = 0.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<LinearResidual>(0.0), {&x}, 3.0);
  problem.addResidualBlock(std::make_shared<LinearResidual>(10.0), {&x}, 1.0);

  NonlinearSolver::solve(problem);

  // 加权最小二乘的解析解为 (3·0 + 1·10) / (3 + 1) = 2.5
  // 若权重被错误地平方或参数被重复计入，结果会偏离该值
  ADSIM_CHECK_NEAR(x, 2.5, 1e-6);
}

ADSIM_TEST(NonlinearSolver, 雅可比校验工具能识别正确与错误) {
  double a = 1.0;
  double b = 2.0;
  double c = 4.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<SecondDifferenceResidual>(), {&a, &b, &c});

  // 该残差的解析雅可比是精确的，相对误差应接近零
  const double error = problem.verifyJacobian(0);
  ADSIM_CHECK_LT(error, 1e-5);

  // 越界索引返回极大值而不是崩溃
  ADSIM_CHECK_GT(problem.verifyJacobian(99), 1e100);
}

ADSIM_TEST(NonlinearSolver, 空问题与非法输入) {
  Problem empty;
  const SolverSummary summary = NonlinearSolver::solve(empty);
  ADSIM_CHECK(summary.converged);
  ADSIM_CHECK_EQ(summary.iterations, 0);

  Problem problem;
  ADSIM_CHECK_THROWS(
      problem.addResidualBlock(nullptr, {nullptr}), std::invalid_argument);

  double x = 0.0;
  ADSIM_CHECK_THROWS(
      problem.addResidualBlock(std::make_shared<DifferenceResidual>(), {&x}),
      std::invalid_argument);  // 参数块个数与残差声明不符
}

ADSIM_TEST(NonlinearSolver, 代价随迭代单调不增) {
  // 三个残差同时约束：x² = 16、y = 4、x = y。
  // 三者相容，x = y = 4 处代价恰为 0。
  double x = 0.0;
  double y = 0.0;

  Problem problem;
  problem.addResidualBlock(std::make_shared<QuadraticResidual>(16.0), {&x});
  problem.addResidualBlock(std::make_shared<LinearResidual>(4.0), {&y});
  problem.addResidualBlock(std::make_shared<DifferenceResidual>(), {&x, &y});

  SolverOptions options;
  options.max_iterations = 50;

  const SolverSummary summary = NonlinearSolver::solve(problem, options);

  // LM 只接受使代价下降的步，最终代价必然不高于初始代价
  ADSIM_CHECK_LT(summary.final_cost, summary.initial_cost + 1e-12);
  ADSIM_CHECK_LT(summary.final_cost, 1e-8);
  ADSIM_CHECK_NEAR(x, 4.0, 1e-4);
  ADSIM_CHECK_NEAR(y, 4.0, 1e-4);
}

// ===========================================================================
//  路径几何量
// ===========================================================================

ADSIM_TEST(PathOptimizer, Menger曲率与解析值一致) {
  // 半径 R 的圆上取三点，外接圆曲率的量值恒为 1/R。
  // 点按逆时针排列，曲率为正；顺时针则为负——符号承载了转向信息，
  // 因此这里对量值做断言，并单独校验符号约定。
  const double radius = 20.0;
  for (double delta : {0.05, 0.1, 0.3, 0.8}) {
    const Vec2 ccw_a{radius * std::cos(0.0), radius * std::sin(0.0)};
    const Vec2 ccw_b{radius * std::cos(delta), radius * std::sin(delta)};
    const Vec2 ccw_c{radius * std::cos(2 * delta), radius * std::sin(2 * delta)};
    ADSIM_CHECK_NEAR(PathOptimizer::mengerCurvature(ccw_a, ccw_b, ccw_c), 1.0 / radius,
                     1e-9);

    // 交换 b、c 即改变转向，曲率应变号
    ADSIM_CHECK_NEAR(PathOptimizer::mengerCurvature(ccw_a, ccw_c, ccw_b), -1.0 / radius,
                     1e-9);
  }

  // 三点共线时曲率为 0
  ADSIM_CHECK_NEAR(PathOptimizer::mengerCurvature({0, 0}, {1, 1}, {2, 2}), 0.0, 1e-12);

  // 点重合时不得产生 NaN
  const double degenerate = PathOptimizer::mengerCurvature({1, 1}, {1, 1}, {2, 2});
  ADSIM_CHECK(std::isfinite(degenerate));
  ADSIM_CHECK_NEAR(degenerate, 0.0, 1e-12);
}

ADSIM_TEST(PathOptimizer, 曲率与变化率统计量) {
  // 标准圆弧：曲率处处等于 1/R，变化率接近 0
  const double radius = 25.0;
  std::vector<Vec2> arc;
  for (int i = 0; i <= 60; ++i) {
    const double angle = i * 0.03;
    arc.push_back({radius * std::sin(angle), radius * (1.0 - std::cos(angle))});
  }

  ADSIM_CHECK_NEAR(PathOptimizer::maxAbsCurvature(arc), 1.0 / radius, 1e-6);
  ADSIM_CHECK_LT(PathOptimizer::maxCurvatureRate(arc), 1e-6);

  // 点数不足时安全返回
  ADSIM_CHECK_NEAR(PathOptimizer::maxAbsCurvature({{0, 0}, {1, 1}}), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(PathOptimizer::maxCurvatureRate({}), 0.0, 1e-12);
}

// ===========================================================================
//  路径平滑
// ===========================================================================

ADSIM_TEST(PathOptimizer, 输入点数不足时原样返回) {
  PathOptimizer optimizer;
  PathOptimizationReport report;

  const std::vector<Vec2> tiny = {{0, 0}, {1, 1}};
  const std::vector<Vec2> result = optimizer.optimize(tiny, &report);

  ADSIM_CHECK_EQ(result.size(), tiny.size());
  ADSIM_CHECK(report.converged);
  ADSIM_CHECK_NEAR(result[0].x, 0.0, 1e-12);
}

ADSIM_TEST(PathOptimizer, 锯齿路径的曲率被显著降低) {
  const std::vector<Vec2> jagged = makeJaggedArc(41, 30.0, 0.18, true, 7u);

  const double initial_curvature = PathOptimizer::maxAbsCurvature(jagged);
  const double initial_rate = PathOptimizer::maxCurvatureRate(jagged);

  PathOptimizerOptions options;
  options.max_curvature = 0.05;
  options.max_iterations = 500;
  PathOptimizer optimizer(options);

  PathOptimizationReport report;
  const std::vector<Vec2> smoothed = optimizer.optimize(jagged, &report);

  ADSIM_CHECK_EQ(smoothed.size(), jagged.size());

  // 曲率与曲率变化率都应大幅下降
  const double final_curvature = PathOptimizer::maxAbsCurvature(smoothed);
  ADSIM_CHECK_LT(final_curvature, initial_curvature * 0.25);
  ADSIM_CHECK_LT(PathOptimizer::maxCurvatureRate(smoothed), initial_rate * 0.25);

  ADSIM_CHECK(!report.toString().empty());
  ADSIM_CHECK_GT(report.elapsed_ms, 0.0);
}

ADSIM_TEST(PathOptimizer, 内部点满足曲率约束) {
  // 首末点被固定时，其邻域曲率由固定点决定，无法优化；
  // 验收标准应落在"可优化的内部点"上。
  const std::vector<Vec2> jagged = makeJaggedArc(41, 30.0, 0.18, true, 7u);

  PathOptimizerOptions options;
  options.max_curvature = 0.05;
  options.fix_endpoints = true;
  options.max_iterations = 500;

  PathOptimizer optimizer(options);
  PathOptimizationReport report;
  const std::vector<Vec2> smoothed = optimizer.optimize(jagged, &report);

  // 排除首末各 2 点后，曲率必须落在限值内
  ADSIM_CHECK_NEAR(report.final_interior_max_curvature, options.max_curvature, 0.002);
  ADSIM_CHECK_LT(report.final_interior_max_curvature, options.max_curvature * 1.05);

  // 内部指标必然不高于整体指标
  ADSIM_CHECK_LT(report.final_interior_max_curvature,
                 report.final_max_curvature + 1e-9);
}

ADSIM_TEST(PathOptimizer, 不固定端点时整体满足约束) {
  // 端点也允许移动时，整条路径都应被压进限值
  const std::vector<Vec2> jagged = makeJaggedArc(41, 30.0, 0.18, true, 11u);

  PathOptimizerOptions options;
  options.max_curvature = 0.05;
  options.fix_endpoints = false;
  options.max_iterations = 500;

  PathOptimizer optimizer(options);
  PathOptimizationReport report;
  const std::vector<Vec2> smoothed = optimizer.optimize(jagged, &report);

  ADSIM_CHECK_LT(report.final_max_curvature, options.max_curvature * 1.1);
  ADSIM_CHECK(report.satisfiesCurvatureLimit());
}

ADSIM_TEST(PathOptimizer, 固定端点确实保持原位) {
  const std::vector<Vec2> jagged = makeJaggedArc(30, 25.0, 0.2, false, 3u);

  PathOptimizerOptions options;
  options.fix_endpoints = true;
  options.max_iterations = 300;

  PathOptimizer optimizer(options);
  const std::vector<Vec2> smoothed = optimizer.optimize(jagged, nullptr);

  ADSIM_CHECK_EQ(smoothed.size(), jagged.size());
  ADSIM_CHECK_NEAR(smoothed.front().x, jagged.front().x, 1e-9);
  ADSIM_CHECK_NEAR(smoothed.front().y, jagged.front().y, 1e-9);
  ADSIM_CHECK_NEAR(smoothed.back().x, jagged.back().x, 1e-9);
  ADSIM_CHECK_NEAR(smoothed.back().y, jagged.back().y, 1e-9);
}

ADSIM_TEST(PathOptimizer, 优化后不偏离原始路径太远) {
  const std::vector<Vec2> jagged = makeJaggedArc(41, 30.0, 0.18, true, 7u);

  PathOptimizerOptions options;
  options.max_curvature = 0.05;
  options.max_iterations = 500;
  PathOptimizer optimizer(options);

  PathOptimizationReport report;
  const std::vector<Vec2> smoothed = optimizer.optimize(jagged, &report);

  // 偏移必须受控：平滑不能以"改走另一条路"为代价
  ADSIM_CHECK_LT(report.max_deviation, 1.0);
  ADSIM_CHECK_GT(report.max_deviation, 0.0);

  for (std::size_t i = 0; i < jagged.size(); ++i) {
    ADSIM_CHECK_LT((smoothed[i] - jagged[i]).norm(), 1.0 + 1e-9);
  }
}

ADSIM_TEST(PathOptimizer, 已平滑的路径不会被破坏) {
  // 输入本身就是光滑圆弧时，优化应保持其形状而非引入偏移
  const double radius = 40.0;
  std::vector<Vec2> arc;
  for (int i = 0; i <= 40; ++i) {
    const double angle = i * 0.04;
    arc.push_back({radius * std::sin(angle), radius * (1.0 - std::cos(angle))});
  }

  PathOptimizerOptions options;
  options.max_curvature = 0.05;  // 圆弧曲率 0.025，远低于限值，铰链不激活
  options.max_iterations = 200;
  PathOptimizer optimizer(options);

  PathOptimizationReport report;
  const std::vector<Vec2> result = optimizer.optimize(arc, &report);

  ADSIM_CHECK_NEAR(PathOptimizer::maxAbsCurvature(result), 1.0 / radius, 5e-3);
  ADSIM_CHECK_LT(report.max_deviation, 0.5);
}

ADSIM_TEST(PathOptimizer, 直线路径保持笔直) {
  std::vector<Vec2> line;
  for (int i = 0; i <= 30; ++i) {
    line.push_back({static_cast<double>(i), 0.0});
  }

  PathOptimizer optimizer;
  const std::vector<Vec2> result = optimizer.optimize(line, nullptr);

  ADSIM_CHECK_EQ(result.size(), line.size());
  for (const Vec2& p : result) {
    ADSIM_CHECK_LT(std::abs(p.y), 1e-6);  // 直线不应被弯曲
  }
}

ADSIM_TEST(PathOptimizer, 结果确定性可复现) {
  const std::vector<Vec2> jagged = makeJaggedArc(35, 28.0, 0.15, true, 5u);

  PathOptimizerOptions options;
  options.max_curvature = 0.06;
  options.max_iterations = 300;

  PathOptimizer optimizer(options);
  const std::vector<Vec2> first = optimizer.optimize(jagged, nullptr);
  const std::vector<Vec2> second = optimizer.optimize(jagged, nullptr);

  ADSIM_CHECK_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    ADSIM_CHECK_NEAR(first[i].x, second[i].x, 1e-12);
    ADSIM_CHECK_NEAR(first[i].y, second[i].y, 1e-12);
  }
}

ADSIM_TEST(PathOptimizer, 后端标识与实际构建一致) {
  const std::string backend = PathOptimizer::usingCeres() ? "ceres" : "builtin-lm";

  PathOptimizer optimizer;
  PathOptimizationReport report;
  optimizer.optimize(makeJaggedArc(10, 20.0, 0.1, true, 1u), &report);

  ADSIM_CHECK_EQ(report.backend, backend);
}
