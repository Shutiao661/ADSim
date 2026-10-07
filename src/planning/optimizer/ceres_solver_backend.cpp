// =============================================================================
//  ceres_solver_backend.cpp — Ceres Solver 后端实现
//
//  **可选依赖**：仅在 ADSIM_HAS_CERES 定义时参与编译
//  （构建时加 -DADSIM_WITH_CERES=ON）。
//
//  求解的数学问题与内置后端完全一致，区别在于：
//    * 用 Ceres 的自动微分，残差里可以直接写出 Menger 曲率这类有理函数，
//      不必像内置后端那样退而求其次用块内中心差分
//    * 用成熟的 LM 实现（信赖域 + 非单调下降）
//    * 可选稀疏线性代数，点上规模后优势明显
// =============================================================================
#include "adsim/planning/optimizer/CeresPathOptimizer.h"

// 所有 #include 都必须在文件作用域完成，绝不能写进 namespace 内部。
// 把 <ceres/ceres.h> 放在 `namespace adsim {` 之后会把该头文件间接引入的
// 全部标准库符号也拽进 adsim 命名空间，编译器随后会在 adsim::std 下找不到
// unique_ptr / tuple 等定义，报出一连串看似莫名其妙的错。这类错误只有在
// 真正安装了 Ceres 的机器上才会出现——按 API 写而不编译是看不出来的。
#if defined(ADSIM_HAS_CERES)
#include <ceres/ceres.h>
#endif

#include <algorithm>
#include <cmath>

namespace adsim {

bool ceresBackendAvailable() {
#if defined(ADSIM_HAS_CERES)
  return true;
#else
  return false;
#endif
}

#if defined(ADSIM_HAS_CERES)

namespace {

/// 曲率铰链平滑带宽（与内置后端保持一致，便于对比两者结果）
constexpr double kHingeSoftening = 1e-3;

/// 平滑残差：二阶差分，是曲率的离散近似
struct SmoothnessCost {
  explicit SmoothnessCost(double weight)
      : sqrt_weight(std::sqrt(std::max(weight, 0.0))) {}

  template <typename T>
  bool operator()(const T* const prev, const T* const curr, const T* const next,
                  T* residual) const {
    residual[0] = T(sqrt_weight) * (prev[0] - T(2.0) * curr[0] + next[0]);
    residual[1] = T(sqrt_weight) * (prev[1] - T(2.0) * curr[1] + next[1]);
    return true;
  }

  static ceres::CostFunction* Create(double weight) {
    return new ceres::AutoDiffCostFunction<SmoothnessCost, 2, 2, 2, 2>(
        new SmoothnessCost(weight));
  }

  double sqrt_weight;
};

/// 贴合残差：把路径点拉回原始位置
struct DeviationCost {
  DeviationCost(double weight, double x, double y)
      : sqrt_weight(std::sqrt(std::max(weight, 0.0))), ref_x(x), ref_y(y) {}

  template <typename T>
  bool operator()(const T* const point, T* residual) const {
    residual[0] = T(sqrt_weight) * (point[0] - T(ref_x));
    residual[1] = T(sqrt_weight) * (point[1] - T(ref_y));
    return true;
  }

  static ceres::CostFunction* Create(double weight, double x, double y) {
    return new ceres::AutoDiffCostFunction<DeviationCost, 2, 2>(
        new DeviationCost(weight, x, y));
  }

  double sqrt_weight;
  double ref_x;
  double ref_y;
};

/// 长度残差：惩罚相邻点间距偏离参考间距，防止路径点向两端聚集
struct LengthCost {
  LengthCost(double weight, double reference_spacing)
      : sqrt_weight(std::sqrt(std::max(weight, 0.0))),
        reference(reference_spacing) {}

  template <typename T>
  bool operator()(const T* const a, const T* const b, T* residual) const {
    const T dx = b[0] - a[0];
    const T dy = b[1] - a[1];
    residual[0] = T(sqrt_weight) * (ceres::sqrt(dx * dx + dy * dy) - T(reference));
    return true;
  }

  static ceres::CostFunction* Create(double weight, double reference_spacing) {
    return new ceres::AutoDiffCostFunction<LengthCost, 1, 2, 2>(
        new LengthCost(weight, reference_spacing));
  }

  double sqrt_weight;
  double reference;
};

/// 曲率铰链惩罚残差。
///
/// 曲率用 Menger 公式 κ = 2·cross(ab, bc) / (|ab|·|bc|·|ca|) 计算，
/// 是坐标的有理函数。内置后端因为要手写雅可比而改用中心差分，
/// 这里交给自动微分即可直接写出原式。
struct CurvatureCost {
  CurvatureCost(double weight, double max_curvature, double softening)
      : sqrt_weight(std::sqrt(std::max(weight, 0.0))),
        max_kappa(max_curvature),
        epsilon(softening) {}

  template <typename T>
  bool operator()(const T* const a, const T* const b, const T* const c,
                  T* residual) const {
    const T abx = b[0] - a[0];
    const T aby = b[1] - a[1];
    const T bcx = c[0] - b[0];
    const T bcy = c[1] - b[1];
    const T cax = a[0] - c[0];
    const T cay = a[1] - c[1];

    const T cross = abx * bcy - aby * bcx;
    const T lab = ceres::sqrt(abx * abx + aby * aby);
    const T lbc = ceres::sqrt(bcx * bcx + bcy * bcy);
    const T lca = ceres::sqrt(cax * cax + cay * cay);

    const T denominator = lab * lbc * lca;

    // 三点重合时曲率无从定义，直接给零残差；
    // 用常数阈值避免除零，同时让自动微分不会产生 NaN
    if (denominator < T(1e-12)) {
      residual[0] = T(0.0);
      return true;
    }

    const T kappa = ceres::abs(T(2.0) * cross / denominator);
    const T excess = kappa - T(max_kappa);

    // 平滑铰链：0.5·(e + √(e²+ε²)) − ε/2，处处可导
    const T hinge =
        T(0.5) * (excess + ceres::sqrt(excess * excess + T(epsilon * epsilon))) -
        T(0.5 * epsilon);

    residual[0] = T(sqrt_weight) * hinge;
    return true;
  }

  static ceres::CostFunction* Create(double weight, double max_curvature,
                                     double softening) {
    return new ceres::AutoDiffCostFunction<CurvatureCost, 1, 2, 2, 2>(
        new CurvatureCost(weight, max_curvature, softening));
  }

  double sqrt_weight;
  double max_kappa;
  double epsilon;
};

/// 装配一次问题并求解
bool solveOnce(std::vector<double>& coords, const std::vector<Vec2>& initial_path,
               const PathOptimizerOptions& options, double curvature_weight,
               ceres::Solver::Summary* summary_out) {
  const std::size_t count = initial_path.size();

  ceres::Problem problem;

  // ---- 平滑项 ----
  for (std::size_t i = 1; i + 1 < count; ++i) {
    problem.AddResidualBlock(
        SmoothnessCost::Create(options.smoothness_weight), nullptr,
        &coords[2 * (i - 1)], &coords[2 * i], &coords[2 * (i + 1)]);
  }

  // ---- 贴合项 ----
  for (std::size_t i = 0; i < count; ++i) {
    problem.AddResidualBlock(DeviationCost::Create(options.deviation_weight,
                                                   initial_path[i].x, initial_path[i].y),
                             nullptr, &coords[2 * i]);
  }

  // ---- 长度项 ----
  for (std::size_t i = 0; i + 1 < count; ++i) {
    problem.AddResidualBlock(
        LengthCost::Create(options.length_weight, options.reference_spacing), nullptr,
        &coords[2 * i], &coords[2 * (i + 1)]);
  }

  // ---- 曲率约束项 ----
  for (std::size_t i = 1; i + 1 < count; ++i) {
    problem.AddResidualBlock(
        CurvatureCost::Create(curvature_weight, options.max_curvature, kHingeSoftening),
        nullptr, &coords[2 * (i - 1)], &coords[2 * i], &coords[2 * (i + 1)]);
  }

  // ---- 固定首末点 ----
  if (options.fix_endpoints) {
    for (std::size_t index : {std::size_t(0), count - 1}) {
      problem.SetParameterBlockConstant(&coords[2 * index]);
    }
  }

  ceres::Solver::Options solver_options;
  // 路径点数不多时稠密分解更快；点上规模后可换成 SPARSE_NORMAL_CHOLESKY
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = options.max_iterations;
  solver_options.num_threads = 1;

  // Ceres 默认的 logging_type 是 PER_MINIMIZER_ITERATION，会把每次迭代的
  // 信赖域收敛表打到日志里。以 MINIGLOG 构建时这些内容直接进 stderr，
  // 会把命令行工具的正常输出整个冲掉（verbose 为 false 也挡不住，
  // 因为那是另一条路径）。因此非 verbose 模式下一律显式静音。
  solver_options.logging_type =
      options.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;
  solver_options.minimizer_progress_to_stdout = options.verbose;

  ceres::Solve(solver_options, &problem, summary_out);
  return summary_out->IsSolutionUsable();
}

}  // namespace

bool ceresOptimizePath(std::vector<double>& coords,
                       const std::vector<Vec2>& initial_path,
                       const PathOptimizerOptions& options,
                       PathOptimizationReport* report) {
  const std::size_t count = initial_path.size();
  if (count < 3 || coords.size() != count * 2) {
    return false;
  }

  // ---- 外层约束迭代：与内置后端策略一致 ----
  const double allowed =
      options.max_curvature * (1.0 + std::max(options.constraint_tolerance, 0.0));

  double curvature_weight = options.curvature_weight;
  const int max_rounds = std::max(options.max_constraint_rounds, 1);

  ceres::Solver::Summary last_summary;
  bool usable = false;

  for (int round = 0; round < max_rounds; ++round) {
    ceres::Solver::Summary summary;
    usable = solveOnce(coords, initial_path, options, curvature_weight, &summary);
    last_summary = summary;

    if (report != nullptr) {
      report->constraint_rounds = round + 1;
      // Ceres 2.x 起 Summary::iterations 是逐次迭代记录的 vector，
      // 迭代次数取它的长度而非其本身
      report->iterations = static_cast<int>(summary.iterations.size());
      report->message = summary.message;
      report->initial_cost = summary.initial_cost;
      report->final_cost = summary.final_cost;
    }

    if (!usable) break;

    // 检查是否满足曲率约束
    std::vector<Vec2> path(count);
    for (std::size_t i = 0; i < count; ++i) {
      path[i] = Vec2{coords[2 * i], coords[2 * i + 1]};
    }
    if (PathOptimizer::maxAbsCurvature(path) <= allowed) break;

    if (round + 1 >= max_rounds) break;
    curvature_weight *= std::max(options.constraint_weight_growth, 1.0);
  }

  if (report != nullptr) {
    report->converged = usable;
    report->backend = "ceres";
    report->curvature_limit = options.max_curvature;
  }
  return usable;
}

#endif  // ADSIM_HAS_CERES

}  // namespace adsim
