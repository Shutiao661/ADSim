#include "adsim/planning/optimizer/NonlinearSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace adsim {

// ---------------------------------------------------------------------------
// Problem
// ---------------------------------------------------------------------------

void Problem::addResidualBlock(std::shared_ptr<ResidualBlock> block,
                               const std::vector<double*>& params, double weight) {
  if (block == nullptr) {
    throw std::invalid_argument("残差块不能为空");
  }

  const std::vector<int> sizes = block->parameterBlockSizes();
  if (sizes.size() != params.size()) {
    throw std::invalid_argument("参数块个数与残差块声明不符");
  }
  for (double* ptr : params) {
    if (ptr == nullptr) {
      throw std::invalid_argument("参数指针不能为空");
    }
  }

  Term term;
  term.block = std::move(block);
  term.params = params;
  // 取平方根后同时作用于残差与雅可比，使代价里的权重恰为 w 而非 w²
  term.weight = weight;
  term.sqrt_weight = std::sqrt(std::max(weight, 0.0));
  terms_.push_back(std::move(term));
}

void Problem::setParameterBounds(double* params, std::vector<double> lower,
                                 std::vector<double> upper) {
  bounds_.push_back(Bounds{params, std::move(lower), std::move(upper)});
}

std::size_t Problem::residualCount() const {
  std::size_t total = 0;
  for (const Term& term : terms_) {
    total += static_cast<std::size_t>(term.block->residualDimension());
  }
  return total;
}

double Problem::computeCost() const {
  double cost = 0.0;
  std::vector<double> residuals;

  for (const Term& term : terms_) {
    const auto dim = static_cast<std::size_t>(term.block->residualDimension());
    residuals.resize(dim);
    term.block->evaluate(term.params.data(), residuals.data(), nullptr);
    for (std::size_t i = 0; i < dim; ++i) {
      cost += term.weight * residuals[i] * residuals[i];
    }
  }
  return 0.5 * cost;
}

double Problem::verifyJacobian(std::size_t block_index, double step) const {
  if (block_index >= terms_.size()) {
    return std::numeric_limits<double>::max();
  }

  const Term& term = terms_[block_index];
  const int rdim = term.block->residualDimension();
  const std::vector<int> sizes = term.block->parameterBlockSizes();
  const int pdim = term.block->totalParameterDimension();
  if (rdim <= 0 || pdim <= 0) {
    return std::numeric_limits<double>::max();
  }

  const auto rsize = static_cast<std::size_t>(rdim);
  const auto psize = static_cast<std::size_t>(pdim);

  std::vector<double> analytic(rsize * psize, 0.0);
  std::vector<double> residuals(rsize, 0.0);
  term.block->evaluate(term.params.data(), residuals.data(), analytic.data());

  std::vector<std::vector<double>> params(term.params.size());
  for (std::size_t b = 0; b < term.params.size(); ++b) {
    params[b].assign(term.params[b], term.params[b] + sizes[b]);
  }
  std::vector<const double*> ptrs(term.params.size(), nullptr);

  std::vector<double> r_plus(rsize, 0.0);
  std::vector<double> r_minus(rsize, 0.0);

  double max_error = 0.0;
  double scale = 1e-12;

  int global_col = 0;
  for (std::size_t b = 0; b < params.size(); ++b) {
    for (int j = 0; j < sizes[b]; ++j, ++global_col) {
      // 步长随参数自身量级缩放：参数很大时固定步长会被舍入误差淹没
      const double h = step * std::max(1.0, std::abs(params[b][static_cast<std::size_t>(j)]));

      params[b][static_cast<std::size_t>(j)] += h;
      for (std::size_t k = 0; k < params.size(); ++k) ptrs[k] = params[k].data();
      term.block->evaluate(ptrs.data(), r_plus.data(), nullptr);

      params[b][static_cast<std::size_t>(j)] -= 2.0 * h;
      for (std::size_t k = 0; k < params.size(); ++k) ptrs[k] = params[k].data();
      term.block->evaluate(ptrs.data(), r_minus.data(), nullptr);

      params[b][static_cast<std::size_t>(j)] += h;  // 复原

      for (int i = 0; i < rdim; ++i) {
        const double numeric = (r_plus[static_cast<std::size_t>(i)] -
                                r_minus[static_cast<std::size_t>(i)]) /
                               (2.0 * h);
        const double analytic_value =
            analytic[static_cast<std::size_t>(i) * psize + static_cast<std::size_t>(global_col)];
        max_error = std::max(max_error, std::abs(numeric - analytic_value));
        scale = std::max(scale, std::abs(numeric));
      }
    }
  }

  return max_error / scale;
}

namespace {

/// 原地求解 A·x = b（A 为 n×n 行主序，会被破坏）。矩阵奇异时返回 false。
bool solveDense(std::vector<double>& A, std::vector<double>& b, int n) {
  for (int col = 0; col < n; ++col) {
    // 部分主元：把该列绝对值最大的行换到当前行
    int pivot = col;
    double best = std::abs(A[static_cast<std::size_t>(col) * n + col]);
    for (int row = col + 1; row < n; ++row) {
      const double value = std::abs(A[static_cast<std::size_t>(row) * n + col]);
      if (value > best) {
        best = value;
        pivot = row;
      }
    }

    if (best < 1e-14) {
      return false;  // 主元过小，判定为奇异
    }

    if (pivot != col) {
      for (int k = 0; k < n; ++k) {
        std::swap(A[static_cast<std::size_t>(col) * n + k],
                  A[static_cast<std::size_t>(pivot) * n + k]);
      }
      std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(pivot)]);
    }

    const double diag = A[static_cast<std::size_t>(col) * n + col];
    for (int row = col + 1; row < n; ++row) {
      const double factor = A[static_cast<std::size_t>(row) * n + col] / diag;
      if (factor == 0.0) continue;
      for (int k = col; k < n; ++k) {
        A[static_cast<std::size_t>(row) * n + k] -=
            factor * A[static_cast<std::size_t>(col) * n + k];
      }
      b[static_cast<std::size_t>(row)] -= factor * b[static_cast<std::size_t>(col)];
    }
  }

  for (int row = n - 1; row >= 0; --row) {
    double sum = b[static_cast<std::size_t>(row)];
    for (int k = row + 1; k < n; ++k) {
      sum -= A[static_cast<std::size_t>(row) * n + k] * b[static_cast<std::size_t>(k)];
    }
    b[static_cast<std::size_t>(row)] = sum / A[static_cast<std::size_t>(row) * n + row];
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 求解主循环
// ---------------------------------------------------------------------------

SolverSummary NonlinearSolver::solve(Problem& problem) {
  return solve(problem, SolverOptions());
}

SolverSummary NonlinearSolver::solve(Problem& problem, const SolverOptions& options) {
  const auto wall_start = std::chrono::steady_clock::now();
  SolverSummary summary;

  if (problem.terms_.empty()) {
    summary.converged = true;
    summary.message = "问题为空，无需优化";
    return summary;
  }

  // ---- 1. 建立全局参数表 ----
  // 同一个参数块会被多个残差块引用（相邻的平滑残差共享路径点），
  // 必须按指针去重，否则海森矩阵会被重复叠加。
  std::vector<double*> unique_params;
  std::vector<int> unique_dims;

  for (const Problem::Term& term : problem.terms_) {
    const std::vector<int> sizes = term.block->parameterBlockSizes();
    for (std::size_t b = 0; b < term.params.size(); ++b) {
      auto it = std::find(unique_params.begin(), unique_params.end(), term.params[b]);
      if (it == unique_params.end()) {
        unique_params.push_back(term.params[b]);
        unique_dims.push_back(sizes[b]);
      } else {
        const auto index = static_cast<std::size_t>(it - unique_params.begin());
        if (unique_dims[index] != sizes[b]) {
          // 同一参数块被声明为不同维数，说明代价函数定义有误
          throw std::invalid_argument("同一参数块被不同残差块声明为不同维数");
        }
      }
    }
  }

  std::vector<int> global_offset(unique_params.size(), 0);
  int total_parameters = 0;
  for (std::size_t p = 0; p < unique_params.size(); ++p) {
    global_offset[p] = total_parameters;
    total_parameters += unique_dims[p];
  }

  // 每个残差块的各参数块在全局向量中的起始列
  std::vector<std::vector<int>> term_block_offset(problem.terms_.size());
  for (std::size_t t = 0; t < problem.terms_.size(); ++t) {
    const Problem::Term& term = problem.terms_[t];
    term_block_offset[t].resize(term.params.size(), 0);
    for (std::size_t b = 0; b < term.params.size(); ++b) {
      const auto index = static_cast<std::size_t>(
          std::find(unique_params.begin(), unique_params.end(), term.params[b]) -
          unique_params.begin());
      term_block_offset[t][b] = global_offset[index];
    }
  }

  const int num_residuals = static_cast<int>(problem.residualCount());
  if (total_parameters == 0 || num_residuals == 0) {
    summary.converged = true;
    summary.message = "无有效参数或残差";
    return summary;
  }

  // ---- 2. 装配残差与雅可比 ----
  std::vector<double> residuals(static_cast<std::size_t>(num_residuals), 0.0);
  std::vector<double> jacobian(static_cast<std::size_t>(num_residuals) * total_parameters,
                               0.0);

  // 局部缓冲预分配并复用：assemble 每次迭代要调用多次，逐次分配会成为主要开销
  std::size_t max_local_residual = 0;
  std::size_t max_local_jacobian = 0;
  for (const Problem::Term& term : problem.terms_) {
    const auto rdim = static_cast<std::size_t>(term.block->residualDimension());
    const auto pdim = static_cast<std::size_t>(term.block->totalParameterDimension());
    max_local_residual = std::max(max_local_residual, rdim);
    max_local_jacobian = std::max(max_local_jacobian, rdim * pdim);
  }
  std::vector<double> local_r(max_local_residual, 0.0);
  std::vector<double> local_j(max_local_jacobian, 0.0);

  auto assemble = [&]() {
    std::fill(jacobian.begin(), jacobian.end(), 0.0);

    int row_offset = 0;
    for (std::size_t t = 0; t < problem.terms_.size(); ++t) {
      const Problem::Term& term = problem.terms_[t];
      const int rdim = term.block->residualDimension();
      const std::vector<int> sizes = term.block->parameterBlockSizes();

      term.block->evaluate(term.params.data(), local_r.data(), local_j.data());

      // 残差与雅可比同乘 sqrt(weight)，使 ‖r‖² 的权重为 weight
      for (int i = 0; i < rdim; ++i) {
        residuals[static_cast<std::size_t>(row_offset + i)] =
            local_r[static_cast<std::size_t>(i)] * term.sqrt_weight;
      }

      int local_col = 0;
      for (std::size_t b = 0; b < term.params.size(); ++b) {
        const int col_offset = term_block_offset[t][b];
        for (int j = 0; j < sizes[b]; ++j, ++local_col) {
          for (int i = 0; i < rdim; ++i) {
            jacobian[static_cast<std::size_t>(row_offset + i) * total_parameters +
                     col_offset + j] =
                local_j[static_cast<std::size_t>(i) *
                            static_cast<std::size_t>(term.block->totalParameterDimension()) +
                        static_cast<std::size_t>(local_col)] *
                term.sqrt_weight;
          }
        }
      }

      row_offset += rdim;
    }
  };

  // 把全局增量写回各参数指针；越界时投影回边界
  auto applyDelta = [&](const std::vector<double>& delta, double scale) {
    for (std::size_t p = 0; p < unique_params.size(); ++p) {
      double* ptr = unique_params[p];
      const int dim = unique_dims[p];
      const int offset = global_offset[p];
      for (int i = 0; i < dim; ++i) {
        ptr[i] += scale * delta[static_cast<std::size_t>(offset + i)];
      }
    }

    for (const Problem::Bounds& bound : problem.bounds_) {
      const auto index = static_cast<std::size_t>(
          std::find(unique_params.begin(), unique_params.end(), bound.params) -
          unique_params.begin());
      if (index >= unique_params.size()) continue;
      const int dim = unique_dims[index];
      for (int i = 0; i < dim; ++i) {
        if (i < static_cast<int>(bound.lower.size())) {
          bound.params[i] =
              std::max(bound.params[i], bound.lower[static_cast<std::size_t>(i)]);
        }
        if (i < static_cast<int>(bound.upper.size())) {
          bound.params[i] =
              std::min(bound.params[i], bound.upper[static_cast<std::size_t>(i)]);
        }
      }
    }
  };

  std::vector<std::vector<double>> parameter_backup(unique_params.size());
  auto saveParameters = [&]() {
    for (std::size_t p = 0; p < unique_params.size(); ++p) {
      parameter_backup[p].assign(unique_params[p], unique_params[p] + unique_dims[p]);
    }
  };
  auto restoreParameters = [&]() {
    for (std::size_t p = 0; p < unique_params.size(); ++p) {
      std::copy(parameter_backup[p].begin(), parameter_backup[p].end(), unique_params[p]);
    }
  };

  // ---- 3. 迭代 ----
  assemble();

  double current_cost = 0.0;
  for (double r : residuals) current_cost += 0.5 * r * r;

  summary.initial_cost = current_cost;
  summary.final_cost = current_cost;
  double previous_cost = current_cost;

  const bool use_lm = options.algorithm == SolverOptions::Algorithm::kLevenbergMarquardt;
  double damping = options.initial_damping;

  std::vector<double> hessian(static_cast<std::size_t>(total_parameters) * total_parameters,
                              0.0);
  std::vector<double> gradient(static_cast<std::size_t>(total_parameters), 0.0);
  std::vector<double> delta(static_cast<std::size_t>(total_parameters), 0.0);

  std::string stop_reason = "达到最大迭代次数";

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    summary.iterations = iter + 1;

    // ---- H = JᵀJ，g = Jᵀr ----
    std::fill(hessian.begin(), hessian.end(), 0.0);
    std::fill(gradient.begin(), gradient.end(), 0.0);

    for (int i = 0; i < num_residuals; ++i) {
      const double* row = &jacobian[static_cast<std::size_t>(i) * total_parameters];
      const double r = residuals[static_cast<std::size_t>(i)];

      for (int j = 0; j < total_parameters; ++j) {
        if (row[j] == 0.0) continue;
        gradient[static_cast<std::size_t>(j)] += row[j] * r;
        for (int k = j; k < total_parameters; ++k) {
          if (row[k] == 0.0) continue;
          hessian[static_cast<std::size_t>(j) * total_parameters + k] += row[j] * row[k];
        }
      }
    }
    // 利用对称性补齐下三角
    for (int j = 0; j < total_parameters; ++j) {
      for (int k = 0; k < j; ++k) {
        hessian[static_cast<std::size_t>(j) * total_parameters + k] =
            hessian[static_cast<std::size_t>(k) * total_parameters + j];
      }
    }

    const double gradient_norm = std::sqrt(
        std::inner_product(gradient.begin(), gradient.end(), gradient.begin(), 0.0));
    summary.gradient_norm = gradient_norm;

    if (gradient_norm < options.gradient_tolerance) {
      summary.converged = true;
      stop_reason = "梯度范数已低于阈值";
      break;
    }

    // ---- 求解增量并试探 ----
    bool step_accepted = false;
    double step_scale = 1.0;

    for (int attempt = 0; attempt < 12; ++attempt) {
      std::vector<double> A = hessian;
      std::vector<double> b(total_parameters);

      for (int j = 0; j < total_parameters; ++j) {
        b[static_cast<std::size_t>(j)] = -gradient[static_cast<std::size_t>(j)];
      }

      if (use_lm) {
        // 阻尼按对角元素缩放，使其对各参数尺度自适应
        for (int j = 0; j < total_parameters; ++j) {
          const double diag = hessian[static_cast<std::size_t>(j) * total_parameters + j];
          A[static_cast<std::size_t>(j) * total_parameters + j] +=
              damping * std::max(diag, 1e-6);
        }
      } else {
        // 纯高斯-牛顿在奇异方向上无阻尼会发散，加极小正则项保证可解
        for (int j = 0; j < total_parameters; ++j) {
          A[static_cast<std::size_t>(j) * total_parameters + j] += 1e-12;
        }
      }

      if (!solveDense(A, b, total_parameters)) {
        if (use_lm) {
          damping = std::min(damping * options.damping_increase, options.max_damping);
          continue;  // 加大阻尼后重试
        }
        stop_reason = "海森矩阵奇异，高斯-牛顿无法继续";
        break;
      }

      delta = b;

      saveParameters();
      applyDelta(delta, step_scale);
      assemble();

      double trial_cost = 0.0;
      for (double r : residuals) trial_cost += 0.5 * r * r;

      if (trial_cost < current_cost) {
        current_cost = trial_cost;
        step_accepted = true;
        if (use_lm) {
          damping = std::max(damping * options.damping_decrease, options.min_damping);
        }
        break;
      }

      restoreParameters();
      assemble();

      if (use_lm) {
        damping = std::min(damping * options.damping_increase, options.max_damping);
      } else {
        // 高斯-牛顿没有阻尼可调，退化为缩小步长的线搜索
        step_scale *= 0.5;
        if (step_scale < 1e-6) break;
      }
    }

    if (!step_accepted) {
      summary.final_cost = current_cost;
      stop_reason = use_lm ? "阻尼升至上限仍无法下降" : "线搜索未能找到下降方向";
      break;
    }

    // ---- 收敛判据 ----
    const double delta_norm =
        std::sqrt(std::inner_product(delta.begin(), delta.end(), delta.begin(), 0.0));
    if (delta_norm < options.parameter_tolerance) {
      summary.converged = true;
      stop_reason = "参数更新量已低于阈值";
      break;
    }

    if (iter > 0 && std::abs(previous_cost - current_cost) /
                            std::max(std::abs(current_cost), 1e-30) <
                        options.cost_tolerance) {
      summary.converged = true;
      stop_reason = "代价下降已低于阈值";
      break;
    }
    previous_cost = current_cost;
  }

  summary.final_cost = current_cost;
  summary.message = stop_reason;
  summary.elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start)
          .count();

  return summary;
}

std::string SolverSummary::toString() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);

  oss << "优化" << (converged ? "收敛" : "未收敛");
  oss << ": 迭代 " << iterations << " 次";
  oss.precision(6);
  oss << ", 代价 " << initial_cost << " → " << final_cost;
  oss.precision(2);
  oss << " (下降 " << costReduction() * 100.0 << "%)";
  oss.precision(3);
  oss << ", 梯度范数 " << gradient_norm << ", 耗时 " << elapsed_ms << " ms";
  if (!message.empty()) {
    oss << "  [" << message << "]";
  }
  return oss.str();
}

}  // namespace adsim
