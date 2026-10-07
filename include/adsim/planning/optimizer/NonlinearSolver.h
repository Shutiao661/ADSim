// =============================================================================
//  NonlinearSolver.h — 非线性最小二乘求解器
//
//  路径平滑的本质是一个带约束的非线性最小二乘问题：
//       min_x  Σ wᵢ · ‖rᵢ(x)‖²
//  其中决策变量 x 是路径点的坐标，残差项 rᵢ 分别刻画平滑度、与原路径的
//  偏离度、以及曲率超限的惩罚。
//
//  本文件提供的接口刻意与 Ceres Solver 保持同构：
//      ResidualBlock  ↔ ceres::CostFunction
//      Problem        ↔ ceres::Problem
//      SolverOptions  ↔ ceres::Solver::Options
//      SolverSummary  ↔ ceres::Solver::Summary
//
//  这样做的目的是：默认构建下用自带实现（零依赖、可离线跑、可单步调试），
//  在装有 Ceres 的环境中打开 ADSIM_WITH_CERES 即可切换到工业级求解器，
//  而上层的代价函数定义一行都不用改。
// =============================================================================
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 残差块：代价函数的最小单元。
///
/// 与 Ceres 的 CostFunction 一样支持**多个参数块**——这是必需的，
/// 因为一个平滑残差天然同时依赖相邻三个路径点，把它们拆成三个独立残差
/// 会丢失耦合信息（雅可比里点与点之间的交叉项全部为零）。
class ResidualBlock {
 public:
  virtual ~ResidualBlock() = default;

  /// 残差维数
  virtual int residualDimension() const = 0;

  /// 各参数块的维数。长度即该残差依赖的参数块个数。
  virtual std::vector<int> parameterBlockSizes() const = 0;

  /// 计算残差。
  /// @param params    各参数块的首地址数组，长度等于 parameterBlockSizes().size()
  /// @param residuals 输出残差，长度等于 residualDimension()
  /// @param jacobian  非空时按行主序填充
  ///                  residualDimension() × ΣparameterBlockSizes()
  ///                  的雅可比矩阵；为 nullptr 时只需计算残差
  virtual void evaluate(const double* const* params, double* residuals,
                        double* jacobian) = 0;

  /// 参数总维数
  int totalParameterDimension() const {
    int total = 0;
    for (int size : parameterBlockSizes()) total += size;
    return total;
  }
};

class Problem {
 public:
  /// 添加残差块。
  ///
  /// @param block  代价函数
  /// @param params 各参数块的首地址。**由调用方持有并保证生命周期**，
  ///               求解器只读写其内容，不接管所有权。
  /// @param weight 该残差项的权重（内部对残差与雅可比同时乘以 sqrt(weight)）
  void addResidualBlock(std::shared_ptr<ResidualBlock> block,
                        const std::vector<double*>& params, double weight = 1.0);

  /// 设置某个参数块的上下界；求解器每步更新后会做投影
  void setParameterBounds(double* params, std::vector<double> lower,
                          std::vector<double> upper);

  std::size_t blockCount() const { return terms_.size(); }
  std::size_t residualCount() const;

  /// 当前参数下的代价 ½·Σ wᵢ‖rᵢ‖²
  double computeCost() const;

  /// 用中心差分校验第 block_index 个残差块的解析雅可比，返回最大相对误差。
  /// 索引越界时返回极大值。
  double verifyJacobian(std::size_t block_index, double step = 1e-6) const;

 private:
  // 求解器需要遍历残差块与其参数指针；这与 Ceres 中 Solver 作为 Problem
  // 友元的做法一致——对外部调用方这些细节没有暴露价值。
  friend class NonlinearSolver;

  struct Term {
    std::shared_ptr<ResidualBlock> block;
    std::vector<double*> params;
    double weight{1.0};
    double sqrt_weight{1.0};
  };

  struct Bounds {
    double* params{nullptr};
    std::vector<double> lower;
    std::vector<double> upper;
  };

  std::vector<Term> terms_;
  std::vector<Bounds> bounds_;
};

struct SolverOptions {
  enum class Algorithm {
    kGaussNewton,          ///< 高斯-牛顿：收敛快，但在病态问题上易发散
    kLevenbergMarquardt,   ///< 列文伯格-马夸尔特：带阻尼，鲁棒性更好
  };

  Algorithm algorithm{Algorithm::kLevenbergMarquardt};
  int max_iterations{100};
  /// 梯度范数收敛阈值
  double gradient_tolerance{1e-10};
  /// 参数更新量收敛阈值
  double parameter_tolerance{1e-12};
  /// 代价下降阈值
  double cost_tolerance{1e-12};
  /// LM 初始阻尼系数
  double initial_damping{1e-4};
  /// LM 阻尼的放大/缩小倍数
  double damping_increase{10.0};
  double damping_decrease{0.1};
  double max_damping{1e10};
  double min_damping{1e-14};
  bool verbose{false};
};

struct SolverSummary {
  bool converged{false};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  double gradient_norm{0.0};
  double elapsed_ms{0.0};
  std::string message;

  /// 代价相对初始值的下降比例
  double costReduction() const {
    return initial_cost <= 0.0 ? 0.0 : 1.0 - final_cost / initial_cost;
  }

  std::string toString() const;
};

class NonlinearSolver {
 public:
  static SolverSummary solve(Problem& problem, const SolverOptions& options);

  /// 使用默认选项求解
  static SolverSummary solve(Problem& problem);
};

}  // namespace adsim
