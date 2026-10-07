// =============================================================================
//  CeresPathOptimizer.h — 基于 Ceres Solver 的路径优化后端
//
//  **可选依赖**：需要 Ceres Solver，以 `-DADSIM_WITH_CERES=ON` 构建时才参与编译。
//
//  与内置求解器求解的是**同一个数学问题**：
//      平滑项 + 贴合项 + 长度项 + 曲率铰链惩罚
//  区别只在于求解器实现。切换到 Ceres 的收益：
//      * 自动微分，不再需要手写或差分雅可比
//      * 成熟的 LM 实现，含信赖域、非单调下降等策略
//      * 稀疏线性代数（路径优化问题天然稀疏，点上规模后可显著提速）
//
//  因此在装有 Ceres 的环境中，本后端是更优选择；内置求解器的价值在于
//  让工程在无 Ceres 的环境下依然完整可用、可测试。
// =============================================================================
#pragma once

#include "adsim/planning/optimizer/PathOptimizer.h"

#include <vector>

namespace adsim {

/// 当前构建是否包含 Ceres 后端
bool ceresBackendAvailable();

#if defined(ADSIM_HAS_CERES)

/// 用 Ceres 求解路径平滑问题。
///
/// @param coords          输入输出参数：展平的坐标数组 [x0,y0, x1,y1, ...]，
///                        求解后就地更新，长度必须为 2N
/// @param initial_path    原始路径（贴合项的参考），长度必须为 N
/// @param options         优化选项
/// @param report          非空时填充求解统计
/// @return 是否成功求解（求解器报错时返回 false）
bool ceresOptimizePath(std::vector<double>& coords,
                       const std::vector<Vec2>& initial_path,
                       const PathOptimizerOptions& options,
                       PathOptimizationReport* report);

#endif  // ADSIM_HAS_CERES

}  // namespace adsim
