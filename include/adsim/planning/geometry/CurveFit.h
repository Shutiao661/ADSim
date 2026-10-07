// =============================================================================
//  CurveFit.h — 路径预处理工具
//
//  这里只保留 PathPlanner 与路径平滑实际会用到的工具。
//  早期版本还包含 polyfit / polyeval / movingAverageSmooth / isSelfIntersecting
//  等通用工具，它们有单元测试却没有任何生产调用点——"实现了、测过了、但没人用"
//  是比"没实现"更糟的状态，会让人怀疑整个模块是不是凑数的，因此已移除。
//  需要时再按需加回，而不是提前囤积。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <vector>

namespace adsim {
namespace geometry {

/// Ramer-Douglas-Peucker 折线抽稀。
/// 被路径平滑用作第一步：先去掉直线段上的冗余采样点，再拟合。
std::vector<Vec2> rdpSimplify(const std::vector<Vec2>& points, double epsilon);

/// 折线累计弧长，返回值长度与 points 相同，首元素为 0
std::vector<double> accumulateArcLength(const std::vector<Vec2>& points);

/// 按弧长等间隔重采样折线。
/// 优化器的长度项与曲率项都依赖均匀的点间距，因此平滑前必须重采样。
std::vector<Vec2> resampleByArcLength(const std::vector<Vec2>& points, double step);

/// 点到折线的最短距离，并输出最近点在折线上的弧长位置。
/// 用于量化"平滑结果偏离原路径多远"——这是选择平滑后端的关键判据。
double distanceToPolyline(const std::vector<Vec2>& polyline,
                          const Vec2& query,
                          double* arc_length_out = nullptr);

/// 折线总长度
double polylineLength(const std::vector<Vec2>& points);

}  // namespace geometry
}  // namespace adsim
