// =============================================================================
//  CurveFit.cpp — 曲线拟合与路径预处理工具实现
//
//  本文件是路径规划的前处理/后处理工具箱：
//    * rdpSimplify         —— 折线抽稀，去掉直线段上的冗余采样点
//    * 弧长/重采样/距离/自交 —— 路径几何量计算
//
//  所有函数都对空输入、单点、重复点、零长度线段做了防御，绝不产生除零或 NaN。
// =============================================================================
#include "adsim/planning/geometry/CurveFit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace adsim {
namespace geometry {

namespace {

/// 重合点判定阈值 (m)
constexpr double kTiny = 1e-9;

/// 自交判定中用于叉积符号比较的相对容差（相对于坐标尺度的平方）
constexpr double kOrientEps = 1e-12;

/// 重采样输出的点数上限：防止调用方传入极小步长导致内存爆炸
constexpr std::size_t kMaxResamplePoints = 1000000;

/// 点 p 到直线 ab 的垂距；退化（a 与 b 重合）时退化为点到点距离
double perpendicularDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 <= kTiny * kTiny) return (p - a).norm();
  return std::fabs(ab.cross(p - a)) / std::sqrt(len2);
}

/// 折线段 (a,b) 对查询点的最近距离与最近点的归一化参数 t ∈ [0,1]
double distanceToSegment(const Vec2& query, const Vec2& a, const Vec2& b,
                         double* t_out) {
  const Vec2 ab = b - a;
  const double len2 = ab.squaredNorm();
  double t = 0.0;
  if (len2 > 0.0) {
    t = adsim::clamp((query - a).dot(ab) / len2, 0.0, 1.0);
  }
  if (t_out != nullptr) *t_out = t;
  return (query - (a + ab * t)).norm();
}

}  // namespace

// ---------------------------------------------------------------------------
//  多项式拟合
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  折线抽稀与平滑
// ---------------------------------------------------------------------------

std::vector<Vec2> rdpSimplify(const std::vector<Vec2>& points, double epsilon) {
  const std::size_t n = points.size();
  if (n <= 2) return points;  // 首末点必留，无需处理
  const double eps = epsilon > 0.0 ? epsilon : 0.0;

  std::vector<char> keep(n, 0);
  keep.front() = 1;
  keep.back() = 1;

  // 显式栈代替递归：路点数量可达上万，深递归会耗尽调用栈
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.push_back({0, n - 1});
  while (!stack.empty()) {
    const std::pair<std::size_t, std::size_t> seg = stack.back();
    stack.pop_back();
    const std::size_t a = seg.first;
    const std::size_t b = seg.second;
    if (b <= a + 1) continue;

    double max_dist = -1.0;
    std::size_t max_idx = a;
    for (std::size_t i = a + 1; i < b; ++i) {
      const double d = perpendicularDistance(points[i], points[a], points[b]);
      if (d > max_dist) {
        max_dist = d;
        max_idx = i;
      }
    }
    if (max_dist > eps) {
      keep[max_idx] = 1;
      stack.push_back({a, max_idx});
      stack.push_back({max_idx, b});
    }
  }

  std::vector<Vec2> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (keep[i] != 0) out.push_back(points[i]);
  }
  return out;
}

// ---------------------------------------------------------------------------
//  弧长与重采样
// ---------------------------------------------------------------------------

std::vector<double> accumulateArcLength(const std::vector<Vec2>& points) {
  std::vector<double> arcs(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    arcs[i] = arcs[i - 1] + (points[i] - points[i - 1]).norm();
  }
  return arcs;
}

double polylineLength(const std::vector<Vec2>& points) {
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += (points[i] - points[i - 1]).norm();
  }
  return total;
}

std::vector<Vec2> resampleByArcLength(const std::vector<Vec2>& points,
                                      double step) {
  std::vector<Vec2> out;
  if (points.empty()) return out;
  if (points.size() == 1 || !(step > 0.0)) return points;

  const std::vector<double> arcs = accumulateArcLength(points);
  const double total = arcs.back();
  if (!(total > kTiny)) return {points.front()};  // 零长度折线：退化为单点

  // 步长过小会导致点数爆炸，按上限收缩步长（等间隔性质保持）
  double stride = step;
  if (total / stride > static_cast<double>(kMaxResamplePoints)) {
    stride = total / static_cast<double>(kMaxResamplePoints);
  }

  out.reserve(static_cast<std::size_t>(total / stride) + 2);
  out.push_back(points.front());
  std::size_t seg = 1;  // 当前线段为 points[seg-1] → points[seg]
  for (double s = stride; s < total; s += stride) {
    while (seg + 1 < points.size() && arcs[seg] < s) ++seg;
    const double s0 = arcs[seg - 1];
    const double s1 = arcs[seg];
    const double w = s1 > s0 ? (s - s0) / (s1 - s0) : 0.0;
    out.push_back(points[seg - 1] + (points[seg] - points[seg - 1]) * w);
  }
  // 末段不足一个步长时按实际长度收尾：保证终点被保留
  if ((out.back() - points.back()).norm() > kTiny) out.push_back(points.back());
  return out;
}

double distanceToPolyline(const std::vector<Vec2>& polyline,
                          const Vec2& query, double* arc_length_out) {
  if (arc_length_out != nullptr) *arc_length_out = 0.0;
  if (polyline.empty()) {
    return std::numeric_limits<double>::max();  // 无折线可比，间隙视为无限大
  }
  if (polyline.size() == 1) return (query - polyline.front()).norm();

  double best = std::numeric_limits<double>::max();
  double best_arc = 0.0;
  double acc = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    double t = 0.0;
    const double d = distanceToSegment(query, polyline[i], polyline[i + 1], &t);
    if (d < best) {
      best = d;
      best_arc = acc + (polyline[i + 1] - polyline[i]).norm() * t;
    }
    acc += (polyline[i + 1] - polyline[i]).norm();
  }
  if (arc_length_out != nullptr) *arc_length_out = best_arc;
  return best;
}

}  // namespace geometry
}  // namespace adsim
