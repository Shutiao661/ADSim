// =============================================================================
//  Delaunay.h — Delaunay 三角剖分与 Voronoi 路线图
//
//  用于路径规划中的"可通行区域骨架"提取：对障碍物点集做 Delaunay 剖分，取
//  三角形外心构成 Voronoi 图，即障碍物之间的最大间隙路径集合，在此基础上
//  搜索得到绕障路径。
//
//  采用 Bowyer-Watson 增量插入算法，复杂度 O(n log n) ~ O(n^1.5)。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <utility>
#include <vector>

namespace adsim {
namespace geometry {

/// 三角形：三个顶点在点集中的索引，按逆时针排列
struct Triangle {
  int a{-1};
  int b{-1};
  int c{-1};

  bool valid() const { return a >= 0 && b >= 0 && c >= 0; }

  bool operator==(const Triangle& o) const {
    return a == o.a && b == o.b && c == o.c;
  }
  bool operator!=(const Triangle& o) const { return !(*this == o); }

  /// 排序后的顶点组合，用于判定两个三角形是否相同（忽略顶点顺序）
  bool sameVertexSet(const Triangle& o) const;
};

class DelaunayTriangulation {
 public:
  /// 构造并对给定点集完成剖分
  explicit DelaunayTriangulation(const std::vector<Vec2>& points);

  const std::vector<Vec2>& points() const { return points_; }
  const std::vector<Triangle>& triangles() const { return triangles_; }
  std::size_t triangleCount() const { return triangles_.size(); }

  /// 无向边列表（去重，保证 first < second）
  std::vector<std::pair<int, int>> edges() const;

  /// 凸包顶点索引，逆时针顺序
  std::vector<int> convexHull() const;

  /// 点定位：返回包含点 p 的三角形索引列表；点落在三角形外部时返回空
  std::vector<int> locate(const Vec2& p) const;

  /// 校验 Delaunay 空圆性质：任一三角形的外接圆内不得含其他点
  /// @param tolerance 数值容差，判定时外接圆半径放宽该倍数
  bool satisfiesEmptyCircleProperty(double tolerance = 1e-9) const;

  /// 三角形外接圆半径；退化三角形（三点共线）返回极大值
  static double circumradius(const Vec2& a, const Vec2& b, const Vec2& c);

  /// 三角形外心；退化时返回三点重心
  static Vec2 circumcenter(const Vec2& a, const Vec2& b, const Vec2& c);

  /// 三角形有向面积的两倍（正表示逆时针）
  static double cross2(const Vec2& a, const Vec2& b, const Vec2& c);

  /// 三角形面积
  static double area(const Vec2& a, const Vec2& b, const Vec2& c);

 private:
  void build();

  std::vector<Vec2> points_;
  std::vector<Triangle> triangles_;
};

// ---------------------------------------------------------------------------
//  Voronoi 路线图
//
//  以 Delaunay 三角形外心为节点、以"共享边的三角形"为连接关系构图。该图是
//  障碍物点集的中轴（medial axis）离散近似，天然远离障碍物，适合作为避障
//  路径的搜索空间。
// ---------------------------------------------------------------------------
class VoronoiRoadmap {
 public:
  /// @param obstacle_points 障碍物离散点集（如激光雷达聚类后的障碍物轮廓点）
  explicit VoronoiRoadmap(const std::vector<Vec2>& obstacle_points);

  /// 在路线图上搜索 start → goal 的路径。
  /// 起终点会自动接入最近的路线图节点，返回折线（含真实起终点）。
  /// 搜索失败（起终点被障碍物包围等）时返回空。
  std::vector<Vec2> search(const Vec2& start, const Vec2& goal) const;

  const std::vector<Vec2>& nodes() const { return nodes_; }
  const std::vector<std::vector<int>>& adjacency() const { return adjacency_; }
  std::size_t nodeCount() const { return nodes_.size(); }

  /// 指定位置到最近障碍物点的距离（用于安全裕度评估）
  double clearanceAt(const Vec2& p) const;

  /// 路径上各点的最小间隙
  double minClearanceAlong(const std::vector<Vec2>& path) const;

 private:
  int nearestNode(const Vec2& p) const;

  std::vector<Vec2> obstacles_;
  std::vector<Vec2> nodes_;
  std::vector<std::vector<int>> adjacency_;
};

}  // namespace geometry
}  // namespace adsim
