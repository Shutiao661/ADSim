// =============================================================================
//  test_geometry.cpp — 计算几何层单元测试
//  覆盖：Delaunay 三角剖分（含退化输入、空圆性质、凸包、边去重）、
//        Voronoi 路线图、三次 B 样条（求值/导数/曲率/弧长/最小二乘拟合/曲率约束平滑）、
//        曲线拟合工具（多项式拟合 / RDP / 滑动平均 / 弧长与重采样 / 距离 / 自交）
//
//  断言容差的选取原则：凡是有闭式解或可独立复算的量（端点插值、Menger 曲率、
//  欧拉计数、解析导数、弧长）都按"实测误差放大 10~100 倍"给容差；凡是迭代/数值
//  近似量（拟合 rms、平滑后的最大曲率）都按接口约定给出上界，而不是照抄实测值。
// =============================================================================
#include "TestFramework.h"

#include "adsim/planning/geometry/CurveFit.h"
#include "adsim/planning/geometry/Delaunay.h"
#include "adsim/planning/geometry/Spline.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace adsim;
using namespace adsim::geometry;

namespace {

/// 固定种子的均匀随机点集（测试必须可重复）
std::vector<Vec2> randomPoints(int n, double lo, double hi, unsigned seed,
                               double skip_first_blocks = 0.0) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ud(lo, hi);
  std::vector<Vec2> pts;
  // 先空转若干点，便于取同一序列的不同片段
  for (int i = 0; i < static_cast<int>(skip_first_blocks); ++i) ud(rng);
  pts.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) pts.push_back({ud(rng), ud(rng)});
  return pts;
}

std::vector<Vec2> circlePoints(int count, double radius) {
  std::vector<Vec2> pts;
  pts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    pts.push_back({radius * std::cos(a), radius * std::sin(a)});
  }
  return pts;
}

/// 折线累计弧长（测试侧独立实现，不复用被测代码）
double independentLength(const std::vector<Vec2>& pts) {
  double s = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) s += (pts[i] - pts[i - 1]).norm();
  return s;
}

/// 轨迹 → 折线位置序列
std::vector<Vec2> positions(const Trajectory& traj) {
  std::vector<Vec2> pts;
  pts.reserve(traj.size());
  for (const TrajectoryPoint& tp : traj) pts.push_back(tp.position());
  return pts;
}

/// 统计有向边使用次数（key 已排序，保证 first < second）
std::map<std::pair<int, int>, int> edgeUsage(const std::vector<Triangle>& tris) {
  std::map<std::pair<int, int>, int> use;
  for (const Triangle& t : tris) {
    const int v[3] = {t.a, t.b, t.c};
    for (int k = 0; k < 3; ++k) {
      int x = v[k];
      int y = v[(k + 1) % 3];
      if (x > y) std::swap(x, y);
      ++use[{x, y}];
    }
  }
  return use;
}

/// 双列点阵构成的竖直墙（带缺口时留出 |y| < gap_half 的通道）
std::vector<Vec2> wallPoints(double thickness, double gap_half, double y_half,
                             double spacing) {
  std::vector<Vec2> pts;
  for (double y = -y_half; y <= y_half + 1e-9; y += spacing) {
    if (std::fabs(y) < gap_half) continue;
    pts.push_back({0.0, y});
    pts.push_back({thickness, y});
  }
  return pts;
}

/// 路径与 x = 0 的交点纵坐标；不穿越时返回 1e9
double crossingY(const std::vector<Vec2>& path) {
  double cy = 1e9;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if ((path[i - 1].x < 0.0) != (path[i].x < 0.0)) {
      const double t = -path[i - 1].x / (path[i].x - path[i - 1].x);
      cy = path[i - 1].y + t * (path[i].y - path[i - 1].y);
    }
  }
  return cy;
}

/// 回折 + 参数聚簇的路点（14 点，含 158° 折角）：B 样条最小二乘在该类输入上
/// 法方程条件数急剧恶化，是控制点尺度失控/平滑曲线发散的最小复现用例
std::vector<Vec2> foldedPath() {
  return {{2.781, 3.163},  {3.299, 9.902},   {4.825, 5.232},   {5.485, 14.847},
          {6.770, 16.868}, {8.151, 16.700},  {10.534, 22.940}, {11.307, 31.029},
          {13.998, 30.838},{15.940, 34.236}, {18.046, 33.761}, {20.336, 29.538},
          {23.180, 31.642},{23.766, 30.743}};
}

}  // namespace

// ===========================================================================
//  Delaunay：静态几何量
// ===========================================================================

ADSIM_TEST(Geometry, 三角形基础量) {
  const Vec2 a{0.0, 0.0};
  const Vec2 b{4.0, 0.0};
  const Vec2 c{0.0, 3.0};  // 逆时针
  // cross2 等于有向面积的两倍：逆时针为正、顺时针为负、共线为 0
  ADSIM_CHECK_NEAR(DelaunayTriangulation::cross2(a, b, c), 12.0, 1e-12);
  ADSIM_CHECK_NEAR(DelaunayTriangulation::cross2(a, c, b), -12.0, 1e-12);
  ADSIM_CHECK_NEAR(DelaunayTriangulation::cross2(a, b, {8.0, 0.0}), 0.0, 1e-12);
  // area 取绝对值
  ADSIM_CHECK_NEAR(DelaunayTriangulation::area(a, b, c), 6.0, 1e-12);
  ADSIM_CHECK_NEAR(DelaunayTriangulation::area(a, c, b), 6.0, 1e-12);
  ADSIM_CHECK_NEAR(DelaunayTriangulation::area(a, b, {8.0, 0.0}), 0.0, 1e-12);

  // 直角三角形：斜边中点即外心，半径等于斜边一半
  ADSIM_CHECK_NEAR(DelaunayTriangulation::circumradius(a, b, c), 2.5, 1e-12);
  const Vec2 o = DelaunayTriangulation::circumcenter(a, b, c);
  ADSIM_CHECK_NEAR(o.x, 2.0, 1e-12);
  ADSIM_CHECK_NEAR(o.y, 1.5, 1e-12);
  for (const Vec2& v : {a, b, c}) {
    ADSIM_CHECK_NEAR((v - o).norm(), 2.5, 1e-12);
  }

  // 退化（三点共线）：半径取极大值，外心取重心，均不得为 NaN
  const Vec2 d{8.0, 0.0};
  const double r = DelaunayTriangulation::circumradius(a, b, d);
  ADSIM_CHECK(r >= 1e300);
  const Vec2 oc = DelaunayTriangulation::circumcenter(a, b, d);
  ADSIM_CHECK_NEAR(oc.x, (a.x + b.x + d.x) / 3.0, 1e-12);
  ADSIM_CHECK_NEAR(oc.y, 0.0, 1e-12);
  ADSIM_CHECK(std::isfinite(oc.x) && std::isfinite(oc.y));

  // sameVertexSet：与顶点顺序无关
  const Triangle t{1, 2, 3};
  ADSIM_CHECK(t.sameVertexSet(Triangle{3, 1, 2}));
  ADSIM_CHECK(t.sameVertexSet(Triangle{2, 3, 1}));
  ADSIM_CHECK(!t.sameVertexSet(Triangle{1, 2, 4}));
  ADSIM_CHECK(t.valid());
  // 宏参数里的花括号不保护逗号，故整体再加一层小括号
  ADSIM_CHECK((!Triangle{-1, 0, 1}.valid()));
  ADSIM_CHECK((t == Triangle{1, 2, 3}));
  ADSIM_CHECK((t != Triangle{1, 2, 4}));
}

// ===========================================================================
//  Delaunay：退化输入（不得崩溃、不得死循环、不得产生 NaN）
// ===========================================================================

ADSIM_TEST(Geometry, 退化输入不崩溃) {
  struct Case {
    const char* name;
    std::vector<Vec2> pts;
  };
  const std::vector<Case> cases = {
      {"空集", {}},
      {"单点", {{1.0, 2.0}}},
      {"两点", {{1.0, 2.0}, {3.0, 4.0}}},
      {"三点共线", {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}}},
      {"全重合", {{5.0, 5.0}, {5.0, 5.0}, {5.0, 5.0}, {5.0, 5.0}}},
      {"重合+两点", {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}}},
  };
  for (const Case& c : cases) {
    DelaunayTriangulation dt(c.pts);
    // 不足 3 个不重合点时不构成三角形
    ADSIM_CHECK_MSG(dt.triangleCount() == 0, c.name);
    ADSIM_CHECK_MSG(dt.edges().empty(), c.name);
    ADSIM_CHECK_MSG(dt.satisfiesEmptyCircleProperty(), c.name);
    // 空三角形集合下定位必须返回空而不是非法下标
    const std::vector<int> hits = dt.locate({0.5, 0.5});
    for (int idx : hits) {
      ADSIM_CHECK_MSG(idx >= 0 && static_cast<std::size_t>(idx) < dt.triangles().size(), c.name);
    }
  }

  // 50 点共线：剖分为空，凸包退化为两端点
  std::vector<Vec2> line;
  for (int i = 0; i < 50; ++i) line.push_back({static_cast<double>(i), 2.0 * i});
  DelaunayTriangulation dl(line);
  ADSIM_CHECK_EQ(dl.triangleCount(), std::size_t{0});
  ADSIM_CHECK(dl.satisfiesEmptyCircleProperty());
  const std::vector<int> hl = dl.convexHull();
  ADSIM_CHECK_EQ(hl.size(), std::size_t{2});
  for (int idx : hl) {
    ADSIM_CHECK(idx >= 0 && idx < 50);
  }

  // 大量重合点只保留一个顶点
  std::vector<Vec2> dup(200, Vec2{3.0, 4.0});
  DelaunayTriangulation dd(dup);
  ADSIM_CHECK_EQ(dd.triangleCount(), std::size_t{0});
  ADSIM_CHECK_EQ(dd.convexHull().size(), std::size_t{1});
  // 非法坐标（NaN/Inf）必须被忽略而不是污染结果
  std::vector<Vec2> bad = {{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0},
                           {std::numeric_limits<double>::quiet_NaN(), 1.0},
                           {std::numeric_limits<double>::infinity(), 2.0}};
  DelaunayTriangulation db(bad);
  for (const Triangle& t : db.triangles()) {
    for (int idx : {t.a, t.b, t.c}) {
      ADSIM_CHECK(std::isfinite(db.points()[static_cast<std::size_t>(idx)].x));
      ADSIM_CHECK(std::isfinite(db.points()[static_cast<std::size_t>(idx)].y));
    }
  }
  ADSIM_CHECK_EQ(db.triangleCount(), std::size_t{1});
}

// ===========================================================================
//  Delaunay：随机点集的拓扑正确性与空圆性质
// ===========================================================================

ADSIM_TEST(Geometry, 随机集拓扑与空圆) {
  const std::vector<Vec2> pts = randomPoints(300, -50.0, 50.0, 20240924u);
  DelaunayTriangulation dt(pts);
  const std::vector<Triangle>& tris = dt.triangles();
  const std::vector<int> hull = dt.convexHull();

  // 1) 欧拉关系：平面三角剖分满足 T = 2n - 2 - h，边数 E = 3n - 3 - h
  ADSIM_CHECK_EQ(tris.size(), std::size_t{2} * pts.size() - 2 - hull.size());
  ADSIM_CHECK_EQ(dt.edges().size(), std::size_t{3} * pts.size() - 3 - hull.size());

  // 2) 所有三角形逆时针，且每个点都是某个三角形的顶点（凸包内的点一个不能丢）
  std::set<int> used;
  for (const Triangle& t : tris) {
    ADSIM_CHECK(t.valid());
    ADSIM_CHECK(DelaunayTriangulation::cross2(pts[static_cast<std::size_t>(t.a)],
                                              pts[static_cast<std::size_t>(t.b)],
                                              pts[static_cast<std::size_t>(t.c)]) > 0.0);
    used.insert(t.a);
    used.insert(t.b);
    used.insert(t.c);
  }
  ADSIM_CHECK_EQ(used.size(), pts.size());

  // 3) 每条（无向）边最多被两个三角形共享
  const std::map<std::pair<int, int>, int> use = edgeUsage(tris);
  for (const auto& kv : use) {
    ADSIM_CHECK_MSG(kv.second <= 2, "边被超过两个三角形共享");
    ADSIM_CHECK(kv.first.first < kv.first.second);
  }
  // edges() 必须与三角形导出的边集合完全一致（去重且 first < second）
  ADSIM_CHECK_EQ(dt.edges().size(), use.size());
  for (const std::pair<int, int>& e : dt.edges()) {
    ADSIM_CHECK(e.first < e.second);
    ADSIM_CHECK(use.count(e) == 1);
  }

  // 4) 空圆性质：既用接口自带的判定，也用外心/半径独立复算一遍
  ADSIM_CHECK(dt.satisfiesEmptyCircleProperty(1e-9));
  for (const Triangle& t : tris) {
    const Vec2 a = pts[static_cast<std::size_t>(t.a)];
    const Vec2 b = pts[static_cast<std::size_t>(t.b)];
    const Vec2 c = pts[static_cast<std::size_t>(t.c)];
    const double r = DelaunayTriangulation::circumradius(a, b, c);
    const Vec2 o = DelaunayTriangulation::circumcenter(a, b, c);
    ADSIM_CHECK_NEAR((a - o).norm(), r, 1e-9 * std::max(1.0, r));  // 三个顶点共圆
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (static_cast<int>(i) == t.a || static_cast<int>(i) == t.b ||
          static_cast<int>(i) == t.c) {
        continue;
      }
      ADSIM_CHECK_MSG((pts[i] - o).norm() >= r * (1.0 - 1e-9), "点落在三角形外接圆内");
    }
  }

  // 5) 点定位：凸包内的顶点一定被命中；三角形重心命中唯一；远处点不命中
  for (std::size_t i = 0; i < pts.size(); ++i) {
    bool found = false;
    for (int ti : dt.locate(pts[i])) {
      const Triangle& t = tris[static_cast<std::size_t>(ti)];
      if (t.a == static_cast<int>(i) || t.b == static_cast<int>(i) ||
          t.c == static_cast<int>(i)) {
        found = true;
        break;
      }
    }
    ADSIM_CHECK_MSG(found, "顶点未被 locate 命中");
  }
  std::size_t checked = 0;
  for (const Triangle& t : tris) {
    const Vec2 g = (pts[static_cast<std::size_t>(t.a)] + pts[static_cast<std::size_t>(t.b)] +
                    pts[static_cast<std::size_t>(t.c)]) /
                   3.0;
    ADSIM_CHECK_EQ(dt.locate(g).size(), std::size_t{1});
    ++checked;
    if (checked > 50) break;  // 抽样即可，避免 300 点全量定位拖慢测试
  }
  ADSIM_CHECK(dt.locate({1.0e4, 1.0e4}).empty());
}

// ===========================================================================
//  Delaunay：规则网格（大量点共圆，考验容差）
// ===========================================================================

ADSIM_TEST(Geometry, 网格剖分与共圆容差) {
  std::vector<Vec2> grid;
  for (int i = 0; i < 15; ++i) {
    for (int j = 0; j < 15; ++j) grid.push_back({i * 3.0, j * 3.0});
  }
  DelaunayTriangulation dt(grid);
  // 15x15 网格：边界上有 4*14 = 56 个点参与构成凸包边界，故 T = 2n-2-h
  const std::size_t n = grid.size();
  const std::size_t boundary_pts = 4 * 14;
  ADSIM_CHECK_EQ(dt.triangleCount(), 2 * n - 2 - boundary_pts);
  ADSIM_CHECK_EQ(dt.triangleCount(), std::size_t{392});
  ADSIM_CHECK_EQ(dt.edges().size(), 3 * n - 3 - boundary_pts);
  ADSIM_CHECK_EQ(dt.edges().size(), std::size_t{616});
  // 严格凸包只取 4 个角点
  ADSIM_CHECK_EQ(dt.convexHull().size(), std::size_t{4});
  // 网格上四点共圆极其普遍：容差不足会把共圆点误判为"圆内"
  ADSIM_CHECK(dt.satisfiesEmptyCircleProperty(1e-9));

  std::size_t boundary_edges = 0;
  for (const auto& kv : edgeUsage(dt.triangles())) {
    if (kv.second == 1) ++boundary_edges;
  }
  ADSIM_CHECK_EQ(boundary_edges, boundary_pts);

  // 密集网格（点距 0.01，共圆更密集）
  std::vector<Vec2> dense;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) dense.push_back({i * 0.01, j * 0.01});
  }
  DelaunayTriangulation dd(dense);
  ADSIM_CHECK(dd.triangleCount() > 0);
  ADSIM_CHECK(dd.satisfiesEmptyCircleProperty(1e-9));
  const std::map<std::pair<int, int>, int> duse = edgeUsage(dd.triangles());
  for (const auto& kv : duse) ADSIM_CHECK(kv.second <= 2);

  // 共圆点集：圆周 32 等分点
  DelaunayTriangulation dc(circlePoints(32, 8.0));
  ADSIM_CHECK_EQ(dc.triangleCount(), std::size_t{30});  // 2n-2-h = 64-2-32
  ADSIM_CHECK(dc.satisfiesEmptyCircleProperty(1e-9));
}

// ===========================================================================
//  Delaunay：凸包正确性
// ===========================================================================

ADSIM_TEST(Geometry, 凸包正确性) {
  std::vector<Vec2> pts = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0},  // 四角
                           {5.0, 5.0}, {3.0, 7.0}, {7.0, 3.0},                    // 内部点
                           {5.0, 0.0}, {10.0, 5.0}, {5.0, 10.0}, {0.0, 5.0}};     // 边上点
  DelaunayTriangulation dt(pts);
  const std::vector<int> hull = dt.convexHull();
  // 严格凸包排除边上共线点，只保留 4 个角
  ADSIM_CHECK_EQ(hull.size(), std::size_t{4});
  double area = 0.0;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Vec2& a = pts[static_cast<std::size_t>(hull[i])];
    const Vec2& b = pts[static_cast<std::size_t>(hull[(i + 1) % hull.size()])];
    area += a.cross(b) / 2.0;
  }
  ADSIM_CHECK_NEAR(area, 100.0, 1e-9);  // 逆时针 → 面积为正

  // 凸包上相邻三点必须左转（逆时针、无自交）
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Vec2& a = pts[static_cast<std::size_t>(hull[i])];
    const Vec2& b = pts[static_cast<std::size_t>(hull[(i + 1) % hull.size()])];
    const Vec2& c = pts[static_cast<std::size_t>(hull[(i + 2) % hull.size()])];
    ADSIM_CHECK(DelaunayTriangulation::cross2(a, b, c) > 0.0);
  }
  // 所有点都必须在凸包内或边界上
  for (const Vec2& p : pts) {
    for (std::size_t i = 0; i < hull.size(); ++i) {
      const Vec2& a = pts[static_cast<std::size_t>(hull[i])];
      const Vec2& b = pts[static_cast<std::size_t>(hull[(i + 1) % hull.size()])];
      ADSIM_CHECK_MSG(DelaunayTriangulation::cross2(a, b, p) >= -1e-9, "点跑到凸包外");
    }
  }
  // 顶点必须全部参与剖分（边上共线点也是顶点）
  DelaunayTriangulation dt2(pts);
  std::set<int> used;
  for (const Triangle& t : dt2.triangles()) {
    used.insert(t.a);
    used.insert(t.b);
    used.insert(t.c);
  }
  ADSIM_CHECK_EQ(used.size(), pts.size());
}

// ===========================================================================
//  Voronoi 路线图
// ===========================================================================

ADSIM_TEST(Geometry, Voronoi路线图穿缺口) {
  const std::vector<Vec2> wall = wallPoints(0.4, 1.5, 6.0, 0.4);
  VoronoiRoadmap rm(wall);

  // 节点是三角形外心：至少 3 个障碍点与之等距（外接圆半径）
  ADSIM_CHECK(rm.nodeCount() > 0);
  for (const Vec2& node : rm.nodes()) {
    double nearest = std::numeric_limits<double>::max();
    for (const Vec2& o : wall) nearest = std::min(nearest, (o - node).norm());
    int equidistant = 0;
    for (const Vec2& o : wall) {
      if (std::fabs((o - node).norm() - nearest) <= 1e-9 * std::max(1.0, nearest)) {
        ++equidistant;
      }
    }
    ADSIM_CHECK_MSG(equidistant >= 3, "外心节点到障碍点的距离不足 3 个等距");
    // clearanceAt 恰好等于到最近障碍点的距离
    ADSIM_CHECK_NEAR(rm.clearanceAt(node), nearest, 1e-9);
  }
  ADSIM_CHECK_EQ(rm.adjacency().size(), rm.nodeCount());

  // 搜索：路径必须从真实起点出发、到真实终点结束
  const Vec2 start{-6.0, 0.0};
  const Vec2 goal{6.0, 0.0};
  const std::vector<Vec2> path = rm.search(start, goal);
  ADSIM_CHECK(path.size() >= 2);
  ADSIM_CHECK_NEAR((path.front() - start).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((path.back() - goal).norm(), 0.0, 1e-12);

  // 路径必须从缺口穿过：缺口半宽 1.5，而墙面点距 0.4（点阵缝隙只有 0.2）
  const double cy = crossingY(path);
  ADSIM_CHECK_MSG(std::fabs(cy) <= 1.5, "路径没有从缺口穿过");
  ADSIM_CHECK(rm.minClearanceAlong(path) > 1.0);  // 远大于点阵缝隙 0.2
  // 不应绕远：直线距离 12m，绕墙端至少 12m 以上
  ADSIM_CHECK(independentLength(path) < 13.5);

  // 间隙场语义
  ADSIM_CHECK_NEAR(rm.clearanceAt({0.0, 2.8}), 0.0, 1e-12);        // 障碍点自身
  ADSIM_CHECK_NEAR(rm.clearanceAt({0.2, 0.0}), 1.6124515, 1e-4);   // 缺口中心
  // 远处等于到最近障碍点的真实距离（并非到墙心的距离：缺口两侧最近的点在 y=±1.6）
  double far_expect = std::numeric_limits<double>::max();
  for (const Vec2& o : wall) far_expect = std::min(far_expect, (o - Vec2{30.0, 0.0}).norm());
  ADSIM_CHECK_NEAR(rm.clearanceAt({30.0, 0.0}), far_expect, 1e-9);
  ADSIM_CHECK(rm.clearanceAt({30.0, 0.0}) > 29.0);
  // 障碍点集为空 / 单点：间隙无定义时返回极大值
  ADSIM_CHECK(VoronoiRoadmap({}).clearanceAt({1.0, 1.0}) ==
              std::numeric_limits<double>::max());
  VoronoiRoadmap one({{3.0, 4.0}});
  ADSIM_CHECK_NEAR(one.clearanceAt({0.0, 0.0}), 5.0, 1e-12);
  // 空路径按"无有效间隙"处理（fail-closed：拿不到路径就不应判为安全）
  ADSIM_CHECK_NEAR(rm.minClearanceAlong({}), 0.0, 1e-12);
  // 单点路径等于该点的间隙
  ADSIM_CHECK_NEAR(rm.minClearanceAlong({Vec2{0.2, 0.0}}), rm.clearanceAt({0.2, 0.0}), 1e-12);
  // 路径上取各点间隙的最小值（含障碍点自身 → 0）
  const std::vector<Vec2> probe = {{0.2, 0.0}, {0.0, 2.8}, {0.2, 3.2}};
  const double expect_min = std::min({rm.clearanceAt(probe[0]), rm.clearanceAt(probe[1]),
                                     rm.clearanceAt(probe[2])});
  ADSIM_CHECK_NEAR(rm.minClearanceAlong(probe), expect_min, 1e-12);
  ADSIM_CHECK(rm.minClearanceAlong(probe) < rm.clearanceAt(probe[0]));
}

ADSIM_TEST(Geometry, Voronoi退化输入) {
  // 障碍点少于 3 个：无三角形 → 无节点 → 搜索返回空
  const std::vector<std::vector<Vec2>> cases = {
      {},
      {{0.0, 0.0}},
      {{0.0, 0.0}, {1.0, 0.0}},
      {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},  // 共线
      {{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}},  // 全重合
  };
  for (const std::vector<Vec2>& obs : cases) {
    VoronoiRoadmap rm(obs);
    ADSIM_CHECK_EQ(rm.nodeCount(), std::size_t{0});
    ADSIM_CHECK(rm.search({-1.0, 0.0}, {1.0, 0.0}).empty());
    // 无障碍点集时间隙无定义：返回极大值而不是 NaN
    const double c = rm.clearanceAt({0.0, 0.0});
    ADSIM_CHECK(std::isfinite(c) || c == std::numeric_limits<double>::max());
  }

  // 单个三角形障碍（3 个不共线点）也能建出 1 个节点
  VoronoiRoadmap tri({{0.0, 0.0}, {4.0, 0.0}, {0.0, 4.0}});
  ADSIM_CHECK_EQ(tri.nodeCount(), std::size_t{1});
  const std::vector<Vec2> p = tri.search({-1.0, -1.0}, {2.0, 2.0});
  ADSIM_CHECK_EQ(p.size(), std::size_t{3});  // 起点 + 节点 + 终点
}

// ===========================================================================
//  B 样条：构造与求值
// ===========================================================================

ADSIM_TEST(Geometry, clamped样条端点插值) {
  const std::vector<Vec2> ctrl = {{0.0, 0.0}, {2.0, 4.0}, {6.0, -3.0},
                                  {9.0, 5.0}, {13.0, 1.0}};
  const CubicBSpline s(ctrl, true);
  ADSIM_CHECK(s.clamped());
  ADSIM_CHECK_EQ(s.controlPoints().size(), ctrl.size());
  ADSIM_CHECK_EQ(s.knots().size(), ctrl.size() + 4);  // n + p + 1

  // clamped 节点向量：曲线严格穿过首末控制点，且在参数域 [0,1] 上
  ADSIM_CHECK_NEAR((s.evaluate(0.0) - ctrl.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((s.evaluate(1.0) - ctrl.back()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(s.knots().front(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(s.knots().back(), 1.0, 1e-12);

  // 采样轨迹同样以控制点为端点，点数符合请求
  const Trajectory t = s.sample(7);
  ADSIM_CHECK_EQ(t.size(), std::size_t{7});
  ADSIM_CHECK_NEAR((t.front().position() - ctrl.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((t.back().position() - ctrl.back()).norm(), 0.0, 1e-12);
  const Trajectory ta = s.sampleByArcLength(11);
  ADSIM_CHECK_EQ(ta.size(), std::size_t{11});
  ADSIM_CHECK_NEAR((ta.front().position() - ctrl.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((ta.back().position() - ctrl.back()).norm(), 0.0, 1e-12);
  ADSIM_CHECK(s.sample(0).empty());
  ADSIM_CHECK(s.sampleByArcLength(0).empty());

  // 非 clamped：参数域是 [knots[3], knots[n]]，域外取值必须夹到端点而不是外推
  const CubicBSpline ns(ctrl, false);
  ADSIM_CHECK(!ns.clamped());
  const double u0 = ns.knots()[3];
  const double u1 = ns.knots()[ctrl.size()];
  ADSIM_CHECK(u1 > u0);
  ADSIM_CHECK_NEAR((ns.evaluate(0.0) - ns.evaluate(u0)).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((ns.evaluate(5.0) - ns.evaluate(u1)).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((ns.evaluate(-5.0) - ns.evaluate(u0)).norm(), 0.0, 1e-12);

  // 控制点少于 4 个无法定义三次节点向量：构造即抛异常
  ADSIM_CHECK_THROWS(CubicBSpline({{0.0, 0.0}, {1.0, 1.0}}, true), std::invalid_argument);
}

ADSIM_TEST(Geometry, 导数与差商一致) {
  const std::vector<Vec2> ctrl = {{0, 0}, {2, 5}, {7, -2}, {10, 6}, {14, 0}, {18, 4}, {22, -3}};
  const CubicBSpline s(ctrl, true);
  // 采样点避开节点（0.25/0.5/0.75）与端点，保证差商不含跨节点的折点
  const std::vector<double> us = {0.05, 0.2, 0.4, 0.55, 0.8, 0.95};
  const double h = 1e-6;
  for (double u : us) {
    const Vec2 fd1 = (s.evaluate(u + h) - s.evaluate(u - h)) / (2.0 * h);
    ADSIM_CHECK_NEAR((fd1 - s.derivative(u)).norm(), 0.0, 1e-7);
    const Vec2 fd2 = (s.derivative(u + h) - s.derivative(u - h)) / (2.0 * h);
    ADSIM_CHECK_NEAR((fd2 - s.secondDerivative(u)).norm(), 0.0, 1e-6);
    // evaluateDerivative(order) 必须与专用函数一致
    ADSIM_CHECK_NEAR((s.evaluateDerivative(u, 0) - s.evaluate(u)).norm(), 0.0, 1e-15);
    ADSIM_CHECK_NEAR((s.evaluateDerivative(u, 1) - s.derivative(u)).norm(), 0.0, 1e-15);
    ADSIM_CHECK_NEAR((s.evaluateDerivative(u, 2) - s.secondDerivative(u)).norm(), 0.0, 1e-15);
    // 负阶数按 0 阶处理
    ADSIM_CHECK_NEAR((s.evaluateDerivative(u, -3) - s.evaluate(u)).norm(), 0.0, 1e-15);
  }
  // 三次曲线三阶导只在每个节点区间内为常数（跨节点会跳变），
  // 因此必须在同一区间内取两点比较，不能跨节点比。
  const std::vector<double>& kn = s.knots();
  const double span_lo = kn[3];
  const double span_hi = kn[4];  // 第一段非零节点区间 [kn[3], kn[4]]
  const Vec2 d3a = s.evaluateDerivative(span_lo + 0.25 * (span_hi - span_lo), 3);
  const Vec2 d3b = s.evaluateDerivative(span_lo + 0.75 * (span_hi - span_lo), 3);
  ADSIM_CHECK_NEAR((d3a - d3b).norm(), 0.0, 1e-9 * std::max(1.0, d3a.norm()));
  // 三阶导还必须等于二阶导的差商（同一区间内，不跨节点）
  const double u3 = span_lo + 0.5 * (span_hi - span_lo);
  const Vec2 fd3 =
      (s.evaluateDerivative(u3 + h, 2) - s.evaluateDerivative(u3 - h, 2)) / (2.0 * h);
  ADSIM_CHECK_NEAR((fd3 - s.evaluateDerivative(u3, 3)).norm(), 0.0,
                   1e-5 * std::max(1.0, fd3.norm()));
  // 四阶及以上恒为 0
  ADSIM_CHECK_NEAR(s.evaluateDerivative(0.5, 4).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(s.evaluateDerivative(0.5, 9).norm(), 0.0, 1e-12);

  // clamped 端点导数有闭式解：P'(0) = p/(U_{p+1}-U_1) * (P1-P0)
  const Vec2 exact0 = (ctrl[1] - ctrl[0]) * (3.0 / (kn[4] - kn[3]));
  ADSIM_CHECK_NEAR((exact0 - s.derivative(0.0)).norm(), 0.0, 1e-10);
  // 右端同理：P'(1) = p/(U_n - U_{n-1}) * (P_{n-1}-P_{n-2})，U_n 是尾部首个 1.0
  const std::size_t nc = ctrl.size();
  const Vec2 exact1 = (ctrl[nc - 1] - ctrl[nc - 2]) * (3.0 / (kn[nc] - kn[nc - 1]));
  ADSIM_CHECK_NEAR((exact1 - s.derivative(1.0)).norm(), 0.0, 1e-10);

  // 所有控制点重合：曲线退化为一个点，导数必须为 0 而不是 NaN
  const CubicBSpline deg({{2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0}}, true);
  for (double u : {0.0, 0.3, 1.0}) {
    ADSIM_CHECK_NEAR((deg.evaluate(u) - Vec2{2.0, 2.0}).norm(), 0.0, 1e-12);
    ADSIM_CHECK_NEAR(deg.derivative(u).norm(), 0.0, 1e-12);
    ADSIM_CHECK_NEAR(deg.secondDerivative(u).norm(), 0.0, 1e-12);
    ADSIM_CHECK_NEAR(deg.curvature(u), 0.0, 1e-12);
  }
  ADSIM_CHECK_NEAR(deg.approximateLength(), 0.0, 1e-12);
}

ADSIM_TEST(Geometry, 曲率公式与圆弧) {
  // 直线样条：曲率恒为 0
  const CubicBSpline line({{0, 0}, {3, 0}, {6, 0}, {9, 0}, {12, 0}}, true);
  for (int i = 0; i <= 40; ++i) {
    ADSIM_CHECK_NEAR(line.curvature(static_cast<double>(i) / 40.0), 0.0, 1e-9);
  }

  // 圆弧上的控制点：内部曲率应接近 1/R（避开 clamped 端部畸变区）
  for (double radius : {5.0, 20.0}) {
    const CubicBSpline arc(circlePoints(37, radius), true);
    for (int i = 0; i <= 100; ++i) {
      const double u = 0.15 + 0.7 * static_cast<double>(i) / 100.0;
      const double k = arc.curvature(u);
      ADSIM_CHECK_MSG(k > 0.0, "圆弧样条曲率应为正");
      // 相对偏差：37 个控制点的三次 B 样条逼近整圆，实测偏差 < 1%
      ADSIM_CHECK_NEAR(k * radius, 1.0, 0.02);
    }
  }

  // 曲率实现必须与 derivative/secondDerivative 自洽（公式 |x'y''-y'x''|/(x'²+y'²)^1.5）
  const CubicBSpline s({{0, 0}, {2, 5}, {7, -2}, {10, 6}, {14, 0}, {18, 4}}, true);
  for (int i = 0; i <= 60; ++i) {
    const double u = static_cast<double>(i) / 60.0;
    const Vec2 d1 = s.derivative(u);
    const Vec2 d2 = s.secondDerivative(u);
    const double den = d1.squaredNorm();
    const double expect =
        den < 1e-12 ? 0.0 : std::fabs(d1.x * d2.y - d1.y * d2.x) / (den * std::sqrt(den));
    ADSIM_CHECK_NEAR(s.curvature(u), expect, 1e-12);
    ADSIM_CHECK(std::isfinite(s.curvature(u)));
  }
  // 与二阶差商独立对照（容差取差商自身的截断误差量级）
  const double h = 1e-5;
  for (int i = 1; i < 30; ++i) {
    const double u = static_cast<double>(i) / 30.0;
    const Vec2 d1 = (s.evaluate(u + h) - s.evaluate(u - h)) / (2.0 * h);
    const Vec2 d2 = (s.evaluate(u + h) - s.evaluate(u) * 2.0 + s.evaluate(u - h)) / (h * h);
    const double den = d1.squaredNorm();
    const double kfd = std::fabs(d1.x * d2.y - d1.y * d2.x) / (den * std::sqrt(den));
    ADSIM_CHECK_NEAR(s.curvature(u), kfd, 1e-3);
  }
}

ADSIM_TEST(Geometry, 弧长与等弧长采样) {
  const std::vector<Vec2> ctrl = {{0.0, 0.0}, {2.0, 4.0}, {6.0, -3.0}, {9.0, 5.0}, {13.0, 1.0}};
  const CubicBSpline s(ctrl, true);

  const double len = s.approximateLength(200);
  // 下界：首末直线长度；上界：控制多边形长度（B 样条含于控制多边形的凸包内）
  ADSIM_CHECK(len > (ctrl.back() - ctrl.front()).norm());
  ADSIM_CHECK(len < independentLength(ctrl) + 1e-9);
  // 与 4000 段密集采样的折线长度一致（折线内接于曲线，故略小）
  const double polyLen = independentLength(positions(s.sample(4000)));
  ADSIM_CHECK_NEAR(len, polyLen, 1e-6 * len);
  // Simpson 加密后应稳定（相对变化 < 1e-6）
  const double len2 = s.approximateLength(4000);
  ADSIM_CHECK_NEAR(len2, len, 1e-6 * len);
  ADSIM_CHECK_NEAR(s.approximateLength(1), s.approximateLength(2), 1e-9);
  ADSIM_CHECK(s.approximateLength() > 0.0);

  // 等弧长采样：相邻弦长应基本相等（末段可能不足一个步长）
  const Trajectory t = s.sampleByArcLength(50);
  ADSIM_CHECK_EQ(t.size(), std::size_t{50});
  double gmin = 1e18;
  double gmax = 0.0;
  for (std::size_t i = 1; i < t.size(); ++i) {
    const double d = std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
    gmin = std::min(gmin, d);
    gmax = std::max(gmax, d);
  }
  ADSIM_CHECK_MSG(gmax / gmin < 1.01, "等弧长采样的间距不均匀");
  // 采样折线的弧长必须与 Simpson 积分一致
  ADSIM_CHECK_NEAR(independentLength(positions(t)), len, 1e-3 * len);

  // 航向来自切向量方向（首末采样点分别对应 u0/u1 处的切向）
  const Trajectory t2 = s.sample(2);
  const Vec2 d0 = s.derivative(0.0);
  const Vec2 d1 = s.derivative(1.0);
  ADSIM_CHECK_NEAR(t2[0].theta, std::atan2(d0.y, d0.x), 1e-12);
  ADSIM_CHECK_NEAR(t2[1].theta, std::atan2(d1.y, d1.x), 1e-12);
  // 单点采样：落在参数域起点，航向同样取该处切向
  const Trajectory t1 = s.sample(1);
  ADSIM_CHECK_EQ(t1.size(), std::size_t{1});
  ADSIM_CHECK_NEAR((Vec2{t1[0].x, t1[0].y} - s.evaluate(0.0)).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(t1[0].theta, std::atan2(d0.y, d0.x), 1e-12);
  // 切向退化（所有控制点重合）时航向回退为 0，而不是产生 NaN 方向
  const CubicBSpline flat({{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}}, true);
  const Trajectory tf = flat.sample(2);
  ADSIM_CHECK_NEAR(tf[0].theta, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(tf[1].theta, tf[0].theta, 1e-12);
}

// ===========================================================================
//  B 样条：最小二乘拟合
// ===========================================================================

ADSIM_TEST(Geometry, 最小二乘拟合精度) {
  // 半径 20 的 80 度圆弧，40 个路点
  std::vector<Vec2> arc;
  for (int i = 0; i <= 40; ++i) {
    const double a = deg2rad(-40.0) + deg2rad(80.0) * static_cast<double>(i) / 40.0;
    arc.push_back({20.0 * std::cos(a), 20.0 * std::sin(a)});
  }

  // 控制点越多拟合越贴合：rms 单调下降
  const BSplineFitResult r4 = fitCubicBSpline(arc, 4, 3);
  const BSplineFitResult r6 = fitCubicBSpline(arc, 6, 3);
  const BSplineFitResult r12 = fitCubicBSpline(arc, 12, 3);
  const BSplineFitResult r20 = fitCubicBSpline(arc, 20, 3);
  ADSIM_CHECK_EQ(r4.control_points.size(), std::size_t{4});
  ADSIM_CHECK_EQ(r6.control_points.size(), std::size_t{6});
  ADSIM_CHECK_EQ(r12.control_points.size(), std::size_t{12});
  ADSIM_CHECK_EQ(r20.control_points.size(), std::size_t{20});
  ADSIM_CHECK_MSG(r6.rms_error < r4.rms_error, "增加控制点后拟合误差未下降");
  ADSIM_CHECK_MSG(r12.rms_error < r6.rms_error, "增加控制点后拟合误差未下降");
  ADSIM_CHECK_MSG(r20.rms_error < r12.rms_error, "增加控制点后拟合误差未下降");
  ADSIM_CHECK_LT(r4.rms_error, 5e-2);
  ADSIM_CHECK_LT(r20.rms_error, 1e-5);
  // max_error 不小于 rms，且迭代轮数不超过请求值
  for (const BSplineFitResult* r : {&r4, &r6, &r12, &r20}) {
    ADSIM_CHECK(r->max_error >= r->rms_error);
    ADSIM_CHECK(r->iterations >= 1 && r->iterations <= 3);
  }
  // 拟合曲线严格穿过首末路点
  ADSIM_CHECK_NEAR((r20.control_points.front() - arc.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((r20.control_points.back() - arc.back()).norm(), 0.0, 1e-12);

  // 2 个路点退化：控制点构成直线段，误差为 0
  const BSplineFitResult r2 = fitCubicBSpline({{0, 0}, {3, 4}}, 4, 3);
  ADSIM_CHECK_EQ(r2.control_points.size(), std::size_t{4});
  ADSIM_CHECK_NEAR(r2.rms_error, 0.0, 1e-9);
  ADSIM_CHECK_NEAR(r2.max_error, 0.0, 1e-9);
  // 控制点数不能超过路点数（否则法方程奇异）
  const BSplineFitResult rCap = fitCubicBSpline({{0, 0}, {1, 1}, {2, 0}, {3, 1}}, 10, 3);
  ADSIM_CHECK_EQ(rCap.control_points.size(), std::size_t{4});

  // 病态判据必须与坐标系原点无关：UTM 量级坐标（x≈5e5）下不得被误判而降阶，
  // 且拟合结果应满足平移不变性
  std::vector<Vec2> far_arc;
  for (int i = 0; i <= 40; ++i) {
    const double a = deg2rad(-40.0) + deg2rad(80.0) * static_cast<double>(i) / 40.0;
    far_arc.push_back({5.0e5 + 20.0 * std::cos(a), 5.0e5 + 20.0 * std::sin(a)});
  }
  const BSplineFitResult rFar = fitCubicBSpline(far_arc, 12, 3);
  ADSIM_CHECK_EQ(rFar.control_points.size(), std::size_t{12});
  ADSIM_CHECK_NEAR(rFar.rms_error, r12.rms_error, 1e-9);

  // 病态回归：控制点数接近路点数时，法方程条件数骤降，曾经解出 1e6 量级的
  // 控制点（曲线飞到几百公里外，且 rms 反而更大）。这里直接检查拟合曲线的
  // 活动范围——不论实现如何降阶，曲线都不允许飞出数据范围太远。
  const std::vector<Vec2> folded = foldedPath();
  Vec2 lo = folded.front();
  Vec2 hi = folded.front();
  for (const Vec2& p : folded) {
    lo.x = std::min(lo.x, p.x);
    lo.y = std::min(lo.y, p.y);
    hi.x = std::max(hi.x, p.x);
    hi.y = std::max(hi.y, p.y);
  }
  const Vec2 center = (lo + hi) * 0.5;
  const double radius = std::max(hi.x - lo.x, hi.y - lo.y) * 0.5 + 1.0;
  for (std::size_t cc = folded.size() / 2; cc <= folded.size(); ++cc) {
    const BSplineFitResult rf = fitCubicBSpline(folded, cc, 3);
    ADSIM_CHECK(rf.control_points.size() >= 4);
    ADSIM_CHECK_MSG(std::isfinite(rf.rms_error), "拟合误差非有限");
    const CubicBSpline sp(rf.control_points, true);
    double reach = 0.0;
    for (int i = 0; i <= 200; ++i) {
      const Vec2 p = sp.evaluate(static_cast<double>(i) / 200.0);
      reach = std::max(reach, (p - center).norm());
    }
    ADSIM_CHECK_MSG(reach < 3.0 * radius, "拟合曲线飞出数据范围（法方程病态未处理）");
  }
  // 少于 2 个路点：返回空结果
  ADSIM_CHECK(fitCubicBSpline({}, 4, 3).control_points.empty());
  ADSIM_CHECK(fitCubicBSpline({{1.0, 1.0}}, 4, 3).control_points.empty());
  // 重复点/回折点不产生 NaN
  std::vector<Vec2> dirty;
  for (int i = 0; i <= 20; ++i) dirty.push_back({i * 0.5, 0.0});
  dirty.insert(dirty.end(), dirty.begin(), dirty.end());
  const BSplineFitResult rDirty = fitCubicBSpline(dirty, 8, 3);
  ADSIM_CHECK_EQ(rDirty.control_points.size(), std::size_t{8});
  ADSIM_CHECK(std::isfinite(rDirty.rms_error) && rDirty.rms_error < 1e-6);
  std::vector<Vec2> same(30, Vec2{3.0, 4.0});
  const BSplineFitResult rSame = fitCubicBSpline(same, 6, 2);
  ADSIM_CHECK(std::isfinite(rSame.rms_error));
  // iterations = 0 也应至少跑一轮
  const BSplineFitResult rZero = fitCubicBSpline(arc, 8, 0);
  ADSIM_CHECK_EQ(rZero.control_points.size(), std::size_t{8});
  ADSIM_CHECK(rZero.iterations >= 1);
}

// ===========================================================================
//  B 样条：曲率约束平滑（核心验收项）
// ===========================================================================

ADSIM_TEST(Geometry, 曲率约束平滑生效) {
  // 直角路径 + 轻微横向噪声（有噪声才有曲率峰值，才需要平滑）
  std::mt19937 rng(11u);
  std::normal_distribution<double> noise(0.0, 0.08);
  std::vector<Vec2> path;
  for (int i = 0; i <= 10; ++i) path.push_back({i * 2.0, noise(rng)});
  for (int i = 1; i <= 10; ++i) path.push_back({20.0 + noise(rng), i * 2.0});
  const double raw_len = independentLength(path);
  const double raw_kappa = maxCurvature(pathToTrajectory(path, 1.0));
  ADSIM_CHECK_MSG(raw_kappa > 0.5, "测试路径本身应当有较大曲率");

  for (double limit : {2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.02}) {
    const Trajectory t = smoothPathWithCurvatureLimit(path, limit, 200);
    ADSIM_CHECK(!t.empty());
    // 核心约定：输出的最大曲率不得超过约束（实现里只有 k <= max_kappa 才会返回）
    ADSIM_CHECK_MSG(maxCurvature(t) <= limit, "平滑后最大曲率仍超过约束");
    // 端点必须严格保持，否则轨迹会与真实起终点脱节
    ADSIM_CHECK_NEAR((t.front().position() - path.front()).norm(), 0.0, 1e-9);
    ADSIM_CHECK_NEAR((t.back().position() - path.back()).norm(), 0.0, 1e-9);
    for (const TrajectoryPoint& tp : t) {
      ADSIM_CHECK(std::isfinite(tp.x) && std::isfinite(tp.y));
      ADSIM_CHECK(std::isfinite(tp.kappa) && std::isfinite(tp.theta));
    }
    // 平滑后的路径不应超过原路径的 1.1 倍长度（否则是发散而不是平滑）
    ADSIM_CHECK(independentLength(positions(t)) < raw_len * 1.1);
  }

  // 回折路径（158° 折角）：拟合一旦病态发散，这里会返回一条"曲率很小"却在
  // 几百公里外的轨迹——曲率检查抓不到，必须同时约束它贴近输入路径。
  const std::vector<Vec2> folded = foldedPath();
  const double folded_len = independentLength(folded);
  const double folded_kappa = maxCurvature(pathToTrajectory(folded, 1.0));
  const Trajectory tf = smoothPathWithCurvatureLimit(folded, 0.05, 60);
  ADSIM_CHECK(!tf.empty());
  ADSIM_CHECK_MSG(maxCurvature(tf) <= folded_kappa + 1e-9, "平滑后曲率超过输入");
  double folded_dev = 0.0;
  for (const TrajectoryPoint& tp : tf) {
    folded_dev = std::max(folded_dev, distanceToPolyline(folded, tp.position(), nullptr));
    ADSIM_CHECK(std::isfinite(tp.x) && std::isfinite(tp.y) && std::isfinite(tp.kappa));
  }
  ADSIM_CHECK_MSG(folded_dev < 0.5 * folded_len, "平滑轨迹偏离输入路径过远（发散）");

  // 直线输入：曲率应保持为 0，且形状不变
  std::vector<Vec2> line;
  for (int i = 0; i <= 30; ++i) line.push_back({i * 1.0, 0.0});
  const Trajectory tl = smoothPathWithCurvatureLimit(line, 0.1, 50);
  ADSIM_CHECK_EQ(tl.size(), std::size_t{50});
  ADSIM_CHECK_LT(maxCurvature(tl), 1e-9);
  ADSIM_CHECK_NEAR((tl.back().position() - line.back()).norm(), 0.0, 1e-9);

  // 输入不足 2 点：返回空
  ADSIM_CHECK(smoothPathWithCurvatureLimit({}, 0.2, 100).empty());
  ADSIM_CHECK(smoothPathWithCurvatureLimit({{1.0, 1.0}}, 0.2, 100).empty());
  // 全部重合：不得崩溃，输出要么为空要么是合法轨迹
  const Trajectory ts = smoothPathWithCurvatureLimit({{0, 0}, {0, 0}, {0, 0}}, 0.2, 100);
  for (const TrajectoryPoint& tp : ts) ADSIM_CHECK(std::isfinite(tp.x) && std::isfinite(tp.y));
  // sample_count < 2 也要给出合法结果
  const Trajectory t1 = smoothPathWithCurvatureLimit(line, 0.2, 1);
  ADSIM_CHECK(t1.size() >= 1);
}

// ===========================================================================
//  轨迹生成 / 曲率统计
// ===========================================================================

ADSIM_TEST(Geometry, 轨迹生成与曲率统计) {
  // 圆周折线：离散 Menger 曲率应当精确等于 1/R
  std::vector<Vec2> circle;
  for (int i = 0; i <= 72; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) / 72.0;
    circle.push_back({8.0 * std::cos(a), 8.0 * std::sin(a)});
  }
  const Trajectory tr = pathToTrajectory(circle, 5.0);
  ADSIM_CHECK_EQ(tr.size(), circle.size());
  for (const TrajectoryPoint& tp : tr) {
    ADSIM_CHECK_NEAR(tp.kappa, 0.125, 1e-9);
    ADSIM_CHECK_NEAR(tp.v, 5.0, 1e-12);
  }
  ADSIM_CHECK_NEAR(maxCurvature(tr), 0.125, 1e-9);
  // 时间戳 = 累计弦长 / 标称速度
  ADSIM_CHECK_NEAR(tr.back().t, independentLength(circle) / 5.0, 1e-9);
  ADSIM_CHECK_NEAR(tr.front().t, 0.0, 1e-12);
  // 航向与首段方向一致
  ADSIM_CHECK_NEAR(tr.front().theta, std::atan2(circle[1].y - circle[0].y, circle[1].x - circle[0].x),
                   1e-12);

  // 三点圆弧：中间点的曲率等于 1/R（闭式解）
  const double R = 12.0;
  const std::vector<Vec2> arc3 = {{R, 0.0},
                                  {R * std::cos(0.1), R * std::sin(0.1)},
                                  {R * std::cos(0.2), R * std::sin(0.2)}};
  const Trajectory t3 = pathToTrajectory(arc3, 3.0);
  ADSIM_CHECK_NEAR(t3[1].kappa, 1.0 / R, 1e-9);
  ADSIM_CHECK_NEAR(t3[0].kappa, t3[1].kappa, 1e-12);  // 端点沿用相邻值
  ADSIM_CHECK_NEAR(t3[2].kappa, t3[1].kappa, 1e-12);

  // 左转/右转符号：曲率有向
  const Trajectory left = pathToTrajectory({{0, 0}, {1, 1}, {0, 2}}, 0.0);
  const Trajectory right = pathToTrajectory({{0, 0}, {1, -1}, {0, -2}}, 0.0);
  ADSIM_CHECK(left[1].kappa > 0.0);
  ADSIM_CHECK(right[1].kappa < 0.0);
  // 速度为 0 时时间戳保持 0（避免除零）
  const Trajectory still = pathToTrajectory({{0, 0}, {10, 0}}, 0.0);
  ADSIM_CHECK_NEAR(still.back().t, 0.0, 1e-12);

  // 曲率变化率：直角折线在拐点处有跳变
  std::vector<Vec2> corner;
  for (int i = 0; i <= 10; ++i) corner.push_back({i * 1.0, 0.0});
  for (int i = 1; i <= 10; ++i) corner.push_back({10.0, i * 1.0});
  const Trajectory tc = pathToTrajectory(corner, 2.0);
  ADSIM_CHECK_GT(maxCurvature(tc), 0.5);
  // 圆周上曲率恒定 → 变化率为 0

  // 退化输入
  ADSIM_CHECK(pathToTrajectory({}, 1.0).empty());
  const Trajectory one = pathToTrajectory({{1.0, 2.0}}, 1.0);
  ADSIM_CHECK_EQ(one.size(), std::size_t{1});
  ADSIM_CHECK_NEAR(one[0].kappa, 0.0, 1e-12);
  ADSIM_CHECK_LT(maxCurvature({}), 1e-12);
}

// ===========================================================================
//  CurveFit：多项式拟合与求值
// ===========================================================================

// ===========================================================================
//  CurveFit：RDP 抽稀
// ===========================================================================

ADSIM_TEST(Geometry, RDP抽稀保形) {
  // 近直线折线：小容差下应压成 2 个端点
  std::mt19937 rng(9u);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Vec2> noisy;
  for (int i = 0; i <= 100; ++i) noisy.push_back({i * 0.5, noise(rng)});
  const std::vector<Vec2> simp = rdpSimplify(noisy, 0.05);
  ADSIM_CHECK_EQ(simp.size(), std::size_t{2});
  ADSIM_CHECK_NEAR((simp.front() - noisy.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((simp.back() - noisy.back()).norm(), 0.0, 1e-12);
  // 容差过小时保留全部点（说明容差确实起作用）
  ADSIM_CHECK_EQ(rdpSimplify(noisy, 1e-4).size(), noisy.size());

  // 直角折线：拐点必须保留
  std::vector<Vec2> lshape;
  for (int i = 0; i <= 10; ++i) lshape.push_back({i * 1.0, 0.0});
  for (int i = 1; i <= 10; ++i) lshape.push_back({10.0, i * 1.0});
  const std::vector<Vec2> rl = rdpSimplify(lshape, 0.01);
  ADSIM_CHECK_EQ(rl.size(), std::size_t{3});
  ADSIM_CHECK_NEAR((rl[1] - Vec2{10.0, 0.0}).norm(), 0.0, 1e-12);

  // 抽稀结果必须是原序列的子序列，且原折线上的每点都在容差范围内
  std::vector<Vec2> zig;
  for (int i = 0; i <= 40; ++i) {
    zig.push_back({i * 1.0, (i % 4 == 0) ? 0.3 : ((i % 2 == 0) ? -0.2 : 0.1)});
  }
  const double eps = 0.5;
  const std::vector<Vec2> rz = rdpSimplify(zig, eps);
  ADSIM_CHECK(rz.size() >= 2 && rz.size() <= zig.size());
  ADSIM_CHECK_NEAR((rz.front() - zig.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((rz.back() - zig.back()).norm(), 0.0, 1e-12);
  std::size_t k = 0;
  for (const Vec2& p : rz) {  // 子序列检查
    while (k < zig.size() && (zig[k] - p).norm() > 1e-12) ++k;
    ADSIM_CHECK_MSG(k < zig.size(), "抽稀结果不是原序列的子序列");
    ++k;
  }
  for (const Vec2& p : zig) {
    ADSIM_CHECK_MSG(distanceToPolyline(rz, p) <= eps + 1e-9, "抽稀后偏差超过容差");
  }

  // 退化输入
  ADSIM_CHECK(rdpSimplify({}, 1.0).empty());
  ADSIM_CHECK_EQ(rdpSimplify({{1.0, 1.0}}, 1.0).size(), std::size_t{1});
  ADSIM_CHECK_EQ(rdpSimplify({{0.0, 0.0}, {1.0, 1.0}}, 1.0).size(), std::size_t{2});
  const std::vector<Vec2> same = rdpSimplify({{2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0}}, 0.5);
  for (const Vec2& p : same) ADSIM_CHECK_NEAR((p - Vec2{2.0, 2.0}).norm(), 0.0, 1e-12);
}

// ===========================================================================
//  CurveFit：滑动平均
// ===========================================================================

// ===========================================================================
//  CurveFit：弧长、重采样
// ===========================================================================

ADSIM_TEST(Geometry, 弧长与重采样) {
  const std::vector<Vec2> poly = {{0.0, 0.0}, {3.0, 4.0}, {3.0, 8.0}};
  const std::vector<double> arcs = accumulateArcLength(poly);
  ADSIM_CHECK_EQ(arcs.size(), poly.size());
  ADSIM_CHECK_NEAR(arcs[0], 0.0, 1e-12);
  ADSIM_CHECK_NEAR(arcs[1], 5.0, 1e-12);
  ADSIM_CHECK_NEAR(arcs[2], 9.0, 1e-12);
  ADSIM_CHECK_NEAR(polylineLength(poly), 9.0, 1e-12);
  ADSIM_CHECK_NEAR(polylineLength(poly), arcs.back(), 1e-12);
  for (std::size_t i = 1; i < arcs.size(); ++i) ADSIM_CHECK(arcs[i] >= arcs[i - 1]);
  ADSIM_CHECK_NEAR(polylineLength({}), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(polylineLength({{1.0, 1.0}}), 0.0, 1e-12);
  // 重复点不产生 NaN，弧长单调不减
  const std::vector<double> dup_arc = accumulateArcLength({{0, 0}, {0, 0}, {3, 4}, {3, 4}});
  ADSIM_CHECK_NEAR(dup_arc[1], 0.0, 1e-12);
  ADSIM_CHECK_NEAR(dup_arc[3], 5.0, 1e-12);

  // 重采样：每点落在原折线的整数倍弧长处（用被测的距离函数反查弧长位置），
  // 末点例外——不足一个步长时按实际总长收尾，必须恰好落在终点上
  const std::vector<Vec2> re = resampleByArcLength(poly, 2.0);
  ADSIM_CHECK_EQ(re.size(), std::size_t{6});
  const double total_len = polylineLength(poly);
  for (std::size_t i = 0; i < re.size(); ++i) {
    double arc = -1.0;
    const double d = distanceToPolyline(poly, re[i], &arc);
    ADSIM_CHECK_NEAR(d, 0.0, 1e-12);  // 重采样点必须落在原折线上
    ADSIM_CHECK_NEAR(arc, std::min(2.0 * static_cast<double>(i), total_len), 1e-9);
  }
  ADSIM_CHECK_NEAR((re.back() - poly.back()).norm(), 0.0, 1e-12);

  // 平滑折线（密集圆弧）上，除末段外的相邻间距应≈步长
  std::vector<Vec2> arcpts;
  for (int i = 0; i <= 200; ++i) {
    const double a = kPi * static_cast<double>(i) / 200.0;
    arcpts.push_back({10.0 * std::cos(a), 10.0 * std::sin(a)});
  }
  const double step = 0.5;
  const std::vector<Vec2> r2 = resampleByArcLength(arcpts, step);
  const double total = polylineLength(arcpts);
  ADSIM_CHECK_EQ(r2.size(), static_cast<std::size_t>(total / step) + 2);
  for (std::size_t i = 1; i + 1 < r2.size(); ++i) {
    ADSIM_CHECK_NEAR((r2[i] - r2[i - 1]).norm(), step, 0.01 * step);
  }
  ADSIM_CHECK((r2.back() - arcpts.back()).norm() < 1e-9);
  // 步长大于总长：只剩首末点
  const std::vector<Vec2> big = resampleByArcLength(poly, 100.0);
  ADSIM_CHECK_NEAR((big.front() - poly.front()).norm(), 0.0, 1e-12);
  ADSIM_CHECK_NEAR((big.back() - poly.back()).norm(), 0.0, 1e-12);

  // 退化输入
  ADSIM_CHECK(resampleByArcLength({}, 1.0).empty());
  ADSIM_CHECK_EQ(resampleByArcLength({{5.0, 5.0}}, 1.0).size(), std::size_t{1});
  ADSIM_CHECK_EQ(resampleByArcLength(poly, 0.0).size(), poly.size());
  ADSIM_CHECK_EQ(resampleByArcLength(poly, -1.0).size(), poly.size());
  const std::vector<Vec2> same = resampleByArcLength({{2.0, 2.0}, {2.0, 2.0}, {2.0, 2.0}}, 1.0);
  ADSIM_CHECK_EQ(same.size(), std::size_t{1});  // 零长度折线退化为单点
  ADSIM_CHECK_NEAR((same[0] - Vec2{2.0, 2.0}).norm(), 0.0, 1e-12);
}

// ===========================================================================
//  CurveFit：点到折线距离
// ===========================================================================

ADSIM_TEST(Geometry, 点到折线距离) {
  const std::vector<Vec2> poly = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  double arc = -1.0;
  // 垂足在段内
  ADSIM_CHECK_NEAR(distanceToPolyline(poly, {5.0, 3.0}, &arc), 3.0, 1e-12);
  ADSIM_CHECK_NEAR(arc, 5.0, 1e-12);
  // 命中顶点
  ADSIM_CHECK_NEAR(distanceToPolyline(poly, {10.0, 0.0}, &arc), 0.0, 1e-12);
  ADSIM_CHECK_NEAR(arc, 10.0, 1e-12);
  // 垂足落在段外：取到端点的距离，弧长取该端点
  ADSIM_CHECK_NEAR(distanceToPolyline(poly, {-4.0, 0.0}, &arc), 4.0, 1e-12);
  ADSIM_CHECK_NEAR(arc, 0.0, 1e-12);
  ADSIM_CHECK_NEAR(distanceToPolyline(poly, {14.0, 12.0}, &arc), std::sqrt(16.0 + 4.0), 1e-12);
  ADSIM_CHECK_NEAR(arc, 20.0, 1e-12);
  // 不传输出参数也不能崩
  ADSIM_CHECK_NEAR(distanceToPolyline(poly, {5.0, -2.0}), 2.0, 1e-12);
  // 退化输入
  ADSIM_CHECK(distanceToPolyline({}, {1.0, 1.0}) == std::numeric_limits<double>::max());
  ADSIM_CHECK_NEAR(distanceToPolyline({{2.0, 2.0}}, {0.0, 0.0}), std::sqrt(8.0), 1e-12);
  // 零长度折线（重复点）不产生 NaN
  const double d0 = distanceToPolyline({{3.0, 3.0}, {3.0, 3.0}}, {0.0, 0.0});
  ADSIM_CHECK(std::isfinite(d0));
  ADSIM_CHECK_NEAR(d0, std::sqrt(18.0), 1e-12);
}

// ===========================================================================
//  CurveFit：自交检测
// ===========================================================================

