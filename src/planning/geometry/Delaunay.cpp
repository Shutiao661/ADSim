// =============================================================================
//  Delaunay.cpp — Delaunay 三角剖分与 Voronoi 路线图实现
//
//  增量插入（Bowyer-Watson）的三步：
//    1) 以覆盖全部点的"超级三角形"初始化，使凸包边界无需特判；
//    2) 逐点插入：外接圆包含新点的三角形全部失效，其边界围成的空腔与新点
//       重新三角化（空腔相对新点是星形的，故直接连边即得合法三角形）；
//    3) 删除所有与超级三角形顶点相连的三角形，得到输入点集的剖分。
//
//  纯浮点实现在"恰好共圆 / 恰好共线"的退化输入（规则网格、共线点集、
//  重复点）下会留下不满足空圆性质的三角形，因此末尾追加两趟清理：
//  * 边翻转（局部 Delaunay 修复）——每个内部边都满足局部空圆条件时，
//    整个三角剖分即满足全局 Delaunay 性质；
//  * 退化（零面积）三角形剔除——其外接圆半径趋于无穷，既无意义又会污染
//    空圆校验与 Voronoi 节点提取。
// =============================================================================
#include "adsim/planning/geometry/Delaunay.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace adsim {
namespace geometry {

namespace {

/// InCircle 判定的相对容差。把"数值上共圆"的点视作落在圆内，可避免在
/// 退化输入下得到非星形空腔；该值远大于 double 舍入误差（~1e-16 相对），
/// 又远小于空圆校验的默认容差 1e-9，因此不会引入可观测的误差。
constexpr double kInCircleEps = 1e-13;

/// 三角形退化判定的相对阈值（面积相对于边长平方）
constexpr double kDegenerateEps = 1e-12;

/// 点定位的容差（叉积相对于边长平方），用于把落在边上的点判为"包含"
constexpr double kLocateEps = 1e-9;

/// 共线三角形外接圆半径的标称"无穷大"值
constexpr double kHugeRadius = 1e300;

/// 边翻转的迭代上限，避免退化输入下无限循环
constexpr int kMaxFlipPasses = 64;

bool isFinite(const Vec2& p) {
  return std::isfinite(p.x) && std::isfinite(p.y);
}

/// InCircle 行列式及其量级。
/// 数学上 det = 2 * Area(abc) * (R^2 - |d - O|^2)，要求 abc 为逆时针：
/// det > 0 表示 d 严格落在外接圆内，det == 0 表示四点共圆。
/// 量纲为长度^4，scale 为各项绝对值之和，供相对容差比较使用。
struct Incircle {
  double value{0.0};
  double scale{0.0};
};

Incircle incircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
  const double adx = a.x - d.x, ady = a.y - d.y;
  const double bdx = b.x - d.x, bdy = b.y - d.y;
  const double cdx = c.x - d.x, cdy = c.y - d.y;

  const double abdet = adx * bdy - bdx * ady;
  const double bcdet = bdx * cdy - cdx * bdy;
  const double cadet = cdx * ady - adx * cdy;
  const double alift = adx * adx + ady * ady;
  const double blift = bdx * bdx + bdy * bdy;
  const double clift = cdx * cdx + cdy * cdy;

  Incircle out;
  out.value = alift * bcdet + blift * cadet + clift * abdet;
  out.scale = std::fabs(alift * bcdet) + std::fabs(blift * cadet) +
              std::fabs(clift * abdet);
  return out;
}

/// d 是否落在外接圆内（含"数值共圆"）；要求 abc 为逆时针
bool insideCircumcircle(const Vec2& a, const Vec2& b, const Vec2& c,
                        const Vec2& d) {
  const Incircle r = incircle(a, b, c, d);
  return r.value > -kInCircleEps * r.scale;
}

/// 三角形 t 中除 u、v 之外的第三个顶点
int oppositeVertex(const Triangle& t, int u, int v) {
  if (t.a != u && t.a != v) return t.a;
  if (t.b != u && t.b != v) return t.b;
  return t.c;
}

/// 一趟边翻转：对所有内部边检查局部空圆条件，不满足则翻转对角线。
/// 每趟只在"未被本趟改动过的三角形"上操作（dirty 标记），翻转产生的新
/// 三角形记入 new_tris，用于判重。
/// @return 本趟是否发生了翻转
bool flipIllegalEdgesOnce(std::vector<Triangle>& tris,
                          const std::vector<Vec2>& pts, double area_eps) {
  struct EdgeRef {
    int u;
    int v;
    int tri;
  };

  std::vector<EdgeRef> refs;
  refs.reserve(tris.size() * 3);
  for (std::size_t i = 0; i < tris.size(); ++i) {
    const Triangle& t = tris[i];
    if (!t.valid()) continue;
    const int idx = static_cast<int>(i);
    const int v[3] = {t.a, t.b, t.c};
    for (int k = 0; k < 3; ++k) {
      const int p = v[k];
      const int q = v[(k + 1) % 3];
      refs.push_back(EdgeRef{std::min(p, q), std::max(p, q), idx});
    }
  }
  if (refs.empty()) return false;

  std::sort(refs.begin(), refs.end(), [](const EdgeRef& l, const EdgeRef& r) {
    return l.u != r.u ? l.u < r.u : (l.v != r.v ? l.v < r.v : l.tri < r.tri);
  });

  std::vector<Triangle> new_tris;
  bool flipped = false;

  for (std::size_t i = 0; i < refs.size();) {
    std::size_t j = i;
    while (j < refs.size() && refs[j].u == refs[i].u && refs[j].v == refs[i].v) {
      ++j;
    }
    const std::size_t shared = j - i;
    if (shared != 2) {  // 凸包边或非流形边：不处理
      i = j;
      continue;
    }

    const int u = refs[i].u;
    const int v = refs[i].v;
    Triangle& t1 = tris[static_cast<std::size_t>(refs[i].tri)];
    Triangle& t2 = tris[static_cast<std::size_t>(refs[i + 1].tri)];
    if (!t1.valid() || !t2.valid()) {  // 本趟已被翻转掉
      i = j;
      continue;
    }

    const int c = oppositeVertex(t1, u, v);
    const int d = oppositeVertex(t2, u, v);
    if (c < 0 || d < 0 || c == d) {
      i = j;
      continue;
    }

    const Vec2& pu = pts[static_cast<std::size_t>(u)];
    const Vec2& pv = pts[static_cast<std::size_t>(v)];
    const Vec2& pc = pts[static_cast<std::size_t>(c)];
    const Vec2& pd = pts[static_cast<std::size_t>(d)];

    // 取 {u,v,c} 的逆时针排列，否则 InCircle 行列式符号相反
    const bool ccw_uvc = DelaunayTriangulation::cross2(pu, pv, pc) > 0.0;
    const bool need_flip = ccw_uvc ? insideCircumcircle(pu, pv, pc, pd)
                                   : insideCircumcircle(pv, pu, pc, pd);
    if (!need_flip) {
      i = j;
      continue;
    }

    // 翻转后四边形必须严格凸，否则会生成重叠（非流形）三角形
    const Triangle na = ccw_uvc ? Triangle{u, d, c} : Triangle{v, d, c};
    const Triangle nb = ccw_uvc ? Triangle{d, v, c} : Triangle{d, u, c};
    const Vec2& pa1 = pts[static_cast<std::size_t>(na.a)];
    const Vec2& pa2 = pts[static_cast<std::size_t>(na.b)];
    const Vec2& pa3 = pts[static_cast<std::size_t>(na.c)];
    const Vec2& pb1 = pts[static_cast<std::size_t>(nb.a)];
    const Vec2& pb2 = pts[static_cast<std::size_t>(nb.b)];
    const Vec2& pb3 = pts[static_cast<std::size_t>(nb.c)];
    if (DelaunayTriangulation::cross2(pa1, pa2, pa3) <= area_eps ||
        DelaunayTriangulation::cross2(pb1, pb2, pb3) <= area_eps) {
      i = j;
      continue;
    }

    // 新对角线已存在则会破坏流形性，跳过。refs 已按 (u, v) 排序，二分查找。
    const int cd_lo = std::min(c, d);
    const int cd_hi = std::max(c, d);
    bool duplicate_edge = false;
    const auto it = std::lower_bound(
        refs.begin(), refs.end(), std::make_pair(cd_lo, cd_hi),
        [](const EdgeRef& e, const std::pair<int, int>& key) {
          return e.u < key.first || (e.u == key.first && e.v < key.second);
        });
    if (it != refs.end() && it->u == cd_lo && it->v == cd_hi &&
        tris[static_cast<std::size_t>(it->tri)].valid()) {
      duplicate_edge = true;
    }
    for (const Triangle& t : new_tris) {
      const int tv[3] = {t.a, t.b, t.c};
      for (int k = 0; k < 3; ++k) {
        const int p = std::min(tv[k], tv[(k + 1) % 3]);
        const int q = std::max(tv[k], tv[(k + 1) % 3]);
        if (p == cd_lo && q == cd_hi) duplicate_edge = true;
      }
    }
    if (duplicate_edge) {
      i = j;
      continue;
    }

    t1 = Triangle{-1, -1, -1};
    t2 = Triangle{-1, -1, -1};
    new_tris.push_back(na);
    new_tris.push_back(nb);
    flipped = true;
    i = j;
  }

  if (!flipped) return false;

  // 紧凑化：丢弃本趟被翻转掉的三角形，追加新三角形
  std::vector<Triangle> kept;
  kept.reserve(tris.size() + new_tris.size());
  for (const Triangle& t : tris) {
    if (t.valid()) kept.push_back(t);
  }
  kept.insert(kept.end(), new_tris.begin(), new_tris.end());
  tris.swap(kept);
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
//  Triangle
// ---------------------------------------------------------------------------

bool Triangle::sameVertexSet(const Triangle& o) const {
  int s1[3] = {a, b, c};
  int s2[3] = {o.a, o.b, o.c};
  std::sort(s1, s1 + 3);
  std::sort(s2, s2 + 3);
  return s1[0] == s2[0] && s1[1] == s2[1] && s1[2] == s2[2];
}

// ---------------------------------------------------------------------------
//  DelaunayTriangulation
// ---------------------------------------------------------------------------

DelaunayTriangulation::DelaunayTriangulation(const std::vector<Vec2>& points)
    : points_(points) {
  build();
}

double DelaunayTriangulation::cross2(const Vec2& a, const Vec2& b,
                                     const Vec2& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double DelaunayTriangulation::area(const Vec2& a, const Vec2& b,
                                   const Vec2& c) {
  return 0.5 * std::fabs(cross2(a, b, c));
}

Vec2 DelaunayTriangulation::circumcenter(const Vec2& a, const Vec2& b,
                                         const Vec2& c) {
  const Vec2 ba = b - a;
  const Vec2 ca = c - a;
  const double det = 2.0 * (ba.x * ca.y - ba.y * ca.x);
  const double ba2 = ba.squaredNorm();
  const double ca2 = ca.squaredNorm();
  const double scale = std::max(ba2, ca2);
  // 面积相对边长平方过小时按退化处理：此时外心数值不可信
  if (!(std::fabs(det) > kDegenerateEps * scale) || scale <= 0.0) {
    return (a + b + c) / 3.0;  // 退化 → 重心
  }
  const double ux = (ca.y * ba2 - ba.y * ca2) / det;
  const double uy = (ba.x * ca2 - ca.x * ba2) / det;
  return {a.x + ux, a.y + uy};
}

double DelaunayTriangulation::circumradius(const Vec2& a, const Vec2& b,
                                           const Vec2& c) {
  const double twiced = 2.0 * cross2(a, b, c);  // = 4 * 面积
  const double la = (b - c).squaredNorm();
  const double lb = (c - a).squaredNorm();
  const double lc = (a - b).squaredNorm();
  const double scale = std::max(la, std::max(lb, lc));
  if (scale <= 0.0) return kHugeRadius;                       // 三点重合
  if (!(std::fabs(twiced) > kDegenerateEps * scale)) {        // 三点共线
    return kHugeRadius;
  }
  // R = abc / (4 * Area) = sqrt(la*lb*lc) / (2 * 面积)
  const double r = std::sqrt(la * lb * lc) / std::fabs(twiced);
  return std::isfinite(r) ? r : kHugeRadius;
}

void DelaunayTriangulation::build() {
  triangles_.clear();

  // --- 1. 工作点集：剔除非法点与完全重合的点 -------------------------------
  // 重合点在几何上不构成新的顶点，保留它们只会让空腔出现零面积三角形。
  std::vector<std::size_t> order;
  order.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (isFinite(points_[i])) order.push_back(i);
  }
  std::sort(order.begin(), order.end(), [this](std::size_t l, std::size_t r) {
    const Vec2& a = points_[l];
    const Vec2& b = points_[r];
    return a.x != b.x ? a.x < b.x : a.y < b.y;
  });

  std::vector<Vec2> pts;   // 去重后的工作点集（末尾追加超级三角形顶点）
  std::vector<int> origin; // 工作顶点 -> 原始点集下标
  pts.reserve(order.size() + 3);
  origin.reserve(order.size() + 3);
  for (std::size_t k = 0; k < order.size(); ++k) {
    const Vec2& p = points_[order[k]];
    if (!origin.empty()) {
      const Vec2& prev = pts.back();
      if (prev.x == p.x && prev.y == p.y) continue;  // 完全重合
    }
    pts.push_back(p);
    origin.push_back(static_cast<int>(order[k]));
  }

  const std::size_t real = pts.size();
  if (real < 3) return;

  // --- 2. 超级三角形 --------------------------------------------------------
  BoundingBox2 box;
  for (const Vec2& p : pts) box.expand(p);
  const double extent =
      std::max(box.max.x - box.min.x, box.max.y - box.min.y);
  const double scale = extent > 0.0 ? extent : 1.0;
  const Vec2 mid = box.center();
  // 20 倍尺度即可保证：全部点落在三角形内部，且落在其外接圆内部，
  // 同时又不至于让 InCircle 判定的相对误差显著放大。
  pts.push_back({mid.x - 20.0 * scale, mid.y - 20.0 * scale});
  pts.push_back({mid.x + 20.0 * scale, mid.y - 20.0 * scale});
  pts.push_back({mid.x, mid.y + 20.0 * scale});
  const int s0 = static_cast<int>(real);
  const int s1 = static_cast<int>(real + 1);
  const int s2 = static_cast<int>(real + 2);

  std::vector<Triangle> tris;
  tris.push_back(Triangle{s0, s1, s2});  // 逆时针

  // 空圆判据：判断三角形 t（逆时针）的"外接圆"是否包含点 q。
  // 含超级顶点的三角形一律按"无穷远点"模型处理：把超级顶点沿自身方向推到无穷远
  // 再取极限。外接圆依次退化为：
  //   3 个超级顶点 -> 全平面（任何有界点都在内部）；
  //   2 个超级顶点 -> 过唯一有限顶点 a、法向由两个无穷远方向解出的直线；
  //   1 个超级顶点 -> 过两个有限顶点的直线，判据化为半平面测试（点到直线的有向面积）。
  // 为什么必须做这个极限：超级顶点虽然取得很远（20 倍尺度），外接圆弧相对弦线的
  // 矢高却只有 |ab|²/(8R)，而点集凸包边缘恰好落在这段矢高以内，会被行列式判成
  // "在圆外"。于是凸包边外侧的三角形不进空腔，而它们正是把空腔与外层超级三角形
  // 连成一片的桥；桥一断，空腔就退化成一串只在顶点处相接的三角形，失去星形性，
  // 扇形重连会生成重叠三角形，残留在原地的三角形在剔除超级三角形后变成空洞
  // （实测可损失数平方米，并造成边被使用三次、空圆性质不成立）。
  // 三个分支必须共用同一套极限模型：若部分用极限、部分用行列式，两侧的邻接判断
  // 就会互相矛盾，同样会断桥（这一点在 300 点随机集的第 76 次插入上实测复现）。
  const int real_count = static_cast<int>(real);
  auto containsPoint = [&](const Triangle& t, const Vec2& q) {
    const int vidx[3] = {t.a, t.b, t.c};
    int sup[3] = {-1, -1, -1};
    int ns = 0;
    int a = -1;
    for (int k = 0; k < 3; ++k) {
      if (vidx[k] >= real_count) {
        sup[ns] = vidx[k];
        ++ns;
      } else {
        a = vidx[k];
      }
    }
    if (ns == 0) {
      return insideCircumcircle(pts[static_cast<std::size_t>(t.a)],
                                pts[static_cast<std::size_t>(t.b)],
                                pts[static_cast<std::size_t>(t.c)], q);
    }
    if (ns >= 2) {
      if (ns == 3) return true;  // 外接圆退化为全平面
      // 两个无穷远方向的极限圆心 O = T*u（T 为超级顶点距离，T -> ∞）：
      // 由 |O|² = |O - T*d_i|² 得 u·d_i = |d_i|²/2，两个方向解出 u，
      // 判据 |q - O|² < |O|² 化为 (q - a)·u > 0（与 d 的缩放无关）。
      const Vec2 da = pts[static_cast<std::size_t>(sup[0])] - mid;
      const Vec2 db = pts[static_cast<std::size_t>(sup[1])] - mid;
      const double det = da.x * db.y - da.y * db.x;
      if (det == 0.0) return false;  // 两方向共线：不会发生，保守取"不含"
      const double ra = 0.5 * (da.x * da.x + da.y * da.y);
      const double rb = 0.5 * (db.x * db.x + db.y * db.y);
      const double ux = (ra * db.y - da.y * rb) / det;
      const double uy = (da.x * rb - ra * db.x) / det;
      const Vec2 av = pts[static_cast<std::size_t>(a)];
      return (q.x - av.x) * ux + (q.y - av.y) * uy > 0.0;
    }
    // 逆时针序下超级顶点位于有限边 (x, y) 的左侧，故"圆内" ⇔ 有向面积为正
    int x = t.a;
    int y = t.b;
    if (t.a >= real_count) {  // a 为超级顶点 → 有限边 (b, c)
      x = t.b;
      y = t.c;
    } else if (t.b >= real_count) {  // b 为超级顶点 → 有限边 (c, a)
      x = t.c;
      y = t.a;
    }  // 否则 c 为超级顶点 → 有限边 (a, b)
    return cross2(pts[static_cast<std::size_t>(x)], pts[static_cast<std::size_t>(y)], q) > 0.0;
  };

  std::vector<int> boundary;
  for (std::size_t k = 0; k < real; ++k) {
    const Vec2& p = pts[k];

    // 2.1 找出外接圆包含新点的全部三角形
    std::vector<int> bad;
    for (std::size_t i = 0; i < tris.size(); ++i) {
      const Triangle& t = tris[i];
      if (!t.valid()) continue;
      if (containsPoint(t, p)) bad.push_back(static_cast<int>(i));
    }
    if (bad.empty()) {
      // 理论上只会在"点与已有顶点重合"时发生；此处保守跳过而非插入，
      // 避免生成零面积三角形破坏后续校验。
      continue;
    }

    // 2.2 空腔边界：属于空腔且反向边不在空腔内的有向边
    boundary.clear();
    for (int ti : bad) {
      const Triangle& t = tris[static_cast<std::size_t>(ti)];
      const int v[3] = {t.a, t.b, t.c};
      for (int e = 0; e < 3; ++e) {
        const int a = v[e];
        const int b = v[(e + 1) % 3];
        bool interior = false;
        for (int tj : bad) {
          if (tj == ti) continue;
          const Triangle& o = tris[static_cast<std::size_t>(tj)];
          const int w[3] = {o.a, o.b, o.c};
          for (int f = 0; f < 3; ++f) {
            if (w[f] == b && w[(f + 1) % 3] == a) {
              interior = true;
              break;
            }
          }
          if (interior) break;
        }
        if (!interior) {  // 相邻两条构成有向边 (a, b)
          boundary.push_back(a);
          boundary.push_back(b);
        }
      }
    }

    // 2.3 删除空腔三角形，用边界边与新点重新三角化
    std::vector<Triangle> next;
    next.reserve(tris.size() + boundary.size());
    std::vector<char> removed(tris.size(), 0);
    for (int ti : bad) removed[static_cast<std::size_t>(ti)] = 1;
    for (std::size_t i = 0; i < tris.size(); ++i) {
      if (!removed[i]) next.push_back(tris[i]);
    }
    const int pk = static_cast<int>(k);
    for (std::size_t e = 0; e + 1 < boundary.size(); e += 2) {
      const Triangle nt{boundary[e], boundary[e + 1], pk};
      if (cross2(pts[static_cast<std::size_t>(nt.a)],
                 pts[static_cast<std::size_t>(nt.b)],
                 pts[static_cast<std::size_t>(nt.c)]) == 0.0) {
        continue;  // 零面积（数值退化）不纳入
      }
      next.push_back(nt);
    }
    tris.swap(next);
  }

  // --- 3. 剔除超级三角形、映射回原始下标 ------------------------------------
  const double area_eps = kDegenerateEps * scale * scale;
  triangles_.reserve(tris.size());
  for (const Triangle& t : tris) {
    if (!t.valid()) continue;
    if (t.a >= static_cast<int>(real) || t.b >= static_cast<int>(real) ||
        t.c >= static_cast<int>(real)) {
      continue;  // 与超级三角形顶点相连
    }
    Triangle m{origin[static_cast<std::size_t>(t.a)],
               origin[static_cast<std::size_t>(t.b)],
               origin[static_cast<std::size_t>(t.c)]};
    const double c = cross2(points_[static_cast<std::size_t>(m.a)],
                            points_[static_cast<std::size_t>(m.b)],
                            points_[static_cast<std::size_t>(m.c)]);
    if (std::fabs(c) <= area_eps) continue;  // 零面积三角形
    if (c < 0.0) std::swap(m.b, m.c);        // 统一为逆时针
    triangles_.push_back(m);
  }

  // --- 4. 退化输入的局部 Delaunay 修复 --------------------------------------
  for (int pass = 0; pass < kMaxFlipPasses; ++pass) {
    if (!flipIllegalEdgesOnce(triangles_, points_, area_eps)) break;
  }

  // --- 5. 顶点组合去重（防御性：正常流程不会产生重复三角形） ----------------
  std::vector<Triangle> unique;
  unique.reserve(triangles_.size());
  for (const Triangle& t : triangles_) {
    bool dup = false;
    for (const Triangle& u : unique) {
      if (u.sameVertexSet(t)) {
        dup = true;
        break;
      }
    }
    if (!dup) unique.push_back(t);
  }
  triangles_.swap(unique);
}

std::vector<std::pair<int, int>> DelaunayTriangulation::edges() const {
  std::vector<std::pair<int, int>> out;
  out.reserve(triangles_.size() * 3);
  for (const Triangle& t : triangles_) {
    if (!t.valid()) continue;
    const int v[3] = {t.a, t.b, t.c};
    for (int k = 0; k < 3; ++k) {
      const int p = v[k];
      const int q = v[(k + 1) % 3];
      out.push_back({std::min(p, q), std::max(p, q)});  // 保证 first < second
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<int> DelaunayTriangulation::convexHull() const {
  // Andrew monotone chain：按 x 排序后分别构造下/上凸壳，结果逆时针。
  // 叉积判据取 "<="，因此共线点被排除，凸包为严格凸多边形。
  std::vector<int> idx;
  idx.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (isFinite(points_[i])) idx.push_back(static_cast<int>(i));
  }
  if (idx.empty()) return {};

  std::sort(idx.begin(), idx.end(), [this](int l, int r) {
    const Vec2& a = points_[static_cast<std::size_t>(l)];
    const Vec2& b = points_[static_cast<std::size_t>(r)];
    return a.x != b.x ? a.x < b.x : a.y < b.y;
  });

  std::vector<int> uniq;
  uniq.reserve(idx.size());
  for (int i : idx) {
    if (!uniq.empty()) {
      const Vec2& prev = points_[static_cast<std::size_t>(uniq.back())];
      const Vec2& cur = points_[static_cast<std::size_t>(i)];
      if (prev.x == cur.x && prev.y == cur.y) continue;
    }
    uniq.push_back(i);
  }
  if (uniq.size() <= 2) return uniq;

  std::vector<int> hull(uniq.size() * 2);
  std::size_t k = 0;
  for (std::size_t i = 0; i < uniq.size(); ++i) {
    const Vec2& p = points_[static_cast<std::size_t>(uniq[i])];
    while (k >= 2 &&
           cross2(points_[static_cast<std::size_t>(hull[k - 2])],
                  points_[static_cast<std::size_t>(hull[k - 1])], p) <= 0.0) {
      --k;
    }
    hull[k++] = uniq[i];
  }
  const std::size_t upper_from = k + 1;
  for (std::size_t i = uniq.size() - 1; i-- > 0;) {
    const Vec2& p = points_[static_cast<std::size_t>(uniq[i])];
    while (k >= upper_from &&
           cross2(points_[static_cast<std::size_t>(hull[k - 2])],
                  points_[static_cast<std::size_t>(hull[k - 1])], p) <= 0.0) {
      --k;
    }
    hull[k++] = uniq[i];
  }
  hull.resize(k > 0 ? k - 1 : 0);  // 末点与首点重合
  return hull;
}

std::vector<int> DelaunayTriangulation::locate(const Vec2& p) const {
  std::vector<int> hits;
  for (std::size_t i = 0; i < triangles_.size(); ++i) {
    const Triangle& t = triangles_[i];
    if (!t.valid()) continue;
    const Vec2& a = points_[static_cast<std::size_t>(t.a)];
    const Vec2& b = points_[static_cast<std::size_t>(t.b)];
    const Vec2& c = points_[static_cast<std::size_t>(t.c)];
    const double s =
        std::max((b - a).squaredNorm(),
                 std::max((c - b).squaredNorm(), (a - c).squaredNorm()));
    const double eps = kLocateEps * s;
    // 边界上的点同时属于相邻的多个三角形，故用 >= 判据
    if (cross2(a, b, p) >= -eps && cross2(b, c, p) >= -eps &&
        cross2(c, a, p) >= -eps) {
      hits.push_back(static_cast<int>(i));
    }
  }
  return hits;
}

bool DelaunayTriangulation::satisfiesEmptyCircleProperty(double tolerance) const {
  if (triangles_.empty()) return true;
  // 判定时把外接圆半径放宽 tolerance 倍：恰好共圆（网格点、四点共圆）的
  // 点因浮点误差可能被算到圆内，放宽后不会误报。
  const double relax = 1.0 - (tolerance > 0.0 ? tolerance : 0.0);
  for (const Triangle& t : triangles_) {
    if (!t.valid()) continue;
    const Vec2& a = points_[static_cast<std::size_t>(t.a)];
    const Vec2& b = points_[static_cast<std::size_t>(t.b)];
    const Vec2& c = points_[static_cast<std::size_t>(t.c)];
    const double r = circumradius(a, b, c);
    if (r >= kHugeRadius) continue;  // 退化三角形（构建阶段已剔除）
    const Vec2 o = circumcenter(a, b, c);
    const double r_eff = r * relax;
    for (std::size_t i = 0; i < points_.size(); ++i) {
      if (static_cast<int>(i) == t.a || static_cast<int>(i) == t.b ||
          static_cast<int>(i) == t.c) {
        continue;
      }
      if (!isFinite(points_[i])) continue;
      if ((points_[i] - o).norm() < r_eff) return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
//  VoronoiRoadmap
// ---------------------------------------------------------------------------

VoronoiRoadmap::VoronoiRoadmap(const std::vector<Vec2>& obstacle_points)
    : obstacles_(obstacle_points) {
  nodes_.clear();
  adjacency_.clear();
  if (obstacles_.size() < 3) return;

  const DelaunayTriangulation dt(obstacles_);
  const std::vector<Triangle>& tris = dt.triangles();
  const std::vector<Vec2>& pts = dt.points();
  if (tris.empty()) return;

  // 尺度：用于剔除"外接圆半径相对点集尺寸过大"的退化三角形——其外心远离
  // 障碍物区域，作为路线图节点会引入横穿障碍物的假连接。
  BoundingBox2 box;
  for (const Vec2& p : pts) box.expand(p);
  double extent = 0.0;
  if (box.valid()) {
    extent = std::max(box.max.x - box.min.x, box.max.y - box.min.y);
  }
  if (!(extent > 0.0)) extent = 1.0;
  const double r_limit = extent * 1e3;

  std::vector<int> node_of_tri(tris.size(), -1);
  for (std::size_t i = 0; i < tris.size(); ++i) {
    const Triangle& t = tris[i];
    if (!t.valid()) continue;
    const Vec2& a = pts[static_cast<std::size_t>(t.a)];
    const Vec2& b = pts[static_cast<std::size_t>(t.b)];
    const Vec2& c = pts[static_cast<std::size_t>(t.c)];
    const double r = DelaunayTriangulation::circumradius(a, b, c);
    if (!std::isfinite(r) || r > r_limit) continue;  // 退化：跳过
    if (!(DelaunayTriangulation::area(a, b, c) > 0.0)) continue;
    const Vec2 center = DelaunayTriangulation::circumcenter(a, b, c);
    if (!isFinite(center)) continue;
    node_of_tri[i] = static_cast<int>(nodes_.size());
    nodes_.push_back(center);
  }

  adjacency_.assign(nodes_.size(), std::vector<int>());
  if (nodes_.empty()) return;

  // 共享一条边的两个三角形 → 其外心相连（即 Voronoi 图的一条边）
  struct EdgeRef {
    int u;
    int v;
    int tri;
  };
  std::vector<EdgeRef> refs;
  refs.reserve(tris.size() * 3);
  for (std::size_t i = 0; i < tris.size(); ++i) {
    if (node_of_tri[i] < 0) continue;
    const int v[3] = {tris[i].a, tris[i].b, tris[i].c};
    for (int k = 0; k < 3; ++k) {
      const int p = v[k];
      const int q = v[(k + 1) % 3];
      refs.push_back(EdgeRef{std::min(p, q), std::max(p, q),
                             static_cast<int>(i)});
    }
  }
  std::sort(refs.begin(), refs.end(), [](const EdgeRef& l, const EdgeRef& r) {
    return l.u != r.u ? l.u < r.u : (l.v != r.v ? l.v < r.v : l.tri < r.tri);
  });

  for (std::size_t i = 0; i < refs.size();) {
    std::size_t j = i;
    while (j < refs.size() && refs[j].u == refs[i].u && refs[j].v == refs[i].v) {
      ++j;
    }
    for (std::size_t m = i; m < j; ++m) {
      for (std::size_t n = m + 1; n < j; ++n) {
        const int na = node_of_tri[static_cast<std::size_t>(refs[m].tri)];
        const int nb = node_of_tri[static_cast<std::size_t>(refs[n].tri)];
        if (na < 0 || nb < 0 || na == nb) continue;
        adjacency_[static_cast<std::size_t>(na)].push_back(nb);
        adjacency_[static_cast<std::size_t>(nb)].push_back(na);
      }
    }
    i = j;
  }
}

int VoronoiRoadmap::nearestNode(const Vec2& p) const {
  int best = -1;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const double d2 = (nodes_[i] - p).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best = static_cast<int>(i);
    }
  }
  return best;
}

std::vector<Vec2> VoronoiRoadmap::search(const Vec2& start,
                                         const Vec2& goal) const {
  std::vector<Vec2> path;
  if (nodes_.empty()) return path;

  const int s = nearestNode(start);
  const int g = nearestNode(goal);
  if (s < 0 || g < 0) return path;

  // Dijkstra（惰性删除的优先队列），边权为节点间欧氏距离
  const std::size_t n = nodes_.size();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(n, inf);
  std::vector<int> prev(n, -1);
  std::vector<char> settled(n, 0);
  using Item = std::pair<double, int>;  // 比较先看距离
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;

  dist[static_cast<std::size_t>(s)] = 0.0;
  queue.push({0.0, s});
  while (!queue.empty()) {
    const Item top = queue.top();
    queue.pop();
    const int u = top.second;
    if (settled[static_cast<std::size_t>(u)]) continue;
    settled[static_cast<std::size_t>(u)] = 1;
    if (u == g) break;
    for (int v : adjacency_[static_cast<std::size_t>(u)]) {
      const double w = (nodes_[static_cast<std::size_t>(u)] -
                        nodes_[static_cast<std::size_t>(v)])
                           .norm();
      const double nd = dist[static_cast<std::size_t>(u)] + w;
      if (nd < dist[static_cast<std::size_t>(v)]) {
        dist[static_cast<std::size_t>(v)] = nd;
        prev[static_cast<std::size_t>(v)] = u;
        queue.push({nd, v});
      }
    }
  }
  if (!(dist[static_cast<std::size_t>(g)] < inf)) return path;  // 不连通

  std::vector<int> chain;
  for (int cur = g; cur >= 0; cur = prev[static_cast<std::size_t>(cur)]) {
    chain.push_back(cur);
  }
  std::reverse(chain.begin(), chain.end());

  // 真实起终点接回路径首尾，保证输出是可直接跟踪的完整折线
  path.reserve(chain.size() + 2);
  path.push_back(start);
  for (int idx : chain) path.push_back(nodes_[static_cast<std::size_t>(idx)]);
  path.push_back(goal);
  return path;
}

double VoronoiRoadmap::clearanceAt(const Vec2& p) const {
  if (obstacles_.empty()) {
    return std::numeric_limits<double>::max();  // 无障碍物 → 间隙无限大
  }
  double best = std::numeric_limits<double>::max();
  for (const Vec2& o : obstacles_) {
    best = std::min(best, (o - p).norm());
  }
  return best;
}

double VoronoiRoadmap::minClearanceAlong(const std::vector<Vec2>& path) const {
  if (path.empty()) return 0.0;  // 无路径 → 无有效间隙
  double best = std::numeric_limits<double>::max();
  for (const Vec2& p : path) best = std::min(best, clearanceAt(p));
  return best;
}

}  // namespace geometry
}  // namespace adsim
