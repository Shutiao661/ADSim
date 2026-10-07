// =============================================================================
//  Spline.cpp — 三次 B 样条曲线与路径平滑实现
//
//  求值统一走 De Boor 算法（O(p^2) 且数值稳定），导数不采用"对基函数逐个
//  求导再求和"的朴素做法，而是利用 B 样条的标准性质：p 次样条的 k 阶导数
//  仍是 B 样条，其控制点由差分给出
//      Q_i = p * (P_{i+1} - P_i) / (U_{i+p+1} - U_{i+1})
//  节点向量去掉首末各一个节点、次数降 1。该构造对非 clamped 节点同样成立，
//  差分求导的数值精度与有限差分完全一致（见单元测试）。
//
//  拟合采用"弦长参数化 → 最小二乘解控制点 → 重参数化"的迭代策略：
//  固定首末控制点为数据端点（clamped 条件），仅对内部控制点求解法方程
//  (NᵀN) C = NᵀR，高斯消元带部分主元。
// =============================================================================
#include "adsim/planning/geometry/Spline.h"

#include "adsim/planning/geometry/CurveFit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace adsim {
namespace geometry {

namespace {

/// 三次 B 样条
constexpr int kDegree = 3;

/// 导数范数平方小于该值时视为退化（切向量为零），曲率按 0 处理
constexpr double kMinSpeed2 = 1e-18;

/// 重合点判定阈值：小于该距离的两点视为同一点
constexpr double kDupEps = 1e-9;

/// 拟合前的 RDP 形状容差 (m)：5cm 足以剔除直线段上的冗余采样点，
/// 又不会削掉真实的弯道形状
constexpr double kRdpEps = 0.05;

/// 平滑迭代的曲率检查采样数（比输出采样更密，用于捕捉样条内部的曲率峰值）
constexpr std::size_t kFineCurvatureSamples = 512;

/// 定位 u 所在节点区间（The NURBS Book A2.1）。
/// 采用线性插值搜索；u 落在参数域之外时返回端点区间，等价于把曲线端点
/// 作为取值结果，避免对越界输入抛出异常。
std::size_t findSpanGeneric(const std::vector<double>& knots,
                            std::size_t num_ctrl, int degree, double u) {
  if (num_ctrl <= static_cast<std::size_t>(degree) || knots.empty()) return 0;
  const std::size_t n = num_ctrl - 1;  // 控制点最大下标
  if (n + 1 >= knots.size()) return n;
  if (u >= knots[n + 1]) return n;
  if (u <= knots[static_cast<std::size_t>(degree)]) {
    return static_cast<std::size_t>(degree);
  }
  std::size_t low = static_cast<std::size_t>(degree), high = n + 1;
  std::size_t mid = (low + high) / 2;
  // 二分收敛到相邻区间即停；再加一个迭代上限，杜绝异常节点向量导致的死循环
  for (std::size_t guard = 0; guard < knots.size() * 2 + 8; ++guard) {
    if (!(u < knots[mid] || u >= knots[mid + 1])) break;
    if (u < knots[mid]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
    if (high <= low + 1) break;
  }
  return mid;
}

/// De Boor 求值（The NURBS Book A3.1）。
/// @param knots 长度必须是 ctrl.size() + degree + 1
Vec2 deBoor(const std::vector<Vec2>& ctrl, const std::vector<double>& knots,
            int degree, double u) {
  if (ctrl.empty()) return {0.0, 0.0};
  if (ctrl.size() <= static_cast<std::size_t>(degree)) return ctrl.front();
  // 先把 u 夹到参数域 [knots[degree], knots[ctrl.size()]] 内再求值。
  // De Boor 公式本身允许外推，但外推时权重 alpha 会跑出 [0,1]，控制点被
  // (1-alpha) 成倍放大，曲线迅速冲向远处：实测非 clamped 曲线 evaluate(0)
  // 得到 (4.8, -19.3)，而曲线起点是 (12.3, -15.7)，偏差几十米。夹紧后越界
  // 求值等价于取最近端点，既安全，也与接口约定 "u ∈ [0, 1]" 的语义一致。
  const double u_lo = knots[std::min(static_cast<std::size_t>(degree), knots.size() - 1)];
  const double u_hi = knots[std::min(ctrl.size(), knots.size() - 1)];
  if (u < u_lo) u = u_lo;
  if (u > u_hi) u = u_hi;
  const std::size_t k = findSpanGeneric(knots, ctrl.size(), degree, u);
  std::vector<Vec2> d(static_cast<std::size_t>(degree) + 1);
  for (int j = 0; j <= degree; ++j) {
    const std::size_t idx = k - static_cast<std::size_t>(degree) +
                            static_cast<std::size_t>(j);
    d[static_cast<std::size_t>(j)] = ctrl[idx];
  }
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const std::size_t jj = static_cast<std::size_t>(j);
      const std::size_t ka = k + 1 + jj - static_cast<std::size_t>(r);
      const std::size_t kb = k - static_cast<std::size_t>(degree) + jj;
      const double den = knots[ka] - knots[kb];
      // 零长度节点区间（重复节点）时取 alpha = 0，与标准算法在端点处的
      // 极限行为一致，避免 0/0 产生 NaN
      const double alpha = den > 0.0 ? (u - knots[kb]) / den : 0.0;
      d[jj] = d[jj - 1] * (1.0 - alpha) + d[jj] * alpha;
    }
  }
  return d[static_cast<std::size_t>(degree)];
}

/// u 处 4 个非零三次基函数值（A2.2），依次对应控制点 span-3 .. span
void basisFunctions3(const std::vector<double>& knots, std::size_t span,
                     double u, double* n) {
  double left[4] = {0.0, 0.0, 0.0, 0.0};
  double right[4] = {0.0, 0.0, 0.0, 0.0};
  n[0] = 1.0;
  for (std::size_t j = 1; j <= 3; ++j) {
    left[j] = u - knots[span + 1 - j];
    right[j] = knots[span + j] - u;
    double saved = 0.0;
    for (std::size_t r = 0; r < j; ++r) {
      const double den = right[r + 1] + left[j - r];
      const double temp = den > 0.0 ? n[r] / den : 0.0;
      n[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    n[j] = saved;
  }
}

/// 与 CubicBSpline 构造函数完全一致的 clamped 节点向量。
/// 拟合时必须使用同一节点向量，否则求出的控制点与曲线求值所用的基函数不符。
std::vector<double> clampedKnots(std::size_t num_ctrl) {
  const std::size_t m = num_ctrl + static_cast<std::size_t>(kDegree) + 1;
  std::vector<double> knots(m, 0.0);
  for (std::size_t i = static_cast<std::size_t>(kDegree) + 1; i + 4 < m; ++i) {
    knots[i] = static_cast<double>(i - 3) / static_cast<double>(num_ctrl - 3);
  }
  for (std::size_t i = m - 4; i < m; ++i) knots[i] = 1.0;
  return knots;
}

/// 带部分主元的高斯消元，就地求解 A * X = B（B 有多列，位置与 x/y 两分量对应）。
/// @return 矩阵奇异或解非有限时返回 false
bool solveGauss(std::vector<std::vector<double>>& a,
                std::vector<std::vector<double>>& b) {
  const std::size_t m = a.size();
  if (m == 0) return false;
  const std::size_t nrhs = b.empty() ? 0 : b[0].size();

  for (std::size_t col = 0; col < m; ++col) {
    // 部分主元：选列中绝对值最大的行，抑制小主元引起的误差放大
    std::size_t pivot = col;
    for (std::size_t r = col + 1; r < m; ++r) {
      if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) pivot = r;
    }
    if (a[pivot][col] == 0.0) return false;
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(b[pivot], b[col]);
    }
    const double inv = 1.0 / a[col][col];
    for (std::size_t r = col + 1; r < m; ++r) {
      const double factor = a[r][col] * inv;
      if (factor == 0.0) continue;
      for (std::size_t c = col; c < m; ++c) a[r][c] -= factor * a[col][c];
      for (std::size_t c = 0; c < nrhs; ++c) b[r][c] -= factor * b[col][c];
    }
  }

  for (std::size_t i = m; i-- > 0;) {
    for (std::size_t c = 0; c < nrhs; ++c) {
      double sum = b[i][c];
      for (std::size_t j = i + 1; j < m; ++j) sum -= a[i][j] * b[j][c];
      b[i][c] = sum / a[i][i];
      if (!std::isfinite(b[i][c])) return false;
    }
  }
  return true;
}

/// 弦长参数化：按累计弦长把路点映射到 [0, 1]。
/// 总长为 0（全部重合）时退化为均匀参数化，避免除零。
std::vector<double> chordLengthParams(const std::vector<Vec2>& pts) {
  const std::size_t n = pts.size();
  std::vector<double> u(n, 0.0);
  if (n == 0) return u;
  std::vector<double> acc(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    acc[i] = acc[i - 1] + (pts[i] - pts[i - 1]).norm();
  }
  const double total = acc[n - 1];
  if (total <= kDupEps) {
    for (std::size_t i = 0; i < n; ++i) {
      u[i] = static_cast<double>(i) / static_cast<double>(n - 1);
    }
    return u;
  }
  for (std::size_t i = 0; i < n; ++i) u[i] = acc[i] / total;
  return u;
}

/// 曲线参数域（clamped 时为 [0, 1]）
void splineDomain(const CubicBSpline& s, double* u0, double* u1) {
  const std::vector<double>& knots = s.knots();
  const std::size_t n = s.controlPoints().size();
  const std::size_t lo = static_cast<std::size_t>(kDegree);
  const std::size_t hi = std::min(n, knots.size() - 1);
  *u0 = knots.empty() ? 0.0 : knots[std::min(lo, knots.size() - 1)];
  *u1 = knots.empty() ? 1.0 : knots[hi];
}

/// 点 p 到曲线的最近参数（粗采样定位 + 三分法细化）
double closestParam(const CubicBSpline& s, const Vec2& p, double u0, double u1,
                    std::size_t coarse) {
  if (!(u1 > u0)) return u0;
  const std::size_t m = std::max<std::size_t>(coarse, 4);
  double best_u = u0;
  double best_d2 = std::numeric_limits<double>::max();
  const double step = (u1 - u0) / static_cast<double>(m);
  for (std::size_t i = 0; i <= m; ++i) {
    const double u = u0 + step * static_cast<double>(i);
    const double d2 = (s.evaluate(u) - p).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_u = u;
    }
  }
  // 在最优采样点邻域内三分搜索（距离函数局部单峰）
  double lo = std::max(u0, best_u - step);
  double hi = std::min(u1, best_u + step);
  for (int iter = 0; iter < 48 && hi - lo > 1e-14; ++iter) {
    const double m1 = lo + (hi - lo) / 3.0;
    const double m2 = hi - (hi - lo) / 3.0;
    if ((s.evaluate(m1) - p).squaredNorm() <
        (s.evaluate(m2) - p).squaredNorm()) {
      hi = m2;
    } else {
      lo = m1;
    }
  }
  const double u = 0.5 * (lo + hi);
  return (s.evaluate(u) - p).squaredNorm() < best_d2 ? u : best_u;
}

/// 一次拉普拉斯平滑扫掠：内部控制点向其邻居均值靠拢，首末控制点固定，
/// 从而在降低曲率的同时保证曲线端点不漂移。
void laplacianSweep(std::vector<Vec2>& ctrl, double lambda) {
  if (ctrl.size() < 3) return;
  std::vector<Vec2> next = ctrl;
  for (std::size_t i = 1; i + 1 < ctrl.size(); ++i) {
    next[i] = ctrl[i] + (ctrl[i - 1] + ctrl[i + 1] - ctrl[i] * 2.0) * lambda;
  }
  ctrl.swap(next);
}

/// 逐段折线的方向（跳过重合点）；找不到有效方向时返回零向量
Vec2 segmentDirection(const std::vector<Vec2>& pts, std::size_t from,
                      std::size_t to) {
  for (std::size_t i = from; i < to; ++i) {
    const Vec2 d = pts[i + 1] - pts[i];
    if (d.squaredNorm() > kDupEps * kDupEps) return d;
  }
  return {0.0, 0.0};
}

}  // namespace

// ---------------------------------------------------------------------------
//  CubicBSpline
// ---------------------------------------------------------------------------

CubicBSpline::CubicBSpline(const std::vector<Vec2>& control_points, bool clamped)
    : control_points_(control_points), clamped_(clamped) {
  const std::size_t n = control_points_.size();
  // 三次样条至少需要 4 个控制点，否则节点向量无定义（构造前即报错，
  // 避免后续求值静默产生无意义结果）
  if (n < 4) {
    throw std::invalid_argument("CubicBSpline 至少需要 4 个控制点");
  }
  const std::size_t m = n + static_cast<std::size_t>(kDegree) + 1;  // 节点数
  knots_.assign(m, 0.0);
  if (clamped_) {
    // 首末节点重复度 4：曲线穿过首末控制点；内部节点均匀分布
    for (std::size_t i = 4; i + 4 < m; ++i) {
      knots_[i] = static_cast<double>(i - 3) / static_cast<double>(n - 3);
    }
    for (std::size_t i = m - 4; i < m; ++i) knots_[i] = 1.0;
  } else {
    // 均匀节点向量：参数域为 [knots[3], knots[n]]，不是 [0, 1]
    for (std::size_t i = 0; i < m; ++i) {
      knots_[i] = static_cast<double>(i) / static_cast<double>(m - 1);
    }
  }
}

Vec2 CubicBSpline::evaluate(double u) const {
  return deBoor(control_points_, knots_, kDegree, u);
}

Vec2 CubicBSpline::evaluateDerivative(double u, int order) const {
  if (order <= 0) return evaluate(u);
  // 三阶以上导数为常数 0
  if (order > kDegree) return {0.0, 0.0};

  // 逐阶降阶：Q_i = p (P_{i+1} - P_i) / (U_{i+p+1} - U_{i+1})
  std::vector<Vec2> ctrl = control_points_;
  std::vector<double> knots = knots_;
  int degree = kDegree;
  for (int step = 0; step < order; ++step) {
    if (degree <= 0 || ctrl.size() < 2) break;
    std::vector<Vec2> next;
    next.reserve(ctrl.size() - 1);
    for (std::size_t i = 0; i + 1 < ctrl.size(); ++i) {
      const double den = knots[i + static_cast<std::size_t>(degree) + 1] -
                         knots[i + 1];
      // 重复节点导致的零长度区间：该控制点系数取 0（NURBS 标准约定）
      const double s = den > 0.0 ? static_cast<double>(degree) / den : 0.0;
      next.push_back((ctrl[i + 1] - ctrl[i]) * s);
    }
    ctrl.swap(next);
    knots.erase(knots.begin());
    knots.pop_back();
    --degree;
  }
  return deBoor(ctrl, knots, degree, u);
}

Vec2 CubicBSpline::derivative(double u) const {
  return evaluateDerivative(u, 1);
}

Vec2 CubicBSpline::secondDerivative(double u) const {
  return evaluateDerivative(u, 2);
}

double CubicBSpline::curvature(double u) const {
  const Vec2 d1 = derivative(u);
  const Vec2 d2 = secondDerivative(u);
  const double den = d1.squaredNorm();
  if (den < kMinSpeed2) return 0.0;  // 切向量为零：曲率无定义，按 0 处理
  const double cross = d1.x * d2.y - d1.y * d2.x;
  return std::fabs(cross) / (den * std::sqrt(den));  // |x'y''-y'x''| / |P'|^3
}

double CubicBSpline::approximateLength(std::size_t samples) const {
  // 复合 Simpson：被积函数 |P'(u)| 用差分控制点解析给出，比差商更精确
  std::size_t m = samples;
  if (m < 2) m = 2;
  if (m % 2 != 0) ++m;  // Simpson 需要偶数个区间
  double u0 = 0.0;
  double u1 = 1.0;
  splineDomain(*this, &u0, &u1);
  const double h = (u1 - u0) / static_cast<double>(m);
  if (!(h > 0.0)) return 0.0;

  double sum = 0.0;
  for (std::size_t i = 0; i <= m; ++i) {
    const double u = u0 + h * static_cast<double>(i);
    const double f = derivative(u).norm();
    const double w = (i == 0 || i == m) ? 1.0 : ((i % 2 == 1) ? 4.0 : 2.0);
    sum += w * f;
  }
  return sum * h / 3.0;
}

Trajectory CubicBSpline::sample(std::size_t count) const {
  Trajectory traj;
  if (count == 0) return traj;
  double u0 = 0.0;
  double u1 = 1.0;
  splineDomain(*this, &u0, &u1);
  traj.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const double t = count > 1 ? static_cast<double>(i) /
                                     static_cast<double>(count - 1)
                               : 0.0;
    const double u = u0 + (u1 - u0) * t;
    const Vec2 p = evaluate(u);
    const Vec2 d = derivative(u);

    TrajectoryPoint tp;
    tp.x = p.x;
    tp.y = p.y;
    if (d.squaredNorm() > kMinSpeed2) {
      tp.theta = std::atan2(d.y, d.x);
    } else {
      // 切向量退化：沿用前一点航向，保证输出航向连续
      tp.theta = traj.empty() ? 0.0 : traj.back().theta;
    }
    tp.kappa = curvature(u);
    traj.push_back(tp);
  }
  return traj;
}

Trajectory CubicBSpline::sampleByArcLength(std::size_t count) const {
  Trajectory traj;
  if (count == 0) return traj;
  double u0 = 0.0;
  double u1 = 1.0;
  splineDomain(*this, &u0, &u1);

  // 先建弧长表（比输出采样更密），再按等弧长目标反查参数
  const std::size_t table = std::max<std::size_t>(256, count * 16);
  std::vector<double> uu(table + 1, 0.0);
  std::vector<double> ss(table + 1, 0.0);
  for (std::size_t i = 0; i <= table; ++i) {
    uu[i] = u0 + (u1 - u0) * static_cast<double>(i) /
                      static_cast<double>(table);
    if (i > 0) {
      ss[i] = ss[i - 1] + (evaluate(uu[i]) - evaluate(uu[i - 1])).norm();
    }
  }
  const double total = ss.back();
  if (!(total > 0.0)) return sample(count);  // 零长度曲线：退化为等参数采样

  traj.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double t = count > 1 ? static_cast<double>(i) /
                                     static_cast<double>(count - 1)
                               : 0.0;
    const double target = total * t;
    std::size_t lo = 0, hi = table;
    while (lo + 1 < hi) {
      const std::size_t mid = (lo + hi) / 2;
      if (ss[mid] <= target) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    const double ds = ss[hi] - ss[lo];
    const double w = ds > 0.0 ? (target - ss[lo]) / ds : 0.0;
    const double u = uu[lo] + (uu[hi] - uu[lo]) * w;

    const Vec2 p = evaluate(u);
    const Vec2 d = derivative(u);
    TrajectoryPoint tp;
    tp.x = p.x;
    tp.y = p.y;
    if (d.squaredNorm() > kMinSpeed2) {
      tp.theta = std::atan2(d.y, d.x);
    } else {
      tp.theta = traj.empty() ? 0.0 : traj.back().theta;
    }
    tp.kappa = curvature(u);
    traj.push_back(tp);
  }
  return traj;
}

// ---------------------------------------------------------------------------
//  最小二乘拟合
// ---------------------------------------------------------------------------

BSplineFitResult fitCubicBSpline(const std::vector<Vec2>& waypoints,
                                 std::size_t control_count,
                                 std::size_t iterations) {
  BSplineFitResult result;
  const std::size_t n_data = waypoints.size();
  if (n_data < 2) return result;

  const std::size_t max_iter = std::max<std::size_t>(iterations, 1);

  // 点数不足以构成三次样条（< 4）时退化为精确的贝塞尔段：
  // 2 点 → 直线段；3 点 → 过中点的二次曲线升阶为三次。
  if (n_data < 4) {
    const Vec2& p0 = waypoints.front();
    const Vec2& pz = waypoints.back();
    if (n_data == 2) {
      result.control_points = {p0, p0 + (pz - p0) / 3.0,
                               p0 + (pz - p0) * (2.0 / 3.0), pz};
    } else {
      const Vec2 q1 = waypoints[1] * 2.0 - (p0 + pz) * 0.5;
      result.control_points = {p0, p0 + (q1 - p0) * (2.0 / 3.0),
                               pz + (q1 - pz) * (2.0 / 3.0), pz};
    }
    result.iterations = 1;
    const CubicBSpline spline(result.control_points, true);
    const std::vector<double> params = chordLengthParams(waypoints);
    double u0 = 0.0, u1 = 1.0;
    splineDomain(spline, &u0, &u1);
    double sum2 = 0.0, max_err = 0.0;
    for (std::size_t i = 0; i < n_data; ++i) {
      const double d = (spline.evaluate(params[i]) - waypoints[i]).norm();
      sum2 += d * d;
      max_err = std::max(max_err, d);
    }
    result.rms_error = std::sqrt(sum2 / static_cast<double>(n_data));
    result.max_error = max_err;
    return result;
  }

  // 控制点数不能超过路点数：否则法方程欠定（NᵀN 奇异）
  std::size_t num_ctrl = std::max<std::size_t>(4, control_count);
  num_ctrl = std::min(num_ctrl, n_data);

  // 病态判据：B 样条曲线含于控制点凸包内，拟合良好的控制点必然落在数据附近
  // （凸包比数据包围盒略大是正常的，故留 4 倍余量）。注意必须相对数据包围盒
  // 判断，不能用"到原点的距离"——否则远离原点的路径（如 UTM 坐标 x≈5e5）会被
  // 误判为病态而白白降阶。
  BoundingBox2 data_box;
  for (const Vec2& p : waypoints) data_box.expand(p);
  const double ctrl_margin =
      4.0 * (std::max(data_box.max.x - data_box.min.x,
                      data_box.max.y - data_box.min.y) +
             1.0);

  const std::vector<double> knots = clampedKnots(num_ctrl);
  std::vector<double> params = chordLengthParams(waypoints);
  const std::size_t interior = num_ctrl - 2;  // 待求的内部控制点数

  std::vector<Vec2> ctrl(num_ctrl);
  ctrl.front() = waypoints.front();
  ctrl.back() = waypoints.back();

  std::size_t used = 0;
  for (std::size_t iter = 0; iter < max_iter; ++iter) {
    // --- 最小二乘解法方程 (NᵀN) X = Nᵀ R -------------------------------
    std::vector<std::vector<double>> ata(interior,
                                         std::vector<double>(interior, 0.0));
    std::vector<std::vector<double>> atb(interior, std::vector<double>(2, 0.0));
    for (std::size_t i = 0; i < n_data; ++i) {
      const std::size_t span =
          findSpanGeneric(knots, num_ctrl, kDegree, params[i]);
      double n[4];
      basisFunctions3(knots, span, params[i], n);
      const std::size_t base = span - static_cast<std::size_t>(kDegree);

      // 固定首末控制点后移到右端项
      Vec2 rhs = waypoints[i];
      for (std::size_t j = 0; j < 4; ++j) {
        const std::size_t col = base + j;
        if (col == 0) {
          rhs -= ctrl.front() * n[j];
        } else if (col + 1 == num_ctrl) {
          rhs -= ctrl.back() * n[j];
        }
      }
      for (std::size_t a = 0; a < 4; ++a) {
        const std::size_t ca = base + a;
        if (ca == 0 || ca + 1 == num_ctrl) continue;
        for (std::size_t b = 0; b < 4; ++b) {
          const std::size_t cb = base + b;
          if (cb == 0 || cb + 1 == num_ctrl) continue;
          ata[ca - 1][cb - 1] += n[a] * n[b];
        }
        atb[ca - 1][0] += n[a] * rhs.x;
        atb[ca - 1][1] += n[a] * rhs.y;
      }
    }

    if (!solveGauss(ata, atb)) break;  // 奇异（如路点全部重合）
    for (std::size_t i = 0; i < interior; ++i) {
      ctrl[i + 1] = Vec2(atb[i][0], atb[i][1]);
    }
    ++used;

    // --- 误差评估：路点到拟合曲线的几何距离（可直接与重参数化比较） -------
    const CubicBSpline spline(ctrl, true);
    double u0 = 0.0, u1 = 1.0;
    splineDomain(spline, &u0, &u1);
    double sum2 = 0.0, max_err = 0.0;
    std::vector<double> refined(n_data, 0.0);
    for (std::size_t i = 0; i < n_data; ++i) {
      refined[i] = closestParam(spline, waypoints[i], u0, u1, 64);
      const double d = (spline.evaluate(refined[i]) - waypoints[i]).norm();
      sum2 += d * d;
      max_err = std::max(max_err, d);
    }
    result.control_points = ctrl;
    result.rms_error = std::sqrt(sum2 / static_cast<double>(n_data));
    result.max_error = max_err;
    result.iterations = used;

    // --- 重参数化：用曲线上最近的参数替换弦长参数 ---------------------------
    if (iter + 1 >= max_iter) break;
    std::vector<double> next = params;
    next.front() = 0.0;
    next.back() = 1.0;
    bool monotone = true;
    for (std::size_t i = 1; i + 1 < n_data; ++i) {
      next[i] = refined[i];
      if (!(next[i] > next[i - 1])) {
        monotone = false;  // 路径回折：无法保持参数单调，保留原参数
        break;
      }
    }
    if (!monotone) break;
    params.swap(next);
  }

  // 求解失败（法方程奇异）或控制点尺度失控：降阶重拟合。
  // 为什么必须自检：控制点数接近路点数时，弦长参数化在路径回折处高度聚簇，
  // 法方程条件数急剧恶化——实测 14 个路点请求 14 个控制点时解出 6.2e6 量级的
  // 控制点（曲线飞出 370 万米），且 rms 反而比 9 个控制点更差；这条曲线会被
  // smoothPathWithCurvatureLimit 当成低曲率的"好"结果返回，是真正的隐患。
  // 递归每次把控制点数减半，到 4 个控制点（仅 2 个内部未知量）必停，深度 ≤ log2(n)。
  if (num_ctrl > 4) {
    const bool unsolved = result.control_points.size() < 4;
    bool out_of_scale = false;
    for (const Vec2& c : result.control_points) {
      if (!std::isfinite(c.x) || !std::isfinite(c.y) ||
          c.x < data_box.min.x - ctrl_margin || c.x > data_box.max.x + ctrl_margin ||
          c.y < data_box.min.y - ctrl_margin || c.y > data_box.max.y + ctrl_margin) {
        out_of_scale = true;
        break;
      }
    }
    if (unsolved || out_of_scale) {
      return fitCubicBSpline(waypoints, num_ctrl / 2, iterations);
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
//  路径平滑
// ---------------------------------------------------------------------------

Trajectory smoothPathWithCurvatureLimit(const std::vector<Vec2>& waypoints,
                                        double max_kappa,
                                        std::size_t sample_count) {
  Trajectory out;
  if (waypoints.size() < 2) return out;
  if (sample_count < 2) sample_count = 2;

  // 1. 剔除非法点与相邻重合点：重合点会让弦长参数化、曲率估计同时失效
  std::vector<Vec2> pts;
  pts.reserve(waypoints.size());
  for (const Vec2& p : waypoints) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
    if (!pts.empty() && (pts.back() - p).norm() <= kDupEps) continue;
    pts.push_back(p);
  }
  if (pts.size() < 2) return out;

  // 2. RDP 抽稀：去掉直线段上的冗余点，减少拟合的过约束
  const std::size_t keep = std::min<std::size_t>(4, pts.size());
  double eps = kRdpEps;
  std::vector<Vec2> simplified = rdpSimplify(pts, eps);
  for (int guard = 0; guard < 4 && simplified.size() < keep; ++guard) {
    eps *= 0.1;  // 容差过大导致形状丢失时逐步收紧
    simplified = rdpSimplify(pts, eps);
  }
  if (simplified.size() < 2) simplified = pts;

  // 3. 细采样最大曲率：输出采样较稀，样条内部的曲率峰值可能落在采样点之间
  const auto fineMaxKappa = [](const CubicBSpline& s) {
    double u0 = 0.0, u1 = 1.0;
    splineDomain(s, &u0, &u1);
    double k = 0.0;
    for (std::size_t i = 0; i <= kFineCurvatureSamples; ++i) {
      const double u =
          u0 + (u1 - u0) * static_cast<double>(i) /
                   static_cast<double>(kFineCurvatureSamples);
      k = std::max(k, std::fabs(s.curvature(u)));
    }
    return k;
  };

  double best_kappa = std::numeric_limits<double>::max();
  Trajectory best_traj;
  std::vector<Vec2> best_ctrl;

  // 4. 拟合：先按控制点数递增重拟合（贴合度提升能消除抽稀引入的折角）
  const std::size_t max_ctrl = simplified.size();
  std::size_t ctrl_count = std::min<std::size_t>(max_ctrl, 8);
  for (int round = 0; round < 3; ++round) {
    const BSplineFitResult fit = fitCubicBSpline(simplified, ctrl_count, 3);
    if (fit.control_points.size() < 4) break;
    const CubicBSpline spline(fit.control_points, true);
    Trajectory traj = spline.sample(sample_count);
    const double k = std::max(maxCurvature(traj), fineMaxKappa(spline));
    if (k < best_kappa) {
      best_kappa = k;
      best_traj = traj;
      best_ctrl = fit.control_points;
    }
    if (k <= max_kappa) return traj;
    if (ctrl_count >= max_ctrl) break;
    ctrl_count = std::min(max_ctrl, ctrl_count + 3);
  }

  // 5. 仍超限：对控制多边形做拉普拉斯平滑。控制点向弦线收缩会单调降低
  //    曲率（极限情形退化为直线，曲率为 0），故按几何级数加强平滑强度
  //    必然收敛；轮数设上限以保证有界的时间开销。
  if (!best_ctrl.empty()) {
    const int kMaxRounds = 8;
    std::vector<Vec2> ctrl = best_ctrl;
    std::size_t sweeps = 1;
    for (int round = 0; round < kMaxRounds; ++round) {
      for (std::size_t i = 0; i < sweeps; ++i) laplacianSweep(ctrl, 0.5);
      sweeps *= 2;
      const CubicBSpline spline(ctrl, true);
      Trajectory traj = spline.sample(sample_count);
      const double k = std::max(maxCurvature(traj), fineMaxKappa(spline));
      if (k < best_kappa) {
        best_kappa = k;
        best_traj = traj;
        best_ctrl = ctrl;
      }
      if (k <= max_kappa) return traj;
    }
  }

  // 6. 兜底：拟合无法进行（如路点全部重合）时退化为原始折线轨迹
  if (best_traj.empty()) return pathToTrajectory(simplified, 0.0);
  return best_traj;
}

// ---------------------------------------------------------------------------
//  折线 → 轨迹
// ---------------------------------------------------------------------------

Trajectory pathToTrajectory(const std::vector<Vec2>& path,
                            double nominal_speed) {
  Trajectory traj;
  if (path.empty()) return traj;
  const std::size_t n = path.size();
  traj.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    traj[i].x = path[i].x;
    traj[i].y = path[i].y;
    traj[i].v = nominal_speed;
  }

  // 航向：取指向下一点的方向；末点沿用前一点，保证航向连续
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Vec2 d = segmentDirection(path, i, n - 1);
    traj[i].theta = d.squaredNorm() > 0.0 ? std::atan2(d.y, d.x) : 0.0;
  }
  if (n >= 2) traj[n - 1].theta = traj[n - 2].theta;

  // 曲率：相邻三点的 Menger 曲率 κ = 4*面积 / (|ab||bc||ca|) = 2|a×b| / (|a||b||a+b|)，
  // 符号对应转向（正为左转），端点沿用相邻值。
  // 注意系数 2：三点张成的三角形面积是 |a×b|/2，漏掉它会得到真值的一半。
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Vec2 a = path[i] - path[i - 1];
    const Vec2 b = path[i + 1] - path[i];
    const double la = a.norm();
    const double lb = b.norm();
    const double lc = (path[i + 1] - path[i - 1]).norm();
    if (la <= kDupEps || lb <= kDupEps || lc <= kDupEps) {
      traj[i].kappa = traj[i - 1].kappa;  // 重合点：无法定义三点圆
    } else {
      traj[i].kappa = 2.0 * (a.x * b.y - a.y * b.x) / (la * lb * lc);
    }
  }
  if (n >= 2) {
    traj[0].kappa = traj[1].kappa;
    traj[n - 1].kappa = traj[n - 2].kappa;
  }

  // 时间戳：按标称速度把累计弧长折算成时间；速度为 0 时保持 0
  if (nominal_speed > 0.0) {
    double s = 0.0;
    for (std::size_t i = 1; i < n; ++i) {
      s += (path[i] - path[i - 1]).norm();
      traj[i].t = s / nominal_speed;
    }
  }
  return traj;
}

double maxCurvature(const Trajectory& trajectory) {
  double k = 0.0;
  for (const TrajectoryPoint& p : trajectory) {
    k = std::max(k, std::fabs(p.kappa));
  }
  return k;
}

}  // namespace geometry
}  // namespace adsim
