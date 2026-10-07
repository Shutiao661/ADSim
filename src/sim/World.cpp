// =============================================================================
//  World.cpp — 车道网络与动态物体的几何运算实现
//
//  车道中心线用**离散折线**表示：所有几何查询（投影、插值、曲率估计）都归结为
//  "折线段上的最近点"问题。相比解析曲线（样条/圆弧），折线的好处是
//  查询代价恒定、不会出现数值退化，代价是曲率只能近似估计——对 1m 级采样
//  间隔，离散曲率与真实曲率的相对误差在 1e-3 以内，足够规划模块使用。
// =============================================================================
#include "adsim/sim/World.h"

#include "adsim/sim/SimEngine.h"  // obbIntersect：RoadObject::collidesWith 复用 SAT 实现

#include <algorithm>
#include <cmath>
#include <limits>

namespace adsim {

namespace {

/// 折线采样间隔：1m 是"曲率估计精度"与"内存/查询开销"之间的折中。
constexpr double kSampleSpacing = 1.0;

/// 直行判定阈值（曲率估计用）
constexpr double kCurvEps = 1e-9;

/// 点到线段的最近点参数 t ∈ [0,1]；线段退化为点时返回 0
double projectParamOnSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < kEpsilon) return 0.0;
  return clamp((p - a).dot(ab) / len2, 0.0, 1.0);
}

/// 点到线段的最短距离
double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
  const double t = projectParamOnSegment(p, a, b);
  return (p - (a + (b - a) * t)).norm();
}

/// 点到折线的最短距离（对每段取点到线段距离的最小值）
double pointToPolylineDistance(const std::vector<Vec2>& polyline, const Vec2& p) {
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 1; i < polyline.size(); ++i) {
    best = std::min(best, pointSegmentDistance(p, polyline[i - 1], polyline[i]));
  }
  return best;
}

/// 点到有向包围盒的最短距离（内部为 0）。
/// 凸多边形间的距离必在"顶点—边"对上取到，因此对四条边取点到线段距离即可。
double pointToObbDistance(const Obb2& box, const Vec2& p) {
  if (obbContains(box, p)) return 0.0;
  const std::vector<Vec2> c = box.corners();
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < c.size(); ++i) {
    best = std::min(best, pointSegmentDistance(p, c[i], c[(i + 1) % c.size()]));
  }
  return best;
}

/// 找到 s 所在的折线段下标 i ∈ [1, n-1]，段 i 连接点 i-1 与点 i
std::size_t segmentIndexForS(const std::vector<double>& arc, double s) {
  const std::size_t n = arc.size();
  if (n < 2) return 0;
  const double sc = clamp(s, 0.0, arc.back());
  const auto it = std::upper_bound(arc.begin(), arc.end(), sc);
  std::size_t i = static_cast<std::size_t>(it - arc.begin());
  if (i == 0) i = 1;
  if (i >= n) i = n - 1;
  return i;
}

/// 段方向（未归一化）；退化段返回零向量
Vec2 segmentDirection(const std::vector<Vec2>& polyline, std::size_t i) {
  if (i == 0 || i >= polyline.size()) return Vec2{0.0, 0.0};
  return polyline[i] - polyline[i - 1];
}

/// 离散曲率估计：用相邻两段的方向角变化除以两段的平均长度。
/// 对按弧长均匀采样的圆弧，该估计是渐近精确的（误差 O(ds²)）。
double curvatureAtSegment(const std::vector<Vec2>& polyline, const std::vector<double>& arc,
                          std::size_t seg) {
  const std::size_t n = polyline.size();
  if (n < 3 || arc.size() != n || seg == 0 || seg >= n) return 0.0;

  // 优先用 (seg, seg+1) 两段；seg 已是最后一段时退化为 (seg-1, seg)
  std::size_t i1 = seg;
  std::size_t i2 = seg + 1;
  if (i2 >= n) {
    if (seg < 2) return 0.0;
    i1 = seg - 1;
    i2 = seg;
  }

  const Vec2 d1 = segmentDirection(polyline, i1);
  const Vec2 d2 = segmentDirection(polyline, i2);
  if (d1.squaredNorm() < kEpsilon || d2.squaredNorm() < kEpsilon) return 0.0;

  const double len1 = arc[i1] - arc[i1 - 1];
  const double len2 = arc[i2] - arc[i2 - 1];
  const double ds = 0.5 * (len1 + len2);
  if (ds < kCurvEps) return 0.0;

  const double dtheta = normalizeAngle(d2.heading() - d1.heading());
  return dtheta / ds;
}

/// 在单条车道上做投影（World::project / projectOnLane / laneFrame 共用）
LaneProjection projectOnLaneGeometry(const Lane& lane, const Vec2& point) {
  LaneProjection result;
  const std::vector<Vec2>& line = lane.centerline;
  if (line.size() < 2) return result;

  const std::vector<double> arc = laneArcLength(line);
  double best_dist2 = std::numeric_limits<double>::max();
  std::size_t best_seg = 1;
  double best_t = 0.0;

  for (std::size_t i = 1; i < line.size(); ++i) {
    const Vec2 a = line[i - 1];
    const Vec2 b = line[i];
    const double t = projectParamOnSegment(point, a, b);
    const Vec2 foot = a + (b - a) * t;
    const double dist2 = (point - foot).squaredNorm();
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_seg = i;
      best_t = t;
    }
  }

  const Vec2 a = line[best_seg - 1];
  const Vec2 b = line[best_seg];
  const Vec2 seg = b - a;
  const double seg_len = seg.norm();
  if (seg_len < kEpsilon) return result;  // 全零折线：无法定义航向

  const Vec2 tangent = seg / seg_len;
  const Vec2 foot = a + seg * best_t;
  const Vec2 offset = point - foot;

  result.lane_id = lane.id;
  result.s = arc[best_seg - 1] + seg_len * best_t;
  // 横向偏移：左正右负。沿行驶方向左侧即切向量逆时针 90°，
  // 而 cross(tangent, offset) = |offset|·sin(从 tangent 转到 offset 的角)，
  // 恰好把"左侧为正"编码进来。
  result.lateral = tangent.cross(offset);
  result.heading = std::atan2(tangent.y, tangent.x);
  result.curvature = curvatureAtSegment(line, arc, best_seg);
  result.valid = true;
  return result;
}

/// 生成与直路/弯道共用的车道拓扑（相邻车道 + 前驱后继）
void linkAdjacentLanes(std::vector<Lane>& lanes) {
  const int n = static_cast<int>(lanes.size());
  for (int i = 0; i < n; ++i) {
    // 自左向右编号 ⇒ 编号小的一侧在左（+y 方向）
    lanes[static_cast<std::size_t>(i)].left_lane_id = (i > 0) ? i - 1 : -1;
    lanes[static_cast<std::size_t>(i)].right_lane_id = (i + 1 < n) ? i + 1 : -1;
  }
}

/// 采样点数：至少 2 个（直线也需要两个端点），约每米一个点
int sampleCount(double length) {
  const int count = static_cast<int>(std::ceil(length / kSampleSpacing)) + 1;
  return std::max(2, count);
}

}  // namespace

// ---------------------------------------------------------------------------
// Lane
// ---------------------------------------------------------------------------

double Lane::length() const {
  const std::vector<double> arc = laneArcLength(centerline);
  return arc.empty() ? 0.0 : arc.back();
}

// ---------------------------------------------------------------------------
// 折线工具
// ---------------------------------------------------------------------------

std::vector<double> laneArcLength(const std::vector<Vec2>& centerline) {
  std::vector<double> arc(centerline.size(), 0.0);
  for (std::size_t i = 1; i < centerline.size(); ++i) {
    arc[i] = arc[i - 1] + (centerline[i] - centerline[i - 1]).norm();
  }
  return arc;
}

Vec2 interpolateAlong(const std::vector<Vec2>& polyline, const std::vector<double>& arc,
                      double s) {
  if (polyline.empty()) return Vec2{};
  if (polyline.size() < 2 || arc.size() != polyline.size()) return polyline.front();

  const double total = arc.back();
  if (total < kEpsilon) return polyline.front();  // 所有点重合，整条线退化为一点

  // s 超出 [0, total] 时裁剪到端点（不报错：调用方多为按弧长步进的控制器，
  // 端点裁剪比抛异常更符合"车开到路尽头"的语义）
  const double sc = clamp(s, 0.0, total);
  const std::size_t i = segmentIndexForS(arc, sc);
  const double seg_len = arc[i] - arc[i - 1];
  const double t = seg_len < kEpsilon ? 0.0 : (sc - arc[i - 1]) / seg_len;
  return polyline[i - 1] + (polyline[i] - polyline[i - 1]) * t;
}

double headingAlong(const std::vector<Vec2>& polyline, const std::vector<double>& arc,
                    double s) {
  if (polyline.size() < 2 || arc.size() != polyline.size()) return 0.0;

  const double total = arc.back();
  const double sc = clamp(s, 0.0, total);
  std::size_t i = segmentIndexForS(arc, sc);

  // 退化段（零长度）不携带方向信息，向后找最近的非退化段
  for (std::size_t k = i; k < polyline.size(); ++k) {
    const Vec2 d = segmentDirection(polyline, k);
    if (d.squaredNorm() >= kEpsilon) return std::atan2(d.y, d.x);
  }
  for (std::size_t k = i; k > 1; --k) {
    const Vec2 d = segmentDirection(polyline, k - 1);
    if (d.squaredNorm() >= kEpsilon) return std::atan2(d.y, d.x);
  }
  return 0.0;
}

// ---------------------------------------------------------------------------
// RoadObject
// ---------------------------------------------------------------------------

bool RoadObject::collidesWith(const RoadObject& other) const {
  return obbIntersect(obb(), other.obb());
}

// ---------------------------------------------------------------------------
// 车道管理
// ---------------------------------------------------------------------------

void World::addLane(const Lane& lane) { lanes_.push_back(lane); }

const Lane* World::findLane(int id) const {
  for (const Lane& lane : lanes_) {
    if (lane.id == id) return &lane;
  }
  return nullptr;
}

LaneProjection World::project(const Vec2& point) const {
  LaneProjection best;
  double best_dist = std::numeric_limits<double>::max();

  for (const Lane& lane : lanes_) {
    const LaneProjection p = projectOnLaneGeometry(lane, point);
    if (!p.valid) continue;
    // 用"点到该车道中心线折线的距离"来挑最近车道，而不是用投影给出的横向偏移：
    // s 被裁剪到 [0, length] 之后，纵向越过车道端点的点其横向偏移会退化为 0，
    // 于是"已经开出这条车道"的点仍会被判为落在该车道上——典型症状是在路口
    // 东侧却被投影到西侧进口道（s 卡在末端），限速、可行驶区域判断随之出错。
    // 折线距离对纵向超出同样敏感，不存在这个退化。
    const double d = pointToPolylineDistance(lane.centerline, point);
    if (d < best_dist) {
      best_dist = d;
      best = p;
    }
  }
  return best;
}

LaneProjection World::projectOnLane(int lane_id, const Vec2& point) const {
  const Lane* lane = findLane(lane_id);
  if (lane == nullptr) return LaneProjection{};
  return projectOnLaneGeometry(*lane, point);
}

Vec2 World::pointAt(int lane_id, double s) const {
  const Lane* lane = findLane(lane_id);
  if (lane == nullptr) return Vec2{};
  return interpolateAlong(lane->centerline, laneArcLength(lane->centerline), s);
}

Pose2 World::poseAt(int lane_id, double s) const {
  const Lane* lane = findLane(lane_id);
  if (lane == nullptr) return Pose2{};
  const std::vector<double> arc = laneArcLength(lane->centerline);
  const Vec2 point = interpolateAlong(lane->centerline, arc, s);
  return Pose2(point.x, point.y, headingAlong(lane->centerline, arc, s));
}

bool World::laneFrame(int lane_id, const Vec2& point, double& s, double& lateral,
                      double& heading_error) const {
  const Lane* lane = findLane(lane_id);
  if (lane == nullptr) return false;
  const LaneProjection p = projectOnLaneGeometry(*lane, point);
  if (!p.valid) return false;

  s = p.s;
  lateral = p.lateral;

  // heading_error 的语义说明：
  // 本接口只拿到"一个点"，没有车辆航向——车辆航向无从得知，因此不可能给出
  // 教科书意义上的 θ_lane - θ_vehicle。这里取横向控制器真正可用的量：
  // 以中心线上 s+Ld 处的点为前视点，返回"前视连线方向"与"该处中心线切向"的
  // 夹角 α。α 对位于中心线上的点恒为 0；点偏左时 α<0（提示右打方向），
  // 与纯跟踪控制器的转向修正方向一致。
  const double lookahead = 5.0;
  const std::vector<double> arc = laneArcLength(lane->centerline);
  const Vec2 target = interpolateAlong(lane->centerline, arc, p.s + lookahead);
  const Vec2 dir = target - point;
  if (dir.squaredNorm() < kEpsilon) {
    heading_error = 0.0;
    return true;
  }
  heading_error =
      normalizeAngle(std::atan2(dir.y, dir.x) - headingAlong(lane->centerline, arc, p.s));
  return true;
}

// ---------------------------------------------------------------------------
// 物体查询
// ---------------------------------------------------------------------------

std::vector<std::size_t> World::queryObjects(const Vec2& center, double radius) const {
  std::vector<std::pair<double, std::size_t>> hits;
  hits.reserve(objects_.size());
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    // 用 OBB 间距而非中心距：长车（卡车/公交）即使中心较远，车身也可能已进入范围
    const double d = pointToObbDistance(objects_[i].obb(), center);
    if (d <= radius) hits.emplace_back(d, i);
  }
  std::sort(hits.begin(), hits.end(), [](const std::pair<double, std::size_t>& a,
                                         const std::pair<double, std::size_t>& b) {
    return a.first == b.first ? a.second < b.second : a.first < b.first;
  });

  std::vector<std::size_t> result;
  result.reserve(hits.size());
  for (const auto& h : hits) result.push_back(h.second);
  return result;
}

std::vector<std::size_t> World::queryObstacles(const Vec2& center, double radius) const {
  std::vector<std::pair<double, std::size_t>> hits;
  hits.reserve(obstacles_.size());
  for (std::size_t i = 0; i < obstacles_.size(); ++i) {
    const double d = pointToObbDistance(obstacles_[i], center);
    if (d <= radius) hits.emplace_back(d, i);
  }
  std::sort(hits.begin(), hits.end(), [](const std::pair<double, std::size_t>& a,
                                         const std::pair<double, std::size_t>& b) {
    return a.first == b.first ? a.second < b.second : a.first < b.first;
  });

  std::vector<std::size_t> result;
  result.reserve(hits.size());
  for (const auto& h : hits) result.push_back(h.second);
  return result;
}

// ---------------------------------------------------------------------------
// 可行驶区域
// ---------------------------------------------------------------------------

bool World::isOnRoad(const Vec2& point) const {
  const LaneProjection p = project(point);
  if (!p.valid) return false;
  const Lane* lane = findLane(p.lane_id);
  if (lane == nullptr) return false;
  // 判据：到该车道中心线（折线）的距离不超过半个车道宽。
  // 这里必须用"到折线的距离"而不是投影给出的 |lateral|：s 被裁剪到
  // [0, length] 之后，越过道路端点的点其横向偏移会退化为 0，
  // 只看 lateral 会把"冲出路面端点后继续沿中线行驶"误判为仍在路内。
  // 容差 1e-9：恰好落在车道边界上的点算在路面内。
  return pointToPolylineDistance(lane->centerline, point) <= lane->width * 0.5 + 1e-9;
}

double World::distanceToRoadEdge(const Vec2& point) const {
  const LaneProjection p = project(point);
  if (!p.valid) return std::numeric_limits<double>::max();
  const Lane* lane = findLane(p.lane_id);
  if (lane == nullptr) return std::numeric_limits<double>::max();
  // 正值表示在路面之外（含"离路边多远"与"压过道路端点多远"两种情形）
  return pointToPolylineDistance(lane->centerline, point) - lane->width * 0.5;
}

BoundingBox2 World::bounds() const {
  BoundingBox2 box;
  for (const Lane& lane : lanes_) {
    const double half = lane.width * 0.5;
    for (const Vec2& p : lane.centerline) {
      box.expand({p.x - half, p.y - half});
      box.expand({p.x + half, p.y + half});
    }
  }
  return box;
}

// ---------------------------------------------------------------------------
// 场景构造辅助
// ---------------------------------------------------------------------------

World World::straightRoad(int lane_count, double lane_width, double length,
                          double speed_limit) {
  World world;
  if (lane_count <= 0 || !(lane_width > 0.0) || !(length > 0.0)) return world;

  const int samples = sampleCount(length);
  std::vector<Lane> lanes;
  lanes.reserve(static_cast<std::size_t>(lane_count));
  for (int i = 0; i < lane_count; ++i) {
    Lane lane;
    lane.id = i;
    lane.width = lane_width;
    lane.speed_limit = speed_limit;
    // 车道自左向右编号 0..n-1；车辆沿 +x 行驶时左侧为 +y，
    // 因此 0 号车道 y 最大，整条路关于 y=0 对称
    const double y = (static_cast<double>(lane_count - 1) * 0.5 - static_cast<double>(i)) *
                     lane_width;
    lane.centerline.reserve(static_cast<std::size_t>(samples));
    for (int k = 0; k < samples; ++k) {
      const double s = length * static_cast<double>(k) / static_cast<double>(samples - 1);
      lane.centerline.push_back({s, y});
    }
    lanes.push_back(std::move(lane));
  }

  linkAdjacentLanes(lanes);
  for (const Lane& lane : lanes) world.addLane(lane);
  return world;
}

World World::curvedRoad(int lane_count, double lane_width, double length, double curvature,
                        double speed_limit) {
  World world;
  if (lane_count <= 0 || !(lane_width > 0.0) || !(length > 0.0)) return world;

  const bool straight = std::fabs(curvature) < 1e-9;
  const int samples = sampleCount(length);

  std::vector<Lane> lanes;
  lanes.reserve(static_cast<std::size_t>(lane_count));
  for (int i = 0; i < lane_count; ++i) {
    Lane lane;
    lane.id = i;
    lane.width = lane_width;
    lane.speed_limit = speed_limit;
    // 与直路一致：0 号车道在左（+y 侧）
    const double offset =
        (static_cast<double>(lane_count - 1) * 0.5 - static_cast<double>(i)) * lane_width;

    lane.centerline.reserve(static_cast<std::size_t>(samples));
    for (int k = 0; k < samples; ++k) {
      const double s = length * static_cast<double>(k) / static_cast<double>(samples - 1);
      Vec2 base;
      double theta = 0.0;
      if (straight) {
        base = Vec2{s, 0.0};
      } else {
        // 圆弧参数方程（与 VehicleModel::advanceOnArc 同一套约定，κ>0 向左转）
        theta = curvature * s;
        base = Vec2{std::sin(theta) / curvature, (1.0 - std::cos(theta)) / curvature};
      }
      // 沿左法向平移得到平行曲线：圆弧的等距偏移仍是圆弧，
      // 因此每条车道中心线的曲率严格等于 κ/(1-κ·offset)
      const Vec2 left_normal{-std::sin(theta), std::cos(theta)};
      lane.centerline.push_back(base + left_normal * offset);
    }
    lanes.push_back(std::move(lane));
  }

  linkAdjacentLanes(lanes);
  for (const Lane& lane : lanes) world.addLane(lane);
  return world;
}

World World::intersection(double arm_length, double lane_width) {
  World world;
  if (!(arm_length > 0.0) || !(lane_width > 0.0)) return world;

  const double w = lane_width;      // 单条车道宽
  const double h = lane_width * 0.5;  // 车道中心线到道路中线的距离
  const double L = arm_length;

  // 每条直路取两条反向车道（右行交通）：路口方块为 [-w, w] × [-w, w]，
  // 每条路段的车道中心线终止于 |坐标| = w 处，路口内部由 is_junction 车道贯通。
  struct LaneSpec {
    double x0, y0, x1, y1;
    bool junction;
    int left, right, predecessor, successor;
    double limit;
  };
  const double arm_limit = 13.9;
  const double junction_limit = 8.3;  // 路口内限速更低，符合实际交通设计

  const LaneSpec specs[] = {
      // 西侧路段（自西向东行驶）
      {-w - L, -h, -w, -h, false, 5, -1, -1, 1, arm_limit},
      // 路口内自西向东
      {-w, -h, w, -h, true, 4, -1, 0, 2, junction_limit},
      // 东侧路段（自西向东）
      {w, -h, w + L, -h, false, 3, -1, 1, -1, arm_limit},
      // 东侧路段（自东向西）
      {w + L, h, w, h, false, 2, -1, -1, 4, arm_limit},
      // 路口内自东向西
      {w, h, -w, h, true, 1, -1, 3, 5, junction_limit},
      // 西侧路段（自东向西）
      {-w, h, -w - L, h, false, 0, -1, 4, -1, arm_limit},
      // 南侧路段（自南向北）
      {h, -w - L, h, -w, false, 11, -1, -1, 7, arm_limit},
      // 路口内自南向北
      {h, -w, h, w, true, 10, -1, 6, 8, junction_limit},
      // 北侧路段（自南向北）
      {h, w, h, w + L, false, 9, -1, 7, -1, arm_limit},
      // 北侧路段（自北向南）
      {-h, w + L, -h, w, false, 8, -1, -1, 10, arm_limit},
      // 路口内自北向南
      {-h, w, -h, -w, true, 7, -1, 9, 11, junction_limit},
      // 南侧路段（自北向南）
      {-h, -w, -h, -w - L, false, 6, -1, 10, -1, arm_limit},
  };

  int id = 0;
  for (const LaneSpec& spec : specs) {
    Lane lane;
    lane.id = id++;
    lane.width = w;
    lane.speed_limit = spec.limit;
    lane.left_lane_id = spec.left;
    lane.right_lane_id = spec.right;
    lane.predecessor_id = spec.predecessor;
    lane.successor_id = spec.successor;
    lane.is_junction = spec.junction;
    // 直路段只需两端点；多插几个点是为了让投影的弧长分辨率与其它场景一致
    const double seg_len = (Vec2{spec.x1, spec.y1} - Vec2{spec.x0, spec.y0}).norm();
    const int samples = sampleCount(seg_len);
    for (int k = 0; k < samples; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(samples - 1);
      lane.centerline.push_back({spec.x0 + (spec.x1 - spec.x0) * t,
                                 spec.y0 + (spec.y1 - spec.y0) * t});
    }
    world.addLane(lane);
  }
  return world;
}

}  // namespace adsim
