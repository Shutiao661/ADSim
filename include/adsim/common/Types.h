// =============================================================================
//  Types.h — 平台基础数据类型
//
//  统一全平台的几何 / 时间 / 轨迹表示，避免各模块间重复定义与隐式转换。
//  时间统一采用 ROS 风格的纳秒时间戳，便于与 rosbag 直接对接。
// =============================================================================
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace adsim {

// ---------------------------------------------------------------------------
// 时间
// ---------------------------------------------------------------------------

/// 纳秒时间戳（Unix epoch 起算），与 rosbag 的 ros::Time 表示一致
using Timestamp = std::int64_t;

constexpr Timestamp kInvalidTimestamp = std::numeric_limits<Timestamp>::min();
constexpr double kNanoToSec = 1e-9;
constexpr double kSecToNano = 1e9;

inline double toSeconds(Timestamp t) { return static_cast<double>(t) * kNanoToSec; }
inline Timestamp fromSeconds(double s) { return static_cast<Timestamp>(s * kSecToNano); }

// ---------------------------------------------------------------------------
// 常量
// ---------------------------------------------------------------------------

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

/// 归一化角度到 (-pi, pi]
inline double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

inline double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// 几何
// ---------------------------------------------------------------------------

struct Vec2 {
  double x{0.0};
  double y{0.0};

  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
  Vec2 operator/(double s) const { return {x / s, y / s}; }
  Vec2 operator-() const { return {-x, -y}; }

  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

  double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  /// 二维叉积的标量结果（z 分量）
  double cross(const Vec2& o) const { return x * o.y - y * o.x; }

  double norm() const { return std::sqrt(x * x + y * y); }
  double squaredNorm() const { return x * x + y * y; }
  double heading() const { return std::atan2(y, x); }

  Vec2 normalized() const {
    const double n = norm();
    return n < kEpsilon ? Vec2{0.0, 0.0} : Vec2{x / n, y / n};
  }

  /// 逆时针旋转 90°，用于快速求法向量
  Vec2 perp() const { return {-y, x}; }

  Vec2 rotated(double theta) const {
    const double c = std::cos(theta), s = std::sin(theta);
    return {x * c - y * s, x * s + y * c};
  }
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec2 xy() const { return {x, y}; }
};

/// 二维位姿
struct Pose2 {
  double x{0.0};
  double y{0.0};
  double theta{0.0};

  Pose2() = default;
  Pose2(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

  Vec2 position() const { return {x, y}; }

  /// 将全局坐标点转换到本车位姿坐标系
  Vec2 toLocal(const Vec2& global) const {
    const Vec2 d = global - position();
    const double c = std::cos(theta), s = std::sin(theta);
    return {d.x * c + d.y * s, -d.x * s + d.y * c};
  }

  /// 将本车坐标系下的点转换到全局坐标
  Vec2 toGlobal(const Vec2& local) const {
    const double c = std::cos(theta), s = std::sin(theta);
    return {x + local.x * c - local.y * s, y + local.x * s + local.y * c};
  }
};

/// 轴对齐包围盒
struct BoundingBox2 {
  Vec2 min{std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  Vec2 max{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};

  bool valid() const { return min.x <= max.x && min.y <= max.y; }

  void expand(const Vec2& p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
  }

  bool contains(const Vec2& p) const {
    return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
  }

  bool overlaps(const BoundingBox2& o) const {
    return !(o.min.x > max.x || o.max.x < min.x || o.min.y > max.y || o.max.y < min.y);
  }

  Vec2 center() const { return {(min.x + max.x) * 0.5, (min.y + max.y) * 0.5}; }
};

/// 有向包围盒（车辆轮廓的标准表示）
struct Obb2 {
  Pose2 pose;
  double length{4.5};   ///< 车长 (m)
  double width{1.8};    ///< 车宽 (m)

  Obb2() = default;
  Obb2(const Pose2& p, double l, double w) : pose(p), length(l), width(w) {}

  /// 四个角点，顺序为 左后 → 右后 → 右前 → 左前
  std::vector<Vec2> corners() const {
    const double hl = length * 0.5, hw = width * 0.5;
    return {pose.toGlobal({-hl, -hw}), pose.toGlobal({-hl, hw}),
            pose.toGlobal({hl, hw}),   pose.toGlobal({hl, -hw})};
  }

  BoundingBox2 aabb() const {
    BoundingBox2 box;
    for (const Vec2& c : corners()) box.expand(c);
    return box;
  }
};

// ---------------------------------------------------------------------------
// 轨迹
// ---------------------------------------------------------------------------

/// 轨迹点：位置 + 航向 + 曲率 + 速度 + 加速度 + 相对时间
struct TrajectoryPoint {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double kappa{0.0};   ///< 曲率 (1/m)
  double v{0.0};       ///< 速度 (m/s)
  double a{0.0};       ///< 加速度 (m/s^2)
  double t{0.0};       ///< 相对轨迹起点的时间 (s)

  Pose2 pose() const { return {x, y, theta}; }
  Vec2 position() const { return {x, y}; }
};

using Trajectory = std::vector<TrajectoryPoint>;

// ---------------------------------------------------------------------------
// 传感器数据（与 rosbag 中的消息类型对应）
// ---------------------------------------------------------------------------

/// 单点激光雷达数据
struct LidarPoint {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
};

/// 一帧激光雷达扫描
struct LidarFrame {
  Timestamp stamp{kInvalidTimestamp};
  std::string frame_id{"lidar"};
  std::vector<LidarPoint> points;

  BoundingBox2 aabbXY() const {
    BoundingBox2 box;
    for (const LidarPoint& p : points) box.expand({p.x, p.y});
    return box;
  }
};

/// 全球定位 / 惯性导航数据
struct GpsFrame {
  Timestamp stamp{kInvalidTimestamp};
  double latitude{0.0};    ///< 度
  double longitude{0.0};   ///< 度
  double altitude{0.0};    ///< m
  double heading{0.0};     ///< rad，正北为 0，顺时针为正
  double speed{0.0};       ///< m/s
  int fix_type{0};         ///< 0=无效 1=单点 2=差分 3=RTK固定
  double hdop{99.9};       ///< 水平精度因子
};

/// 车辆底盘 / 状态反馈
struct VehicleState {
  Timestamp stamp{kInvalidTimestamp};
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double speed{0.0};        ///< m/s
  double acceleration{0.0}; ///< m/s^2
  double steering{0.0};     ///< rad，前轮转角
  double yaw_rate{0.0};     ///< rad/s
  double throttle{0.0};     ///< [0,1]
  double brake{0.0};        ///< [0,1]
  int gear{0};              ///< 0=N 1=D -1=R

  Pose2 pose() const { return {x, y, theta}; }
};

// ---------------------------------------------------------------------------
// 坐标系转换工具
// ---------------------------------------------------------------------------

/// WGS84 经纬高 → 局部 ENU 平面坐标（以给定原点为参考）
/// 采用等距圆柱投影，在 <10km 范围内误差可忽略，满足路测场景需求
struct GeoProjector {
  double origin_lat{0.0};
  double origin_lon{0.0};
  double origin_alt{0.0};

  static constexpr double kEarthRadius = 6378137.0;  ///< WGS84 长半轴 (m)

  GeoProjector() = default;
  GeoProjector(double lat, double lon, double alt)
      : origin_lat(lat), origin_lon(lon), origin_alt(alt) {}

  Vec3 toLocal(double lat, double lon, double alt) const {
    const double lat_rad = deg2rad(lat);
    const double lon_rad = deg2rad(lon);
    const double origin_lat_rad = deg2rad(origin_lat);
    const double origin_lon_rad = deg2rad(origin_lon);

    const double x = (lon_rad - origin_lon_rad) * std::cos(origin_lat_rad) * kEarthRadius;
    const double y = (lat_rad - origin_lat_rad) * kEarthRadius;
    return {x, y, alt - origin_alt};
  }

  void toGeodetic(const Vec3& local, double& lat, double& lon, double& alt) const {
    const double origin_lat_rad = deg2rad(origin_lat);
    lat = rad2deg(deg2rad(origin_lat) + local.y / kEarthRadius);
    lon = rad2deg(deg2rad(origin_lon) + local.x / (kEarthRadius * std::cos(origin_lat_rad)));
    alt = local.z + origin_alt;
  }
};

// ---------------------------------------------------------------------------
// 统计辅助
// ---------------------------------------------------------------------------

struct Statistics {
  double min{0.0};
  double max{0.0};
  double mean{0.0};
  double stddev{0.0};
  std::size_t count{0};

  std::string toString() const;
};

/// 计算一组数据的统计量
Statistics computeStatistics(const std::vector<double>& values);

}  // namespace adsim
