// =============================================================================
//  World.h — 仿真世界：车道网络、静态障碍与动态物体
//
//  用轻量的二维车道图描述道路，替代 CARLA 的完整三维场景。这样做的取舍是：
//  行为决策与路径规划所需的全部几何信息（车道拓扑、曲率、限速、可行驶区域）
//  都能精确表达，而仿真吞吐量提升数个量级，使得大批量场景回归成为可能。
//
//  接入 CARLA 时（ADSIM_WITH_CARLA=ON），CarlaBridge 会把 CARLA 的 OpenDRIVE
//  地图转换为本结构，从而让同一套决策/规划代码在真实引擎上运行。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace adsim {

/// 一条车道中心线及其拓扑关系
struct Lane {
  int id{-1};
  std::vector<Vec2> centerline;   ///< 中心线离散点，按行驶方向排列
  double width{3.5};              ///< 车道宽度 (m)
  double speed_limit{13.9};       ///< 限速 (m/s)
  int left_lane_id{-1};           ///< 左侧相邻车道，-1 表示无
  int right_lane_id{-1};          ///< 右侧相邻车道，-1 表示无
  int predecessor_id{-1};         ///< 前驱车道
  int successor_id{-1};           ///< 后继车道
  bool is_junction{false};        ///< 是否为路口内部车道

  /// 车道中心线总长度
  double length() const;
};

/// 动态物体（其他车辆、行人等）
struct RoadObject {
  enum class Type { kVehicle, kPedestrian, kBicycle, kStatic };

  int id{0};
  Type type{Type::kVehicle};
  Pose2 pose;
  Vec2 velocity;              ///< 世界坐标系下的速度 (m/s)
  double length{4.5};
  double width{1.8};
  double speed{0.0};          ///< 沿航向的标量速度

  Obb2 obb() const { return Obb2(pose, length, width); }
  Vec2 position() const { return pose.position(); }
  double heading() const { return pose.theta; }

  /// 是否与另一物体发生碰撞（OBB 相交检测）
  bool collidesWith(const RoadObject& other) const;
};

/// 车道投影结果
struct LaneProjection {
  int lane_id{-1};
  double s{0.0};          ///< 沿中心线的弧长坐标
  double lateral{0.0};    ///< 横向偏移，左正右负
  double heading{0.0};    ///< 该处中心线航向
  double curvature{0.0};  ///< 该处中心线曲率
  bool valid{false};
};

class World {
 public:
  World() = default;

  // -------------------------------------------------------------------------
  // 车道
  // -------------------------------------------------------------------------

  void addLane(const Lane& lane);
  const std::vector<Lane>& lanes() const { return lanes_; }
  const Lane* findLane(int id) const;
  std::size_t laneCount() const { return lanes_.size(); }

  /// 将世界坐标投影到最近车道，返回弧长与横向偏移
  LaneProjection project(const Vec2& point) const;

  /// 将位姿投影到指定车道
  LaneProjection projectOnLane(int lane_id, const Vec2& point) const;

  /// 取车道中心线上弧长 s 处的点
  Vec2 pointAt(int lane_id, double s) const;

  /// 取车道中心线上弧长 s 处的位姿（含航向）
  Pose2 poseAt(int lane_id, double s) const;

  /// 计算某点在车道坐标系下的期望位姿（用于横向控制）
  bool laneFrame(int lane_id, const Vec2& point, double& s, double& lateral,
                 double& heading_error) const;

  // -------------------------------------------------------------------------
  // 障碍物与动态物体
  // -------------------------------------------------------------------------

  void addObstacle(const Obb2& obstacle) { obstacles_.push_back(obstacle); }
  const std::vector<Obb2>& obstacles() const { return obstacles_; }

  std::vector<RoadObject>& objects() { return objects_; }
  const std::vector<RoadObject>& objects() const { return objects_; }

  /// 查询以 center 为中心、radius 为半径范围内的动态物体下标
  std::vector<std::size_t> queryObjects(const Vec2& center, double radius) const;

  /// 查询范围内的静态障碍
  std::vector<std::size_t> queryObstacles(const Vec2& center, double radius) const;

  /** 可行驶区域边界检查：点是否位于任意车道内（含车道宽度） */
  bool isOnRoad(const Vec2& point) const;

  /// 距离该点最近车道的横向距离（超出车道宽度时为正）
  double distanceToRoadEdge(const Vec2& point) const;

  // -------------------------------------------------------------------------
  // 场景构造辅助
  // -------------------------------------------------------------------------

  /// 生成一条直路，返回各车道 id（自左向右）
  static World straightRoad(int lane_count, double lane_width, double length,
                            double speed_limit = 13.9);

  /// 生成一条带弯道的道路
  static World curvedRoad(int lane_count, double lane_width, double length,
                          double curvature, double speed_limit = 13.9);

  /// 生成一个十字路口（四条直路，中部为路口车道）
  static World intersection(double arm_length, double lane_width);

  BoundingBox2 bounds() const;

 private:
  std::vector<Lane> lanes_;
  std::vector<Obb2> obstacles_;
  std::vector<RoadObject> objects_;
};

/// 计算折线各点的累计弧长，长度与输入相同，首元素为 0
std::vector<double> laneArcLength(const std::vector<Vec2>& centerline);

/// 在折线上按弧长插值取点
Vec2 interpolateAlong(const std::vector<Vec2>& polyline, const std::vector<double>& arc,
                      double s);

/// 在折线上按弧长插值取航向
double headingAlong(const std::vector<Vec2>& polyline, const std::vector<double>& arc,
                    double s);

}  // namespace adsim
