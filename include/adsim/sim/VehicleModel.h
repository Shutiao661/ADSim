// =============================================================================
//  VehicleModel.h — 车辆运动学模型
//
//  采用运动学自行车模型：忽略轮胎侧偏力，直接以几何关系描述车辆运动。
//  在低速城市工况（<15 m/s）下与动力学模型误差很小，却能显著降低仿真与
//  规划的计算量——路径规划所需的曲率-转角映射正是由该模型直接给出。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <string>

namespace adsim {

/// 车辆物理与执行器约束
struct VehicleParams {
  double wheelbase{2.7};            ///< 轴距 (m)
  double front_overhang{0.9};       ///< 前悬 (m)
  double rear_overhang{1.0};        ///< 后悬 (m)
  double width{1.8};                ///< 车宽 (m)

  double max_steering{0.6};         ///< 前轮最大转角 (rad)，约 34°
  double max_steering_rate{0.5};    ///< 转角速率上限 (rad/s)
  double max_speed{30.0};           ///< m/s
  double min_speed{-2.0};           ///< 允许倒车
  double max_acceleration{3.0};     ///< m/s²
  double max_deceleration{-6.0};    ///< m/s²（常规制动，不含紧急制动）
  double max_jerk{5.0};             ///< m/s³

  double length() const { return front_overhang + wheelbase + rear_overhang; }
};

class VehicleModel {
 public:
  VehicleModel();
  explicit VehicleModel(const VehicleParams& params);

  const VehicleParams& params() const { return params_; }

  // -------------------------------------------------------------------------
  // 运动学关系
  // -------------------------------------------------------------------------

  /// 由前轮转角求轨迹曲率 κ = tan(δ) / L
  double curvatureFromSteering(double steering) const;

  /// 由曲率求所需前轮转角 δ = atan(κ · L)
  double steeringFromCurvature(double curvature) const;

  /// 最小转弯半径 R = L / tan(δ_max)
  double minTurningRadius() const;

  /// 后轴中心沿圆弧前进 ds 后的位姿（精确积分，非欧拉近似）
  Pose2 advanceOnArc(const Pose2& pose, double curvature, double ds) const;

  // -------------------------------------------------------------------------
  // 仿真推进
  // -------------------------------------------------------------------------

  /// 按给定油门/制动、转角推进一个时间步。
  ///
  /// 纵向采用加速度受限的一阶模型，横向按自行车模型精确积分。
  /// 输入会被裁剪到车辆物理约束范围内，输出保证满足约束。
  ///
  /// @param throttle 归一化驱动力 [0,1]
  /// @param brake    归一化制动力 [0,1]
  /// @param steering 目标前轮转角 (rad)
  /// @param dt       时间步长 (s)
  VehicleState step(const VehicleState& state, double throttle, double brake,
                    double steering, double dt) const;

  /// 按目标加速度与目标转角推进（规划模块的常用接口）
  VehicleState stepByAcceleration(const VehicleState& state, double acceleration,
                                  double steering, double dt) const;

  /// 沿给定曲率与速度推进（纯跟踪式，用于回放轨迹）
  VehicleState stepAlongPath(const VehicleState& state, double curvature, double speed,
                             double dt) const;

  // -------------------------------------------------------------------------
  // 几何与安全检查
  // -------------------------------------------------------------------------

  /// 车辆轮廓（有向包围盒）
  Obb2 boundingBox(const VehicleState& state) const;

  /// 按当前速度与转角推算的转弯半径；直行时返回极大值
  double currentTurningRadius(const VehicleState& state) const;

  /// 判断状态是否满足车辆运动学约束（速度、加速度、转角均在范围内）
  bool isStateFeasible(const VehicleState& state) const;

  /// 给定速度下不发生侧滑的最大曲率（基于最大转角）
  double maxCurvatureAtSpeed(double speed) const;

  /// 参数的静态校验，返回问题描述；参数合法时返回空字符串
  std::string validate() const;

 private:
  VehicleParams params_;
};

}  // namespace adsim
