// =============================================================================
//  SpeedPlanner.h — 速度规划
//
//  路径确定了"走哪条线"，速度规划确定"以多快的速度走这条线"。
//
//  三个约束共同决定速度上限，取最小值：
//    * 曲率约束 —— 横向加速度 a_lat = v²·κ 不得超过舒适上限，否则乘客会被甩
//    * 前车约束 —— 按车头时距跟车，间距不足时减速（IDM 模型）
//    * 舒适约束 —— 纵向加速度与加加速度（jerk）都要有界，否则会有顿挫感
//
//  得到速度剖面后再做一次前向-后向传播：前向传播保证加速度可行，
//  后向传播保证能在每个减速点之前减下来。只做单向传播会得到"看着可行、
//  实际刹不住"的剖面。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <string>
#include <vector>

namespace adsim {

class SpeedPlanner {
 public:
  struct Config {
    /// 纵向加速度上限 (m/s²)
    double max_acceleration{2.0};
    /// 常规减速度上限 (m/s²，取正值表示大小)
    double max_deceleration{4.0};
    /// 紧急减速度上限 (m/s²)
    double emergency_deceleration{8.0};
    /// 加加速度上限 (m/s³)，决定加速度变化的平滑程度
    double max_jerk{2.5};
    /// 允许的最大横向加速度 (m/s²)，通常取 1.5~3.0
    double max_lateral_acceleration{2.0};
    /// 跟车时距 (s)
    double time_headway{1.8};
    /// 最小静止间距 (m)
    double min_gap{5.0};
    /// 期望减速度（IDM 中的舒适减速度）
    double comfortable_deceleration{2.0};
  };

  SpeedPlanner();
  explicit SpeedPlanner(const Config& config);

  /// 由路径生成带速度剖面的轨迹。
  ///
  /// @param path           路径点
  /// @param target_speed   期望巡航速度 (m/s)
  /// @param current_speed  自车当前速度，用于保证首点速度连续
  /// @param dt             相邻路径点之间的时间步（影响 jerk 约束的强度）
  Trajectory plan(const std::vector<Vec2>& path, double target_speed,
                  double current_speed, double dt = 0.1) const;

  /// 由曲率反推速度上限：v = sqrt(a_lat_max / κ)
  static double curvatureSpeedLimit(double curvature, double max_lateral_acceleration);

  /// IDM（智能驾驶员模型）跟车目标速度
  /// @param ego_speed  自车速度
  /// @param lead_speed 前车速度
  /// @param gap        实际间距 (m)
  double followSpeed(double ego_speed, double lead_speed, double gap) const;

  /// 对速度序列做加速度与 jerk 约束下的可行性修正
  static void enforceFeasibility(std::vector<double>& speeds,
                                 const std::vector<double>& spacings,
                                 const Config& config);

  const Config& config() const { return config_; }
  Config& config() { return config_; }

 private:
  Config config_;
};

}  // namespace adsim
