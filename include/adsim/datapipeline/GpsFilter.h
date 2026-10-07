// =============================================================================
//  GpsFilter.h — GPS / RTK 定位滤波
//
//  路测中 GPS 存在三类问题，必须滤波后才能用于仿真回放：
//    1. 随机噪声 —— 单点定位水平误差可达米级
//    2. 离群跳点 —— 城市峡谷、隧道出口处位置瞬间跳变数十米
//    3. 精度时变 —— RTK 固定解与单点解的置信度差两个数量级
//
//  采用常速度模型的线性卡尔曼滤波：
//      状态 x = [px, py, vx, vy]^T
//      观测 z = [px, py]^T
//  并通过新息（innovation）的马氏距离做卡方检验，剔除离群跳点。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace adsim {

/// 滤波后的一帧定位结果（局部 ENU 平面坐标）
struct FilteredGpsSample {
  Timestamp stamp{kInvalidTimestamp};
  Vec2 position;          ///< 滤波后位置
  Vec2 velocity;          ///< 估计速度
  Vec2 raw_position;      ///< 滤波前位置，便于对比
  bool accepted{true};    ///< 是否通过离群点检验
  bool initialized{false};
  int fix_type{0};
};

class GpsFilter {
 public:
  struct Config {
    /// 过程噪声强度（白噪声加速度的功率谱密度，m²/s³）。
    ///
    /// 取值决定滤波器对机动的响应能力：过小则跟不上真实运动、误差反而
    /// 大于原始观测；过大则退化为直通、失去平滑作用。车辆加速度可达
    /// 3 m/s²、相关时间约 0.5s，故量级在 1~10 之间。经实测标定取 5.0。
    double process_noise{5.0};
    /// 观测噪声基准（米²）
    double measurement_noise{1.0};
    /// 离群点门控阈值：新息马氏距离平方的上限。
    ///
    /// 9.21 对应 2 自由度卡方分布的 99% 分位数——这同时意味着**约 1% 的
    /// 正常观测会被固有地判为异常**，这是门控机制无法回避的代价。
    /// 对误拒更敏感的场景可放宽到 13.82（99.9%），代价是更容忍真实跳点。
    double outlier_gate{9.21};
    bool enable_outlier_rejection{true};
    /// 是否按定位质量（fix_type / hdop）动态调整观测噪声
    bool use_quality_weighting{true};

    /// 自适应观测噪声。
    ///
    /// 实车数据的 fix_type 标注与实际噪声经常不符（例如接收机报 RTK 固定解，
    /// 实际水平误差仍有米级）。若完全相信质量加权，R 会被压到极小，滤波器
    /// 过度自信，反而把大量正常观测当成离群点拒掉——这是纯质量加权方案最
    /// 典型的失效模式。
    ///
    /// 本开关直接用新息的二阶矩反推 R：
    ///     E[y₀² + y₁²] = P₀₀ + P₁₁ + 2R   ⇒   R = (E[y²] − P₀₀ − P₁₁) / 2
    ///
    /// 注意这里刻意**不**采用"NIS 偏离 2 就按比例缩放 R"的反馈式做法：
    /// 该映射是递减的，在不动点处斜率约为 −1，实际表现为持续振荡而非收敛。
    /// 直接矩估计不含这种反馈回路，因而稳定。
    bool enable_adaptive_noise{true};
    /// 新息二阶矩滑动平均的更新系数
    double adaptation_rate{0.1};
    /// 观测噪声方差的自适应上限 (m²)。
    ///
    /// 这里用**有物理意义的上限**而不是"基准 R 的若干倍"：质量加权把基准 R
    /// 压到 0.01 m² 量级时，即便放大 25 倍也只有 0.25 m²，仍远低于真实噪声，
    /// 自适应机制会被这个相对的帽子卡死。400 m² 对应 20 m 标准差，
    /// 已覆盖 GPS 的合理最差情形，同时百米级跳点的马氏距离仍有 4 个数量级
    /// 的裕度，不会被误放进来。
    double max_measurement_variance{400.0};

    /// 初速度的先验方差 (m/s)²。
    ///
    /// 首帧时车辆速度完全未知——可能静止，也可能在高速行驶。给一个偏大的
    /// 先验是诚实的做法：若先验过于自信（例如只给到 3 m/s 的标准差），
    /// 滤波器收敛期的真实滞后会被离群点门控判定为异常并拒掉，而拒绝又使
    /// 状态停止更新、滞后进一步扩大，最终"锁死"在错误预测上再也拉不回来。
    double initial_velocity_variance{400.0};

    /// 观测被拒时重新打开状态不确定度。
    ///
    /// 门控的前提是滤波器已处于稳态。当车辆做出模型未覆盖的急机动时，
    /// 新息会短暂超限而被拒；若不作处理，滤波器会一直用旧模型外推。
    /// 开启本项后每次拒绝都会放大位置协方差，使门控自然重新打开。
    bool enable_rejection_recovery{true};
    /// 位置协方差的恢复上限 (m²)，防止长时间异常后门控被撑得形同虚设
    double max_position_variance{100.0};
  };

  struct Statistics {
    std::size_t total{0};
    std::size_t accepted{0};
    std::size_t rejected{0};
    std::size_t rtk_fixed{0};
    std::size_t single_point{0};
    double max_rejected_jump{0.0};

    double acceptanceRate() const {
      return total == 0 ? 0.0 : static_cast<double>(accepted) / static_cast<double>(total);
    }

    std::string toString() const;
  };

  GpsFilter();
  explicit GpsFilter(const Config& config);

  /// 重置滤波器状态（换段数据时必须调用）
  void reset();

  /// 处理一帧定位。
  /// @param position_out 滤波后位置（局部 ENU，单位为米）
  /// @return 是否被接受；被拒绝时 position_out 为滤波器预测值
  bool update(const GpsFrame& frame, Vec2& position_out);

  /// 批量处理：首帧确定投影原点，输出逐帧滤波结果
  std::vector<FilteredGpsSample> filterAll(const std::vector<GpsFrame>& frames);

  /// 获取本次批量处理使用的投影原点
  const GeoProjector& projector() const { return projector_; }

  Vec2 velocity() const { return Vec2{state_[2], state_[3]}; }
  bool initialized() const { return initialized_; }
  const Statistics& statistics() const { return stats_; }

  // -------------------------------------------------------------------------
  // 对照组：滑动平均滤波
  //
  //  实现最简单，常用于基线对比。但它对离群点毫无抵抗能力——一个跳点会
  //  在窗口内污染多个输出，且必然引入相位滞后。保留此实现正是为了在
  //  报告中量化卡尔曼滤波相对它的增益。
  // -------------------------------------------------------------------------
  static std::vector<GpsFrame> movingAverage(const std::vector<GpsFrame>& frames,
                                             std::size_t window);

  /// 计算轨迹相对参考的均方根误差，用于对比不同滤波器的去噪效果
  static double rootMeanSquareError(const std::vector<Vec2>& estimate,
                                    const std::vector<Vec2>& reference);

 private:
  void predict(double dt);
  void correct(const Vec2& measurement, double measurement_variance);

  Config config_;
  GeoProjector projector_;

  bool initialized_{false};
  Timestamp last_stamp_{kInvalidTimestamp};

  /// 状态 [px, py, vx, vy]
  double state_[4]{0.0, 0.0, 0.0, 0.0};
  /// 状态协方差 4×4
  double covariance_[4][4]{};

  /// 新息二阶矩 (y₀²+y₁²) 的滑动平均，仅统计被接受的观测；
  /// 负值表示尚未积累样本
  double innovation_sq_ema_{-1.0};

  Statistics stats_;
};

// ---------------------------------------------------------------------------
// 航向滤波：GPS 航向在低速时极不稳定，用圆周统计做平滑
// ---------------------------------------------------------------------------
class HeadingFilter {
 public:
  /// @param alpha 指数平滑系数 (0,1]，越大越信任新观测
  explicit HeadingFilter(double alpha = 0.3);

  /// 输入原始航向，返回平滑后的航向。角度按最短路径累积，无 ±π 跳变。
  double update(double heading);

  void reset();
  std::size_t sampleCount() const { return count_; }

 private:
  double alpha_;
  double unwrapped_{0.0};  ///< 累积展开后的角度，避免跨 ±π 时被错误平均
  bool initialized_{false};
  std::size_t count_{0};
};

}  // namespace adsim
