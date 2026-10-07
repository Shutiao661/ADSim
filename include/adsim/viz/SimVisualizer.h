// =============================================================================
//  SimVisualizer.h — 仿真过程可视化
//
//  把一次仿真或一段回放渲染成可直接查阅的图：
//    * 俯视图：车道、车辆轮廓、轨迹、规划路径、安全边界
//    * 时序图：速度 / 加速度 / 曲率 / TTC 随时间变化，并标注安全事件
//
//  时序图的横轴为时间，纵轴为归一化后的各物理量，事件以竖线标注——
//  这样"决策在哪一帧出问题、当时的曲率与 TTC 是多少"能在一张图里读出来。
// =============================================================================
#pragma once

#include "adsim/common/Types.h"
#include "adsim/datapipeline/DataPipeline.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/viz/SvgCanvas.h"

#include <string>
#include <vector>

namespace adsim {

struct VisualizerConfig {
  double width{1280.0};
  double height{720.0};
  bool draw_grid{true};
  bool draw_lane_centerlines{true};
  bool draw_safety_margin{true};
  double safety_margin{1.0};       ///< 安全边界外扩距离 (m)
  bool draw_velocity_arrows{true};
  bool draw_events{true};
  double grid_spacing{5.0};
};

class SimVisualizer {
 public:
  explicit SimVisualizer(const VisualizerConfig& config);

  /// 渲染仿真俯视图（取指定时刻的快照）
  /// @param step 取第几步的状态；负数表示取最后一步
  std::string renderSnapshot(const SimulationResult& result, const World& world,
                             int step = -1) const;

  /// 渲染完整仿真：俯视图叠加整条轨迹
  std::string renderOverview(const SimulationResult& result, const World& world) const;

  /// 渲染时序图（速度/加速度/曲率/TTC + 事件标注）
  std::string renderTimeline(const SimulationResult& result) const;

  /// 一键产出完整报告（多张 SVG 拼接为一个 SVG 文件）
  bool writeReport(const SimulationResult& result, const World& world,
                   const std::string& output_path) const;

  /// 用数据管道处理结果做"数字回放"可视化：把清洗后的路测数据画出来，
  /// 与仿真轨迹叠加对比，用于核对仿真是否忠实还原了真实场景。
  std::string renderReplayComparison(const std::vector<AlignedFrame>& frames,
                                     const SimulationResult& result,
                                     const World& world) const;

  const VisualizerConfig& config() const { return config_; }

 private:
  VisualizerConfig config_;
};

/// 生成 HTML 报告，内嵌多张 SVG 与指标表格，可直接用浏览器打开
std::string buildHtmlReport(const std::vector<std::pair<std::string, std::string>>& sections,
                            const std::string& title);

}  // namespace adsim
