// =============================================================================
//  QtVisualizer.h — Qt 交互式可视化前端
//
//  与 SvgCanvas 共用同一套几何数据与样式语义，区别在于：
//    * 支持鼠标平移/缩放、时间轴拖动、图层开关
//    * 用于交互式复盘——工程师可以逐帧回看某个决策失误点
//
//  **可选依赖**：需要 Qt5/Qt6 Widgets，以 `ADSIM_WITH_QT=ON` 构建时才编译。
//  未启用时本头文件仍可被包含（内部以 ADSIM_HAS_QT 分支屏蔽 Qt 类型），
//  调用方通过 isCompiled() 判断是否可用。
// =============================================================================
#pragma once

#include "adsim/sim/Scenario.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/viz/SimVisualizer.h"

#include <memory>
#include <string>

namespace adsim {

/// 交互式可视化窗口的配置
struct QtVisualizerConfig {
  VisualizerConfig render;
  std::string window_title{"ADSim 仿真复盘"};
  bool show_timeline{true};
  bool show_object_labels{true};
  bool follow_ego{false};   ///< 视角是否跟随自车
};

class QtVisualizer {
 public:
  QtVisualizer();
  explicit QtVisualizer(const QtVisualizerConfig& config);
  ~QtVisualizer();

  QtVisualizer(const QtVisualizer&) = delete;
  QtVisualizer& operator=(const QtVisualizer&) = delete;

  /// 装载一次仿真结果供交互浏览
  void loadSimulation(const SimulationResult& result, const World& world);

  /// 装载路测回放数据
  void loadReplay(const std::vector<AlignedFrame>& frames, const World& world);

  /// 显示窗口并进入事件循环，返回退出码
  int exec();

  /// 无界面环境下导出当前帧为 SVG（用于 CI 自动出图）
  bool exportCurrentFrame(const std::string& path) const;

  /// 当前构建是否包含 Qt 支持
  static bool isCompiled();

  const QtVisualizerConfig& config() const { return config_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  QtVisualizerConfig config_;
};

}  // namespace adsim
