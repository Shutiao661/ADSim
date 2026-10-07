// =============================================================================
//  QtVisualizerImpl.h — QtVisualizer 的实现细节（仅供 qt_visualizer.cpp 包含）
//
//  **本文件需要 Qt5/Qt6 Widgets，且必须以 `ADSIM_WITH_QT=ON` 构建。**
//  未定义 ADSIM_HAS_QT 时，QtVisualizer::Impl 退化为空壳，接口全部安全返回，
//  这样 QtVisualizer.h 可以被任何翻译单元包含而不引入 Qt 依赖。
//
//  为什么单独拆一个头文件：QtVisualizer::Impl 是私有嵌套类型，只有在其成员
//  函数的定义处（即 QtVisualizer 的成员函数实现中）才能访问它的定义；
//  把实现类放在这里，可以让 qt_visualizer.cpp 保持"接口 + 装配"的清晰结构。
//
//  交互模型：
//    * 视图变换 = 等比缩放 + 平移 + Y 轴翻转，与 SvgCanvas 语义完全一致，
//      因此屏幕上的几何关系与离线导出的 SVG 一一对应；
//    * 画布只持有指向数据的指针，数据本体由 Impl 持有——Impl 的成员声明顺序
//      保证数据比窗口活得久（先销毁窗口，再销毁数据）。
// =============================================================================
#pragma once

#include "adsim/viz/QtVisualizer.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#if defined(ADSIM_HAS_QT)

#include <QPointF>
#include <QWidget>

class QCheckBox;
class QLabel;
class QMainWindow;
class QSlider;
class QTimer;

namespace adsim {

/// 交互式仿真画布：用 QPainter 复刻 SimVisualizer 的俯视图绘制逻辑。
/// 不声明 Q_OBJECT：本工程不启用 AUTOMOC，信号一律连接到 lambda，
/// 因此这里不需要 moc 生成的元对象代码（重写的都是 QWidget 的虚函数）。
class SimulationCanvas : public QWidget {
 public:
  explicit SimulationCanvas(QWidget* parent = nullptr);

  /// 装载仿真结果。仅保存指针，数据本体由 QtVisualizer::Impl 持有。
  void bindSimulation(const SimulationResult* result, const World* world);
  /// 装载回放数据（用于对比视图）
  void bindReplay(const std::vector<AlignedFrame>* frames);

  void setStep(int step);
  int step() const { return step_; }
  int stepCount() const;

  void setRenderConfig(const VisualizerConfig& config) { config_ = config; }
  void setShowObjectLabels(bool show) { show_labels_ = show; }
  void setFollowEgo(bool follow) { follow_ego_ = follow; update(); }

  /// 图层开关
  void setLayerGrid(bool on) { layer_grid_ = on; update(); }
  void setLayerTrajectory(bool on) { layer_trajectory_ = on; update(); }
  void setLayerPlanned(bool on) { layer_planned_ = on; update(); }
  void setLayerSafety(bool on) { layer_safety_ = on; update(); }

  /// 复位视图：按当前世界范围重新适配
  void resetView();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  // 世界 ↔ 屏幕。与 SvgCanvas 一致：等比缩放、Y 轴翻转。
  QPointF toScreen(const Vec2& world) const;
  Vec2 toWorld(const QPointF& screen) const;
  void zoomAt(const QPointF& anchor, double factor);
  /// 重新计算"适配世界范围"的初始缩放与居中
  void fitToData();
  /// 世界范围（车道 + 障碍 + 物体 + 轨迹），与 SimVisualizer 同源
  BoundingBox2 dataBounds() const;

  const SimulationResult* result_{nullptr};
  const World* world_{nullptr};
  const std::vector<AlignedFrame>* frames_{nullptr};

  VisualizerConfig config_;
  int step_{0};
  double view_scale_{4.0};  ///< 像素 / 米
  QPointF pan_{0.0, 0.0};
  Vec2 center_{0.0, 0.0};
  QPointF drag_origin_;
  bool dragging_{false};
  bool view_initialized_{false};

  bool show_labels_{true};
  bool follow_ego_{false};
  bool layer_grid_{true};
  bool layer_trajectory_{true};
  bool layer_planned_{true};
  bool layer_safety_{true};
};

/// QtVisualizer 的实现体
struct QtVisualizer::Impl {
  QtVisualizerConfig config;

  // 数据本体必须声明在窗口之前：Impl 析构时按声明逆序销毁，
  // 窗口先被销毁 → 画布先于它引用的数据消失。
  SimulationResult result;
  World world;
  std::vector<AlignedFrame> frames;
  bool has_result{false};
  bool has_replay{false};

  std::unique_ptr<QMainWindow> window;
  SimulationCanvas* canvas{nullptr};
  QSlider* slider{nullptr};
  QLabel* step_label{nullptr};
  QTimer* timer{nullptr};
  bool playing{false};
  bool owns_application{false};

  /// 构建窗口（首次 exec / 导出前调用）
  void buildUi(const std::string& title);
  /// 刷新状态栏与滑块的联动
  void refreshStatus();
  const SimulationResult& currentResult() const { return result; }
};

}  // namespace adsim

#else  // !ADSIM_HAS_QT

namespace adsim {

/// 未启用 Qt：Impl 退化为空壳，所有成员函数安全返回
struct QtVisualizer::Impl {};

}  // namespace adsim

#endif  // ADSIM_HAS_QT
