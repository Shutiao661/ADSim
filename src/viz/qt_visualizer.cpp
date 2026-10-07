// =============================================================================
//  qt_visualizer.cpp — Qt 交互式可视化前端
//
//  **本文件需要 Qt5/Qt6 Widgets，并且只有在以 `ADSIM_WITH_QT=ON` 构建时才会
//  被编译进 adsim_core（见根 CMakeLists.txt 的 ADSIM_OPTIONAL_SOURCES）。**
//  未定义 ADSIM_HAS_QT 时，本文件退化为纯桩实现：isCompiled() 返回 false，
//  其余接口安全返回、构造不崩溃，从而保证"关掉 Qt 也能编译通过"。
//
//  交互能力：
//    * 滚轮缩放（以鼠标位置为锚点，缩放时锚点下的世界坐标保持不动）
//    * 按住左键拖拽平移
//    * 底部时间轴拖动切换仿真帧，支持播放/暂停
//    * 图层开关：网格 / 轨迹 / 规划路径 / 安全边界
//
//  绘制复用 SimVisualizer 的几何与配色约定（等比缩放 + Y 轴翻转、
//  绿→黄→红的曲率配色），保证"屏幕看到的"与"导出 SVG 得到的"一致。
// =============================================================================
#include "adsim/viz/QtVisualizerImpl.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#if defined(ADSIM_HAS_QT)

// Qt 头文件必须先于 Impl 定义被包含：Impl 持有 unique_ptr<QMainWindow>，
// 其析构需要完整类型。
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QFont>
#include <QFontMetricsF>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QPolygonF>
#include <QPushButton>
#include <QSlider>
#include <QStatusBar>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

namespace adsim {

namespace {

// ---------------------------------------------------------------------------
//  视图变换：世界坐标 ↔ 屏幕坐标
//  与 SvgCanvas 完全同构——等比缩放、Y 轴翻转，否则车辆轮廓会变形。
// ---------------------------------------------------------------------------
struct ViewTransform {
  double scale{4.0};   ///< 像素 / 米
  Vec2 center;         ///< 视图中心对应的世界坐标
  QPointF pan{0.0, 0.0};
  double width{1.0};
  double height{1.0};

  QPointF toScreen(const Vec2& world) const {
    return QPointF(width * 0.5 + (world.x - center.x) * scale + pan.x(),
                   height * 0.5 - (world.y - center.y) * scale + pan.y());
  }

  Vec2 toWorld(const QPointF& screen) const {
    return Vec2{center.x + (screen.x() - width * 0.5 - pan.x()) / scale,
                center.y - (screen.y() - height * 0.5 - pan.y()) / scale};
  }
};

// ---------------------------------------------------------------------------
//  配色（与 SimVisualizer.cpp 保持一致，便于屏幕与导出图对照阅读）
// ---------------------------------------------------------------------------

QColor objectColor(RoadObject::Type type) {
  switch (type) {
    case RoadObject::Type::kVehicle:
      return QColor("#1565C0");
    case RoadObject::Type::kPedestrian:
      return QColor("#EF6C00");
    case RoadObject::Type::kBicycle:
      return QColor("#6A1B9A");
    case RoadObject::Type::kStatic:
      return QColor("#546E7A");
  }
  return QColor("#546E7A");
}

QColor eventColor(SafetyEvent event) {
  switch (event) {
    case SafetyEvent::kCollision:
      return QColor("#B71C1C");
    case SafetyEvent::kNearMiss:
    case SafetyEvent::kLowTtc:
      return QColor("#E53935");
    case SafetyEvent::kHarshBraking:
      return QColor("#EF6C00");
    case SafetyEvent::kOffRoad:
      return QColor("#6A1B9A");
    case SafetyEvent::kOverSpeed:
      return QColor("#F9A825");
    case SafetyEvent::kNone:
      break;
  }
  return QColor("#546E7A");
}

/// 事件名（内核的 toString(SafetyEvent) 不在可视化层的依赖范围内）
QString eventLabel(SafetyEvent event) {
  switch (event) {
    case SafetyEvent::kCollision:
      return QStringLiteral("碰撞");
    case SafetyEvent::kNearMiss:
      return QStringLiteral("接近事故");
    case SafetyEvent::kLowTtc:
      return QStringLiteral("低 TTC");
    case SafetyEvent::kOffRoad:
      return QStringLiteral("驶出道路");
    case SafetyEvent::kOverSpeed:
      return QStringLiteral("超速");
    case SafetyEvent::kHarshBraking:
      return QStringLiteral("急刹");
    case SafetyEvent::kNone:
      break;
  }
  return QStringLiteral("事件");
}

/// 曲率 → 颜色：绿（平直）→ 黄 → 红（接近 max κ）
QColor curvatureColor(double t) {
  const double u = clamp(t, 0.0, 1.0);
  const QColor cold(46, 125, 50);
  const QColor warm(249, 168, 37);
  const QColor hot(198, 40, 40);
  const QColor& a = u < 0.5 ? cold : warm;
  const QColor& b = u < 0.5 ? warm : hot;
  const double k = u < 0.5 ? u * 2.0 : (u - 0.5) * 2.0;
  return QColor(static_cast<int>(a.red() + (b.red() - a.red()) * k),
                static_cast<int>(a.green() + (b.green() - a.green()) * k),
                static_cast<int>(a.blue() + (b.blue() - a.blue()) * k));
}

QPolygonF toPolygon(const std::vector<Vec2>& points, const ViewTransform& view) {
  QPolygonF polygon;
  for (const Vec2& p : points) polygon << view.toScreen(p);
  return polygon;
}

/// 带底衬的文字标签（车辆 ID、事件名等），保证在任何底色上都可读
void drawTaggedText(QPainter& painter, const QPointF& at, const QString& text,
                    const QColor& background, const QColor& foreground, double font_px = 11.0) {
  QFont font = painter.font();
  font.setPixelSize(static_cast<int>(font_px));
  painter.setFont(font);

  const QFontMetricsF metrics(font);
  const QRectF box(at.x(), at.y() - metrics.height(),
                   metrics.horizontalAdvance(text) + 8.0, metrics.height() + 2.0);

  painter.setPen(Qt::NoPen);
  painter.setBrush(background);
  painter.drawRect(box);
  painter.setBrush(Qt::NoBrush);
  painter.setPen(foreground);
  painter.drawText(box.adjusted(4.0, 0.0, 0.0, 0.0), Qt::AlignVCenter | Qt::AlignLeft, text);
}

/// 鼠标位置：Qt5 用 localPos()，Qt6 用 position()。
/// 直接用 pos() 在 Qt5.15/Qt6 上会触发弃用警告，而本工程要求零警告。
QPointF mousePosition(const QMouseEvent* event) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  return event->position();
#else
  return event->localPos();
#endif
}

/// 滚轮位置：position() 自 Qt 5.14 起提供，更早版本只能退化为 pos()
QPointF wheelPosition(const QWheelEvent* event) {
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
  return event->position();
#else
  return QPointF(event->pos());
#endif
}

void drawPlainText(QPainter& painter, const QPointF& at, const QString& text,
                   const QColor& color, double font_px = 12.0) {
  QFont font = painter.font();
  font.setPixelSize(static_cast<int>(font_px));
  painter.setFont(font);
  painter.setPen(color);
  painter.drawText(at, text);
}

// ---------------------------------------------------------------------------
//  道路与物体绘制（等价于 SimVisualizer::drawLane / drawObjects）
// ---------------------------------------------------------------------------

std::vector<Vec2> offsetPolyline(const std::vector<Vec2>& line, double offset) {
  std::vector<Vec2> out;
  out.reserve(line.size());
  for (std::size_t i = 0; i < line.size(); ++i) {
    const Vec2& prev = line[i == 0 ? 0 : i - 1];
    const Vec2& next = line[i + 1 < line.size() ? i + 1 : line.size() - 1];
    Vec2 tangent = (next - prev).normalized();
    if (tangent.norm() < kEpsilon) tangent = Vec2{1.0, 0.0};
    out.push_back(line[i] + tangent.perp() * offset);
  }
  return out;
}

void paintLanes(QPainter& painter, const ViewTransform& view, const World& world,
                const VisualizerConfig& config) {
  for (const Lane& lane : world.lanes()) {
    if (lane.centerline.size() < 2) continue;
    const double half = std::max(lane.width, 0.5) * 0.5;

    const std::vector<Vec2> left = offsetPolyline(lane.centerline, half);
    const std::vector<Vec2> right = offsetPolyline(lane.centerline, -half);

    // 路面填充：先填色再画边线，视觉层次与俯视 SVG 一致
    QPolygonF surface = toPolygon(left, view);
    const QPolygonF right_polygon = toPolygon(right, view);
    for (int i = right_polygon.size() - 1; i >= 0; --i) surface << right_polygon.at(i);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(lane.is_junction ? "#FFF3E0" : "#ECEFF1"));
    painter.drawPolygon(surface);

    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor("#90A4AE"), 1.4));
    painter.drawPolyline(toPolygon(left, view));
    painter.drawPolyline(right_polygon);

    if (config.draw_lane_centerlines) {
      QPen dash(QColor("#FFB300"), 1.0, Qt::DashLine);
      painter.setPen(dash);
      painter.drawPolyline(toPolygon(lane.centerline, view));
    }
  }
}

void paintObstacles(QPainter& painter, const ViewTransform& view, const World& world) {
  painter.setPen(QPen(QColor("#455A64"), 1.2));
  painter.setBrush(QColor(69, 90, 100, 90));
  for (const Obb2& obstacle : world.obstacles()) {
    painter.drawPolygon(toPolygon(obstacle.corners(), view));
  }
  painter.setBrush(Qt::NoBrush);
}

void paintObjects(QPainter& painter, const ViewTransform& view,
                  const std::vector<RoadObject>& objects, bool show_labels,
                  const VisualizerConfig& config) {
  for (const RoadObject& object : objects) {
    const QColor color = objectColor(object.type);
    QColor fill = color;
    fill.setAlpha(90);

    painter.setPen(QPen(color, 1.6));
    painter.setBrush(fill);
    painter.drawPolygon(toPolygon(object.obb().corners(), view));

    if (show_labels) {
      const QPointF at = view.toScreen(object.position() + Vec2{0.0, object.width * 0.5 + 1.6});
      drawTaggedText(painter, at, QString("#%1").arg(object.id), QColor("#FFFFFF"), color);
    }

    if (config.draw_velocity_arrows && object.speed > 0.5) {
      const Vec2 head = object.position() + object.velocity;
      painter.setPen(QPen(color, 1.6));
      painter.setBrush(color);
      painter.drawLine(view.toScreen(object.position()), view.toScreen(head));
      painter.drawEllipse(view.toScreen(head), 3.0, 3.0);
    }
  }
  painter.setBrush(Qt::NoBrush);
}

void paintEgo(QPainter& painter, const ViewTransform& view, const TrajectoryPoint& ego,
              bool with_safety_margin, double safety_margin) {
  if (with_safety_margin && safety_margin > 0.0) {
    QPen dash(QColor("#E53935"), 1.2, Qt::DashLine);
    painter.setPen(dash);
    painter.setBrush(Qt::NoBrush);
    painter.drawPolygon(
        toPolygon(Obb2(ego.pose(), 4.5 + 2.0 * safety_margin, 1.8 + 2.0 * safety_margin)
                      .corners(),
                  view));
  }

  painter.setPen(QPen(QColor("#C2185B"), 2.4));
  painter.setBrush(QColor(194, 24, 91, 150));
  painter.drawPolygon(toPolygon(Obb2(ego.pose(), 4.5, 1.8).corners(), view));
  painter.setBrush(Qt::NoBrush);

  drawTaggedText(painter, view.toScreen(ego.position() + Vec2{0.0, 2.7}),
                 QStringLiteral("EGO"), QColor("#C2185B"), QColor("#FFFFFF"));
}

}  // namespace

// ---------------------------------------------------------------------------
//  SimulationCanvas
// ---------------------------------------------------------------------------

SimulationCanvas::SimulationCanvas(QWidget* parent) : QWidget(parent) {
  setMinimumSize(640, 400);
  setAutoFillBackground(true);
  setMouseTracking(false);
  setFocusPolicy(Qt::StrongFocus);
}

void SimulationCanvas::bindSimulation(const SimulationResult* result, const World* world) {
  result_ = result;
  world_ = world;
  frames_ = nullptr;
  view_initialized_ = false;
  step_ = result != nullptr ? std::max(static_cast<int>(result->ego_states.size()) - 1, 0) : 0;
  fitToData();
  update();
}

void SimulationCanvas::bindReplay(const std::vector<AlignedFrame>* frames) {
  frames_ = frames;
  fitToData();
  update();
}

int SimulationCanvas::stepCount() const {
  if (result_ == nullptr) return 0;
  return static_cast<int>(result_->ego_states.size());
}

void SimulationCanvas::setStep(int step) {
  const int count = stepCount();
  if (count <= 0) {
    step_ = 0;
  } else {
    step_ = clamp(step, 0, count - 1);
  }
  update();
}

BoundingBox2 SimulationCanvas::dataBounds() const {
  BoundingBox2 box;
  if (world_ != nullptr) {
    for (const Lane& lane : world_->lanes()) {
      for (const Vec2& p : lane.centerline) box.expand(p);
    }
    for (const Obb2& obstacle : world_->obstacles()) {
      for (const Vec2& corner : obstacle.corners()) box.expand(corner);
    }
    for (const RoadObject& object : world_->objects()) box.expand(object.position());
  }
  if (result_ != nullptr) {
    for (const TrajectoryPoint& point : result_->ego_states) box.expand(point.position());
  }
  if (frames_ != nullptr) {
    for (const AlignedFrame& frame : *frames_) {
      if (frame.has_odom) box.expand(frame.odom.pose().position());
    }
  }
  if (!box.valid()) {
    box.expand({-10.0, -10.0});
    box.expand({10.0, 10.0});
  }
  return box;
}

void SimulationCanvas::fitToData() {
  const BoundingBox2 box = dataBounds();
  center_ = box.center();
  pan_ = QPointF(0.0, 0.0);

  const double span_x = std::max(box.max.x - box.min.x, 1.0);
  const double span_y = std::max(box.max.y - box.min.y, 1.0);
  const double avail_w = std::max(static_cast<double>(width()) - 60.0, 10.0);
  const double avail_h = std::max(static_cast<double>(height()) - 60.0, 10.0);

  // 取较小者 → 等比缩放，保证与导出 SVG 的观感一致
  view_scale_ = std::max(std::min(avail_w / span_x, avail_h / span_y), 0.05);
  view_initialized_ = true;
}

void SimulationCanvas::resetView() {
  fitToData();
  update();
}

QPointF SimulationCanvas::toScreen(const Vec2& world) const {
  const ViewTransform view{view_scale_, center_, pan_, static_cast<double>(width()),
                           static_cast<double>(height())};
  return view.toScreen(world);
}

Vec2 SimulationCanvas::toWorld(const QPointF& screen) const {
  const ViewTransform view{view_scale_, center_, pan_, static_cast<double>(width()),
                           static_cast<double>(height())};
  return view.toWorld(screen);
}

void SimulationCanvas::zoomAt(const QPointF& anchor, double factor) {
  if (factor <= 0.0 || !std::isfinite(factor)) return;

  const ViewTransform before{view_scale_, center_, pan_, static_cast<double>(width()),
                             static_cast<double>(height())};
  const Vec2 world_under_cursor = before.toWorld(anchor);

  view_scale_ = clamp(view_scale_ * factor, 0.05, 400.0);

  // 反解 pan，使锚点下的世界坐标在缩放前后保持不动（以鼠标为中心缩放）
  const double half_w = static_cast<double>(width()) * 0.5;
  const double half_h = static_cast<double>(height()) * 0.5;
  pan_ = QPointF(anchor.x() - half_w - (world_under_cursor.x - center_.x) * view_scale_,
                 anchor.y() - half_h + (world_under_cursor.y - center_.y) * view_scale_);
  update();
}

void SimulationCanvas::wheelEvent(QWheelEvent* event) {
  const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
  if (std::fabs(steps) < 1e-9) {
    event->ignore();
    return;
  }
  zoomAt(wheelPosition(event), std::pow(1.15, steps));
  event->accept();
}

void SimulationCanvas::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    drag_origin_ = mousePosition(event);
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QWidget::mousePressEvent(event);
}

void SimulationCanvas::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_) {
    const QPointF current = mousePosition(event);
    pan_ += current - drag_origin_;
    drag_origin_ = current;
    update();
    event->accept();
    return;
  }
  QWidget::mouseMoveEvent(event);
}

void SimulationCanvas::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton && dragging_) {
    dragging_ = false;
    unsetCursor();
    event->accept();
    return;
  }
  QWidget::mouseReleaseEvent(event);
}

void SimulationCanvas::paintEvent(QPaintEvent* event) {
  (void)event;
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  painter.fillRect(rect(), QColor("#FFFFFF"));

  if (!view_initialized_) fitToData();

  ViewTransform view{view_scale_, center_, pan_, static_cast<double>(width()),
                     static_cast<double>(height())};

  // 跟随自车：把自车位置钉在画布中心，长距离场景下不必手动拖拽
  if (follow_ego_ && result_ != nullptr && !result_->ego_states.empty()) {
    const std::size_t index = static_cast<std::size_t>(step_);
    if (index < result_->ego_states.size()) {
      center_ = result_->ego_states[index].position();
      pan_ = QPointF(0.0, 0.0);
      view.center = center_;
      view.pan = pan_;
    }
  }

  // 背景网格
  if (layer_grid_) {
    const Vec2 top_left = view.toWorld(QPointF(0.0, 0.0));
    const Vec2 bottom_right =
        view.toWorld(QPointF(static_cast<double>(width()), static_cast<double>(height())));
    double spacing = std::max(config_.grid_spacing, 0.5);
    while ((bottom_right.x - top_left.x) / spacing > 200.0) spacing *= 2.0;

    painter.setPen(QPen(QColor("#E8ECEF"), 1.0));
    // 屏幕左上角对应世界坐标的最大 Y、最小 X，故横向网格线从
    // top_left.x 画到 bottom_right.x，纵向网格线从 bottom_right.y 画到 top_left.y。
    for (double y = std::floor(bottom_right.y / spacing) * spacing; y <= top_left.y;
         y += spacing) {
      painter.drawLine(view.toScreen({top_left.x, y}), view.toScreen({bottom_right.x, y}));
    }
    for (double x = std::floor(top_left.x / spacing) * spacing; x <= bottom_right.x;
         x += spacing) {
      painter.drawLine(view.toScreen({x, bottom_right.y}), view.toScreen({x, top_left.y}));
    }
  }

  if (world_ != nullptr) {
    paintLanes(painter, view, *world_, config_);
    paintObstacles(painter, view, *world_);
  }

  if (result_ != nullptr) {
    const std::size_t index = static_cast<std::size_t>(step_);

    // 其他物体：优先取该步的历史记录，缺失时退回世界中的物体列表
    const std::vector<RoadObject>* objects = nullptr;
    if (index < result_->object_history.size()) {
      objects = &result_->object_history[index];
    } else if (world_ != nullptr && !world_->objects().empty()) {
      objects = &world_->objects();
    }
    if (objects != nullptr) {
      paintObjects(painter, view, *objects, show_labels_, config_);
    }

    // 整条轨迹：按曲率着色（默认）或统一冷色
    if (layer_trajectory_ && result_->ego_states.size() >= 2) {
      double kappa_ref = result_->max_curvature;
      if (kappa_ref <= kEpsilon) {
        for (const TrajectoryPoint& point : result_->ego_states) {
          kappa_ref = std::max(kappa_ref, std::fabs(point.kappa));
        }
      }
      if (kappa_ref <= kEpsilon) kappa_ref = 1.0;

      for (std::size_t i = 0; i + 1 < result_->ego_states.size(); ++i) {
        const double kappa =
            0.5 * (std::fabs(result_->ego_states[i].kappa) +
                   std::fabs(result_->ego_states[i + 1].kappa));
        painter.setPen(QPen(curvatureColor(kappa / kappa_ref), 2.5));
        painter.drawLine(view.toScreen(result_->ego_states[i].position()),
                         view.toScreen(result_->ego_states[i + 1].position()));
      }
    }

    // 规划路径：虚线
    if (layer_planned_ && result_->planned_paths.size() >= 2) {
      std::vector<Vec2> path;
      path.reserve(result_->planned_paths.size());
      for (const TrajectoryPoint& point : result_->planned_paths) {
        path.push_back(point.position());
      }
      painter.setPen(QPen(QColor("#00838F"), 2.0, Qt::DashLine));
      painter.setBrush(Qt::NoBrush);
      painter.drawPolyline(toPolygon(path, view));
    }

    if (index < result_->ego_states.size()) {
      paintEgo(painter, view, result_->ego_states[index], layer_safety_, config_.safety_margin);
    }
  }

  // 回放轨迹：橙色，与仿真轨迹（蓝色）形成对比
  if (frames_ != nullptr) {
    std::vector<Vec2> replay;
    replay.reserve(frames_->size());
    for (const AlignedFrame& frame : *frames_) {
      if (frame.has_odom) replay.push_back(frame.odom.pose().position());
    }
    if (replay.size() >= 2) {
      painter.setPen(QPen(QColor("#EF6C00"), 2.5));
      painter.setBrush(Qt::NoBrush);
      painter.drawPolyline(toPolygon(replay, view));
    }
  }

  // 安全事件标记
  if (config_.draw_events && result_ != nullptr) {
    for (const SafetyEventRecord& record : result_->events) {
      if (record.event == SafetyEvent::kNone) continue;

      std::size_t index = 0;
      bool found = false;
      for (std::size_t i = 0; i < result_->time.size() && i < result_->ego_states.size(); ++i) {
        if (result_->time[i] >= record.time) {
          index = i;
          found = true;
          break;
        }
      }
      if (result_->ego_states.empty()) continue;
      if (!found) index = result_->ego_states.size() - 1;

      const QPointF at = view.toScreen(result_->ego_states[index].position());
      const QColor color = eventColor(record.event);
      painter.setPen(QPen(color, 2.0));
      painter.setBrush(Qt::NoBrush);
      painter.drawEllipse(at, 7.0, 7.0);
      painter.drawLine(at + QPointF(-7.0, 0.0), at + QPointF(7.0, 0.0));
      painter.drawLine(at + QPointF(0.0, -7.0), at + QPointF(0.0, 7.0));
      drawTaggedText(painter, at + QPointF(0.0, -12.0), eventLabel(record.event),
                     QColor("#FFFFFF"), color);
    }
  }

  // HUD
  if (result_ != nullptr) {
    const std::size_t index = static_cast<std::size_t>(step_);
    QString title = QString::fromUtf8(result_->scenario_name.c_str());
    if (title.isEmpty()) title = QStringLiteral("未命名场景");
    if (index < result_->time.size()) {
      title += QString("  t=%1s").arg(result_->time[index], 0, 'f', 2);
    }
    if (index < result_->ego_states.size()) {
      const TrajectoryPoint& ego = result_->ego_states[index];
      title += QString("  v=%1 m/s  a=%2 m/s²  κ=%3 1/m")
                   .arg(ego.v, 0, 'f', 2)
                   .arg(ego.a, 0, 'f', 2)
                   .arg(ego.kappa, 0, 'f', 4);
    }
    drawPlainText(painter, QPointF(12.0, 24.0), title, QColor("#263238"), 13.0);
    drawPlainText(painter, QPointF(12.0, 44.0),
                  QStringLiteral("滚轮缩放 · 左键拖拽平移 · 底部时间轴切帧"),
                  QColor("#78909C"), 12.0);
  }
}

// ---------------------------------------------------------------------------
//  QtVisualizer::Impl
// ---------------------------------------------------------------------------

void QtVisualizer::Impl::buildUi(const std::string& title) {
  if (window) return;

  window = std::make_unique<QMainWindow>();
  window->setWindowTitle(QString::fromUtf8(title.c_str()));
  window->resize(static_cast<int>(config.render.width) + 40,
                 static_cast<int>(config.render.height) + 120);

  auto* central = new QWidget(window.get());
  auto* layout = new QVBoxLayout(central);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(6);

  canvas = new SimulationCanvas(central);
  canvas->setRenderConfig(config.render);
  canvas->setShowObjectLabels(config.show_object_labels);
  canvas->setFollowEgo(config.follow_ego);
  layout->addWidget(canvas, 1);

  // ---- 控制条 ----
  auto* controls = new QWidget(central);
  auto* controls_layout = new QHBoxLayout(controls);
  controls_layout->setContentsMargins(0, 0, 0, 0);

  auto* play_button = new QPushButton(QStringLiteral("播放"), controls);
  controls_layout->addWidget(play_button);

  slider = new QSlider(Qt::Horizontal, controls);
  slider->setRange(0, std::max(canvas->stepCount() - 1, 0));
  slider->setValue(canvas->step());
  controls_layout->addWidget(slider, 1);

  step_label = new QLabel(controls);
  controls_layout->addWidget(step_label);

  // ---- 图层开关 ----
  auto* grid_box = new QCheckBox(QStringLiteral("网格"), controls);
  grid_box->setChecked(true);
  auto* traj_box = new QCheckBox(QStringLiteral("轨迹"), controls);
  traj_box->setChecked(true);
  auto* planned_box = new QCheckBox(QStringLiteral("规划路径"), controls);
  planned_box->setChecked(true);
  auto* safety_box = new QCheckBox(QStringLiteral("安全边界"), controls);
  safety_box->setChecked(true);
  controls_layout->addWidget(grid_box);
  controls_layout->addWidget(traj_box);
  controls_layout->addWidget(planned_box);
  controls_layout->addWidget(safety_box);

  layout->addWidget(controls);
  window->setCentralWidget(central);

  if (!config.show_timeline) {
    slider->setVisible(false);
    play_button->setVisible(false);
  }

  timer = new QTimer(window.get());
  timer->setInterval(40);  // 25 fps：逐帧回看足够流畅，也不会吃满 CPU

  // 信号一律接到 lambda：本工程未启用 AUTOMOC，画布类没有 Q_OBJECT，
  // 但连接目标（QSlider/QCheckBox/QTimer）都是标准控件，无需 moc 参与。
  QObject::connect(slider, &QSlider::valueChanged, canvas, [this](int value) {
    canvas->setStep(value);
    refreshStatus();
  });

  QObject::connect(play_button, &QPushButton::clicked, window.get(), [this, play_button]() {
    playing = !playing;
    if (playing) {
      timer->start();
      play_button->setText(QStringLiteral("暂停"));
    } else {
      timer->stop();
      play_button->setText(QStringLiteral("播放"));
    }
  });

  QObject::connect(timer, &QTimer::timeout, window.get(), [this]() {
    const int count = canvas->stepCount();
    if (count <= 0) return;
    const int next = (canvas->step() + 1) % count;
    canvas->setStep(next);
    slider->setValue(next);
    refreshStatus();
  });

  QObject::connect(grid_box, &QCheckBox::stateChanged, canvas,
                   [this](int state) { canvas->setLayerGrid(state != 0); });
  QObject::connect(traj_box, &QCheckBox::stateChanged, canvas,
                   [this](int state) { canvas->setLayerTrajectory(state != 0); });
  QObject::connect(planned_box, &QCheckBox::stateChanged, canvas,
                   [this](int state) { canvas->setLayerPlanned(state != 0); });
  QObject::connect(safety_box, &QCheckBox::stateChanged, canvas,
                   [this](int state) { canvas->setLayerSafety(state != 0); });

  refreshStatus();
}

void QtVisualizer::Impl::refreshStatus() {
  if (step_label == nullptr || canvas == nullptr) return;

  const int count = canvas->stepCount();
  const int index = canvas->step();
  QString text = QString("%1/%2 帧").arg(index).arg(std::max(count - 1, 0));

  const std::size_t i = static_cast<std::size_t>(index);
  if (has_result && i < result.time.size()) {
    text += QString("  t=%1s").arg(result.time[i], 0, 'f', 2);
  }
  if (has_result && i < result.ego_states.size()) {
    text += QString("  v=%1 m/s").arg(result.ego_states[i].v, 0, 'f', 2);
  }
  if (has_result) {
    text += QString("  事件 %1").arg(result.events.size());
  }
  step_label->setText(text);
  if (window && window->statusBar()) window->statusBar()->showMessage(text);
}

namespace {

/// 保证进程内存在唯一的 QApplication。Qt 规定一个进程只能有一个 QApplication，
/// 且必须在创建任何 QWidget 之前构造，因此这里用一个函数内静态量持有它。
QApplication* ensureApplication() {
  static std::unique_ptr<QApplication> owned;
  if (QApplication::instance() == nullptr && !owned) {
    static int argc = 1;
    static char name[] = "adsim_viz";
    static char* argv[] = {name, nullptr};
    owned = std::make_unique<QApplication>(argc, argv);
  }
  return qobject_cast<QApplication*>(QApplication::instance());
}

}  // namespace

// ---------------------------------------------------------------------------
//  QtVisualizer
// ---------------------------------------------------------------------------

QtVisualizer::QtVisualizer() : QtVisualizer(QtVisualizerConfig{}) {}

QtVisualizer::QtVisualizer(const QtVisualizerConfig& config)
    : impl_(std::make_unique<Impl>()), config_(config) {
  impl_->config = config;
  impl_->owns_application = ensureApplication() != nullptr;
}

QtVisualizer::~QtVisualizer() {
  if (impl_ && impl_->timer) impl_->timer->stop();
  // window 是 unique_ptr，随 Impl 一起销毁；画布是其子控件，先于数据成员析构
}

void QtVisualizer::loadSimulation(const SimulationResult& result, const World& world) {
  if (!impl_) return;
  impl_->result = result;
  impl_->world = world;
  impl_->has_result = true;
  impl_->buildUi(config_.window_title);

  impl_->canvas->bindSimulation(&impl_->result, &impl_->world);
  if (impl_->slider) {
    impl_->slider->setRange(0, std::max(impl_->canvas->stepCount() - 1, 0));
    impl_->slider->setValue(impl_->canvas->step());
  }
  impl_->refreshStatus();
}

void QtVisualizer::loadReplay(const std::vector<AlignedFrame>& frames, const World& world) {
  if (!impl_) return;
  impl_->frames = frames;
  impl_->world = world;
  impl_->has_replay = true;
  impl_->buildUi(config_.window_title);

  if (impl_->has_result) {
    impl_->canvas->bindSimulation(&impl_->result, &impl_->world);
  }
  impl_->canvas->bindReplay(&impl_->frames);
  impl_->refreshStatus();
}

int QtVisualizer::exec() {
  if (!impl_) return 0;
  impl_->buildUi(config_.window_title);

  QApplication* app = ensureApplication();
  if (app == nullptr) return 1;  // 例如缺少显示环境（无 DISPLAY 的纯终端）

  impl_->window->show();
  impl_->refreshStatus();
  return app->exec();
}

bool QtVisualizer::exportCurrentFrame(const std::string& path) const {
  if (!impl_ || path.empty()) return false;

  // 复用 SimVisualizer 出图：保证"屏幕上看到的当前帧"与导出结果同源同版式
  SimVisualizer visualizer(config_.render);
  std::string svg;
  if (impl_->has_result) {
    const int step = impl_->canvas != nullptr ? impl_->canvas->step() : -1;
    svg = visualizer.renderSnapshot(impl_->result, impl_->world, step);
  } else if (impl_->has_replay) {
    svg = visualizer.renderReplayComparison(impl_->frames, SimulationResult{}, impl_->world);
  }
  if (svg.empty()) return false;

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) return false;
  out.write(svg.data(), static_cast<std::streamsize>(svg.size()));
  out.flush();
  return out.good();
}

bool QtVisualizer::isCompiled() { return true; }

}  // namespace adsim

#else  // !ADSIM_HAS_QT —— 未启用 Qt 时的桩实现

namespace adsim {

QtVisualizer::QtVisualizer() : QtVisualizer(QtVisualizerConfig{}) {}

QtVisualizer::QtVisualizer(const QtVisualizerConfig& config)
    : impl_(nullptr), config_(config) {}

// Impl 为空壳，unique_ptr 的析构在此处实例化是安全的
QtVisualizer::~QtVisualizer() = default;

void QtVisualizer::loadSimulation(const SimulationResult&, const World&) {
  // 未启用 Qt：静默忽略，调用方可通过 isCompiled() 预判
}

void QtVisualizer::loadReplay(const std::vector<AlignedFrame>&, const World&) {}

int QtVisualizer::exec() { return 0; }

bool QtVisualizer::exportCurrentFrame(const std::string&) const { return false; }

bool QtVisualizer::isCompiled() { return false; }

}  // namespace adsim

#endif  // ADSIM_HAS_QT
