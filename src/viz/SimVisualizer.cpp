// =============================================================================
//  SimVisualizer.cpp — 仿真过程可视化（俯视图 / 时序图 / 报告）
//
//  实现要点：
//    * 俯视图的绘制顺序严格按"背景 → 车道 → 静态障碍 → 其他物体 → 自车"，
//      保证自车永远压在最上层——复盘时第一眼看的是自车，而不是被谁挡住；
//    * 时序图的横轴统一为时间，四个子图各自归一化并**标出纵轴刻度值**，
//      否则工程师只能看到曲线形状，读不出"当时到底刹到几个 g"；
//    * 世界包围盒由车道 / 障碍 / 轨迹共同决定，任一为空都不影响出图。
//
//  注意：本文件只使用 World / SimulationResult / AlignedFrame 的**字段与内联接口**，
//  不调用定义在 sim/*.cpp 中的函数（bounds()、isCritical() 之外的成员、
//  toString(SafetyEvent) 等），这样可视化层可以脱离仿真内核单独编译与测试。
// =============================================================================
#include "adsim/viz/SimVisualizer.h"

#include "adsim/common/Types.h"
#include "adsim/datapipeline/TimeAligner.h"
#include "adsim/sim/SimEngine.h"
#include "adsim/sim/World.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace adsim {

namespace {

// ---------------------------------------------------------------------------
//  通用小工具
// ---------------------------------------------------------------------------

/// 数值 → 短字符串（保留 2 位小数，去掉尾随 0）
std::string fmt(double value, int precision = 2) {
  if (!std::isfinite(value)) return "n/a";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  std::string s = oss.str();
  const std::size_t dot = s.find('.');
  if (dot != std::string::npos) {
    s.erase(s.find_last_not_of('0') + 1);
    if (!s.empty() && s.back() == '.') s.pop_back();
  }
  if (s == "-0") s = "0";
  return s;
}

/// XML 转义。SvgCanvas::escape 是私有的，报告拼装阶段需要在类外转义标题，
/// 故此处保留一份同语义的实现。
std::string escapeXml(const std::string& text) {
  std::string out;
  out.reserve(text.size() + text.size() / 8);
  for (const char ch : text) {
    switch (ch) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      case '\'':
        out += "&apos;";
        break;
      default: {
        const unsigned char c = static_cast<unsigned char>(ch);
        if (c < 0x20 && ch != '\t' && ch != '\n' && ch != '\r') continue;
        out.push_back(ch);
        break;
      }
    }
  }
  return out;
}

/// 去掉 XML 声明：把一段 SVG 内嵌进 HTML 或另一段 SVG 时，
/// <?xml ...?> 出现在文档中间是非法的。
std::string stripXmlDeclaration(const std::string& svg) {
  const std::size_t begin = svg.find("<?xml");
  if (begin == std::string::npos) return svg;
  const std::size_t end = svg.find("?>", begin);
  if (end == std::string::npos) return svg;
  return svg.substr(0, begin) + svg.substr(end + 2);
}

/// 取出 <svg> 的内部内容（丢掉外层标签），用于把多张图拼成一张大 SVG
std::string svgInner(const std::string& svg) {
  const std::size_t open = svg.find("<svg");
  if (open == std::string::npos) return {};
  const std::size_t open_end = svg.find('>', open);
  if (open_end == std::string::npos) return {};
  const std::size_t close = svg.rfind("</svg>");
  if (close == std::string::npos || close <= open_end) return {};
  return svg.substr(open_end + 1, close - open_end - 1);
}

/// 安全事件名。内核的 toString(SafetyEvent) 定义在 sim/SimEngine.cpp，
/// 可视化层不依赖它，这里保留一份用于图注的映射。
const char* eventLabel(SafetyEvent event) {
  switch (event) {
    case SafetyEvent::kNone:
      return "无";
    case SafetyEvent::kCollision:
      return "碰撞";
    case SafetyEvent::kNearMiss:
      return "接近事故";
    case SafetyEvent::kLowTtc:
      return "低 TTC";
    case SafetyEvent::kOffRoad:
      return "驶出道路";
    case SafetyEvent::kOverSpeed:
      return "超速";
    case SafetyEvent::kHarshBraking:
      return "急刹";
  }
  return "未知";
}

/// 事件标注用色：越危险越暖
std::string eventColor(SafetyEvent event) {
  switch (event) {
    case SafetyEvent::kCollision:
      return "#B71C1C";
    case SafetyEvent::kNearMiss:
    case SafetyEvent::kLowTtc:
      return "#E53935";
    case SafetyEvent::kHarshBraking:
      return "#EF6C00";
    case SafetyEvent::kOffRoad:
      return "#6A1B9A";
    case SafetyEvent::kOverSpeed:
      return "#F9A825";
    case SafetyEvent::kNone:
      break;
  }
  return "#546E7A";
}

/// 物体类型 → 颜色
std::string objectColor(RoadObject::Type type) {
  switch (type) {
    case RoadObject::Type::kVehicle:
      return "#1565C0";  // 蓝：普通车辆
    case RoadObject::Type::kPedestrian:
      return "#EF6C00";  // 橙：行人（最需要一眼识别）
    case RoadObject::Type::kBicycle:
      return "#6A1B9A";  // 紫：两轮车
    case RoadObject::Type::kStatic:
      return "#546E7A";  // 灰：静态物体
  }
  return "#546E7A";
}

/// 车辆轮廓样式（描边 + 半透明填充，便于判断重叠）
Style boxStyle(const std::string& color, double width, double opacity = 0.35) {
  Style s;
  s.stroke = color;
  s.fill = color;
  s.stroke_width = width;
  s.opacity = opacity;
  return s;
}

// ---------------------------------------------------------------------------
//  包围盒
// ---------------------------------------------------------------------------

/// 由车道、障碍、物体与轨迹共同确定世界包围盒。
/// 不调用 World::bounds()，因为它定义在 sim/World.cpp 中。
BoundingBox2 computeBounds(const World& world, const SimulationResult& result) {
  BoundingBox2 box;

  for (const Lane& lane : world.lanes()) {
    for (const Vec2& p : lane.centerline) box.expand(p);
  }
  for (const Obb2& obstacle : world.obstacles()) {
    for (const Vec2& corner : obstacle.corners()) box.expand(corner);
  }
  for (const RoadObject& object : world.objects()) box.expand(object.position());
  for (const TrajectoryPoint& point : result.ego_states) box.expand(point.position());
  for (const std::vector<RoadObject>& frame : result.object_history) {
    for (const RoadObject& object : frame) box.expand(object.position());
  }

  if (!box.valid()) {
    // 完全空的世界：给一个 20m 见方的默认视窗，保证出图仍然是合法 SVG
    box.expand({-10.0, -10.0});
    box.expand({10.0, 10.0});
    return box;
  }

  // 向外扩一点，避免贴边的车辆轮廓被裁掉半个
  const double pad = 3.0;
  box.min.x -= pad;
  box.min.y -= pad;
  box.max.x += pad;
  box.max.y += pad;
  return box;
}

/// 步号归一化：负数或越界一律取最后一步（复盘时最关心终态）
std::size_t resolveStep(const SimulationResult& result, int step) {
  if (result.ego_states.empty()) return 0;
  const std::size_t last = result.ego_states.size() - 1;
  if (step < 0) return last;
  const std::size_t requested = static_cast<std::size_t>(step);
  return std::min(requested, last);
}

/// 取某一步的其他物体；该步无记录时退回世界里的静态物体列表
const std::vector<RoadObject>* objectsAt(const SimulationResult& result, const World& world,
                                         std::size_t step) {
  if (step < result.object_history.size()) return &result.object_history[step];
  if (!world.objects().empty()) return &world.objects();
  return nullptr;
}

// ---------------------------------------------------------------------------
//  俯视图公共绘制块
// ---------------------------------------------------------------------------

void drawLane(SvgCanvas& canvas, const Lane& lane, const VisualizerConfig& config) {
  if (lane.centerline.size() < 2) return;

  const double half = std::max(lane.width, 0.5) * 0.5;

  // 沿中心线法向偏移出左右边界；法向由相邻点的切向旋转 90° 得到
  std::vector<Vec2> left;
  std::vector<Vec2> right;
  left.reserve(lane.centerline.size());
  right.reserve(lane.centerline.size());

  for (std::size_t i = 0; i < lane.centerline.size(); ++i) {
    const Vec2& prev = lane.centerline[i == 0 ? 0 : i - 1];
    const Vec2& next =
        lane.centerline[i + 1 < lane.centerline.size() ? i + 1 : lane.centerline.size() - 1];
    Vec2 tangent = (next - prev).normalized();
    if (tangent.norm() < kEpsilon) tangent = Vec2{1.0, 0.0};
    const Vec2 normal = tangent.perp() * half;
    left.push_back(lane.centerline[i] + normal);
    right.push_back(lane.centerline[i] - normal);
  }

  // 路面：用中心线两侧边界围成的多边形填充，使"哪里是路"一目了然
  std::vector<Vec2> surface = left;
  surface.insert(surface.end(), right.rbegin(), right.rend());
  Style fill;
  fill.stroke = "none";
  fill.fill = lane.is_junction ? "#FFF3E0" : "#ECEFF1";
  fill.opacity = 1.0;
  canvas.drawPolygon(surface, fill);

  const Style edge = Style::solid("#90A4AE", 1.4);
  canvas.drawPolyline(left, edge);
  canvas.drawPolyline(right, edge);

  if (config.draw_lane_centerlines) {
    canvas.drawPolyline(lane.centerline, Style::dashedLine("#FFB300", 1.0, 6.0));
  }
}

/// 画道路全貌（网格 + 车道 + 静态障碍）
void drawWorldBase(SvgCanvas& canvas, const World& world, const VisualizerConfig& config) {
  if (config.draw_grid) {
    canvas.drawGrid(config.grid_spacing, Style::solid("#E0E0E0", 0.6));
  }

  for (const Lane& lane : world.lanes()) drawLane(canvas, lane, config);

  const Style obstacle_style = boxStyle("#455A64", 1.2, 0.5);
  for (const Obb2& obstacle : world.obstacles()) canvas.drawObb(obstacle, obstacle_style);
}

/// 画其他物体（含 ID 标签与速度箭头）
void drawObjects(SvgCanvas& canvas, const std::vector<RoadObject>& objects,
                 const VisualizerConfig& config) {
  for (const RoadObject& object : objects) {
    const std::string color = objectColor(object.type);
    canvas.drawObb(object.obb(), boxStyle(color, 1.6));

    // 标签挂在物体正上方（世界坐标 Y 向上，故沿 +Y 偏移）
    const Vec2 label_pos = object.position() + Vec2{0.0, object.width * 0.5 + 1.6};
    canvas.drawLabel(label_pos, "#" + std::to_string(object.id), "#FFFFFF", color);

    if (config.draw_velocity_arrows && object.speed > 0.5) {
      const Vec2 head = object.position() + object.velocity * 1.0;
      canvas.drawArrow(object.position(), head, Style::solid(color, 1.5), 0.7);
    }
  }
}

/// 画自车：高亮轮廓 + 安全边界 + 速度箭头 + "EGO" 标签
void drawEgo(SvgCanvas& canvas, const TrajectoryPoint& point, const VisualizerConfig& config,
             double vehicle_length = 4.5, double vehicle_width = 1.8) {
  const Obb2 ego_box(point.pose(), vehicle_length, vehicle_width);

  if (config.draw_safety_margin && config.safety_margin > 0.0) {
    const Obb2 margin_box(point.pose(), vehicle_length + 2.0 * config.safety_margin,
                          vehicle_width + 2.0 * config.safety_margin);
    canvas.drawObb(margin_box, Style::dashedLine("#E53935", 1.2, 3.0));
  }

  canvas.drawObb(ego_box, boxStyle("#C2185B", 2.4, 0.7));
  canvas.drawLabel(point.position() + Vec2{0.0, vehicle_width * 0.5 + 1.8}, "EGO", "#C2185B",
                   "#FFFFFF");

  if (config.draw_velocity_arrows && point.v > 0.5) {
    const Vec2 dir{std::cos(point.theta), std::sin(point.theta)};
    // 箭头长度取"1 秒后的位置"，长度即速度，读图时可直接换算
    canvas.drawArrow(point.position(), point.position() + dir * point.v,
                     Style::solid("#00897B", 2.0), 0.8);
  }
}

/// 屏幕坐标定位：把文本放在画布像素 (x, y) 处。
/// 通过 toWorld 反解，避免为了 HUD 文本而重复实现一套坐标换算。
Vec2 screenToWorld(const SvgCanvas& canvas, double x, double y) {
  return canvas.toWorld({x, y});
}

/// 左上角角标：场景名 + 时刻
void drawHud(SvgCanvas& canvas, const SimulationResult& result, std::size_t step) {
  std::string title = result.scenario_name.empty() ? std::string("未命名场景")
                                                   : result.scenario_name;
  if (step < result.time.size()) title += "  t=" + fmt(result.time[step], 2) + "s";
  canvas.drawText(screenToWorld(canvas, 12.0, 20.0), title, 15.0, "#263238", "start");

  if (step < result.ego_states.size()) {
    const TrajectoryPoint& ego = result.ego_states[step];
    const std::string info = "v=" + fmt(ego.v, 2) + " m/s   a=" + fmt(ego.a, 2) +
                             " m/s²   κ=" + fmt(ego.kappa, 4) + " 1/m";
    canvas.drawText(screenToWorld(canvas, 12.0, 40.0), info, 13.0, "#37474F", "start");
  }
}

/// 事件标记：在自车轨迹上事件时刻对应的位置画一个菱形标记
void drawEventMarkers(SvgCanvas& canvas, const SimulationResult& result) {
  for (const SafetyEventRecord& record : result.events) {
    if (record.event == SafetyEvent::kNone) continue;

    // 找到事件时刻对应的轨迹点；时间序列为空时退化为按索引取
    std::size_t index = 0;
    bool found = false;
    for (std::size_t i = 0; i < result.time.size() && i < result.ego_states.size(); ++i) {
      if (result.time[i] >= record.time) {
        index = i;
        found = true;
        break;
      }
    }
    if (!found) {
      if (result.ego_states.empty()) continue;
      index = result.ego_states.size() - 1;
    }

    const Vec2 pos = result.ego_states[index].position();
    const std::string color = eventColor(record.event);
    const double radius = 1.2;

    canvas.drawCircle(pos, radius, boxStyle(color, 1.0, 0.9));
    canvas.drawLine(pos + Vec2{-radius, 0.0}, pos + Vec2{radius, 0.0},
                    Style::solid("#FFFFFF", 1.0));
    canvas.drawLine(pos + Vec2{0.0, -radius}, pos + Vec2{0.0, radius},
                    Style::solid("#FFFFFF", 1.0));

    std::string text = eventLabel(record.event);
    if (record.value > 0.0 && std::isfinite(record.value)) {
      text += "(" + fmt(record.value) + ")";
    }
    canvas.drawLabel(pos + Vec2{0.0, radius + 1.4}, text, "#FFFFFF", color);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
//  SimVisualizer
// ---------------------------------------------------------------------------

SimVisualizer::SimVisualizer(const VisualizerConfig& config) : config_(config) {}

std::string SimVisualizer::renderSnapshot(const SimulationResult& result, const World& world,
                                          int step) const {
  SvgCanvas canvas(config_.width, config_.height);
  canvas.setWorldBounds(computeBounds(world, result), 30.0);

  drawWorldBase(canvas, world, config_);

  const std::size_t index = resolveStep(result, step);
  if (const std::vector<RoadObject>* objects = objectsAt(result, world, index)) {
    drawObjects(canvas, *objects, config_);
  }

  if (!result.ego_states.empty()) {
    drawEgo(canvas, result.ego_states[index], config_);
  }
  drawHud(canvas, result, index);

  if (config_.draw_events) drawEventMarkers(canvas, result);

  return canvas.toString();
}

std::string SimVisualizer::renderOverview(const SimulationResult& result,
                                          const World& world) const {
  SvgCanvas canvas(config_.width, config_.height);
  canvas.setWorldBounds(computeBounds(world, result), 30.0);

  drawWorldBase(canvas, world, config_);

  const std::size_t index = resolveStep(result, -1);
  if (const std::vector<RoadObject>* objects = objectsAt(result, world, index)) {
    drawObjects(canvas, *objects, config_);
  }

  // 规划路径：虚线，与"实际行驶轨迹"区分开，
  // 两者的分叉处往往就是决策失误点。
  if (result.planned_paths.size() >= 2) {
    std::vector<Vec2> path;
    path.reserve(result.planned_paths.size());
    for (const TrajectoryPoint& point : result.planned_paths) path.push_back(point.position());
    canvas.drawPolyline(path, Style::dashedLine("#00838F", 2.0, 5.0));
  }

  // 整条自车轨迹按曲率着色：一眼看出哪里不平滑
  if (result.ego_states.size() >= 2) {
    canvas.drawTrajectoryColored(result.ego_states, result.max_curvature, 2.5);
  }

  if (!result.ego_states.empty()) {
    drawEgo(canvas, result.ego_states[index], config_);
  }
  drawHud(canvas, result, index);

  if (config_.draw_events) drawEventMarkers(canvas, result);

  // 图例
  const double legend_y = 64.0;
  canvas.drawLine(screenToWorld(canvas, 14.0, legend_y),
                  screenToWorld(canvas, 44.0, legend_y), Style::solid("#2E7D32", 3.0));
  canvas.drawText(screenToWorld(canvas, 50.0, legend_y), "轨迹按曲率着色：绿=平直，红=接近 max κ",
                  12.0, "#37474F", "start");
  canvas.drawLine(screenToWorld(canvas, 14.0, legend_y + 16.0),
                  screenToWorld(canvas, 44.0, legend_y + 16.0),
                  Style::solid("#00838F", 2.0));
  canvas.drawText(screenToWorld(canvas, 50.0, legend_y + 16.0), "规划路径（虚线：规划，实线：实际）",
                  12.0, "#37474F", "start");

  return canvas.toString();
}

std::string SimVisualizer::renderTimeline(const SimulationResult& result) const {
  const double canvas_w = config_.width;
  const double canvas_h = config_.height;

  SvgCanvas canvas(canvas_w, canvas_h);
  // 时序图完全在屏幕坐标系里绘制：把世界包围盒设为整块画布、margin 取 0，
  // 此时 scale == 1，toCanvas 退化为 (x, H - y) 的纯翻转，
  // 于是所有像素坐标都可以通过 P() 精确落位，同时复用 SvgCanvas 的图元与转义。
  BoundingBox2 pixel_bounds;
  pixel_bounds.expand({0.0, 0.0});
  pixel_bounds.expand({canvas_w, canvas_h});
  canvas.setWorldBounds(pixel_bounds, 0.0);
  const double H = canvas_h;
  auto P = [H](double x, double y) { return Vec2{x, H - y}; };

  const double left = 92.0;
  const double right = canvas_w - 36.0;
  const double top = 64.0;
  const double bottom = canvas_h - 46.0;
  const double gap = 12.0;
  const double plot_h = std::max((bottom - top - 3.0 * gap) / 4.0, 10.0);
  const double plot_w = std::max(right - left, 10.0);

  canvas.drawText(P(12.0, 24.0),
                  result.scenario_name.empty() ? "仿真时序图" : "仿真时序图 — " + result.scenario_name,
                  16.0, "#263238", "start");
  canvas.drawText(P(12.0, 44.0),
                  "横轴：时间(s)　纵轴：各子图独立归一化，刻度已标注；红色竖线为安全事件",
                  12.0, "#546E7A", "start");

  // ---- 组装四条曲线 ----
  const std::size_t count = std::min(result.time.size(), result.ego_states.size());

  struct Series {
    std::string title;
    std::string color;
    std::vector<double> t;
    std::vector<double> v;
  };

  Series speed{"速度 (m/s)", "#1E88E5", {}, {}};
  Series accel{"加速度 (m/s²)", "#E53935", {}, {}};
  Series curvature{"曲率 (1/m)", "#8E24AA", {}, {}};
  Series ttc{"TTC (s)", "#00897B", {}, {}};

  bool has_accel = false;
  bool has_kappa = false;
  for (std::size_t i = 0; i < count; ++i) {
    if (std::fabs(result.ego_states[i].a) > 1e-6) has_accel = true;
    if (std::fabs(result.ego_states[i].kappa) > 1e-9) has_kappa = true;
  }

  for (std::size_t i = 0; i < count; ++i) {
    const TrajectoryPoint& point = result.ego_states[i];
    const double time = result.time[i];

    speed.t.push_back(time);
    speed.v.push_back(point.v);

    // 加速度优先取记录值；未记录时用相邻速度差分（真实时间差，防除零）
    double a = point.a;
    if (!has_accel && i > 0) {
      const double dt = time - result.time[i - 1];
      if (dt > kEpsilon) a = (point.v - result.ego_states[i - 1].v) / dt;
    }
    accel.t.push_back(time);
    accel.v.push_back(a);

    // 曲率同理：优先取记录值，缺失时用三点几何曲率
    double kappa = std::fabs(point.kappa);
    if (!has_kappa && i > 0 && i + 1 < count) {
      const Vec2& a0 = result.ego_states[i - 1].position();
      const Vec2& b0 = point.position();
      const Vec2& c0 = result.ego_states[i + 1].position();
      const double area2 = (b0 - a0).cross(c0 - a0);
      const double denom = (b0 - a0).norm() * (c0 - b0).norm() * (c0 - a0).norm();
      kappa = denom > kEpsilon ? std::fabs(2.0 * area2 / denom) : 0.0;
    }
    curvature.t.push_back(time);
    curvature.v.push_back(kappa);
  }

  // TTC 没有逐步记录，只能来自安全事件；无事件时退化为"全程最小 TTC"参考线
  for (const SafetyEventRecord& record : result.events) {
    if (record.event != SafetyEvent::kLowTtc && record.event != SafetyEvent::kNearMiss) continue;
    ttc.t.push_back(record.time);
    ttc.v.push_back(record.value);
  }

  const std::vector<Series*> series = {&speed, &accel, &curvature, &ttc};

  // 时间轴范围：所有子图共用，保证四条曲线横向可比
  double t_min = 0.0;
  double t_max = 1.0;
  if (!speed.t.empty()) {
    t_min = speed.t.front();
    t_max = speed.t.back();
  } else if (!result.time.empty()) {
    t_min = result.time.front();
    t_max = result.time.back();
  }
  if (t_max - t_min < kEpsilon) t_max = t_min + 1.0;

  // ---- 逐个子图绘制 ----
  for (std::size_t s = 0; s < series.size(); ++s) {
    const Series& data = *series[s];
    const double y0 = top + static_cast<double>(s) * (plot_h + gap);
    const double y1 = y0 + plot_h;

    // 子图底板与边框
    Style frame;
    frame.stroke = "#B0BEC5";
    frame.fill = "#FAFAFA";
    frame.stroke_width = 1.0;
    canvas.drawRectangle(P(left, y1), plot_w, plot_h, frame);

    // 纵轴范围：数据为空时给一个占位范围，避免除零
    const bool is_ttc_plot = (s + 1 == series.size());
    double v_min = 0.0;
    double v_max = 1.0;
    if (!data.v.empty()) {
      v_min = *std::min_element(data.v.begin(), data.v.end());
      v_max = *std::max_element(data.v.begin(), data.v.end());
      if (!std::isfinite(v_min) || !std::isfinite(v_max)) {
        v_min = 0.0;
        v_max = 1.0;
      }
    }
    // TTC 只有事件级采样：把"全程最小 TTC"纳入量程，参考线才有意义
    if (is_ttc_plot && result.min_ttc < 1e8) {
      if (data.v.empty()) {
        v_min = std::max(0.0, result.min_ttc - 1.0);
        v_max = result.min_ttc + 1.0;
      } else {
        v_min = std::min(v_min, result.min_ttc);
        v_max = std::max(v_max, result.min_ttc);
      }
    }
    if (v_max - v_min < 1e-6) {
      v_min -= 0.5;
      v_max += 0.5;
    }

    auto to_px = [&](double t, double v) {
      const double px = left + (t - t_min) / (t_max - t_min) * plot_w;
      const double py = y1 - (v - v_min) / (v_max - v_min) * plot_h;
      return P(px, py);
    };

    // 纵向刻度：5 条水平网格线，并标出数值（右端给出该刻度对应的物理量）
    const int ticks = 4;
    for (int k = 0; k <= ticks; ++k) {
      const double ratio = static_cast<double>(k) / static_cast<double>(ticks);
      const double v = v_min + (v_max - v_min) * ratio;
      const double py = y1 - ratio * plot_h;
      canvas.drawLine(P(left, py), P(right, py), Style::solid("#ECEFF1", 1.0));
      canvas.drawText(P(left - 6.0, py), fmt(v, 2), 10.0, "#546E7A", "end");
    }

    // 横轴刻度：每个子图都画竖网格线，只有最下方子图标注时间，避免重复噪声
    for (int k = 0; k <= ticks; ++k) {
      const double ratio = static_cast<double>(k) / static_cast<double>(ticks);
      const double px = left + ratio * plot_w;
      canvas.drawLine(P(px, y0), P(px, y1), Style::solid("#ECEFF1", 1.0));
      if (s + 1 == series.size()) {
        const double t = t_min + (t_max - t_min) * ratio;
        canvas.drawText(P(px, bottom + 14.0), fmt(t, 1) + "s", 10.0, "#546E7A", "middle");
      }
    }

    // 曲线本体
    if (data.v.size() >= 2) {
      std::vector<Vec2> polyline;
      polyline.reserve(data.v.size());
      for (std::size_t i = 0; i < data.v.size(); ++i) {
        polyline.push_back(to_px(data.t[i], data.v[i]));
      }
      canvas.drawPolyline(polyline, Style::solid(data.color, 2.0));
    } else {
      // 只有一个采样点时折线退化为不可见，改用标记点，否则子图看起来是空的
      for (std::size_t i = 0; i < data.v.size(); ++i) {
        canvas.drawCircle(to_px(data.t[i], data.v[i]), 0.35,
                          Style::filled(data.color, 0.9));
      }
    }

    // 子图标题 + 纵轴范围（工程师定位问题时最需要的两个数）。
    // 用带底衬的标签，避免与刻度线、刻度值糊在一起。
    const std::string stats =
        data.v.empty() ? std::string("　无数据")
                       : "　轴范围 max=" + fmt(v_max, 2) + " min=" + fmt(v_min, 2);
    canvas.drawLabel(P(left + 6.0, y0 + 23.0), data.title + stats, "#FFFFFF", data.color);

    // TTC 通常只有事件级采样，不足两点时画出"全程最小 TTC"参考线，
    // 否则这个子图读不出任何危险程度。
    if (is_ttc_plot && data.v.size() < 2 && result.min_ttc < 1e8) {
      const double py = y1 - (result.min_ttc - v_min) / (v_max - v_min) * plot_h;
      canvas.drawLine(P(left, py), P(right, py), Style::dashedLine("#00897B", 1.5, 5.0));
      canvas.drawText(P(right - 8.0, py - 10.0), "全程最小 TTC " + fmt(result.min_ttc) + "s",
                      11.0, "#00897B", "end");
    }
  }

  // ---- 安全事件：贯穿全图的竖线 + 文字标注 ----
  if (config_.draw_events) {
    double last_label_x = -1e9;
    int row = 0;
    for (const SafetyEventRecord& record : result.events) {
      if (record.event == SafetyEvent::kNone) continue;

      const double ratio = (record.time - t_min) / (t_max - t_min);
      const double px = left + clamp(ratio, 0.0, 1.0) * plot_w;
      const std::string color = eventColor(record.event);

      canvas.drawLine(P(px, top), P(px, bottom), Style::dashedLine(color, 1.6, 4.0));

      // 标签错行排布，事件密集时不至于糊成一团
      if (px - last_label_x < 90.0) {
        row = (row + 1) % 2;
      } else {
        row = 0;
      }
      last_label_x = px;

      std::string text = eventLabel(record.event);
      if (std::isfinite(record.value) && record.value > 0.0) {
        text += " " + fmt(record.value);
      }
      // 追加自由文本描述：事件类型只说明"发生了什么类别"，
      // 描述才写清了"具体怎么回事"，复盘时后者往往更关键
      if (!record.description.empty()) {
        text += " — " + record.description;
      }
      canvas.drawText(P(px, top - 24.0 + static_cast<double>(row) * 14.0), text, 11.0, color,
                      "middle");
    }
  }

  return canvas.toString();
}

bool SimVisualizer::writeReport(const SimulationResult& result, const World& world,
                                const std::string& output_path) const {
  const std::string overview = renderOverview(result, world);
  const std::string timeline = renderTimeline(result);

  const double w = config_.width;
  const double h = config_.height;
  const double header = 34.0;

  // 拼成一张大 SVG：两幅图各自保留自己的 viewBox，按 x/y/width/height 摆放，
  // 这样单一文件即可包含俯视图与时序图，便于归档与 diff。
  std::ostringstream oss;
  oss << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
  oss << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"" << fmt(w, 0)
      << "\" height=\"" << fmt(header + 2.0 * h, 0) << "\" viewBox=\"0 0 " << fmt(w, 0) << ' '
      << fmt(header + 2.0 * h, 0) << "\">\n";
  oss << "<rect x=\"0\" y=\"0\" width=\"" << fmt(w, 0) << "\" height=\""
      << fmt(header + 2.0 * h, 0) << "\" fill=\"#FFFFFF\"/>\n";
  oss << "<text x=\"12\" y=\"" << fmt(header * 0.68, 1)
      << "\" font-family=\"sans-serif\" font-size=\"17\" fill=\"#263238\">"
      << escapeXml(result.scenario_name.empty() ? std::string("ADSim 仿真报告")
                                                : "ADSim 仿真报告 — " + result.scenario_name)
      << "</text>\n";
  oss << "<text x=\"" << fmt(w - 12.0, 0) << "\" y=\"" << fmt(header * 0.68, 1)
      << "\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#78909C\" "
         "text-anchor=\"end\">"
      << "碰撞 " << result.collision_count << " 次　near-miss " << result.near_miss_count
      << " 次　min TTC " << fmt(result.min_ttc) << "s　min gap " << fmt(result.min_distance)
      << "m</text>\n";

  oss << "<svg x=\"0\" y=\"" << fmt(header, 0) << "\" width=\"" << fmt(w, 0) << "\" height=\""
      << fmt(h, 0) << "\" viewBox=\"0 0 " << fmt(w, 0) << ' ' << fmt(h, 0) << "\">\n"
      << svgInner(overview) << "\n</svg>\n";

  oss << "<svg x=\"0\" y=\"" << fmt(header + h, 0) << "\" width=\"" << fmt(w, 0)
      << "\" height=\"" << fmt(h, 0) << "\" viewBox=\"0 0 " << fmt(w, 0) << ' ' << fmt(h, 0)
      << "\">\n"
      << svgInner(timeline) << "\n</svg>\n";

  oss << "</svg>\n";

  std::ofstream out(output_path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) return false;
  const std::string document = oss.str();
  out.write(document.data(), static_cast<std::streamsize>(document.size()));
  out.flush();
  return out.good();
}

std::string SimVisualizer::renderReplayComparison(const std::vector<AlignedFrame>& frames,
                                                  const SimulationResult& result,
                                                  const World& world) const {
  SvgCanvas canvas(config_.width, config_.height);

  // 包围盒要同时容纳仿真轨迹与回放轨迹，否则回放偏出去就看不见了
  BoundingBox2 box = computeBounds(world, result);
  std::vector<Vec2> replay;
  replay.reserve(frames.size());
  for (const AlignedFrame& frame : frames) {
    if (!frame.has_odom) continue;  // 未对齐到车辆状态的帧不参与对比
    const Vec2 p = frame.odom.pose().position();
    replay.push_back(p);
    box.expand(p);
  }
  if (!box.valid()) {
    box.expand({-10.0, -10.0});
    box.expand({10.0, 10.0});
  }
  canvas.setWorldBounds(box, 30.0);

  drawWorldBase(canvas, world, config_);

  // 仿真轨迹：蓝色实线；回放轨迹：橙色实线。两者用颜色而非线型区分，
  // 是因为线型在缩略图上不可辨，而颜色可以。
  if (result.ego_states.size() >= 2) {
    std::vector<Vec2> sim;
    sim.reserve(result.ego_states.size());
    for (const TrajectoryPoint& point : result.ego_states) sim.push_back(point.position());
    canvas.drawPolyline(sim, Style::solid("#1565C0", 2.5));
  }
  if (replay.size() >= 2) {
    canvas.drawPolyline(replay, Style::solid("#EF6C00", 2.5));
  }

  // 起点/终点标记：核对方向是否一致（回放数据可能被倒序处理过）
  if (!result.ego_states.empty()) {
    canvas.drawCircle(result.ego_states.front().position(), 0.8,
                      boxStyle("#1565C0", 1.0, 0.9));
    canvas.drawCircle(result.ego_states.back().position(), 0.8, boxStyle("#1565C0", 1.0, 0.9));
  }
  if (replay.size() >= 2) {
    canvas.drawCircle(replay.front(), 0.8, boxStyle("#EF6C00", 1.0, 0.9));
    canvas.drawCircle(replay.back(), 0.8, boxStyle("#EF6C00", 1.0, 0.9));
  }

  // 图例
  const double legend_y = 24.0;
  auto legendItem = [&](double y, const std::string& color, const std::string& text) {
    canvas.drawLine(screenToWorld(canvas, 14.0, y), screenToWorld(canvas, 44.0, y),
                    Style::solid(color, 3.0));
    canvas.drawText(screenToWorld(canvas, 50.0, y), text, 12.0, "#37474F", "start");
  };
  legendItem(legend_y, "#1565C0", "仿真轨迹");
  legendItem(legend_y + 16.0, "#EF6C00", "回放轨迹（清洗后路测数据）");

  canvas.drawText(screenToWorld(canvas, 12.0, 44.0 + 16.0),
                  result.scenario_name.empty() ? "回放 vs 仿真对比" : result.scenario_name,
                  15.0, "#263238", "start");

  // 平均偏差：直接回答"仿真是否忠实还原了真实场景"
  if (replay.size() >= 2 && result.ego_states.size() >= 2) {
    double sum = 0.0;
    std::size_t samples = 0;
    const std::size_t stride = std::max<std::size_t>(1, replay.size() / 200);
    for (std::size_t i = 0; i < replay.size(); i += stride) {
      double best = 1e18;
      for (const TrajectoryPoint& point : result.ego_states) {
        best = std::min(best, (point.position() - replay[i]).norm());
      }
      sum += best;
      ++samples;
    }
    const std::string text = samples > 0
                                 ? "回放点到仿真轨迹的平均偏差 " + fmt(sum / samples) + " m"
                                 : std::string();
    if (!text.empty()) {
      canvas.drawText(screenToWorld(canvas, 12.0, 66.0 + 16.0), text, 12.0, "#546E7A", "start");
    }
  } else if (replay.size() < 2) {
    canvas.drawText(screenToWorld(canvas, 12.0, 66.0 + 16.0), "无有效回放轨迹（缺 odom）", 12.0,
                    "#B71C1C", "start");
  }

  return canvas.toString();
}

// ---------------------------------------------------------------------------
//  HTML 报告
// ---------------------------------------------------------------------------

std::string buildHtmlReport(const std::vector<std::pair<std::string, std::string>>& sections,
                            const std::string& title) {
  std::ostringstream oss;
  oss << "<!DOCTYPE html>\n"
      << "<html lang=\"zh-CN\">\n<head>\n"
      << "<meta charset=\"UTF-8\">\n"
      << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
      << "<title>" << escapeXml(title) << "</title>\n"
      << "<style>\n"
      << "  body { margin: 0; padding: 24px; background: #F5F7FA; color: #263238;\n"
      << "         font-family: -apple-system, 'Segoe UI', 'Noto Sans CJK SC', sans-serif; }\n"
      << "  h1 { font-size: 22px; margin: 0 0 4px 0; }\n"
      << "  .meta { color: #78909C; font-size: 13px; margin-bottom: 20px; }\n"
      << "  section { background: #FFFFFF; border-radius: 8px; padding: 16px;\n"
      << "            margin-bottom: 20px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); }\n"
      << "  h2 { font-size: 16px; margin: 0 0 12px 0; color: #37474F; }\n"
      << "  svg { max-width: 100%; height: auto; display: block; }\n"
      << "</style>\n</head>\n<body>\n"
      << "<h1>" << escapeXml(title) << "</h1>\n"
      << "<div class=\"meta\">由 ADSim 自动生成 —— 单文件报告，内嵌全部 SVG，可直接归档</div>\n";

  for (const std::pair<std::string, std::string>& section : sections) {
    oss << "<section>\n<h2>" << escapeXml(section.first) << "</h2>\n"
        // SVG 是自由文本里的唯一可信来源（本工程自产），直接内嵌；
        // XML 声明必须去掉，否则出现在 HTML 正文中间是非法的。
        << stripXmlDeclaration(section.second) << "\n</section>\n";
  }

  oss << "</body>\n</html>\n";
  return oss.str();
}

}  // namespace adsim
