// =============================================================================
//  SvgCanvas.h — 轻量二维绘图画布（SVG 输出）
//
//  可视化工具的核心诉求是"让算法工程师直观复盘决策失误点"。为此需要：
//    * 世界坐标系与画布坐标系的自动变换（含等比缩放与 Y 轴翻转）
//    * 语义化的图元：车道、车辆轮廓、轨迹、规划路径、安全边界
//    * 零依赖、可离线产出——SVG 是纯文本，便于归档、diff 与嵌入报告
//
//  同一套绘制逻辑同时服务于两条渲染路径：
//    * SvgCanvas        —— 离线出图，无需图形环境
//    * QtVisualizer     —— 在线交互，见 QtVisualizer.h
// =============================================================================
#pragma once

#include "adsim/common/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace adsim {

/// 绘图样式
struct Style {
  std::string stroke{"#000000"};
  std::string fill{"none"};
  double stroke_width{1.0};
  double opacity{1.0};
  bool dashed{false};
  double dash_length{4.0};

  static Style solid(const std::string& color, double width = 1.0);
  static Style filled(const std::string& color, double opacity = 1.0);
  static Style dashedLine(const std::string& color, double width = 1.0,
                          double dash = 4.0);
};

class SvgCanvas {
 public:
  /// @param width  画布宽度（像素）
  /// @param height 画布高度（像素）
  SvgCanvas(double width, double height);

  double width() const { return width_; }
  double height() const { return height_; }

  /// 设置世界坐标范围。画布会按等比缩放适配，并留出 margin 边距。
  /// Y 轴自动翻转（世界坐标 Y 向上，SVG Y 向下）。
  void setWorldBounds(const BoundingBox2& bounds, double margin = 20.0);

  /// 当前缩放系数（像素 / 米）
  double scale() const { return scale_; }

  /// 世界坐标 → 画布坐标
  Vec2 toCanvas(const Vec2& world) const;

  /// 画布坐标 → 世界坐标
  Vec2 toWorld(const Vec2& canvas) const;

  // -------------------------------------------------------------------------
  // 图元
  // -------------------------------------------------------------------------

  void drawLine(const Vec2& a, const Vec2& b, const Style& style);
  void drawPolyline(const std::vector<Vec2>& points, const Style& style);
  void drawPolygon(const std::vector<Vec2>& points, const Style& style);
  void drawCircle(const Vec2& center, double radius, const Style& style);
  void drawRectangle(const Vec2& corner, double width, double height, const Style& style);

  /// 有向包围盒（车辆轮廓）
  void drawObb(const Obb2& box, const Style& style);

  /// 文本框。anchor 取 "start" / "middle" / "end"。
  void drawText(const Vec2& position, const std::string& text, double font_size,
                const std::string& color, const std::string& anchor = "start");

  /// 带背景的标签，用于标注车辆 ID、事件等
  void drawLabel(const Vec2& position, const std::string& text,
                 const std::string& background = "#FFFFFF",
                 const std::string& foreground = "#000000");

  /// 箭头（用于表示速度方向）
  void drawArrow(const Vec2& from, const Vec2& to, const Style& style, double head_size = 0.6);

  /// 世界坐标下的背景网格
  void drawGrid(double spacing, const Style& style);
  void drawAxes(double length);

  /// 按曲率着色绘制轨迹：曲率越大的段颜色越暖。
  /// 用于一眼看出"路径在哪里不平滑、曲率在哪里突变"。
  void drawTrajectoryColored(const Trajectory& trajectory, double max_kappa,
                             double stroke_width = 2.5);

  /// 原始输出
  std::string toString() const;
  bool save(const std::string& path) const;

 private:
  std::string escape(const std::string& text) const;

  double width_;
  double height_;
  double scale_{1.0};
  double margin_{20.0};
  BoundingBox2 world_bounds_;
  bool has_bounds_{false};

  std::vector<std::string> elements_;
};

}  // namespace adsim
