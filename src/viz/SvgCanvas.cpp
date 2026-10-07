// =============================================================================
//  SvgCanvas.cpp — 轻量二维绘图画布（SVG 输出）
//
//  实现要点：
//    * 世界坐标 → 画布坐标采用**等比缩放**（取 x/y 缩放系数的较小者），
//      否则车辆轮廓会被拉成平行四边形，"看起来像什么"就不再可信；
//    * Y 轴必须翻转：世界坐标 Y 向上，SVG 画布 Y 向下；
//    * 所有数值输出统一走 num()，保留 3 位小数并去掉尾随 0，
//      否则一份报告里会塞满 "12.000000000000002" 这类无意义的浮点串；
//    * 所有文本统一走 escape()，否则文本里出现 & < > " ' 会直接产出非法 XML。
// =============================================================================
#include "adsim/viz/SvgCanvas.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace adsim {

namespace {

/// 浮点数 → SVG 属性串。统一控制有效位数，并剔除 NaN / Inf，
/// 避免非法数值把整份 SVG 变成浏览器拒绝渲染的坏文档。
std::string num(double value) {
  if (!std::isfinite(value)) return "0";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << value;
  std::string s = oss.str();
  const std::size_t dot = s.find('.');
  if (dot != std::string::npos) {
    s.erase(s.find_last_not_of('0') + 1);
    if (!s.empty() && s.back() == '.') s.pop_back();
  }
  if (s == "-0") s = "0";
  return s;
}

/// 颜色通道 → 两位十六进制
std::string hexByte(double channel) {
  const int v = static_cast<int>(clamp(std::lround(channel), 0L, 255L));
  static const char* kDigits = "0123456789ABCDEF";
  std::string out(2, '0');
  out[0] = kDigits[(v >> 4) & 0x0F];
  out[1] = kDigits[v & 0x0F];
  return out;
}

/// 由归一化曲率 t∈[0,1] 取颜色：冷色（绿）→ 暖色（红）。
/// 之所以做成 绿→黄→红 的三段线性插值而不是 绿→红 的两段，
/// 是因为中途的黄能给出一个明确的"警戒"参照，读图时更容易分辨量级。
std::string curvatureColor(double t) {
  struct Rgb {
    double r;
    double g;
    double b;
  };
  const Rgb cold{46.0, 125.0, 50.0};    // #2E7D32 绿：小曲率（直行）
  const Rgb warm{249.0, 168.0, 37.0};   // #F9A825 黄：中曲率
  const Rgb hot{198.0, 40.0, 40.0};     // #C62828 红：接近 max_kappa

  const double u = clamp(t, 0.0, 1.0);
  const Rgb& a = u < 0.5 ? cold : warm;
  const Rgb& b = u < 0.5 ? warm : hot;
  const double k = u < 0.5 ? u * 2.0 : (u - 0.5) * 2.0;

  return "#" + hexByte(a.r + (b.r - a.r) * k) + hexByte(a.g + (b.g - a.g) * k) +
         hexByte(a.b + (b.b - a.b) * k);
}

/// 粗略估算文本像素宽度：ASCII 约 0.6 倍字号，非 ASCII（中文等）按 1 倍字号计。
/// 只用于给标签画底框，不追求排版精度，但必须避免中文标签底框过窄。
double estimateTextWidth(const std::string& text, double font_size) {
  double width = 0.0;
  for (std::size_t i = 0; i < text.size();) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    if (c < 0x80) {
      width += font_size * 0.6;
      i += 1;
    } else if ((c & 0xE0) == 0xC0) {
      width += font_size;
      i += 2;
    } else if ((c & 0xF0) == 0xE0) {
      width += font_size;
      i += 3;
    } else {
      width += font_size;
      i += 4;
    }
  }
  return width;
}

/// 把 Style 转成 SVG 描边/填充属性串
std::string styleAttrs(const Style& style) {
  std::ostringstream oss;
  oss << " stroke=\"" << style.stroke << "\""
      << " stroke-width=\"" << num(style.stroke_width) << "\""
      << " fill=\"" << style.fill << "\"";
  if (style.opacity < 1.0) oss << " opacity=\"" << num(style.opacity) << "\"";
  if (style.dashed) {
    oss << " stroke-dasharray=\"" << num(style.dash_length) << ","
        << num(style.dash_length) << "\"";
  }
  return oss.str();
}

/// 输出一条线段。round_cap 用于相邻线段首尾相接的轨迹——
/// 斜接（butt）在转折处会露出缺口，看起来像轨迹断了一节。
void appendLine(std::vector<std::string>& out, const Vec2& a, const Vec2& b,
                const Style& style, bool round_cap) {
  const Vec2 p0 = a;
  const Vec2 p1 = b;
  std::ostringstream oss;
  oss << "<line x1=\"" << num(p0.x) << "\" y1=\"" << num(p0.y) << "\" x2=\"" << num(p1.x)
      << "\" y2=\"" << num(p1.y) << "\"" << styleAttrs(style);
  if (round_cap) oss << " stroke-linecap=\"round\"";
  oss << "/>";
  out.push_back(oss.str());
}

/// 三点定圆的曲率（带符号）。用于轨迹点未填 kappa 时按几何退化估算，
/// 保证"曲率着色"在数据不完整时依然可用而不是整条轨迹一个颜色。
double geometricCurvature(const Vec2& a, const Vec2& b, const Vec2& c) {
  const double area2 = (b - a).cross(c - a);
  const double la = (b - a).norm();
  const double lb = (c - b).norm();
  const double lc = (c - a).norm();
  const double denom = la * lb * lc;
  if (denom < kEpsilon) return 0.0;
  return 2.0 * area2 / denom;
}

}  // namespace

// ---------------------------------------------------------------------------
//  Style
// ---------------------------------------------------------------------------

Style Style::solid(const std::string& color, double width) {
  Style s;
  s.stroke = color;
  s.fill = "none";
  s.stroke_width = width;
  return s;
}

Style Style::filled(const std::string& color, double opacity) {
  Style s;
  s.stroke = color;
  s.fill = color;
  s.opacity = opacity;
  return s;
}

Style Style::dashedLine(const std::string& color, double width, double dash) {
  Style s;
  s.stroke = color;
  s.fill = "none";
  s.stroke_width = width;
  s.dashed = true;
  s.dash_length = dash > kEpsilon ? dash : 4.0;
  return s;
}

// ---------------------------------------------------------------------------
//  坐标变换
// ---------------------------------------------------------------------------

SvgCanvas::SvgCanvas(double width, double height)
    : width_(width > kEpsilon ? width : 1.0), height_(height > kEpsilon ? height : 1.0) {}

void SvgCanvas::setWorldBounds(const BoundingBox2& bounds, double margin) {
  margin_ = std::max(0.0, margin);
  world_bounds_ = bounds;

  // 退化包围盒（所有点重合、或 min>max）会让缩放系数变成 0 或无穷，
  // 这里统一兜底成 1m × 1m 的窗口，保证后续除法安全。
  double span_x = bounds.max.x - bounds.min.x;
  double span_y = bounds.max.y - bounds.min.y;
  if (!std::isfinite(span_x) || span_x < kEpsilon) span_x = 1.0;
  if (!std::isfinite(span_y) || span_y < kEpsilon) span_y = 1.0;

  const double avail_w = std::max(width_ - 2.0 * margin_, 1.0);
  const double avail_h = std::max(height_ - 2.0 * margin_, 1.0);

  // 取较小值 → 等比缩放，长短边都不会被拉变形
  scale_ = std::min(avail_w / span_x, avail_h / span_y);
  if (!std::isfinite(scale_) || scale_ < kEpsilon) scale_ = 1.0;

  has_bounds_ = true;
}

Vec2 SvgCanvas::toCanvas(const Vec2& world) const {
  const Vec2 center = has_bounds_ ? world_bounds_.center() : Vec2{0.0, 0.0};
  // Y 取负号即完成翻转：世界 Y 越大 → 画布 Y 越小（越靠上）
  return {width_ * 0.5 + (world.x - center.x) * scale_,
          height_ * 0.5 - (world.y - center.y) * scale_};
}

Vec2 SvgCanvas::toWorld(const Vec2& canvas) const {
  const Vec2 center = has_bounds_ ? world_bounds_.center() : Vec2{0.0, 0.0};
  return {center.x + (canvas.x - width_ * 0.5) / scale_,
          center.y - (canvas.y - height_ * 0.5) / scale_};
}

// ---------------------------------------------------------------------------
//  图元
// ---------------------------------------------------------------------------

void SvgCanvas::drawLine(const Vec2& a, const Vec2& b, const Style& style) {
  appendLine(elements_, toCanvas(a), toCanvas(b), style, false);
}

void SvgCanvas::drawPolyline(const std::vector<Vec2>& points, const Style& style) {
  if (points.size() < 2) return;  // 单点无法构成折线，静默跳过

  std::ostringstream oss;
  oss << "<polyline points=\"";
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Vec2 p = toCanvas(points[i]);
    if (i != 0) oss << ' ';
    oss << num(p.x) << ',' << num(p.y);
  }
  oss << "\"" << styleAttrs(style) << " stroke-linejoin=\"round\"/>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawPolygon(const std::vector<Vec2>& points, const Style& style) {
  if (points.size() < 3) return;  // 少于三点构不成多边形

  std::ostringstream oss;
  oss << "<polygon points=\"";
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Vec2 p = toCanvas(points[i]);
    if (i != 0) oss << ' ';
    oss << num(p.x) << ',' << num(p.y);
  }
  oss << "\"" << styleAttrs(style) << " stroke-linejoin=\"round\"/>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawCircle(const Vec2& center, double radius, const Style& style) {
  if (radius <= 0.0 || !std::isfinite(radius)) return;
  const Vec2 c = toCanvas(center);
  std::ostringstream oss;
  oss << "<circle cx=\"" << num(c.x) << "\" cy=\"" << num(c.y) << "\" r=\""
      << num(radius * scale_) << "\"" << styleAttrs(style) << "/>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawRectangle(const Vec2& corner, double width, double height,
                              const Style& style) {
  if (width <= 0.0 || height <= 0.0) return;

  // 世界坐标下 (corner.x, corner.y) 是左下角；Y 翻转后画布上的左上角是
  // (corner.x, corner.y + height)，故这里显式换算而不是直接复用 corner。
  const Vec2 top_left = toCanvas({corner.x, corner.y + height});
  const double w = width * scale_;
  const double h = height * scale_;

  std::ostringstream oss;
  oss << "<rect x=\"" << num(top_left.x) << "\" y=\"" << num(top_left.y) << "\" width=\""
      << num(w) << "\" height=\"" << num(h) << "\"" << styleAttrs(style) << "/>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawObb(const Obb2& box, const Style& style) {
  if (box.length <= 0.0 || box.width <= 0.0) return;
  drawPolygon(box.corners(), style);
}

void SvgCanvas::drawText(const Vec2& position, const std::string& text, double font_size,
                         const std::string& color, const std::string& anchor) {
  const Vec2 p = toCanvas(position);
  std::ostringstream oss;
  oss << "<text x=\"" << num(p.x) << "\" y=\"" << num(p.y) << "\" font-size=\""
      << num(font_size) << "\" font-family=\"sans-serif\" fill=\"" << color
      << "\" text-anchor=\"" << escape(anchor) << "\" dominant-baseline=\"middle\">"
      << escape(text) << "</text>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawLabel(const Vec2& position, const std::string& text,
                          const std::string& background, const std::string& foreground) {
  // 标签尺寸随画布尺寸轻微缩放，保证 4K 导出与缩略图都还读得清
  const double font_size = clamp(width_ * 0.011, 9.0, 18.0);
  const double padding = 3.0;
  const double box_w = estimateTextWidth(text, font_size) + 2.0 * padding;
  const double box_h = font_size + 2.0 * padding;

  const Vec2 p = toCanvas(position);

  std::ostringstream rect;
  rect << "<rect x=\"" << num(p.x) << "\" y=\"" << num(p.y - box_h) << "\" width=\""
       << num(box_w) << "\" height=\"" << num(box_h)
       << "\" fill=\"" << background << "\" fill-opacity=\"0.85\" stroke=\"" << foreground
       << "\" stroke-width=\"0.5\"/>";
  elements_.push_back(rect.str());

  std::ostringstream oss;
  oss << "<text x=\"" << num(p.x + padding) << "\" y=\"" << num(p.y - box_h * 0.5)
      << "\" font-size=\"" << num(font_size)
      << "\" font-family=\"sans-serif\" fill=\"" << foreground
      << "\" text-anchor=\"start\" dominant-baseline=\"middle\">" << escape(text)
      << "</text>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawArrow(const Vec2& from, const Vec2& to, const Style& style,
                          double head_size) {
  const Vec2 a = toCanvas(from);
  const Vec2 b = toCanvas(to);
  const Vec2 delta = b - a;
  const double length = delta.norm();

  // 箭头尺寸以世界单位给出，必须随缩放换算成像素；退化为零长度时直接跳过，
  // 否则单位方向向量无定义，会画出 NaN 坐标。
  const double head_px = head_size * scale_;
  if (length < kEpsilon || head_px < kEpsilon) return;

  const Vec2 dir = delta / length;
  const Vec2 perp{-dir.y, dir.x};

  // 杆身缩短一个箭头长度，避免线头从三角形尖端戳出来
  const double shaft = std::max(length - head_px, 0.0);
  appendLine(elements_, a, a + dir * shaft, style, false);

  const Vec2 tip = a + dir * length;
  const Vec2 base = tip - dir * head_px;
  const Vec2 half = perp * (head_px * 0.4);

  std::ostringstream oss;
  oss << "<polygon points=\"" << num(tip.x) << ',' << num(tip.y) << ' ' << num(base.x + half.x)
      << ',' << num(base.y + half.y) << ' ' << num(base.x - half.x) << ','
      << num(base.y - half.y) << "\" fill=\"" << style.stroke << "\" stroke=\""
      << style.stroke << "\" stroke-width=\"" << num(style.stroke_width) << "\"/>";
  elements_.push_back(oss.str());
}

void SvgCanvas::drawGrid(double spacing, const Style& style) {
  if (spacing <= kEpsilon || !std::isfinite(spacing)) return;

  // 未设置世界范围时以当前视口反推，保证网格始终铺满画布
  BoundingBox2 box = world_bounds_;
  if (!has_bounds_) {
    box.expand(toWorld({0.0, 0.0}));
    box.expand(toWorld({width_, 0.0}));
    box.expand(toWorld({0.0, height_}));
    box.expand(toWorld({width_, height_}));
  }
  if (!box.valid()) return;

  // 网格过密时按倍数稀疏化：几万条线的 SVG 既没人看也打不开
  double step = spacing;
  while ((box.max.x - box.min.x) / step > 200.0 || (box.max.y - box.min.y) / step > 200.0) {
    step *= 2.0;
    if (step > 1e7) return;
  }

  const double x0 = std::floor(box.min.x / step) * step;
  const double y0 = std::floor(box.min.y / step) * step;

  for (double x = x0; x <= box.max.x; x += step) {
    drawLine({x, box.min.y}, {x, box.max.y}, style);
  }
  for (double y = y0; y <= box.max.y; y += step) {
    drawLine({box.min.x, y}, {box.max.x, y}, style);
  }
}

void SvgCanvas::drawAxes(double length) {
  if (length <= kEpsilon) return;
  const Style axis = Style::solid("#455A64", 1.5);
  drawArrow({0.0, 0.0}, {length, 0.0}, axis, length * 0.03);
  drawArrow({0.0, 0.0}, {0.0, length}, axis, length * 0.03);
  drawText({length, 0.0}, "X", 12.0, "#37474F", "start");
  drawText({0.0, length}, "Y", 12.0, "#37474F", "start");
}

void SvgCanvas::drawTrajectoryColored(const Trajectory& trajectory, double max_kappa,
                                      double stroke_width) {
  if (trajectory.size() < 2) return;  // 一个点画不出有向的段

  // 参考曲率缺失时退化为"轨迹自身的最大曲率"，至少能看出相对冷热；
  // 若轨迹是直线，则任意参考值都得到冷色，符合直觉。
  double kappa_ref = max_kappa;
  if (!std::isfinite(kappa_ref) || kappa_ref <= kEpsilon) {
    for (const TrajectoryPoint& p : trajectory) {
      kappa_ref = std::max(kappa_ref, std::fabs(p.kappa));
    }
  }
  if (kappa_ref <= kEpsilon) kappa_ref = 1.0;

  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const TrajectoryPoint& p0 = trajectory[i];
    const TrajectoryPoint& p1 = trajectory[i + 1];

    // 用该段的平均曲率取色：段内两个端点的 kappa 均值，
    // 比只看起点更接近"这一段有多弯"的直观。
    double kappa = 0.5 * (std::fabs(p0.kappa) + std::fabs(p1.kappa));
    if (kappa <= kEpsilon) {
      // 数据未填 kappa 时按三点几何曲率退化估算（端点借用相邻点）
      const Vec2& a = trajectory[i == 0 ? 0 : i - 1].position();
      const Vec2& b = p0.position();
      const Vec2& c = p1.position();
      if (i == 0 && trajectory.size() > 2) {
        kappa = std::fabs(geometricCurvature(b, c, trajectory[2].position()));
      } else {
        kappa = std::fabs(geometricCurvature(a, b, c));
      }
    }

    const std::string color = curvatureColor(kappa / kappa_ref);
    // linecap=round：相邻线段首尾相接，避免转折处出现缺口
    appendLine(elements_, toCanvas(p0.position()), toCanvas(p1.position()),
               Style::solid(color, stroke_width), true);
  }
}

// ---------------------------------------------------------------------------
//  输出
// ---------------------------------------------------------------------------

std::string SvgCanvas::toString() const {
  std::ostringstream oss;
  oss << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
  oss << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\""
      << num(width_) << "\" height=\"" << num(height_) << "\" viewBox=\"0 0 " << num(width_)
      << ' ' << num(height_) << "\">\n";

  // 白底：SVG 默认透明，深色背景下截图/嵌报告会把线条吞掉
  oss << "<rect x=\"0\" y=\"0\" width=\"" << num(width_) << "\" height=\"" << num(height_)
      << "\" fill=\"#FFFFFF\"/>\n";

  for (const std::string& element : elements_) {
    oss << element << '\n';
  }

  oss << "</svg>\n";
  return oss.str();
}

bool SvgCanvas::save(const std::string& path) const {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) return false;

  const std::string document = toString();
  out.write(document.data(), static_cast<std::streamsize>(document.size()));
  out.flush();
  return out.good();
}

std::string SvgCanvas::escape(const std::string& text) const {
  // 单趟处理：既转义 XML 特殊字符，又丢弃 XML 1.0 不允许的控制字符
  // （例如从二进制数据里带出来的 0x00~0x1F），保证输出的文本节点永远合法。
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

}  // namespace adsim
