// =============================================================================
//  ScenarioTagger.cpp — 交互场景打标（特征提取 + 规则 + LLM 增强）
//
//  三段式的取舍：
//    * 特征提取只做数值计算，不掺任何语义判断——保证结论客观、可复现；
//    * 规则打标给出**始终可用**的基线，且每条规则的 rationale 都写清命中了
//      哪一条、数值是多少，便于工程师复核而不是盲信分类结果；
//    * LLM 只在规则之上做"增强"：成功则覆盖类别与描述，失败则原样保留规则结论。
//      打标流水线不依赖网络可用性，这是本模块最重要的工程约束。
//
//  另一条约束来自数据本身：路测数据的时间戳可能不连续（丢帧、补帧），
//  因此所有求导（加速度、曲率）都必须用相邻样本的**真实时间差**并防止除零。
// =============================================================================
#include "adsim/agent/ScenarioTagger.h"

#include "adsim/common/Logger.h"
#include "adsim/common/Types.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace adsim {

namespace {

// ---------------------------------------------------------------------------
//  判定阈值（集中在此，便于按数据分布调参）
// ---------------------------------------------------------------------------

/// 横向偏移超过该值即认为"已偏离车道中心"
constexpr double kLateralOffsetThreshold = 1.0;
/// 横向偏移必须持续该时长以上，才认定是有意为之（排除抖动与瞬时摆动）
constexpr double kLateralSustainSeconds = 0.5;
/// 换道过程中航向与初始航向的最大允许夹角 (rad)。
/// 没有这一条，一条长弯道会被误判成换道——弯道同样会持续产生横向偏移。
constexpr double kHeadingParallelTolerance = 0.35;
/// 越过该横向偏移量（约半个车道宽）才认为确实完成了一次换道
constexpr double kLaneChangeOffset = 2.0;
/// 曲率超过该值（转弯半径 < 20m）判定为路口转弯
constexpr double kSharpCurvature = 0.05;
/// "无交互"的判定裕度
constexpr double kNoInteractionTtc = 10.0;
constexpr double kNoInteractionGap = 30.0;
/// 跟车的最小车头时距阈值
constexpr double kFollowingHeadway = 3.0;
/// 加塞判定中的"距离接近"阈值 (m)
constexpr double kCutInGap = 15.0;
/// min_ttc / min_gap 的"未观测到"阈值：默认值 1e9 量级即表示无数据
constexpr double kUnobserved = 1e8;

// ---------------------------------------------------------------------------
//  通用小工具
// ---------------------------------------------------------------------------

std::string fmtNum(double value, int precision = 2) {
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

/// 是否近似全零：用于判断"记录值缺失、需要改用差分推算"
bool allNearZero(const std::vector<double>& values, double eps = 1e-6) {
  for (const double v : values) {
    if (std::fabs(v) > eps) return false;
  }
  return true;
}

double maxAbs(const std::vector<double>& values) {
  double best = 0.0;
  for (const double v : values) best = std::max(best, std::fabs(v));
  return best;
}

/// 归一到 [0,1] 的"裕度"：指标超出阈值越多越接近 1。
/// 置信度用它调制，避免所有命中都给出同一个数。
double marginRatio(double value, double threshold) {
  if (threshold <= kEpsilon) return 1.0;
  return clamp(std::fabs(value) / std::fabs(threshold), 0.0, 1.0);
}

// ---------------------------------------------------------------------------
//  横向行为分析（换道识别）
// ---------------------------------------------------------------------------

struct Sample {
  Vec2 position;
  double heading{0.0};
  double time{0.0};
};

struct LateralRun {
  double max_offset{0.0};   ///< 相对初始航向线的最大横向偏移
  double duration{0.0};     ///< 偏移持续超阈值的最长时间
  bool sustained{false};
};

/// 以"首点位置 + 首点航向"建立局部坐标系，统计横向偏移。
/// 之所以同时要求航向保持平行：单纯的横向偏移在弯道上也会持续存在，
/// 只有"车头始终朝着原方向、车身却整体平移"才是换道。
LateralRun analyzeLateral(const std::vector<Sample>& samples) {
  LateralRun run;
  if (samples.size() < 2) return run;

  const Vec2 origin = samples.front().position;
  const double heading0 = samples.front().heading;
  const Vec2 normal = Vec2{std::cos(heading0), std::sin(heading0)}.perp();

  bool inside = false;
  double run_start = 0.0;
  for (const Sample& sample : samples) {
    const double lateral = std::fabs((sample.position - origin).dot(normal));
    run.max_offset = std::max(run.max_offset, lateral);

    const double heading_error =
        std::fabs(normalizeAngle(sample.heading - heading0));
    const bool in_band =
        lateral > kLateralOffsetThreshold && heading_error < kHeadingParallelTolerance;

    if (in_band) {
      if (!inside) {
        inside = true;
        run_start = sample.time;
      }
      run.duration = std::max(run.duration, sample.time - run_start);
    } else {
      inside = false;
    }
  }

  run.sustained = run.duration >= kLateralSustainSeconds;
  return run;
}

// ---------------------------------------------------------------------------
//  手写 JSON 提取（不引入第三方库）
// ---------------------------------------------------------------------------

std::size_t skipWhitespace(const std::string& text, std::size_t index) {
  while (index < text.size() &&
         (text[index] == ' ' || text[index] == '\t' || text[index] == '\n' ||
          text[index] == '\r')) {
    ++index;
  }
  return index;
}

void appendUtf8(std::string& out, unsigned int code_point) {
  if (code_point <= 0x7F) {
    out.push_back(static_cast<char>(code_point));
  } else if (code_point <= 0x7FF) {
    out.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  } else if (code_point <= 0xFFFF) {
    out.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  }
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

/// 解析从 quote_pos 处开始的 JSON 字符串字面量（支持全部标准转义）。
/// 未闭合、含非法转义或裸控制字符时返回 false。
bool parseJsonString(const std::string& text, std::size_t quote_pos, std::string& out,
                     std::size_t& end_pos) {
  if (quote_pos >= text.size() || text[quote_pos] != '"') return false;

  std::string value;
  for (std::size_t i = quote_pos + 1; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '\\') {
      if (i + 1 >= text.size()) return false;
      const char esc = text[++i];
      switch (esc) {
        case '"':
          value.push_back('"');
          break;
        case '\\':
          value.push_back('\\');
          break;
        case '/':
          value.push_back('/');
          break;
        case 'b':
          value.push_back('\b');
          break;
        case 'f':
          value.push_back('\f');
          break;
        case 'n':
          value.push_back('\n');
          break;
        case 'r':
          value.push_back('\r');
          break;
        case 't':
          value.push_back('\t');
          break;
        case 'u': {
          if (i + 4 >= text.size()) return false;
          unsigned int code_point = 0;
          for (int k = 0; k < 4; ++k) {
            const int digit = hexValue(text[i + 1 + static_cast<std::size_t>(k)]);
            if (digit < 0) return false;
            code_point = (code_point << 4) | static_cast<unsigned int>(digit);
          }
          i += 4;
          appendUtf8(value, code_point);
          break;
        }
        default:
          return false;  // 非法转义序列
      }
    } else if (c == '"') {
      out = value;
      end_pos = i + 1;
      return true;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      return false;  // 字符串里出现裸控制字符：非法 JSON
    } else {
      value.push_back(c);
    }
  }
  return false;  // 引号未闭合
}

/// 查找 "key" : "value" 形式的字符串字段。
/// 关键点：只有当 key 后面确实跟着冒号时才算命中——
/// 否则 {"type":"text"} 里的值 "text" 会被误当成键。
bool findJsonString(const std::string& text, const std::string& key, std::string& out,
                    std::size_t start = 0) {
  const std::string pattern = "\"" + key + "\"";
  std::size_t pos = text.find(pattern, start);
  while (pos != std::string::npos) {
    std::size_t index = skipWhitespace(text, pos + pattern.size());
    if (index < text.size() && text[index] == ':') {
      index = skipWhitespace(text, index + 1);
      std::size_t end = 0;
      if (index < text.size() && text[index] == '"') {
        return parseJsonString(text, index, out, end);
      }
      return false;  // 键存在但取值不是字符串
    }
    pos = text.find(pattern, pos + 1);
  }
  return false;
}

/// 查找 "key" : <number> 形式的数值字段
bool findJsonNumber(const std::string& text, const std::string& key, double& out,
                    std::size_t start = 0) {
  const std::string pattern = "\"" + key + "\"";
  std::size_t pos = text.find(pattern, start);
  while (pos != std::string::npos) {
    std::size_t index = skipWhitespace(text, pos + pattern.size());
    if (index < text.size() && text[index] == ':') {
      index = skipWhitespace(text, index + 1);
      if (index >= text.size()) return false;
      const char* begin = text.c_str() + index;
      char* end = nullptr;
      const double value = std::strtod(begin, &end);
      if (end == begin) return false;  // 没解析出任何数字
      out = value;
      return true;
    }
    pos = text.find(pattern, pos + 1);
  }
  return false;
}

// ---------------------------------------------------------------------------
//  类别名称表：一套定义同时服务于 toString / categoryFromString / 提示词
// ---------------------------------------------------------------------------

struct CategoryName {
  InteractionCategory category;
  const char* chinese;
  const char* identifier;  ///< 交给 LLM 的英文标识，也是 JSON 中的取值
};

const CategoryName kCategoryNames[] = {
    {InteractionCategory::kUnknown, "未知", "unknown"},
    {InteractionCategory::kCarFollowing, "跟车", "car_following"},
    {InteractionCategory::kCutIn, "加塞", "cut_in"},
    {InteractionCategory::kCutOut, "切出", "cut_out"},
    {InteractionCategory::kLeadBraking, "前车急刹", "lead_braking"},
    {InteractionCategory::kUnprotectedLeftTurn, "无保护左转", "unprotected_left_turn"},
    {InteractionCategory::kUnprotectedRightTurn, "无保护右转", "unprotected_right_turn"},
    {InteractionCategory::kPedestrianCrossing, "行人横穿", "pedestrian_crossing"},
    {InteractionCategory::kIntersectionYield, "路口让行", "intersection_yield"},
    {InteractionCategory::kLaneChange, "自车换道", "lane_change"},
    {InteractionCategory::kOvertaking, "超车", "overtaking"},
    {InteractionCategory::kEmergencyBraking, "紧急制动", "emergency_braking"},
    {InteractionCategory::kFreeCruising, "自由巡航", "free_cruising"},
};

const CategoryName* findCategoryName(InteractionCategory category) {
  for (const CategoryName& entry : kCategoryNames) {
    if (entry.category == category) return &entry;
  }
  return nullptr;
}

/// 规范化：转小写、去掉下划线与连字符、去掉枚举风格的前缀 'k'。
/// 这样 "cut_in" / "CutIn" / "kCutIn" / "cutin" 都能被识别。
std::string normalizeName(const std::string& name) {
  std::string out;
  out.reserve(name.size());
  for (const char c : name) {
    if (c == '_' || c == '-' || c == ' ') continue;
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (!out.empty() && out[0] == 'k') out.erase(out.begin());
  return out;
}

/// 规则命中的候选结论
struct RuleHit {
  InteractionCategory category{InteractionCategory::kUnknown};
  double confidence{0.0};
  std::string rationale;
};

}  // namespace

// ---------------------------------------------------------------------------
//  枚举 ↔ 字符串
// ---------------------------------------------------------------------------

const char* toString(InteractionCategory category) {
  const CategoryName* entry = findCategoryName(category);
  return entry != nullptr ? entry->chinese : "未知";
}

const char* toMachineName(InteractionCategory category) {
  const CategoryName* entry = findCategoryName(category);
  return entry != nullptr ? entry->identifier : "unknown";
}

InteractionCategory categoryFromString(const std::string& name) {
  const std::string normalized = normalizeName(name);
  for (const CategoryName& entry : kCategoryNames) {
    if (normalized == normalizeName(entry.identifier) ||
        normalized == normalizeName(entry.chinese)) {
      return entry.category;
    }
  }
  return InteractionCategory::kUnknown;
}

// ---------------------------------------------------------------------------
//  InteractionFeatures
// ---------------------------------------------------------------------------

std::string InteractionFeatures::toPromptText() const {
  std::ostringstream oss;
  oss << "【场景量化特征】\n";
  oss << "- 片段时长: " << fmtNum(duration) << " s，自车行驶距离: " << fmtNum(ego_distance)
      << " m\n";
  oss << "- 自车速度: 平均 " << fmtNum(ego_mean_speed) << " m/s，最大 "
      << fmtNum(ego_max_speed) << " m/s，最小 " << fmtNum(ego_min_speed) << " m/s\n";
  oss << "- 最大减速度: " << fmtNum(ego_max_deceleration)
      << " m/s²（负值越大表示制动越猛）\n";
  oss << "- 最大横向加速度: " << fmtNum(ego_max_lateral_acceleration) << " m/s²\n";
  oss << "- 轨迹最大曲率: " << fmtNum(max_curvature, 4) << " 1/m（半径约 "
      << (max_curvature > kEpsilon ? fmtNum(1.0 / max_curvature, 1) : std::string("∞"))
      << " m）\n";

  oss << "- 最小 TTC: ";
  if (min_ttc < kUnobserved) {
    oss << fmtNum(min_ttc) << " s\n";
  } else {
    oss << "本片段未观测到潜在冲突\n";
  }
  oss << "- 与其他物体最小间距: ";
  if (min_gap < kUnobserved) {
    oss << fmtNum(min_gap) << " m\n";
  } else {
    oss << "本片段未观测到其他物体\n";
  }
  oss << "- 最小车头时距: ";
  if (min_headway < kUnobserved) {
    oss << fmtNum(min_headway) << " s\n";
  } else {
    oss << "无有效跟车数据\n";
  }
  oss << "- 最大接近速度: " << fmtNum(closing_speed) << " m/s\n";

  oss << "- 交互物体数量: " << object_count << "\n";
  oss << "- 是否发生碰撞: " << (has_collision ? "是" : "否") << "\n";
  oss << "- 是否驶出可行驶区域: " << (has_off_road ? "是" : "否") << "\n";
  oss << "- 是否存在持续横向偏移(疑似换道): " << (lane_change_detected ? "是" : "否")
      << "，最大横向偏移 " << fmtNum(max_lateral_offset) << " m\n";
  oss << "- 危险工况判定: " << (isCritical() ? "是" : "否") << "\n";
  oss << "\n请依据以上数值判定场景类别。";
  return oss.str();
}

std::string InteractionFeatures::toString() const {
  std::ostringstream oss;
  oss << "时长 " << fmtNum(duration) << "s"
      << " 里程 " << fmtNum(ego_distance) << "m"
      << " 速度[avg/max/min] " << fmtNum(ego_mean_speed) << '/' << fmtNum(ego_max_speed)
      << '/' << fmtNum(ego_min_speed) << "m/s"
      << " 最大减速度 " << fmtNum(ego_max_deceleration) << "m/s²"
      << " 最大横向加速度 " << fmtNum(ego_max_lateral_acceleration) << "m/s²"
      << " 最大曲率 " << fmtNum(max_curvature, 4) << "1/m";

  oss << " 最小TTC ";
  if (min_ttc < kUnobserved) {
    oss << fmtNum(min_ttc) << "s";
  } else {
    oss << "无";
  }
  oss << " 最小间距 ";
  if (min_gap < kUnobserved) {
    oss << fmtNum(min_gap) << "m";
  } else {
    oss << "无";
  }

  oss << " 接近速度 " << fmtNum(closing_speed) << "m/s"
      << " 物体数 " << object_count << " 碰撞 " << (has_collision ? "是" : "否")
      << " 换道 " << (lane_change_detected ? "是" : "否") << "(横向偏移 "
      << fmtNum(max_lateral_offset) << "m)";
  return oss.str();
}

// ---------------------------------------------------------------------------
//  TaggingResult
// ---------------------------------------------------------------------------

std::string TaggingResult::toString() const {
  std::ostringstream oss;
  // 显式限定命名空间：成员函数 toString() 会遮蔽同名的自由重载
  oss << "类别=" << adsim::toString(primary_category);
  if (!secondary_categories.empty()) {
    oss << "（次要: ";
    for (std::size_t i = 0; i < secondary_categories.size(); ++i) {
      if (i != 0) oss << ", ";
      oss << adsim::toString(secondary_categories[i]);
    }
    oss << "）";
  }
  oss << " 置信度=" << fmtNum(confidence)
      << " 危险=" << (critical ? "是" : "否")
      << " 来源=" << (from_llm ? "规则+LLM" : "规则");
  if (!rationale.empty()) oss << "\n  依据: " << rationale;
  if (!description.empty()) oss << "\n  描述: " << description;
  return oss.str();
}

// ---------------------------------------------------------------------------
//  ScenarioTagger：构造与装配
// ---------------------------------------------------------------------------

ScenarioTagger::ScenarioTagger() : ScenarioTagger(Config{}) {}

ScenarioTagger::ScenarioTagger(const Config& config) : config_(config) {}

void ScenarioTagger::setLlmClient(std::shared_ptr<LlmClient> client) {
  llm_client_ = std::move(client);
}

// ---------------------------------------------------------------------------
//  特征提取：仿真结果
// ---------------------------------------------------------------------------

InteractionFeatures ScenarioTagger::extractFeatures(const SimulationResult& result) {
  InteractionFeatures features;

  const std::vector<TrajectoryPoint>& states = result.ego_states;
  if (states.empty()) return features;

  // 时间轴：优先用 result.time；长度不匹配时退回轨迹点自带的时间戳，
  // 两者都不可用时后续求导会被"真实时间差"检查自动跳过，不会产生除零。
  std::vector<double> times;
  if (result.time.size() == states.size()) {
    times = result.time;
  } else {
    times.reserve(states.size());
    for (const TrajectoryPoint& point : states) times.push_back(point.t);
  }

  if (times.size() >= 2) {
    features.duration = std::max(times.back() - times.front(), 0.0);
  }

  // 行驶距离：内核累计值更准（含曲率修正），缺失时按折线累加兜底
  double distance = 0.0;
  for (std::size_t i = 1; i < states.size(); ++i) {
    distance += (states[i].position() - states[i - 1].position()).norm();
  }
  features.ego_distance = result.total_distance > 0.0 ? result.total_distance : distance;

  // ---- 速度 ----
  std::vector<double> speeds;
  speeds.reserve(states.size());
  for (const TrajectoryPoint& point : states) speeds.push_back(point.v);

  if (allNearZero(speeds, 1e-3)) {
    // 轨迹点未填速度：用位置差分补出速度，同样使用真实时间差
    speeds.clear();
    speeds.push_back(0.0);
    for (std::size_t i = 1; i < states.size(); ++i) {
      const double dt = times.size() == states.size() ? times[i] - times[i - 1] : 0.0;
      const double segment = (states[i].position() - states[i - 1].position()).norm();
      speeds.push_back(dt > kEpsilon ? segment / dt : speeds.back());
    }
  }

  if (!speeds.empty()) {
    double sum = 0.0;
    for (const double v : speeds) sum += v;
    features.ego_mean_speed = sum / static_cast<double>(speeds.size());
    features.ego_max_speed = *std::max_element(speeds.begin(), speeds.end());
    features.ego_min_speed = *std::min_element(speeds.begin(), speeds.end());
  }

  // ---- 加速度：优先取记录值，缺失时用 dv/dt 推算 ----
  std::vector<double> accelerations;
  accelerations.reserve(states.size());
  for (const TrajectoryPoint& point : states) accelerations.push_back(point.a);

  if (allNearZero(accelerations, 1e-6)) {
    accelerations.assign(states.size(), 0.0);
    for (std::size_t i = 1; i < states.size() && i < speeds.size(); ++i) {
      const double dt = times.size() == states.size() ? times[i] - times[i - 1] : 0.0;
      // dt <= 0 表示时间戳重复或倒退：跳过而不是除零
      accelerations[i] = dt > kEpsilon ? (speeds[i] - speeds[i - 1]) / dt : accelerations[i - 1];
    }
  }
  if (!accelerations.empty()) {
    features.ego_max_deceleration = *std::min_element(accelerations.begin(),
                                                     accelerations.end());
  }

  // ---- 曲率与横向加速度 ----
  features.max_curvature = result.max_curvature;
  if (features.max_curvature <= kEpsilon) {
    std::vector<double> kappas;
    kappas.reserve(states.size());
    for (const TrajectoryPoint& point : states) kappas.push_back(std::fabs(point.kappa));
    features.max_curvature = maxAbs(kappas);
  }
  // 曲率仍缺失时用三点几何曲率兜底（道路本身有弯，但轨迹点没填曲率）
  if (features.max_curvature <= kEpsilon && states.size() >= 3) {
    for (std::size_t i = 1; i + 1 < states.size(); ++i) {
      const Vec2& a = states[i - 1].position();
      const Vec2& b = states[i].position();
      const Vec2& c = states[i + 1].position();
      const double denom = (b - a).norm() * (c - b).norm() * (c - a).norm();
      if (denom < kEpsilon) continue;
      features.max_curvature =
          std::max(features.max_curvature, std::fabs(2.0 * (b - a).cross(c - a) / denom));
    }
  }

  features.ego_max_lateral_acceleration = result.max_lateral_acceleration;
  if (features.ego_max_lateral_acceleration <= kEpsilon) {
    // 横向加速度 a_lat = v² · κ，用本帧速度与曲率估算
    for (std::size_t i = 0; i < states.size() && i < speeds.size(); ++i) {
      const double lateral = speeds[i] * speeds[i] * std::fabs(states[i].kappa);
      features.ego_max_lateral_acceleration =
          std::max(features.ego_max_lateral_acceleration, lateral);
    }
  }

  // ---- 已有安全指标：直接取用，避免重复实现两套判定 ----
  features.min_ttc = result.min_ttc;
  features.min_gap = result.min_distance;
  features.has_collision = result.collision_count > 0;
  features.has_off_road = result.off_road_count > 0;

  // ---- 其他物体：数量、最大接近速度 ----
  std::size_t max_objects = 0;
  double closing = 0.0;
  for (std::size_t i = 0; i < result.object_history.size(); ++i) {
    const std::vector<RoadObject>& frame = result.object_history[i];
    max_objects = std::max(max_objects, frame.size());
    if (i >= states.size()) continue;

    const TrajectoryPoint& ego = states[i];
    const Vec2 ego_velocity{ego.v * std::cos(ego.theta), ego.v * std::sin(ego.theta)};
    for (const RoadObject& object : frame) {
      const Vec2 delta = object.position() - ego.position();
      const double range = delta.norm();
      if (range < kEpsilon) continue;
      const Vec2 relative_velocity = object.velocity - ego_velocity;
      // 径向速度分量为负表示两者在接近，取反即为接近速度
      const double radial = relative_velocity.dot(delta) / range;
      closing = std::max(closing, -radial);
    }
  }
  features.object_count = static_cast<int>(max_objects);
  features.closing_speed = std::max(closing, 0.0);

  // ---- 换道识别 ----
  std::vector<Sample> samples;
  samples.reserve(states.size());
  for (std::size_t i = 0; i < states.size(); ++i) {
    samples.push_back(Sample{states[i].position(), states[i].theta,
                             i < times.size() ? times[i] : 0.0});
  }
  const LateralRun run = analyzeLateral(samples);
  features.max_lateral_offset = run.max_offset;
  features.lane_change_detected = run.sustained && run.max_offset > kLaneChangeOffset;

  return features;
}

// ---------------------------------------------------------------------------
//  特征提取：对齐后的路测数据
// ---------------------------------------------------------------------------

InteractionFeatures ScenarioTagger::extractFeatures(const std::vector<AlignedFrame>& frames) {
  InteractionFeatures features;

  // 只保留具备车辆状态的帧；没有 odom 的帧无法参与运动学计算
  std::vector<const AlignedFrame*> valid;
  valid.reserve(frames.size());
  for (const AlignedFrame& frame : frames) {
    if (frame.has_odom) valid.push_back(&frame);
  }
  if (valid.empty()) return features;

  std::vector<double> times;
  times.reserve(valid.size());
  for (const AlignedFrame* frame : valid) {
    times.push_back(toSeconds(frame->stamp));
  }
  // 时间戳可能不连续甚至倒退：duration 只取正的时间跨度
  if (times.size() >= 2) {
    features.duration = std::max(times.back() - times.front(), 0.0);
    if (features.duration <= 0.0) {
      const double low = *std::min_element(times.begin(), times.end());
      const double high = *std::max_element(times.begin(), times.end());
      features.duration = std::max(high - low, 0.0);
    }
  }

  double distance = 0.0;
  for (std::size_t i = 1; i < valid.size(); ++i) {
    distance += (valid[i]->odom.pose().position() - valid[i - 1]->odom.pose().position())
                    .norm();
  }
  features.ego_distance = distance;

  // ---- 速度：优先取底盘反馈，全零时用位置差分 ----
  std::vector<double> speeds;
  speeds.reserve(valid.size());
  for (const AlignedFrame* frame : valid) speeds.push_back(frame->odom.speed);

  if (allNearZero(speeds, 1e-3)) {
    speeds.assign(valid.size(), 0.0);
    for (std::size_t i = 1; i < valid.size(); ++i) {
      const double dt = times[i] - times[i - 1];
      const double segment =
          (valid[i]->odom.pose().position() - valid[i - 1]->odom.pose().position()).norm();
      speeds[i] = dt > kEpsilon ? segment / dt : speeds[i - 1];
    }
  }

  if (!speeds.empty()) {
    double sum = 0.0;
    for (const double v : speeds) sum += v;
    features.ego_mean_speed = sum / static_cast<double>(speeds.size());
    features.ego_max_speed = *std::max_element(speeds.begin(), speeds.end());
    features.ego_min_speed = *std::min_element(speeds.begin(), speeds.end());
  }

  // ---- 加速度：优先取记录值，缺失时用真实时间差求导 ----
  std::vector<double> recorded;
  recorded.reserve(valid.size());
  for (const AlignedFrame* frame : valid) recorded.push_back(frame->odom.acceleration);

  std::vector<double> accelerations;
  if (!allNearZero(recorded, 1e-6)) {
    accelerations = recorded;
  } else {
    accelerations.assign(valid.size(), 0.0);
    for (std::size_t i = 1; i < valid.size(); ++i) {
      const double dt = times[i] - times[i - 1];
      accelerations[i] = dt > kEpsilon ? (speeds[i] - speeds[i - 1]) / dt : 0.0;
    }
  }
  features.ego_max_deceleration = *std::min_element(accelerations.begin(),
                                                    accelerations.end());

  // ---- 曲率：κ ≈ Δθ / Δs，两者都用真实时间差与真实位移 ----
  double max_kappa = 0.0;
  double max_lateral = 0.0;
  for (std::size_t i = 1; i < valid.size(); ++i) {
    const VehicleState& prev = valid[i - 1]->odom;
    const VehicleState& curr = valid[i]->odom;
    const double arc = (curr.pose().position() - prev.pose().position()).norm();
    if (arc > kEpsilon) {
      const double kappa = std::fabs(normalizeAngle(curr.theta - prev.theta)) / arc;
      max_kappa = std::max(max_kappa, kappa);
      // a_lat = v² · κ
      max_lateral = std::max(max_lateral, curr.speed * curr.speed * kappa);
    }
    // 有横摆角速度时，a_lat = v · ω 更贴近实车数据
    if (std::fabs(curr.yaw_rate) > 1e-9) {
      max_lateral = std::max(max_lateral, std::fabs(curr.speed * curr.yaw_rate));
    }
  }
  features.max_curvature = max_kappa;
  features.ego_max_lateral_acceleration = max_lateral;

  // min_gap / min_ttc / object_count 等需要其他物体的信息，
  // 对齐帧里没有目标列表，这里**保持默认值**而不是编造数字。
  features.object_count = 0;

  // ---- 换道识别 ----
  std::vector<Sample> samples;
  samples.reserve(valid.size());
  for (std::size_t i = 0; i < valid.size(); ++i) {
    samples.push_back(Sample{valid[i]->odom.pose().position(), valid[i]->odom.theta,
                             times[i] - times.front()});
  }
  const LateralRun run = analyzeLateral(samples);
  features.max_lateral_offset = run.max_offset;
  features.lane_change_detected = run.sustained && run.max_offset > kLaneChangeOffset;

  return features;
}

// ---------------------------------------------------------------------------
//  规则打标
// ---------------------------------------------------------------------------

TaggingResult ScenarioTagger::tagByRules(const InteractionFeatures& features) const {
  TaggingResult result;
  result.features = features;
  result.critical = features.isCritical();

  std::vector<RuleHit> hits;

  // R1 碰撞：最高优先级，任何其他规则都不应掩盖它
  if (features.has_collision) {
    hits.push_back({InteractionCategory::kEmergencyBraking, 0.95,
                    "规则 R1：has_collision=true，片段内已发生碰撞"});
  }

  // R2 极小 TTC
  if (features.min_ttc < config_.critical_ttc && features.min_ttc < kUnobserved) {
    const double margin = 1.0 - marginRatio(features.min_ttc, config_.critical_ttc);
    hits.push_back({InteractionCategory::kEmergencyBraking, 0.55 + 0.40 * margin,
                    "规则 R2：最小 TTC " + fmtNum(features.min_ttc) + "s < 阈值 " +
                        fmtNum(config_.critical_ttc) + "s"});
  }

  // R3 极小间距
  if (features.min_gap < config_.critical_gap && features.min_gap < kUnobserved) {
    const double margin = 1.0 - marginRatio(features.min_gap, config_.critical_gap);
    hits.push_back({InteractionCategory::kEmergencyBraking, 0.50 + 0.40 * margin,
                    "规则 R3：最小间距 " + fmtNum(features.min_gap) + "m < 阈值 " +
                        fmtNum(config_.critical_gap) + "m"});
  }

  // R4 减速度超阈值
  if (features.ego_max_deceleration <= config_.harsh_braking_threshold) {
    const double margin =
        clamp(std::fabs(features.ego_max_deceleration) /
                  std::max(std::fabs(config_.harsh_braking_threshold), kEpsilon) -
                  1.0,
              0.0, 1.0);
    hits.push_back({InteractionCategory::kLeadBraking, 0.60 + 0.35 * margin,
                    "规则 R4：最大减速度 " + fmtNum(features.ego_max_deceleration) +
                        " m/s² 低于阈值 " + fmtNum(config_.harsh_braking_threshold) +
                        " m/s²"});
  }

  // R5 加塞：横向侵入 + 距离接近 + 正在接近
  const bool intrusion = features.max_lateral_offset > config_.cut_in_lateral_threshold;
  const bool gap_close = features.min_gap < kCutInGap;
  if (intrusion && gap_close && features.closing_speed > 1.0) {
    const double margin = 1.0 - marginRatio(features.min_gap, kCutInGap);
    hits.push_back({InteractionCategory::kCutIn, 0.55 + 0.35 * margin,
                    "规则 R5：横向偏移 " + fmtNum(features.max_lateral_offset) +
                        "m 超过侵入阈值 " + fmtNum(config_.cut_in_lateral_threshold) +
                        "m，最小间距 " + fmtNum(features.min_gap) + "m，接近速度 " +
                        fmtNum(features.closing_speed) + "m/s"});
  }

  // R6 换道：持续横向偏移且已越过半个车道宽
  if (features.lane_change_detected) {
    const double margin = clamp(features.max_lateral_offset / 3.5, 0.0, 1.0);
    hits.push_back({InteractionCategory::kLaneChange, 0.55 + 0.35 * margin,
                    "规则 R6：横向偏移 " + fmtNum(features.max_lateral_offset) +
                        "m 持续超过 " + fmtNum(kLateralSustainSeconds) +
                        "s，判定为自车换道"});
  }

  // R7 路口转弯：曲率极大。
  // 注意：特征里只有曲率大小、没有左右符号，因此无法区分左转/右转，
  // 只能给出"路口让行"这一中性类别；左右转语义留给 LLM 增强补充。
  if (features.max_curvature > kSharpCurvature) {
    const double margin =
        clamp(features.max_curvature / kSharpCurvature - 1.0, 0.0, 1.0);
    hits.push_back({InteractionCategory::kIntersectionYield, 0.50 + 0.35 * margin,
                    "规则 R7：最大曲率 " + fmtNum(features.max_curvature, 4) +
                        " 1/m（转弯半径约 " +
                        fmtNum(1.0 / std::max(features.max_curvature, kEpsilon), 1) +
                        "m）超过阈值 " + fmtNum(kSharpCurvature, 3) + " 1/m"});
  }

  // R8 跟车：有前车且车头时距偏小，但未触发危险规则
  if (features.min_headway < kFollowingHeadway && features.min_headway < kUnobserved &&
      hits.empty()) {
    hits.push_back({InteractionCategory::kCarFollowing, 0.55,
                    "规则 R8：最小车头时距 " + fmtNum(features.min_headway) +
                        "s < " + fmtNum(kFollowingHeadway) + "s，属于常规跟车"});
  }

  // R9 自由巡航：既没有其他物体，也没有明显冲突
  if (hits.empty() &&
      (features.object_count == 0 || (features.min_ttc > kNoInteractionTtc &&
                                      features.min_gap > kNoInteractionGap))) {
    const std::string detail =
        features.object_count == 0
            ? "片段内无其他交通参与者"
            : ("最小 TTC " + fmtNum(features.min_ttc) + "s 与最小间距 " +
               fmtNum(features.min_gap) + "m 均远大于交互阈值");
    hits.push_back({InteractionCategory::kFreeCruising, 0.65, "规则 R9：" + detail});
  }

  if (hits.empty()) {
    result.primary_category = InteractionCategory::kUnknown;
    result.confidence = 0.30;
    result.rationale = "未命中任何规则（特征未显示明显交互或危险），建议人工复核";
    result.description = "场景特征不典型，规则无法给出确定结论。";
    return result;
  }

  result.primary_category = hits.front().category;
  result.rationale = hits.front().rationale;

  // 置信度：基础值来自首要规则（已含裕度），命中的规则越多说明结论相互印证，
  // 每多一条上浮 0.05，上限 0.99——不给"绝对确定"。
  double confidence = hits.front().confidence;
  if (hits.size() > 1) {
    confidence += 0.05 * static_cast<double>(std::min<std::size_t>(hits.size() - 1, 3));
    result.rationale += "；同时命中 " + std::to_string(hits.size() - 1) + " 条其他规则";
  }
  result.confidence = clamp(confidence, 0.05, 0.99);

  for (std::size_t i = 1; i < hits.size(); ++i) {
    const InteractionCategory other = hits[i].category;
    if (other == result.primary_category) continue;
    if (std::find(result.secondary_categories.begin(), result.secondary_categories.end(),
                  other) == result.secondary_categories.end()) {
      result.secondary_categories.push_back(other);
    }
  }

  result.description = std::string("规则判定为「") + toString(result.primary_category) + "」";
  return result;
}

// ---------------------------------------------------------------------------
//  完整打标（规则 + 可选 LLM 增强）
// ---------------------------------------------------------------------------

namespace {

/// 调用 LLM 增强规则结论。
/// 失败路径（网络错误、返回不可解析）一律**原样返回规则结论**，
/// 只在日志里留下原因——打标流水线不因外部服务不可用而降级。
TaggingResult enhanceWithLlm(const ScenarioTagger& tagger, const TaggingResult& rules,
                             LlmClient& client) {
  const LlmRequest request = tagger.buildRequest(rules.features);
  const LlmResponse response = client.complete(request);

  if (!response.ok) {
    ADSIM_LOG_WARN("LLM 增强失败，保留规则结论: ", response.error);
    return rules;
  }

  TaggingResult enhanced = rules;
  if (!ScenarioTagger::parseLlmResponse(response.text, enhanced)) {
    ADSIM_LOG_WARN("LLM 返回内容无法解析为 JSON，保留规则结论: ",
                   response.text.substr(0, 120));
    return rules;
  }

  // LLM 明确表示"无法判定"时，不覆盖规则的确定结论：
  // 规则是基线，LLM 的价值在于补充语义，而不是把已知信息擦成 unknown。
  if (enhanced.primary_category == InteractionCategory::kUnknown &&
      rules.primary_category != InteractionCategory::kUnknown) {
    ADSIM_LOG_DEBUG("LLM 未给出确定类别，保留规则类别 ",
                    toString(rules.primary_category));
    enhanced.primary_category = rules.primary_category;
    if (enhanced.description.empty()) enhanced.description = rules.description;
  }

  enhanced.features = rules.features;
  enhanced.critical = rules.critical;
  enhanced.secondary_categories = rules.secondary_categories;
  enhanced.from_llm = true;
  return enhanced;
}

}  // namespace

TaggingResult ScenarioTagger::tag(const SimulationResult& result) {
  TaggingResult tagged = tagByRules(extractFeatures(result));
  // 仿真内核给出的危险判定与特征判定取并集，避免因特征缺失漏掉危险片段
  tagged.critical = tagged.critical || result.isCritical();

  if (!config_.enable_llm || !llm_client_ || !llm_client_->isAvailable()) return tagged;
  return enhanceWithLlm(*this, tagged, *llm_client_);
}

TaggingResult ScenarioTagger::tag(const std::vector<AlignedFrame>& frames) {
  TaggingResult tagged = tagByRules(extractFeatures(frames));
  if (!config_.enable_llm || !llm_client_ || !llm_client_->isAvailable()) return tagged;
  return enhanceWithLlm(*this, tagged, *llm_client_);
}

std::vector<TaggingResult> ScenarioTagger::tagBatch(
    const std::vector<SimulationResult>& results) {
  std::vector<TaggingResult> tagged;
  tagged.reserve(results.size());
  for (const SimulationResult& result : results) tagged.push_back(tag(result));

  // 排序：危险的排前面 → 置信度高的排前面 → 名称稳定兜底，
  // 保证同一批数据多次运行得到相同顺序（便于增量复核）。
  std::stable_sort(tagged.begin(), tagged.end(),
                   [](const TaggingResult& a, const TaggingResult& b) {
                     if (a.critical != b.critical) return a.critical;
                     if (std::fabs(a.confidence - b.confidence) > 1e-9) {
                       return a.confidence > b.confidence;
                     }
                     return false;
                   });
  return tagged;
}

// ---------------------------------------------------------------------------
//  提示词与响应解析
// ---------------------------------------------------------------------------

LlmRequest ScenarioTagger::buildRequest(const InteractionFeatures& features) const {
  LlmRequest request;
  // 打标需要稳定、可复现的输出，温度取 0
  request.temperature = 0.0;
  request.max_tokens = 512;
  request.json_mode = true;
  request.timeout_seconds = 30.0;

  std::string system_prompt;
  system_prompt +=
      "你是自动驾驶场景分析专家。请阅读给定的交互特征，判定这段场景的类别。\n"
      "输出要求（必须严格遵守）：\n"
      "1. 只输出一个 JSON 对象，不要输出任何解释性文字，不要使用 Markdown 代码块；\n"
      "2. JSON 字段固定为 category / confidence / rationale / description；\n"
      "3. category 必须严格取下列英文标识之一：";
  for (std::size_t i = 0; i < sizeof(kCategoryNames) / sizeof(kCategoryNames[0]); ++i) {
    if (i != 0) system_prompt += ", ";
    system_prompt += kCategoryNames[i].identifier;
  }
  system_prompt += "；\n";
  system_prompt +=
      "4. confidence 为 0~1 之间的小数，表示你对判定的把握；\n"
      "5. rationale 用中文说明判定依据，并引用上面的具体数值；\n"
      "6. description 用一句中文描述该场景发生了什么。";

  request.messages.push_back(LlmMessage::system(system_prompt));
  request.messages.push_back(LlmMessage::user(features.toPromptText()));
  return request;
}

bool ScenarioTagger::parseLlmResponse(const std::string& text, TaggingResult& result) {
  if (text.empty()) return false;

  std::string payload = text;

  // 容错一：模型把 JSON 包在 Markdown 代码块里
  const std::size_t fence = payload.find("```");
  if (fence != std::string::npos) {
    const std::size_t open = payload.find('{', fence);
    const std::size_t close = payload.rfind('}');
    if (open == std::string::npos || close == std::string::npos || close <= open) {
      return false;
    }
    payload = payload.substr(open, close - open + 1);
  }

  // 容错二：JSON 前后夹带了说明文字，只取最外层大括号之间的部分
  const std::size_t begin = payload.find('{');
  const std::size_t end = payload.rfind('}');
  if (begin == std::string::npos || end == std::string::npos || end <= begin) return false;

  // 大括号必须配平，否则是截断的响应
  int depth = 0;
  bool in_string = false;
  for (std::size_t i = begin; i <= end; ++i) {
    const char c = payload[i];
    if (in_string) {
      if (c == '\\') {
        ++i;  // 跳过被转义的字符，避免把 \" 当成字符串结束
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
    } else if (c == '{') {
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth < 0) return false;
    }
  }
  if (depth != 0 || in_string) return false;

  const std::string json = payload.substr(begin, end - begin + 1);

  // category 是唯一必需字段；缺失即视为解析失败
  std::string category;
  if (!findJsonString(json, "category", category)) return false;

  // 在副本上组装结果：解析失败时调用方的 result 必须原封不动
  TaggingResult updated = result;
  updated.primary_category = categoryFromString(category);

  std::string rationale;
  if (findJsonString(json, "rationale", rationale)) updated.rationale = rationale;

  std::string description;
  if (findJsonString(json, "description", description)) updated.description = description;

  double confidence = 0.0;
  if (findJsonNumber(json, "confidence", confidence)) {
    updated.confidence = clamp(confidence, 0.0, 1.0);
  }

  result = updated;
  return true;
}

}  // namespace adsim
