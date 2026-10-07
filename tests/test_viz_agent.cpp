// =============================================================================
//  test_viz_agent.cpp — 可视化与场景打标单元测试
// =============================================================================
#include "TestFramework.h"

#include "adsim/agent/LlmClient.h"
#include "adsim/agent/ScenarioTagger.h"
#include "adsim/viz/SimVisualizer.h"
#include "adsim/viz/SvgCanvas.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

using namespace adsim;

namespace {

/// 临时文件，析构时自动清理
class TempFile {
 public:
  explicit TempFile(const std::string& name) : path_("/tmp/adsim_viz_" + name) {}
  ~TempFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

/// 读取整个文件内容
std::string readFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) return {};
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

/// 构造一个最小可用的仿真结果
SimulationResult makeResult(std::size_t steps, double speed) {
  SimulationResult result;
  result.scenario_name = "测试场景";

  for (std::size_t i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) * 0.1;

    TrajectoryPoint point;
    point.x = speed * t;
    point.y = 2.0 * std::sin(t);
    point.theta = std::atan2(2.0 * std::cos(t), speed);
    point.kappa = 0.01;
    point.v = speed;
    point.a = 0.0;
    point.t = t;

    result.time.push_back(t);
    result.ego_states.push_back(point);
  }

  result.min_ttc = 3.0;
  result.min_distance = 12.0;
  result.max_curvature = 0.01;
  result.max_lateral_acceleration = 1.5;
  result.total_distance = speed * 0.1 * static_cast<double>(steps - 1);
  result.average_speed = speed;
  return result;
}

/// 构造一个带有安全事件的仿真结果
SimulationResult makeCriticalResult() {
  SimulationResult result = makeResult(30, 12.0);

  SafetyEventRecord record;
  record.event = SafetyEvent::kLowTtc;
  record.time = 1.5;
  record.value = 1.2;
  record.description = "与前车距离过近";
  result.events.push_back(record);

  result.min_ttc = 1.2;
  result.near_miss_count = 1;
  return result;
}

}  // namespace

// ===========================================================================
//  SvgCanvas
// ===========================================================================

ADSIM_TEST(Viz, 坐标变换往返一致) {
  SvgCanvas canvas(800.0, 600.0);

  BoundingBox2 bounds;
  bounds.expand({0.0, 0.0});
  bounds.expand({100.0, 50.0});
  canvas.setWorldBounds(bounds, 20.0);

  const std::vector<Vec2> points = {{0.0, 0.0}, {50.0, 25.0}, {100.0, 50.0},
                                    {-10.0, 80.0}};

  for (const Vec2& p : points) {
    const Vec2 back = canvas.toWorld(canvas.toCanvas(p));
    ADSIM_CHECK_NEAR(back.x, p.x, 1e-9);
    ADSIM_CHECK_NEAR(back.y, p.y, 1e-9);
  }
}

ADSIM_TEST(Viz, Y轴翻转正确) {
  SvgCanvas canvas(400.0, 400.0);

  BoundingBox2 bounds;
  bounds.expand({0.0, 0.0});
  bounds.expand({10.0, 10.0});
  canvas.setWorldBounds(bounds, 0.0);

  // 世界坐标 Y 越大，画布 Y 越小（SVG 的 Y 轴向下）
  const Vec2 bottom = canvas.toCanvas({5.0, 0.0});
  const Vec2 top = canvas.toCanvas({5.0, 10.0});

  ADSIM_CHECK_GT(bottom.y, top.y);
}

ADSIM_TEST(Viz, 等比缩放不产生形变) {
  SvgCanvas canvas(800.0, 200.0);  // 宽高比 4:1

  BoundingBox2 bounds;
  bounds.expand({0.0, 0.0});
  bounds.expand({10.0, 10.0});  // 正方形世界范围
  canvas.setWorldBounds(bounds, 0.0);

  // 世界中的正方形在画布上仍是正方形（x/y 缩放一致）
  const Vec2 origin = canvas.toCanvas({0.0, 0.0});
  const Vec2 dx = canvas.toCanvas({1.0, 0.0}) - origin;
  const Vec2 dy = canvas.toCanvas({0.0, 1.0}) - origin;

  ADSIM_CHECK_NEAR(std::abs(dx.x), std::abs(dy.y), 1e-6);
}

ADSIM_TEST(Viz, 空画布仍产出合法SVG) {
  SvgCanvas canvas(100.0, 100.0);
  const std::string svg = canvas.toString();

  ADSIM_CHECK(!svg.empty());
  ADSIM_CHECK(svg.find("<svg") != std::string::npos);
  ADSIM_CHECK(svg.find("</svg>") != std::string::npos);
  ADSIM_CHECK(svg.find("xmlns") != std::string::npos);
}

ADSIM_TEST(Viz, 特殊字符被正确转义) {
  SvgCanvas canvas(200.0, 200.0);

  // 含 XML 特殊字符的文本若不转义会产出非法 SVG
  canvas.drawText({0.0, 0.0}, "a < b & c > d \"e\" 'f'", 10.0, "#000000");

  const std::string svg = canvas.toString();
  ADSIM_CHECK(svg.find("&lt;") != std::string::npos);
  ADSIM_CHECK(svg.find("&amp;") != std::string::npos);
  ADSIM_CHECK(svg.find("&gt;") != std::string::npos);

  // 不得出现未转义的裸特殊字符
  ADSIM_CHECK(svg.find("a < b") == std::string::npos);
}

ADSIM_TEST(Viz, 各图元均可绘制) {
  SvgCanvas canvas(400.0, 400.0);

  BoundingBox2 bounds;
  bounds.expand({0.0, 0.0});
  bounds.expand({20.0, 20.0});
  canvas.setWorldBounds(bounds);

  canvas.drawLine({0, 0}, {10, 10}, Style::solid("#FF0000"));
  canvas.drawPolyline({{0, 0}, {5, 5}, {10, 0}}, Style::solid("#00FF00"));
  canvas.drawPolygon({{2, 2}, {8, 2}, {8, 8}}, Style::filled("#0000FF"));
  canvas.drawCircle({10, 10}, 2.0, Style::solid("#000000"));
  canvas.drawRectangle({1, 1}, 3.0, 4.0, Style::solid("#333333"));
  canvas.drawObb(Obb2(Pose2(5.0, 5.0, 0.5), 4.0, 2.0), Style::filled("#888888"));
  canvas.drawArrow({0, 0}, {10, 10}, Style::solid("#FF00FF"));
  canvas.drawGrid(2.0, Style::solid("#EEEEEE"));
  canvas.drawAxes(5.0);
  canvas.drawLabel({3, 3}, "标签");

  const std::string svg = canvas.toString();
  ADSIM_CHECK(svg.find("<svg") != std::string::npos);
  // 应产生足够多的图元
  ADSIM_CHECK_GT(svg.size(), std::size_t(500));
}

ADSIM_TEST(Viz, 点数不足时不产生非法元素) {
  SvgCanvas canvas(200.0, 200.0);

  // 空或单点折线、不足三点的多边形都不应产出元素
  canvas.drawPolyline({}, Style::solid("#000000"));
  canvas.drawPolyline({{0, 0}}, Style::solid("#000000"));
  canvas.drawPolygon({{0, 0}, {1, 1}}, Style::filled("#000000"));

  const std::string svg = canvas.toString();
  ADSIM_CHECK(svg.find("<polyline") == std::string::npos);
  ADSIM_CHECK(svg.find("<polygon") == std::string::npos);
}

ADSIM_TEST(Viz, 轨迹着色随曲率变化) {
  // 关键性质：小曲率段与大曲率段的颜色必须不同，
  // 否则"一眼看出曲率突变"这个核心价值就不成立
  SvgCanvas flat_canvas(400.0, 400.0);
  SvgCanvas curved_canvas(400.0, 400.0);

  Trajectory flat(10);
  Trajectory curved(10);
  for (std::size_t i = 0; i < 10; ++i) {
    flat[i].x = static_cast<double>(i);
    flat[i].y = 0.0;
    flat[i].kappa = 0.0;

    curved[i].x = static_cast<double>(i);
    curved[i].y = 0.0;
    curved[i].kappa = 0.5;  // 远高于 max_kappa
  }

  flat_canvas.drawTrajectoryColored(flat, 0.5);
  curved_canvas.drawTrajectoryColored(curved, 0.5);

  const std::string flat_svg = flat_canvas.toString();
  const std::string curved_svg = curved_canvas.toString();

  // 两者都应产出内容，且颜色不同
  ADSIM_CHECK(flat_svg.find("<line") != std::string::npos ||
              flat_svg.find("<path") != std::string::npos);
  ADSIM_CHECK(flat_svg != curved_svg);
}

ADSIM_TEST(Viz, 保存后可重新读回) {
  TempFile file("canvas.svg");

  SvgCanvas canvas(300.0, 300.0);
  canvas.drawCircle({0, 0}, 5.0, Style::filled("#123456"));

  ADSIM_CHECK(canvas.save(file.path()));

  const std::string content = readFile(file.path());
  ADSIM_CHECK(!content.empty());
  ADSIM_CHECK(content.find("<svg") != std::string::npos);
}

ADSIM_TEST(Viz, 样式工厂函数) {
  const Style solid = Style::solid("#FF0000", 2.0);
  ADSIM_CHECK_EQ(solid.stroke, std::string("#FF0000"));
  ADSIM_CHECK_NEAR(solid.stroke_width, 2.0, 1e-9);
  ADSIM_CHECK(!solid.dashed);

  const Style dashed = Style::dashedLine("#00FF00", 1.5, 6.0);
  ADSIM_CHECK(dashed.dashed);
  ADSIM_CHECK_NEAR(dashed.dash_length, 6.0, 1e-9);

  const Style filled = Style::filled("#0000FF", 0.5);
  ADSIM_CHECK_EQ(filled.fill, std::string("#0000FF"));
  ADSIM_CHECK_NEAR(filled.opacity, 0.5, 1e-9);
}

// ===========================================================================
//  SimVisualizer
// ===========================================================================

ADSIM_TEST(Viz, 仿真可视化各渲染方法产出合法SVG) {
  const SimulationResult result = makeResult(50, 10.0);
  const World world = World::straightRoad(2, 3.5, 200.0);

  SimVisualizer visualizer{VisualizerConfig()};

  const std::string snapshot = visualizer.renderSnapshot(result, world, 10);
  ADSIM_CHECK(snapshot.find("<svg") != std::string::npos);

  const std::string overview = visualizer.renderOverview(result, world);
  ADSIM_CHECK(overview.find("<svg") != std::string::npos);

  const std::string timeline = visualizer.renderTimeline(result);
  ADSIM_CHECK(timeline.find("<svg") != std::string::npos);
}

ADSIM_TEST(Viz, 空结果不崩溃) {
  const SimulationResult empty;
  const World world = World::straightRoad(1, 3.5, 100.0);

  SimVisualizer visualizer{VisualizerConfig()};

  const std::string snapshot = visualizer.renderSnapshot(empty, world);
  ADSIM_CHECK(snapshot.find("<svg") != std::string::npos);

  const std::string timeline = visualizer.renderTimeline(empty);
  ADSIM_CHECK(timeline.find("<svg") != std::string::npos);
}

ADSIM_TEST(Viz, 快照步号越界取末帧) {
  const SimulationResult result = makeResult(20, 8.0);
  const World world = World::straightRoad(1, 3.5, 100.0);

  SimVisualizer visualizer{VisualizerConfig()};

  const std::string beyond = visualizer.renderSnapshot(result, world, 9999);
  ADSIM_CHECK(beyond.find("<svg") != std::string::npos);

  const std::string negative = visualizer.renderSnapshot(result, world, -5);
  ADSIM_CHECK(negative.find("<svg") != std::string::npos);
}

ADSIM_TEST(Viz, 时序图标注安全事件) {
  const SimulationResult critical = makeCriticalResult();

  SimVisualizer visualizer{VisualizerConfig()};
  const std::string timeline = visualizer.renderTimeline(critical);

  ADSIM_CHECK(timeline.find("<svg") != std::string::npos);
  // 事件描述应出现在图中
  ADSIM_CHECK(timeline.find("与前车距离过近") != std::string::npos);
}

ADSIM_TEST(Viz, HTML报告结构完整) {
  const SimulationResult result = makeResult(30, 10.0);
  const World world = World::straightRoad(1, 3.5, 100.0);

  SimVisualizer visualizer{VisualizerConfig()};
  const std::string html = buildHtmlReport(
      {{"总览", visualizer.renderOverview(result, world)},
       {"时序", visualizer.renderTimeline(result)}},
      "测试报告");

  ADSIM_CHECK(html.find("<!DOCTYPE html>") != std::string::npos ||
              html.find("<!doctype html>") != std::string::npos);
  ADSIM_CHECK(html.find("测试报告") != std::string::npos);
  ADSIM_CHECK(html.find("<svg") != std::string::npos);
  ADSIM_CHECK(html.find("</html>") != std::string::npos);
}

ADSIM_TEST(Viz, 报告写出文件) {
  TempFile file("report.svg");

  const SimulationResult result = makeResult(25, 9.0);
  const World world = World::straightRoad(1, 3.5, 100.0);

  SimVisualizer visualizer{VisualizerConfig()};
  ADSIM_CHECK(visualizer.writeReport(result, world, file.path()));

  const std::string content = readFile(file.path());
  ADSIM_CHECK(content.find("<svg") != std::string::npos);
}

// ===========================================================================
//  LlmClient
// ===========================================================================

ADSIM_TEST(Agent, 消息工厂函数) {
  const LlmMessage system = LlmMessage::system("系统提示");
  ADSIM_CHECK(system.role == LlmMessage::Role::kSystem);
  ADSIM_CHECK_EQ(system.content, std::string("系统提示"));

  const LlmMessage user = LlmMessage::user("用户输入");
  ADSIM_CHECK(user.role == LlmMessage::Role::kUser);

  const LlmMessage assistant = LlmMessage::assistant("模型输出");
  ADSIM_CHECK(assistant.role == LlmMessage::Role::kAssistant);
}

ADSIM_TEST(Agent, 桩客户端确定性且返回合法JSON) {
  MockLlmClient client;

  LlmRequest request;
  request.messages.push_back(LlmMessage::user("请分析这段数据：检测到旁车加塞"));

  const LlmResponse first = client.complete(request);
  const LlmResponse second = client.complete(request);

  ADSIM_CHECK(first.ok);
  ADSIM_CHECK(!first.text.empty());

  // 两次调用必须完全一致（确定性是离线桩的核心价值）
  ADSIM_CHECK_EQ(first.text, second.text);
  ADSIM_CHECK_EQ(client.callCount(), std::size_t(2));

  // 返回内容必须是合法 JSON 且含约定字段
  ADSIM_CHECK(first.text.find("category") != std::string::npos);
  ADSIM_CHECK(first.text.find("confidence") != std::string::npos);
  ADSIM_CHECK(first.text.find("rationale") != std::string::npos);
  ADSIM_CHECK(first.text.find("description") != std::string::npos);

  // 大括号必须配对
  std::size_t open = 0;
  std::size_t close = 0;
  for (char c : first.text) {
    if (c == '{') ++open;
    if (c == '}') ++close;
  }
  ADSIM_CHECK_EQ(open, close);
}

ADSIM_TEST(Agent, 桩客户端可模拟失败) {
  MockLlmClient client;

  LlmRequest request;
  request.messages.push_back(LlmMessage::user("测试"));

  client.setFailNext(true);
  const LlmResponse failed = client.complete(request);
  ADSIM_CHECK(!failed.ok);
  ADSIM_CHECK(!failed.error.empty());

  // 失败只影响一次，后续调用恢复正常
  const LlmResponse recovered = client.complete(request);
  ADSIM_CHECK(recovered.ok);
}

ADSIM_TEST(Agent, HTTP客户端可用性与构建一致) {
  const bool compiled = HttpLlmClient::compiledIn();

#if defined(ADSIM_HAS_CURL)
  ADSIM_CHECK(compiled);
#else
  ADSIM_CHECK(!compiled);

  // 未编译进 libcurl 时构造应抛异常，而不是静默失败
  HttpLlmConfig config;
  ADSIM_CHECK_THROWS(HttpLlmClient client(config), std::runtime_error);
#endif
}

// ===========================================================================
//  ScenarioTagger
// ===========================================================================

ADSIM_TEST(Agent, 类别名与枚举互转) {
  // 展示名（中文）与机器名（英文）刻意分成两套：
  // 报告给人看要中文，进出 JSON / LLM 要稳定的标识符
  ADSIM_CHECK_EQ(std::string(toString(InteractionCategory::kCutIn)), std::string("加塞"));
  ADSIM_CHECK_EQ(std::string(toMachineName(InteractionCategory::kCutIn)),
                 std::string("cut_in"));

  // 机器名与其解析必须构成严格往返
  for (InteractionCategory category :
       {InteractionCategory::kCutIn, InteractionCategory::kLeadBraking,
        InteractionCategory::kUnprotectedLeftTurn,
        InteractionCategory::kPedestrianCrossing, InteractionCategory::kFreeCruising,
        InteractionCategory::kEmergencyBraking}) {
    ADSIM_CHECK(categoryFromString(toMachineName(category)) == category);
  }

  // 解析器要容忍常见变体（大小写、下划线、枚举前缀）
  ADSIM_CHECK(categoryFromString("cut_in") == InteractionCategory::kCutIn);
  ADSIM_CHECK(categoryFromString("CutIn") == InteractionCategory::kCutIn);
  ADSIM_CHECK(categoryFromString("kCutIn") == InteractionCategory::kCutIn);

  ADSIM_CHECK(categoryFromString("不存在的类别") == InteractionCategory::kUnknown);
}

ADSIM_TEST(Agent, 从仿真结果提取特征) {
  const SimulationResult result = makeCriticalResult();
  const InteractionFeatures features = ScenarioTagger::extractFeatures(result);

  ADSIM_CHECK_GT(features.duration, 0.0);
  ADSIM_CHECK_GT(features.ego_mean_speed, 0.0);
  ADSIM_CHECK_NEAR(features.min_ttc, 1.2, 1e-9);
  ADSIM_CHECK(features.isCritical());  // TTC 1.2 < 2.0

  ADSIM_CHECK(!features.toString().empty());
  ADSIM_CHECK(!features.toPromptText().empty());
}

ADSIM_TEST(Agent, 空数据特征提取不崩溃) {
  const SimulationResult empty;
  const InteractionFeatures features = ScenarioTagger::extractFeatures(empty);

  ADSIM_CHECK_NEAR(features.duration, 0.0, 1e-9);
  ADSIM_CHECK(std::isfinite(features.min_ttc));
  ADSIM_CHECK(std::isfinite(features.ego_mean_speed));
}

ADSIM_TEST(Agent, 规则打标给出可解释结论) {
  ScenarioTagger tagger;

  InteractionFeatures features;
  features.duration = 8.0;
  features.ego_mean_speed = 8.0;
  features.min_ttc = 1.0;   // 危险
  features.min_gap = 3.0;
  features.ego_max_deceleration = -5.0;
  features.object_count = 1;

  const TaggingResult result = tagger.tagByRules(features);

  ADSIM_CHECK(result.primary_category != InteractionCategory::kUnknown);
  ADSIM_CHECK_GT(result.confidence, 0.0);
  ADSIM_CHECK_LE(result.confidence, 1.0);
  ADSIM_CHECK(!result.rationale.empty());   // 判定依据必须可读
  ADSIM_CHECK(result.critical);
  ADSIM_CHECK(!result.toString().empty());
}

ADSIM_TEST(Agent, 自由巡航片段不被判为危险) {
  ScenarioTagger tagger;

  InteractionFeatures features;
  features.duration = 10.0;
  features.ego_mean_speed = 12.0;
  features.min_ttc = 1e9;
  features.min_gap = 1e9;
  features.ego_max_deceleration = -0.5;
  features.object_count = 0;

  const TaggingResult result = tagger.tagByRules(features);

  ADSIM_CHECK(!result.critical);
  ADSIM_CHECK(result.primary_category == InteractionCategory::kFreeCruising);
}

ADSIM_TEST(Agent, LLM响应解析处理各种格式) {
  TaggingResult result;

  // 标准格式
  const std::string well_formed =
      R"({"category": "cut_in", "confidence": 0.88, "rationale": "横向侵入", "description": "旁车加塞"})";
  ADSIM_CHECK(ScenarioTagger::parseLlmResponse(well_formed, result));
  ADSIM_CHECK(result.primary_category == InteractionCategory::kCutIn);
  ADSIM_CHECK_NEAR(result.confidence, 0.88, 1e-6);
  ADSIM_CHECK_EQ(result.rationale, std::string("横向侵入"));

  // 字段乱序 + 多余空白
  TaggingResult reordered;
  const std::string shuffled =
      R"({  "description" : "行人横穿" ,
           "confidence":0.9,
           "category"  :  "pedestrian_crossing" ,
           "rationale":"行人进入车道"  })";
  ADSIM_CHECK(ScenarioTagger::parseLlmResponse(shuffled, reordered));
  ADSIM_CHECK(reordered.primary_category == InteractionCategory::kPedestrianCrossing);

  // 非法内容必须返回 false 且不破坏原结果
  TaggingResult untouched;
  untouched.primary_category = InteractionCategory::kCarFollowing;
  ADSIM_CHECK(!ScenarioTagger::parseLlmResponse("这不是 JSON", untouched));
  ADSIM_CHECK(untouched.primary_category == InteractionCategory::kCarFollowing);

  ADSIM_CHECK(!ScenarioTagger::parseLlmResponse("", untouched));
  ADSIM_CHECK(!ScenarioTagger::parseLlmResponse("{不完整的", untouched));
}

ADSIM_TEST(Agent, LLM失败时回退到规则结果) {
  auto client = std::make_shared<MockLlmClient>();
  client->setFailNext(true);  // 让 LLM 调用失败

  ScenarioTagger tagger;
  tagger.setLlmClient(client);

  const SimulationResult result = makeCriticalResult();
  const TaggingResult tagged = tagger.tag(result);

  // 规则结论必须保留，不能因为 LLM 失败就丢失
  ADSIM_CHECK(tagged.primary_category != InteractionCategory::kUnknown);
  ADSIM_CHECK(!tagged.rationale.empty());
  ADSIM_CHECK(!tagged.from_llm);
}

ADSIM_TEST(Agent, 无LLM客户端时仍可打标) {
  ScenarioTagger tagger;  // 不注入客户端

  const SimulationResult result = makeCriticalResult();
  const TaggingResult tagged = tagger.tag(result);

  ADSIM_CHECK(tagged.primary_category != InteractionCategory::kUnknown);
  ADSIM_CHECK(!tagged.from_llm);
}

ADSIM_TEST(Agent, 提示词包含关键特征) {
  ScenarioTagger tagger;

  InteractionFeatures features;
  features.duration = 6.0;
  features.min_ttc = 1.4;
  features.min_gap = 4.5;
  features.ego_max_deceleration = -5.5;

  const LlmRequest request = tagger.buildRequest(features);

  ADSIM_CHECK(!request.messages.empty());
  ADSIM_CHECK(request.json_mode);       // 打标要求结构化输出
  ADSIM_CHECK_LT(request.temperature, 1.0);  // 低温度保证稳定

  // 请求中必须带上量化特征，否则模型无从判断
  std::string combined;
  for (const LlmMessage& message : request.messages) {
    combined += message.content;
  }
  ADSIM_CHECK(combined.find("1.4") != std::string::npos);
  ADSIM_CHECK(combined.find("json") != std::string::npos ||
              combined.find("JSON") != std::string::npos);
}

ADSIM_TEST(Agent, 批量打标按价值排序) {
  ScenarioTagger tagger;

  std::vector<SimulationResult> results;
  results.push_back(makeResult(20, 10.0));        // 常规
  results.push_back(makeCriticalResult());        // 危险
  SimulationResult mild = makeResult(20, 10.0);
  mild.min_ttc = 5.0;
  results.push_back(mild);

  const std::vector<TaggingResult> tagged = tagger.tagBatch(results);

  ADSIM_CHECK_EQ(tagged.size(), results.size());
  // 危险的应排在前面
  ADSIM_CHECK(tagged.front().critical);
}
