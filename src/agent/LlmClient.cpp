// =============================================================================
//  LlmClient.cpp — 大语言模型客户端抽象 + 离线桩实现
//
//  本文件承担两件事：
//    1. MockLlmClient：**确定性**的关键词桩实现。它存在的意义不是"像 LLM"，
//       而是让打标流程、报告生成与单元测试在完全无网的环境下可跑、可复现——
//       同一份输入必须得到逐字节相同的输出，否则测试会间歇性失败。
//    2. 未启用 libcurl 时 HttpLlmClient 的退化实现：构造即抛 std::runtime_error，
//       让调用方显式地回退到 MockLlmClient，而不是在运行期静默失败。
//       （启用 libcurl 时，真正的实现位于 src/agent/http_llm_client.cpp，
//        两个文件中的定义通过 ADSIM_HAS_CURL 互斥。）
// =============================================================================
#include "adsim/agent/LlmClient.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace adsim {

// ---------------------------------------------------------------------------
//  LlmMessage
// ---------------------------------------------------------------------------

LlmMessage LlmMessage::system(const std::string& text) {
  LlmMessage message;
  message.role = Role::kSystem;
  message.content = text;
  return message;
}

LlmMessage LlmMessage::user(const std::string& text) {
  LlmMessage message;
  message.role = Role::kUser;
  message.content = text;
  return message;
}

LlmMessage LlmMessage::assistant(const std::string& text) {
  LlmMessage message;
  message.role = Role::kAssistant;
  message.content = text;
  return message;
}

namespace {

/// JSON 字符串转义。桩实现同样要输出**合法 JSON**：
/// 只要有一个未转义的引号或换行，上层解析就会失败，桩也就失去了价值。
std::string jsonEscape(const std::string& text) {
  std::string out;
  out.reserve(text.size() + text.size() / 8);
  for (const char ch : text) {
    switch (ch) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default: {
        const unsigned char c = static_cast<unsigned char>(ch);
        if (c < 0x20) {
          // 其余控制字符用 \u00XX 表示，这是 JSON 规范允许的最小转义
          static const char* kHex = "0123456789abcdef";
          out += "\\u00";
          out.push_back(kHex[(c >> 4) & 0x0F]);
          out.push_back(kHex[c & 0x0F]);
        } else {
          out.push_back(ch);
        }
        break;
      }
    }
  }
  return out;
}

/// 桩实现的分类结论
struct MockRule {
  std::string category;
  double confidence{0.3};
  std::string matched;     ///< 命中的关键词（写进 rationale，便于人工核对）
  std::string description;
};

/// 关键词匹配。**规则表顺序即优先级**：先命中的先返回，
/// 因此同一段文本永远得到同一个结论（确定性是桩实现的第一要求）。
MockRule matchRule(const std::string& text) {
  struct KeywordRule {
    const char* category;
    double confidence;
    const char* keywords[4];  ///< 以 nullptr 结尾
    const char* description;
  };

  static const KeywordRule kRules[] = {
      {"cut_in",
       0.86,
       {"加塞", "切入", nullptr},
       "相邻车道车辆切入自车前方，自车需要让行或减速。"},
      {"unprotected_left_turn",
       0.84,
       {"无保护左转", "无保护掉头", nullptr},
       "自车在无保护条件下左转，需与对向直行车辆博弈。"},
      {"pedestrian_crossing",
       0.82,
       {"行人", "横穿", nullptr},
       "行人横穿道路，自车减速避让。"},
      {"lead_braking",
       0.80,
       {"急刹", "紧急制动", nullptr},
       "前车紧急制动，自车需要快速响应以避免追尾。"},
      {"free_cruising",
       0.70,
       {"自由巡航", "无交互", "巡航", nullptr},
       "全程无其他交通参与者交互，属于自由巡航片段。"},
  };

  for (const KeywordRule& rule : kRules) {
    for (std::size_t i = 0; i < 4 && rule.keywords[i] != nullptr; ++i) {
      if (text.find(rule.keywords[i]) != std::string::npos) {
        return MockRule{rule.category, rule.confidence, rule.keywords[i], rule.description};
      }
    }
  }

  // 未命中时 matched 留空：上层据此产出"无关键词可依据"的说明，
  // 而不是拼出一句"命中「无」"这种自相矛盾的话
  return MockRule{"unknown", 0.30, "", "未匹配到已知交互模式，建议人工复核。"};
}

/// 把请求里的所有消息拼成一段文本用于关键词匹配
std::string flatten(const LlmRequest& request) {
  std::string text;
  for (const LlmMessage& message : request.messages) {
    text += message.content;
    text.push_back('\n');
  }
  return text;
}

}  // namespace

// ---------------------------------------------------------------------------
//  MockLlmClient
// ---------------------------------------------------------------------------

LlmResponse MockLlmClient::complete(const LlmRequest& request) {
  ++call_count_;

  const auto start = std::chrono::steady_clock::now();

  // 模拟延迟：上限 2s，避免测试用例被"模拟网络"拖成慢测试
  if (latency_ms_ > 0.0) {
    const double capped = std::min(latency_ms_, 2000.0);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(capped)));
  }

  LlmResponse response;
  response.model = "mock-keyword-rules";
  // 提示词长度按字符数粗估，仅用于让 token 统计非零，便于上层流程验证
  response.prompt_tokens = static_cast<int>(flatten(request).size() / 2);
  response.completion_tokens = 48;

  if (fail_next_) {
    // 一次性失败：清标志，下一次调用恢复正常——用于测试错误分支与回退逻辑
    fail_next_ = false;
    response.ok = false;
    response.error = "MockLlmClient: 本次调用被要求失败（setFailNext）";
    response.latency_ms = std::max(std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - start)
                                       .count(),
                                   latency_ms_);
    return response;
  }

  const MockRule rule = matchRule(flatten(request));

  // 未命中任何关键词时 matched 为空。此时必须给出"没有可依据的信息"这一
  // 明确说明，而不是拼出"命中「」"这类无意义文本——后者会让复核的人
  // 以为规则真的命中了什么。
  const std::string rationale =
      rule.matched.empty()
          ? "提示词中未出现任何已知交互关键词，无法据此判定类别"
          : "关键词规则命中「" + rule.matched + "」";

  // 输出严格 JSON：字段固定为 category / confidence / rationale / description。
  // 不输出任何多余文字，模拟 json_mode 下模型的行为，便于上层解析逻辑统一。
  std::string json = "{";
  json += "\"category\":\"" + jsonEscape(rule.category) + "\",";
  json += "\"confidence\":" + std::to_string(rule.confidence) + ",";
  json += "\"rationale\":\"" + jsonEscape(rationale) + "\",";
  json += "\"description\":\"" + jsonEscape(rule.description) + "\"";
  json += "}";

  response.ok = true;
  response.text = json;
  response.latency_ms = std::max(std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - start)
                                     .count(),
                                 latency_ms_);
  return response;
}

// ---------------------------------------------------------------------------
//  HttpLlmClient —— 未启用 libcurl 时的退化实现
// ---------------------------------------------------------------------------

#if !defined(ADSIM_HAS_CURL)

/// 占位实现：即便永远不构造成功，也需要完整类型才能实例化 unique_ptr 的析构
struct HttpLlmClient::Impl {};

HttpLlmClient::HttpLlmClient(const HttpLlmConfig&) {
  // 构造即失败：把"缺少依赖"暴露在装配阶段而不是第一次调用时，
  // 调用方据此回退到 MockLlmClient。
  throw std::runtime_error(
      "HttpLlmClient 不可用：本构建未启用 libcurl。"
      "请以 -DADSIM_WITH_CURL=ON 重新配置构建，或改用 MockLlmClient。");
}

HttpLlmClient::~HttpLlmClient() = default;

LlmResponse HttpLlmClient::complete(const LlmRequest&) {
  LlmResponse response;
  response.ok = false;
  response.error = "libcurl 未编译进本构建，无法发起 HTTP 请求";
  return response;
}

bool HttpLlmClient::isAvailable() const { return false; }

bool HttpLlmClient::compiledIn() { return false; }

void HttpLlmClient::clearCache() {
  // 无缓存可清；统计计数保留，便于调用方观察历史命中情况
}

#endif  // !ADSIM_HAS_CURL

}  // namespace adsim
