// =============================================================================
//  http_llm_client.cpp — 基于 libcurl 的真实 LLM 客户端（可选依赖）
//
//  编译前提：CMake 配置 -DADSIM_WITH_CURL=ON（该选项会定义 ADSIM_HAS_CURL）。
//  未启用时本文件不产生任何定义——HttpLlmClient 的退化实现（构造即抛异常）
//  位于 LlmClient.cpp，两者由 ADSIM_HAS_CURL 互斥，不会重复定义。
//
//  三个实现要点：
//    1. libcurl 只出现在本文件的实现里，头文件不暴露任何 curl 类型（pimpl），
//       这样未安装 libcurl 的机器上核心库依然能完整编译。
//    2. 网络抖动是常态：链接失败与 429/5xx 按指数退避重试，4xx（参数错误、
//       Key 无效）不重试——重试解决不了的问题只会浪费配额。
//    3. 相同请求命中内存缓存，避免重复计费。缓存键由完整请求体哈希得到，
//       只要有一个字段不同就是不同的键。
// =============================================================================
#include "adsim/agent/LlmClient.h"

#if defined(ADSIM_HAS_CURL)

#include "adsim/common/Logger.h"
#include "adsim/common/Types.h"

#include <curl/curl.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace adsim {

namespace {

// ---------------------------------------------------------------------------
//  字符串 / JSON 小工具
// ---------------------------------------------------------------------------

std::string numToString(double value, int precision = 3) {
  std::ostringstream oss;
  // 固定 classic locale：某些环境下默认 locale 会把小数点写成逗号，
  // 那会直接产出非法 JSON。
  oss.imbue(std::locale::classic());
  oss << std::fixed << std::setprecision(precision) << value;
  std::string s = oss.str();
  const std::size_t dot = s.find('.');
  if (dot != std::string::npos) {
    s.erase(s.find_last_not_of('0') + 1);
    if (!s.empty() && s.back() == '.') s.pop_back();
  }
  return s;
}

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

std::size_t skipWhitespace(const std::string& text, std::size_t index) {
  while (index < text.size() &&
         (text[index] == ' ' || text[index] == '\t' || text[index] == '\n' ||
          text[index] == '\r')) {
    ++index;
  }
  return index;
}

/// 解析 JSON 字符串字面量（含全部标准转义）
bool parseJsonString(const std::string& text, std::size_t quote_pos, std::string& out) {
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
            const char h = text[i + 1 + static_cast<std::size_t>(k)];
            int digit = -1;
            if (h >= '0' && h <= '9') {
              digit = h - '0';
            } else if (h >= 'a' && h <= 'f') {
              digit = h - 'a' + 10;
            } else if (h >= 'A' && h <= 'F') {
              digit = h - 'A' + 10;
            }
            if (digit < 0) return false;
            code_point = (code_point << 4) | static_cast<unsigned int>(digit);
          }
          i += 4;
          if (code_point <= 0x7F) {
            value.push_back(static_cast<char>(code_point));
          } else if (code_point <= 0x7FF) {
            value.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
            value.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
          } else {
            value.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
            value.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
            value.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
          }
          break;
        }
        default:
          return false;
      }
    } else if (c == '"') {
      out = value;
      return true;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      return false;
    } else {
      value.push_back(c);
    }
  }
  return false;
}

/// 查找 "key" : "value"：key 必须紧跟冒号，避免把 {"type":"text"} 里的
/// 值 "text" 误认成键。
bool findJsonString(const std::string& text, const std::string& key, std::string& out) {
  const std::string pattern = "\"" + key + "\"";
  std::size_t pos = text.find(pattern);
  while (pos != std::string::npos) {
    std::size_t index = skipWhitespace(text, pos + pattern.size());
    if (index < text.size() && text[index] == ':') {
      index = skipWhitespace(text, index + 1);
      if (index < text.size() && text[index] == '"') return parseJsonString(text, index, out);
      return false;
    }
    pos = text.find(pattern, pos + 1);
  }
  return false;
}

bool findJsonInt(const std::string& text, const std::string& key, int& out) {
  const std::string pattern = "\"" + key + "\"";
  std::size_t pos = text.find(pattern);
  while (pos != std::string::npos) {
    std::size_t index = skipWhitespace(text, pos + pattern.size());
    if (index < text.size() && text[index] == ':') {
      index = skipWhitespace(text, index + 1);
      const char* begin = text.c_str() + index;
      char* end = nullptr;
      const long value = std::strtol(begin, &end, 10);
      if (end == begin) return false;
      out = static_cast<int>(value);
      return true;
    }
    pos = text.find(pattern, pos + 1);
  }
  return false;
}

// ---------------------------------------------------------------------------
//  curl 回调
// ---------------------------------------------------------------------------

size_t writeCallback(char* ptr, size_t size, size_t nmemb, void* userdata) {
  const std::size_t bytes = size * nmemb;
  auto* sink = static_cast<std::string*>(userdata);
  sink->append(ptr, bytes);
  return bytes;
}

/// curl 全局初始化。进程内只需一次，且必须在多线程使用前完成。
void ensureCurlGlobalInit() {
  static std::once_flag once;
  std::call_once(once, []() {
    const CURLcode code = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (code != CURLE_OK) {
      ADSIM_LOG_ERROR("curl_global_init 失败，HTTP 客户端将不可用");
    }
  });
}

/// 该 HTTP 状态码是否值得重试：429（限流）与 5xx（服务端故障）可以，
/// 4xx 其余（Key 错误、参数非法）重试没有意义。
bool retryableStatus(long status) { return status == 429 || status >= 500; }

}  // namespace

// ---------------------------------------------------------------------------
//  Impl
// ---------------------------------------------------------------------------

struct HttpLlmClient::Impl {
  HttpLlmConfig config;
  std::string api_key;

  /// 响应缓存：key 为完整请求体的哈希
  std::unordered_map<std::string, LlmResponse> cache;
  std::deque<std::string> cache_order;  ///< 淘汰顺序（先进先出）
  std::mutex mutex;

  /// 本次请求的可读描述，出错时写进 error 便于定位
  std::string describeRequest(const std::vector<LlmMessage>& messages) const;

  std::string buildBody(const LlmRequest& request) const;
  std::string makeCacheKey(const LlmRequest& request) const;

  /// 单次 HTTP 尝试。transport_ok 表示链路层是否成功
  LlmResponse performOnce(const LlmRequest& request, const std::string& body,
                          bool& transport_ok, long& http_status);
};

std::string HttpLlmClient::Impl::buildBody(const LlmRequest& request) const {
  std::string system_text;
  std::string messages_json;

  for (const LlmMessage& message : request.messages) {
    if (message.role == LlmMessage::Role::kSystem) {
      // Anthropic Messages API 的 system 是顶层字段，不属于 messages 数组
      if (!system_text.empty()) system_text += "\n";
      system_text += message.content;
      continue;
    }
    if (!messages_json.empty()) messages_json += ",";
    messages_json += "{\"role\":\"";
    messages_json += (message.role == LlmMessage::Role::kAssistant) ? "assistant" : "user";
    messages_json += "\",\"content\":\"" + jsonEscape(message.content) + "\"}";
  }

  if (request.json_mode) {
    // 服务本身不强制 JSON 时，用系统提示收口：模型偶尔仍会包裹代码块，
    // 调用方的解析器已按"容忍围栏"实现，两道防线叠加足够。
    if (!system_text.empty()) system_text += "\n";
    system_text += "请只输出一个 JSON 对象，不要输出任何解释文字或 Markdown 代码块。";
  }

  if (messages_json.empty()) {
    // messages 为空会被服务端判为非法请求；补一个占位用户消息让请求合法
    messages_json = "{\"role\":\"user\",\"content\":\"\"}";
  }

  std::string body = "{";
  body += "\"model\":\"" + jsonEscape(config.model) + "\",";
  body += "\"max_tokens\":" + std::to_string(request.max_tokens) + ",";
  body += "\"temperature\":" + numToString(request.temperature) + ",";
  if (!system_text.empty()) {
    body += "\"system\":\"" + jsonEscape(system_text) + "\",";
  }
  body += "\"messages\":[" + messages_json + "]";
  body += "}";
  return body;
}

std::string HttpLlmClient::Impl::makeCacheKey(const LlmRequest& request) const {
  const std::string material =
      config.endpoint + "|" + config.model + "|" + buildBody(request);
  // std::hash 的具体值不保证跨进程一致，但缓存本身也只活在进程内
  const std::size_t h = std::hash<std::string>{}(material);
  std::ostringstream oss;
  oss << std::hex << h << ":" << material.size();
  return oss.str();
}

std::string HttpLlmClient::Impl::describeRequest(
    const std::vector<LlmMessage>& messages) const {
  for (const LlmMessage& message : messages) {
    if (message.role == LlmMessage::Role::kUser && !message.content.empty()) {
      return message.content.substr(0, 80);
    }
  }
  return "<空请求>";
}

LlmResponse HttpLlmClient::Impl::performOnce(const LlmRequest& request,
                                             const std::string& body, bool& transport_ok,
                                             long& http_status) {
  transport_ok = false;
  http_status = 0;

  LlmResponse response;
  response.model = config.model;

  CURL* handle = curl_easy_init();
  if (handle == nullptr) {
    response.error = "curl_easy_init 失败：无法创建 HTTP 句柄";
    return response;
  }

  std::string response_body;
  curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "content-type: application/json");
  headers = curl_slist_append(headers, "anthropic-version: 2023-06-01");
  const std::string api_key_header = "x-api-key: " + api_key;
  headers = curl_slist_append(headers, api_key_header.c_str());

  const double timeout_seconds =
      request.timeout_seconds > 0.0
          ? std::min(request.timeout_seconds, config.request_timeout_seconds)
          : config.request_timeout_seconds;

  curl_easy_setopt(handle, CURLOPT_URL, config.endpoint.c_str());
  curl_easy_setopt(handle, CURLOPT_POST, 1L);
  curl_easy_setopt(handle, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, writeCallback);
  curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response_body);
  curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS,
                   static_cast<long>(config.connect_timeout_seconds * 1000.0));
  curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_seconds * 1000.0));
  curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);  // 多线程下必须关闭信号处理
  curl_easy_setopt(handle, CURLOPT_USERAGENT, "adsim-http-llm-client/1.0");
#if defined(CURL_HTTP_VERSION_2_0)
  curl_easy_setopt(handle, CURLOPT_HTTP_VERSION,
                   static_cast<long>(CURL_HTTP_VERSION_2TLS));
#endif

  const CURLcode code = curl_easy_perform(handle);
  if (code != CURLE_OK) {
    response.error = std::string("HTTP 请求失败: ") + curl_easy_strerror(code);
    curl_slist_free_all(headers);
    curl_easy_cleanup(handle);
    return response;
  }

  long status = 0;
  curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
  http_status = status;
  transport_ok = true;

  curl_slist_free_all(headers);
  curl_easy_cleanup(handle);

  if (status != 200) {
    // 错误响应里通常带 {"error":{"type":...,"message":...}}
    std::string message;
    if (findJsonString(response_body, "message", message) && !message.empty()) {
      response.error = "HTTP " + std::to_string(status) + ": " + message;
    } else {
      response.error = "HTTP " + std::to_string(status) + ": " + response_body.substr(0, 200);
    }
    return response;
  }

  // content 是内容块数组，取第一个文本块即可（打标场景只有一段输出）
  std::string text;
  if (!findJsonString(response_body, "text", text)) {
    response.error = "响应中未找到 content[].text: " + response_body.substr(0, 200);
    return response;
  }
  response.text = text;

  std::string model;
  if (findJsonString(response_body, "model", model)) response.model = model;

  findJsonInt(response_body, "input_tokens", response.prompt_tokens);
  findJsonInt(response_body, "output_tokens", response.completion_tokens);

  response.ok = true;
  return response;
}

// ---------------------------------------------------------------------------
//  HttpLlmClient
// ---------------------------------------------------------------------------

HttpLlmClient::HttpLlmClient(const HttpLlmConfig& config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
  impl_->api_key = config.api_key;

  if (impl_->api_key.empty()) {
    // 生产环境从环境变量取 Key，避免把密钥写进配置文件或代码
    const char* env_key = std::getenv("ANTHROPIC_API_KEY");
    if (env_key != nullptr) impl_->api_key = env_key;
  }

  if (impl_->api_key.empty()) {
    throw std::runtime_error(
        "HttpLlmClient 初始化失败：未提供 API Key。"
        "请在 HttpLlmConfig::api_key 中设置，或配置环境变量 ANTHROPIC_API_KEY。");
  }
  if (impl_->config.endpoint.empty()) {
    throw std::runtime_error("HttpLlmClient 初始化失败：endpoint 为空。");
  }

  ensureCurlGlobalInit();
}

HttpLlmClient::~HttpLlmClient() = default;

bool HttpLlmClient::compiledIn() { return true; }

bool HttpLlmClient::isAvailable() const {
  return impl_ != nullptr && !impl_->api_key.empty() && !impl_->config.endpoint.empty();
}

void HttpLlmClient::clearCache() {
  if (impl_ == nullptr) return;
  const std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->cache.clear();
  impl_->cache_order.clear();
}

LlmResponse HttpLlmClient::complete(const LlmRequest& request) {
  LlmResponse response;
  if (impl_ == nullptr) {
    response.error = "HttpLlmClient 未正确初始化";
    return response;
  }

  const std::string body = impl_->buildBody(request);
  const std::string cache_key = impl_->makeCacheKey(request);

  // ---- 查缓存 ----
  if (impl_->config.enable_cache) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto it = impl_->cache.find(cache_key);
    if (it != impl_->cache.end()) {
      ++cache_hits_;
      LlmResponse cached = it->second;
      cached.from_cache = true;
      cached.latency_ms = 0.0;  // 缓存命中不产生网络耗时
      return cached;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  const int max_retries = clamp(impl_->config.max_retries, 0, 10);

  std::string last_error = "未发起任何请求";
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    if (attempt > 0) {
      // 指数退避：250ms 起步、逐次翻倍、上限 8s，
      // 给服务端一点恢复时间，也避免重试风暴被限流得更狠。
      const long long delay_ms =
          std::min<long long>(250LL << std::min(attempt - 1, 5), 8000LL);
      ADSIM_LOG_DEBUG("LLM 请求第 ", attempt, " 次重试，等待 ", delay_ms, " ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    bool transport_ok = false;
    long http_status = 0;
    LlmResponse attempt_response = impl_->performOnce(request, body, transport_ok, http_status);

    if (attempt_response.ok) {
      attempt_response.latency_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count();

      if (impl_->config.enable_cache) {
        const std::lock_guard<std::mutex> lock(impl_->mutex);
        // 容量上限：先进先出淘汰。LLM 响应体积不小，无上限缓存迟早吃光内存。
        if (impl_->config.cache_capacity > 0 &&
            impl_->cache.size() >= impl_->config.cache_capacity &&
            !impl_->cache_order.empty()) {
          impl_->cache.erase(impl_->cache_order.front());
          impl_->cache_order.pop_front();
        }
        if (impl_->config.cache_capacity > 0) {
          impl_->cache[cache_key] = attempt_response;
          impl_->cache_order.push_back(cache_key);
        }
      }
      return attempt_response;
    }

    last_error = attempt_response.error;

    // 不可重试的错误（Key 无效、请求非法）立即返回，把配额留给能成功的调用
    if (transport_ok && !retryableStatus(http_status)) break;
  }

  response.ok = false;
  response.error = last_error + "（请求: " + impl_->describeRequest(request.messages) + "）";
  response.latency_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
          .count();
  return response;
}

}  // namespace adsim

#else  // !ADSIM_HAS_CURL

namespace adsim {
namespace {
// 未启用 libcurl 时本文件为空：HttpLlmClient 的退化实现（构造抛异常、
// compiledIn() 返回 false）位于 LlmClient.cpp，两者由 ADSIM_HAS_CURL 互斥。
}  // namespace
}  // namespace adsim

#endif  // ADSIM_HAS_CURL
