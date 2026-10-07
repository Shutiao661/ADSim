// =============================================================================
//  BehaviorTree.h — 行为树
//
//  上层行为决策框架之一：与有限状态机配合，处理换道、避障、跟车等任务的
//  组合逻辑。相比纯 FSM，行为树在"可复用性"和"运行中中断/恢复"上更有优势。
//
//  节点返回三态：
//    Success — 本次任务已完成
//    Failure — 本次任务无法完成（由父节点决定是否换策略）
//    Running — 任务执行中，需要下一周期继续 tick
//
//  每 tick 从根节点深度优先遍历，只有 Running 的节点会被继续 tick；
//  切换到其他分支时，被放弃的分支会收到 halt() 以清理内部状态。
// =============================================================================
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace adsim {

/// 行为树节点执行状态
enum class NodeStatus { kSuccess, kFailure, kRunning };

const char* toString(NodeStatus status);

/// 黑板：节点间共享数据的键值存储
class Blackboard {
 public:
  void setBool(const std::string& key, bool value);
  void setDouble(const std::string& key, double value);
  void setInt(const std::string& key, int value);
  void setString(const std::string& key, const std::string& value);

  bool getBool(const std::string& key, bool default_value = false) const;
  double getDouble(const std::string& key, double default_value = 0.0) const;
  int getInt(const std::string& key, int default_value = 0) const;
  std::string getString(const std::string& key, const std::string& default_value = "") const;

  bool has(const std::string& key) const;
  void erase(const std::string& key);
  void clear();

 private:
  struct Value {
    enum class Type { kBool, kDouble, kInt, kString } type{Type::kBool};
    bool b{false};
    double d{0.0};
    int i{0};
    std::string s;
  };

  std::vector<std::pair<std::string, Value>> values_;
};

// ---------------------------------------------------------------------------
// 节点基类
// ---------------------------------------------------------------------------
class Node {
 public:
  using Ptr = std::unique_ptr<Node>;

  explicit Node(std::string name) : name_(std::move(name)) {}
  virtual ~Node() = default;

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  /// 执行一次 tick
  virtual NodeStatus tick() = 0;

  /// 中断该节点（及其子树），清理内部状态
  virtual void halt() { halted_ = true; }

  /// 节点重置，重新开始执行
  virtual void reset() { halted_ = false; }

  const std::string& name() const { return name_; }

  /// 最近一次 tick 的耗时（毫秒），用于性能分析
  double lastTickMs() const { return last_tick_ms_; }
  void setLastTickMs(double ms) { last_tick_ms_ = ms; }

  /// 累计 tick 次数与各状态计数
  std::size_t tickCount() const { return tick_count_; }
  void recordTick(NodeStatus status);

 protected:
  void beginTick() { tick_start_ = std::chrono::steady_clock::now(); }
  void endTick() {
    const auto now = std::chrono::steady_clock::now();
    last_tick_ms_ = std::chrono::duration<double, std::milli>(now - tick_start_).count();
  }

  std::string name_;
  bool halted_{false};
  std::size_t tick_count_{0};
  double last_tick_ms_{0.0};
  std::chrono::steady_clock::time_point tick_start_{};
};

// ---------------------------------------------------------------------------
// 组合节点
// ---------------------------------------------------------------------------

/// 顺序节点：依次执行子节点，任一失败则失败；全部成功才成功
class Sequence : public Node {
 public:
  explicit Sequence(std::string name) : Node(std::move(name)) {}

  void addChild(Node::Ptr child);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  const std::vector<Node::Ptr>& children() const { return children_; }
  std::size_t currentIndex() const { return current_index_; }

 private:
  std::vector<Node::Ptr> children_;
  std::size_t current_index_{0};
};

/// 选择节点：依次尝试子节点，任一成功则成功；全部失败才失败
class Selector : public Node {
 public:
  explicit Selector(std::string name) : Node(std::move(name)) {}

  void addChild(Node::Ptr child);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  const std::vector<Node::Ptr>& children() const { return children_; }
  std::size_t currentIndex() const { return current_index_; }

 private:
  std::vector<Node::Ptr> children_;
  std::size_t current_index_{0};
};

/// 并行节点：每 tick 执行所有未完成的子节点
class Parallel : public Node {
 public:
  enum class Policy {
    kRequireAll,   ///< 全部成功才成功
    kRequireOne,   ///< 任一成功即成功
  };

  Parallel(std::string name, Policy success_policy, std::size_t failure_threshold);

  void addChild(Node::Ptr child);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  const std::vector<Node::Ptr>& children() const { return children_; }
  Policy successPolicy() const { return success_policy_; }
  std::size_t failureThreshold() const { return failure_threshold_; }

 private:
  std::vector<Node::Ptr> children_;
  Policy success_policy_;
  std::size_t failure_threshold_;
  std::vector<bool> finished_;
  std::size_t success_count_{0};
  std::size_t failure_count_{0};
};

// ---------------------------------------------------------------------------
// 装饰节点
// ---------------------------------------------------------------------------

/// 反转节点：Success ↔ Failure，Running 保持不变
class Inverter : public Node {
 public:
  explicit Inverter(std::string name, Node::Ptr child);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  Node* child() const { return child_.get(); }

 private:
  Node::Ptr child_;
};

/// 重试节点：子节点失败时重试，直到成功或达到次数上限
class Retry : public Node {
 public:
  Retry(std::string name, Node::Ptr child, std::size_t max_attempts);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  Node* child() const { return child_.get(); }
  std::size_t attempts() const { return attempts_; }
  std::size_t maxAttempts() const { return max_attempts_; }

 private:
  Node::Ptr child_;
  std::size_t max_attempts_;
  std::size_t attempts_{0};
};

/// 持续节点：子节点返回 Success 时改为返回 Running，使其反复执行
/// （用于"保持车道"这类需要持续维持的行为）
class RepeatForever : public Node {
 public:
  RepeatForever(std::string name, Node::Ptr child);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  Node* child() const { return child_.get(); }

 private:
  Node::Ptr child_;
};

/// 冷却节点：子节点成功后进入冷却期，冷却期内直接返回 Failure
class Cooldown : public Node {
 public:
  Cooldown(std::string name, Node::Ptr child, double cooldown_seconds);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  Node* child() const { return child_.get(); }
  bool cooling() const { return cooling_; }
  double cooldownSeconds() const { return cooldown_seconds_; }

 private:
  Node::Ptr child_;
  double cooldown_seconds_;
  std::chrono::steady_clock::time_point last_success_{};
  bool cooling_{false};
};

/// 时间限制节点：子节点运行超时则中断并返回失败
class TimeLimit : public Node {
 public:
  TimeLimit(std::string name, Node::Ptr child, double limit_seconds);
  NodeStatus tick() override;
  void halt() override;
  void reset() override;

  Node* child() const { return child_.get(); }
  double limitSeconds() const { return limit_seconds_; }

 private:
  Node::Ptr child_;
  double limit_seconds_;
  std::chrono::steady_clock::time_point start_{};
  bool started_{false};
};

// ---------------------------------------------------------------------------
// 叶子节点
// ---------------------------------------------------------------------------

/// 动作节点：执行具体行为
class ActionNode : public Node {
 public:
  using Action = std::function<NodeStatus(Blackboard&)>;

  ActionNode(std::string name, Action action, Blackboard* blackboard);
  NodeStatus tick() override;

 private:
  Action action_;
  Blackboard* blackboard_;
};

/// 条件节点：判定是否满足条件，只返回 Success / Failure
class ConditionNode : public Node {
 public:
  using Condition = std::function<bool(const Blackboard&)>;

  ConditionNode(std::string name, Condition condition, Blackboard* blackboard);
  NodeStatus tick() override;

 private:
  Condition condition_;
  Blackboard* blackboard_;
};

// ---------------------------------------------------------------------------
// 行为树
// ---------------------------------------------------------------------------
class BehaviorTree {
 public:
  explicit BehaviorTree(Blackboard* blackboard);

  void setRoot(Node::Ptr root);
  Node* root() { return root_.get(); }
  const Node* root() const { return root_.get(); }

  /// 执行一次 tick；根节点为空时返回 Failure
  NodeStatus tick();

  /// 中断整棵树
  void halt();

  /// 重置整棵树到初始状态
  void reset();

  Blackboard& blackboard() { return *blackboard_; }
  const Blackboard& blackboard() const { return *blackboard_; }

  /// 累计 tick 次数
  std::size_t tickCount() const { return tick_count_; }

  /// 导出树结构为文本（用于调试与文档）
  std::string dump() const;

 private:
  static void dumpNode(const Node* node, int depth, std::string& out);

  Blackboard* blackboard_;
  Node::Ptr root_;
  std::size_t tick_count_{0};
};

}  // namespace adsim
