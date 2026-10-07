// =============================================================================
//  BehaviorTree.cpp — 行为树实现
//
//  实现要点：
//    * 组合节点把"执行到第几个子节点"记为下标：子节点返回 Running 时下标保持
//      不动，下个周期接着 tick 同一个子节点。这是行为树能够"运行中中断/恢复"
//      的根本原因，也是本文件最容易写错的地方。
//    * 每个 tick() 以 beginTick() 开头，退出时由作用域守卫统一调用
//      endTick() + recordTick()。tick() 里有多条 return 路径（Running /
//      Failure / Success），手写收尾极易漏掉其中一条。
//    * halt() 只沿"正在执行"的那条分支向下传播，不打扰兄弟分支；
//      被放弃的分支必须收到 halt() 才能清理自身状态（例如组合节点的下标）。
//    * 显式 reset() 表示"整棵子树重新开始"，因此向下级联；而 tick() 内部的
//      失败复位只清本节点自己的执行状态，不必牵动子节点。
// =============================================================================
#include "adsim/planning/BehaviorTree.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace adsim {

namespace {

/// 作用域守卫：析构时执行收尾动作。
/// 用 RAII 而不是"在每个 return 前手写收尾"，是为了保证任何返回路径
/// （包括将来新增的分支）都不会漏掉耗时统计与 tick 计数。
template <typename Finalize>
class ScopeExit {
 public:
  explicit ScopeExit(Finalize finalize) : finalize_(std::move(finalize)) {}
  ~ScopeExit() { finalize_(); }

  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  ScopeExit(ScopeExit&&) = delete;
  ScopeExit& operator=(ScopeExit&&) = delete;

 private:
  Finalize finalize_;
};

/// 构造守卫的工厂函数。C++17 保证返回值直接构造到调用方对象上，
/// 因此 ScopeExit 即使不可拷贝、不可移动也能这样返回。
template <typename Finalize>
ScopeExit<Finalize> onScopeExit(Finalize finalize) {
  return ScopeExit<Finalize>(std::move(finalize));
}

/// 两个时间点之间的间隔（秒）
double secondsBetween(const std::chrono::steady_clock::time_point& from,
                      const std::chrono::steady_clock::time_point& to) {
  return std::chrono::duration<double>(to - from).count();
}

}  // namespace

// ---------------------------------------------------------------------------
// 状态名
// ---------------------------------------------------------------------------

const char* toString(NodeStatus status) {
  switch (status) {
    case NodeStatus::kSuccess:
      return "Success";
    case NodeStatus::kFailure:
      return "Failure";
    case NodeStatus::kRunning:
      return "Running";
  }
  // 枚举被越界赋值（如从外部字节流转来）时的兜底，避免未定义行为
  return "Unknown";
}

// ---------------------------------------------------------------------------
// 黑板
// ---------------------------------------------------------------------------
//  存储用 vector 而非 unordered_map：行为树的键通常只有十几个，线性查找
//  在缓存上更友好，也省去了哈希与节点分配的开销。

void Blackboard::setBool(const std::string& key, bool value) {
  // 键已存在则原地改写（连同类型），否则追加——保证同一键只有一份值
  auto it = std::find_if(values_.begin(), values_.end(),
                         [&key](const std::pair<std::string, Value>& kv) {
                           return kv.first == key;
                         });
  if (it == values_.end()) {
    values_.emplace_back(key, Value{});
    it = std::prev(values_.end());
  }
  it->second.type = Value::Type::kBool;
  it->second.b = value;
}

void Blackboard::setDouble(const std::string& key, double value) {
  auto it = std::find_if(values_.begin(), values_.end(),
                         [&key](const std::pair<std::string, Value>& kv) {
                           return kv.first == key;
                         });
  if (it == values_.end()) {
    values_.emplace_back(key, Value{});
    it = std::prev(values_.end());
  }
  it->second.type = Value::Type::kDouble;
  it->second.d = value;
}

void Blackboard::setInt(const std::string& key, int value) {
  auto it = std::find_if(values_.begin(), values_.end(),
                         [&key](const std::pair<std::string, Value>& kv) {
                           return kv.first == key;
                         });
  if (it == values_.end()) {
    values_.emplace_back(key, Value{});
    it = std::prev(values_.end());
  }
  it->second.type = Value::Type::kInt;
  it->second.i = value;
}

void Blackboard::setString(const std::string& key, const std::string& value) {
  auto it = std::find_if(values_.begin(), values_.end(),
                         [&key](const std::pair<std::string, Value>& kv) {
                           return kv.first == key;
                         });
  if (it == values_.end()) {
    values_.emplace_back(key, Value{});
    it = std::prev(values_.end());
  }
  it->second.type = Value::Type::kString;
  it->second.s = value;
}

bool Blackboard::getBool(const std::string& key, bool default_value) const {
  for (const auto& kv : values_) {
    if (kv.first != key) continue;
    // 类型不匹配时回退默认值而不是硬转：把 0.0 当 false、把非零 int 当 true
    // 这类隐式转换会静默改变决策语义，宁可让调用方明确拿到"没读到"。
    return kv.second.type == Value::Type::kBool ? kv.second.b : default_value;
  }
  return default_value;  // 键不存在
}

double Blackboard::getDouble(const std::string& key, double default_value) const {
  for (const auto& kv : values_) {
    if (kv.first != key) continue;
    return kv.second.type == Value::Type::kDouble ? kv.second.d : default_value;
  }
  return default_value;
}

int Blackboard::getInt(const std::string& key, int default_value) const {
  for (const auto& kv : values_) {
    if (kv.first != key) continue;
    return kv.second.type == Value::Type::kInt ? kv.second.i : default_value;
  }
  return default_value;
}

std::string Blackboard::getString(const std::string& key,
                                 const std::string& default_value) const {
  for (const auto& kv : values_) {
    if (kv.first != key) continue;
    return kv.second.type == Value::Type::kString ? kv.second.s : default_value;
  }
  return default_value;
}

bool Blackboard::has(const std::string& key) const {
  return std::any_of(values_.begin(), values_.end(),
                     [&key](const std::pair<std::string, Value>& kv) {
                       return kv.first == key;
                     });
}

void Blackboard::erase(const std::string& key) {
  values_.erase(std::remove_if(values_.begin(), values_.end(),
                               [&key](const std::pair<std::string, Value>& kv) {
                                 return kv.first == key;
                               }),
                values_.end());
}

void Blackboard::clear() { values_.clear(); }

// ---------------------------------------------------------------------------
// 节点基类
// ---------------------------------------------------------------------------

void Node::recordTick(NodeStatus status) {
  // 头文件把耗时统计交给 beginTick()/endTick()（见各 tick() 中的作用域守卫），
  // recordTick() 只负责累加次数，两者职责不重叠。
  // status 目前不参与存储：头文件未提供按状态分类的计数器，显式求值
  // 是为满足 -Wextra 的未使用参数检查。
  (void)status;
  ++tick_count_;
}

// ---------------------------------------------------------------------------
// 顺序节点
// ---------------------------------------------------------------------------

void Sequence::addChild(Node::Ptr child) {
  // 空指针不入树：否则 tick() 需要处处判空，且装配错误应当尽早暴露
  if (child) children_.push_back(std::move(child));
}

NodeStatus Sequence::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kSuccess;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  while (current_index_ < children_.size()) {
    const NodeStatus child_status = children_[current_index_]->tick();

    if (child_status == NodeStatus::kRunning) {
      // 关键语义：下标保持不动，下个周期继续 tick 同一个子节点（断点续跑）
      result = NodeStatus::kRunning;
      return result;
    }
    if (child_status == NodeStatus::kFailure) {
      // 任一子节点失败即整体失败，并清零自身执行状态，下次从头再来
      Node::reset();
      current_index_ = 0;
      result = NodeStatus::kFailure;
      return result;
    }
    // 子节点成功：同一次 tick 内继续推进到下一个子节点（行为树标准语义，
    // 否则一个纯 Success 序列会被拆散到多个周期，白白拉长决策延迟）
    ++current_index_;
  }

  // 所有子节点都成功：本次任务完成，下标归零以便下个周期重新执行整条序列
  Node::reset();
  current_index_ = 0;
  result = NodeStatus::kSuccess;
  return result;
}

void Sequence::halt() {
  Node::halt();
  // 只中断"正在执行"的那条分支：下标所指的子节点即上次 tick 未完成的分支
  if (current_index_ < children_.size()) children_[current_index_]->halt();
  current_index_ = 0;
}

void Sequence::reset() {
  Node::reset();
  current_index_ = 0;
  // 显式 reset() 表示整棵子树重新开始，因此向下级联
  for (Node::Ptr& child : children_) {
    if (child) child->reset();
  }
}

// ---------------------------------------------------------------------------
// 选择节点（与顺序节点对偶）
// ---------------------------------------------------------------------------

void Selector::addChild(Node::Ptr child) {
  if (child) children_.push_back(std::move(child));
}

NodeStatus Selector::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  while (current_index_ < children_.size()) {
    const NodeStatus child_status = children_[current_index_]->tick();

    if (child_status == NodeStatus::kRunning) {
      // 与前一个分支的"约定"尚未结束，下标不动，下个周期继续问它
      result = NodeStatus::kRunning;
      return result;
    }
    if (child_status == NodeStatus::kSuccess) {
      // 找到可用分支：本次决策完成，下标归零以便下个周期重新评估
      Node::reset();
      current_index_ = 0;
      result = NodeStatus::kSuccess;
      return result;
    }
    // 失败：立刻换下一个分支尝试
    ++current_index_;
  }

  // 所有分支都失败
  Node::reset();
  current_index_ = 0;
  result = NodeStatus::kFailure;
  return result;
}

void Selector::halt() {
  Node::halt();
  if (current_index_ < children_.size()) children_[current_index_]->halt();
  current_index_ = 0;
}

void Selector::reset() {
  Node::reset();
  current_index_ = 0;
  for (Node::Ptr& child : children_) {
    if (child) child->reset();
  }
}

// ---------------------------------------------------------------------------
// 并行节点
// ---------------------------------------------------------------------------

Parallel::Parallel(std::string name, Policy success_policy,
                   std::size_t failure_threshold)
    : Node(std::move(name)),
      success_policy_(success_policy),
      failure_threshold_(failure_threshold) {}

void Parallel::addChild(Node::Ptr child) {
  if (!child) return;
  children_.push_back(std::move(child));
  // 完成标记与子节点同步增长，保证两者下标一一对应
  finished_.push_back(false);
}

NodeStatus Parallel::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kRunning;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  for (std::size_t i = 0; i < children_.size(); ++i) {
    if (finished_[i]) continue;  // 已完成的子节点本周期不再执行
    const NodeStatus child_status = children_[i]->tick();
    if (child_status == NodeStatus::kRunning) continue;  // 仍在执行，下周期继续
    finished_[i] = true;
    if (child_status == NodeStatus::kSuccess) {
      ++success_count_;
    } else {
      ++failure_count_;
    }
  }

  // 计数是累计值：子节点可能分处不同周期完成，只有累计才能正确判定终态
  const bool all_finished = (success_count_ + failure_count_) == children_.size();

  if (success_policy_ == Policy::kRequireAll &&
      success_count_ == children_.size()) {
    result = NodeStatus::kSuccess;
  } else if (success_policy_ == Policy::kRequireOne && success_count_ > 0) {
    result = NodeStatus::kSuccess;
  } else if (failure_count_ >= failure_threshold_) {
    // 失败数达到阈值：提前放弃，不再等其他分支（阈值 0 属退化配置，
    // 语义上等价于"一 tick 就放弃"，正常使用应不小于 1）
    result = NodeStatus::kFailure;
  } else if (all_finished) {
    // 所有子节点都已结束却没有凑够成功条件：本节点不可能再成功。
    // 必须判失败而不是继续返回 Running，否则整棵树会永久卡在这一支。
    result = NodeStatus::kFailure;
  }

  if (result != NodeStatus::kRunning) {
    // 进入终态：清空累计计数与完成标记，便于下一次重新执行
    success_count_ = 0;
    failure_count_ = 0;
    std::fill(finished_.begin(), finished_.end(), false);
  }
  return result;
}

void Parallel::halt() {
  Node::halt();
  // 未完成的子节点可能正在执行，逐个中断；已完成的无需打扰
  for (std::size_t i = 0; i < children_.size() && i < finished_.size(); ++i) {
    if (!finished_[i]) children_[i]->halt();
  }
  // 中断后从干净状态重新开始
  success_count_ = 0;
  failure_count_ = 0;
  std::fill(finished_.begin(), finished_.end(), false);
}

void Parallel::reset() {
  Node::reset();
  success_count_ = 0;
  failure_count_ = 0;
  std::fill(finished_.begin(), finished_.end(), false);
  for (Node::Ptr& child : children_) {
    if (child) child->reset();
  }
}

// ---------------------------------------------------------------------------
// 装饰节点
// ---------------------------------------------------------------------------

Inverter::Inverter(std::string name, Node::Ptr child)
    : Node(std::move(name)), child_(std::move(child)) {}

NodeStatus Inverter::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!child_) return result;  // 未挂子节点属于装配错误，统一按失败处理

  switch (child_->tick()) {
    case NodeStatus::kSuccess:
      result = NodeStatus::kFailure;
      break;
    case NodeStatus::kFailure:
      result = NodeStatus::kSuccess;
      break;
    case NodeStatus::kRunning:
      // Running 是"正在执行"而非结论，反转它没有意义，原样透传
      result = NodeStatus::kRunning;
      break;
  }
  return result;
}

void Inverter::halt() {
  Node::halt();
  if (child_) child_->halt();
}

void Inverter::reset() {
  Node::reset();
  if (child_) child_->reset();
}

Retry::Retry(std::string name, Node::Ptr child, std::size_t max_attempts)
    : Node(std::move(name)),
      child_(std::move(child)),
      max_attempts_(max_attempts) {}

NodeStatus Retry::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!child_) return result;

  // 上限内的重试在同一次 tick 内完成：Retry 表达的是"这次尝试必须成功"，
  // 若把重试摊到多个周期，父节点就无法区分"正在重试"与"子节点自身在运行"。
  while (attempts_ < max_attempts_) {
    const NodeStatus child_status = child_->tick();

    if (child_status == NodeStatus::kSuccess) {
      result = NodeStatus::kSuccess;
      return result;
    }
    if (child_status == NodeStatus::kRunning) {
      // 子节点只是还没做完，不算一次失败的尝试；
      // 否则耗时长但最终会成功的子节点会被重试额度提前掐死
      result = NodeStatus::kRunning;
      return result;
    }

    // 子节点失败：计一次尝试，额度没用完就立刻重试
    // （刻意不复位子节点——"重试"就是再问一次；若某个子节点需要从头开始，
    //  应由它自己在失败时复位，组合节点本来就会这么做）
    ++attempts_;
    if (attempts_ >= max_attempts_) {
      result = NodeStatus::kFailure;
      return result;
    }
  }

  // 额度已用尽：不再执行子节点，等上层 halt()/reset() 给出新的额度
  return result;
}

void Retry::halt() {
  Node::halt();
  if (child_) child_->halt();
  attempts_ = 0;  // 被中断后重试额度重新计数，否则再次进入这一支会直接判失败
}

void Retry::reset() {
  Node::reset();
  attempts_ = 0;
  if (child_) child_->reset();
}

RepeatForever::RepeatForever(std::string name, Node::Ptr child)
    : Node(std::move(name)), child_(std::move(child)) {}

NodeStatus RepeatForever::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!child_) return result;

  const NodeStatus child_status = child_->tick();
  if (child_status == NodeStatus::kFailure) {
    // 失败要透传：持续执行不等于"永远成功"，上层仍需有机会换策略
    result = NodeStatus::kFailure;
    return result;
  }
  if (child_status == NodeStatus::kRunning) {
    result = NodeStatus::kRunning;
    return result;
  }

  // 子节点成功 → 本节点返回 Running，表示"这个行为要一直保持下去"
  // （例如保持车道）。复位子节点，下个周期它才能重新执行一遍。
  child_->reset();
  result = NodeStatus::kRunning;
  return result;
}

void RepeatForever::halt() {
  Node::halt();
  if (child_) child_->halt();
}

void RepeatForever::reset() {
  Node::reset();
  if (child_) child_->reset();
}

Cooldown::Cooldown(std::string name, Node::Ptr child, double cooldown_seconds)
    : Node(std::move(name)),
      child_(std::move(child)),
      cooldown_seconds_(cooldown_seconds) {}

NodeStatus Cooldown::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!child_) return result;

  if (cooling_ && secondsBetween(last_success_, std::chrono::steady_clock::now()) <
                      cooldown_seconds_) {
    // 冷却期内直接失败，且不 tick 子节点——子节点往往是重动作
    // （换道、鸣笛），这正是冷却要拦住的重复触发。
    // 返回 Failure 而非 Running：本节点此刻确实没做事，上层应转去别的分支。
    return result;
  }
  cooling_ = false;  // 冷却结束（或本就未冷却）

  result = child_->tick();
  if (result == NodeStatus::kSuccess) {
    // 以真实成功时刻起算，而不是本次 tick 的进入时刻
    last_success_ = std::chrono::steady_clock::now();
    cooling_ = true;
  }
  return result;
}

void Cooldown::halt() {
  Node::halt();
  if (child_) child_->halt();
  // 刻意不清 cooling_：冷却约束的是墙钟时间上的触发频率，
  // 与执行是否被中断无关，中断后重新进入也不应立即再次触发。
}

void Cooldown::reset() {
  Node::reset();
  if (child_) child_->reset();
  cooling_ = false;
  last_success_ = std::chrono::steady_clock::time_point{};
}

TimeLimit::TimeLimit(std::string name, Node::Ptr child, double limit_seconds)
    : Node(std::move(name)),
      child_(std::move(child)),
      limit_seconds_(limit_seconds) {}

NodeStatus TimeLimit::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!child_) return result;

  if (!started_) {
    // 计时从"子节点开始执行"这一刻起，而不是节点构造时刻
    start_ = std::chrono::steady_clock::now();
    started_ = true;
  }

  result = child_->tick();

  if (result == NodeStatus::kRunning) {
    // 用子节点执行完之后的时刻判定，这样耗时的子节点本身也算进超时
    if (secondsBetween(start_, std::chrono::steady_clock::now()) >=
        limit_seconds_) {
      // 超时：中断子节点（它可能还持有半成品状态），下次 tick 重新计时
      child_->halt();
      started_ = false;
      result = NodeStatus::kFailure;
    }
    return result;
  }

  // 子节点已给出结论，本次计时结束；下次执行重新开始计时
  started_ = false;
  return result;
}

void TimeLimit::halt() {
  Node::halt();
  if (child_) child_->halt();
  started_ = false;
}

void TimeLimit::reset() {
  Node::reset();
  if (child_) child_->reset();
  started_ = false;
}

// ---------------------------------------------------------------------------
// 叶子节点
// ---------------------------------------------------------------------------

ActionNode::ActionNode(std::string name, Action action, Blackboard* blackboard)
    : Node(std::move(name)),
      action_(std::move(action)),
      blackboard_(blackboard) {}

NodeStatus ActionNode::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  // 回调未设置（或黑板未绑定）时安全失败：一个没装配好的节点只应让这一支
  // 失败，不应把整棵树的 tick 变成异常或崩溃。
  if (!action_ || blackboard_ == nullptr) return result;

  result = action_(*blackboard_);
  return result;
}

ConditionNode::ConditionNode(std::string name, Condition condition,
                             Blackboard* blackboard)
    : Node(std::move(name)),
      condition_(std::move(condition)),
      blackboard_(blackboard) {}

NodeStatus ConditionNode::tick() {
  beginTick();
  NodeStatus result = NodeStatus::kFailure;
  const auto scope = onScopeExit([this, &result]() {
    endTick();
    recordTick(result);
  });

  if (!condition_ || blackboard_ == nullptr) return result;

  // 条件节点没有 Running：它只回答"满足/不满足"
  result = condition_(*blackboard_) ? NodeStatus::kSuccess : NodeStatus::kFailure;
  return result;
}

// ---------------------------------------------------------------------------
// 行为树
// ---------------------------------------------------------------------------

BehaviorTree::BehaviorTree(Blackboard* blackboard) : blackboard_(blackboard) {}

void BehaviorTree::setRoot(Node::Ptr root) { root_ = std::move(root); }

NodeStatus BehaviorTree::tick() {
  // 空树没有可执行的行为，按失败处理，让调用方明确知道树没装配好
  if (!root_) return NodeStatus::kFailure;

  const NodeStatus status = root_->tick();
  // 计数在根 tick 之后累加：即使根节点内部抛出异常也不会算作一次有效 tick
  ++tick_count_;
  return status;
}

void BehaviorTree::halt() {
  if (root_) root_->halt();
}

void BehaviorTree::reset() {
  // tick_count_ 是累计诊断量，不随 reset 清零，否则性能统计会出现断层
  if (root_) root_->reset();
}

namespace {

/// 节点类型名：头文件未提供 type() 虚函数，只能用 dynamic_cast 判定。
/// 各类之间没有继承关系，判定顺序不影响结果。
const char* nodeTypeName(const Node* node) {
  if (dynamic_cast<const Sequence*>(node) != nullptr) return "Sequence";
  if (dynamic_cast<const Selector*>(node) != nullptr) return "Selector";
  if (dynamic_cast<const Parallel*>(node) != nullptr) return "Parallel";
  if (dynamic_cast<const Inverter*>(node) != nullptr) return "Inverter";
  if (dynamic_cast<const Retry*>(node) != nullptr) return "Retry";
  if (dynamic_cast<const RepeatForever*>(node) != nullptr) return "RepeatForever";
  if (dynamic_cast<const Cooldown*>(node) != nullptr) return "Cooldown";
  if (dynamic_cast<const TimeLimit*>(node) != nullptr) return "TimeLimit";
  if (dynamic_cast<const ActionNode*>(node) != nullptr) return "ActionNode";
  if (dynamic_cast<const ConditionNode*>(node) != nullptr) return "ConditionNode";
  return "Node";
}

/// Parallel 策略的可读名字。头文件没有提供这个转换，
/// 因此在实现文件内本地提供一个，避免为了打印去污染公开接口。
const char* policyName(Parallel::Policy policy) {
  switch (policy) {
    case Parallel::Policy::kRequireAll:
      return "RequireAll";
    case Parallel::Policy::kRequireOne:
      return "RequireOne";
  }
  return "Unknown";  // 枚举被越界赋值时的兜底
}

/// 秒数格式化为固定三位小数：dump 是给人看的调试输出，
/// 精度统一才便于不同节点之间对齐比较。
std::string formatSeconds(double seconds) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << seconds;
  return oss.str();
}

/// 追加节点的配置值与运行状态，形如 " max_attempts=3 attempts=0"
/// （紧跟节点名之后）。调试一棵"卡住不动"的树时，配置值回答
/// "阈值/时限是多少"，运行态回答"重试了几次""是否还在冷却"。
void appendNodeState(const Node* node, std::string& out) {
  if (const auto* parallel = dynamic_cast<const Parallel*>(node)) {
    out += " policy=";
    out += policyName(parallel->successPolicy());
    out += " failure_threshold=";
    out += std::to_string(parallel->failureThreshold());
    return;
  }
  if (const auto* retry = dynamic_cast<const Retry*>(node)) {
    out += " max_attempts=";
    out += std::to_string(retry->maxAttempts());
    out += " attempts=";
    out += std::to_string(retry->attempts());
    return;
  }
  if (const auto* cooldown = dynamic_cast<const Cooldown*>(node)) {
    out += " cooldown_seconds=";
    out += formatSeconds(cooldown->cooldownSeconds());
    out += " cooling=";
    out += cooldown->cooling() ? "true" : "false";
    return;
  }
  if (const auto* time_limit = dynamic_cast<const TimeLimit*>(node)) {
    out += " limit_seconds=";
    out += formatSeconds(time_limit->limitSeconds());
    return;
  }
  // 其余节点（组合节点 / Inverter / RepeatForever / 叶子节点）没有可标注的配置
}

}  // namespace

std::string BehaviorTree::dump() const {
  // 空树导出空串：没有结构可描述
  if (!root_) return std::string();

  std::string out;
  dumpNode(root_.get(), 0, out);
  return out;
}

void BehaviorTree::dumpNode(const Node* node, int depth, std::string& out) {
  if (node == nullptr) return;

  // 每层两个空格，层次越深缩进越多。形如：Selector"避障决策"
  out.append(static_cast<std::size_t>(depth) * 2, ' ');
  out += nodeTypeName(node);
  out += '"';
  out += node->name();
  out += '"';
  appendNodeState(node, out);
  out += '\n';

  // 组合节点：按顺序展开全部子节点
  if (const auto* sequence = dynamic_cast<const Sequence*>(node)) {
    for (const Node::Ptr& child : sequence->children()) {
      dumpNode(child.get(), depth + 1, out);
    }
    return;
  }
  if (const auto* selector = dynamic_cast<const Selector*>(node)) {
    for (const Node::Ptr& child : selector->children()) {
      dumpNode(child.get(), depth + 1, out);
    }
    return;
  }
  if (const auto* parallel = dynamic_cast<const Parallel*>(node)) {
    for (const Node::Ptr& child : parallel->children()) {
      dumpNode(child.get(), depth + 1, out);
    }
    return;
  }

  // 装饰节点：展开其唯一的子节点
  if (const auto* inverter = dynamic_cast<const Inverter*>(node)) {
    dumpNode(inverter->child(), depth + 1, out);
    return;
  }
  if (const auto* retry = dynamic_cast<const Retry*>(node)) {
    dumpNode(retry->child(), depth + 1, out);
    return;
  }
  if (const auto* repeat = dynamic_cast<const RepeatForever*>(node)) {
    dumpNode(repeat->child(), depth + 1, out);
    return;
  }
  if (const auto* cooldown = dynamic_cast<const Cooldown*>(node)) {
    dumpNode(cooldown->child(), depth + 1, out);
    return;
  }
  if (const auto* time_limit = dynamic_cast<const TimeLimit*>(node)) {
    dumpNode(time_limit->child(), depth + 1, out);
    return;
  }
  // 叶子节点（ActionNode / ConditionNode）没有子树，到此为止
}

}  // namespace adsim
