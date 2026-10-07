// =============================================================================
//  RingBuffer.h — 单生产者单消费者无锁环形缓冲
//
//  用于传感器数据采集与解析线程之间的高吞吐传递：采集线程 push，解析线程
//  pop，全程无锁、无系统调用。高频场景下（Lidar 20Hz × 数十万点）相比
//  mutex + queue 可显著降低尾延迟。
//
//  内存序说明：
//    * 生产者写数据后以 release 语义发布 tail，保证数据先于索引可见
//    * 消费者以 acquire 语义读取 tail，保证读到完整数据
//    * head / tail 分处不同 cache line，避免伪共享
// =============================================================================
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace adsim {

/// 单生产者单消费者无锁环形缓冲（容量固定，满时丢弃并计数）
template <typename T>
class SpscRingBuffer {
 public:
  /// @param capacity 期望容量，内部向上取整为 2 的幂
  explicit SpscRingBuffer(std::size_t capacity)
      : capacity_(roundUpToPowerOfTwo(capacity)),
        mask_(capacity_ - 1),
        buffer_(capacity_) {}

  SpscRingBuffer(const SpscRingBuffer&) = delete;
  SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

  /// 生产者接口：写入一个元素。缓冲区满时返回 false 并累加丢弃计数。
  bool push(const T& item) { return emplace(item); }
  bool push(T&& item) { return emplace(std::move(item)); }

  template <typename... Args>
  bool emplace(Args&&... args) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);

    if (tail - head >= capacity_) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    buffer_[tail & mask_] = T(std::forward<Args>(args)...);
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  /// 消费者接口：读取一个元素。为空时返回 false。
  bool pop(T& out) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);

    if (head == tail) {
      return false;
    }

    out = std::move(buffer_[head & mask_]);
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  /// 消费者接口：批量读取，返回实际读取数量
  std::size_t popBatch(T* out, std::size_t max_count) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    const std::size_t available = tail - head;
    const std::size_t n = available < max_count ? available : max_count;

    for (std::size_t i = 0; i < n; ++i) {
      out[i] = std::move(buffer_[(head + i) & mask_]);
    }
    if (n > 0) {
      head_.store(head + n, std::memory_order_release);
    }
    return n;
  }

  /// 当前可读元素数（近似值，供监控使用）
  std::size_t size() const {
    return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
  }

  bool empty() const { return size() == 0; }
  bool full() const { return size() >= capacity_; }
  std::size_t capacity() const { return capacity_; }

  /// 因缓冲已满而被拒绝的 push 调用次数。
  ///
  /// 注意语义：统计的是"失败的写入尝试"，不是"丢失的数据"。若生产者采用
  /// 自旋重试（push 失败后重试直到成功），该计数会随重试次数增长，但数据
  /// 并不会丢失。只有在生产者放弃重试时才等价于数据丢失量。
  std::uint64_t droppedCount() const { return dropped_.load(std::memory_order_relaxed); }
  void resetDroppedCount() { dropped_.store(0, std::memory_order_relaxed); }

  /// 清空缓冲（仅可在生产者与消费者均停止时调用）
  void clear() {
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
  }

 private:
  static std::size_t roundUpToPowerOfTwo(std::size_t v) {
    if (v <= 2) return 2;
    --v;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    return v + 1;
  }

  const std::size_t capacity_;
  const std::size_t mask_;
  std::vector<T> buffer_;

  // 索引分处独立 cache line，避免生产者与消费者之间的伪共享
  alignas(64) std::atomic<std::size_t> head_{0};  ///< 消费者持有
  alignas(64) std::atomic<std::size_t> tail_{0};  ///< 生产者持有
  alignas(64) std::atomic<std::uint64_t> dropped_{0};
};

// ---------------------------------------------------------------------------
// 单生产者多消费者场景下的替代方案：加锁环形缓冲
//
//  多消费者无法用无锁方式安全共享同一读指针，此处回退到轻量自旋锁保护，
//  仍保留"容量固定、零动态分配"的核心收益。
// ---------------------------------------------------------------------------
template <typename T>
class MpscRingBuffer {
 public:
  explicit MpscRingBuffer(std::size_t capacity)
      : capacity_(capacity), buffer_(capacity), head_(0), tail_(0) {}

  template <typename... Args>
  bool emplace(Args&&... args) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ >= capacity_) {
      ++dropped_;
      return false;
    }
    buffer_[(tail_) % capacity_] = T(std::forward<Args>(args)...);
    ++tail_;
    ++size_;
    return true;
  }

  bool pop(T& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) return false;
    out = std::move(buffer_[head_ % capacity_]);
    ++head_;
    --size_;
    return true;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }

  std::size_t capacity() const { return capacity_; }
  std::uint64_t droppedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }

 private:
  const std::size_t capacity_;
  std::vector<T> buffer_;
  mutable std::mutex mutex_;
  std::size_t head_{0};
  std::size_t tail_{0};
  std::size_t size_{0};
  std::uint64_t dropped_{0};
};

}  // namespace adsim
