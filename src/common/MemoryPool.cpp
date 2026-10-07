#include "adsim/common/MemoryPool.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

namespace adsim {

namespace {

/// 将块大小向上对齐到 2 的幂，使注册表可以按块大小共享池实例
std::size_t roundUpToPowerOfTwo(std::size_t v) {
  if (v <= 1) return 1;
  --v;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  return v + 1;
}

constexpr std::size_t kMinSlotSize = sizeof(void*);

/// 单调递增的池编号，保证析构后的编号不会被新池复用
std::uint64_t nextPoolId() {
  static std::atomic<std::uint64_t> counter{0};
  return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}

}  // namespace

// ---------------------------------------------------------------------------
// 线程本地缓存
// ---------------------------------------------------------------------------
struct MemoryPool::ThreadCache {
  MemoryPool* pool{nullptr};
  std::vector<void*> blocks;

  ~ThreadCache() {
    // 池对象若已析构，pool 会被置空，此时无需（也不能）归还
    if (pool != nullptr) {
      pool->unregisterCache(this);
    }
  }
};

MemoryPool::ThreadCacheMap& MemoryPool::threadCacheMap() {
  static thread_local ThreadCacheMap cache_map;
  return cache_map;
}

MemoryPool::ThreadCache* MemoryPool::acquireThreadCache(MemoryPool* pool) {
  ThreadCacheMap& cache_map = threadCacheMap();

  // 以池编号为键：池析构后编号不再被复用，因此不会命中失效缓存
  auto it = cache_map.find(pool->id_);
  if (it != cache_map.end()) {
    return it->second.get();
  }

  auto cache = std::unique_ptr<ThreadCache>(new ThreadCache());
  cache->pool = pool;
  cache->blocks.reserve(pool->config_.thread_cache_capacity);
  ThreadCache* raw = cache.get();
  cache_map.emplace(pool->id_, std::move(cache));
  pool->registerCache(raw);
  return raw;
}

void MemoryPool::discardThreadCache(std::uint64_t pool_id) {
  // 仅清理当前线程的表项；其他线程的表项在池析构时已被置空，是无害的残留
  threadCacheMap().erase(pool_id);
}

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

MemoryPool::MemoryPool(const Config& config)
    : id_(nextPoolId()),
      config_(config),
      slot_size_(std::max(roundUpToPowerOfTwo(config.block_size), kMinSlotSize)) {
  if (config_.blocks_per_chunk == 0) config_.blocks_per_chunk = 1;
  if (config_.thread_cache_capacity == 0) config_.enable_thread_cache = false;
  chunk_bytes_ = slot_size_ * config_.blocks_per_chunk;
}

MemoryPool::MemoryPool(std::size_t block_size, std::size_t blocks_per_chunk)
    : MemoryPool(Config{block_size, blocks_per_chunk, 64, true}) {}

MemoryPool::~MemoryPool() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 断开所有线程缓存与本池的关联，防止线程退出时访问已析构对象。
    // 同时清空其块列表——这些块即将随 Chunk 一起释放，留用会成为悬垂指针。
    for (ThreadCache* cache : caches_) {
      cache->pool = nullptr;
      cache->blocks.clear();
    }
    caches_.clear();

    for (void* chunk : chunks_) {
      std::free(chunk);
    }
    chunks_.clear();
    central_free_ = nullptr;
    block_capacity_ = 0;
  }

  // 必须在锁外执行：表项析构会尝试调用 unregisterCache 回锁，持锁调用将死锁。
  // 此时该缓存的 pool 已被置空，析构函数不会再回锁。
  discardThreadCache(id_);
}

// ---------------------------------------------------------------------------
// 缓存注册
// ---------------------------------------------------------------------------

void MemoryPool::registerCache(ThreadCache* cache) {
  std::lock_guard<std::mutex> lock(mutex_);
  caches_.push_back(cache);
}

void MemoryPool::unregisterCache(ThreadCache* cache) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 把该线程缓存的块归还中心链表，避免线程退出导致块泄漏
  for (void* block : cache->blocks) {
    *static_cast<void**>(block) = central_free_;
    central_free_ = block;
  }
  blocks_in_cache_.fetch_sub(cache->blocks.size(), std::memory_order_relaxed);
  cache->blocks.clear();

  caches_.erase(std::remove(caches_.begin(), caches_.end(), cache), caches_.end());
}

// ---------------------------------------------------------------------------
// 中心链表操作（需持锁）
// ---------------------------------------------------------------------------

void* MemoryPool::popFromCentralUnlocked() {
  if (central_free_ == nullptr) {
    return nullptr;
  }
  void* block = central_free_;
  central_free_ = *static_cast<void**>(block);
  return block;
}

void* MemoryPool::growUnlocked() {
  // malloc 保证满足 max_align_t 对齐，可安全承载任意基本类型
  void* chunk = std::malloc(chunk_bytes_);
  if (chunk == nullptr) {
    throw std::bad_alloc();
  }
  chunks_.push_back(chunk);

  // 将 Chunk 切分为块并串入中心空闲链表
  char* base = static_cast<char*>(chunk);
  for (std::size_t i = 0; i < config_.blocks_per_chunk; ++i) {
    void* block = base + i * slot_size_;
    *static_cast<void**>(block) = central_free_;
    central_free_ = block;
  }

  block_capacity_ += config_.blocks_per_chunk;
  chunk_growths_.fetch_add(1, std::memory_order_relaxed);
  return popFromCentralUnlocked();
}

// ---------------------------------------------------------------------------
// 分配 / 归还
// ---------------------------------------------------------------------------

void* MemoryPool::allocate() {
  allocate_calls_.fetch_add(1, std::memory_order_relaxed);

  if (config_.enable_thread_cache) {
    ThreadCache* cache = acquireThreadCache(this);

    if (cache->blocks.empty()) {
      // 批量补充：一次性从中心链表取走一批，摊薄加锁成本
      std::lock_guard<std::mutex> lock(mutex_);
      const std::size_t batch = config_.thread_cache_capacity;
      for (std::size_t i = 0; i < batch; ++i) {
        void* block = popFromCentralUnlocked();
        if (block == nullptr) {
          block = growUnlocked();
        }
        cache->blocks.push_back(block);
      }
      blocks_in_cache_.fetch_add(cache->blocks.size(), std::memory_order_relaxed);
    }

    void* block = cache->blocks.back();
    cache->blocks.pop_back();
    blocks_in_cache_.fetch_sub(1, std::memory_order_relaxed);

    const std::size_t now = blocks_in_use_.fetch_add(1, std::memory_order_relaxed) + 1;
    updatePeak(now);
    thread_cache_hits_.fetch_add(1, std::memory_order_relaxed);
    return block;
  }

  // 无线程缓存路径
  std::lock_guard<std::mutex> lock(mutex_);
  void* block = popFromCentralUnlocked();
  if (block == nullptr) {
    block = growUnlocked();
  }
  central_hits_.fetch_add(1, std::memory_order_relaxed);
  const std::size_t now = blocks_in_use_.fetch_add(1, std::memory_order_relaxed) + 1;
  updatePeak(now);
  return block;
}

void MemoryPool::deallocate(void* block) {
  if (block == nullptr) return;

  deallocate_calls_.fetch_add(1, std::memory_order_relaxed);
  blocks_in_use_.fetch_sub(1, std::memory_order_relaxed);

  if (config_.enable_thread_cache) {
    ThreadCache* cache = acquireThreadCache(this);
    if (cache->blocks.size() < config_.thread_cache_capacity) {
      cache->blocks.push_back(block);
      blocks_in_cache_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  *static_cast<void**>(block) = central_free_;
  central_free_ = block;
  central_hits_.fetch_add(1, std::memory_order_relaxed);
}

void MemoryPool::updatePeak(std::size_t current) {
  std::size_t peak = peak_blocks_in_use_.load(std::memory_order_relaxed);
  while (current > peak &&
         !peak_blocks_in_use_.compare_exchange_weak(peak, current, std::memory_order_relaxed)) {
    // CAS 失败时 peak 已被刷新为最新值，循环重试
  }
}

// ---------------------------------------------------------------------------
// 容量管理
// ---------------------------------------------------------------------------

std::size_t MemoryPool::blockCapacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return block_capacity_;
}

void MemoryPool::reserve(std::size_t block_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  while (block_capacity_ < block_count) {
    // growUnlocked 会把新块串入链表，这里不需要返回值
    void* block = growUnlocked();
    if (block != nullptr) {
      *static_cast<void**>(block) = central_free_;
      central_free_ = block;
    }
  }
}

// ---------------------------------------------------------------------------
// 统计
// ---------------------------------------------------------------------------

MemoryPool::Stats MemoryPool::stats() const {
  Stats s;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    s.chunk_count = chunks_.size();
    s.block_capacity = block_capacity_;
  }
  s.block_size = slot_size_;
  s.blocks_in_use = blocks_in_use_.load(std::memory_order_relaxed);
  s.blocks_in_thread_cache = blocks_in_cache_.load(std::memory_order_relaxed);
  s.allocate_calls = allocate_calls_.load(std::memory_order_relaxed);
  s.deallocate_calls = deallocate_calls_.load(std::memory_order_relaxed);
  s.thread_cache_hits = thread_cache_hits_.load(std::memory_order_relaxed);
  s.central_hits = central_hits_.load(std::memory_order_relaxed);
  s.chunk_growths = chunk_growths_.load(std::memory_order_relaxed);
  s.memory_footprint_bytes = s.chunk_count * chunk_bytes_;
  s.peak_blocks_in_use = static_cast<double>(peak_blocks_in_use_.load(std::memory_order_relaxed));

  const std::size_t total = s.thread_cache_hits + s.central_hits;
  s.thread_cache_hit_rate =
      total == 0 ? 0.0 : static_cast<double>(s.thread_cache_hits) / static_cast<double>(total);
  return s;
}

void MemoryPool::resetStats() {
  allocate_calls_.store(0, std::memory_order_relaxed);
  deallocate_calls_.store(0, std::memory_order_relaxed);
  thread_cache_hits_.store(0, std::memory_order_relaxed);
  central_hits_.store(0, std::memory_order_relaxed);
  chunk_growths_.store(0, std::memory_order_relaxed);
  peak_blocks_in_use_.store(blocks_in_use_.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// 全局注册表
// ---------------------------------------------------------------------------

MemoryPoolRegistry& MemoryPoolRegistry::instance() {
  static MemoryPoolRegistry registry;
  return registry;
}

MemoryPool& MemoryPoolRegistry::getImpl(std::size_t block_size) {
  const std::size_t key = roundUpToPowerOfTwo(std::max(block_size, kMinSlotSize));
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : pools_) {
    if (entry.first == key) {
      return *entry.second;
    }
  }
  auto pool = std::unique_ptr<MemoryPool>(new MemoryPool(key));
  MemoryPool* raw = pool.get();
  pools_.emplace_back(key, std::move(pool));
  return *raw;
}

MemoryPool& MemoryPoolRegistry::get(std::size_t block_size) {
  return instance().getImpl(block_size);
}

std::vector<std::pair<std::size_t, MemoryPool::Stats>> MemoryPoolRegistry::allStats() {
  MemoryPoolRegistry& reg = instance();
  std::lock_guard<std::mutex> lock(reg.mutex_);
  std::vector<std::pair<std::size_t, MemoryPool::Stats>> result;
  result.reserve(reg.pools_.size());
  for (auto& entry : reg.pools_) {
    result.emplace_back(entry.first, entry.second->stats());
  }
  return result;
}

}  // namespace adsim
