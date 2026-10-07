// =============================================================================
//  MemoryPool.h — 定长块内存池
//
//  设计目标：消除高频数据（Lidar 点云 / 车辆状态）序列化与反序列化过程中的
//  逐条 malloc/free 开销与内存碎片。
//
//  采用三层结构：
//    1. Chunk       — 向系统批量申请大块内存（默认 1024 块/Chunk），摊薄系统调用
//    2. CentralFree — 全局空闲链表，用块自身内存做侵入式链表，零额外开销
//    3. ThreadCache — 线程本地缓存，一次从中心链表批量取 N 块，减少锁竞争
//
//  在 16 线程解析场景下，线程本地缓存可将锁竞争降低约 1~2 个数量级。
// =============================================================================
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <unordered_map>
#include <vector>

namespace adsim {

class MemoryPool {
 public:
  struct Config {
    std::size_t block_size{256};           ///< 单块字节数
    std::size_t blocks_per_chunk{1024};    ///< 每次向系统申请的块数
    std::size_t thread_cache_capacity{64}; ///< 每线程缓存块上限
    bool enable_thread_cache{true};        ///< 关闭后所有分配走中心链表
  };

  struct Stats {
    std::size_t block_size{0};
    std::size_t chunk_count{0};
    std::size_t block_capacity{0};       ///< 已从系统申请的块总数
    std::size_t blocks_in_use{0};        ///< 当前被业务持有
    std::size_t blocks_in_thread_cache{0};
    std::size_t allocate_calls{0};
    std::size_t deallocate_calls{0};
    std::size_t thread_cache_hits{0};
    std::size_t central_hits{0};
    std::size_t chunk_growths{0};        ///< Chunk 扩容次数
    std::size_t memory_footprint_bytes{0};  ///< 向系统申请的总字节数
    double thread_cache_hit_rate{0.0};
    double peak_blocks_in_use{0.0};
  };

  explicit MemoryPool(const Config& config);
  explicit MemoryPool(std::size_t block_size, std::size_t blocks_per_chunk = 1024);
  ~MemoryPool();

  MemoryPool(const MemoryPool&) = delete;
  MemoryPool& operator=(const MemoryPool&) = delete;

  /// 分配一块内存；块大小由构造时确定。线程安全。
  void* allocate();

  /// 归还内存块；传入 nullptr 安全。
  void deallocate(void* block);

  std::size_t blockSize() const { return slot_size_; }
  std::size_t blockCapacity() const;
  std::size_t blocksInUse() const { return blocks_in_use_.load(std::memory_order_relaxed); }

  /// 池实例的全局唯一编号，供线程本地缓存做键使用
  std::uint64_t id() const { return id_; }

  /// 预申请指定数量的块，避免运行期扩容抖动
  void reserve(std::size_t block_count);

  Stats stats() const;
  void resetStats();

 private:
  struct ThreadCache;
  using ThreadCacheMap = std::unordered_map<std::uint64_t, std::unique_ptr<ThreadCache>>;

  /// 当前线程的缓存表（每线程一份）
  static ThreadCacheMap& threadCacheMap();

  /// 在中心链表中取一块（内部无锁保护，调用方需持锁）
  void* popFromCentralUnlocked();
  /// 申请一个新的 Chunk，返回其中第一块
  void* growUnlocked();

  static ThreadCache* acquireThreadCache(MemoryPool* pool);
  static void discardThreadCache(std::uint64_t pool_id);

  void registerCache(ThreadCache* cache);
  void unregisterCache(ThreadCache* cache);

  /// 全局唯一编号。线程本地缓存必须以它为键，绝不能用 this 指针——
  /// 池对象析构后栈/堆地址可能被新池复用，用地址做键会命中已失效的缓存，
  /// 进而返回指向已释放 Chunk 的悬垂块。
  const std::uint64_t id_;

  Config config_;
  std::size_t slot_size_{0};             ///< 对齐后的实际槽位大小
  std::size_t chunk_bytes_{0};

  mutable std::mutex mutex_;
  void* central_free_{nullptr};
  std::vector<void*> chunks_;
  std::vector<ThreadCache*> caches_;
  std::size_t block_capacity_{0};

  std::atomic<std::size_t> blocks_in_use_{0};
  std::atomic<std::size_t> blocks_in_cache_{0};
  std::atomic<std::size_t> allocate_calls_{0};
  std::atomic<std::size_t> deallocate_calls_{0};
  std::atomic<std::size_t> thread_cache_hits_{0};
  std::atomic<std::size_t> central_hits_{0};
  std::atomic<std::size_t> chunk_growths_{0};
  std::atomic<std::size_t> peak_blocks_in_use_{0};

  void updatePeak(std::size_t current);
};

// ---------------------------------------------------------------------------
//  PooledBlock — 内存池块的 RAII 句柄
//
//  用法：
//      MemoryPool pool(4096);
//      {
//        PooledBlock block(pool);
//        std::memcpy(block.data(), src, n);
//        ...
//      }  // 自动归还
// ---------------------------------------------------------------------------
class PooledBlock {
 public:
  PooledBlock() = default;
  explicit PooledBlock(MemoryPool& pool) : pool_(&pool), data_(pool.allocate()) {}

  PooledBlock(PooledBlock&& other) noexcept
      : pool_(other.pool_), data_(other.data_) {
    other.pool_ = nullptr;
    other.data_ = nullptr;
  }

  PooledBlock& operator=(PooledBlock&& other) noexcept {
    if (this != &other) {
      release();
      pool_ = other.pool_;
      data_ = other.data_;
      other.pool_ = nullptr;
      other.data_ = nullptr;
    }
    return *this;
  }

  PooledBlock(const PooledBlock&) = delete;
  PooledBlock& operator=(const PooledBlock&) = delete;

  ~PooledBlock() { release(); }

  void* data() { return data_; }
  const void* data() const { return data_; }

  template <typename T>
  T* as() {
    return static_cast<T*>(data_);
  }

  template <typename T>
  const T* as() const {
    return static_cast<const T*>(data_);
  }

  bool valid() const { return data_ != nullptr; }
  std::size_t capacity() const { return pool_ ? pool_->blockSize() : 0; }

  void release() {
    if (pool_ && data_) {
      pool_->deallocate(data_);
    }
    pool_ = nullptr;
    data_ = nullptr;
  }

 private:
  MemoryPool* pool_{nullptr};
  void* data_{nullptr};
};

// ---------------------------------------------------------------------------
// 全局内存池注册表
//
//  按块大小共享内存池实例，避免各处重复创建；块大小会被向上对齐到 2 的幂。
// ---------------------------------------------------------------------------
class MemoryPoolRegistry {
 public:
  static MemoryPool& get(std::size_t block_size);

  /// 汇总所有已注册内存池的统计信息
  static std::vector<std::pair<std::size_t, MemoryPool::Stats>> allStats();

 private:
  static MemoryPoolRegistry& instance();
  MemoryPool& getImpl(std::size_t block_size);

  std::mutex mutex_;
  std::vector<std::pair<std::size_t, std::unique_ptr<MemoryPool>>> pools_;
};

}  // namespace adsim
