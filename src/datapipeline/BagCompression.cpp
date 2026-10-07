#include "adsim/datapipeline/BagCompression.h"

// 所有 #include 必须在文件作用域完成，不能写进 namespace 内部
#if defined(ADSIM_HAS_LZ4)
#include <lz4frame.h>
#endif

#if defined(ADSIM_HAS_BZ2)
#include <bzlib.h>
#endif

#include <limits>
#include <sstream>

namespace adsim {
namespace bag {

namespace {

#if defined(ADSIM_HAS_LZ4)

/// 解压 LZ4 Frame 格式的数据。
///
/// 用标准的帧解码器而不是手写分帧：roslz4 写出的确实是标准 LZ4 Frame
/// （magic 0x184D2204），它自己手写解析只是历史原因。
/// 帧解码器会自动处理块独立性标志、流校验和等细节。
bool decompressLz4(const std::uint8_t* input, std::size_t input_size,
                   std::size_t expected_size, std::vector<std::uint8_t>& output,
                   std::string& error) {
  LZ4F_dctx* context = nullptr;
  const LZ4F_errorCode_t created =
      LZ4F_createDecompressionContext(&context, LZ4F_VERSION);
  if (LZ4F_isError(created)) {
    error = std::string("LZ4 解码上下文创建失败: ") + LZ4F_getErrorName(created);
    return false;
  }

  output.assign(expected_size, 0);

  std::size_t source_offset = 0;
  std::size_t dest_offset = 0;

  // 逐段喂入：帧解码器可能在一次调用中只消费部分输入，
  // 因此必须循环直到输入耗尽或帧结束
  for (;;) {
    if (source_offset >= input_size) break;

    std::size_t dest_available = output.size() - dest_offset;
    std::size_t source_available = input_size - source_offset;

    const std::size_t hint =
        LZ4F_decompress(context, output.data() + dest_offset, &dest_available,
                        input + source_offset, &source_available, nullptr);

    if (LZ4F_isError(hint)) {
      error = std::string("LZ4 解压失败: ") + LZ4F_getErrorName(hint);
      LZ4F_freeDecompressionContext(context);
      return false;
    }

    source_offset += source_available;
    dest_offset += dest_available;

    // hint == 0 表示一个完整的帧已经解完
    if (hint == 0) break;

    // 既不消费输入也不产出输出 → 输入数据不完整，继续循环只会死转
    if (source_available == 0 && dest_available == 0) {
      error = "LZ4 数据不完整：解码器无法继续推进";
      LZ4F_freeDecompressionContext(context);
      return false;
    }
  }

  LZ4F_freeDecompressionContext(context);

  if (dest_offset != expected_size) {
    std::ostringstream oss;
    oss << "LZ4 解压长度不符：声明 " << expected_size << " 字节，实际得到 "
        << dest_offset << " 字节";
    error = oss.str();
    return false;
  }
  return true;
}

#endif  // ADSIM_HAS_LZ4

#if defined(ADSIM_HAS_BZ2)

bool decompressBz2(const std::uint8_t* input, std::size_t input_size,
                   std::size_t expected_size, std::vector<std::uint8_t>& output,
                   std::string& error) {
  // BZ2 的接口用 unsigned int 表示长度，超过 4GB 会溢出。
  // rosbag 的 Chunk 远小于这个量级，遇到时明确报错而不是静默截断。
  constexpr std::size_t kMaxBz2Size = std::numeric_limits<unsigned int>::max();
  if (expected_size > kMaxBz2Size || input_size > kMaxBz2Size) {
    error = "BZ2 Chunk 超过 4GB，超出 bzip2 接口的长度表示能力";
    return false;
  }

  output.assign(expected_size, 0);

  unsigned int dest_len = static_cast<unsigned int>(expected_size);
  // BZ2 的接口不接受 const 源指针，但它只读该缓冲区
  const int result = BZ2_bzBuffToBuffDecompress(
      reinterpret_cast<char*>(output.data()), &dest_len,
      const_cast<char*>(reinterpret_cast<const char*>(input)),
      static_cast<unsigned int>(input_size), 0, 0);

  if (result != BZ_OK) {
    std::ostringstream oss;
    oss << "BZ2 解压失败（错误码 " << result << "）";
    switch (result) {
      case BZ_MEM_ERROR: oss << ": 内存不足"; break;
      case BZ_OUTBUFF_FULL: oss << ": 输出缓冲区不足"; break;
      case BZ_DATA_ERROR: oss << ": 数据完整性校验失败"; break;
      case BZ_DATA_ERROR_MAGIC: oss << ": 数据缺少 bzip2 魔数"; break;
      case BZ_UNEXPECTED_EOF: oss << ": 数据意外结束"; break;
      case BZ_PARAM_ERROR: oss << ": 参数非法"; break;
      case BZ_CONFIG_ERROR: oss << ": 库配置错误"; break;
      default: break;
    }
    error = oss.str();
    return false;
  }

  if (dest_len != expected_size) {
    std::ostringstream oss;
    oss << "BZ2 解压长度不符：声明 " << expected_size << " 字节，实际得到 " << dest_len
        << " 字节";
    error = oss.str();
    return false;
  }
  return true;
}

#endif  // ADSIM_HAS_BZ2

#if defined(ADSIM_HAS_LZ4)
/// 构造与 roslz4 一致的 LZ4 Frame 参数。
///
/// roslz4 在解压时会强制校验这几个标志位（见 roslz4/src/lz4s.c 的
/// processHeader），不满足就直接判为数据错误。因此写出的帧必须对齐：
///   version=1（标准）、块独立、无块校验和、有流校验和、不写 content size
LZ4F_preferences_t lz4Preferences() {
  LZ4F_preferences_t prefs{};
  prefs.frameInfo.contentSize = 0;  // 不写 content size（roslz4 不支持）
  prefs.frameInfo.blockMode = LZ4F_blockIndependent;
  prefs.frameInfo.blockChecksumFlag = LZ4F_noBlockChecksum;
  prefs.frameInfo.contentChecksumFlag = LZ4F_contentChecksumEnabled;
  // roslz4 默认 block_size_id = 6，对应 1MB；两边的索引定义完全一致
  // （roslz4: 1 << (8 + 2*id)，LZ4F: max64KB=4 / max256KB=5 / max1MB=6 / max4MB=7）
  prefs.frameInfo.blockSizeID = LZ4F_max1MB;
  prefs.compressionLevel = 0;  // 0 = 使用库的默认级别
  return prefs;
}
#endif

}  // namespace

bool isCompressionSupported(Compression compression) {
  switch (compression) {
    case Compression::kNone:
      return true;
#if defined(ADSIM_HAS_BZ2)
    case Compression::kBz2:
      return true;
#endif
#if defined(ADSIM_HAS_LZ4)
    case Compression::kLz4:
      return true;
#endif
    default:
      return false;
  }
}

std::string supportedCompressions() {
  std::string result = "none";
#if defined(ADSIM_HAS_BZ2)
  result += ", bz2";
#endif
#if defined(ADSIM_HAS_LZ4)
  result += ", lz4";
#endif
  return result;
}

bool decompressChunk(Compression compression,
                     const std::uint8_t* input,
                     std::size_t input_size,
                     std::size_t uncompressed_size,
                     std::vector<std::uint8_t>& output,
                     std::string& error) {
  error.clear();

  if (uncompressed_size == 0) {
    output.clear();
    return true;
  }
  if (input == nullptr || input_size == 0) {
    error = "压缩数据为空";
    return false;
  }

  switch (compression) {
    case Compression::kNone:
      if (input_size != uncompressed_size) {
        std::ostringstream oss;
        oss << "未压缩 Chunk 的长度与声明不符：声明 " << uncompressed_size
            << " 字节，实际 " << input_size << " 字节";
        error = oss.str();
        return false;
      }
      output.assign(input, input + input_size);
      return true;

    case Compression::kBz2:
#if defined(ADSIM_HAS_BZ2)
      return decompressBz2(input, input_size, uncompressed_size, output, error);
#else
      error = "本构建未包含 bzip2 支持（构建时未找到 libbz2）";
      return false;
#endif

    case Compression::kLz4:
#if defined(ADSIM_HAS_LZ4)
      return decompressLz4(input, input_size, uncompressed_size, output, error);
#else
      error = "本构建未包含 LZ4 支持（构建时未找到 liblz4）";
      return false;
#endif

    case Compression::kUnknown:
    default:
      error = "未知的 Chunk 压缩方式";
      return false;
  }
}

bool compressChunk(Compression compression,
                   const std::uint8_t* input,
                   std::size_t input_size,
                   std::vector<std::uint8_t>& output,
                   std::string& error) {
  error.clear();

  // 指针合法性检查必须在任何解引用之前
  if (input == nullptr && input_size > 0) {
    error = "输入指针为空";
    return false;
  }

  if (compression == Compression::kNone) {
    output.clear();
    if (input_size > 0) {
      output.assign(input, input + input_size);
    }
    return true;
  }

  // 空输入时给一个合法的非空指针：底层库（bz2/lz4）都会对 NULL 源指针
  // 报参数错误，即便长度为零。用一个静态占位字节绕开，不改变语义。
  //
  // 这两个变量只在启用了压缩库的分支里被用到，未启用时会被标为未使用，
  // 因此显式标注 maybe_unused 而不是用条件编译把代码切碎。
  static const std::uint8_t kEmptyPlaceholder = 0;
  [[maybe_unused]] const std::uint8_t* source =
      (input != nullptr) ? input : &kEmptyPlaceholder;
  [[maybe_unused]] constexpr std::size_t kMaxCompressSize =
      std::numeric_limits<unsigned int>::max();

  switch (compression) {
    case Compression::kBz2: {
#if defined(ADSIM_HAS_BZ2)
      if (input_size > kMaxCompressSize) {
        error = "数据超过 4GB，超出 bzip2 接口的长度表示能力";
        return false;
      }
      // bzip2 官方给出的输出上界：n + n/100 + 600
      output.resize(input_size + input_size / 100 + 600);

      unsigned int dest_len = static_cast<unsigned int>(output.size());
      const int result = BZ2_bzBuffToBuffCompress(
          reinterpret_cast<char*>(output.data()), &dest_len,
          const_cast<char*>(reinterpret_cast<const char*>(source)),
          static_cast<unsigned int>(input_size), 9 /* blockSize100k */, 0, 0);
      if (result != BZ_OK) {
        std::ostringstream oss;
        oss << "BZ2 压缩失败（错误码 " << result << "）";
        if (result == BZ_MEM_ERROR) oss << ": 内存不足";
        else if (result == BZ_PARAM_ERROR) oss << ": 参数非法";
        error = oss.str();
        return false;
      }
      output.resize(dest_len);
      return true;
#else
      error = "本构建未包含 bzip2 支持";
      return false;
#endif
    }

    case Compression::kLz4: {
#if defined(ADSIM_HAS_LZ4)
      const LZ4F_preferences_t prefs = lz4Preferences();
      const std::size_t bound = LZ4F_compressFrameBound(input_size, &prefs);
      if (LZ4F_isError(bound)) {
        error = std::string("LZ4 压缩上界计算失败: ") + LZ4F_getErrorName(bound);
        return false;
      }

      output.resize(bound);
      const std::size_t written =
          LZ4F_compressFrame(output.data(), bound, source, input_size, &prefs);
      if (LZ4F_isError(written)) {
        error = std::string("LZ4 压缩失败: ") + LZ4F_getErrorName(written);
        return false;
      }
      output.resize(written);
      return true;
#else
      error = "本构建未包含 LZ4 支持";
      return false;
#endif
    }

    default:
      error = "未知的压缩方式";
      return false;
  }
}

}  // namespace bag
}  // namespace adsim
