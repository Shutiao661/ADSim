// =============================================================================
//  BagCompression.h — ROS Bag Chunk 解压
//
//  rosbag 支持三种 Chunk 压缩方式（`compression` 字段）：
//    none —— 原样存储
//    bz2  —— bzip2 压缩
//    lz4  —— LZ4 **Frame** 格式压缩
//
//  实车数据出于存储考虑经常是压缩的，因此不支持解压的解析器实际可用性有限。
//
//  格式细节已对照 ROS 官方实现（ros_comm/utilities/roslz4/src/lz4s.c）
//  核实，不是凭 API 猜的：
//    * LZ4 用的是标准 Frame 格式（magic 0x184D2204），roslz4 强制
//      version=1、block_independence=1、block_checksum=0、stream_size=0、
//      stream_checksum=1。这些约束标准帧解码器都能处理，因此直接用
//      LZ4F_decompress 即可，不必手写分帧。
//    * bz2 用的是 BZ2_bzBuffToBuffDecompress，small=0、verbosity=0。
//
//  bz2/lz4 是**可选依赖**：系统上有对应库时自动启用，没有则编译为不支持，
//  运行时给出明确提示而不是崩溃。这样既保住了核心库"零依赖可构建"的承诺，
//  又在有能力的环境里提供完整功能。
// =============================================================================
#pragma once

#include "adsim/datapipeline/RosBagFormat.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace adsim {
namespace bag {

/// 当前构建是否支持该压缩方式
bool isCompressionSupported(Compression compression);

/// 本构建支持的压缩方式列表，用于诊断信息
std::string supportedCompressions();

/// 解压一个 Chunk 的数据区。
///
/// @param compression       压缩方式
/// @param input             压缩数据
/// @param input_size        压缩数据长度
/// @param uncompressed_size 解压后应有的长度（来自 Chunk 头的 size 字段）
/// @param output            输出缓冲区，会被 resize 到 uncompressed_size
/// @param error             失败时写入可读原因
/// @return 是否成功。
///
/// **解压后的长度与声明不符一律视为失败**：这是发现文件损坏或格式理解
/// 有误的重要信号，静默接受会在后续解析中产生难以定位的错误。
bool decompressChunk(Compression compression,
                     const std::uint8_t* input,
                     std::size_t input_size,
                     std::size_t uncompressed_size,
                     std::vector<std::uint8_t>& output,
                     std::string& error);

/// 压缩一段数据。
///
/// 用于写入压缩 bag，也便于做「压缩 → 解压」的往返自检。
/// 输出的格式与 ROS 官方实现兼容：
///   * bz2 —— BZ2_bzBuffToBuffCompress，blockSize100k=9
///   * lz4 —— 标准 LZ4 Frame，且满足 roslz4 的约束（块独立、无块校验和、
///            有流校验和、不写 content size）
bool compressChunk(Compression compression,
                   const std::uint8_t* input,
                   std::size_t input_size,
                   std::vector<std::uint8_t>& output,
                   std::string& error);

}  // namespace bag
}  // namespace adsim
