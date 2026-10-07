// =============================================================================
//  TextTable.h — 终端表格对齐工具
//
//  std::setw 按**字节数**计宽，而中文在 UTF-8 下一个字占 3 字节却只占 2 个
//  显示列。中英混排时直接用 setw 会让表头与数据行错位、列与列挤在一起。
//
//  命令行报告里经常要输出这种表（性能剖析、后端对照），因此把显示宽度计算
//  抽成公共工具，避免每处各写一份、各错一遍。
// =============================================================================
#pragma once

#include <cstddef>
#include <string>

namespace adsim {

/// 按显示宽度计算字符串宽度：三字节及以上的 UTF-8 序列按 2 列计，其余按 1 列。
///
/// 这个近似对 CJK 场景足够准确（绝大多数中日韩字符是三字节、占两列），
/// 且不需要引入 locale 或宽字符转换。
std::size_t displayWidth(const std::string& text);

/// 把字符串补齐到指定显示宽度。
/// @param left_align true 表示左对齐（右侧补空格），false 表示右对齐
/// @return 补齐后的字符串；原串已超宽时原样返回
std::string padTo(const std::string& text, std::size_t width, bool left_align);

/// 便捷重载：把数值转成字符串后按显示宽度补齐
std::string padTo(double value, int precision, std::size_t width, bool left_align);

}  // namespace adsim
