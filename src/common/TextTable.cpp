#include "adsim/common/TextTable.h"

#include <iomanip>
#include <sstream>

namespace adsim {

std::size_t displayWidth(const std::string& text) {
  std::size_t width = 0;

  for (std::size_t i = 0; i < text.size();) {
    const auto byte = static_cast<unsigned char>(text[i]);

    std::size_t length = 1;
    if ((byte & 0x80) == 0x00) length = 1;
    else if ((byte & 0xE0) == 0xC0) length = 2;
    else if ((byte & 0xF0) == 0xE0) length = 3;
    else if ((byte & 0xF8) == 0xF0) length = 4;

    // 三字节及以上多为 CJK 全角字符，按两列计
    width += (length >= 3) ? 2 : 1;
    i += length;
  }
  return width;
}

std::string padTo(const std::string& text, std::size_t width, bool left_align) {
  const std::size_t current = displayWidth(text);
  if (current >= width) return text;

  const std::string spaces(width - current, ' ');
  return left_align ? text + spaces : spaces + text;
}

std::string padTo(double value, int precision, std::size_t width, bool left_align) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(precision);
  oss << value;
  return padTo(oss.str(), width, left_align);
}

}  // namespace adsim
