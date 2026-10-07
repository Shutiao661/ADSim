#include "adsim/datapipeline/RosBagFormat.h"

#include <cstring>
#include <stdexcept>

namespace adsim {
namespace bag {

namespace {

/// 从字节序列小端读取定长整数
template <typename T>
T readLittleEndian(const std::uint8_t* data) {
  T value = 0;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

void appendLittleEndian(std::vector<std::uint8_t>& out, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  out.insert(out.end(), bytes, bytes + size);
}

template <typename T>
void appendValue(std::vector<std::uint8_t>& out, T value) {
  appendLittleEndian(out, &value, sizeof(T));
}

}  // namespace

const char kMagic[13] = {'#', 'R', 'O', 'S', 'B', 'A', 'G', ' ', 'V', '2', '.', '0', '\n'};

Compression compressionFromString(const std::string& name) {
  if (name == "none" || name.empty()) return Compression::kNone;
  if (name == "bz2") return Compression::kBz2;
  if (name == "lz4") return Compression::kLz4;
  return Compression::kUnknown;
}

const char* toString(Compression compression) {
  switch (compression) {
    case Compression::kNone: return "none";
    case Compression::kBz2: return "bz2";
    case Compression::kLz4: return "lz4";
    case Compression::kUnknown: return "unknown";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Header
// ---------------------------------------------------------------------------

void Header::parse(const std::uint8_t* data, std::size_t size) {
  fields_.clear();

  std::size_t offset = 0;
  while (offset < size) {
    if (offset + 4 > size) {
      throw std::runtime_error("bag 头部损坏: 字段长度字段越界");
    }
    const std::uint32_t field_len = readLittleEndian<std::uint32_t>(data + offset);
    offset += 4;

    if (field_len == 0 || offset + field_len > size) {
      throw std::runtime_error("bag 头部损坏: 字段长度非法");
    }

    const std::uint8_t name_len = data[offset];
    if (static_cast<std::size_t>(name_len) + 2 > field_len) {
      throw std::runtime_error("bag 头部损坏: 字段名长度非法");
    }

    const char* name_begin = reinterpret_cast<const char*>(data + offset + 1);
    const std::string name(name_begin, name_len);

    // name 与 value 之间以 '=' 分隔
    const std::size_t value_offset = offset + 1 + name_len + 1;
    const std::size_t value_len = field_len - 1 - name_len - 1;
    const std::string value(reinterpret_cast<const char*>(data + value_offset), value_len);

    fields_.emplace_back(name, value);
    offset += field_len;
  }
}

const std::string* Header::find(const std::string& key) const {
  for (const auto& field : fields_) {
    if (field.first == key) return &field.second;
  }
  return nullptr;
}

bool Header::has(const std::string& key) const { return find(key) != nullptr; }

std::uint8_t Header::getU8(const std::string& key, std::uint8_t default_value) const {
  const std::string* v = find(key);
  if (v == nullptr) return default_value;
  return static_cast<std::uint8_t>((*v)[0]);
}

std::uint32_t Header::getU32(const std::string& key, std::uint32_t default_value) const {
  const std::string* v = find(key);
  if (v == nullptr || v->size() < sizeof(std::uint32_t)) return default_value;
  return readLittleEndian<std::uint32_t>(reinterpret_cast<const std::uint8_t*>(v->data()));
}

std::uint64_t Header::getU64(const std::string& key, std::uint64_t default_value) const {
  const std::string* v = find(key);
  if (v == nullptr || v->size() < sizeof(std::uint64_t)) return default_value;
  return readLittleEndian<std::uint64_t>(reinterpret_cast<const std::uint8_t*>(v->data()));
}

std::string Header::getString(const std::string& key, const std::string& default_value) const {
  const std::string* v = find(key);
  return v == nullptr ? default_value : *v;
}

// ---------------------------------------------------------------------------
// HeaderWriter
// ---------------------------------------------------------------------------

void HeaderWriter::addBytes(const std::string& key, const void* data, std::size_t size) {
  if (key.size() > 255) {
    throw std::runtime_error("bag 头部字段名过长: " + key);
  }

  // field_len 覆盖 "name_len + name + '=' + value" 全部字节
  const std::uint32_t field_len =
      static_cast<std::uint32_t>(1 + key.size() + 1 + size);
  appendValue<std::uint32_t>(buffer_, field_len);

  const auto name_len = static_cast<std::uint8_t>(key.size());
  appendValue<std::uint8_t>(buffer_, name_len);
  appendLittleEndian(buffer_, key.data(), key.size());

  const std::uint8_t separator = '=';
  appendValue<std::uint8_t>(buffer_, separator);

  if (size > 0 && data != nullptr) {
    appendLittleEndian(buffer_, data, size);
  }
}

void HeaderWriter::addU8(const std::string& key, std::uint8_t value) {
  addBytes(key, &value, sizeof(value));
}

void HeaderWriter::addU32(const std::string& key, std::uint32_t value) {
  addBytes(key, &value, sizeof(value));
}

void HeaderWriter::addU64(const std::string& key, std::uint64_t value) {
  addBytes(key, &value, sizeof(value));
}

void HeaderWriter::addString(const std::string& key, const std::string& value) {
  addBytes(key, value.data(), value.size());
}

// ---------------------------------------------------------------------------
// MessageView
// ---------------------------------------------------------------------------

namespace {
const std::string kEmptyString;
}  // namespace

const std::string& MessageView::topic() const {
  return connection == nullptr ? kEmptyString : connection->topic;
}

const std::string& MessageView::type() const {
  return connection == nullptr ? kEmptyString : connection->type;
}

}  // namespace bag
}  // namespace adsim
