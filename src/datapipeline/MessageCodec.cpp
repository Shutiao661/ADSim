#include "adsim/datapipeline/MessageCodec.h"

#include <cmath>
#include <cstring>

namespace adsim {
namespace ros {

namespace {

/// PointField 的 datatype 取值（见 sensor_msgs/PointField）
constexpr std::uint8_t kPointFieldFloat32 = 7;

/// PointCloud2 中单点的字段布局：x, y, z, intensity 各 4 字节
constexpr std::uint32_t kPointStep = 16;

void appendRaw(std::vector<std::uint8_t>& out, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  out.insert(out.end(), bytes, bytes + size);
}

template <typename T>
void appendPod(std::vector<std::uint8_t>& out, T value) {
  appendRaw(out, &value, sizeof(T));
}

}  // namespace

// ---------------------------------------------------------------------------
// MessageWriter
// ---------------------------------------------------------------------------

void MessageWriter::writeBytes(const void* data, std::size_t size) {
  appendRaw(buffer_, data, size);
}

void MessageWriter::writeU8(std::uint8_t value) { appendPod(buffer_, value); }
void MessageWriter::writeI8(std::int8_t value) { appendPod(buffer_, value); }
void MessageWriter::writeU16(std::uint16_t value) { appendPod(buffer_, value); }
void MessageWriter::writeI16(std::int16_t value) { appendPod(buffer_, value); }
void MessageWriter::writeU32(std::uint32_t value) { appendPod(buffer_, value); }
void MessageWriter::writeI32(std::int32_t value) { appendPod(buffer_, value); }
void MessageWriter::writeU64(std::uint64_t value) { appendPod(buffer_, value); }
void MessageWriter::writeF32(float value) { appendPod(buffer_, value); }
void MessageWriter::writeF64(double value) { appendPod(buffer_, value); }

void MessageWriter::writeString(const std::string& value) {
  writeU32(static_cast<std::uint32_t>(value.size()));
  appendRaw(buffer_, value.data(), value.size());
}

void MessageWriter::writeTime(Timestamp stamp) {
  if (stamp == kInvalidTimestamp) {
    writeU32(0);
    writeU32(0);
    return;
  }
  const auto secs = static_cast<std::uint32_t>(stamp / 1000000000LL);
  const auto nsecs = static_cast<std::uint32_t>(stamp % 1000000000LL);
  writeU32(secs);
  writeU32(nsecs);
}

void MessageWriter::writeDuration(double seconds) {
  const auto secs = static_cast<std::int32_t>(seconds);
  const auto nsecs = static_cast<std::int32_t>((seconds - secs) * 1e9);
  writeI32(secs);
  writeI32(nsecs);
}

// ---------------------------------------------------------------------------
// MessageReader
// ---------------------------------------------------------------------------

bool MessageReader::require(std::size_t n) {
  if (!ok_) return false;
  if (offset_ + n > size_) {
    ok_ = false;
    return false;
  }
  return true;
}

void MessageReader::skip(std::size_t n) {
  if (require(n)) offset_ += n;
}

std::uint8_t MessageReader::readU8() {
  if (!require(1)) return 0;
  return data_[offset_++];
}

std::int8_t MessageReader::readI8() {
  return static_cast<std::int8_t>(readU8());
}

std::uint16_t MessageReader::readU16() {
  if (!require(2)) return 0;
  std::uint16_t value = 0;
  std::memcpy(&value, data_ + offset_, sizeof(value));
  offset_ += 2;
  return value;
}

std::int16_t MessageReader::readI16() {
  return static_cast<std::int16_t>(readU16());
}

std::uint32_t MessageReader::readU32() {
  if (!require(4)) return 0;
  std::uint32_t value = 0;
  std::memcpy(&value, data_ + offset_, sizeof(value));
  offset_ += 4;
  return value;
}

std::int32_t MessageReader::readI32() {
  return static_cast<std::int32_t>(readU32());
}

std::uint64_t MessageReader::readU64() {
  if (!require(8)) return 0;
  std::uint64_t value = 0;
  std::memcpy(&value, data_ + offset_, sizeof(value));
  offset_ += 8;
  return value;
}

float MessageReader::readF32() {
  if (!require(4)) return 0.0f;
  float value = 0.0f;
  std::memcpy(&value, data_ + offset_, sizeof(value));
  offset_ += 4;
  return value;
}

double MessageReader::readF64() {
  if (!require(8)) return 0.0;
  double value = 0.0;
  std::memcpy(&value, data_ + offset_, sizeof(value));
  offset_ += 8;
  return value;
}

std::string MessageReader::readString() {
  const std::uint32_t length = readU32();
  if (!ok_) return {};
  if (!require(length)) return {};
  const std::string value(reinterpret_cast<const char*>(data_ + offset_), length);
  offset_ += length;
  return value;
}

Timestamp MessageReader::readTime() {
  const std::uint32_t secs = readU32();
  const std::uint32_t nsecs = readU32();
  if (!ok_) return kInvalidTimestamp;
  return static_cast<Timestamp>(secs) * 1000000000LL + static_cast<Timestamp>(nsecs);
}

double MessageReader::readDuration() {
  const std::int32_t secs = readI32();
  const std::int32_t nsecs = readI32();
  return static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
}

// ---------------------------------------------------------------------------
// std_msgs/Header
// ---------------------------------------------------------------------------

void writeStdHeader(MessageWriter& writer, const StdHeader& header) {
  writer.writeU32(header.seq);
  writer.writeTime(header.stamp);
  writer.writeString(header.frame_id);
}

StdHeader readStdHeader(MessageReader& reader) {
  StdHeader header;
  header.seq = reader.readU32();
  header.stamp = reader.readTime();
  header.frame_id = reader.readString();
  return header;
}

// ---------------------------------------------------------------------------
// sensor_msgs/NavSatFix
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> encodeNavSatFix(const GpsFrame& frame) {
  MessageWriter writer;
  writeStdHeader(writer, StdHeader{0, frame.stamp, "gps"});

  // NavSatStatus: int8 status + uint16 service
  // fix_type 0/1/2/3 映射为 STATUS_NO_FIX / FIX / GBAS_FIX
  writer.writeI8(frame.fix_type == 0 ? -1 : 0);
  writer.writeU16(1);  // SERVICE_GPS

  writer.writeF64(frame.latitude);
  writer.writeF64(frame.longitude);
  writer.writeF64(frame.altitude);

  // position_covariance[9]：由 hdop 估算水平/垂直方差
  const double variance = frame.hdop * frame.hdop;
  for (int i = 0; i < 9; ++i) {
    writer.writeF64((i == 0 || i == 4 || i == 8) ? variance : 0.0);
  }

  // COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
  writer.writeU8(frame.hdop < 99.0 ? 2 : 0);
  return writer.take();
}

GpsFrame decodeNavSatFix(const std::uint8_t* data, std::size_t size) {
  MessageReader reader(data, size);
  GpsFrame frame;

  const StdHeader header = readStdHeader(reader);
  frame.stamp = header.stamp;

  const std::int8_t status = reader.readI8();
  reader.readU16();  // service
  frame.fix_type = status < 0 ? 0 : 3;

  frame.latitude = reader.readF64();
  frame.longitude = reader.readF64();
  frame.altitude = reader.readF64();

  // 由协方差对角线反推 hdop
  const double var_x = reader.readF64();
  for (int i = 1; i < 9; ++i) reader.readF64();
  frame.hdop = var_x > 0.0 ? std::sqrt(var_x) : 99.9;

  if (!reader.ok()) {
    frame.stamp = kInvalidTimestamp;
  }
  return frame;
}

// ---------------------------------------------------------------------------
// nav_msgs/Odometry
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> encodeOdometry(const VehicleState& state) {
  MessageWriter writer;
  writeStdHeader(writer, StdHeader{0, state.stamp, "map"});
  writer.writeString("base_link");

  // ---- PoseWithCovariance ----
  writer.writeF64(state.x);
  writer.writeF64(state.y);
  writer.writeF64(0.0);  // 平面运动，z 恒为 0

  // 航向角转四元数（仅绕 z 轴旋转）
  const double half = state.theta * 0.5;
  writer.writeF64(0.0);            // qx
  writer.writeF64(0.0);            // qy
  writer.writeF64(std::sin(half));  // qz
  writer.writeF64(std::cos(half));  // qw

  for (int i = 0; i < 36; ++i) {
    writer.writeF64(i % 7 == 0 ? 0.01 : 0.0);
  }

  // ---- TwistWithCovariance ----
  writer.writeF64(state.speed * std::cos(state.theta));
  writer.writeF64(state.speed * std::sin(state.theta));
  writer.writeF64(0.0);
  writer.writeF64(0.0);
  writer.writeF64(0.0);
  writer.writeF64(state.yaw_rate);

  for (int i = 0; i < 36; ++i) {
    writer.writeF64(i % 7 == 0 ? 0.01 : 0.0);
  }

  return writer.take();
}

VehicleState decodeOdometry(const std::uint8_t* data, std::size_t size) {
  MessageReader reader(data, size);
  VehicleState state;

  const StdHeader header = readStdHeader(reader);
  state.stamp = header.stamp;

  reader.readString();  // child_frame_id

  state.x = reader.readF64();
  state.y = reader.readF64();
  reader.readF64();  // z
  reader.readF64();  // qx
  reader.readF64();  // qy
  const double qz = reader.readF64();
  const double qw = reader.readF64();
  state.theta = normalizeAngle(2.0 * std::atan2(qz, qw));

  for (int i = 0; i < 36; ++i) reader.readF64();

  const double vx = reader.readF64();
  const double vy = reader.readF64();
  reader.readF64();  // linear.z
  reader.readF64();  // angular.x
  reader.readF64();  // angular.y
  state.yaw_rate = reader.readF64();
  state.speed = std::sqrt(vx * vx + vy * vy);

  if (!reader.ok()) {
    state.stamp = kInvalidTimestamp;
  }
  return state;
}

// ---------------------------------------------------------------------------
// sensor_msgs/PointCloud2
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> encodePointCloud2(const LidarFrame& frame) {
  MessageWriter writer;
  writeStdHeader(writer, StdHeader{0, frame.stamp, frame.frame_id});

  const auto point_count = static_cast<std::uint32_t>(frame.points.size());

  writer.writeU32(1);            // height
  writer.writeU32(point_count);  // width

  // fields：x, y, z, intensity 四个 FLOAT32
  const char* names[4] = {"x", "y", "z", "intensity"};
  writer.writeU32(4);
  for (std::uint32_t i = 0; i < 4; ++i) {
    writer.writeString(names[i]);
    writer.writeU32(i * 4);                 // offset
    writer.writeU8(kPointFieldFloat32);     // datatype
    writer.writeU32(1);                     // count
  }

  writer.writeBool(false);  // is_bigendian
  writer.writeU32(kPointStep);
  writer.writeU32(kPointStep * point_count);  // row_step

  // data：紧密排列的点数据
  writer.writeU32(kPointStep * point_count);
  std::vector<std::uint8_t> raw(static_cast<std::size_t>(kPointStep) * point_count);
  std::size_t offset = 0;
  for (const LidarPoint& p : frame.points) {
    std::memcpy(raw.data() + offset, &p, sizeof(float) * 4);
    offset += kPointStep;
  }
  writer.writeBytes(raw.data(), raw.size());

  writer.writeBool(true);  // is_dense
  return writer.take();
}

LidarFrame decodePointCloud2(const std::uint8_t* data, std::size_t size) {
  MessageReader reader(data, size);
  LidarFrame frame;

  const StdHeader header = readStdHeader(reader);
  frame.stamp = header.stamp;
  frame.frame_id = header.frame_id;

  reader.readU32();  // height
  const std::uint32_t width = reader.readU32();

  // 解析字段布局，定位 x / y / z / intensity 的字节偏移
  struct FieldLayout {
    std::uint32_t offset{0};
    bool present{false};
  };
  FieldLayout fx, fy, fz, fi;

  const std::uint32_t field_count = reader.readU32();
  for (std::uint32_t i = 0; i < field_count && reader.ok(); ++i) {
    const std::string name = reader.readString();
    const std::uint32_t offset = reader.readU32();
    reader.readU8();   // datatype（本实现假定 4 字节浮点，实际布局以 offset 为准）
    reader.readU32();  // count

    if (name == "x") { fx = {offset, true}; }
    else if (name == "y") { fy = {offset, true}; }
    else if (name == "z") { fz = {offset, true}; }
    else if (name == "intensity") { fi = {offset, true}; }
  }

  reader.readBool();  // is_bigendian
  const std::uint32_t point_step = reader.readU32();
  reader.readU32();  // row_step

  const std::uint32_t data_len = reader.readU32();
  const std::uint8_t* point_data = nullptr;
  if (reader.ok() && reader.remaining() >= data_len) {
    // 直接引用原始缓冲区中的点数据区
    point_data = data + reader.offset();
    reader.skip(data_len);
  }
  reader.readBool();  // is_dense

  if (!reader.ok() || point_data == nullptr || point_step == 0) {
    frame.stamp = kInvalidTimestamp;
    return frame;
  }

  const std::uint32_t available = data_len / point_step;
  const std::uint32_t count = width < available ? width : available;
  frame.points.reserve(count);

  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint8_t* base = point_data + static_cast<std::size_t>(i) * point_step;
    LidarPoint p;
    if (fx.present) std::memcpy(&p.x, base + fx.offset, sizeof(float));
    if (fy.present) std::memcpy(&p.y, base + fy.offset, sizeof(float));
    if (fz.present) std::memcpy(&p.z, base + fz.offset, sizeof(float));
    if (fi.present) std::memcpy(&p.intensity, base + fi.offset, sizeof(float));
    frame.points.push_back(p);
  }

  return frame;
}

// ---------------------------------------------------------------------------
// 类型元信息
// ---------------------------------------------------------------------------

namespace {

const char* kNavSatFixDefinition = R"(# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

Header header

# satellite fix status information
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position.
float64[9] position_covariance

# If position_covariance is not specified or is not a constant, set
# position_covariance_type to COVARIANCE_TYPE_UNKNOWN.
uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
uint32 seq
time stamp
string frame_id

================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.
uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
)";

const char* kOdometryDefinition = R"(# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation.
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
)";

const char* kPointCloud2Definition = R"(# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc.

Header header

# The point cloud data may be organized 2d (image-like) or 1d (unordered).
uint32 height

# If the cloud is unordered, height is 1 and width is the length of the point cloud.
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)
bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
)";

}  // namespace

MessageTypeInfo lookupMessageType(const std::string& type) {
  if (type == "sensor_msgs/NavSatFix") {
    return {type, "2d3a8cd499b9b4a0249fb98fd05cfa48", kNavSatFixDefinition};
  }
  if (type == "nav_msgs/Odometry") {
    return {type, "cd5e73d190d741a2f92e81eda573aca7", kOdometryDefinition};
  }
  if (type == "sensor_msgs/PointCloud2") {
    return {type, "1158d486dd51d683ce2f1be655c3c181", kPointCloud2Definition};
  }
  return {};
}

bool isSupportedType(const std::string& type) {
  return !lookupMessageType(type).type.empty();
}

}  // namespace ros
}  // namespace adsim
