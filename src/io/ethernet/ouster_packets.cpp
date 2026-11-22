#include "ouster_packets.hpp"

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/property_tree/json_parser.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace lidar_viewer
{
namespace
{
using boost::property_tree::ptree;
constexpr float kRadians = kPi / 180.f;

std::uint32_t Little(const std::uint8_t * data, unsigned count)
{
  std::uint32_t value = 0;
  for (unsigned i = 0; i < count; ++i) value |= static_cast<std::uint32_t>(data[i]) << (8 * i);
  return value;
}

std::vector<float> Numbers(const ptree & data, const char * field, std::size_t count)
{
  const auto & values = data.get_child(field);
  if (values.size() != count) throw std::runtime_error(std::string("Invalid Ouster metadata field: ") + field);
  std::vector<float> result;
  result.reserve(count);
  for (const auto & entry : values) {
    const auto value = entry.second.get_value<float>();
    if (!entry.first.empty() || !std::isfinite(value) || std::abs(value) > 100000.f) {
      throw std::runtime_error(std::string("Invalid Ouster metadata value: ") + field);
    }
    result.push_back(value);
  }
  return result;
}
}  // namespace

bool LooksLikeOusterMetadata(std::string_view calibration)
{
  const auto start = calibration.find_first_not_of(" \t\r\n");
  return start != std::string_view::npos && calibration[start] == '{';
}

OusterCalibration::OusterCalibration(std::string_view json)
{
  ptree root;
  try {
    std::istringstream stream{std::string(json)};
    boost::property_tree::read_json(stream, root);
    const auto lidar_data = root.get_child_optional("lidar_data_format");
    const auto legacy_data = root.get_child_optional("data_format");
    const auto & format = lidar_data ? *lidar_data : legacy_data ? *legacy_data : root;
    const auto beam_intrinsics = root.get_child_optional("beam_intrinsics");
    const auto lidar_intrinsics = root.get_child_optional("lidar_intrinsics");
    const auto & beam = beam_intrinsics ? *beam_intrinsics : root;
    const auto & lidar = lidar_intrinsics ? *lidar_intrinsics : root;
    width_ = format.get<std::size_t>("columns_per_frame");
    height_ = format.get<std::size_t>("pixels_per_column");
    columns_per_packet_ = format.get<std::size_t>("columns_per_packet");
    if (width_ < 1 || width_ > 4096 || height_ < 1 || height_ > 128 ||
        columns_per_packet_ < 1 || columns_per_packet_ > LidarPacket(DirectLidarModel::OusterOs).azimuth.size()) {
      throw std::runtime_error("Ouster scan dimensions outside the supported range");
    }
    const auto port = root.get<int>("config_params.udp_port_lidar", root.get<int>("udp_port_lidar", 7502));
    if (port > 0 && port <= 65535) port_ = static_cast<std::uint16_t>(port);
    const auto imu = root.get<int>("config_params.udp_port_imu", root.get<int>("udp_port_imu", 7503));
    if (imu >= 0 && imu <= 65535) imu_port_ = static_cast<std::uint16_t>(imu);
    auto serial = root.get_optional<std::uint64_t>("sensor_info.prod_sn");
    if (!serial) serial = root.get_optional<std::uint64_t>("prod_sn");
    if (serial && *serial) serial_ = *serial;
    const auto profile = format.get<std::string>("udp_profile_lidar", "LEGACY");
    legacy_ = profile == "LEGACY";
    fusa_ = profile == "FUSA_RNG15_RFL8_NIR8_DUAL" ||
      format.get<std::string>("header_type", "") == "FUSA";
    if (legacy_) {
      channel_bytes_ = 12; range_mask_ = 0xfffff; reflectivity_offset_ = 4;
    } else if (profile == "RNG19_RFL8_SIG16_NIR16") {
      channel_bytes_ = 12; range_mask_ = 0x7ffff; reflectivity_offset_ = 4;
    } else if (profile == "RNG19_RFL8_SIG16_NIR16_DUAL") {
      channel_bytes_ = 16; range_mask_ = 0x7ffff; reflectivity_offset_ = 3; dual_ = true;
    } else if (profile == "RNG15_RFL8_NIR8") {
      channel_bytes_ = 4; range_mask_ = 0x7fff; reflectivity_offset_ = 2; scale_mm_ = 8;
    } else if (profile == "FUSA_RNG15_RFL8_NIR8_DUAL" || profile == "RNG15_RFL8_NIR8_DUAL") {
      channel_bytes_ = 8; range_mask_ = 0x7fff; reflectivity_offset_ = 2; scale_mm_ = 8; dual_ = true;
    } else {
      throw std::runtime_error("Unsupported Ouster UDP profile: " + profile);
    }
    column_bytes_ = (legacy_ ? 20 : 12) + channel_bytes_ * height_;
    packet_size_ = columns_per_packet_ * column_bytes_ + (legacy_ ? 0 : 64);
    if (packet_size_ > kMaxUdpPayload) throw std::runtime_error("Ouster packet size exceeds IPv4 UDP limits");
    const auto matrix = Numbers(lidar, "lidar_to_sensor_transform", 16);
    std::copy(matrix.begin(), matrix.end(), transform_.begin());
    if (beam.get_child_optional("beam_to_lidar_transform")) {
      const auto origin = Numbers(beam, "beam_to_lidar_transform", 16);
      origin_x_ = origin[3]; origin_z_ = origin[11];
    } else {
      origin_x_ = beam.get<float>("lidar_origin_to_beam_origin_mm");
      if (!std::isfinite(origin_x_) || std::abs(origin_x_) > 1000.f)
        throw std::runtime_error("Invalid Ouster beam origin");
    }
    beam_distance_ = std::hypot(origin_x_, origin_z_);
    const auto elevation = Numbers(beam, "beam_altitude_angles", height_);
    const auto azimuth = Numbers(beam, "beam_azimuth_angles", height_);
    for (std::size_t i = 0; i < height_; ++i) {
      cos_elevation_.push_back(std::cos(elevation[i] * kRadians));
      sin_elevation_.push_back(std::sin(elevation[i] * kRadians));
      cos_azimuth_.push_back(std::cos(azimuth[i] * kRadians));
      sin_azimuth_.push_back(-std::sin(azimuth[i] * kRadians));
    }
    for (std::size_t i = 0; i < width_; ++i) {
      const float encoder = -2 * kPi * static_cast<float>(i) / static_cast<float>(width_);
      cos_encoder_.push_back(std::cos(encoder));
      sin_encoder_.push_back(std::sin(encoder));
    }
  } catch (const boost::property_tree::ptree_error & error) {
    throw std::runtime_error(std::string("Ouster metadata is incomplete: ") + error.what());
  }
}

const std::uint8_t * OusterCalibration::Column(const std::uint8_t * packet, std::size_t column) const
{
  return packet + (legacy_ ? 0 : 32) + column * column_bytes_;
}

std::optional<LidarPacket> OusterCalibration::Inspect(const std::uint8_t * data, std::size_t size) const
{
  if (size != packet_size_) return {};
  if (!legacy_) {
    if (Little(data, fusa_ ? 1 : 2) != 1) return {};
    const auto * serial = data + (fusa_ ? 11 : 7);
    const auto value = Little(serial, 4) | (static_cast<std::uint64_t>(serial[4]) << 32);
    if (serial_ && value != *serial_) return {};
  }
  LidarPacket packet(DirectLidarModel::OusterOs, columns_per_packet_);
  const auto frame = fusa_ ? Little(data + 4, 4) : Little(data + 2, 2);
  for (std::size_t column = 0; column < columns_per_packet_; ++column) {
    const auto * header = Column(data, column);
    const auto measurement = Little(header + 8, 2);
    if (measurement >= width_) return {};
    packet.azimuth[column] = static_cast<std::uint16_t>(measurement);
    packet.scan_ids[column] = legacy_ ? Little(header + 10, 2) : frame;
    packet.column_timestamps[column] = Little(header, 4) | (static_cast<std::uint64_t>(Little(header + 4, 4)) << 32);
  }
  return packet;
}

void OusterCalibration::AppendColumn(FrameData & frame, const std::uint8_t * data,
  const LidarPacket & packet, std::size_t column) const
{
  const auto * data_column = Column(data, column);
  const bool valid = legacy_ ? (Little(data_column + column_bytes_ - 4, 4) & 1) != 0 :
    (data_column[10] & 1) != 0;
  if (!valid) return;
  const auto measurement = packet.azimuth[column];
  const auto cos_encoder = cos_encoder_[measurement], sin_encoder = sin_encoder_[measurement];
  const auto * pixel = data_column + (legacy_ ? 16 : 12);
  for (std::size_t row = 0; row < height_; ++row, pixel += channel_bytes_) {
    const float dx = (cos_encoder * cos_azimuth_[row] - sin_encoder * sin_azimuth_[row]) * cos_elevation_[row];
    const float dy = (sin_encoder * cos_azimuth_[row] + cos_encoder * sin_azimuth_[row]) * cos_elevation_[row];
    const auto append = [&](const std::uint8_t * value) {
      const auto range = (Little(value, range_mask_ == 0x7fff ? 2 : 3) & range_mask_) * scale_mm_;
      if (range == 0) return;
      const float r = static_cast<float>(range) - beam_distance_;
      const float x = r*dx + origin_x_*cos_encoder, y = r*dy + origin_x_*sin_encoder, z = r*sin_elevation_[row] + origin_z_;
      frame.points.push_back({
        (transform_[0]*x+transform_[1]*y+transform_[2]*z+transform_[3])*.001f,
        (transform_[4]*x+transform_[5]*y+transform_[6]*z+transform_[7])*.001f,
        (transform_[8]*x+transform_[9]*y+transform_[10]*z+transform_[11])*.001f,
        static_cast<float>(value[reflectivity_offset_])});
    };
    append(pixel);
    if (dual_) append(pixel + 4);
  }
}
}  // namespace lidar_viewer
