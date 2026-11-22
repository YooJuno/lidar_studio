#pragma once

#include "lidar_packets.hpp"

namespace lidar_viewer
{
class OusterCalibration
{
public:
  explicit OusterCalibration(std::string_view json);
  std::optional<LidarPacket> Inspect(const std::uint8_t * data, std::size_t size) const;
  void AppendColumn(FrameData & frame, const std::uint8_t * data, const LidarPacket & packet, std::size_t column) const;
  std::uint16_t Port() const {return port_;}
  std::uint16_t ImuPort() const {return imu_port_;}

private:
  const std::uint8_t * Column(const std::uint8_t * packet, std::size_t column) const;
  std::uint16_t port_{7502}, imu_port_{7503};
  std::size_t width_{}, height_{}, columns_per_packet_{}, column_bytes_{}, channel_bytes_{};
  std::size_t reflectivity_offset_{}, packet_size_{};
  std::uint32_t range_mask_{}, scale_mm_{1};
  bool legacy_{false}, fusa_{false}, dual_{false};
  std::optional<std::uint64_t> serial_;
  std::array<float, 16> transform_{};
  float origin_x_{}, origin_z_{}, beam_distance_{};
  std::vector<float> cos_elevation_, sin_elevation_, cos_azimuth_, sin_azimuth_;
  std::vector<float> cos_encoder_, sin_encoder_;
};

bool LooksLikeOusterMetadata(std::string_view calibration);
}  // namespace lidar_viewer
