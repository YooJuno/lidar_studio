#pragma once

#include "lidar_viewer/live_io.hpp"

#include <array>
#include <string_view>

namespace lidar_viewer
{
inline constexpr std::size_t kMaxUdpPayload = 65507;
inline constexpr std::size_t kMaxLidarScanPoints = 1024 * 1024;

struct UdpRoute
{
  std::array<std::uint8_t, 4> source_ip{};
  std::uint16_t source_port{0}, destination_port{0};
  std::array<std::uint8_t, 4> destination_ip{255, 255, 255, 255};
  bool operator==(const UdpRoute & other) const
  {
    return source_ip == other.source_ip && source_port == other.source_port &&
      destination_port == other.destination_port;
  }
};

struct LidarDatagram
{
  std::vector<std::uint8_t> payload;
  UdpRoute route;
  std::int64_t received_ns{0};
};

struct LidarPacket
{
  DirectLidarModel model;
  std::size_t blocks{1};
  explicit LidarPacket(DirectLidarModel type, std::size_t block_count = 1)
  : model(type), blocks(block_count) {}
  std::array<std::uint16_t, 32> azimuth{};
  std::array<std::uint32_t, 32> scan_ids{};
  std::optional<std::uint32_t> sequence{};
  std::uint16_t motor_rpm{0};
  bool dual_return{false};
  std::uint16_t point_count{0};
  std::uint8_t data_type{0};
  std::uint64_t sensor_timestamp_ns{0};
  std::array<std::uint64_t, 32> column_timestamps{};
};

struct LidarClock
{
  // Keep an epoch anchor for display and derive intervals from the sensor clock.
  std::int64_t AtFrame(const LidarPacket & packet, std::size_t block, std::int64_t received_ns);
  std::int64_t last_received{0}, last_frame{0};
  std::uint64_t last_sensor{0};
};

struct LidarSequence
{
  std::optional<std::uint32_t> previous;
  std::uint64_t missing{0};
  bool Accept(std::optional<std::uint32_t> sequence);
};

struct LidarSweep
{
  std::optional<std::uint16_t> previous;
  std::optional<std::uint64_t> time_slice;
  bool Crossed(const LidarPacket & packet, std::size_t block);
};

struct BeamAngles
{
  std::array<float, 32> elevation{}, azimuth{};
};

bool IsHesaiXt(DirectLidarModel model);
class OusterCalibration;
class HesaiAtCalibration
{
public:
  static constexpr std::uint32_t kOutsideSector = 3;
  explicit HesaiAtCalibration(std::string_view bytes);
  std::uint32_t Sector(std::uint32_t encoder) const;
  void AppendBlock(FrameData & frame, const std::uint8_t * data, const LidarPacket & packet, std::size_t block) const;

private:
  static constexpr std::size_t kBeams = 128, kSamples = 180;
  std::size_t sectors_{0};
  float adjustment_unit_{0};
  std::array<std::uint32_t, kOutsideSector> start_{}, width_{};
  std::array<float, kBeams> azimuth_{}, elevation_{};
  std::array<std::int8_t, kBeams * kSamples> azimuth_adjustment_{}, elevation_adjustment_{};
};
bool LooksLikeHesaiAtCalibration(std::string_view bytes);
std::optional<LidarPacket> InspectLidarPacket(const std::uint8_t * data, std::size_t size,
  const OusterCalibration * ouster = nullptr, const HesaiAtCalibration * at = nullptr);
std::string DescribeLidarPacket(const std::uint8_t * data, std::size_t size);
std::string ReadLidarCalibration(const fs::path & path);
const char * LidarFrameId(DirectLidarModel model);

class LidarPacketDecoder
{
public:
  LidarPacketDecoder(DirectLidarModel model, std::string_view calibration = {}, const OusterCalibration * ouster = nullptr,
    const HesaiAtCalibration * at = nullptr);
  void AppendBlock(FrameData & frame, const std::uint8_t * data, const LidarPacket & packet, std::size_t block);

private:
  void UpdateHesaiAzimuth(DirectLidarModel model, int rpm);
  BeamAngles angles_;
  std::array<float, 32> cos_elevation_{}, sin_elevation_{}, cos_azimuth_{}, sin_azimuth_{};
  int previous_rpm_{-1};
  const OusterCalibration * ouster_;
  const HesaiAtCalibration * at_;
};
}  // namespace lidar_viewer
