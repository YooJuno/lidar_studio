#pragma once

#include "lidar_viewer/data_io.hpp"
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace lidar_viewer
{
struct LiveOptions
{
  std::string points_topic{"/b2/points"};
  std::string imu_topic{"/vn100/imu/data_ned"};
};
enum class DirectLidarModel {
  Auto, HesaiXt32, HesaiXt16, HesaiXt32M2X, HesaiAt128, VelodyneVlp16, VelodynePuckHiRes, VelodyneHdl32e,
  VelodyneVlp32c, LivoxMid360, OusterOs,
  RobosenseM1, RobosenseM2, RobosenseM3,
};
inline constexpr std::uint16_t kDefaultLidarUdpPort = 2368;
const char * DirectLidarModelName(DirectLidarModel model);
std::uint16_t DirectLidarDefaultPort(DirectLidarModel model);

struct DirectLiveOptions
{
  DirectLidarModel model{DirectLidarModel::Auto};
  std::uint16_t udp_port{0}; // Model default (Auto listens on the common ports).
  std::uint16_t auxiliary_port{0}; // Use the model's IMU, positioning or DIFOP port by default.
  std::string sensor_ipv4;
  std::string multicast_ipv4;
  std::filesystem::path calibration_file;
};
struct LiveFrame
{
  FrameData frame;
  std::uint64_t sequence{0};
};
struct LiveStats
{
  bool connected{false}, recording{false}, record_busy{false}, record_stopping{false};
  std::uint64_t clouds{0}, imu_samples{0}, decode_errors{0}, replaced_frames{0};
  std::uint64_t recorded_messages{0}, recorded_bytes{0}, record_dropped{0};
  std::uint64_t recorded_point_packets{0}; // Direct Ethernet packets on the selected stream.
  double cloud_hz{0}, cloud_age_ms{-1}, imu_age_ms{-1};
  std::filesystem::path recording_path;
  std::string error, record_error;
  DirectLidarModel sensor_model{DirectLidarModel::Auto};
  std::string sensor_address;
  std::uint64_t packets{0}, sequence_gaps{0};
  std::uint16_t auxiliary_port{0};
  std::string auxiliary_error;
  bool factory_calibration{false};
};

class DirectLiveSession;

// The display owns a frozen copy while the receiver retains one latest frame.
// Both backends record before display filtering.
class LiveSession
{
public:
  LiveSession();
  ~LiveSession();
  LiveSession(const LiveSession &) = delete;
  LiveSession & operator=(const LiveSession &) = delete;
  static bool Available();  // Optional ROS 2 backend.
  void Connect(const LiveOptions & options);
  void ConnectDirect(const DirectLiveOptions & options);
  void Disconnect();
  bool Active() const;
  std::unique_ptr<LiveFrame> TakeLatest();
  LiveStats Stats() const;
  void StartRecording(const std::filesystem::path & destination);
  // Asynchronous: drain pending messages and finalize the recording.
  void StopRecording();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::unique_ptr<DirectLiveSession> direct_;
};
}  // namespace lidar_viewer
