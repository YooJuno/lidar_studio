#pragma once

#include "lidar_packets.hpp"

#include <fstream>

namespace lidar_viewer
{
// Holds only a bounded packet buffer. The PCAP writer preserves the exact
// UDP payload, source IP/port and receive time using standard IPv4 records.
class UdpPcapWriter
{
public:
  UdpPcapWriter(const fs::path & path, std::string_view calibration);
  std::size_t Write(const LidarDatagram & packet);
  void Finish();

private:
  fs::path path_, temporary_;
  std::ofstream stream_;
  std::uint64_t packets_{0};
};
}  // namespace lidar_viewer
