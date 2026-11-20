#pragma once

#include "lidar_viewer/data_io.hpp"

namespace lidar_viewer
{
inline constexpr const char * kLivePcapFilename = "lidar.pcap";
inline constexpr const char * kPcapCorrectionSuffix = ".calibration";

fs::path FindLidarPcap(const fs::path & path);
std::vector<FrameHandle> IndexLidarPcap(const fs::path & path, InputLoadState * load = nullptr);
FrameData ReadLidarPcapFrame(const FrameHandle & handle);
}  // namespace lidar_viewer
