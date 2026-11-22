#pragma once

#include "lidar_viewer/algorithms/ground_removal.hpp"
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace lidar_viewer
{
struct TerraSegStatus {
  bool configured{false}, ready{false}, failed{false};
  std::string message;
};

// Call Segment from a geometry worker. Status can be read by the UI while
// the model is loading or classifying; keep one instance across scans.
class TerraSeg {
public:
  explicit TerraSeg(const std::filesystem::path & python_root);
  ~TerraSeg();
  TerraSeg(const TerraSeg &) = delete;
  TerraSeg & operator=(const TerraSeg &) = delete;
  TerraSegStatus Status() const;
  void Cancel();
  std::optional<algorithms::GroundRemovalResult> Segment(
    const std::vector<KittiPoint> & points, float ground_height, float display_yaw, const Vec3 & sensor_origin);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lidar_viewer
