#pragma once

#include "lidar_viewer/algorithms/clustering.hpp"
#include "lidar_viewer/algorithms/ground_removal.hpp"
#include "lidar_viewer/display_alignment.hpp"
#include "lidar_viewer/roi.hpp"

#include <memory>
#include <optional>

namespace lidar_viewer
{
class TerraSeg;

namespace frame_processing
{
struct AlignmentOptions
{
  DisplayAlignment manual;
  ImuMountSettings imu_mount;
  float ground_offset{0};
  bool imu_enabled{false};
};

struct GeometryOptions
{
  AlignmentOptions alignment;
  algorithms::GroundRemovalParams ground;
  algorithms::ClusteringParams clustering;
  RoiSettings exclusions, display_roi;
  bool ground_enabled{false};
};

struct DetectorFrame
{
  std::shared_ptr<const std::vector<KittiPoint>> points;
  float yaw{0};
};

struct DisplayFrame
{
  FrameData frame;
  std::vector<algorithms::Cluster> clusters;
  std::size_t ground_points{0};
  std::optional<double> clustering_ms;
  double elapsed_ms{0};
};

struct GeometryResult
{
  std::shared_ptr<const FrameData> processed;
  DisplayFrame display;
  DetectorFrame detector;
  RoiCoordinateFrame coordinates;
  ImuLevelingResult leveling;
  std::size_t obstacles{0}, excluded_points{0};
  bool ground_pass{false};
  std::optional<double> ground_ms;
  double elapsed_ms{0};
};

void ApplyGlobalAlignment(FrameData & frame, float ground_offset);
DisplayFrame BuildDisplayFrame(const FrameData & processed, std::size_t obstacles,
  const RoiCoordinateFrame & coordinates, const RoiSettings & display_roi,
  bool ground_enabled, const algorithms::ClusteringParams & clustering);
DetectorFrame BuildDetectorFrame(const FrameData & source, const AlignmentOptions & options);

// The source is already translated by the dataset floor offset. Keep it in
// sensor point order for learned inference, then remove exclusions for ground.
GeometryResult ProcessFrame(const FrameData & source, const GeometryOptions & options,
  bool prepare_detector, TerraSeg * terraseg);
}  // namespace frame_processing
}  // namespace lidar_viewer
