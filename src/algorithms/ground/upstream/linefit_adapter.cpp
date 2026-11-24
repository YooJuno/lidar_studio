#include "lidar_viewer/algorithms/ground_partition.hpp"

#include <ground_segmentation/segment.h>

#include <array>
#include <cmath>
#include <optional>

namespace lidar_viewer::algorithms
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumRange = 0.5;
constexpr int kSectors = 360;
constexpr double kSectorStep = 2 * kPi / kSectors;
constexpr double kNeighborSearch = 0.1;

struct PolarPoint
{
  double range{0};
  int sector{-1};
};

using Sectors = std::array<std::optional<Segment>, kSectors>;

double GroundDistance(Sectors & sectors, const PolarPoint & point, double sensor_z)
{
  auto distance_to_sector = [&](int index) {
    auto & sector = sectors[index];
    return sector ? sector->verticalDistanceToLine(point.range, sensor_z) : -1.0;
  };
  double distance = distance_to_sector(point.sector);
  for (int step = 1; distance < 0 && step * kSectorStep < kNeighborSearch; ++step) {
    const double next = distance_to_sector((point.sector + step) % kSectors);
    const double previous = distance_to_sector((point.sector + kSectors - step) % kSectors);
    distance = next < 0 ? previous : previous < 0 ? next : std::min(next, previous);
  }
  return distance;
}
}  // namespace

GroundRemovalResult RunGroundRemovalLineFit(const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  if (input.empty()) return {};
  for (const float value : {params.linefit_max_range_m, params.linefit_distance_threshold_m,
      params.sensor_height_m, params.gseg3d_ground_height_m}) {
    if (!std::isfinite(value)) throw std::invalid_argument("LineFit parameters must be finite");
  }
  const double range = std::clamp<double>(params.linefit_max_range_m, 10, 160);
  const double thickness = std::clamp<double>(params.linefit_distance_threshold_m, 0.02, 0.5);
  const double height = std::max<double>(params.sensor_height_m, 0.05);
  const double sensor_origin_z = params.gseg3d_ground_height_m + height;
  // Preserve the upstream 120 bins over 0.5–50 m when changing the range.
  const auto bins = static_cast<unsigned>(std::ceil((range - kMinimumRange) * 120 / 49.5));
  const double bin_step = (range - kMinimumRange) / bins;
  Sectors sectors;
  std::vector<PolarPoint> polar(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    const auto & point = input[i];
    const double x = point.x, y = point.y;
    const double squared = x * x + y * y;
    if (!(squared > kMinimumRange * kMinimumRange && squared < range * range) || !std::isfinite(point.z)) {
      continue;
    }
    const double distance = std::sqrt(squared);
    const double angle = std::atan2(y, x);
    const auto index = static_cast<int>((angle + kPi) / kSectorStep) % kSectors;
    const auto bin = std::min(bins - 1, static_cast<unsigned>((distance - kMinimumRange) / bin_step));
    auto & sector = sectors[index];
    if (!sector) sector.emplace(bins, 0.0, 0.3, 0.05 * 0.05, 1.0, 0.1, 0.2, height);
    (*sector)[bin].addPoint(distance, point.z - sensor_origin_z);
    polar[i] = {distance, index};
  }
  for (auto & sector : sectors) {
    if (sector) sector->fitSegmentLines();
  }

  std::vector<unsigned char> mask(input.size(), 0);
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (polar[i].sector < 0) continue;
    const double distance = GroundDistance(sectors, polar[i], input[i].z - sensor_origin_z);
    mask[i] = distance >= 0 && distance < thickness;
  }
  return PartitionGround(input, mask);
}
}  // namespace lidar_viewer::algorithms
