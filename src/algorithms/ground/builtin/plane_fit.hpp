#pragma once

#include "lidar_viewer/algorithms/ground_removal_cuda.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace lidar_viewer::algorithms::ground_detail
{
// The CPU fitter and CUDA scoring path use the same coefficient layout.
using Plane = CudaPlaneModel;

struct PlaneFit
{
  Plane plane;
  int inliers{-1};
  bool used_cuda{false};
};

std::optional<PlaneFit> FitCandidatePlane(
  const std::vector<KittiPoint> & input, const GroundRemovalParams & params,
  float min_normal_z, float threshold);

inline float SignedDistanceToPlane(const Plane & plane, const KittiPoint & point)
{
  return plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d;
}

inline bool IsCandidate(const KittiPoint & point, const GroundRemovalParams & params)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
    point.z < params.min_candidate_z || point.z > params.max_candidate_z) {
    return false;
  }

  const float radius_sq = point.x * point.x + point.y * point.y;
  const float max_radius = std::max(params.max_candidate_radius_m, 1.0f);
  return radius_sq <= max_radius * max_radius;
}

inline bool IsWithinRadius(const KittiPoint & point, const GroundRemovalParams & params)
{
  const float radius_sq = point.x * point.x + point.y * point.y;
  const float max_radius = std::max(params.max_candidate_radius_m, 1.0f);
  return radius_sq <= max_radius * max_radius;
}

inline float RangeAdaptiveThreshold(
  const KittiPoint & point,
  float base_threshold,
  const GroundRemovalParams & params)
{
  const float radius = std::sqrt(point.x * point.x + point.y * point.y);
  const float distance_scaled = base_threshold + std::max(params.adaptive_threshold_scale, 0.0f) * radius;
  return std::clamp(
    std::max(base_threshold, distance_scaled),
    base_threshold,
    std::max(params.max_adaptive_threshold_m, base_threshold));
}

inline std::uint64_t GridKey(std::int32_t ix, std::int32_t iy)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) ^ static_cast<std::uint32_t>(iy);
}

inline std::uint64_t GridKey(const KittiPoint & point, float cell_size)
{
  const auto ix = static_cast<std::int32_t>(std::floor(point.x / cell_size));
  const auto iy = static_cast<std::int32_t>(std::floor(point.y / cell_size));
  return GridKey(ix, iy);
}
}  // namespace lidar_viewer::algorithms::ground_detail
