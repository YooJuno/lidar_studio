#include "plane_fit.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include <unordered_map>

namespace lidar_viewer::algorithms
{
namespace
{
using ground_detail::GridKey;
using ground_detail::IsCandidate;

std::unordered_map<std::uint64_t, float> BuildLocalGroundHeights(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  const float cell_size = std::max(params.candidate_grid_cell_m, 0.1f);
  std::unordered_map<std::uint64_t, float> lowest_z_by_cell;
  lowest_z_by_cell.reserve(input.size() / 8);

  for (const auto & point : input) {
    if (!IsCandidate(point, params)) {
      continue;
    }

    const std::uint64_t key = GridKey(point, cell_size);
    auto it = lowest_z_by_cell.find(key);
    if (it == lowest_z_by_cell.end() || point.z < it->second) {
      lowest_z_by_cell[key] = point.z;
    }
  }

  return lowest_z_by_cell;
}

bool IsNearLocalGround(
  const KittiPoint & point,
  const GroundRemovalParams & params,
  const std::unordered_map<std::uint64_t, float> & local_ground_heights)
{
  const float cell_size = std::max(params.candidate_grid_cell_m, 0.1f);
  const auto ix = static_cast<std::int32_t>(std::floor(point.x / cell_size));
  const auto iy = static_cast<std::int32_t>(std::floor(point.y / cell_size));

  float local_min_z = std::numeric_limits<float>::max();
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const std::uint64_t key = GridKey(ix + dx, iy + dy);
      const auto it = local_ground_heights.find(key);
      if (it != local_ground_heights.end()) {
        local_min_z = std::min(local_min_z, it->second);
      }
    }
  }

  if (local_min_z == std::numeric_limits<float>::max()) {
    return false;
  }

  return point.z <= local_min_z + std::max(params.local_ground_margin_m, 0.05f);
}

void LogGroundCudaBackendStatus(bool requested, bool used)
{
  static std::atomic<bool> logged{false};
  if (!requested || logged.exchange(true, std::memory_order_relaxed)) {
    return;
  }

  if (used) {
    std::cout << "Ground RANSAC CUDA backend enabled\n";
  } else {
    std::cout << "Ground RANSAC CUDA backend unavailable; using CPU fallback\n";
  }
}
}  // namespace

GroundRemovalResult RunGroundRemovalRansac(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  using namespace ground_detail;
  GroundRemovalResult result;
  if (!params.enabled || input.size() < 3) {
    result.non_ground_points = input;
    return result;
  }

  const float max_angle_rad = params.max_ground_angle_deg * 3.14159265358979323846f / 180.0f;
  const float min_normal_z = std::cos(max_angle_rad);
  const float threshold = std::max(params.distance_threshold_m, 0.01f);
  const float final_threshold = std::max(params.final_distance_threshold_m, threshold);
  const auto fit = FitCandidatePlane(input, params, min_normal_z, threshold);
  if (!fit) {
    result.non_ground_points = input;
    return result;
  }
  const auto & best_plane = fit->plane;
  const int best_inliers = fit->inliers;
  LogGroundCudaBackendStatus(params.use_cuda, fit->used_cuda);

  if (best_inliers < params.min_inliers) {
    result.non_ground_points = input;
    return result;
  }

  const auto local_ground_heights = BuildLocalGroundHeights(input, params);

  result.ground_points.reserve(static_cast<std::size_t>(best_inliers));
  result.non_ground_points.reserve(input.size() - static_cast<std::size_t>(best_inliers));
  for (const auto & point : input) {
    const float signed_distance = SignedDistanceToPlane(best_plane, point);
    const float adaptive_threshold = RangeAdaptiveThreshold(point, final_threshold, params);
    const float tight_threshold = std::min(final_threshold, 0.18f);
    const bool near_plane = std::abs(signed_distance) <= adaptive_threshold;
    const bool very_near_plane = std::abs(signed_distance) <= tight_threshold;
    const bool plausible_ground_height =
      signed_distance <= std::max(params.max_above_plane_m, final_threshold) &&
      signed_distance >= -std::max(params.max_below_plane_m, final_threshold);
    if (IsWithinRadius(point, params) && plausible_ground_height && near_plane &&
      (very_near_plane || IsNearLocalGround(point, params, local_ground_heights)))
    {
      result.ground_points.push_back(point);
    } else {
      result.non_ground_points.push_back(point);
    }
  }

  return result;
}
}  // namespace lidar_viewer::algorithms
