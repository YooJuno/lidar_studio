#include "plane_fit.hpp"

#include <algorithm>
#include <cstdint>
#include <random>
#include <unordered_map>

namespace lidar_viewer::algorithms::ground_detail
{
namespace
{
Vec3 Subtract(const KittiPoint & lhs, const KittiPoint & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 CrossProduct(const Vec3 & lhs, const Vec3 & rhs)
{
  return {
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x
  };
}

float DotProduct(const Vec3 & lhs, const Vec3 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

float Length(const Vec3 & value)
{
  return std::sqrt(DotProduct(value, value));
}

bool TryMakePlane(
  const KittiPoint & p0,
  const KittiPoint & p1,
  const KittiPoint & p2,
  Plane & plane)
{
  Vec3 normal = CrossProduct(Subtract(p1, p0), Subtract(p2, p0));
  const float len = Length(normal);
  if (len < 1e-5f) {
    return false;
  }

  normal = normal * (1.0f / len);
  if (normal.z < 0.0f) {
    normal = normal * -1.0f;
  }

  plane.a = normal.x;
  plane.b = normal.y;
  plane.c = normal.z;
  plane.d = -(normal.x * p0.x + normal.y * p0.y + normal.z * p0.z);
  return true;
}

float DistanceToPlane(const Plane & plane, const KittiPoint & point)
{
  return std::abs(plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d);
}

std::vector<std::size_t> BuildLowestPointCandidates(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  const float cell_size = std::max(params.candidate_grid_cell_m, 0.1f);
  std::unordered_map<std::uint64_t, std::size_t> lowest_by_cell;
  lowest_by_cell.reserve(input.size() / 8);

  for (std::size_t i = 0; i < input.size(); ++i) {
    if (!IsCandidate(input[i], params)) {
      continue;
    }

    const std::uint64_t key = GridKey(input[i], cell_size);
    auto it = lowest_by_cell.find(key);
    if (it == lowest_by_cell.end() || input[i].z < input[it->second].z) {
      lowest_by_cell[key] = i;
    }
  }

  std::vector<std::size_t> candidates;
  candidates.reserve(lowest_by_cell.size());
  for (const auto & [_, index] : lowest_by_cell) {
    candidates.push_back(index);
  }
  return candidates;
}

std::vector<CudaPlaneModel> BuildPlaneHypotheses(
  const std::vector<KittiPoint> & input,
  const std::vector<std::size_t> & candidates,
  int iterations,
  float min_normal_z)
{
  std::mt19937 rng(1337u);
  std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);

  std::vector<CudaPlaneModel> planes;
  planes.reserve(static_cast<std::size_t>(iterations));
  for (int iter = 0; iter < iterations; ++iter) {
    const std::size_t i0 = candidates[pick(rng)];
    const std::size_t i1 = candidates[pick(rng)];
    const std::size_t i2 = candidates[pick(rng)];
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    Plane plane;
    if (!TryMakePlane(input[i0], input[i1], input[i2], plane)) {
      continue;
    }
    if (plane.c < min_normal_z) {
      continue;
    }

    planes.push_back({plane.a, plane.b, plane.c, plane.d});
  }

  return planes;
}

std::vector<int> ScorePlanesCpu(
  const std::vector<KittiPoint> & input,
  const std::vector<std::size_t> & candidates,
  const std::vector<CudaPlaneModel> & planes,
  float threshold)
{
  std::vector<int> inlier_counts(planes.size(), 0);
  for (std::size_t plane_index = 0; plane_index < planes.size(); ++plane_index) {
    const Plane plane{
      planes[plane_index].a,
      planes[plane_index].b,
      planes[plane_index].c,
      planes[plane_index].d
    };

    int inliers = 0;
    for (const std::size_t candidate_index : candidates) {
      if (DistanceToPlane(plane, input[candidate_index]) <= threshold) {
        ++inliers;
      }
    }
    inlier_counts[plane_index] = inliers;
  }
  return inlier_counts;
}
}  // namespace

std::optional<PlaneFit> FitCandidatePlane(
  const std::vector<KittiPoint> & input, const GroundRemovalParams & params,
  float min_normal_z, float threshold)
{
  const auto candidates = BuildLowestPointCandidates(input, params);
  if (candidates.size() < 3) {
    return std::nullopt;
  }

  const auto planes =
    BuildPlaneHypotheses(input, candidates, std::max(params.max_iterations, 1), min_normal_z);
  if (planes.empty()) {
    return std::nullopt;
  }

  PlaneFit fit;
  std::vector<int> inlier_counts;
  fit.used_cuda = params.use_cuda && ScorePlanesCuda(input, candidates, planes, threshold, inlier_counts);
  if (!fit.used_cuda) {
    inlier_counts = ScorePlanesCpu(input, candidates, planes, threshold);
  }

  for (std::size_t index = 0; index < planes.size(); ++index) {
    if (inlier_counts[index] > fit.inliers) {
      fit.inliers = inlier_counts[index];
      fit.plane = planes[index];
    }
  }
  return fit;
}
}  // namespace lidar_viewer::algorithms::ground_detail
