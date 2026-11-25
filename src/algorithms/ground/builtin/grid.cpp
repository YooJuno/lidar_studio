#include "plane_fit.hpp"

#include <algorithm>
#include <limits>

namespace lidar_viewer::algorithms
{
namespace
{
struct GridCellGround
{
  float min_z{std::numeric_limits<float>::max()};
  float max_z{std::numeric_limits<float>::lowest()};
  float radius{0.0f};
  bool valid{false};
  bool accepted_ground{false};
  float accepted_z{0.0f};
};

float CellCenterDistance(int cx, int cy, float cell_size)
{
  const float x = (static_cast<float>(cx) + 0.5f) * cell_size;
  const float y = (static_cast<float>(cy) + 0.5f) * cell_size;
  return std::sqrt(x * x + y * y);
}
}  // namespace

GroundRemovalResult RunGroundRemovalGrid(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  using namespace ground_detail;
  GroundRemovalResult result;
  if (!params.enabled || input.empty()) {
    result.non_ground_points = input;
    return result;
  }

  const float cell_size = std::max(params.candidate_grid_cell_m, 0.1f);
  const float max_angle_rad = params.max_ground_angle_deg * 3.14159265358979323846f / 180.0f;
  const float min_normal_z = std::cos(max_angle_rad);
  const float threshold = std::max(params.distance_threshold_m, 0.01f);
  const float final_threshold = std::max(params.final_distance_threshold_m, threshold);
  const float max_radius = std::max(params.max_candidate_radius_m, 1.0f);
  const float grid_min_x = -max_radius;
  const float grid_min_y = -max_radius;
  const int grid_width = std::max(1, static_cast<int>(std::ceil((2.0f * max_radius) / cell_size)) + 1);
  const int grid_height = grid_width;
  const std::size_t cell_count = static_cast<std::size_t>(grid_width) * static_cast<std::size_t>(grid_height);

  const auto fit = FitCandidatePlane(input, params, min_normal_z, threshold);
  const bool has_plane_prior = fit && fit->inliers >= std::max(params.min_inliers / 2, 40);
  const Plane best_plane = fit ? fit->plane : Plane{};

  std::vector<GridCellGround> cells(cell_count);
  std::vector<float> min_z_grid;
  std::vector<unsigned char> valid_grid;
  const bool used_cuda =
    params.use_cuda &&
    BuildMinZGridCuda(
      input,
      params,
      grid_min_x,
      grid_min_y,
      grid_width,
      grid_height,
      cell_size,
      min_z_grid,
      valid_grid);

  auto cell_index = [grid_width](int cx, int cy) {
    return static_cast<std::size_t>(cy) * static_cast<std::size_t>(grid_width) + static_cast<std::size_t>(cx);
  };

  bool has_valid_cell = false;
  for (const auto & point : input) {
    if (!IsCandidate(point, params)) {
      continue;
    }

    const int cx = static_cast<int>(std::floor((point.x - grid_min_x) / cell_size));
    const int cy = static_cast<int>(std::floor((point.y - grid_min_y) / cell_size));
    if (cx < 0 || cx >= grid_width || cy < 0 || cy >= grid_height) {
      continue;
    }
    auto & cell = cells[cell_index(cx, cy)];
    if (!cell.valid) {
      cell.radius = CellCenterDistance(
        cx - grid_width / 2,
        cy - grid_height / 2,
        cell_size);
    }
    cell.max_z = std::max(cell.max_z, point.z);
    // The CPU path can accumulate both extrema in a single pass over the scan.
    if (!used_cuda) {
      cell.valid = true;
      cell.min_z = std::min(cell.min_z, point.z);
      has_valid_cell = true;
    }
  }

  if (used_cuda) {
    for (int cy = 0; cy < grid_height; ++cy) {
      for (int cx = 0; cx < grid_width; ++cx) {
        const std::size_t index = cell_index(cx, cy);
        if (valid_grid[index] == 0) {
          continue;
        }
        auto & cell = cells[index];
        cell.valid = true;
        cell.min_z = min_z_grid[index];
        has_valid_cell = true;
      }
    }
  }

  if (!has_valid_cell) {
    result.non_ground_points = input;
    return result;
  }

  std::vector<std::size_t> ordered_cells;
  ordered_cells.reserve(std::min(cell_count, input.size()));
  for (std::size_t index = 0; index < cells.size(); ++index) {
    if (cells[index].valid) {
      ordered_cells.push_back(index);
    }
  }
  std::sort(
    ordered_cells.begin(),
    ordered_cells.end(),
    [&cells](std::size_t lhs, std::size_t rhs) {
      return cells[lhs].radius < cells[rhs].radius;
    });

  const float seed_radius = std::max(params.grid_seed_radius_m, cell_size * 2.0f);
  const float max_height_step = std::max(params.grid_max_height_step_m, 0.02f);
  const float max_local_slope = std::max(params.grid_max_local_slope_m_per_m, 0.01f);
  const float max_vertical_extent = std::max(params.grid_max_vertical_extent_m, 0.05f);

  for (const std::size_t index : ordered_cells) {
    auto & cell = cells[index];
    const float radius = cell.radius;
    const int cx = static_cast<int>(index % static_cast<std::size_t>(grid_width));
    const int cy = static_cast<int>(index / static_cast<std::size_t>(grid_width));

    if (radius <= seed_radius) {
      if ((cell.max_z - cell.min_z) > max_vertical_extent) {
        continue;
      }
      if (has_plane_prior) {
        const float cell_x = grid_min_x + (static_cast<float>(cx) + 0.5f) * cell_size;
        const float cell_y = grid_min_y + (static_cast<float>(cy) + 0.5f) * cell_size;
        const KittiPoint probe{cell_x, cell_y, cell.min_z, 0.0f};
        const float signed_distance =
          best_plane.a * cell_x + best_plane.b * cell_y + best_plane.c * cell.min_z + best_plane.d;
        if (std::abs(signed_distance) > RangeAdaptiveThreshold(probe, final_threshold, params)) {
          continue;
        }
      }
      cell.accepted_ground = true;
      cell.accepted_z = cell.min_z;
      continue;
    }

    if ((cell.max_z - cell.min_z) > max_vertical_extent) {
      continue;
    }

    bool found_neighbor_ground = false;
    int supporting_neighbors = 0;
    float best_neighbor_z = std::numeric_limits<float>::max();
    float highest_support_z = std::numeric_limits<float>::lowest();
    float best_allowed_z = std::numeric_limits<float>::max();

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (nx < 0 || nx >= grid_width || ny < 0 || ny >= grid_height) {
          continue;
        }
        const auto & neighbor = cells[cell_index(nx, ny)];
        if (!neighbor.accepted_ground) {
          continue;
        }
        if (neighbor.radius > radius + cell_size * 0.5f) {
          continue;
        }

        const float distance = std::sqrt(static_cast<float>(dx * dx + dy * dy)) * cell_size;
        const float allowed_z = neighbor.accepted_z + max_height_step + max_local_slope * distance;
        best_neighbor_z = std::min(best_neighbor_z, neighbor.accepted_z);
        highest_support_z = std::max(highest_support_z, neighbor.accepted_z);
        best_allowed_z = std::min(best_allowed_z, allowed_z);
        ++supporting_neighbors;
        found_neighbor_ground = true;
      }
    }

    if (!found_neighbor_ground || supporting_neighbors < 2) {
      continue;
    }

    if ((highest_support_z - best_neighbor_z) > max_height_step * 1.5f) {
      continue;
    }

    if (cell.min_z <= best_allowed_z && cell.min_z >= best_neighbor_z - std::max(params.max_below_plane_m, 0.2f)) {
      if (has_plane_prior) {
        const float cell_x = grid_min_x + (static_cast<float>(cx) + 0.5f) * cell_size;
        const float cell_y = grid_min_y + (static_cast<float>(cy) + 0.5f) * cell_size;
        const KittiPoint probe{cell_x, cell_y, cell.min_z, 0.0f};
        const float signed_distance =
          best_plane.a * cell_x + best_plane.b * cell_y + best_plane.c * cell.min_z + best_plane.d;
        if (
          signed_distance > std::max(params.max_above_plane_m, final_threshold) ||
          signed_distance < -std::max(params.max_below_plane_m, final_threshold) ||
          std::abs(signed_distance) > RangeAdaptiveThreshold(probe, final_threshold, params))
        {
          continue;
        }
      }
      cell.accepted_ground = true;
      cell.accepted_z = std::min(cell.min_z, best_allowed_z);
    }
  }

  std::vector<unsigned char> is_ground_mask;
  bool used_cuda_classification = false;
  if (params.use_cuda) {
    std::vector<float> accepted_ground_z_grid(cell_count, std::numeric_limits<float>::max());
    std::vector<unsigned char> accepted_ground_valid_grid(cell_count, 0);
    for (std::size_t index = 0; index < cells.size(); ++index) {
      if (!cells[index].accepted_ground) {
        continue;
      }
      accepted_ground_z_grid[index] = cells[index].accepted_z;
      accepted_ground_valid_grid[index] = 1;
    }
    used_cuda_classification = ClassifyGroundPointsGridCuda(
      input,
      params,
      grid_min_x,
      grid_min_y,
      grid_width,
      grid_height,
      cell_size,
      accepted_ground_z_grid,
      accepted_ground_valid_grid,
      is_ground_mask);
  }

  result.ground_points.reserve(input.size() / 3);
  result.non_ground_points.reserve(input.size());
  const float ego_hood_min_z = params.gseg3d_ground_height_m + 0.2f;
  if (used_cuda_classification) {
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto & point = input[i];
      const bool inside_ego_hood_box =
        point.x >= 0.5f && point.x <= 4.0f &&
        std::abs(point.y) <= 1.4f &&
        point.z >= ego_hood_min_z;
      const float signed_distance =
        has_plane_prior ? SignedDistanceToPlane(best_plane, point) : 0.0f;
      const bool plane_ok =
        !has_plane_prior ||
        (
          signed_distance <= std::max(params.max_above_plane_m, final_threshold) &&
          signed_distance >= -std::max(params.max_below_plane_m, final_threshold) &&
          std::abs(signed_distance) <= RangeAdaptiveThreshold(point, final_threshold, params)
        );
      if (is_ground_mask[i] != 0 && !inside_ego_hood_box && plane_ok) {
        result.ground_points.push_back(input[i]);
      } else {
        result.non_ground_points.push_back(input[i]);
      }
    }
  } else {
    for (const auto & point : input) {
      if (!IsWithinRadius(point, params)) {
        result.non_ground_points.push_back(point);
        continue;
      }

      const bool inside_ego_hood_box =
        point.x >= 0.5f && point.x <= 4.0f &&
        std::abs(point.y) <= 1.4f &&
        point.z >= ego_hood_min_z;
      if (inside_ego_hood_box) {
        result.non_ground_points.push_back(point);
        continue;
      }

      const int cx = static_cast<int>(std::floor((point.x - grid_min_x) / cell_size));
      const int cy = static_cast<int>(std::floor((point.y - grid_min_y) / cell_size));
      bool classified_ground = false;
      float local_ground_z = std::numeric_limits<float>::max();
      float highest_ground_z = std::numeric_limits<float>::lowest();
      int support_count = 0;

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || nx >= grid_width || ny < 0 || ny >= grid_height) {
            continue;
          }
          const auto & cell = cells[cell_index(nx, ny)];
          if (!cell.accepted_ground || (cell.max_z - cell.min_z) > max_vertical_extent) {
            continue;
          }
          local_ground_z = std::min(local_ground_z, cell.accepted_z);
          highest_ground_z = std::max(highest_ground_z, cell.accepted_z);
          ++support_count;
        }
      }

      if (
        local_ground_z != std::numeric_limits<float>::max() &&
        support_count >= ((std::sqrt(point.x * point.x + point.y * point.y) > 25.0f) ? 1 : 2) &&
        (highest_ground_z - local_ground_z) <= (std::max(params.max_above_plane_m, 0.05f) * 1.5f))
      {
        const float allowed_above =
          RangeAdaptiveThreshold(point, std::max(params.final_distance_threshold_m, 0.05f), params);
        const float dz = point.z - local_ground_z;
        const float signed_distance =
          has_plane_prior ? SignedDistanceToPlane(best_plane, point) : 0.0f;
        const bool plane_ok =
          !has_plane_prior ||
          (
            signed_distance <= std::max(params.max_above_plane_m, final_threshold) &&
            signed_distance >= -std::max(params.max_below_plane_m, final_threshold) &&
            std::abs(signed_distance) <= allowed_above
          );
        classified_ground =
          plane_ok &&
          dz <= std::min(std::max(params.max_above_plane_m, allowed_above), allowed_above) &&
          dz >= -std::max(params.max_below_plane_m, 0.1f);
      }

      if (classified_ground) {
        result.ground_points.push_back(point);
      } else {
        result.non_ground_points.push_back(point);
      }
    }
  }

  result.non_ground_points.shrink_to_fit();
  result.ground_points.shrink_to_fit();
  return result;
}
}  // namespace lidar_viewer::algorithms
