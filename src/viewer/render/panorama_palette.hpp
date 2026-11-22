#pragma once

#include <imgui.h>

namespace lidar_viewer::rendering
{
// Viridis by Nathaniel J. Smith, Stefan van der Walt, and Eric Firing (CC0).
// https://github.com/BIDS/colormap/blob/master/colormaps.py
inline constexpr ImU32 kPanoramaRangeColors[] = {
  IM_COL32(68, 1, 84, 255), IM_COL32(72, 26, 108, 255), IM_COL32(71, 47, 125, 255),
  IM_COL32(65, 68, 135, 255), IM_COL32(57, 86, 140, 255), IM_COL32(49, 104, 142, 255),
  IM_COL32(42, 120, 142, 255), IM_COL32(35, 136, 142, 255), IM_COL32(31, 152, 139, 255),
  IM_COL32(34, 168, 132, 255), IM_COL32(53, 183, 121, 255), IM_COL32(84, 197, 104, 255),
  IM_COL32(122, 209, 81, 255), IM_COL32(165, 219, 54, 255), IM_COL32(210, 226, 27, 255),
  IM_COL32(253, 231, 37, 255),
};
}  // namespace lidar_viewer::rendering
