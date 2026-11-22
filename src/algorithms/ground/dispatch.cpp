#include "lidar_viewer/algorithms/ground_removal.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace lidar_viewer::algorithms
{
GroundRemovalResult RunGroundSegmentation(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  std::string mode = params.mode;
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  if (mode == "patchworkpp" || mode == "patchwork++" || mode == "patchwork") {
    return RunGroundRemovalPatchworkPP(input, params);
  }
  if (mode == "patchwork_classic") return RunGroundRemovalPatchworkClassic(input, params);
  if (mode == "fugseg") return RunGroundRemovalFugSeg(input,params);
  if (mode == "grid") {
    return RunGroundRemovalGrid(input, params);
  }
  if (mode == "gseg3d" || mode == "gseg") {
    return RunGroundRemovalGSeg3D(input, params);
  }
  if (mode == "fast_grid") {
    return RunGroundRemovalFastGrid(input, params);
  }
  if (mode=="ransac") return RunGroundRemovalRansac(input, params);
  throw std::invalid_argument("Unknown ground segmentation method: "+params.mode);
}
}  // namespace lidar_viewer::algorithms
