#include "lidar_viewer/algorithms/ground_removal.hpp"

#include <stdexcept>

namespace lidar_viewer::algorithms
{
GroundRemovalResult RunGroundSegmentation(
  const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  const std::string mode = NormalizeGroundMode(params.mode);
  if (mode == "patchworkpp") {
    return RunGroundRemovalPatchworkPP(input, params);
  }
  if (mode == "patchwork_classic") return RunGroundRemovalPatchworkClassic(input, params);
  if (mode == "fugseg") return RunGroundRemovalFugSeg(input,params);
  if (mode == "grid") {
    return RunGroundRemovalGrid(input, params);
  }
  if (mode == "gseg3d") {
    return RunGroundRemovalGSeg3D(input, params);
  }
  if (mode == "fast_grid") {
    return RunGroundRemovalFastGrid(input, params);
  }
  if (mode=="ransac") return RunGroundRemovalRansac(input, params);
  throw std::invalid_argument("Unknown ground segmentation method: "+params.mode);
}
}  // namespace lidar_viewer::algorithms
