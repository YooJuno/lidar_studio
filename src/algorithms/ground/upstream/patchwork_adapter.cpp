#include "lidar_viewer/algorithms/ground_removal.hpp"
#include "lidar_viewer/algorithms/ground_partition.hpp"
#include <patchwork/patchwork.h>
#include <patchwork/patchworkpp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lidar_viewer::algorithms
{
namespace
{
template <typename Model, typename Options, std::size_t KeySize>
GroundRemovalResult ClassifyWithPatchwork(const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params, const Options & options, const std::array<double, KeySize> & key)
{
  if (input.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("Point cloud exceeds Patchwork index capacity");

  struct Workspace {
    std::unique_ptr<Model> model;
    std::array<double, KeySize> key{};
    std::uint64_t sequence{0};
    Eigen::MatrixXf cloud;
    std::vector<int> original;
    std::vector<unsigned char> ground;
  };
  static thread_local Workspace workspace;
  if (!workspace.model || workspace.key!=key || workspace.sequence!=params.sequence_id) {
    workspace.model=std::make_unique<Model>(options);
    workspace.key=key;
    workspace.sequence=params.sequence_id;
  }
  workspace.cloud.resize(static_cast<Eigen::Index>(input.size()), 3);
  workspace.original.clear();
  workspace.original.reserve(input.size());
  for (std::size_t i=0; i<input.size(); ++i) {
    const auto & point=input[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const auto row=static_cast<Eigen::Index>(workspace.original.size());
    workspace.cloud.row(row) << point.x, point.y, point.z-params.gseg3d_ground_height_m-options.sensor_height;
    workspace.original.push_back(static_cast<int>(i));
  }
  workspace.ground.assign(input.size(), 0);
  if (!workspace.original.empty()) {
    workspace.cloud.conservativeResize(static_cast<Eigen::Index>(workspace.original.size()), Eigen::NoChange);
    // Patchwork++ takes the matrix by value; classic takes a const reference.
    workspace.model->estimateGround(std::move(workspace.cloud));
    const auto indices=workspace.model->getGroundIndices();
    for (std::size_t i=0; i<static_cast<std::size_t>(indices.size()); ++i) {
      const int index=indices[i];
      if (index>=0 && static_cast<std::size_t>(index)<workspace.original.size())
        workspace.ground[workspace.original[index]]=1;
    }
  }
  return PartitionGround(input, workspace.ground);
}
}  // namespace

GroundRemovalResult RunGroundRemovalPatchworkPP(const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  if (!params.enabled || input.empty()) return {{}, input};
  patchwork::Params options;
  options.verbose=false;
  // Intensity is normalized per frame in this viewer, so the original raw
  // reflectivity threshold cannot be applied faithfully. Keep geometry-based
  // R-VPF, A-GLE and TGR enabled; preserve every unclassified point as obstacle.
  options.enable_RNR=false;
  options.sensor_height=std::clamp(static_cast<double>(params.sensor_height_m),.05,20.);
  options.min_range=std::max(0.,static_cast<double>(params.patchwork_min_range_m));
  options.max_range=std::max(options.min_range+1.,static_cast<double>(params.max_candidate_radius_m));
  options.num_iter=std::clamp(params.patchwork_refine_iterations,1,20);
  options.num_lpr=std::max(3,params.patchwork_seed_count);
  options.num_min_pts=std::max(3,params.patchwork_min_points_per_patch);
  options.th_seeds=std::max(.01,static_cast<double>(params.patchwork_seed_threshold_m));
  options.th_dist=std::max(.01,static_cast<double>(params.patchwork_distance_threshold_m));
  options.uprightness_thr=std::cos(std::clamp(static_cast<double>(params.max_ground_angle_deg),0.,60.)*kPi/180.);
  const double sectors=std::clamp(params.patchwork_num_sectors,4,128)/32.;
  options.num_sectors_each_zone={std::max(4,static_cast<int>(16*sectors)),std::max(4,static_cast<int>(32*sectors)),
    std::max(4,static_cast<int>(54*sectors)),std::max(4,static_cast<int>(32*sectors))};
  const std::array<double,11> key{{options.sensor_height,options.min_range,options.max_range,
    static_cast<double>(options.num_iter),static_cast<double>(options.num_lpr),static_cast<double>(options.num_min_pts),
    options.th_seeds,options.th_dist,options.uprightness_thr,sectors,params.gseg3d_ground_height_m}};
  return ClassifyWithPatchwork<patchwork::PatchWorkpp>(input, params, options, key);
}

GroundRemovalResult RunGroundRemovalPatchworkClassic(const std::vector<KittiPoint> & input,
  const GroundRemovalParams & params)
{
  if (!params.enabled || input.empty()) return {{}, input};
  patchwork::PatchworkParams options;
  // The pinned core starts its below-ground noise cutoff at its default
  // sensor height. Run the leveled scan at that reference height and map
  // labels back to the original points. Align supplies the ground reference.
  options.ATAT_ON=false;
  options.min_range=std::max(0., static_cast<double>(params.patchwork_min_range_m));
  options.max_range=std::max(options.min_range+1., static_cast<double>(params.max_candidate_radius_m));
  const double span=options.max_range-options.min_range;
  if (!std::isfinite(span) || span<1. || !std::isfinite(params.gseg3d_ground_height_m))
    throw std::invalid_argument("Invalid Patchwork classic ground reference or range");
  options.min_ranges={options.min_range, options.min_range+span/8.,
    options.min_range+span/4., options.min_range+span/2.};
  const std::array<double, 3> key{{options.min_range, options.max_range, params.gseg3d_ground_height_m}};
  return ClassifyWithPatchwork<patchwork::PatchWork>(input, params, options, key);
}
}  // namespace lidar_viewer::algorithms
