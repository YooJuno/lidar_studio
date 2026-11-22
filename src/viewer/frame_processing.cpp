#include "frame_processing.hpp"

#include "lidar_viewer/terraseg.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace lidar_viewer::frame_processing
{
namespace
{
using Clock = std::chrono::steady_clock;

double Elapsed(Clock::time_point started)
{
  return std::chrono::duration<double, std::milli>(Clock::now()-started).count();
}

void RecomputeBounds(FrameData & frame)
{
  if (frame.points.empty()) {
    frame.center = {};
    frame.radius = 10.0f;
    frame.min_z = 0.0f;
    frame.max_z = 0.0f;
    return;
  }

  Vec3 min_corner{
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max()
  };
  Vec3 max_corner{
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest(),
    std::numeric_limits<float>::lowest()
  };
  for (const auto & p : frame.points) {
    min_corner.x = std::min(min_corner.x, p.x);
    min_corner.y = std::min(min_corner.y, p.y);
    min_corner.z = std::min(min_corner.z, p.z);
    max_corner.x = std::max(max_corner.x, p.x);
    max_corner.y = std::max(max_corner.y, p.y);
    max_corner.z = std::max(max_corner.z, p.z);
  }

  frame.center = {
    0.5f * (min_corner.x + max_corner.x),
    0.5f * (min_corner.y + max_corner.y),
    0.5f * (min_corner.z + max_corner.z)
  };
  const Vec3 extent = max_corner - min_corner;
  frame.radius = std::max(std::max({extent.x, extent.y, extent.z}) * 0.7f, 10.0f);
  frame.min_z = min_corner.z;
  frame.max_z = max_corner.z;
}

struct CalibrationTransform
{
  std::array<float, 9> rotation{};
  float z_offset{0.0f};

  Vec3 Apply(const KittiPoint & p) const
  {
    return {
      rotation[0] * p.x + rotation[1] * p.y + rotation[2] * p.z,
      rotation[3] * p.x + rotation[4] * p.y + rotation[5] * p.z,
      rotation[6] * p.x + rotation[7] * p.y + rotation[8] * p.z + z_offset
    };
  }
};

CalibrationTransform BuildTransform(const DisplayAlignment & calibration)
{
  const float yaw = calibration.yaw_deg * kPi / 180.0f;
  const float pitch = calibration.pitch_deg * kPi / 180.0f;
  const float roll = calibration.roll_deg * kPi / 180.0f;
  const float cy = std::cos(yaw), sy = std::sin(yaw);
  const float cp = std::cos(pitch), sp = std::sin(pitch);
  const float cr = std::cos(roll), sr = std::sin(roll);
  return {{
    cp * cy, (sr * sp * cy - cr * sy), (cr * sp * cy + sr * sy),
    cp * sy, (sr * sp * sy + cr * cy), (cr * sp * sy - sr * cy),
    -sp,     sr * cp,                  cr * cp
  }, calibration.z_m};
}

ImuLevelingResult DetermineLeveling(const FrameData & source, const AlignmentOptions & options)
{
  if (options.imu_enabled) return ComputeImuLeveling(source.imu, options.imu_mount);
  ImuLevelingResult leveling;
  leveling.status = "Off";
  return leveling;
}

template<class AlignedPoint>
DetectorFrame PrepareDetector(const FrameData & source, float yaw, float ground_offset, AlignedPoint aligned)
{
  auto points = std::make_shared<std::vector<KittiPoint>>();
  points->reserve(source.points.size());
  const auto * intensity = source.source_intensities && source.source_intensities->size() == source.points.size() ?
    source.source_intensities->data() : nullptr;
  for (std::size_t i = 0; i < source.points.size(); ++i) {
    const auto & raw = source.points[i];
    // Test the field at the sensor origin before it becomes an aligned pillar.
    if (raw.x == 0 && raw.y == 0 && raw.z == -ground_offset) continue;
    auto point = aligned(i);
    point.intensity = intensity ? intensity[i] : std::numeric_limits<float>::quiet_NaN();
    points->push_back(point);
  }
  return {std::move(points), yaw};
}

void RemoveGround(FrameData & frame, GeometryResult & output, const GeometryOptions & options,
  const PreparedRoi & exclusions, const CalibrationTransform & transform, float yaw, TerraSeg * terraseg)
{
  output.obstacles = frame.points.size();
  const bool learned = options.ground.mode == "terraseg";
  if (!options.ground_enabled || (learned && !terraseg) || frame.points.empty()) {
    output.excluded_points = exclusions.CountRejected(frame.points);
    return;
  }

  auto skipped = exclusions.ExtractRejected(frame.points);
  output.excluded_points = skipped.size();
  auto params = options.ground;
  params.enabled = true;
  params.sensor_height_m = std::max(.05f, transform.z_offset-options.alignment.ground_offset-params.gseg3d_ground_height_m);
  std::optional<algorithms::GroundRemovalResult> result;
  if (!frame.points.empty()) {
    double ground_ms = 0;
    if (learned) {
      const auto origin = transform.Apply({0, 0, -options.alignment.ground_offset, 0});
      result = terraseg->Segment(frame.points, params.gseg3d_ground_height_m, yaw, origin, ground_ms);
    } else {
      const auto started = Clock::now();
      result = algorithms::RunGroundSegmentation(frame.points, params);
      ground_ms = Elapsed(started);
    }
    if (result) output.ground_ms = ground_ms;
  }
  if (result) {
    frame.points = std::move(result->non_ground_points);
    frame.points.reserve(frame.points.size()+skipped.size()+result->ground_points.size());
  }
  // Ground-excluded points remain in the obstacle prefix in input order.
  frame.points.insert(frame.points.end(), skipped.begin(), skipped.end());
  output.obstacles = frame.points.size();
  if (result) {
    for (auto point : result->ground_points) {
      point.intensity = -1;
      frame.points.push_back(point);
    }
  }
  output.ground_pass = result.has_value() || !learned;
}
}  // namespace

void ApplyGlobalAlignment(FrameData & frame, float ground_offset)
{
  if (frame.points.empty() || ground_offset == 0.0f) return;
  for (auto & p : frame.points) p.z -= ground_offset;
  frame.center.z -= ground_offset;
  frame.min_z -= ground_offset;
  frame.max_z -= ground_offset;
  frame.ground_z -= ground_offset;
}

DisplayFrame BuildDisplayFrame(const FrameData & processed, std::size_t obstacles,
  const RoiCoordinateFrame & coordinates, const RoiSettings & display_roi,
  bool ground_enabled, const algorithms::ClusteringParams & clustering)
{
  const auto started = Clock::now();
  DisplayFrame result;
  auto & frame = result.frame;
  frame.frame_id = processed.frame_id;
  frame.imu = processed.imu;
  frame.ground_z = processed.ground_z;
  frame.timestamp_ns = processed.timestamp_ns;
  frame.acquisition_timestamp_ns = processed.acquisition_timestamp_ns;
  frame.receipt_timestamp_ns = processed.receipt_timestamp_ns;
  if (ground_enabled && clustering.enabled && obstacles) {
    // Clustering sees the obstacle prefix before the visual crop.
    const std::vector<KittiPoint> input(processed.points.begin(),
      processed.points.begin()+static_cast<std::ptrdiff_t>(obstacles));
    const auto clustering_started = Clock::now();
    result.clusters = algorithms::RunClustering(input, clustering);
    result.clustering_ms = Elapsed(clustering_started);
  }
  std::size_t visible_obstacles = obstacles;
  if (display_roi.enabled) {
    const PreparedRoi display(display_roi, coordinates);
    frame.points.reserve(std::min(processed.points.size(), std::max<std::size_t>(8192, processed.points.size()/2)));
    visible_obstacles = 0;
    for (std::size_t i = 0; i < processed.points.size(); ++i) {
      const auto & point = processed.points[i];
      if (!display.Contains({point.x, point.y, point.z})) continue;
      frame.points.push_back(point);
      if (i < obstacles) ++visible_obstacles;
    }
  } else {
    frame.points = processed.points;
  }
  RecomputeBounds(frame);
  result.ground_points = frame.points.size()-visible_obstacles;
  result.elapsed_ms = Elapsed(started);
  return result;
}

DetectorFrame BuildDetectorFrame(const FrameData & source, const AlignmentOptions & options)
{
  const auto leveling = DetermineLeveling(source, options);
  const auto resolved = ResolveDisplayAlignment(options.manual, options.imu_enabled, leveling);
  const auto transform = BuildTransform(resolved.effective);
  return PrepareDetector(source, resolved.effective.yaw_deg*kPi/180.f, options.ground_offset,
    [&](std::size_t i) {
      const auto p = transform.Apply(source.points[i]);
      return KittiPoint{p.x, p.y, p.z, 0};
    });
}

GeometryResult ProcessFrame(const FrameData & source, const GeometryOptions & options,
  bool prepare_detector, TerraSeg * terraseg)
{
  const auto started = Clock::now();
  GeometryResult output;
  FrameData frame = source;
  frame.source_intensities.reset();
  if (frame.annotations) {
    output.obstacles = frame.points.size();
    output.display.frame = frame;
    output.processed = std::make_shared<const FrameData>(std::move(frame));
    return output;
  }

  output.leveling = DetermineLeveling(source, options.alignment);
  const auto resolved = ResolveDisplayAlignment(options.alignment.manual, options.alignment.imu_enabled, output.leveling);
  const auto transform = BuildTransform(resolved.effective);
  const float yaw = resolved.effective.yaw_deg*kPi/180.f;
  output.coordinates.rotation = transform.rotation;
  output.coordinates.translation = output.coordinates.Rotate({0, 0, -options.alignment.ground_offset})+
    Vec3{0, 0, transform.z_offset};
  const PreparedRoi exclusions(options.exclusions, output.coordinates, RoiFilterStage::Preprocessing);
  for (auto & point : frame.points) {
    const auto p = transform.Apply(point);
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
  }
  if (prepare_detector)
    output.detector = PrepareDetector(source, yaw, options.alignment.ground_offset,
      [&](std::size_t i) {return frame.points[i];});
  RemoveGround(frame, output, options, exclusions, transform, yaw, terraseg);
  output.display = BuildDisplayFrame(frame, output.obstacles, output.coordinates, options.display_roi,
    options.ground_enabled, options.clustering);
  output.processed = std::make_shared<const FrameData>(std::move(frame));
  output.elapsed_ms = Elapsed(started);
  return output;
}
}  // namespace lidar_viewer::frame_processing
