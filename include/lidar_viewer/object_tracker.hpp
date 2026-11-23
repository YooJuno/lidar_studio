#pragma once

#include "lidar_viewer/algorithms/detection.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lidar_viewer
{
struct TrackingParams
{
  bool enabled{true}, show_trails{false}, show_speed{false};
  float max_missed_s{.8f};
  int confirmation_hits{3};
};

struct TrackingFrame
{
  std::uint64_t session{0}, sequence{0};
  std::int64_t timestamp_ns{0};
  bool allow_skipped_frames{false};
};

struct ObjectTrack
{
  algorithms::Detection detection;
  std::uint64_t id{0};
  Vec3 velocity{};
  bool confirmed{false}, predicted{false};
  bool low_confidence_update{false};
  float last_strong_confidence{0};
  float missed_s{0};
  std::optional<std::size_t> measurement_index;
  std::vector<Vec3> trail;
};

struct TrackingStats
{
  std::size_t confirmed{0}, tentative{0}, coasting{0}, capacity_dropped{0};
  double processing_ms{0};
  std::string reset_reason;
};

// Sensor-time 3D constant-velocity Kalman filtering and confidence-staged matching.
// No rendering, wall-clock aging, inference process or unbounded history.
class ObjectTracker
{
public:
  static constexpr std::size_t kMaxTracks=256, kMaxMeasurements=256, kTrailLength=32;
  ObjectTracker();
  ~ObjectTracker();
  ObjectTracker(const ObjectTracker &)=delete;
  ObjectTracker & operator=(const ObjectTracker &)=delete;
  const std::vector<ObjectTrack> & Update(const std::vector<algorithms::Detection> & detections,
    const TrackingFrame & frame,const TrackingParams & params,float min_confidence);
  // Clear active tracks while keeping IDs monotonic within the recording.
  void Reset();
  // Start a new recording with a fresh ID namespace.
  void ResetSequence();
  const std::vector<ObjectTrack> & Tracks() const;
  const TrackingStats & Stats() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lidar_viewer
