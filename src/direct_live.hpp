#pragma once

#include "lidar_viewer/live_io.hpp"

namespace lidar_viewer
{
class DirectLiveSession
{
public:
  explicit DirectLiveSession(const DirectLiveOptions & options);
  ~DirectLiveSession();
  void Disconnect();
  std::unique_ptr<LiveFrame> TakeLatest();
  LiveStats Stats() const;
  void StartRecording(const fs::path & destination);
  void StopRecording();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lidar_viewer
