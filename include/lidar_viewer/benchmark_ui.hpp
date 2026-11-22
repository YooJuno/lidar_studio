#pragma once

#include <memory>

namespace lidar_viewer
{
class BenchmarkWorkspace
{
public:
  BenchmarkWorkspace();
  ~BenchmarkWorkspace();
  void Draw(bool & open, float scale);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lidar_viewer
