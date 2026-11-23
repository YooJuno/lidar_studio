#pragma once

#include "lidar_viewer/example_dataset.hpp"

namespace lidar_viewer {

// The caller owns an empty staging directory and verifies the archive and
// unpacked member checksums before making the staging files available.
void UnpackExampleArchive(const std::filesystem::path & zip, const std::filesystem::path & stage,
  const ExampleFile * members, std::size_t count, const std::function<bool()> & keep_running);

} // namespace lidar_viewer
