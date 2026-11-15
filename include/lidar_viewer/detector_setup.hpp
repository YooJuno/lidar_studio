#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace lidar_viewer {
enum class DetectorSetupStage {Idle, Working, Ready, Failed, Canceled};
struct DetectorSetupProgress {
  DetectorSetupStage stage{DetectorSetupStage::Idle};
  std::string message, error, log;
  std::uint64_t bytes{0}, total_bytes{0};
  bool Active() const {return stage==DetectorSetupStage::Working;}
};
// Downloads and helper processes are owned by one background thread. Config is
// published only by a successful installer; cancellation preserves prior models.
class DetectorSetup {
public:
  explicit DetectorSetup(std::filesystem::path prepared_python={});~DetectorSetup();
  DetectorSetup(const DetectorSetup &)=delete;
  DetectorSetup & operator=(const DetectorSetup &)=delete;
  bool Start(const std::filesystem::path & python_root,const std::filesystem::path & setup_script);
  void Cancel();
  DetectorSetupProgress Progress() const;
  static std::filesystem::path DataDirectory();
private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace lidar_viewer
