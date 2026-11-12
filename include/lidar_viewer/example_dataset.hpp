#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace lidar_viewer {
struct ExampleFile {const char * name;std::uint64_t bytes;const char * sha256;};
const std::array<ExampleFile,6> & DrivingExampleFiles();
std::filesystem::path DrivingExampleDirectory();
bool IsDrivingExample(const std::filesystem::path & path);
enum class ExampleStage {Idle,Checking,Downloading,Ready,Canceled,Failed};
struct ExampleProgress {
  ExampleStage stage{ExampleStage::Idle};
  std::uint64_t bytes{0},total_bytes{0};
  std::size_t files{0},cached{0};
  std::string filename,error;
  bool Active() const {return stage==ExampleStage::Checking || stage==ExampleStage::Downloading;}
};
// Optional transport also permits deterministic cache/cancellation tests without a network.
using ExampleTransfer=std::function<void(const ExampleFile &,const std::filesystem::path &,
  const std::function<bool(std::uint64_t)> &)>;
class ExampleDataset {
public:
  explicit ExampleDataset(std::filesystem::path directory=DrivingExampleDirectory(),ExampleTransfer transfer={});
  ~ExampleDataset();
  ExampleDataset(const ExampleDataset &)=delete;
  ExampleDataset & operator=(const ExampleDataset &)=delete;
  bool Start();
  void Cancel();
  ExampleProgress Progress() const;
  std::filesystem::path Directory() const;
private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace lidar_viewer
