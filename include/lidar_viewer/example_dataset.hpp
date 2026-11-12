#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace lidar_viewer {
// bytes / sha256 describe the original download. Six-column sources are
// normalized to XYZI; xyzi_sha256 verifies that cached representation.
struct ExampleFile {const char * name;std::uint64_t bytes;const char * sha256;const char * xyzi_sha256{nullptr};};
enum class ExampleKind {Quick,Urban30,Urban60,Velodyne30,Velodyne60,At12830,At12860};
struct ExampleDefinition {
  ExampleKind kind;const char * id;const char * title;const char * source;
  const char * base_url;const ExampleFile * files;std::size_t count;
  const char * sensor;unsigned channels;const char * profile;
  std::uint64_t TotalBytes() const;
};
const std::array<ExampleDefinition,7> & DrivingExamples();
const ExampleDefinition & DrivingExample(ExampleKind kind);
const std::array<ExampleFile,6> & DrivingExampleFiles();
std::filesystem::path DrivingExampleDirectory(ExampleKind kind=ExampleKind::Quick);
bool IsDrivingExample(const std::filesystem::path & path);
struct ExampleCacheInfo {std::size_t files{0};std::uint64_t bytes{0};};
// Metadata only; full SHA-256 verification is performed before opening.
ExampleCacheInfo InspectExampleCache(ExampleKind kind);
// Deletes only this app-owned cache selection. Does not follow directory symlinks.
void DeleteExampleCache(ExampleKind kind);
enum class ExampleStage {Idle,Checking,Downloading,Ready,Canceled,Failed};
struct ExampleProgress {
  ExampleStage stage{ExampleStage::Idle};
  std::uint64_t bytes{0},total_bytes{0};
  std::size_t files{0},cached{0},total_files{0};
  double eta_seconds{-1};
  std::string filename,error;
  bool Active() const {return stage==ExampleStage::Checking || stage==ExampleStage::Downloading;}
};
// Optional transport also permits deterministic cache/cancellation tests without a network.
using ExampleTransfer=std::function<void(const ExampleFile &,const std::filesystem::path &,
  const std::function<bool(std::uint64_t)> &)>;
class ExampleDataset {
public:
  explicit ExampleDataset(std::filesystem::path directory=DrivingExampleDirectory(),ExampleTransfer transfer={});
  explicit ExampleDataset(ExampleKind kind,std::filesystem::path directory={},ExampleTransfer transfer={});
  ~ExampleDataset();
  ExampleDataset(const ExampleDataset &)=delete;
  ExampleDataset & operator=(const ExampleDataset &)=delete;
  bool Start();
  void Cancel();
  ExampleProgress Progress() const;
  std::filesystem::path Directory() const;
  const ExampleDefinition & Definition() const;
private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace lidar_viewer
