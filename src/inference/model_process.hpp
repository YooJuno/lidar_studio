#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <vector>
#ifndef _WIN32
#include <optional>
#include <sys/types.h>
#endif

namespace lidar_viewer::inference
{
// Synchronous binary pipes to one owned model child. The caller serializes
// open/kill/close, owns the wire protocol and controls SIGPIPE on POSIX.
class ModelProcess
{
public:
  ModelProcess() = default;
  ~ModelProcess();
  ModelProcess(const ModelProcess &) = delete;
  ModelProcess & operator=(const ModelProcess &) = delete;

  void Open(const std::filesystem::path & python,
    const std::vector<std::filesystem::path> & arguments, const std::filesystem::path & log);
  void Kill();
  void Close();
  void Read(void * data, std::size_t size);
  void Write(const void * data, std::size_t size);
#ifndef _WIN32
  void SetDeadline(std::chrono::seconds timeout);
#endif

private:
  void Transfer(void * data, std::size_t size, bool writing);

  std::filesystem::path log_path_;
#ifdef _WIN32
  void * process_{nullptr};
  void * read_{nullptr};
  void * write_{nullptr};
#else
  pid_t pid_{-1};
  int read_{-1};
  int write_{-1};
  std::optional<std::chrono::steady_clock::time_point> deadline_;
#endif
};
} // namespace lidar_viewer::inference
