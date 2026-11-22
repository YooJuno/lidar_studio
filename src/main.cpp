#include "lidar_viewer/app.hpp"
#include "lidar_viewer/perception_benchmark.hpp"
#include <string_view>
#ifdef _WIN32
#include <filesystem>
#include <memory>
#include <windows.h>
#include <shellapi.h>
#endif

int main(int argc, char ** argv)
{
  if (argc > 1 && std::string_view(argv[1]) == "--benchmark-backend") {
#ifdef _WIN32
    int argc_wide = 0;
    const std::unique_ptr<wchar_t *, decltype(&LocalFree)> wide(
      CommandLineToArgvW(GetCommandLineW(), &argc_wide), &LocalFree);
    if (!wide) return 1;
    std::vector<std::string> arguments;
    for (int i = 2; i < argc_wide; ++i)
      arguments.push_back(std::filesystem::path(wide.get()[i]).u8string());
    return lidar_viewer::RunPerceptionBenchmark(arguments);
#else
    return lidar_viewer::RunPerceptionBenchmark(std::vector<std::string>(argv + 2, argv + argc));
#endif
  }
  return RunViewer(argc, argv);
}
