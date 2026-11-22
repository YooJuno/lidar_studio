#include "lidar_viewer/perception_benchmark.hpp"

#include <filesystem>

#ifdef _WIN32
int wmain(int argc, wchar_t ** argv)
{
  std::vector<std::string> arguments;
  for (int i = 1; i < argc; ++i) arguments.push_back(std::filesystem::path(argv[i]).u8string());
  return lidar_viewer::RunPerceptionBenchmark(arguments);
}
#else
int main(int argc, char ** argv)
{
  return lidar_viewer::RunPerceptionBenchmark(std::vector<std::string>(argv + 1, argv + argc));
}
#endif
