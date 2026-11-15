execute_process(COMMAND git describe --always --dirty --abbrev=7
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE LIDAR_STUDIO_BUILD_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT LIDAR_STUDIO_BUILD_REVISION)
  set(LIDAR_STUDIO_BUILD_REVISION "unknown")
endif()
string(TIMESTAMP LIDAR_STUDIO_BUILD_TIME "%Y-%m-%d %H:%M UTC" UTC)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/resources/app_info.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/lidar_viewer/app_info.hpp" @ONLY)
target_include_directories(lidar_studio PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
