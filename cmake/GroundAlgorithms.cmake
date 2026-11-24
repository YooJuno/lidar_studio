include("${CMAKE_CURRENT_LIST_DIR}/third_party/Ground.cmake")

# Studio adapters are the only consumers of the original library headers.
add_library(lidar_viewer_ground_adapters STATIC
  src/algorithms/ground/upstream/fugseg_adapter.cpp
  src/algorithms/ground/upstream/gseg3d_adapter.cpp
  src/algorithms/ground/upstream/linefit_adapter.cpp
  src/algorithms/ground/upstream/patchwork_adapter.cpp)
target_include_directories(lidar_viewer_ground_adapters PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(lidar_viewer_ground_adapters PRIVATE
  lidar_viewer_fugseg lidar_viewer_gseg3d lidar_viewer_linefit lidar_viewer_patchworkpp)
lidar_warnings(lidar_viewer_ground_adapters)

add_library(lidar_viewer_geometry STATIC
  src/algorithms/ground/dispatch.cpp
  src/algorithms/ground/builtin/fast_grid.cpp
  src/algorithms/ground/builtin/grid.cpp
  src/algorithms/ground/builtin/plane_fit.cpp
  src/algorithms/ground/builtin/ransac.cpp
  src/algorithms/clustering.cpp
  ${LIDAR_VIEWER_CUDA_SOURCES})
target_include_directories(lidar_viewer_geometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(lidar_viewer_geometry PRIVATE lidar_viewer_ground_adapters)
lidar_warnings(lidar_viewer_geometry)
