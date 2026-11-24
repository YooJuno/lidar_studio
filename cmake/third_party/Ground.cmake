# Build only the pinned segmentation cores. Studio-specific adapters and
# generated headers leave the source checkouts under third_party/ unchanged.
include("${CMAKE_CURRENT_LIST_DIR}/FugSeg.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/GSeg3D.cmake")

set(linefit_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/linefit_ground_segmentation")
add_library(lidar_viewer_linefit STATIC
  "${linefit_root}/src/bin.cc"
  "${linefit_root}/src/segment.cc")
target_include_directories(lidar_viewer_linefit SYSTEM PUBLIC "${linefit_root}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_viewer_linefit PUBLIC Eigen3::Eigen)

set(patchwork_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/patchwork_plusplus/cpp")
if(NOT EXISTS "${patchwork_root}/patchworkpp/src/patchworkpp.cpp")
  message(FATAL_ERROR "Missing Patchwork++ submodule. Run: git submodule update --init --recursive")
endif()
add_library(lidar_viewer_patchworkpp STATIC
  "${patchwork_root}/patchworkpp/src/patchworkpp.cpp"
  "${patchwork_root}/patchwork/src/patchwork.cpp"
  "${patchwork_root}/common/src/plane_fit.cpp")
target_include_directories(lidar_viewer_patchworkpp SYSTEM PUBLIC
  "${patchwork_root}/patchworkpp/include"
  "${patchwork_root}/patchwork/include"
  "${patchwork_root}/common/include")
target_link_libraries(lidar_viewer_patchworkpp PUBLIC Eigen3::Eigen)
