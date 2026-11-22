include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
FetchContent_Declare(dear_imgui
  URL https://codeload.github.com/ocornut/imgui/tar.gz/f5befd2d29e66809cd1110a152e375a7f1981f06
  URL_HASH SHA256=85f4ce357df05bcc331b587f01976f47fb55f19fadf477a7907289686bc3f4c8
)
FetchContent_MakeAvailable(dear_imgui)
add_library(lidar_viewer_imgui STATIC
  ${dear_imgui_SOURCE_DIR}/imgui.cpp
  ${dear_imgui_SOURCE_DIR}/imgui_draw.cpp
  ${dear_imgui_SOURCE_DIR}/imgui_tables.cpp
  ${dear_imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(lidar_viewer_imgui PUBLIC
  ${dear_imgui_SOURCE_DIR} ${dear_imgui_SOURCE_DIR}/backends)
target_link_libraries(lidar_viewer_imgui PUBLIC glfw ${CMAKE_DL_LIBS})
