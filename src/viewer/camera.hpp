#pragma once

#include "lidar_viewer/data_io.hpp"
#include "lidar_viewer/panorama.hpp"

#include <array>

namespace lidar_viewer::viewer
{
struct CameraState
{
  enum class ProjectionMode
  {
    View3D,
    View2D,
    Panorama,
  };

  struct ViewBookmark
  {
    Vec3 target{};
    float yaw{0.0f};
    float pitch{0.0f};
    float distance{40.0f};
    bool valid{false};
  };

  Vec3 target{};
  float yaw{0.2f};
  float pitch{-0.35f};
  float distance{40.0f};
  float point_size{2.0f};
  ProjectionMode projection_mode{ProjectionMode::View3D};
  lidar_viewer::PanoramaView panorama;
  bool has_saved_view{false};
  ViewBookmark saved_3d_view{{}, 0.2f, -0.35f, 40.0f, false};
  ViewBookmark saved_2d_view{{}, 0.0f, -1.5707f, 40.0f, false};
  bool dragging_left{false};
  bool dragging_right{false};
  double last_x{0.0};
  double last_y{0.0};
};

struct Mat4
{
  std::array<float, 16> m{};
};

Vec3 Cross(const Vec3 & a, const Vec3 & b);
Vec3 Normalize(const Vec3 & v);
Mat4 Multiply(const Mat4 & a, const Mat4 & b);
Mat4 Perspective(float fov_y_rad, float aspect, float z_near, float z_far);
Mat4 Orthographic(float left, float right, float bottom, float top, float z_near, float z_far);
Mat4 LookAt(const Vec3 & eye, const Vec3 & center, const Vec3 & up);
Vec3 CameraPosition(const CameraState & camera);
void StoreCurrentProjectionView(CameraState & camera);
void ResetCameraToFrame(CameraState & camera, const FrameData & frame, const FrameData & raw, float ground_offset);
void SetProjectionMode(CameraState & camera, CameraState::ProjectionMode mode,
  const FrameData & frame, const FrameData & raw, float ground_offset);
} // namespace lidar_viewer::viewer
