#include "camera.hpp"

#include "lidar_viewer/camera_2d.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_viewer::viewer
{
static float Dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Cross(const Vec3 & a, const Vec3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

static float Length(const Vec3 & v)
{
  return std::sqrt(Dot(v, v));
}

Vec3 Normalize(const Vec3 & v)
{
  const float len = Length(v);
  if (len < 1e-6f) {
    return {0.0f, 0.0f, 0.0f};
  }
  return v * (1.0f / len);
}

static Mat4 Identity()
{
  return Mat4{{{
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  }}};
}

Mat4 Multiply(const Mat4 & a, const Mat4 & b)
{
  Mat4 out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      float value = 0.0f;
      for (int k = 0; k < 4; ++k) {
        value += a.m[k * 4 + row] * b.m[col * 4 + k];
      }
      out.m[col * 4 + row] = value;
    }
  }
  return out;
}

Mat4 Perspective(float fov_y_rad, float aspect, float z_near, float z_far)
{
  const float f = 1.0f / std::tan(fov_y_rad * 0.5f);
  return Mat4{{{
    f / aspect, 0.0f, 0.0f, 0.0f,
    0.0f, f, 0.0f, 0.0f,
    0.0f, 0.0f, (z_far + z_near) / (z_near - z_far), -1.0f,
    0.0f, 0.0f, (2.0f * z_far * z_near) / (z_near - z_far), 0.0f
  }}};
}

Mat4 Orthographic(float left, float right, float bottom, float top, float z_near, float z_far)
{
  Mat4 out = Identity();
  out.m[0] = 2.0f / (right - left);
  out.m[5] = 2.0f / (top - bottom);
  out.m[10] = -2.0f / (z_far - z_near);
  out.m[12] = -(right + left) / (right - left);
  out.m[13] = -(top + bottom) / (top - bottom);
  out.m[14] = -(z_far + z_near) / (z_far - z_near);
  return out;
}

Mat4 LookAt(const Vec3 & eye, const Vec3 & center, const Vec3 & up)
{
  const Vec3 forward = Normalize(center - eye);
  const Vec3 right = Normalize(Cross(forward, up));
  const Vec3 camera_up = Cross(right, forward);

  Mat4 out = Identity();
  out.m[0] = right.x;
  out.m[1] = camera_up.x;
  out.m[2] = -forward.x;
  out.m[4] = right.y;
  out.m[5] = camera_up.y;
  out.m[6] = -forward.y;
  out.m[8] = right.z;
  out.m[9] = camera_up.z;
  out.m[10] = -forward.z;
  out.m[12] = -Dot(right, eye);
  out.m[13] = -Dot(camera_up, eye);
  out.m[14] = Dot(forward, eye);
  return out;
}

Vec3 CameraPosition(const CameraState & camera)
{
  const float cp = std::cos(camera.pitch);
  return {
    camera.target.x + camera.distance * cp * std::sin(camera.yaw),
    camera.target.y + camera.distance * cp * std::cos(camera.yaw),
    camera.target.z + camera.distance * std::sin(camera.pitch)
  };
}

void StoreCurrentProjectionView(CameraState & camera)
{
  if(camera.projection_mode==CameraState::ProjectionMode::Panorama) return;
  auto & bookmark =
    (camera.projection_mode == CameraState::ProjectionMode::View2D)
    ? camera.saved_2d_view
    : camera.saved_3d_view;
  bookmark.target = camera.target;
  bookmark.yaw = camera.yaw;
  bookmark.pitch = camera.pitch;
  bookmark.distance = camera.distance;
  bookmark.valid = true;
}

void ResetCameraToFrame(CameraState & camera, const FrameData & frame, const FrameData & raw, float ground_offset)
{
  if(camera.projection_mode==CameraState::ProjectionMode::Panorama) {
    lidar_viewer::FitPanorama(camera.panorama,raw.points,ground_offset);return;
  }
  camera.target = frame.center;
  if (camera.projection_mode == CameraState::ProjectionMode::View2D) {
    camera.target.z = 0.0f;
    camera.distance = std::max(frame.radius * 1.15f, 12.0f);
    camera.pitch = -1.5707f;
    camera.yaw = 0.0f;
  } else {
    camera.distance = frame.radius * 2.2f;
    camera.pitch = 0.55f;
    camera.yaw = -0.8f;
  }
  StoreCurrentProjectionView(camera);
}

void SetProjectionMode(CameraState & camera, CameraState::ProjectionMode mode,
  const FrameData & frame, const FrameData & raw, float ground_offset)
{
  if (camera.projection_mode == mode) {
    return;
  }

  StoreCurrentProjectionView(camera);
  camera.projection_mode = mode;
  if(mode==CameraState::ProjectionMode::Panorama) {
    if(!camera.panorama.initialized) ResetCameraToFrame(camera, frame, raw, ground_offset);
    return;
  }
  const auto & bookmark =
    (mode == CameraState::ProjectionMode::View2D)
    ? camera.saved_2d_view
    : camera.saved_3d_view;

  if (bookmark.valid) {
    camera.target = bookmark.target;
    camera.yaw = bookmark.yaw;
    camera.pitch = bookmark.pitch;
    camera.distance = mode==CameraState::ProjectionMode::View2D ?
      lidar_viewer::TopViewHalfHeight(bookmark.distance) : std::max(bookmark.distance,1.f);
  } else if (mode == CameraState::ProjectionMode::View2D) {
    camera.target.z = 0.0f;
    camera.distance = lidar_viewer::TopViewHalfHeight(camera.distance);
    camera.pitch = -1.5707f;
    camera.yaw = 0.0f;
  } else {
    camera.distance = std::max(camera.distance, 2.0f);
    camera.pitch = 0.55f;
    camera.yaw = -0.8f;
  }
  StoreCurrentProjectionView(camera);
}
} // namespace lidar_viewer::viewer
