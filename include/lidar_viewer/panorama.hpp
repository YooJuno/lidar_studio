#pragma once

#include "lidar_viewer/data_io.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

namespace lidar_viewer
{
// Spherical range view: front in the middle, sensor +Y to the left.
// Uses measured elevation, never an invented laser/ring ID. No sensor model
// or uniform vertical channel spacing is assumed.
struct PanoramaView
{
  float center_deg{0}, elevation_deg{-10}, horizontal_fov{360}, vertical_fov{40};
  float range_max{80};
  int color_mode{0}; // Distance / existing point colors.
  bool initialized{false};
};
struct PanoramaAngles {float bearing, elevation, range;};
inline float WrapDegrees(float angle) {return std::remainder(angle,360.f);}
inline std::optional<PanoramaAngles> SensorAngles(Vec3 point)
{
  if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return {};
  const float range=std::hypot(std::hypot(point.x,point.y),point.z);
  if(!std::isfinite(range) || range<=.05f) return {};
  return PanoramaAngles{-std::atan2(point.y,point.x)*180/kPi,
    std::atan2(point.z,std::hypot(point.x,point.y))*180/kPi,range};
}
inline void ClampPanorama(PanoramaView & view)
{
  view.center_deg=WrapDegrees(view.center_deg);
  view.horizontal_fov=std::clamp(view.horizontal_fov,2.f,360.f);
  view.vertical_fov=std::clamp(view.vertical_fov,2.f,180.f);
  const float limit=90-view.vertical_fov*.5f;
  view.elevation_deg=std::clamp(view.elevation_deg,-limit,limit);
}
inline void FitPanorama(PanoramaView & view,const std::vector<KittiPoint> & points,float ground_offset=0)
{
  float low=90,high=-90;
  for(const auto & p:points) if(const auto angle=SensorAngles({p.x,p.y,p.z+ground_offset})) {
    low=std::min(low,angle->elevation);high=std::max(high,angle->elevation);
  }
  if(low>high) return; // Empty clouds must not lock in a fake sensor FOV.
  view.center_deg=0;view.horizontal_fov=360;
  view.elevation_deg=(low+high)*.5f;
  view.vertical_fov=std::clamp(high-low+4.f,6.f,180.f);
  view.initialized=true;ClampPanorama(view);
}
inline void PanPanorama(PanoramaView & view,float dx_fraction,float dy_fraction)
{
  view.center_deg-=dx_fraction*view.horizontal_fov;
  view.elevation_deg+=dy_fraction*view.vertical_fov;ClampPanorama(view);
}
inline void ZoomPanorama(PanoramaView & view,float factor,float x_ndc,float y_ndc)
{
  const float h=view.horizontal_fov,v=view.vertical_fov;
  view.horizontal_fov=std::clamp(h*factor,2.f,360.f);
  view.vertical_fov=std::clamp(v*factor,2.f,180.f);
  view.center_deg+=x_ndc*(h-view.horizontal_fov)*.5f;
  view.elevation_deg+=y_ndc*(v-view.vertical_fov)*.5f;ClampPanorama(view);
}
} // namespace lidar_viewer
