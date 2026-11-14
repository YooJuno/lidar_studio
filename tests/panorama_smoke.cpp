#include "lidar_viewer/panorama.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>

using namespace lidar_viewer;
void Check(bool ok) {if(!ok) throw std::runtime_error("Panorama geometry regression");}
bool Near(float a,float b) {return std::abs(a-b)<1e-3f;}
int main()
{
  Check(Near(SensorAngles({10,0,0})->bearing,0));
  Check(Near(SensorAngles({0,10,0})->bearing,-90));
  Check(Near(SensorAngles({0,-10,0})->bearing,90));
  Check(Near(std::abs(SensorAngles({-10,0,0})->bearing),180));
  Check(Near(SensorAngles({10,0,10})->elevation,45));
  Check(Near(SensorAngles({10,0,-10})->elevation,-45));
  Check(!SensorAngles({0,0,0}) && !SensorAngles({.001f,0,0}));
  Check(!SensorAngles({std::numeric_limits<float>::quiet_NaN(),0,1}));
  Check(!SensorAngles({0,std::numeric_limits<float>::infinity(),1}));
  // Nonuniform channel elevations and a dataset floor translation must be
  // handled without guessing a channel count, order, or uniform ring spacing.
  std::vector<KittiPoint> scan;
  for(float elevation:{-25.f,-11.7f,-4.5f,3.f}) {
    const float a=elevation*kPi/180;
    scan.push_back({20*std::cos(a),0,20*std::sin(a)-2.3f,.5f});
  }
  PanoramaView view;FitPanorama(view,scan,2.3f);
  Check(view.initialized && Near(view.horizontal_fov,360));
  Check(Near(view.elevation_deg,-11) && Near(view.vertical_fov,32));
  const float x=.4f,y=-.2f;
  const float bearing=view.center_deg+x*view.horizontal_fov*.5f;
  const float elevation=view.elevation_deg+y*view.vertical_fov*.5f;
  ZoomPanorama(view,.5f,x,y);
  Check(Near(WrapDegrees(bearing-view.center_deg),x*view.horizontal_fov*.5f));
  Check(Near(elevation-view.elevation_deg,y*view.vertical_fov*.5f));
  PanPanorama(view,3,0);Check(std::abs(view.center_deg)<=180);
  Check(Near(WrapDegrees(-179-179),2)); // Zoom and pan across the rear seam.
  Check(Near(WrapDegrees(179-(-179)),-2));
  for(int i=0;i<100;++i) ZoomPanorama(view,.1f,0,0);
  Check(view.horizontal_fov==2 && view.vertical_fov==2);
  for(int i=0;i<100;++i) ZoomPanorama(view,10,0,0);
  Check(view.horizontal_fov==360 && view.vertical_fov==180 && view.elevation_deg==0);
  PanoramaView empty;FitPanorama(empty,{});Check(!empty.initialized);
  std::cout<<"Panorama bearings, elevation, invalid returns, fit, seam and anchored zoom passed\n";
}
