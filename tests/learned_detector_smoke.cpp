#include "lidar_viewer/learned_detector.hpp"
#include "lidar_viewer/terraseg.hpp"
#include "lidar_viewer/platform.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

using namespace lidar_viewer;
namespace {
void Check(bool condition,const char * message) {if (!condition) throw std::runtime_error(message);}
void ConfigEnvironment(const std::string & value) {
#ifdef _WIN32
  _putenv_s("XDG_CONFIG_HOME",value.c_str());
#else
  if (value.empty()) unsetenv("XDG_CONFIG_HOME");else setenv("XDG_CONFIG_HOME",value.c_str(),1);
#endif
}
template<class F> void Wait(F ready) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
  while (!ready()) {Check(std::chrono::steady_clock::now()<deadline,"Detector test timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
}
}
int main(int argc,char ** argv) {
  const std::string previous=std::getenv("XDG_CONFIG_HOME") ? std::getenv("XDG_CONFIG_HOME") : "";
  std::filesystem::path temporary;
  try {
    Check(argc==2,"Pass Python interpreter");temporary=TemporaryDirectory("lidar-detector-test-")/std::filesystem::u8path("runtime space 한글");
    std::filesystem::create_directories(temporary);ConfigEnvironment(temporary.u8string());
    const auto folder=UserConfigurationDirectory();std::filesystem::create_directories(folder);
    const auto worker=temporary/"detector_worker.py";
    std::ofstream(worker)<<R"PY(import struct,sys,time
source=sys.stdin.buffer;sink=sys.stdout.buffer
req=struct.Struct('<4sQIf');reply=struct.Struct('<4sQIfI')
def exact(n):
 b=b''
 while len(b)<n:
  c=source.read(n-len(b))
  if not c:raise SystemExit()
  b+=c
 return b
def send(b):
 for i in range(0,len(b),3):sink.write(b[i:i+3]);sink.flush()
send(reply.pack(b'LDR1',0,0,0,4)+b'CPU!')
while True:
 tag,revision,count,score=req.unpack(exact(req.size))
 transfusion='transfusion' in sys.argv
 if tag!=(b'LDQ2' if transfusion else b'LDQ1'):raise SystemExit(2)
 data=exact(count*(16 if transfusion else 12))
 if transfusion and struct.unpack_from('<f',data,12)[0]!=83:raise SystemExit(4)
 x,y,z=struct.unpack_from('<fff',data)
 if x < -900:raise SystemExit(3)
 time.sleep(.04)
 box=struct.pack('<9f',x,y,z,4,2,1.5,.25,.8,0)
 send(reply.pack(b'LDR1',revision,1,40,2)+b'ok'+box)
)PY";
#ifndef _WIN32
    std::ifstream protocol_source(worker);
    const std::string protocol((std::istreambuf_iterator<char>(protocol_source)),std::istreambuf_iterator<char>());
#endif
    std::ofstream(folder/"detector.cfg")<<"python="<<argv[1]<<"\nmodel_dir="<<temporary.u8string()<<
      "\ntransfusion_model_dir="<<temporary.u8string()<<"\nprovider=cpu\n";
    {
      LearnedDetector detector;detector.Start(temporary);
      Wait([&]{auto status=detector.Status();Check(!status.failed,status.message.c_str());return status.ready;});
      detector.Submit(42,{{3,4,1,0}},.35f,1.5707963268f);
      std::optional<DetectorResult> result;
      Wait([&]{result=detector.Poll();return result.has_value();});
      Check(result->revision==42 && result->detections.size()==1 && result->detail=="ok","Fragmented protocol reply lost data");
      const auto & box=result->detections[0].oriented_bounds;
      Check(std::abs(box.center.x-3)<1e-4 && std::abs(box.center.y-4)<1e-4,"Display yaw / model heading round trip is wrong");
      Check(box.valid && std::abs(box.axis_x.x-std::cos(.25f+1.5707963268f))<1e-4,"Learned yaw did not reach the oriented box");
      for (std::uint64_t revision=43;revision<=100;++revision) detector.Submit(revision,{{1,2,3,0}},.35f,0);
      Wait([&]{if (auto next=detector.Poll()) return next->revision==100;return false;});
      detector.Submit(101,{{-999,0,0,0}},.35f,0);
      Wait([&]{return detector.Status().failed;});
      detector.Stop();
      detector.Start(temporary,"transfusion");
      Wait([&]{const auto status=detector.Status();Check(!status.failed,status.message.c_str());return status.ready;});
      detector.Submit(102,{{1,2,3,83}},.25f,0);
      Wait([&]{const auto status=detector.Status();Check(!status.failed,status.message.c_str());
        result=detector.Poll();return result.has_value();});
      Check(result->revision==102 && result->detections.size()==1,"TransFusion did not receive original intensity");
      detector.Stop();
      // Stopping during a blocked model startup must not hang application exit.
      std::ofstream(worker)<<"import time\ntime.sleep(60)\n";
      detector.Start(temporary);std::this_thread::sleep_for(std::chrono::milliseconds(60));
      const auto start=std::chrono::steady_clock::now();detector.Stop();
      Check(std::chrono::steady_clock::now()-start<std::chrono::seconds(2),"Canceling model startup blocked UI shutdown");
    }
    const float invalid[]={0,0,0,-1,1,1,0,.9f,0};bool rejected=false;
    try {algorithms::DecodeCenterPointBox(invalid,0);} catch (...) {rejected=true;}
    Check(rejected,"Invalid neural box dimensions accepted");
    algorithms::DetectionParams filters;
    algorithms::Detection car,person,bicycle;car.class_id=0;person.class_id=3;bicycle.class_id=4;
    car.confidence=person.confidence=bicycle.confidence=.8f;
    Check(filters.Accepts(car) && filters.Accepts(person) && filters.Accepts(bicycle),"Default targets omit a supported class");
    filters.classes=1u<<3;
    Check(filters.Accepts(person) && !filters.Accepts(car) && !filters.Accepts(bicycle),"Person selection leaks other classes");
    filters.min_confidence=.9f;Check(!filters.Accepts(person),"Class selection bypasses the score filter");
    filters.classes=0;filters.min_confidence=.1f;Check(!filters.Accepts(person),"No targets still displays a class");
    std::filesystem::remove(folder/"detector.cfg");
    {LearnedDetector missing;missing.Start(temporary);Check(!missing.Status().configured,"Missing runtime was presented as working detection");}
#ifndef _WIN32
    const auto bundle=temporary/std::filesystem::u8path("LiDAR Studio.app/Contents/Resources");
    const auto scripts=bundle/"python";
    std::filesystem::create_directories(scripts);
    std::ofstream(scripts/"detector_worker.py")<<protocol;
    const auto runtime=bundle/"detector/runtime/bin";
    std::filesystem::create_directories(runtime);
    std::filesystem::create_symlink(std::filesystem::absolute(argv[1]),runtime/"python3");
    for(const auto & variant:{std::string("tiny"),std::string("short_range")}) {
      std::filesystem::create_directories(bundle/"detector/models"/(variant=="tiny" ? "centerpoint-tiny-v4.1" : "centerpoint-shortrange-v2.0.0"));
      LearnedDetector bundled;bundled.Start(scripts,variant);
      Wait([&]{const auto status=bundled.Status();Check(!status.failed,status.message.c_str());return status.ready;});
      bundled.Submit(301,{{1,2,3,0}},.35f,0);
      std::optional<DetectorResult> result;
      Wait([&]{result=bundled.Poll();return result.has_value();});
      Check(result->detections.size()==1,"Bundled runtime did not return detections");
      Check(!std::filesystem::exists(folder/"detector.cfg"),"Bundled runtime required a user setup file");
    }
    std::ofstream(folder/"detector.cfg")<<"python=/missing/python\nmodel_dir="<<temporary.u8string()<<'\n';
    {LearnedDetector explicit_config;explicit_config.Start(scripts);Check(!explicit_config.Status().configured,"Bundled fallback silently replaced an explicit user configuration");}
    const auto optional_base=temporary/"optional-base";
    std::filesystem::create_directories(optional_base);
    std::ofstream(folder/"detector.cfg")<<"python="<<(runtime/"python3").u8string()<<
      "\nmodel_dir="<<(temporary/"uninstalled-tiny").u8string()<<"\nbase_model_dir="<<optional_base.u8string()<<'\n';
    for (const auto * variant : {"base", "tiny", "short_range"}) {
      LearnedDetector detector;detector.Start(scripts,variant);
      Wait([&]{const auto status=detector.Status();Check(!status.failed,status.message.c_str());return status.ready;});
    }
#endif
#if defined(__linux__) && defined(__x86_64__)
    const auto checkpoint=temporary/"test-terraseg.pth";
    std::ofstream(checkpoint)<<"protocol fixture";
    std::ofstream(folder/"terraseg.cfg")<<"python="<<argv[1]<<"\ncheckpoint="<<checkpoint.u8string()<<'\n';
    std::ofstream(temporary/"terraseg_worker.py")<<R"PY(import struct,sys
src=sys.stdin.buffer
dst=sys.stdout.buffer
result=struct.Struct('<4sQII')
request=struct.Struct('<4sQI')
dst.write(result.pack(b'TGR1',0,0,0))
dst.flush()
while header:=src.read(request.size):
    tag,revision,count=request.unpack(header)
    if tag!=b'TGQ1':raise SystemExit(2)
    scan=struct.iter_unpack('<fff',src.read(count*12))
    mask=bytes(int(x>0 and abs(z)<.1) for x,y,z in scan)
    dst.write(result.pack(b'TGR1',revision,len(mask),0)+mask)
    dst.flush()
)PY";
    {
      TerraSeg ground(temporary);
      const std::vector<KittiPoint> points{{0,2,.5f,83},{0,3,1.5f,17},{0,1,.5f,0}};
      for (int i=0;i<2;++i) {
        const auto partitions=ground.Segment(points,.5f,1.5707963268f,{0,1,.5f});
        Check(partitions && partitions->ground_points.size()==1 && partitions->ground_points[0].intensity==83 &&
          partitions->non_ground_points.size()==2 && partitions->non_ground_points.back().intensity==0,
          "TerraSeg did not preserve source point order, missing returns or ground reference");
      }
      ground.Cancel();
      Check(!ground.Segment(points,0,0,{}),"Canceled TerraSeg accepted another scan");
    }
#endif
    ConfigEnvironment(previous);std::filesystem::remove_all(temporary.parent_path());
    std::cout<<"learned_detector_smoke: protocol, latest request, yaw, process failure and cancellation passed\n";return 0;
  } catch (const std::exception & error) {
    ConfigEnvironment(previous);if (!temporary.empty()) std::filesystem::remove_all(temporary.parent_path());
    std::cerr<<error.what()<<'\n';return 1;
  }
}
