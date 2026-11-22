#include "lidar_viewer/learned_detector.hpp"
#include "lidar_viewer/platform.hpp"
#include "model_process.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#ifndef _WIN32
#include <csignal>
#include <pthread.h>
#endif

namespace lidar_viewer
{
namespace
{
using inference::ModelProcess;

// All supported release architectures are little endian. Encode fields
// individually: the wire protocol contains no C++ struct padding.
static_assert(sizeof(float)==4,"Detector protocol requires float32");

struct Request {
  std::uint64_t revision{0};float threshold{.35f},yaw{0};
  std::shared_ptr<const std::vector<KittiPoint>> scan;
};
DetectorResult ReadReply(ModelProcess & child,float yaw) {
  std::array<char,4> magic{};DetectorResult result;std::uint32_t count=0,length=0;
  std::array<char,24> header{};child.Read(header.data(),header.size());
  std::memcpy(magic.data(),header.data(),4);std::memcpy(&result.revision,header.data()+4,8);
  std::memcpy(&count,header.data()+12,4);std::memcpy(&result.latency_ms,header.data()+16,4);std::memcpy(&length,header.data()+20,4);
  const std::string tag(magic.data(),4);
  if ((tag!="LDR1" && tag!="LDE1") || count>4096 || length>16384)
    throw std::runtime_error("Invalid detector reply");
  result.detail.resize(length);child.Read(result.detail.data(),length);
  if (tag=="LDE1") {result.error=result.detail;if (count) throw std::runtime_error("Invalid detector error reply");}
  result.detections.reserve(count);
  // Read the contiguous wire payload once instead of issuing one pipe read per box.
  static thread_local std::vector<std::array<float,9>> boxes;
  boxes.resize(count);child.Read(boxes.data(),static_cast<std::size_t>(count)*36);
  for (const auto & box:boxes)
    result.detections.push_back(algorithms::DecodeCenterPointBox(box.data(),yaw));
  return result;
}

} // namespace

struct LearnedDetector::Impl {
  mutable std::mutex mutex;std::mutex process_mutex;std::condition_variable cv;
  ModelProcess child;std::thread thread;bool stop{false};
  DetectorStatus status;std::optional<Request> pending;std::optional<DetectorResult> result;
};
LearnedDetector::LearnedDetector():impl_(std::make_unique<Impl>()) {}
LearnedDetector::~LearnedDetector() {Stop();}
void LearnedDetector::Stop() {
  {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->stop=true;impl_->pending.reset();}
  impl_->cv.notify_all();
  {std::lock_guard<std::mutex> lock(impl_->process_mutex);impl_->child.Kill();}
  if (impl_->thread.joinable()) impl_->thread.join();
  {std::lock_guard<std::mutex> lock(impl_->process_mutex);impl_->child.Close();}
}
void LearnedDetector::Start(const std::filesystem::path & root,const std::string & variant) {
  if (!algorithms::IsDetectorModel(variant)) throw std::invalid_argument("Unknown detector model");
  Stop();
  const auto config=UserConfigurationDirectory()/"detector.cfg";
  std::filesystem::path python,model,short_model,base_model,transfusion_model;std::string provider="auto",line;bool managed=false;
  std::ifstream file(config);
  while (std::getline(file,line)) {
    if (!line.empty() && line.back()=='\r') line.pop_back();
    const auto equal=line.find('=');if (equal==std::string::npos) continue;
    const auto value=line.substr(equal+1),key=line.substr(0,equal);
    if (key=="python") python=std::filesystem::u8path(value);
    else if (key=="model_dir") model=std::filesystem::u8path(value);
    else if (key=="short_range_model_dir") short_model=std::filesystem::u8path(value);
    else if (key=="base_model_dir") base_model=std::filesystem::u8path(value);
    else if (key=="transfusion_model_dir") transfusion_model=std::filesystem::u8path(value);
    else if (key=="provider" && value=="cpu") provider=value;
    else if (key=="managed_runtime") managed=value=="1";
  }
  const auto model_root = model.empty() ? std::filesystem::path{} : model.parent_path();
  if (variant=="short_range") model=!short_model.empty() ? short_model : model_root.empty() ?
    std::filesystem::path{} : model_root/"centerpoint-shortrange-v2.0.0";
  if (variant=="base") model=!base_model.empty() ? base_model : model_root.empty() ?
    std::filesystem::path{} : model_root/"centerpoint-base-v4.1";
  if (variant=="transfusion") model=transfusion_model;
  // Packaged Mac builds contain an offline, relocatable CPU runtime. Explicit
  // user configurations retain priority; a new Mac needs no setup command.
  const auto assets=root.parent_path()/"detector";
  const auto runtime=assets/"runtime/bin/python3";
  bool bundled=managed || python==runtime;
  // Installing only Base with the bundled interpreter leaves the two
  // optional user model directories empty. Continue using their bundled weights.
  if ((variant=="tiny" || variant=="short_range") && ((python.empty() && model.empty()) ||
      (python==runtime && !std::filesystem::is_directory(model)))) {
    const auto weights=assets/"models"/(variant=="short_range" ? "centerpoint-shortrange-v2.0.0" : "centerpoint-tiny-v4.1");
    if(std::filesystem::is_regular_file(runtime) && std::filesystem::is_directory(weights)) {
      python=runtime;model=weights;provider="cpu";bundled=true;
    }
  }
  {std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stop=false;impl_->result.reset();impl_->status={};
    if (!std::filesystem::is_regular_file(python) || !std::filesystem::is_directory(model)) {
      impl_->status.message="Model runtime is not installed. Run detector setup.";return;
    }
    impl_->status.configured=true;impl_->status.message="Loading "+std::string(bundled ? "bundled " : "")+variant+" model...";
  }
  impl_->thread=std::thread([this,root,python,model,variant,provider,config,bundled] {
#ifndef _WIN32
    // A closed worker pipe must raise an error, never terminate the GUI.
    sigset_t blocked;sigemptyset(&blocked);sigaddset(&blocked,SIGPIPE);pthread_sigmask(SIG_BLOCK,&blocked,nullptr);
#endif
    try {
      std::filesystem::create_directories(config.parent_path());
      {std::lock_guard<std::mutex> process_lock(impl_->process_mutex);
        std::lock_guard<std::mutex> lock(impl_->mutex);if (impl_->stop) return;
        std::vector<std::filesystem::path> arguments{"-B","-u"};
        if (bundled) arguments.insert(arguments.end(),{"-E","-s"});
        arguments.insert(arguments.end(),{root/"detector_worker.py","--model-dir",model,
          "--provider",provider,"--model",variant,"--native-dir",ExecutablePath().parent_path()});
        impl_->child.Open(python,arguments,config.parent_path()/"detector.log");
      }
      auto ready=ReadReply(impl_->child,0);
      if (ready.revision || !ready.error.empty()) throw std::runtime_error(ready.error.empty() ? "Invalid detector startup reply" : ready.error);
      {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->status.ready=true;impl_->status.provider=ready.detail;impl_->status.message="Ready";}
      const bool with_intensity=variant=="transfusion";
      const std::size_t stride=with_intensity ? 4 : 3;
      std::vector<float> coordinates; // Reuse IPC storage across scans.
      while (true) {
        Request request;
        {std::unique_lock<std::mutex> lock(impl_->mutex);impl_->cv.wait(lock,[this]{return impl_->stop || impl_->pending.has_value();});
          if (impl_->stop) break;
          request=std::move(*impl_->pending);impl_->pending.reset();}
        const auto processing_started=std::chrono::steady_clock::now();
        coordinates.clear();coordinates.reserve(request.scan->size()*stride);
        const float c=std::cos(request.yaw),s=std::sin(request.yaw);
        for (const auto & point:*request.scan) {
          coordinates.push_back(c*point.x+s*point.y);coordinates.push_back(-s*point.x+c*point.y);coordinates.push_back(point.z);
          if (with_intensity) coordinates.push_back(point.intensity);
        }
        const auto count=static_cast<std::uint32_t>(request.scan->size());
        request.scan.reset();
        std::array<char,20> header{};std::memcpy(header.data(),with_intensity ? "LDQ2" : "LDQ1",4);
        std::memcpy(header.data()+4,&request.revision,8);std::memcpy(header.data()+12,&count,4);std::memcpy(header.data()+16,&request.threshold,4);
        impl_->child.Write(header.data(),header.size());
        impl_->child.Write(coordinates.data(),coordinates.size()*sizeof(float));
        auto result=ReadReply(impl_->child,request.yaw);
        result.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-processing_started).count();
        if (result.revision!=request.revision) throw std::runtime_error("Detector frame revision mismatch");
        std::lock_guard<std::mutex> lock(impl_->mutex);impl_->result=std::move(result);
      }
    } catch (const std::exception & error) {
      {std::lock_guard<std::mutex> process_lock(impl_->process_mutex);impl_->child.Close();}
      std::lock_guard<std::mutex> lock(impl_->mutex);
      if (!impl_->stop) {impl_->status.failed=true;impl_->status.ready=false;impl_->status.message=error.what();}
    }
  });
}
DetectorStatus LearnedDetector::Status() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->status;}
void LearnedDetector::Submit(std::uint64_t revision,const std::vector<KittiPoint> & scan,float threshold,float yaw) {
  Submit(revision,std::make_shared<const std::vector<KittiPoint>>(scan),threshold,yaw);
}
void LearnedDetector::Submit(std::uint64_t revision,std::shared_ptr<const std::vector<KittiPoint>> scan,float threshold,float yaw) {
  if (!scan || scan->size()>4000000) throw std::runtime_error("Detector supports at most 4 million points per scan");
  if (!std::isfinite(yaw) || !std::isfinite(threshold) || threshold<.01f || threshold>.99f)
    throw std::invalid_argument("Invalid detector threshold or yaw");
  Request request;request.revision=revision;request.threshold=threshold;request.yaw=yaw;request.scan=std::move(scan);
  {std::lock_guard<std::mutex> lock(impl_->mutex);if (impl_->stop || !impl_->status.configured || impl_->status.failed) return;impl_->pending=std::move(request);}
  impl_->cv.notify_one();
}
std::optional<DetectorResult> LearnedDetector::Poll() {
  std::lock_guard<std::mutex> lock(impl_->mutex);auto result=std::move(impl_->result);impl_->result.reset();return result;
}
} // namespace lidar_viewer
