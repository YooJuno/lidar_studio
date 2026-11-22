#include "lidar_viewer/learned_detector.hpp"
#include "lidar_viewer/terraseg.hpp"
#include "lidar_viewer/algorithms/ground_partition.hpp"
#include "lidar_viewer/platform.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>
#ifndef _WIN32
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <pthread.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
extern char ** environ;
#endif

namespace lidar_viewer
{
namespace
{
// All supported release architectures are little endian. Encode fields
// individually: the wire protocol contains no C++ struct padding.
static_assert(sizeof(float)==4,"Detector protocol requires float32");
class Child {
public:
  ~Child() {Close();}
  void Open(const std::filesystem::path & python,
    const std::vector<std::filesystem::path> & arguments,const std::filesystem::path & log) {
    log_path_=log;
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{sizeof(SECURITY_ATTRIBUTES),nullptr,TRUE};
    HANDLE child_in=nullptr,child_out=nullptr;
    if (!CreatePipe(&child_in,&write_,&sa,0)) throw std::runtime_error("Cannot create detector pipes");
    if (!CreatePipe(&read_,&child_out,&sa,0)) {
      CloseHandle(child_in);throw std::runtime_error("Cannot create detector pipes");
    }
    SetHandleInformation(write_,HANDLE_FLAG_INHERIT,0);SetHandleInformation(read_,HANDLE_FLAG_INHERIT,0);
    HANDLE error=CreateFileW(log.c_str(),GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,&sa,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    auto quote=[](const std::wstring & value) {
      std::wstring result=L"\"";std::size_t slashes=0;
      for (wchar_t c:value) {
        if (c==L'\\') {++slashes;continue;}
        result.append(c==L'"' ? slashes*2+1 : slashes,L'\\');slashes=0;result+=c;
      }
      result.append(slashes*2,L'\\');return result+L'"';
    };
    std::wstring command=quote(python.wstring());
    for (const auto & argument : arguments) command+=L" "+quote(argument.wstring());
    STARTUPINFOW start{};start.cb=sizeof(start);start.dwFlags=STARTF_USESTDHANDLES;
    start.hStdInput=child_in;start.hStdOutput=child_out;start.hStdError=error;
    PROCESS_INFORMATION process{};
    const bool ok=CreateProcessW(python.c_str(),command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&start,&process)!=0;
    CloseHandle(child_in);CloseHandle(child_out);if (error!=INVALID_HANDLE_VALUE) CloseHandle(error);
    if (!ok) throw std::runtime_error("Cannot start detector Python runtime");
    process_=process.hProcess;CloseHandle(process.hThread);
#else
    int input[2],output[2];
    const auto open_pipe=[](int descriptors[2]) {
#ifdef __linux__
      return pipe2(descriptors,O_CLOEXEC);
#else
      const int result=pipe(descriptors);
      if (result==0) for (int i=0;i<2;++i) fcntl(descriptors[i],F_SETFD,FD_CLOEXEC);
      return result;
#endif
    };
    if (open_pipe(input)!=0) throw std::runtime_error("Cannot create model input pipe");
    if (open_pipe(output)!=0) {close(input[0]);close(input[1]);throw std::runtime_error("Cannot create model output pipe");}
    const int error=open(log.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0600);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions,input[0],STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions,output[1],STDOUT_FILENO);
    if (error>=0) posix_spawn_file_actions_adddup2(&actions,error,STDERR_FILENO);
    std::vector<std::string> args{python.string()};
    for (const auto & argument : arguments) args.push_back(argument.string());
    std::vector<char *> argv;for(auto & arg:args) argv.push_back(arg.data());argv.push_back(nullptr);
    const int rc=posix_spawnp(&pid_,python.c_str(),&actions,nullptr,argv.data(),environ);
    posix_spawn_file_actions_destroy(&actions);close(input[0]);close(output[1]);if (error>=0) close(error);
    if (rc!=0) {pid_=-1;close(input[1]);close(output[0]);throw std::runtime_error(std::string("Cannot start detector Python runtime: ")+std::strerror(rc));}
    write_=input[1];read_=output[0];
#endif
  }
  void Kill() {
#ifdef _WIN32
    if (process_) TerminateProcess(process_,1);
#else
    if (pid_>0) kill(pid_,SIGKILL);
#endif
  }
  void Close() {
    Kill();
#ifdef _WIN32
    if (process_) {WaitForSingleObject(process_,5000);CloseHandle(process_);process_=nullptr;}
    if (read_) {CloseHandle(read_);read_=nullptr;}if (write_) {CloseHandle(write_);write_=nullptr;}
#else
    if (pid_>0) {while (waitpid(pid_,nullptr,0)<0 && errno==EINTR) {}pid_=-1;}
    if (read_>=0) {close(read_);read_=-1;}if (write_>=0) {close(write_);write_=-1;}
#endif
  }
  void Read(void * data,std::size_t size) {Transfer(data,size,false);}
  void Write(const void * data,std::size_t size) {Transfer(const_cast<void *>(data),size,true);}
#ifndef _WIN32
  void SetDeadline(std::chrono::seconds timeout) {deadline_=std::chrono::steady_clock::now()+timeout;}
#endif
private:
  void Transfer(void * data,std::size_t size,bool writing) {
    auto * bytes=static_cast<unsigned char *>(data);
    while (size) {
#ifdef _WIN32
      DWORD transferred=0;const auto count=static_cast<DWORD>(std::min<std::size_t>(size,1<<20));
      const bool ok=writing ? WriteFile(write_,bytes,count,&transferred,nullptr)!=0 : ReadFile(read_,bytes,count,&transferred,nullptr)!=0;
      if (!ok || !transferred) throw std::runtime_error("Model process disconnected; see "+log_path_.filename().u8string());
#else
      std::size_t count=size;
      if (deadline_) {
        const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(
          *deadline_-std::chrono::steady_clock::now());
        if (remaining.count()<=0) throw std::runtime_error("Model process timed out");
        pollfd descriptor{writing ? write_ : read_,static_cast<short>(writing ? POLLOUT : POLLIN),0};
        const int ready=poll(&descriptor,1,static_cast<int>(remaining.count()));
        if (ready<0 && errno==EINTR) continue;
        if (!ready) throw std::runtime_error("Model process timed out");
        if (ready<0) throw std::runtime_error("Cannot poll model process");
        if (writing) count=std::min<std::size_t>(count,4096);
      }
      const auto transferred=writing ? write(write_,bytes,count) : read(read_,bytes,count);
      if (transferred<0 && errno==EINTR) continue;
      if (transferred<=0) throw std::runtime_error("Model process disconnected; see "+log_path_.filename().u8string());
#endif
      bytes+=transferred;size-=transferred;
    }
  }
  std::filesystem::path log_path_;
#ifdef _WIN32
  HANDLE process_{nullptr},read_{nullptr},write_{nullptr};
#else
  pid_t pid_{-1};int read_{-1},write_{-1};
  std::optional<std::chrono::steady_clock::time_point> deadline_;
#endif
};
struct Request {
  std::uint64_t revision{0};float threshold{.35f},yaw{0};
  std::shared_ptr<const std::vector<KittiPoint>> scan;
};
DetectorResult ReadReply(Child & child,float yaw) {
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

#ifdef __linux__
// TerraSeg also runs in synchronous inspect mode. Suppress a broken-pipe
// signal for this write without changing the caller's signal policy.
class BlockSigpipe {
public:
  BlockSigpipe() {
    sigemptyset(&blocked_);sigaddset(&blocked_,SIGPIPE);
    if (pthread_sigmask(SIG_BLOCK,&blocked_,&previous_)!=0)
      throw std::runtime_error("Cannot protect the model worker pipe");
  }
  ~BlockSigpipe() {
    sigset_t pending{};
    if (!sigismember(&previous_,SIGPIPE) && sigpending(&pending)==0 && sigismember(&pending,SIGPIPE)) {
      int signal=0;sigwait(&blocked_,&signal);
    }
    pthread_sigmask(SIG_SETMASK,&previous_,nullptr);
  }
private:
  sigset_t blocked_{},previous_{};
};

struct TerraSegReply {
  std::uint64_t revision{};
  bool error{};
  std::string message;
  std::vector<unsigned char> mask;
};

TerraSegReply ReadTerraSegReply(Child & child, std::size_t expected_points)
{
  std::array<char,20> header{};
  child.Read(header.data(),header.size());
  const std::string tag(header.data(),4);
  TerraSegReply reply;
  std::uint32_t points{},length{};
  std::memcpy(&reply.revision,header.data()+4,8);
  std::memcpy(&points,header.data()+12,4);
  std::memcpy(&length,header.data()+16,4);
  reply.error=tag=="TGE1";
  if ((!reply.error && tag!="TGR1") || length>16384 || points!=(reply.error ? 0 : expected_points))
    throw std::runtime_error("Invalid TerraSeg worker reply");
  reply.message.resize(length);
  child.Read(reply.message.data(),length);
  reply.mask.resize(points);
  child.Read(reply.mask.data(),points);
  if (std::any_of(reply.mask.begin(),reply.mask.end(),[](unsigned char label){return label>1;}))
    throw std::runtime_error("Invalid TerraSeg ground labels");
  return reply;
}
#endif
} // namespace

struct TerraSeg::Impl {
  mutable std::mutex status_lock;
  TerraSegStatus status;
#ifdef __linux__
  std::mutex request_lock,process_lock;
  std::atomic<bool> canceled{false};
  Child child;
  std::filesystem::path python,checkpoint,worker,log;
  std::string ready_message;
  std::vector<float> xyz;
  std::uint64_t next_revision{1};

  void SetStatus(bool ready, bool failed, std::string message) {
    std::lock_guard<std::mutex> lock(status_lock);
    status.ready=ready;status.failed=failed;status.message=std::move(message);
  }
#endif
};

TerraSeg::TerraSeg(const std::filesystem::path & python_root) : impl_(std::make_unique<Impl>())
{
#if defined(__linux__) && defined(__x86_64__)
  auto & model=*impl_;
  const auto folder=UserConfigurationDirectory();
  std::ifstream file(folder/"terraseg.cfg");
  for (std::string line; std::getline(file,line);) {
    if (!line.empty() && line.back()=='\r') line.pop_back();
    if (line.rfind("python=",0)==0) model.python=std::filesystem::u8path(line.substr(7));
    if (line.rfind("checkpoint=",0)==0) model.checkpoint=std::filesystem::u8path(line.substr(11));
  }
  model.worker=python_root/"terraseg_worker.py";
  model.log=folder/"terraseg.log";
  std::error_code error;
  model.status.configured=std::filesystem::is_regular_file(model.worker,error) &&
    std::filesystem::is_regular_file(model.python,error) &&
    std::filesystem::is_regular_file(model.checkpoint,error);
  model.status.message=model.status.configured ? "Waiting for a scan" : "TerraSeg runtime or weights are missing";
#else
  (void)python_root;
  impl_->status.message="TerraSeg requires Linux x64 and NVIDIA CUDA";
#endif
}

TerraSeg::~TerraSeg() = default;

TerraSegStatus TerraSeg::Status() const
{
  std::lock_guard<std::mutex> lock(impl_->status_lock);
  return impl_->status;
}

void TerraSeg::Cancel()
{
#ifdef __linux__
  impl_->canceled=true;
  std::lock_guard<std::mutex> lock(impl_->process_lock);
  impl_->child.Kill();
#endif
}

std::optional<algorithms::GroundRemovalResult> TerraSeg::Segment(
  const std::vector<KittiPoint> & points, float ground_height, float display_yaw, const Vec3 & sensor_origin,
  double & classification_ms)
{
  classification_ms=0;
#ifdef __linux__
  auto & model=*impl_;
  std::lock_guard<std::mutex> lock(model.request_lock);
  const auto status=Status();
  if (!status.configured || status.failed || model.canceled) return std::nullopt;
  if (points.empty()) return algorithms::GroundRemovalResult{};
  if (points.size()>4000000) {
    model.SetStatus(status.ready,false,"TerraSeg supports at most 4 million points per scan");
    return std::nullopt;
  }
  try {
    if (!status.ready) {
      model.SetStatus(false,false,"Loading TerraSeg Small");
      {
        std::lock_guard<std::mutex> process(model.process_lock);
        if (model.canceled) return std::nullopt;
        model.child.Open(model.python,{"-E","-s","-B","-u",model.worker,"--checkpoint",model.checkpoint},model.log);
      }
      model.child.SetDeadline(std::chrono::seconds(120));
      const auto ready=ReadTerraSegReply(model.child,0);
      if (ready.revision || ready.error) throw std::runtime_error(ready.message.empty() ? "Invalid TerraSeg startup reply" : ready.message);
      model.ready_message=ready.message.empty() ? "Ready · CUDA" : "Ready · "+ready.message;
    }
    const auto started=std::chrono::steady_clock::now();
    const float cosine=std::cos(display_yaw),sine=std::sin(display_yaw);
    model.xyz.clear();model.xyz.reserve(points.size()*3);
    for (const auto & point : points) {
      const bool missing=point.x==sensor_origin.x && point.y==sensor_origin.y && point.z==sensor_origin.z;
      model.xyz.push_back(cosine*point.x+sine*point.y);
      model.xyz.push_back(-sine*point.x+cosine*point.y);
      // Skip no-return placeholders in the model while retaining the input index.
      model.xyz.push_back(missing ? std::numeric_limits<float>::quiet_NaN() : point.z-ground_height);
    }
    const auto revision=model.next_revision++;
    const auto count=static_cast<std::uint32_t>(points.size());
    std::array<char,16> header{};
    std::memcpy(header.data(),"TGQ1",4);
    std::memcpy(header.data()+4,&revision,8);
    std::memcpy(header.data()+12,&count,4);
    model.child.SetDeadline(std::chrono::seconds(45));
    {
      BlockSigpipe blocked;
      model.child.Write(header.data(),header.size());
      model.child.Write(model.xyz.data(),model.xyz.size()*sizeof(float));
    }
    auto reply=ReadTerraSegReply(model.child,points.size());
    if (reply.revision!=revision) throw std::runtime_error("TerraSeg frame revision mismatch");
    model.SetStatus(true,false,reply.error ? reply.message : model.ready_message);
    if (reply.error) return std::nullopt;
    auto segments=algorithms::PartitionGround(points,reply.mask);
    classification_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return segments;
  } catch (const std::exception & error) {
    {
      std::lock_guard<std::mutex> process(model.process_lock);
      model.child.Close();
    }
    model.SetStatus(false,true,error.what());
    return std::nullopt;
  }
#else
  (void)points;(void)ground_height;(void)display_yaw;(void)sensor_origin;
  return std::nullopt;
#endif
}

struct LearnedDetector::Impl {
  mutable std::mutex mutex;std::mutex process_mutex;std::condition_variable cv;
  Child child;std::thread thread;bool stop{false};
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
