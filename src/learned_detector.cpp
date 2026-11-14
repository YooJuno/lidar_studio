#include "lidar_viewer/learned_detector.hpp"
#include "lidar_viewer/platform.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <condition_variable>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>
#ifndef _WIN32
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <pthread.h>
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
  void Open(const std::filesystem::path & python,const std::filesystem::path & worker,
    const std::filesystem::path & model,const std::string & variant,const std::string & provider,const std::filesystem::path & log,bool bundled=false) {
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
    std::wstring command=quote(python.wstring())+(bundled ? L" -E -s -B -u " : L" -B -u ")+quote(worker.wstring())+L" --model-dir "+quote(model.wstring())+
      L" --provider "+quote(std::wstring(provider.begin(),provider.end()))+
      L" --model "+quote(std::wstring(variant.begin(),variant.end()))+
      L" --native-dir "+quote(ExecutablePath().parent_path().wstring());
    STARTUPINFOW start{};start.cb=sizeof(start);start.dwFlags=STARTF_USESTDHANDLES;
    start.hStdInput=child_in;start.hStdOutput=child_out;start.hStdError=error;
    PROCESS_INFORMATION process{};
    const bool ok=CreateProcessW(python.c_str(),command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&start,&process)!=0;
    CloseHandle(child_in);CloseHandle(child_out);if (error!=INVALID_HANDLE_VALUE) CloseHandle(error);
    if (!ok) throw std::runtime_error("Cannot start detector Python runtime");
    process_=process.hProcess;CloseHandle(process.hThread);
#else
    int input[2],output[2];
    if (pipe(input)!=0) throw std::runtime_error("Cannot create detector input pipe");
    if (pipe(output)!=0) {close(input[0]);close(input[1]);throw std::runtime_error("Cannot create detector output pipe");}
    for (int fd:{input[0],input[1],output[0],output[1]}) fcntl(fd,F_SETFD,FD_CLOEXEC);
    const int error=open(log.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0600);
    posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions,input[0],STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions,output[1],STDOUT_FILENO);
    if (error>=0) posix_spawn_file_actions_adddup2(&actions,error,STDERR_FILENO);
    std::vector<std::string> args{python.string(),"-B","-u"};
    if(bundled) {args.push_back("-E");args.push_back("-s");}
    args.insert(args.end(),{worker.string(),"--model-dir",model.string(),"--provider",provider,
      "--model",variant,"--native-dir",ExecutablePath().parent_path().string()});
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
private:
  void Transfer(void * data,std::size_t size,bool writing) {
    auto * bytes=static_cast<unsigned char *>(data);
    while (size) {
#ifdef _WIN32
      DWORD transferred=0;const auto count=static_cast<DWORD>(std::min<std::size_t>(size,1<<20));
      const bool ok=writing ? WriteFile(write_,bytes,count,&transferred,nullptr)!=0 : ReadFile(read_,bytes,count,&transferred,nullptr)!=0;
      if (!ok || !transferred) throw std::runtime_error("Detector process disconnected; see detector.log");
#else
      const auto transferred=writing ? write(write_,bytes,size) : read(read_,bytes,size);
      if (transferred<0 && errno==EINTR) continue;
      if (transferred<=0) throw std::runtime_error("Detector process disconnected; see detector.log");
#endif
      bytes+=transferred;size-=transferred;
    }
  }
#ifdef _WIN32
  HANDLE process_{nullptr},read_{nullptr},write_{nullptr};
#else
  pid_t pid_{-1};int read_{-1},write_{-1};
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
} // namespace

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
  if (variant!="tiny" && variant!="short_range") throw std::invalid_argument("Unknown detector model");
  Stop();
  const auto config=UserConfigurationDirectory()/"detector.cfg";
  std::filesystem::path python,model,short_model;std::string provider="auto",line;
  std::ifstream file(config);
  while (std::getline(file,line)) {
    if (!line.empty() && line.back()=='\r') line.pop_back();
    const auto equal=line.find('=');if (equal==std::string::npos) continue;
    const auto value=line.substr(equal+1),key=line.substr(0,equal);
    if (key=="python") python=std::filesystem::u8path(value);
    else if (key=="model_dir") model=std::filesystem::u8path(value);
    else if (key=="short_range_model_dir") short_model=std::filesystem::u8path(value);
    else if (key=="provider" && value=="cpu") provider=value;
  }
  if (variant=="short_range") model=!short_model.empty() ? short_model : model.empty() ?
    std::filesystem::path{} : model.parent_path()/"centerpoint-shortrange-v2.0.0";
  // Packaged Mac builds contain an offline, relocatable CPU runtime. Explicit
  // user configurations retain priority; a new Mac needs no setup command.
  bool bundled=false;
  if((variant=="tiny" || variant=="short_range") && python.empty() && model.empty()) {
    const auto assets=root.parent_path()/"detector";
    const auto runtime=assets/"runtime/bin/python3";
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
        impl_->child.Open(python,root/"detector_worker.py",model,variant,provider,config.parent_path()/"detector.log",bundled);
      }
      auto ready=ReadReply(impl_->child,0);
      if (ready.revision || !ready.error.empty()) throw std::runtime_error(ready.error.empty() ? "Invalid detector startup reply" : ready.error);
      {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->status.ready=true;impl_->status.provider=ready.detail;impl_->status.message="Ready";}
      std::vector<float> xyz; // Reuse IPC storage across scans, outside the GUI.
      while (true) {
        Request request;
        {std::unique_lock<std::mutex> lock(impl_->mutex);impl_->cv.wait(lock,[this]{return impl_->stop || impl_->pending.has_value();});
          if (impl_->stop) break;
          request=std::move(*impl_->pending);impl_->pending.reset();}
        const auto processing_started=std::chrono::steady_clock::now();
        xyz.clear();xyz.reserve(request.scan->size()*3);
        const float c=std::cos(request.yaw),s=std::sin(request.yaw);
        for (const auto & point:*request.scan) {
          xyz.push_back(c*point.x+s*point.y);xyz.push_back(-s*point.x+c*point.y);xyz.push_back(point.z);
        }
        request.scan.reset();
        const std::uint32_t count=static_cast<std::uint32_t>(xyz.size()/3);
        std::array<char,20> header{};std::memcpy(header.data(),"LDQ1",4);
        std::memcpy(header.data()+4,&request.revision,8);std::memcpy(header.data()+12,&count,4);std::memcpy(header.data()+16,&request.threshold,4);
        impl_->child.Write(header.data(),header.size());
        impl_->child.Write(xyz.data(),xyz.size()*sizeof(float));
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
