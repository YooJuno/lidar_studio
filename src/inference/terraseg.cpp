#include "lidar_viewer/terraseg.hpp"
#include "lidar_viewer/algorithms/ground_partition.hpp"
#include "lidar_viewer/platform.hpp"
#include "model_process.hpp"

#include <mutex>
#include <utility>
#ifdef __linux__
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <limits>
#include <pthread.h>
#include <stdexcept>
#endif

namespace lidar_viewer
{
namespace
{
#ifdef __linux__
using inference::ModelProcess;
static_assert(sizeof(float)==4,"TerraSeg protocol requires float32");

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

TerraSegReply ReadTerraSegReply(ModelProcess & child, std::size_t expected_points)
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
  ModelProcess child;
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

} // namespace lidar_viewer
