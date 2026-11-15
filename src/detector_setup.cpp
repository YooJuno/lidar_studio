#include "lidar_viewer/detector_setup.hpp"
#include "lidar_viewer/platform.hpp"
#include "lidar_viewer/training_job.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <curl/curl.h>
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#ifndef _WIN32
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

namespace lidar_viewer {
namespace fs=std::filesystem;
namespace {
struct Canceled {};
class SetupLock {
public:
  explicit SetupLock(const fs::path & path) {
#ifdef _WIN32
    handle_=CreateFileW(path.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    if(handle_==INVALID_HANDLE_VALUE) throw std::runtime_error("Another window is preparing models. Wait for it to finish, then retry.");
#else
    descriptor_=open(path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC,0600);
    if(descriptor_<0) throw std::runtime_error("Cannot create model preparation lock");
    if(flock(descriptor_,LOCK_EX|LOCK_NB)!=0) {
      close(descriptor_);descriptor_=-1;
      throw std::runtime_error("Another window is preparing models. Wait for it to finish, then retry.");
    }
#endif
  }
  ~SetupLock() {
#ifdef _WIN32
    CloseHandle(handle_);
#else
    close(descriptor_);
#endif
  }
private:
#ifdef _WIN32
  HANDLE handle_{INVALID_HANDLE_VALUE};
#else
  int descriptor_{-1};
#endif
};
struct TemporaryFolder {
  fs::path path;
  ~TemporaryFolder() {std::error_code error;fs::remove_all(path,error);}
};
std::string Sha256(const fs::path & path) {
  std::ifstream file(path,std::ios::binary);if(!file) return {};
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> digest(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if(!digest || EVP_DigestInit_ex(digest.get(),EVP_sha256(),nullptr)!=1) throw std::runtime_error("Cannot verify runtime download");
  std::array<char,65536> block{};
  while(file) {file.read(block.data(),block.size());if(EVP_DigestUpdate(digest.get(),block.data(),static_cast<std::size_t>(file.gcount()))!=1)
    throw std::runtime_error("Cannot verify runtime download");}
  if(!file.eof()) throw std::runtime_error("Cannot read runtime download");
  std::array<unsigned char,EVP_MAX_MD_SIZE> bytes{};unsigned count=0;
  if(EVP_DigestFinal_ex(digest.get(),bytes.data(),&count)!=1) throw std::runtime_error("Cannot verify runtime download");
  std::ostringstream output;output<<std::hex<<std::setfill('0');
  for(unsigned i=0;i<count;++i) output<<std::setw(2)<<static_cast<unsigned>(bytes[i]);
  return output.str();
}
}
fs::path DetectorSetup::DataDirectory() {
  if(const char * data=std::getenv("XDG_DATA_HOME");data && *data && fs::u8path(data).is_absolute())
    return fs::u8path(data)/"lidar_viewer/detector";
#ifdef _WIN32
  if(const char * data=std::getenv("LOCALAPPDATA");data && *data) return fs::u8path(data)/"LiDARViewer/detector";
  return UserHome()/"AppData/Local/LiDARViewer/detector";
#elif defined(__APPLE__)
  return UserHome()/"Library/Application Support/LiDARViewer/detector";
#else
  return UserHome()/".local/share/lidar_viewer/detector";
#endif
}
struct DetectorSetup::Impl {
  mutable std::mutex mutex;
  DetectorSetupProgress progress;
  std::atomic_bool canceled{false};
  std::thread worker;
  fs::path prepared_python;
  void Check() const {if(canceled) throw Canceled{};}
  void Report(const std::string & message,std::uint64_t bytes=0,std::uint64_t total=0) {
    std::lock_guard<std::mutex> lock(mutex);progress.message=message;progress.bytes=bytes;progress.total_bytes=total;
  }
  void RunCommand(const std::vector<std::string> & arguments,const fs::path & log,const fs::path & status={}) {
    Check();TrainingJob process;process.StartCommand(arguments,log);
    while(process.Running()) {
      if(canceled) process.Cancel();
      process.Poll();
      if(!status.empty()) {
        try {
          // A filesystem-path stream preserves Unicode paths on Windows;
          // Boost's narrow filename overload cannot represent every username.
          std::ifstream file(status);
          boost::property_tree::ptree state;boost::property_tree::read_json(file,state);
          Report(state.get<std::string>("message"),state.get<std::uint64_t>("bytes",0),state.get<std::uint64_t>("total_bytes",0));
        } catch(const std::exception &) {} // Atomic status may not exist yet.
      }
      {std::lock_guard<std::mutex> lock(mutex);progress.log=process.LogTail(8192);}
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Check();
    if(process.ExitCode()!=0) throw std::runtime_error("Model preparation failed. Check the installation details and retry.");
  }
  struct Transfer {
    Impl & owner;std::ofstream output;std::uint64_t count{0};
    static size_t Write(char * data,size_t size,size_t count,void * context) noexcept {
      auto & transfer=*static_cast<Transfer *>(context);
      if(size && count>static_cast<size_t>(-1)/size) return 0;
      const auto bytes=size*count;
      if(transfer.owner.canceled || bytes>256*1024*1024-transfer.count) return 0;
      transfer.output.write(data,static_cast<std::streamsize>(bytes));transfer.count+=bytes;
      return transfer.output ? bytes : 0;
    }
    static int Progress(void * context,curl_off_t total,curl_off_t now,curl_off_t,curl_off_t) noexcept {
      auto & owner=static_cast<Transfer *>(context)->owner;
      try {owner.Report("Downloading private model runtime",static_cast<std::uint64_t>(std::max<curl_off_t>(0,now)),
        static_cast<std::uint64_t>(std::max<curl_off_t>(0,total)));} catch(...) {return 1;}
      return owner.canceled ? 1 : 0;
    }
  };
  void Download(const std::string & url,const fs::path & target) {
    struct CurlGlobal {
      CurlGlobal() {if(curl_global_init(CURL_GLOBAL_DEFAULT)!=CURLE_OK) throw std::runtime_error("Cannot initialize model download");}
      ~CurlGlobal() {curl_global_cleanup();}
    };
    static const CurlGlobal global;
    std::unique_ptr<CURL,decltype(&curl_easy_cleanup)> curl(curl_easy_init(),curl_easy_cleanup);
    if(!curl) throw std::runtime_error("Cannot start model download");
    Transfer transfer{*this,std::ofstream(target,std::ios::binary)};
    if(!transfer.output) throw std::runtime_error("Cannot save model runtime");
    std::array<char,CURL_ERROR_SIZE> error{};
    curl_easy_setopt(curl.get(),CURLOPT_URL,url.c_str());curl_easy_setopt(curl.get(),CURLOPT_ERRORBUFFER,error.data());
    curl_easy_setopt(curl.get(),CURLOPT_FOLLOWLOCATION,1L);curl_easy_setopt(curl.get(),CURLOPT_MAXREDIRS,5L);
#if LIBCURL_VERSION_NUM >= 0x075500
    curl_easy_setopt(curl.get(),CURLOPT_PROTOCOLS_STR,"https");curl_easy_setopt(curl.get(),CURLOPT_REDIR_PROTOCOLS_STR,"https");
#else
    curl_easy_setopt(curl.get(),CURLOPT_PROTOCOLS,CURLPROTO_HTTPS);curl_easy_setopt(curl.get(),CURLOPT_REDIR_PROTOCOLS,CURLPROTO_HTTPS);
#endif
#if (defined(_WIN32) || defined(__APPLE__)) && defined(CURLSSLOPT_NATIVE_CA)
    curl_easy_setopt(curl.get(),CURLOPT_SSL_OPTIONS,static_cast<long>(CURLSSLOPT_NATIVE_CA));
#endif
    curl_easy_setopt(curl.get(),CURLOPT_SSL_VERIFYPEER,1L);curl_easy_setopt(curl.get(),CURLOPT_SSL_VERIFYHOST,2L);
    curl_easy_setopt(curl.get(),CURLOPT_FAILONERROR,1L);curl_easy_setopt(curl.get(),CURLOPT_NOSIGNAL,1L);
    curl_easy_setopt(curl.get(),CURLOPT_CONNECTTIMEOUT,10L);curl_easy_setopt(curl.get(),CURLOPT_TIMEOUT,300L);
    curl_easy_setopt(curl.get(),CURLOPT_LOW_SPEED_TIME,20L);curl_easy_setopt(curl.get(),CURLOPT_LOW_SPEED_LIMIT,1024L);
    curl_easy_setopt(curl.get(),CURLOPT_USERAGENT,"LiDAR-Studio/model-setup");
    curl_easy_setopt(curl.get(),CURLOPT_WRITEFUNCTION,Transfer::Write);curl_easy_setopt(curl.get(),CURLOPT_WRITEDATA,&transfer);
    curl_easy_setopt(curl.get(),CURLOPT_NOPROGRESS,0L);curl_easy_setopt(curl.get(),CURLOPT_XFERINFOFUNCTION,Transfer::Progress);
    curl_easy_setopt(curl.get(),CURLOPT_XFERINFODATA,&transfer);
    const auto result=curl_easy_perform(curl.get());transfer.output.close();Check();
    if(result!=CURLE_OK) throw std::runtime_error(error[0] ? error.data() : curl_easy_strerror(result));
    if(!transfer.output) throw std::runtime_error("Cannot save model runtime");
  }
  fs::path Python(const fs::path & root,const fs::path & data,const fs::path & log) {
    if(!prepared_python.empty()) return prepared_python;
#if defined(__APPLE__) && defined(__aarch64__)
    const auto runtime=data/"app-runtime",python=runtime/"bin/python3";
    if(fs::is_regular_file(python)) return python;
    boost::property_tree::ptree manifest;
    boost::property_tree::read_json((root/"lidar_detection/runtime.json").string(),manifest);
    const auto url=manifest.get<std::string>("macos_arm64.url"),sha=manifest.get<std::string>("macos_arm64.sha256");
    if(sha.size()!=64) throw std::runtime_error("Invalid model runtime checksum manifest");
    const auto archive=data/"app-runtime.tar.gz";
    if(Sha256(archive)!=sha) {
      const auto partial=data/"app-runtime.tar.gz.part";
      try {
        Download(url,partial);Report("Verifying private runtime");
        if(Sha256(partial)!=sha) throw std::runtime_error("Runtime checksum mismatch; retry the download");
        fs::rename(partial,archive);
      } catch(...) {std::error_code error;fs::remove(partial,error);throw;}
    }
    Check();Report("Preparing private model runtime");
    TemporaryFolder temporary{data/(".runtime-"+std::to_string(ProcessId())+"-"+
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))};
    if(!fs::create_directory(temporary.path)) throw std::runtime_error("Cannot prepare private model runtime folder");
    RunCommand({"/usr/bin/tar","-xzf",archive.u8string(),"-C",temporary.path.u8string()},log);
    if(!fs::is_regular_file(temporary.path/"python/bin/python3")) throw std::runtime_error("Downloaded runtime has no Python executable");
    Check();fs::rename(temporary.path/"python",runtime);return python;
#else
    (void)root;(void)data;(void)log;
    // Other platforms keep using their supported Python installation. The
    // helper validates its version and reports any missing runtime in the UI.
#ifdef _WIN32
    return "python";
#else
    return "python3";
#endif
#endif
  }
  void Run(const fs::path & root,const fs::path & script) {
    try {
      if(!fs::is_regular_file(script)) throw std::runtime_error("The app is missing its model installer");
      const auto config=UserConfigurationDirectory(),data=DataDirectory();
      fs::create_directories(config);fs::create_directories(data);
      {
      const SetupLock installation_lock(data/".setup-lock");
      const auto log=config/"detector-setup.log",status=config/"detector-setup-progress.json";
      {std::error_code error;fs::remove(status,error);}
      const auto python=Python(root,data,log);
      Report("Preparing CPU models");
      std::vector<std::string> arguments{python.u8string(),"-E","-s","-B","-u",script.u8string(),"--progress-file",status.u8string()};
#if defined(__APPLE__) && defined(__aarch64__)
      // Packages go into the app-owned standalone interpreter, never system Python.
      arguments.insert(arguments.end(),{"--runtime-python",python.u8string(),"--install-packages"});
#endif
      RunCommand(arguments,log,status);Check();
      if(!fs::is_regular_file(config/"detector.cfg")) throw std::runtime_error("Model preparation did not publish a detector configuration");
      } // Release the installation lock before exposing the completed state.
      std::lock_guard<std::mutex> lock(mutex);progress.stage=DetectorSetupStage::Ready;
      progress.message="Models ready";progress.bytes=progress.total_bytes=0;
    } catch(const Canceled &) {
      std::lock_guard<std::mutex> lock(mutex);progress.stage=DetectorSetupStage::Canceled;progress.message="Model preparation canceled";
    } catch(const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex);progress.stage=DetectorSetupStage::Failed;progress.error=error.what();progress.message="Model preparation failed";
    }
  }
};
DetectorSetup::DetectorSetup(fs::path prepared_python):impl_(std::make_unique<Impl>()) {impl_->prepared_python=std::move(prepared_python);}
DetectorSetup::~DetectorSetup() {Cancel();if(impl_->worker.joinable()) impl_->worker.join();}
bool DetectorSetup::Start(const fs::path & root,const fs::path & script) {
  if(Progress().Active()) return false;
  if(impl_->worker.joinable()) impl_->worker.join();
  impl_->canceled=false;
  {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->progress={};impl_->progress.stage=DetectorSetupStage::Working;impl_->progress.message="Preparing model runtime";}
  impl_->worker=std::thread([this,root,script]{impl_->Run(root,script);});return true;
}
void DetectorSetup::Cancel() {impl_->canceled=true;}
DetectorSetupProgress DetectorSetup::Progress() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->progress;}
} // namespace lidar_viewer
