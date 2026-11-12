#include "lidar_viewer/example_dataset.hpp"
#include "lidar_viewer/platform.hpp"
#include <curl/curl.h>
#include <openssl/evp.h>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace lidar_viewer {
namespace fs=std::filesystem;
const std::array<ExampleFile,6> & DrivingExampleFiles() {
  static constexpr std::array<ExampleFile,6> files{{
    {"000000.bin",1994688,"bf272996d5b6d25cc5589e1089137cb20a98b63bd4823a7fea5631b359f6d68c"},
    {"000001.bin",1993680,"d937cb1bc1ce9ca4e03ccaf69b7537e175c625ecef631b6d668b96aee002faa8"},
    {"000002.bin",1991648,"35ff6de88522818c44984113a3cbf9663e36dae8ae5d0f13f3dfb3bd389d15f2"},
    {"000003.bin",1986672,"7325ce02b1270bee72f4e11bf933cef4747edbf7671c8bdddee93fdec30dee8f"},
    {"000004.bin",1983504,"7d056c5acfbdb56f6c8cc78afb4db6dd0619acb54dd7d4e9cdce7abc4ad6238f"},
    {"000005.bin",1982784,"40eb337a4dc11381be53cfcbd005423dc3ff78f657bf90cbe8ab5e56a7043436"}
  }};return files;
}
fs::path DrivingExampleDirectory() {
  fs::path root;
  if(const char * value=std::getenv("XDG_DATA_HOME");value && *value && fs::u8path(value).is_absolute()) root=fs::u8path(value)/"lidar_viewer";
#ifdef _WIN32
  else if(const char * value=std::getenv("LOCALAPPDATA");value && *value) root=fs::u8path(value)/"LiDARViewer";
  else root=UserHome()/"AppData/Local/LiDARViewer";
#elif defined(__APPLE__)
  else root=UserHome()/"Library/Application Support/LiDARViewer";
#else
  else root=UserHome()/".local/share/lidar_viewer";
#endif
  return root/"examples/driving-v1";
}
bool IsDrivingExample(const fs::path & path) {
  std::error_code error;
  const auto absolute=fs::absolute(path,error);if(error) return false;
  const auto root=fs::weakly_canonical(DrivingExampleDirectory(),error);if(error) return false;
  const auto selected=fs::weakly_canonical(absolute,error);if(error) return false;
  return selected==root || selected.parent_path()==root;
}
namespace {
bool Matches(const fs::path & path,const ExampleFile & file) {
  std::error_code ec;if(fs::file_size(path,ec)!=file.bytes || ec) return false;
  std::ifstream input(path,std::ios::binary);if(!input) return false;
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> digest(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if(!digest || EVP_DigestInit_ex(digest.get(),EVP_sha256(),nullptr)!=1) throw std::runtime_error("Cannot initialize example checksum");
  std::array<char,65536> buffer{};
  while(input) {input.read(buffer.data(),buffer.size());if(EVP_DigestUpdate(digest.get(),buffer.data(),static_cast<std::size_t>(input.gcount()))!=1) throw std::runtime_error("Cannot verify example checksum");}
  if(!input.eof()) return false;
  std::array<unsigned char,EVP_MAX_MD_SIZE> bytes{};unsigned size=0;
  if(EVP_DigestFinal_ex(digest.get(),bytes.data(),&size)!=1) throw std::runtime_error("Cannot finish example checksum");
  std::ostringstream hex;hex<<std::hex<<std::setfill('0');for(unsigned i=0;i<size;++i) hex<<std::setw(2)<<static_cast<unsigned>(bytes[i]);
  return hex.str()==file.sha256;
}
struct CurlGlobal {
  CurlGlobal() {if(curl_global_init(CURL_GLOBAL_DEFAULT)!=CURLE_OK) throw std::runtime_error("Cannot initialize HTTPS downloads");}
  ~CurlGlobal() {curl_global_cleanup();}
};
class CurlSession {
  std::unique_ptr<CURL,decltype(&curl_easy_cleanup)> handle_{nullptr,curl_easy_cleanup};
  struct Write {
    std::ofstream output;const ExampleFile & file;
    const std::function<bool(std::uint64_t)> & progress;
    std::uint64_t count{0};
  };
  static size_t Receive(char * data,size_t size,size_t n,void * context) noexcept {
    try {
      auto & write=*static_cast<Write *>(context);
      if(size && n>std::numeric_limits<size_t>::max()/size) return 0;
      const auto bytes=size*n;
      if(bytes>write.file.bytes-write.count || !write.progress(write.count)) return 0;
      write.output.write(data,static_cast<std::streamsize>(bytes));if(!write.output) return 0;
      write.count+=bytes;return bytes;
    } catch(...) {return 0;}
  }
  static int Progress(void * context,curl_off_t,curl_off_t now,curl_off_t,curl_off_t) noexcept {
    try {return static_cast<Write *>(context)->progress(static_cast<std::uint64_t>(std::max<curl_off_t>(0,now))) ? 0 : 1;}
    catch(...) {return 1;}
  }
public:
  CurlSession() {
    static const CurlGlobal global;
    handle_.reset(curl_easy_init());if(!handle_) throw std::runtime_error("Cannot create HTTPS download session");
  }
  void Fetch(const ExampleFile & file,const fs::path & path,const std::function<bool(std::uint64_t)> & progress) {
    const std::string url=std::string("https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/data/")+file.name;
    Write write{std::ofstream(path,std::ios::binary),file,progress};if(!write.output) throw std::runtime_error("Cannot create example download file");
    auto * curl=handle_.get();curl_easy_reset(curl);std::array<char,CURL_ERROR_SIZE> error{};
    curl_easy_setopt(curl,CURLOPT_URL,url.c_str());curl_easy_setopt(curl,CURLOPT_ERRORBUFFER,error.data());
    curl_easy_setopt(curl,CURLOPT_FOLLOWLOCATION,1L);curl_easy_setopt(curl,CURLOPT_MAXREDIRS,3L);
#if LIBCURL_VERSION_NUM >= 0x075500
    curl_easy_setopt(curl,CURLOPT_PROTOCOLS_STR,"https");curl_easy_setopt(curl,CURLOPT_REDIR_PROTOCOLS_STR,"https");
#else
    curl_easy_setopt(curl,CURLOPT_PROTOCOLS,CURLPROTO_HTTPS);curl_easy_setopt(curl,CURLOPT_REDIR_PROTOCOLS,CURLPROTO_HTTPS);
#endif
#if (defined(_WIN32) || defined(__APPLE__)) && defined(CURLSSLOPT_NATIVE_CA)
    curl_easy_setopt(curl,CURLOPT_SSL_OPTIONS,static_cast<long>(CURLSSLOPT_NATIVE_CA));
#endif
    curl_easy_setopt(curl,CURLOPT_SSL_VERIFYPEER,1L);curl_easy_setopt(curl,CURLOPT_SSL_VERIFYHOST,2L);
    curl_easy_setopt(curl,CURLOPT_FAILONERROR,1L);curl_easy_setopt(curl,CURLOPT_NOSIGNAL,1L);
    curl_easy_setopt(curl,CURLOPT_CONNECTTIMEOUT_MS,5000L);curl_easy_setopt(curl,CURLOPT_TIMEOUT_MS,90000L);
    curl_easy_setopt(curl,CURLOPT_LOW_SPEED_TIME,15L);curl_easy_setopt(curl,CURLOPT_LOW_SPEED_LIMIT,1024L);
    curl_easy_setopt(curl,CURLOPT_USERAGENT,"LiDAR-Studio/0.2 example-downloader");
    curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,Receive);curl_easy_setopt(curl,CURLOPT_WRITEDATA,&write);
    curl_easy_setopt(curl,CURLOPT_NOPROGRESS,0L);curl_easy_setopt(curl,CURLOPT_XFERINFOFUNCTION,Progress);curl_easy_setopt(curl,CURLOPT_XFERINFODATA,&write);
    const auto result=curl_easy_perform(curl);write.output.close();
    if(result!=CURLE_OK) throw std::runtime_error(error[0] ? error.data() : curl_easy_strerror(result));
    if(!write.output || write.count!=file.bytes) throw std::runtime_error("Incomplete example download");
  }
};
struct PartialFile {fs::path path;~PartialFile() {std::error_code ec;fs::remove(path,ec);}};
}
struct ExampleDataset::Impl {
  fs::path directory;ExampleTransfer transfer;mutable std::mutex mutex;
  ExampleProgress progress;std::atomic_bool canceled{false};std::thread worker;
  void Run() {
    try {
      fs::create_directories(directory);std::unique_ptr<CurlSession> session;
      std::uint64_t complete=0;
      for(const auto & file:DrivingExampleFiles()) {
        if(canceled) throw std::runtime_error("Download canceled");
        {std::lock_guard<std::mutex> lock(mutex);progress.filename=file.name;progress.stage=ExampleStage::Checking;}
        const auto destination=directory/file.name;
        if(Matches(destination,file)) {
          complete+=file.bytes;std::lock_guard<std::mutex> lock(mutex);++progress.files;++progress.cached;progress.bytes=complete;continue;
        }
        {std::lock_guard<std::mutex> lock(mutex);progress.stage=ExampleStage::Downloading;}
        const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
        PartialFile temporary{directory/(std::string(file.name)+"."+std::to_string(ProcessId())+"-"+std::to_string(stamp)+".part")};
        const std::function<bool(std::uint64_t)> report=[&](std::uint64_t bytes) {
          if(canceled) return false;
          std::lock_guard<std::mutex> lock(mutex);progress.bytes=std::max(progress.bytes,complete+std::min(bytes,file.bytes));return true;
        };
        if(transfer) transfer(file,temporary.path,report);
        else {if(!session) session=std::make_unique<CurlSession>();session->Fetch(file,temporary.path,report);}
        if(canceled) throw std::runtime_error("Download canceled");
        if(!Matches(temporary.path,file)) throw std::runtime_error(std::string("Checksum mismatch: ")+file.name);
        // Another app instance may have finished this scan while we downloaded.
        if(!Matches(destination,file)) {
          std::error_code ec;fs::remove(destination,ec);fs::rename(temporary.path,destination);
        }
        complete+=file.bytes;std::lock_guard<std::mutex> lock(mutex);++progress.files;progress.bytes=complete;
      }
      session.reset();std::lock_guard<std::mutex> lock(mutex);progress.stage=canceled ? ExampleStage::Canceled : ExampleStage::Ready;
    } catch(const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex);progress.stage=canceled ? ExampleStage::Canceled : ExampleStage::Failed;
      if(!canceled) progress.error=error.what();
    }
  }
};
ExampleDataset::ExampleDataset(fs::path directory,ExampleTransfer transfer):impl_(std::make_unique<Impl>()) {
  impl_->directory=std::move(directory);impl_->transfer=std::move(transfer);
}
ExampleDataset::~ExampleDataset() {Cancel();if(impl_->worker.joinable()) impl_->worker.join();}
bool ExampleDataset::Start() {
  if(Progress().Active()) return false;
  if(impl_->worker.joinable()) impl_->worker.join();
  impl_->canceled=false;
  {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->progress={};impl_->progress.stage=ExampleStage::Checking;
    for(const auto & file:DrivingExampleFiles()) impl_->progress.total_bytes+=file.bytes;}
  try {impl_->worker=std::thread([this]{impl_->Run();});}
  catch(...) {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->progress.stage=ExampleStage::Failed;throw;}
  return true;
}
void ExampleDataset::Cancel() {impl_->canceled=true;}
ExampleProgress ExampleDataset::Progress() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->progress;}
fs::path ExampleDataset::Directory() const {return impl_->directory;}
} // namespace lidar_viewer
