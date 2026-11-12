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
#include <vector>

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
namespace {
const ExampleFile urban_files[]{
#include "example_urbanv2x.inc"
};
const ExampleFile hdl32_files[]{
#include "example_hdl32.inc"
};
const ExampleFile at128_files[]{
#include "example_at128.inc"
};
}
std::uint64_t ExampleDefinition::TotalBytes() const {
  std::uint64_t bytes=0;for(std::size_t i=0;i<count;++i) bytes+=files[i].bytes;return bytes;
}
const std::array<ExampleDefinition,7> & DrivingExamples() {
  static const std::array<ExampleDefinition,7> examples{{
    {ExampleKind::Quick,"driving-v1","Quick preview | 6 frames","Patchwork++ public sample",
      "https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/data/",DrivingExampleFiles().data(),6,"Quick preview",0,"example"},
    {ExampleKind::Urban30,"urbanv2x-xt32-30s-v1","UrbanV2X XT32 | 300 frames | 0:30","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/xt32/",urban_files,300,"Hesai XT32",32,"example-xt32"},
    {ExampleKind::Urban60,"urbanv2x-xt32-60s-v1","UrbanV2X XT32 | 600 frames | 1:00","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/xt32/",urban_files,600,"Hesai XT32",32,"example-xt32"},
    {ExampleKind::Velodyne30,"urbanv2x-hdl32-30s-v1","UrbanV2X HDL-32E | 300 frames | 0:30","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/hdl32/",hdl32_files,300,"Velodyne HDL-32E",32,"example-hdl32"},
    {ExampleKind::Velodyne60,"urbanv2x-hdl32-60s-v1","UrbanV2X HDL-32E | 600 frames | 1:00","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/hdl32/",hdl32_files,600,"Velodyne HDL-32E",32,"example-hdl32"},
    {ExampleKind::At12830,"urbanv2x-at128-30s-v1","UrbanV2X AT128P | 300 frames | 0:30","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_02/lidar/at128p/",at128_files,300,"Hesai AT128P",128,"example-at128"},
    {ExampleKind::At12860,"urbanv2x-at128-60s-v1","UrbanV2X AT128P | 600 frames | 1:00","TAS Lab, Hong Kong Polytechnic University",
      "https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_02/lidar/at128p/",at128_files,600,"Hesai AT128P",128,"example-at128"}
  }};return examples;
}
const ExampleDefinition & DrivingExample(ExampleKind kind) {
  for(const auto & example:DrivingExamples()) if(example.kind==kind) return example;
  throw std::invalid_argument("Unknown driving example");
}
fs::path DrivingExampleDirectory(ExampleKind kind) {
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
  return root/"examples"/DrivingExample(kind).id;
}
bool IsDrivingExample(const fs::path & path) {
  std::error_code error;
  const auto absolute=fs::absolute(path,error);if(error) return false;
  const auto selected=fs::weakly_canonical(absolute,error);if(error) return false;
  for(const auto & example:DrivingExamples()) {
    const auto root=fs::weakly_canonical(DrivingExampleDirectory(example.kind),error);if(error) continue;
    if(selected==root || selected.parent_path()==root) return true;
  }
  return false;
}
ExampleCacheInfo InspectExampleCache(ExampleKind kind) {
  ExampleCacheInfo info;const auto directory=DrivingExampleDirectory(kind);const auto & example=DrivingExample(kind);
  for(std::size_t i=0;i<example.count;++i) {
    std::error_code error;const auto bytes=fs::file_size(directory/example.files[i].name,error);
    if(!error) {++info.files;info.bytes+=bytes;}
  }
  return info;
}
void DeleteExampleCache(ExampleKind kind) {
  // The catalog supplies the path and allowlist, never a user-entered path.
  const auto directory=DrivingExampleDirectory(kind);const auto & example=DrivingExample(kind);
  for(int attempt=0;;++attempt) {
    try {
      std::error_code error;
      const auto status=fs::symlink_status(directory,error);
      if(error && error!=std::errc::no_such_file_or_directory) throw fs::filesystem_error("Cannot inspect example directory",directory,error);
      if(!fs::exists(status)) return;
      if(fs::is_symlink(directory)) {fs::remove(directory);return;}
      for(std::size_t i=0;i<example.count;++i) fs::remove(directory/example.files[i].name);
      for(const auto & entry:fs::directory_iterator(directory)) {
        const auto name=entry.path().filename().u8string();if(entry.path().extension()!=".part") continue;
        for(std::size_t i=0;i<example.count;++i) {
          const auto prefix=std::string(example.files[i].name)+".";
          if(name.compare(0,prefix.size(),prefix)==0) {fs::remove(entry.path());break;}
        }
      }
      // Preserve exports or other user files placed alongside downloaded scans.
      fs::remove(directory,error);
      if(error && error!=std::errc::directory_not_empty) throw fs::filesystem_error("Cannot delete example directory",directory,error);
      return;
    } catch(const fs::filesystem_error &) {
      // A canceled frame read may briefly retain a file handle on Windows.
      if(attempt>=20) throw;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
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
  void Fetch(const ExampleDefinition & example,const ExampleFile & file,const fs::path & path,const std::function<bool(std::uint64_t)> & progress) {
    const std::string url=std::string(example.base_url)+file.name;
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
ExampleFile CachedFile(const ExampleFile & file) {
  return {file.name,file.xyzi_sha256 ? file.bytes/6*4 : file.bytes,file.xyzi_sha256 ? file.xyzi_sha256 : file.sha256};
}
void NormalizeXyzi(const fs::path & input_path,const fs::path & output_path,const std::atomic_bool & canceled) {
  std::ifstream input(input_path,std::ios::binary);std::ofstream output(output_path,std::ios::binary);
  if(!input || !output) throw std::runtime_error("Cannot normalize example point fields");
  std::array<char,24*2048> source{};std::array<char,16*2048> xyzi{};
  while(input) {
    if(canceled) throw std::runtime_error("Download canceled");
    input.read(source.data(),source.size());const auto bytes=input.gcount();
    if(bytes%24) throw std::runtime_error("Incomplete six-column point record");
    for(std::size_t i=0;i<static_cast<std::size_t>(bytes)/24;++i) std::copy_n(source.data()+i*24,16,xyzi.data()+i*16);
    output.write(xyzi.data(),bytes/24*16);
  }
  output.close();if(!input.eof() || !output) throw std::runtime_error("Cannot save normalized example scan");
}
struct PartialFile {fs::path path;~PartialFile() {std::error_code ec;fs::remove(path,ec);}};
}
struct ExampleDataset::Impl {
  fs::path directory;ExampleKind kind{ExampleKind::Quick};ExampleTransfer transfer;mutable std::mutex mutex;
  ExampleProgress progress;std::atomic_bool canceled{false},failed{false};std::thread worker;
  std::vector<std::uint64_t> received;
  std::uint64_t downloaded{0};std::chrono::steady_clock::time_point started;
  void Report(std::size_t index,std::uint64_t bytes,bool cached=false) {
    std::lock_guard<std::mutex> lock(mutex);
    bytes=std::min(bytes,DrivingExample(kind).files[index].bytes);
    if(bytes>received[index]) {
      const auto delta=bytes-received[index];progress.bytes+=delta;
      if(!cached) downloaded+=delta;
      received[index]=bytes;
    }
    const auto seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    if(seconds>=1 && downloaded) progress.eta_seconds=seconds*(progress.total_bytes-progress.bytes)/downloaded;
  }
  void Run() {
    try {
      fs::create_directories(directory);const auto & example=DrivingExample(kind);
      std::atomic_size_t next{0};
      const auto work=[&] {
        try {
          std::unique_ptr<CurlSession> session;
          while(!canceled && !failed) {
            const auto index=next.fetch_add(1);if(index>=example.count) break;
            const auto & file=example.files[index];const auto cached_file=CachedFile(file);
            {std::lock_guard<std::mutex> lock(mutex);progress.filename=file.name;}
            const auto destination=directory/file.name;
            bool cached=Matches(destination,cached_file);
            // 30s and 60s selections share verified scans without another download.
            if(!cached && kind!=ExampleKind::Quick) {
              for(const auto & other:DrivingExamples()) {
                if(other.kind==kind || other.files!=example.files) continue;
                const auto sibling=directory.parent_path()/other.id/file.name;
                if(!Matches(sibling,cached_file)) continue;
                std::error_code ec;fs::remove(destination,ec);ec.clear();fs::create_hard_link(sibling,destination,ec);
                if(ec) fs::copy_file(sibling,destination,fs::copy_options::overwrite_existing);
                cached=true;break;
              }
            }
            if(!cached) {
              {std::lock_guard<std::mutex> lock(mutex);progress.stage=ExampleStage::Downloading;}
              const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
              PartialFile temporary{directory/(std::string(file.name)+"."+std::to_string(ProcessId())+"-"+std::to_string(stamp)+".part")};
              const std::function<bool(std::uint64_t)> report=[&,index](std::uint64_t bytes) {
                if(canceled || failed) return false;
                Report(index,bytes);return true;
              };
              // Retry transient transport errors; verified completed scans survive failures.
              for(int attempt=0;;++attempt) {
                try {
                  if(transfer) transfer(file,temporary.path,report);
                  else {if(!session) session=std::make_unique<CurlSession>();session->Fetch(example,file,temporary.path,report);}
                  break;
                } catch(...) {if(transfer || attempt>=2 || canceled || failed) throw;}
              }
              if(canceled || failed) break;
              if(!Matches(temporary.path,file)) throw std::runtime_error(std::string("Checksum mismatch: ")+file.name);
              auto normalized_path=temporary.path;normalized_path+=".xyzi.part";PartialFile normalized{normalized_path};
              const auto * verified=&temporary.path;
              if(file.xyzi_sha256) {
                NormalizeXyzi(temporary.path,normalized.path,canceled);
                if(!Matches(normalized.path,cached_file)) throw std::runtime_error("Normalized scan checksum mismatch");
                verified=&normalized.path;
              }
              // Another app instance may have finished this scan while we downloaded.
              if(!Matches(destination,cached_file)) {
                std::error_code ec;fs::remove(destination,ec);fs::rename(*verified,destination);
              }
            }
            Report(index,file.bytes,cached);
            {std::lock_guard<std::mutex> lock(mutex);++progress.files;if(cached) ++progress.cached;}
          }
        } catch(const std::exception & error) {
          failed=true;std::lock_guard<std::mutex> lock(mutex);if(progress.error.empty()) progress.error=error.what();
        }
      };
      // Bound connections, buffers and temporary files, while reusing each HTTPS session.
      std::vector<std::thread> workers;
      try {for(int i=0;i<(kind==ExampleKind::Quick ? 1 : 4);++i) workers.emplace_back(work);}
      catch(...) {failed=true;for(auto & thread:workers) thread.join();throw;}
      for(auto & thread:workers) thread.join();
      std::lock_guard<std::mutex> lock(mutex);
      progress.stage=canceled ? ExampleStage::Canceled : failed ? ExampleStage::Failed : ExampleStage::Ready;
      if(progress.stage==ExampleStage::Ready) progress.eta_seconds=0;
      if(canceled) progress.error.clear();
    } catch(const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex);progress.stage=canceled ? ExampleStage::Canceled : ExampleStage::Failed;
      if(!canceled) progress.error=error.what();
    }
  }
};
ExampleDataset::ExampleDataset(fs::path directory,ExampleTransfer transfer):ExampleDataset(ExampleKind::Quick,std::move(directory),std::move(transfer)) {}
ExampleDataset::ExampleDataset(ExampleKind kind,fs::path directory,ExampleTransfer transfer):impl_(std::make_unique<Impl>()) {
  impl_->kind=kind;impl_->directory=directory.empty() ? DrivingExampleDirectory(kind) : std::move(directory);impl_->transfer=std::move(transfer);
}
ExampleDataset::~ExampleDataset() {Cancel();if(impl_->worker.joinable()) impl_->worker.join();}
bool ExampleDataset::Start() {
  if(Progress().Active()) return false;
  if(impl_->worker.joinable()) impl_->worker.join();
  impl_->canceled=false;impl_->failed=false;impl_->downloaded=0;impl_->started=std::chrono::steady_clock::now();
  const auto & example=Definition();impl_->received.assign(example.count,0);
  {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->progress={};impl_->progress.stage=ExampleStage::Checking;
    impl_->progress.total_files=example.count;impl_->progress.total_bytes=example.TotalBytes();}
  try {impl_->worker=std::thread([this]{impl_->Run();});}
  catch(...) {std::lock_guard<std::mutex> lock(impl_->mutex);impl_->progress.stage=ExampleStage::Failed;throw;}
  return true;
}
const ExampleDefinition & ExampleDataset::Definition() const {return DrivingExample(impl_->kind);}

void ExampleDataset::Cancel() {impl_->canceled=true;}
ExampleProgress ExampleDataset::Progress() const {std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->progress;}
fs::path ExampleDataset::Directory() const {return impl_->directory;}
} // namespace lidar_viewer
