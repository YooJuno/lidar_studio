#include "lidar_viewer/example_dataset.hpp"
#include "lidar_viewer/platform.hpp"
#include "lidar_viewer/settings_io.hpp"
#include "example_archive.hpp"
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
namespace {
constexpr auto kArchiveReady=".example-ready";
bool ArchiveReady(const fs::path & directory) {
  std::error_code error;
  return fs::is_regular_file(directory/kArchiveReady,error);
}
}
const ExampleDefinition * FindExampleAtPath(const fs::path & path) {
  std::error_code error;
  const auto absolute=fs::absolute(path,error);if(error) return nullptr;
  const auto selected=fs::weakly_canonical(absolute,error);if(error) return nullptr;
  for(const auto & example:DrivingExamples()) {
    const auto root=fs::weakly_canonical(DrivingExampleDirectory(example.kind),error);if(error) continue;
    if(selected==root || selected.parent_path()==root) return &example;
  }
  return nullptr;
}
bool IsDrivingExample(const fs::path & path) {return FindExampleAtPath(path)!=nullptr;}
bool IsIncompleteExample(const fs::path & path) {
  const auto * example=FindExampleAtPath(path);
  return example && example->archive && !ArchiveReady(DrivingExampleDirectory(example->kind));
}
ExampleCacheInfo InspectExampleCache(ExampleKind kind) {
  ExampleCacheInfo info;const auto directory=DrivingExampleDirectory(kind);const auto & example=DrivingExample(kind);
  const auto * files=example.archive ? example.archive_members : example.files;
  const auto count=example.archive ? example.archive_member_count : example.count;
  for(std::size_t i=0;i<count;++i) {
    std::error_code error;const auto bytes=fs::file_size(directory/files[i].name,error);
    if(!error) {
      info.present=true;
      info.bytes+=bytes;
      if(!example.archive || bytes==files[i].bytes) ++info.files;
    }
  }
  if(example.archive) info.files=info.files==count && ArchiveReady(directory) ? example.count : 0;
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
      if(example.archive) fs::remove(directory/kArchiveReady);
      for(std::size_t i=0;i<example.count;++i) fs::remove(directory/example.files[i].name);
      for(std::size_t i=0;i<example.archive_member_count;++i) fs::remove(directory/example.archive_members[i].name);
      if(!example.archive) for(const auto & entry:fs::directory_iterator(directory)) {
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
    curl_easy_setopt(curl,CURLOPT_CONNECTTIMEOUT_MS,5000L);
    curl_easy_setopt(curl,CURLOPT_TIMEOUT_MS,example.archive ? 900000L : 90000L);
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
struct ArchiveStage {fs::path path;~ArchiveStage() {std::error_code ec;fs::remove_all(path,ec);}};

fs::path TemporaryPrefix(const fs::path & directory,const ExampleFile & file) {
  const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
  return directory/(std::string(file.name)+"."+std::to_string(ProcessId())+"-"+std::to_string(stamp));
}
}
struct ExampleDataset::Impl {
  fs::path directory;ExampleKind kind{ExampleKind::Quick};ExampleTransfer transfer;mutable std::mutex mutex;
  ExampleProgress progress;std::atomic_bool canceled{false},failed{false};std::thread worker;
  std::vector<std::uint64_t> received;
  std::uint64_t downloaded{0};std::chrono::steady_clock::time_point started;
  bool Continue() const {return !canceled && !failed;}
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
  bool Fetch(const ExampleDefinition & example,const ExampleFile & file,std::size_t index,
    const fs::path & path,std::unique_ptr<CurlSession> & session)
  {
    {std::lock_guard<std::mutex> lock(mutex);progress.stage=ExampleStage::Downloading;}
    const auto report=[&,index](std::uint64_t bytes) {
      if(!Continue()) return false;
      Report(index,bytes);return true;
    };
    for(int attempt=0;;++attempt) {
      try {
        if(transfer) transfer(file,path,report);
        else {
          if(!session) session=std::make_unique<CurlSession>();
          session->Fetch(example,file,path,report);
        }
        break;
      } catch(...) {if(transfer || attempt>=2 || !Continue()) throw;}
    }
    if(!Continue()) return false;
    if(!Matches(path,file)) throw std::runtime_error(std::string("Checksum mismatch: ")+file.name);
    return Continue();
  }

  void RunArchive(const ExampleDefinition & example) {
    const auto & archive=*example.files;
    const auto * begin=example.archive_members;
    const auto * end=begin+example.archive_member_count;
    const bool cached=std::all_of(begin,end,[&](const ExampleFile & member) {
      return Continue() && Matches(directory/member.name,member);
    });
    if(!cached) {
      if(!Continue()) return;
      fs::remove(directory/kArchiveReady);
      auto prefix=TemporaryPrefix(directory.parent_path(),archive);
      auto zip_path=prefix;zip_path+=".part";
      PartialFile zip{std::move(zip_path)};
      std::unique_ptr<CurlSession> session;
      if(!Fetch(example,archive,0,zip.path,session)) return;
      Report(0,archive.bytes);
      {std::lock_guard<std::mutex> lock(mutex);progress.stage=ExampleStage::Preparing;}
      prefix+=".extract.part";
      if(!fs::create_directory(prefix)) throw std::runtime_error("Cannot create example staging directory");
      ArchiveStage stage{std::move(prefix)};
      UnpackExampleArchive(zip.path,stage.path,begin,example.archive_member_count,[this]{return Continue();});
      for(auto member=begin;member!=end;++member) {
        if(!Continue()) return;
        if(!Matches(stage.path/member->name,*member))
          throw std::runtime_error(std::string("Unpacked example checksum mismatch: ")+member->name);
      }
      if(!Continue()) return;
      for(auto member=begin;member!=end;++member)
        fs::rename(stage.path/member->name,directory/member->name);
    }
    if(!ArchiveReady(directory)) settings::AtomicWriteText(directory/kArchiveReady,archive.sha256);
    Report(0,archive.bytes,cached);
    std::lock_guard<std::mutex> lock(mutex);
    progress.files=1;progress.cached=cached ? 1 : 0;
  }

  void Run() {
    try {
      fs::create_directories(directory);const auto & example=DrivingExample(kind);
      if(example.archive) RunArchive(example);
      else RunScans(example);
      std::lock_guard<std::mutex> lock(mutex);
      progress.stage=canceled ? ExampleStage::Canceled : failed ? ExampleStage::Failed : ExampleStage::Ready;
      if(progress.stage==ExampleStage::Ready) progress.eta_seconds=0;
      if(canceled) progress.error.clear();
    } catch(const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex);progress.stage=canceled ? ExampleStage::Canceled : ExampleStage::Failed;
      if(!canceled) progress.error=error.what();
    }
  }

  void RunScans(const ExampleDefinition & example) {
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
            auto path=TemporaryPrefix(directory,file);path+=".part";
            PartialFile temporary{std::move(path)};
            if(!Fetch(example,file,index,temporary.path,session)) break;
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
