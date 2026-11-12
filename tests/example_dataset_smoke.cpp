#include "lidar_viewer/example_dataset.hpp"
#include "lidar_viewer/platform.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <set>
#include <cstring>
#include <cstdlib>

using namespace lidar_viewer;
namespace fs=std::filesystem;
namespace {
void Check(bool condition,const char * message) {if(!condition) throw std::runtime_error(message);}
void Finish(ExampleDataset & dataset) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  std::uint64_t last=0;
  while(dataset.Progress().Active()) {
    const auto status=dataset.Progress();Check(status.bytes>=last,"Progress moved backwards");last=status.bytes;
    Check(std::chrono::steady_clock::now()<deadline,"Example job did not finish");std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
void NoPartials(const fs::path & folder) {for(const auto & entry:fs::directory_iterator(folder)) Check(entry.path().extension()!=".part","A partial file survived");}
}
int main(int argc,char ** argv) {
  const auto temporary=TemporaryDirectory("lidar-example-");
  try {
    const auto & short_drive=DrivingExample(ExampleKind::Urban30);
    const auto & long_drive=DrivingExample(ExampleKind::Urban60);
    Check(short_drive.count==300 && long_drive.count==600,"Wrong continuous recording lengths");
    Check(short_drive.TotalBytes()==921600000 && long_drive.TotalBytes()==1843200000,"Download sizes disagree with manifest");
    Check(short_drive.files==long_drive.files,"Recording lengths do not share a contiguous prefix");
    Check(DrivingExampleDirectory(ExampleKind::Quick)!=DrivingExampleDirectory(ExampleKind::Urban30) &&
      DrivingExampleDirectory(ExampleKind::Urban30)!=DrivingExampleDirectory(ExampleKind::Urban60),"Example caches overlap");
    for(const auto & example:DrivingExamples()) {
      if(example.kind==ExampleKind::Quick) continue;
      Check(example.channels==32 || example.channels==128,"Missing sensor channel information");
      Check(example.count==300 || example.count==600,"Unexpected driving sample length");
      std::set<std::string> names;std::uint64_t prior=0;
      for(std::size_t i=0;i<example.count;++i) {
        const auto & file=example.files[i];
        Check(fs::path(file.name).filename()==fs::path(file.name) && names.insert(file.name).second,"Unsafe or duplicate scan path");
        Check(file.bytes%24==0 && file.bytes>0 && std::strlen(file.sha256)==64 && file.xyzi_sha256 && std::strlen(file.xyzi_sha256)==64,"Invalid six-column scan manifest");
        const auto stamp=std::stoull(fs::path(file.name).stem().string());
        Check(!prior || (stamp>prior && stamp-prior>90000 && stamp-prior<110000),"Manifest is not consecutive 10 Hz scans");prior=stamp;
      }
    }
    Check(IsDrivingExample(DrivingExampleDirectory(ExampleKind::Urban30)/long_drive.files[0].name),"Long example not recognized");
    Check(!IsDrivingExample(temporary/long_drive.files[0].name),"Unrelated data treated as an example");
    Check(argc==2,"Pass the upstream sample directory");const fs::path source=fs::u8path(argv[1]);std::atomic<int> transfers{0};
    const auto copy=[&](const ExampleFile & file,const fs::path & path,const auto & progress) {
      ++transfers;if(!progress(0)) throw std::runtime_error("canceled");fs::copy_file(source/file.name,path);progress(file.bytes);
    };
    const auto folder=temporary/fs::u8path("sample space 한글");ExampleDataset dataset(folder,copy);
    Check(dataset.Start(),"Initial download did not start");Finish(dataset);auto status=dataset.Progress();
    Check(status.stage==ExampleStage::Ready && status.files==6 && status.cached==0 && transfers==6,"Valid downloads were not published");
    Check(status.bytes==status.total_bytes,"Completed byte count differs");NoPartials(folder);
    const auto changed=fs::last_write_time(folder/DrivingExampleFiles()[0].name);
    dataset.Start();Finish(dataset);status=dataset.Progress();
    Check(status.stage==ExampleStage::Ready && status.cached==6 && transfers==6,"Cache was downloaded twice");
    Check(fs::last_write_time(folder/DrivingExampleFiles()[0].name)==changed,"Cache hit rewrote a scan");
    {std::fstream damaged(folder/DrivingExampleFiles()[2].name,std::ios::binary|std::ios::in|std::ios::out);damaged.put('!');}
    dataset.Start();Finish(dataset);status=dataset.Progress();
    Check(status.stage==ExampleStage::Ready && status.cached==5 && transfers==7,"Same-size corruption was not repaired selectively");
    Check(fs::last_write_time(folder/DrivingExampleFiles()[0].name)==changed,"Repair touched another valid scan");
    bool corrupt=true;
    ExampleDataset retry(temporary/"retry",[&](const ExampleFile & file,const fs::path & path,const auto & progress) {
      if(corrupt) {std::ofstream output(path,std::ios::binary);output<<"not a point cloud";progress(17);}else copy(file,path,progress);
    });
    retry.Start();Finish(retry);Check(retry.Progress().stage==ExampleStage::Failed && !fs::exists(retry.Directory()/"000000.bin"),"Invalid content was exposed as a scan");NoPartials(retry.Directory());
    corrupt=false;retry.Start();Finish(retry);Check(retry.Progress().stage==ExampleStage::Ready,"Retry after checksum failure did not recover");
    std::atomic_bool waiting{false};
    ExampleDataset canceled(temporary/"cancel",[&](const ExampleFile &,const fs::path & path,const auto & progress) {
      {std::ofstream partial(path);partial<<"partial";}waiting=true;
      while(progress(0)) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });
    canceled.Start();const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!waiting) {Check(std::chrono::steady_clock::now()<deadline,"Transfer did not enter");std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    Check(!canceled.Start(),"Duplicate click started another download");
    canceled.Cancel();Finish(canceled);Check(canceled.Progress().stage==ExampleStage::Canceled,"Cancellation became a download error");NoPartials(canceled.Directory());
    Check(!fs::exists(canceled.Directory()/"000000.bin"),"A canceled scan became visible");
    ExampleDataset failed(temporary/"failed",[](const auto &,const auto &,const auto &){throw std::runtime_error("network unavailable");});
    failed.Start();Finish(failed);Check(failed.Progress().stage==ExampleStage::Failed && failed.Progress().error=="network unavailable","Network error was lost");
    NoPartials(failed.Directory());
    std::atomic<int> active{0};
    ExampleDataset parallel(ExampleKind::Urban30,temporary/"parallel",[&](const auto &,const fs::path & path,const auto & progress) {
      {std::ofstream partial(path);partial<<"pending";}++active;
      while(progress(0)) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    });
    parallel.Start();const auto parallel_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(active<4) {Check(std::chrono::steady_clock::now()<parallel_deadline,"Parallel download pool did not start");std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    Check(active==4 && !parallel.Start(),"Connections were not bounded or duplicate job started");
    parallel.Cancel();Finish(parallel);Check(parallel.Progress().stage==ExampleStage::Canceled,"Parallel cancellation failed");
    Check(parallel.Progress().files==0 && parallel.Progress().total_files==300,"Canceled downloads published a scan");NoPartials(parallel.Directory());
    // Isolate the app-owned cache APIs from the real user's downloaded examples.
    {
      const char * previous=std::getenv("XDG_DATA_HOME");const std::string original=previous ? previous : "";const bool had_original=previous!=nullptr;
      struct RestoreEnvironment {
        const std::string & value;bool had;
        ~RestoreEnvironment() {
#ifdef _WIN32
          _putenv_s("XDG_DATA_HOME",had ? value.c_str() : "");
#else
          if(had) setenv("XDG_DATA_HOME",value.c_str(),1);else unsetenv("XDG_DATA_HOME");
#endif
        }
      } restore{original,had_original};
      const auto data=temporary/"isolated-cache";
#ifdef _WIN32
      _putenv_s("XDG_DATA_HOME",data.u8string().c_str());
#else
      setenv("XDG_DATA_HOME",data.u8string().c_str(),1);
#endif
      const auto small=DrivingExampleDirectory(ExampleKind::Urban30),large=DrivingExampleDirectory(ExampleKind::Urban60);
      fs::create_directories(small);fs::create_directories(large);
      const auto name=long_drive.files[0].name;{std::ofstream output(small/name);output<<"shared scan";}
      fs::create_hard_link(small/name,large/name);
      const auto info=InspectExampleCache(ExampleKind::Urban30);Check(info.files==1 && info.bytes==11,"Saved cache info is incorrect");
      DeleteExampleCache(ExampleKind::Urban30);Check(!fs::exists(small) && fs::file_size(large/name)==11,"Deleting one length removed shared data in another");
      Check(InspectExampleCache(ExampleKind::Urban30).files==0,"Deleted example is still listed as saved");
      DeleteExampleCache(ExampleKind::Urban30);DeleteExampleCache(ExampleKind::Urban60);Check(!fs::exists(large),"Cache deletion did not remove its directory");
      fs::create_directories(small/"exports");
      {std::ofstream output(small/name);output<<"downloaded";}
      {std::ofstream output(small/"exports/points.npy");output<<"user export";}
      {std::ofstream output(small/(std::string(name)+".123-456.part"));output<<"partial";}
      DeleteExampleCache(ExampleKind::Urban30);
      Check(fs::exists(small/"exports/points.npy") && !fs::exists(small/name) && !fs::exists(small/(std::string(name)+".123-456.part")),"Deleting downloads removed an export or left a download fragment");
    }
    fs::remove_all(temporary);std::cout<<"Example cache, repair, cancel, retry and integrity checks passed\n";
  } catch(const std::exception & error) {std::cerr<<error.what()<<'\n';fs::remove_all(temporary);return 1;}
}
