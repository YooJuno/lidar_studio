#include "lidar_viewer/example_dataset.hpp"
#include "lidar_viewer/platform.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

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
    NoPartials(failed.Directory());fs::remove_all(temporary);std::cout<<"Example cache, repair, cancel, retry and integrity checks passed\n";
  } catch(const std::exception & error) {std::cerr<<error.what()<<'\n';fs::remove_all(temporary);return 1;}
}
