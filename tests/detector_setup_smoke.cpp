#include "lidar_viewer/detector_setup.hpp"
#include "lidar_viewer/platform.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void Check(bool value,const char * message) {if(!value) throw std::runtime_error(message);}
void Environment(const char * key,const std::string & value) {
#ifdef _WIN32
  _putenv_s(key,value.c_str());
  // Keep the UTF-8 CRT fixture and the UTF-16 environment inherited by
  // CreateProcessW consistent, including the Korean test-directory name.
  Check(SetEnvironmentVariableW(std::filesystem::u8path(key).c_str(),std::filesystem::u8path(value).c_str())!=0,
    "Cannot set the Unicode helper environment");
#else
  setenv(key,value.c_str(),1);
#endif
}
std::string Read(const std::filesystem::path & path) {std::ifstream file(path);return {std::istreambuf_iterator<char>(file),{}};}
template<class F> void Wait(F condition) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(8);
  while(!condition()) {
    Check(std::chrono::steady_clock::now()<deadline,"Model setup did not finish without blocking");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
}
int main(int argc,char ** argv) {
  namespace fs=std::filesystem;using namespace lidar_viewer;
  fs::path root;
  try {
    Check(argc==2,"Pass a Python interpreter");root=TemporaryDirectory("lidar-setup-test-")/fs::u8path("model space 한글");fs::create_directories(root);
    Environment("XDG_CONFIG_HOME",(root/"settings").u8string());Environment("XDG_DATA_HOME",(root/"data").u8string());
    const auto config=UserConfigurationDirectory();fs::create_directories(config);
    const auto script=root/"setup 'literal'.py",mode=root/"mode",entered=root/"entered";
    std::ofstream(config/"detector.cfg")<<"previous configuration";
    std::ofstream(script)<<R"(import json,os,sys,time
from pathlib import Path
root=Path(__file__).parent
mode=(root/'mode').read_text()
(root/'entered').write_text(mode)
status=Path(sys.argv[sys.argv.index('--progress-file')+1])
status.write_text(json.dumps(dict(message='fixture downloading',bytes=4,total_bytes=10)))
print('fixture helper output',flush=True)
if mode=='slow':time.sleep(60)
if mode=='fail':raise SystemExit(7)
config=Path(os.environ['XDG_CONFIG_HOME'])/'lidar_viewer'
(config/'detector.cfg').write_text('new validated configuration')
)";
    DetectorSetup setup{fs::u8path(argv[1])};std::ofstream(mode)<<"slow";
    Check(setup.Start(root,script),"Preparation did not start");Check(!setup.Start(root,script),"Duplicate preparation started");
    Wait([&]{const auto progress=setup.Progress();
      if(progress.stage==DetectorSetupStage::Failed) throw std::runtime_error(progress.error+"\n"+progress.log);
      return fs::exists(entered) && progress.bytes==4;});
    Check(setup.Progress().Active(),"Preparation ran on the calling thread");
    {DetectorSetup concurrent{fs::u8path(argv[1])};concurrent.Start(root,script);
      Wait([&]{return !concurrent.Progress().Active();});
      Check(concurrent.Progress().stage==DetectorSetupStage::Failed,"Concurrent installer did not honor the model lock");}
    setup.Cancel();Wait([&]{return !setup.Progress().Active();});
    Check(setup.Progress().stage==DetectorSetupStage::Canceled,"Cancellation was reported as a model failure");
    Check(Read(config/"detector.cfg")=="previous configuration","Cancellation changed a working configuration");
    std::ofstream(mode)<<"fail";setup.Start(root,script);Wait([&]{return !setup.Progress().Active();});
    Check(setup.Progress().stage==DetectorSetupStage::Failed && setup.Progress().log.find("fixture helper output")!=std::string::npos,"Installer failure/logs were lost");
    Check(Read(config/"detector.cfg")=="previous configuration","Failure changed a working configuration");
    std::ofstream(mode)<<"ready";setup.Start(root,script);Wait([&]{return !setup.Progress().Active();});
    Check(setup.Progress().stage==DetectorSetupStage::Ready && Read(config/"detector.cfg")=="new validated configuration","Retry did not publish the successful setup");
    Check(DetectorSetup::DataDirectory()==root/"data/lidar_viewer/detector","Data directory ignored its override");
    fs::remove_all(root.parent_path());std::cout<<"Detector setup: background progress, exclusive install, cancellation, failure preservation and retry passed\n";return 0;
  } catch(const std::exception & error) {std::cerr<<error.what()<<'\n';if(!root.empty()) fs::remove_all(root.parent_path());return 1;}
}
