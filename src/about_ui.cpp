#include "lidar_viewer/about_ui.hpp"
#include "lidar_viewer/app_info.hpp"
#include "lidar_viewer/config.hpp"
#include "lidar_viewer/detector_setup.hpp"
#include <imgui.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace lidar_viewer {
namespace {
using Document=std::pair<std::string,std::filesystem::path>;
std::vector<Document> LicenseDocuments() {
  namespace fs=std::filesystem;
  std::vector<Document> result;
  const auto add=[&](const std::string & name,const fs::path & path) {
    if(fs::is_regular_file(path)) result.emplace_back(name,path);
  };
  add("LiDAR Studio / MIT",ProjectResourcePath("LICENSE"));
  add("Third-party notices",ProjectResourcePath("THIRD_PARTY_NOTICES.md"));
  auto directory=ProjectResourcePath("third-party-licenses");
  if(!fs::is_directory(directory)) directory=ProjectResourcePath("third_party/licenses");
  if(fs::is_directory(directory)) for(const auto & file:fs::directory_iterator(directory))
    add(file.path().filename().u8string(),file.path());
  add("FugSeg / MIT",ProjectResourcePath("third_party/fugseg/LICENSE"));
  add("GSeg3D / BSD-3-Clause",ProjectResourcePath("third_party/ground_segmentation/LICENSE.txt"));
  add("Patchwork++ / BSD-2-Clause",ProjectResourcePath("third_party/patchwork_plusplus/LICENSE"));
  const auto data=DetectorSetup::DataDirectory();
  for(const char * folder:{"centerpoint-tiny-v4.1","centerpoint-shortrange-v2.0.0"}) {
    add(std::string(folder)+" / model card",data/folder/"MODEL_CARD.md");
    add(std::string(folder)+" / license",data/folder/"LICENSE");
  }
  // Keep upstream package license files intact and make installed runtime
  // notices readable without opening a terminal or leaving the application.
  const auto runtime=data/"app-runtime/lib";
  if(fs::is_directory(runtime)) for(const auto & file:fs::recursive_directory_iterator(runtime,fs::directory_options::skip_permission_denied)) {
    if(result.size()>=180) break;
    const auto relative=fs::relative(file.path(),runtime).generic_u8string(),name=file.path().filename().u8string();
    if(relative.find(".dist-info/")!=std::string::npos &&
      (name.rfind("LICENSE",0)==0 || name.rfind("COPYING",0)==0 || name.rfind("NOTICE",0)==0))
      add("Runtime / "+relative,file.path());
  }
  if(result.size()>2) std::sort(result.begin()+2,result.end(),[](const auto & a,const auto & b){return a.first<b.first;});
  return result;
}
std::string ReadLicense(const std::filesystem::path & path) {
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file) return "This license document is unavailable.";
  const auto length=file.tellg();
  if(length<0 || length>2*1024*1024) return "This document exceeds the license viewer's size limit.";
  file.seekg(0);std::string text(static_cast<std::size_t>(length),'\0');file.read(text.data(),length);return text;
}
}
void DrawAboutDialog(bool & open) {
  static bool previously_open=false;
  static std::vector<Document> documents;
  static int selected=0,loaded=-1;
  static std::string content,error;
  if(!open) {previously_open=false;return;}
  if(!previously_open) {
    documents.clear();error.clear();selected=0;loaded=-1;
    try {documents=LicenseDocuments();} catch(const std::exception & exception) {error=exception.what();}
  }
  previously_open=true;
  const float scale=ImGui::GetFontSize()/15.f;
  const auto * viewport=ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->GetCenter(),ImGuiCond_Appearing,{.5f,.5f});
  ImGui::SetNextWindowSize({std::min(640.f*scale,viewport->WorkSize.x-24),std::min(610.f*scale,viewport->WorkSize.y-24)},ImGuiCond_Appearing);
  if(ImGui::Begin("About LiDAR Studio",&open,ImGuiWindowFlags_NoCollapse)) {
    if(ImGui::BeginTabBar("AboutTabs")) {
      if(ImGui::BeginTabItem("About")) {
        ImGui::Spacing();ImGui::SeparatorText("LiDAR Studio");
        ImGui::TextWrapped("A desktop workspace for point clouds, ground segmentation, object detection and dataset preparation.");
        ImGui::Spacing();ImGui::Text("Version %s",app_info::Version);
        ImGui::TextWrapped("Revision: %s",app_info::Revision);
        ImGui::Text("Platform: %s",app_info::Platform);
        ImGui::Text("Built: %s",app_info::Built);
        ImGui::Spacing();ImGui::SeparatorText("Developer & contact");
        ImGui::TextUnformatted(app_info::Developer);
        ImGui::TextLinkOpenURL(app_info::Email,"mailto:juno980220@naver.com");
        ImGui::SameLine();if(ImGui::SmallButton("Copy email")) ImGui::SetClipboardText(app_info::Email);
        ImGui::TextLinkOpenURL("GitHub repository",app_info::Repository);
        ImGui::TextLinkOpenURL("Report an issue","https://github.com/YooJuno/lidar_studio/issues");
        ImGui::Spacing();ImGui::SeparatorText("License");
        ImGui::TextUnformatted(app_info::License);
        ImGui::TextWrapped("Third-party code, models and example data keep their own licenses. Dataset-derived demo media may have noncommercial restrictions. See Licenses for the original notices.");
        ImGui::Spacing();ImGui::TextDisabled("Copyright (c) 2026 Yoo Junho");
        if(ImGui::Button("Copy build information")) {
          const std::string information=std::string("LiDAR Studio ")+app_info::Version+"\nRevision: "+app_info::Revision+
            "\nPlatform: "+app_info::Platform+"\nBuilt: "+app_info::Built+"\nDeveloper: "+app_info::Developer+"\nContact: "+app_info::Email;
          ImGui::SetClipboardText(information.c_str());
        }
        ImGui::EndTabItem();
      }
      if(ImGui::BeginTabItem("Licenses")) {
        ImGui::TextWrapped("Project code is MIT unless a file states otherwise. The original third-party terms below remain in effect.");
        if(!error.empty()) ImGui::TextWrapped("%s",error.c_str());
        if(documents.empty()) ImGui::TextWrapped("License documents are unavailable in this build.");
        else {
          ImGui::SetNextItemWidth(-1);
          if(ImGui::BeginCombo("##LicenseDocument",documents[selected].first.c_str())) {
            for(std::size_t i=0;i<documents.size();++i) if(ImGui::Selectable(documents[i].first.c_str(),selected==static_cast<int>(i))) selected=static_cast<int>(i);
            ImGui::EndCombo();
          }
          if(loaded!=selected) {content=ReadLicense(documents[selected].second);loaded=selected;}
          ImGui::BeginChild("LicenseText",{0,-40*scale},ImGuiChildFlags_Borders);
          ImGui::PushTextWrapPos(0);ImGui::TextUnformatted(content.c_str());ImGui::PopTextWrapPos();ImGui::EndChild();
        }
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    if(ImGui::Button("Close")) open=false;
  }
  ImGui::End();
}
} // namespace lidar_viewer
