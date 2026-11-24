#include "lidar_viewer/example_dataset.hpp"

#include <iterator>
#include <stdexcept>

namespace lidar_viewer {
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
constexpr ExampleFile urban_files[]{
#include "example_urbanv2x.inc"
};
constexpr ExampleFile hdl32_files[]{
#include "example_hdl32.inc"
};
constexpr ExampleFile at128_files[]{
#include "example_at128.inc"
};
constexpr auto urban_source="TAS Lab, Hong Kong Polytechnic University";
constexpr auto urban_license="CC BY-NC-SA 4.0 (non-commercial)";
constexpr auto hdl32_url="https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/hdl32/";
constexpr auto xt32_url="https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_01/lidar/xt32/";
constexpr auto at128_url="https://huggingface.co/datasets/polyutaslab/UrbanV2X/resolve/71a5320fd03b42026fd2b6154ae9e84653c9de08/hkstp_s_02/lidar/at128p/";
constexpr auto ouster_url="https://data.ouster.io/sdk-samples/Rev-06-fw23/";
constexpr auto ouster_license="CC BY-NC-SA 2.0 (non-commercial)";
constexpr DisplayAlignment xt32_mount{-178.28761f,.30224f,21.37109f};
constexpr DisplayAlignment hdl32_mount{1.99441f,.04641f,.15235f};
constexpr DisplayAlignment at128_mount{90.57078f,.71492f,-.09157f};

constexpr ExampleFile os0_zip{"OS0-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip",158506486,
  "a14a9ece4829ee7e3536133322ca2b18ece8dbe5a2e2a1025dd552b41b614da6"};
constexpr ExampleFile os0_files[]{
  {"lidar.pcap",425779826,"8d5f2f86a3a18e5dc64477dfc9b097dbdd3cb2fef91975b15a4bb1451e2fceec",nullptr,"OS0-128_Rev-06_fw23_Urban-Drive_Dual-Returns.pcap"},
  {"lidar.json",7285,"0139087681e5a109a0189d5ce61246d6f0b01d4ab887f7b0ef91b237238b095b",nullptr,"OS0-128_Rev-06_fw23_Urban-Drive_Dual-Returns.json"}
};
constexpr ExampleFile os1_zip{"OS1-128_Rev-06_fw23_Urban-Drive_Low-Bandwidth.zip",99006397,
  "ce3fbe0729f6335ad6fc405ded3de61243cb05585899e46986633c6ba90dfd76"};
constexpr ExampleFile os1_files[]{
  {"lidar.pcap",140000082,"25a5fa30e05c41ea986ce72e7d72de818f65a767fbfc4c148139459a3ea4f928",nullptr,"OS1-128_Rev-06_fw23_Urban-Drive_Low-Bandwidth.pcap"},
  {"lidar.json",7187,"3df9e8ebbd62c79aaea34981b0c838d6ada847d9f9b5d0af95770a4036057e49",nullptr,"OS1-128_Rev-06_fw23_Urban-Drive_Low-Bandwidth.json"}
};
constexpr ExampleFile os2_zip{"OS2-128_Rev-06_fw23_Urban-Drive_Dual-Returns.zip",200034559,
  "80b9bb2a045c41d7debecb0be83edf944babf5ada8e0fe2c6aab536c4102cfac"};
constexpr ExampleFile os2_files[]{
  {"lidar.pcap",425779826,"508cddfcc3c67b553161f13b55970a9d41f6c2b1cb71defc7562243926f04177",nullptr,"OS2-128_Rev-06_fw23_Urban-Drive_Dual-Returns.pcap"},
  {"lidar.json",7138,"3bc686fd6659b9a935578bf280cff361a71063c087ad18b71bc3e44d75ec3bca",nullptr,"OS2-128_Rev-06_fw23_Urban-Drive_Dual-Returns.json"}
};
}

std::uint64_t ExampleDefinition::TotalBytes() const {
  std::uint64_t bytes=0;for(std::size_t i=0;i<count;++i) bytes+=files[i].bytes;return bytes;
}

const std::array<ExampleDefinition,10> & DrivingExamples() {
  static const std::array<ExampleDefinition,10> examples{{
    {ExampleKind::Quick,"driving-v1","Quick preview | 6 frames","Patchwork++ public sample",
      "https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/data/",DrivingExampleFiles().data(),6,"Quick preview",0,"example",
      "Patchwork++","Sample scans","","",0,6,"example_quick"},
    {ExampleKind::Urban30,"urbanv2x-xt32-30s-v1","UrbanV2X XT32 | 300 frames | 0:30",urban_source,
      xt32_url,urban_files,300,"Hesai XT32",32,"example-xt32","UrbanV2X","Hong Kong Science Park",urban_license,"360 deg",10,300,"example_xt32",
      false,nullptr,0,xt32_mount,"LiDAR to published IMU axes"},
    {ExampleKind::Urban60,"urbanv2x-xt32-60s-v1","UrbanV2X XT32 | 600 frames | 1:00",urban_source,
      xt32_url,urban_files,600,"Hesai XT32",32,"example-xt32","UrbanV2X","Hong Kong Science Park",urban_license,"360 deg",10,600,"example_xt32",
      false,nullptr,0,xt32_mount,"LiDAR to published IMU axes"},
    {ExampleKind::Velodyne30,"urbanv2x-hdl32-30s-v1","UrbanV2X HDL-32E | 300 frames | 0:30",urban_source,
      hdl32_url,hdl32_files,300,"Velodyne HDL-32E",32,"example-hdl32","UrbanV2X","Hong Kong Science Park",urban_license,"360 deg",10,300,"example_hdl32",
      false,nullptr,0,hdl32_mount,"LiDAR to published IMU axes"},
    {ExampleKind::Velodyne60,"urbanv2x-hdl32-60s-v1","UrbanV2X HDL-32E | 600 frames | 1:00",urban_source,
      hdl32_url,hdl32_files,600,"Velodyne HDL-32E",32,"example-hdl32","UrbanV2X","Hong Kong Science Park",urban_license,"360 deg",10,600,"example_hdl32",
      false,nullptr,0,hdl32_mount,"LiDAR to published IMU axes"},
    {ExampleKind::At12830,"urbanv2x-at128-30s-v1","UrbanV2X AT128P | 300 frames | 0:30",urban_source,
      at128_url,at128_files,300,"Hesai AT128P",128,"example-at128","UrbanV2X","Hong Kong Science Park",urban_license,"120 deg forward",10,300,"example_at128",
      false,nullptr,0,at128_mount,"LiDAR to published IMU axes"},
    {ExampleKind::At12860,"urbanv2x-at128-60s-v1","UrbanV2X AT128P | 600 frames | 1:00",urban_source,
      at128_url,at128_files,600,"Hesai AT128P",128,"example-at128","UrbanV2X","Hong Kong Science Park",urban_license,"120 deg forward",10,600,"example_at128",
      false,nullptr,0,at128_mount,"LiDAR to published IMU axes"},
    {ExampleKind::OusterOs0,"ouster-os0-urban-v1","Ouster OS0-128 | Urban drive | 201 frames","Ouster",
      ouster_url,&os0_zip,1,"Ouster OS0-128",128,"example-os0","Ouster SDK","Urban drive / wide view",ouster_license,"360 deg / dual returns",10,201,"example_os0",true,os0_files,std::size(os0_files)},
    {ExampleKind::OusterOs1,"ouster-os1-urban-v1","Ouster OS1-128 | Urban drive | 258 frames","Ouster",
      ouster_url,&os1_zip,1,"Ouster OS1-128",128,"example-os1","Ouster SDK","Urban drive / low bandwidth",ouster_license,"360 deg",10,258,"example_os1",true,os1_files,std::size(os1_files)},
    {ExampleKind::OusterOs2,"ouster-os2-urban-v1","Ouster OS2-128 | Urban drive | 201 frames","Ouster",
      ouster_url,&os2_zip,1,"Ouster OS2-128",128,"example-os2","Ouster SDK","Urban drive / long range",ouster_license,"360 deg / dual returns",10,201,"example_os2",true,os2_files,std::size(os2_files)},
  }};
  return examples;
}

const ExampleDefinition & DrivingExample(ExampleKind kind) {
  for(const auto & example:DrivingExamples()) if(example.kind==kind) return example;
  throw std::invalid_argument("Unknown LiDAR example");
}

const ExampleDefinition * FindExampleProfile(std::string_view profile) {
  for(const auto & example:DrivingExamples()) if(profile==example.profile) return &example;
  return nullptr;
}

} // namespace lidar_viewer
