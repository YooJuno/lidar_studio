#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "lidar_viewer/live_io.hpp"
#include "lidar_viewer/pcap_io.hpp"
#include "lidar_packets.hpp"
#include "ouster_packets.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
using namespace lidar_viewer;
using Model = DirectLidarModel;

void Check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

struct DatagramSocket
{
  DatagramSocket()
  {
#ifdef _WIN32
    WSADATA data{};
    Check(WSAStartup(MAKEWORD(2,2), &data) == 0, "Winsock setup failed");
#endif
    handle=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    Check(handle != invalid, "UDP socket could not be opened");
  }
  ~DatagramSocket()
  {
#ifdef _WIN32
    closesocket(handle);
    WSACleanup();
#else
    close(handle);
#endif
  }
  std::uint16_t FreePort()
  {
    auto address = Address(0);
    Check(bind(handle, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0,"UDP test port unavailable");
#ifdef _WIN32
    int size = sizeof(address);
#else
    socklen_t size = sizeof(address);
#endif
    Check(getsockname(handle, reinterpret_cast<sockaddr *>(&address), &size) == 0,"Cannot read test port");
    return ntohs(address.sin_port);
  }
  void Send(const std::vector<std::uint8_t> & data,std::uint16_t port)
  {
    const auto address = Address(port);
    Check(sendto(handle,reinterpret_cast<const char *>(data.data()),static_cast<int>(data.size()),0,
        reinterpret_cast<const sockaddr *>(&address),sizeof(address)) == static_cast<int>(data.size()),
      "Could not send LiDAR packet");
  }
private:
  static sockaddr_in Address(std::uint16_t port)
  {
    sockaddr_in address{};
    address.sin_family=AF_INET;
    address.sin_port=htons(port);
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    return address;
  }
#ifdef _WIN32
  SOCKET handle{INVALID_SOCKET};
  static constexpr SOCKET invalid=INVALID_SOCKET;
#else
  int handle{-1};
  static constexpr int invalid=-1;
#endif
};

void Little(std::vector<std::uint8_t> & data, std::size_t offset, std::uint64_t value, unsigned bytes)
{
  for (unsigned i=0;i<bytes;++i) data[offset+i]=static_cast<std::uint8_t>(value>>(8*i));
}

std::vector<std::uint8_t> XtPacket(unsigned index, unsigned channels=32, unsigned blocks=8)
{
  const auto stride=2+4*channels;
  std::vector<std::uint8_t> data(12+blocks*stride+28);
  data[0]=0xee;data[1]=0xff;data[2]=6;data[3]=1;data[6]=static_cast<std::uint8_t>(channels);
  data[7]=static_cast<std::uint8_t>(blocks);data[9]=4;data[11]=1;
  for(unsigned block=0;block<blocks;++block) {
    const auto offset=12+block*stride;
    Little(data,offset,(index*blocks+block)*1000%36000,2);
    for(unsigned i=0;i<channels;++i) {
      Little(data,offset+2+4*i,2500+i,2);
      data[offset+4+4*i]=173;
    }
  }
  const auto tail = 12 + blocks * stride;
  const auto seconds = 1720000000u + index / 100;
  for (unsigned i = 0; i < 4; ++i) data[tail + 15 + i] = static_cast<std::uint8_t>(seconds >> (24 - 8 * i));
  Little(data, tail + 19, (index % 100) * 10000, 4);
  Little(data,data.size()-4,index,4);
  return data;
}

void ProtocolChecks()
{
  for(const auto entry : {std::pair{0x21,Model::VelodyneHdl32e},{0x22,Model::VelodyneVlp16},
      {0x24,Model::VelodynePuckHiRes},{0x28,Model::VelodyneVlp32c}}) {
    std::vector<std::uint8_t> data(1206);
    data[1204]=0x37;data[1205]=static_cast<std::uint8_t>(entry.first);
    for(unsigned block=0;block<12;++block) Little(data,block*100,0xeeff,2);
    Little(data,4,2500,2);data[6]=197;
    const auto packet=InspectLidarPacket(data.data(),data.size());
    Check(packet && packet->model==entry.second,"Velodyne product ID selected the wrong decoder");
    FrameData frame;
    LidarPacketDecoder decoder(packet->model);
    decoder.AppendBlock(frame,data.data(),*packet,0);
    Check(frame.points.size()==1 && frame.points.front().intensity==197,"Velodyne points lost reflectivity or zero filtering");
    data[1170]=12;data[0]=0;
    Check(!InspectLidarPacket(data.data(),data.size()),"Invalid Velodyne block accepted");
  }

  const auto xt16=XtPacket(0,16);
  const auto hesai=InspectLidarPacket(xt16.data(),xt16.size());
  Check(hesai && hesai->model==Model::HesaiXt16,"XT16 confused with XT32");
  Check(!InspectLidarPacket(xt16.data(),xt16.size()-1),"Truncated XT packet accepted");
  const auto xt32m2x=XtPacket(0,32,6);
  const auto extended=InspectLidarPacket(xt32m2x.data(),xt32m2x.size());
  Check(extended && extended->model==Model::HesaiXt32M2X && extended->blocks==6,"XT32M2X was misidentified");
  LidarSequence sequence;
  Check(sequence.Accept(0xffffffffu) && sequence.Accept(0u) && !sequence.Accept(0xffffffffu) &&
      sequence.Accept(3u) && sequence.missing==2,
    "XT sequence tracking failed on rollover, reordering or loss");

  std::vector<std::uint8_t> angles(48 + 16 + 128 * 368);
  angles[0]=0xee;angles[1]=0xff;angles[2]=1;angles[3]=5;
  angles[4]=128;angles[5]=2;angles[6]=2;angles[15]=1;
  Little(angles,20,180*25600,4);Little(angles,24,120*25600,4);Little(angles,28,300*25600,4);
  const HesaiAtCalibration at_calibration({reinterpret_cast<const char *>(angles.data()), angles.size()});
  std::vector<std::uint8_t> at(1118);
  at[0]=0xee;at[1]=0xff;at[2]=4;at[3]=3;at[6]=128;at[7]=2;at[9]=4;at[11]=1;
  Little(at,12,4500,2);Little(at,15,2500,2);at[17]=112;
  Little(at,527,22500,2);Little(at,530,2500,2);
  const auto at_packet=InspectLidarPacket(at.data(),at.size(),nullptr,&at_calibration);
  Check(at_packet && at_packet->model==Model::HesaiAt128,"AT128 was not identified");
  LidarSweep sectors;
  Check(!sectors.Crossed(*at_packet,0) && sectors.Crossed(*at_packet,1),"AT128 mirror boundary lost");
  FrameData at_cloud;
  LidarPacketDecoder at_decoder(Model::HesaiAt128,{},nullptr,&at_calibration);
  at_decoder.AppendBlock(at_cloud,at.data(),*at_packet,0);
  Check(at_cloud.points.size()==1 && std::abs(at_cloud.points.front().x-10.f)<1e-5f &&
      std::abs(at_cloud.points.front().y)<1e-5f && at_cloud.points.front().intensity==112,
    "AT128 calibration was not reflected in coordinates");

  for (const auto entry : {std::pair{1210,Model::RobosenseM1},{1342,Model::RobosenseM2},{1448,Model::RobosenseM3}}) {
    std::vector<std::uint8_t> rs(entry.first);
    rs[0]=0x55;rs[1]=0xaa;rs[2]=0x5a;rs[3]=0xa5;
    rs[34]=0x07;rs[35]=0xd0;
    const bool polar=entry.second==Model::RobosenseM1;
    if (polar) {rs[36]=0x80;rs[38]=0x80;rs[40]=128;}
    else {rs[36]=0x40;rs[42]=128;}
    const auto packet=InspectLidarPacket(rs.data(),rs.size());
    Check(packet && packet->model==entry.second,"RoboSense M-series model was misidentified");
    LidarPacketDecoder decoder(packet->model);
    FrameData scan;
    decoder.AppendBlock(scan,rs.data(),*packet,0);
    Check(scan.points.size()==1 && scan.points[0].intensity==128 &&
        std::abs(scan.points[0].x-(polar ? 10.f : 5.f))<1e-5f,"RoboSense packet decoded incorrectly");
  }

  std::vector<std::uint8_t> livox(50);
  Little(livox,1,livox.size(),2);Little(livox,5,1,2);livox[10]=1;
  Little(livox,28,1000000000,8);Little(livox,36,10000,4);
  Little(livox,40,0xfffffc18u,4);Little(livox,44,250,4);livox[48]=100;
  const auto mid360=InspectLidarPacket(livox.data(),livox.size());
  Check(mid360 && mid360->model==Model::LivoxMid360,"Mid-360 packet did not identify");
  LidarClock device_clock;
  auto next_mid = *mid360;
  next_mid.sensor_timestamp_ns += 100000000;
  Check(device_clock.AtFrame(*mid360,0,2000000000)==2000000000 &&
      device_clock.AtFrame(next_mid,0,2000004000)==2100000000 &&
      device_clock.AtFrame(*mid360,0,2000008000)==2100004000,
    "Frame clock ignored a sensor interval or accepted a backwards clock");
  FrameData cloud;
  LidarPacketDecoder mid_decoder(Model::LivoxMid360);
  mid_decoder.AppendBlock(cloud,livox.data(),*mid360,0);
  Check(cloud.points.size()==1 && std::abs(cloud.points.front().x-10.f)<1e-5f &&
      std::abs(cloud.points.front().y+1.f)<1e-5f && cloud.points.front().z==.25f,
    "Mid-360 coordinates did not preserve signed millimetres");

  OusterCalibration metadata(R"({
    "lidar_data_format":{"columns_per_frame":512,"columns_per_packet":1,"pixels_per_column":1,
      "udp_profile_lidar":"RNG19_RFL8_SIG16_NIR16_DUAL"},
    "beam_intrinsics":{"beam_altitude_angles":[0],"beam_azimuth_angles":[0],
      "beam_to_lidar_transform":[1,0,0,30,0,1,0,0,0,0,1,0,0,0,0,1]},
    "lidar_intrinsics":{"lidar_to_sensor_transform":[-1,0,0,0,0,-1,0,0,0,0,1,40,0,0,0,1]}
  })");
  std::vector<std::uint8_t> ouster(92);
  ouster[0]=1;ouster[42]=1;
  Little(ouster,44,6981,4);ouster[47]=121;
  Little(ouster,48,7304,4);ouster[51]=122;
  const auto decoded=metadata.Inspect(ouster.data(),ouster.size());
  Check(decoded.has_value(),"Valid Ouster format did not identify");
  FrameData scan;
  metadata.AppendColumn(scan,ouster.data(),*decoded,0);
  Check(scan.points.size()==2 && std::abs(scan.points[0].x+6.981f)<1e-5f &&
      std::abs(scan.points[0].z-.04f)<1e-5f && scan.points[1].intensity==122,
    "Ouster dual returns or sensor transform incorrect");
}

template<class Predicate> void WaitFor(Predicate ready, const char * failure)
{
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while (!ready()) {
    Check(std::chrono::steady_clock::now()<deadline,failure);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
}  // namespace

int main()
{
  fs::path temporary;
  try {
    ProtocolChecks();
    const auto unique=std::chrono::steady_clock::now().time_since_epoch().count();
    temporary=fs::temp_directory_path()/("lidar-udp-"+std::to_string(unique));
    Check(fs::create_directory(temporary),"Could not create test directory");
    std::uint16_t port, auxiliary;
    {
      DatagramSocket primary_test_port, auxiliary_test_port;
      port = primary_test_port.FreePort();
      auxiliary = auxiliary_test_port.FreePort();
    }
    DatagramSocket sender;
    LiveSession live;
    DirectLiveOptions options;
    options.udp_port=port;
    options.auxiliary_port=auxiliary;
    options.sensor_ipv4="127.0.0.1";
    live.ConnectDirect(options);
    const auto send=[&](unsigned first,unsigned last) {
      for(unsigned i=first;i<last;++i) sender.Send(XtPacket(i),port);
    };
    send(0,16);
    WaitFor([&]{return live.Stats().clouds>=2;},"No complete scan over native UDP");
    Check(live.Stats().sensor_model==Model::HesaiXt32,"Automatic selection missed XT32");
    Check(live.Stats().auxiliary_port==auxiliary,"Auxiliary port was not opened");
    const auto displayed=live.TakeLatest();
    Check(displayed && displayed->frame.points.size()==1152 && displayed->frame.source_intensities &&
       displayed->frame.source_intensities->at(0)==173,"UDP sweep assembly or reflectivity preservation failed");
    const auto folder=temporary/"capture";
    live.StartRecording(folder);
    const std::vector<std::uint8_t> raw_aux{0x37,0x08,0x29,0x99,0x00,0x02,0xab,0x42,0xaf};
    const std::vector<std::uint8_t> raw_unknown{0x72,0x19,0x23,0x45,0x67,0x89,0xcd,0xef};
    sender.Send(raw_aux,auxiliary);
    sender.Send(raw_unknown,port);
    send(16,35);
    WaitFor([&]{return live.Stats().recorded_messages==21;},"Recorder missed raw point or auxiliary UDP while the display stopped consuming");
    Check(!fs::exists(folder/kLivePcapFilename) && live.Stats().replaced_frames>0,
      "Recorder exposed an unfinished capture or retained a playback queue");
    live.StopRecording();
    WaitFor([&]{return !live.Stats().record_busy;},"PCAP finalization did not complete");
    const auto path=FindLidarPcap(folder);
    Check(!path.empty() && live.Stats().record_error.empty(),"No finalized PCAP");
    std::ifstream capture(path,std::ios::binary);
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(capture), {}};
    capture.close();
    const auto originals = XtPacket(20);
    for (const auto * payload : {&raw_aux, &raw_unknown, &originals}) {
      const auto record = std::search(bytes.begin(),bytes.end(),payload->begin(),payload->end());
      Check(record!=bytes.end(),"PCAP did not preserve the full UDP payload");
      Check(record-bytes.begin()>28 && record[-12]==127 && record[-9]==1,"PCAP lost the destination IPv4 address");
    }
    const auto frames=IndexLidarPcap(path);
    Check(frames.size()>2,"PCAP did not retain full sweeps");
    Check(frames[2].bag_timestamp_ns-frames[1].bag_timestamp_ns>=40000000,
      "Playback lost sensor timestamps after the back-to-back capture");
    std::size_t count=0;
    for(const auto & frame : frames) {
      const auto points=ReadLidarPcapFrame(frame);
      Check(!points.points.empty() && points.points.front().intensity==173,"PCAP scan did not decode raw intensity");
      count+=points.points.size();
    }
    Check(count==19*8*32,"Playback lost or duplicated returns at a cut inside a packet");
    live.StartRecording(temporary/"disconnect");
    send(35,41);
    WaitFor([&]{return live.Stats().packets>=41;},"Receiver stopped after finalizing");
    live.Disconnect();
    Check(!FindLidarPcap(temporary/"disconnect").empty(),"Disconnect did not finish the capture");
    live.ConnectDirect(options);
    Check(live.Active() && live.Stats().packets==0,"Reconnect did not release and rebind the port");
    live.Disconnect();
    fs::remove_all(temporary);
    std::cout<<"Native UDP, model signatures, frozen display, PCAP round-trip and reconnect passed\n";
    return 0;
  } catch(const std::exception & error) {
    if (!temporary.empty()) fs::remove_all(temporary);
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
