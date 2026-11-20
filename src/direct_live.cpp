#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mswsock.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "direct_live.hpp"
#include "lidar_packets.hpp"
#include "lidar_viewer/pcap_io.hpp"
#include "ouster_packets.hpp"
#include "udp_pcap.hpp"

#include <curl/curl.h>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace lidar_viewer
{
namespace
{
using Steady = std::chrono::steady_clock;
using Model = DirectLidarModel;
#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr auto kInvalidSocket = INVALID_SOCKET;
void CloseSocket(SocketHandle fd) {closesocket(fd);}
int SocketError() {return WSAGetLastError();}
bool Interrupted(int error) {return error == WSAEINTR || error == WSAEWOULDBLOCK;}
#else
using SocketHandle = int;
constexpr int kInvalidSocket = -1;
void CloseSocket(SocketHandle fd) {close(fd);}
int SocketError() {return errno;}
bool Interrupted(int error) {return error == EINTR || error == EAGAIN || error == EWOULDBLOCK;}
#endif

std::runtime_error NetworkError(const std::string & text)
{
  return std::runtime_error(text + " (socket error " + std::to_string(SocketError()) + ")");
}

struct NetworkContext
{
  NetworkContext()
  {
#ifdef _WIN32
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) throw std::runtime_error("Could not start Winsock");
#endif
  }
  ~NetworkContext()
  {
#ifdef _WIN32
    WSACleanup();
#endif
  }
};

struct UdpSocket
{
  SocketHandle handle{kInvalidSocket};
  std::uint16_t port;
  bool auxiliary;
#ifdef _WIN32
  LPFN_WSARECVMSG recv_message{nullptr};
#endif

  explicit UdpSocket(std::uint16_t listen_port, const std::optional<in_addr> & multicast, bool ancillary = false)
  : port(listen_port), auxiliary(ancillary)
  {
    const auto fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd == kInvalidSocket) throw NetworkError("Could not create a LiDAR UDP socket");
    try {
#ifdef _WIN32
      const int exclusive = 1;
      if (setsockopt(fd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char *>(&exclusive), sizeof(exclusive)) != 0)
        throw NetworkError("Could not reserve the LiDAR UDP port");
      GUID recv_message_id = WSAID_WSARECVMSG;
      DWORD returned = 0;
      if (setsockopt(fd, IPPROTO_IP, IP_PKTINFO, reinterpret_cast<const char *>(&exclusive), sizeof(exclusive)) == 0) {
        WSAIoctl(fd, SIO_GET_EXTENSION_FUNCTION_POINTER, &recv_message_id, sizeof(recv_message_id),
          &recv_message, sizeof(recv_message), &returned, nullptr, nullptr);
      }
#else
      const int enabled = 1;
#ifdef IP_RECVDSTADDR
      setsockopt(fd, IPPROTO_IP, IP_RECVDSTADDR, &enabled, sizeof(enabled));
#else
      setsockopt(fd, IPPROTO_IP, IP_PKTINFO, &enabled, sizeof(enabled));
#endif
#endif
      const int buffer = 4 * 1024 * 1024;
      setsockopt(fd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&buffer), sizeof(buffer));
      sockaddr_in bind_address{};
      bind_address.sin_family = AF_INET;
      bind_address.sin_port = htons(port);
      bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
      if (bind(fd, reinterpret_cast<const sockaddr *>(&bind_address), sizeof(bind_address)) != 0) {
        throw NetworkError("Could not listen on UDP port " + std::to_string(port) + ". Check whether another program is using it");
      }
      if (multicast) {
        const ip_mreq group{*multicast, {htonl(INADDR_ANY)}};
        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char *>(&group), sizeof(group)) != 0)
          throw NetworkError("Could not join the LiDAR multicast group");
      }
    } catch (...) {
      CloseSocket(fd);
      throw;
    }
    handle = fd;
  }
  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;
  UdpSocket(UdpSocket && other) noexcept
  : handle(std::exchange(other.handle, kInvalidSocket)), port(other.port), auxiliary(other.auxiliary)
#ifdef _WIN32
    , recv_message(other.recv_message)
#endif
  {}
  ~UdpSocket() {if (handle != kInvalidSocket) CloseSocket(handle);}

  int Receive(std::uint8_t * bytes, std::size_t size, sockaddr_in & sender, UdpRoute & route) const
  {
#ifdef _WIN32
    if (!recv_message) {
      int address_size = sizeof(sender);
      return recvfrom(handle, reinterpret_cast<char *>(bytes), static_cast<int>(size), 0,
        reinterpret_cast<sockaddr *>(&sender), &address_size);
    }
    WSABUF payload{static_cast<ULONG>(size), reinterpret_cast<char *>(bytes)};
    alignas(WSACMSGHDR) char control[WSA_CMSG_SPACE(sizeof(IN_PKTINFO))]{};
    WSAMSG message{};
    message.name = reinterpret_cast<sockaddr *>(&sender);
    message.namelen = sizeof(sender);
    message.lpBuffers = &payload;
    message.dwBufferCount = 1;
    message.Control.buf = control;
    message.Control.len = sizeof(control);
    DWORD count = 0;
    if (recv_message(handle, &message, &count, nullptr, nullptr) != 0) return -1;
    for (auto * header = WSA_CMSG_FIRSTHDR(&message); header; header = WSA_CMSG_NXTHDR(&message, header)) {
      if (header->cmsg_level != IPPROTO_IP || header->cmsg_type != IP_PKTINFO ||
          header->cmsg_len < WSA_CMSG_LEN(sizeof(IN_PKTINFO))) continue;
      IN_PKTINFO address{};
      std::memcpy(&address, WSA_CMSG_DATA(header), sizeof(address));
      std::memcpy(route.destination_ip.data(), &address.ipi_addr, route.destination_ip.size());
    }
    return static_cast<int>(count);
#else
    iovec payload{bytes, size};
#ifdef IP_RECVDSTADDR
    constexpr int option = IP_RECVDSTADDR;
    using Ipv4PacketInfo = in_addr;
#else
    constexpr int option = IP_PKTINFO;
    using Ipv4PacketInfo = in_pktinfo;
#endif
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(Ipv4PacketInfo))]{};
    msghdr message{};
    message.msg_name = &sender;
    message.msg_namelen = sizeof(sender);
    message.msg_iov = &payload;
    message.msg_iovlen = 1;
    message.msg_control = control;
    message.msg_controllen = sizeof(control);
    const auto count = recvmsg(handle, &message, 0);
    if (count < 0) return -1;
    for (auto * header = CMSG_FIRSTHDR(&message); header; header = CMSG_NXTHDR(&message, header)) {
      if (header->cmsg_level != IPPROTO_IP || header->cmsg_type != option ||
          header->cmsg_len < CMSG_LEN(sizeof(Ipv4PacketInfo))) continue;
      Ipv4PacketInfo address{};
      std::memcpy(&address, CMSG_DATA(header), sizeof(address));
#ifdef IP_RECVDSTADDR
      std::memcpy(route.destination_ip.data(), &address, route.destination_ip.size());
#else
      std::memcpy(route.destination_ip.data(), &address.ipi_addr, route.destination_ip.size());
#endif
    }
    return static_cast<int>(count);
#endif
  }
};

std::optional<in_addr> ParseIPv4(const std::string & text, const char * description)
{
  if (text.empty()) return {};
  in_addr address{};
  if (inet_pton(AF_INET, text.c_str(), &address) != 1)
    throw std::runtime_error(std::string("Enter a valid ") + description + " IPv4 address");
  return address;
}

const UdpSocket * WaitForUdp(const std::vector<UdpSocket> & sockets)
{
#ifdef _WIN32
  fd_set ready;
  FD_ZERO(&ready);
  for (const auto & socket : sockets) FD_SET(socket.handle, &ready);
  timeval timeout{0, 50000};
  const auto result = select(0, &ready, nullptr, nullptr, &timeout);
  if (result > 0) for (const auto & socket : sockets) if (FD_ISSET(socket.handle, &ready)) return &socket;
#else
  std::array<pollfd, 5> ready{};
  for (std::size_t i = 0; i < sockets.size(); ++i) ready[i] = {sockets[i].handle, POLLIN, 0};
  const auto result = poll(ready.data(), sockets.size(), 50);
  if (result > 0) {
    for (std::size_t i = 0; i < sockets.size(); ++i) {
      if (ready[i].revents & POLLIN) return &sockets[i];
      if (ready[i].revents & (POLLERR | POLLNVAL | POLLHUP)) throw std::runtime_error("LiDAR UDP input closed");
    }
  }
#endif
  if (result < 0 && !Interrupted(SocketError())) throw NetworkError("Could not wait for LiDAR UDP packets");
  return nullptr;
}

std::int64_t NowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string FetchOusterMetadata(const std::string & address)
{
  struct CurlRuntime {
    CurlRuntime() {
      if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK)
        throw std::runtime_error("Could not initialize Ouster metadata download");
    }
    ~CurlRuntime() {curl_global_cleanup();}
  };
  static const CurlRuntime runtime;
  std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> request(curl_easy_init(), curl_easy_cleanup);
  if (!request) throw std::runtime_error("Could not initialize Ouster HTTP request");
  const std::string url = "http://" + address + "/api/v1/sensor/metadata";
  std::string json;
  const auto receive = +[](char * data, std::size_t size, std::size_t count, void * context) -> std::size_t {
    auto & output = *static_cast<std::string *>(context);
    const auto length = size * count;
    if (length > 1024 * 1024 - output.size()) return 0;
    try {output.append(data,length);}
    catch (...) {return 0;}
    return length;
  };
  curl_easy_setopt(request.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(request.get(), CURLOPT_PROXY, "");
  curl_easy_setopt(request.get(), CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(request.get(), CURLOPT_CONNECTTIMEOUT_MS, 600L);
  curl_easy_setopt(request.get(), CURLOPT_TIMEOUT_MS, 1500L);
  curl_easy_setopt(request.get(), CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(request.get(), CURLOPT_WRITEFUNCTION, receive);
  curl_easy_setopt(request.get(), CURLOPT_WRITEDATA, &json);
  if (curl_easy_perform(request.get()) != CURLE_OK || !LooksLikeOusterMetadata(json)) {
    throw std::runtime_error("Could not read Ouster metadata. Check the sensor IPv4 address or choose its JSON file.");
  }
  return json;
}

void FinishLiveFrame(FrameData & frame)
{
  auto reflectivity = std::make_shared<std::vector<float>>();
  reflectivity->reserve(frame.points.size());
  Vec3 low{frame.points.front().x, frame.points.front().y, frame.points.front().z};
  Vec3 high = low;
  for (auto & point : frame.points) {
    reflectivity->push_back(point.intensity);
    point.intensity /= 255.f;
    low = {std::min(low.x, point.x), std::min(low.y, point.y), std::min(low.z, point.z)};
    high = {std::max(high.x, point.x), std::max(high.y, point.y), std::max(high.z, point.z)};
  }
  frame.source_intensities = std::move(reflectivity);
  frame.center = (low + high) * .5f;
  frame.radius = std::max(10.f, .7f * std::max({high.x - low.x, high.y - low.y, high.z - low.z}));
  frame.min_z = low.z;
  frame.max_z = high.z;
}
}  // namespace

struct DirectLiveSession::Impl
{
  NetworkContext network;
  DirectLiveOptions options;
  std::vector<UdpSocket> sockets;
  std::optional<in_addr> filtered_ip;
  std::string calibration;
  std::optional<OusterCalibration> ouster;
  std::optional<HesaiAtCalibration> hesai_at;
  std::atomic<bool> quit{false};
  std::thread receiver, recorder;
  mutable std::mutex mutex;
  std::condition_variable ready;
  LiveStats stats;
  std::unique_ptr<LiveFrame> latest;
  std::deque<Steady::time_point> cloud_times;
  std::deque<LidarDatagram> queue;
  std::size_t queued_bytes{0};
  bool stop_requested{false}, shutting_down{false};

  void Start(const DirectLiveOptions & selected)
  {
    options = selected;
    filtered_ip = ParseIPv4(options.sensor_ipv4, "LiDAR");
    const auto group = ParseIPv4(options.multicast_ipv4, "multicast");
    if (group && (ntohl(group->s_addr) >> 28) != 14)
      throw std::runtime_error("Multicast address must be in 224.0.0.0 to 239.255.255.255");
    if (!options.calibration_file.empty()) {
      calibration = ReadLidarCalibration(options.calibration_file);
      if (LooksLikeOusterMetadata(calibration)) {
        if (options.model != Model::Auto && options.model != Model::OusterOs)
          throw std::runtime_error("Select Ouster or Auto for an Ouster metadata JSON");
        options.model = Model::OusterOs;
        ouster.emplace(calibration);
      } else if (LooksLikeHesaiAtCalibration(calibration)) {
        if (options.model != Model::Auto && options.model != Model::HesaiAt128)
          throw std::runtime_error("Select AT128 / AT128P or Auto for a Hesai angle correction .dat file");
        options.model = Model::HesaiAt128;
        hesai_at.emplace(calibration);
      } else if (options.model != Model::Auto) {
        LidarPacketDecoder(options.model, calibration);
      }
    }
    if (options.model == Model::OusterOs && !ouster && !filtered_ip)
      throw std::runtime_error("Enter the Ouster sensor IPv4 or choose its metadata JSON");
    if (options.model == Model::HesaiAt128 && !hesai_at)
      throw std::runtime_error("Choose the AT128 sensor's angle correction .dat file");
    if (options.model != Model::OusterOs || ouster) OpenSockets(group);
    stats.connected = true;
    recorder = std::thread([this] {RecordLoop();});
    receiver = std::thread([this, group] {
      try {
        if (options.model == Model::OusterOs && !ouster) {
          calibration = FetchOusterMetadata(options.sensor_ipv4);
          ouster.emplace(calibration);
          OpenSockets(group);
        }
        ReceiveLoop(group);
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex);
        stats.error = error.what();
        stats.connected = false;
        if (stats.record_busy) {stop_requested = true; stats.record_stopping = true;}
        ready.notify_all();
      }
    });
  }

  void OpenSockets(const std::optional<in_addr> & group)
  {
    if (options.udp_port) {
      sockets.emplace_back(options.udp_port, group);
    } else if (options.model != Model::Auto) {
      sockets.emplace_back(ouster ? ouster->Port() : DirectLidarDefaultPort(options.model), group);
    } else {
      for (const auto port : {kDefaultLidarUdpPort, DirectLidarDefaultPort(Model::LivoxMid360),
          DirectLidarDefaultPort(Model::OusterOs), DirectLidarDefaultPort(Model::RobosenseM1)}) {
        try {
          sockets.emplace_back(port, group);
        } catch (const std::exception & error) {
          if (!stats.error.empty()) stats.error += "\n";
          stats.error += error.what();
        }
      }
      if (sockets.empty()) throw std::runtime_error(stats.error);
    }
  }

  void OpenAuxiliary(Model model, const std::optional<in_addr> & group)
  {
    std::uint16_t port = options.auxiliary_port;
    if (!port) {
      switch (model) {
        case Model::LivoxMid360: port = 56401; break;
        case Model::OusterOs: port = ouster->ImuPort(); break;
        case Model::HesaiXt32: case Model::HesaiXt16: case Model::HesaiXt32M2X: port = 10110; break;
        case Model::VelodyneVlp16: case Model::VelodynePuckHiRes:
        case Model::VelodyneHdl32e: case Model::VelodyneVlp32c: port = 8308; break;
        case Model::RobosenseM1: case Model::RobosenseM2: case Model::RobosenseM3: port = 7788; break;
        default: return;
      }
    }
    if (!port || std::any_of(sockets.begin(), sockets.end(), [port](const auto & socket) {return socket.port == port;})) return;
    try {
      sockets.emplace_back(port, group, true);
      std::lock_guard<std::mutex> lock(mutex);
      stats.auxiliary_port = port;
    } catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(mutex);
      stats.auxiliary_error = error.what();
    }
  }

  void QueueRecordingPacket(const std::uint8_t * data, std::size_t size, const UdpRoute & from, std::int64_t received_ns)
  {
    // The receiver calls this with the recording state locked.
    if (!stats.record_busy || stop_requested) return;
    constexpr std::size_t queue_limit = 8 * 1024 * 1024;
    if (queued_bytes + size + sizeof(LidarDatagram) > queue_limit) {
      ++stats.record_dropped;
      stats.record_error = "Recording storage is too slow. The write queue dropped packets.";
      return;
    }
    queue.push_back({{data, data + size}, from, received_ns});
    queued_bytes += size + sizeof(LidarDatagram);
    ready.notify_one();
  }

  void ReceiveLoop(const std::optional<in_addr> & group)
  {
    std::array<std::uint8_t, kMaxUdpPayload> data;
    std::optional<UdpRoute> route;
    std::optional<LidarPacketDecoder> decoder;
    std::optional<Model> failed_calibration;
    LidarSequence sequence;
    LidarSweep sweep;
    LidarClock clock;
    FrameData scan;
    bool synced = false;
    bool tried_ouster = false;
    std::string ouster_error;
    auto last_packet = Steady::now();
    while (!quit.load(std::memory_order_relaxed)) {
      const auto * socket = WaitForUdp(sockets);
      if (!socket) continue;
      sockaddr_in sender{};
      UdpRoute from;
      const auto bytes = socket->Receive(data.data(), data.size(), sender, from);
      if (bytes < 0) {
        if (Interrupted(SocketError())) continue;
        throw NetworkError("Could not receive a LiDAR packet");
      }
      if (filtered_ip && sender.sin_addr.s_addr != filtered_ip->s_addr) continue;
      from.source_port = ntohs(sender.sin_port);
      from.destination_port = socket->port;
      std::memcpy(from.source_ip.data(), &sender.sin_addr, 4);
      const auto size = static_cast<std::size_t>(bytes);
      const auto received_ns = NowNs();
      if (route) {
        if (route->source_ip != from.source_ip) continue;
        if (socket->auxiliary || route->destination_port == from.destination_port) {
          std::lock_guard<std::mutex> lock(mutex);
          QueueRecordingPacket(data.data(), size, from, received_ns);
        }
        if (!(*route == from)) continue;
      }
      if (socket->auxiliary) continue;
      auto packet = InspectLidarPacket(data.data(), size, ouster ? &*ouster : nullptr, hesai_at ? &*hesai_at : nullptr);
      if (!packet && !route && options.model == Model::Auto && !ouster && !tried_ouster &&
          (socket->port == DirectLidarDefaultPort(Model::OusterOs) || (filtered_ip && options.udp_port))) {
        tried_ouster = true;
        if (filtered_ip) {
          try {
            auto json = FetchOusterMetadata(options.sensor_ipv4);
            OusterCalibration metadata(json);
            packet = metadata.Inspect(data.data(),size);
            if (packet) {
              ouster.emplace(std::move(metadata));
              calibration = std::move(json);
            }
          } catch (const std::exception & error) {ouster_error = error.what();}
        } else {
          ouster_error = "UDP data on port 7502. For Ouster, enter the sensor IPv4 or choose its metadata JSON.";
        }
      }
      if (!packet || (options.model != Model::Auto && options.model != packet->model)) {
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.decode_errors;
        if (!stats.packets && stats.decode_errors == 1) {
          stats.error = packet
            ? std::string("Received ") + DirectLidarModelName(packet->model) + ". Match the selected model or use Auto."
            : (!ouster_error.empty() ? ouster_error : DescribeLidarPacket(data.data(), size));
        }
        continue;
      }
      if (!route) {
        if (failed_calibration == packet->model) continue;
        try {
          decoder.emplace(packet->model, calibration, ouster ? &*ouster : nullptr, hesai_at ? &*hesai_at : nullptr);
        } catch (const std::exception & error) {
          failed_calibration = packet->model;
          std::lock_guard<std::mutex> lock(mutex);
          ++stats.decode_errors;
          stats.error = error.what();
          continue;
        }
        route = from;
        OpenAuxiliary(packet->model, group);
        char address[INET_ADDRSTRLEN]{};
        inet_ntop(AF_INET, &sender.sin_addr, address, sizeof(address));
        std::lock_guard<std::mutex> lock(mutex);
        stats.sensor_model = packet->model;
        stats.sensor_address = address;
        stats.factory_calibration = !calibration.empty();
      }
      const auto now = Steady::now();
      if (now - last_packet > std::chrono::seconds(1)) {
        sequence.previous.reset();
        sweep = {};
        synced = false;
        scan.points.clear();
      }
      last_packet = now;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (packet->model != stats.sensor_model) {
          ++stats.decode_errors;
          continue;
        }
        ++stats.packets;
        stats.error.clear();
      }
      if (!sequence.Accept(packet->sequence)) continue;
      for (std::size_t block = 0; block < packet->blocks; ++block) {
        if (sweep.Crossed(*packet, block)) {
          if (synced && !scan.points.empty()) PublishScan(std::move(scan), sequence.missing, now);
          scan = {};
          scan.frame_id = LidarFrameId(packet->model);
          scan.timestamp_ns = clock.AtFrame(*packet, block, received_ns);
          scan.points.reserve(128 * 1024);
          synced = true;
        }
        if (!synced) continue;
        if (scan.points.size() > kMaxLidarScanPoints - 256) {
          scan.points.clear();
          synced = false;
          std::lock_guard<std::mutex> lock(mutex);
          ++stats.decode_errors;
          break;
        }
        decoder->AppendBlock(scan, data.data(), *packet, block);
      }
    }
  }

  void PublishScan(FrameData frame, std::uint64_t missing, Steady::time_point time)
  {
    FinishLiveFrame(frame);
    auto result = std::make_unique<LiveFrame>(LiveFrame{std::move(frame), 0});
    std::lock_guard<std::mutex> lock(mutex);
    result->sequence = ++stats.clouds;
    stats.sequence_gaps = missing;
    if (latest) ++stats.replaced_frames;
    latest = std::move(result);
    cloud_times.push_back(time);
    while (cloud_times.size() > 2 && time - cloud_times.front() > std::chrono::seconds(1))
      cloud_times.pop_front();
  }

  void RecordLoop()
  {
    std::unique_lock<std::mutex> lock(mutex);
    for (;;) {
      ready.wait(lock, [&] {return shutting_down || stats.record_busy;});
      if (!stats.record_busy) break;
      const auto path = stats.recording_path;
      lock.unlock();
      try {
        UdpPcapWriter writer(path, calibration);
        lock.lock();
        stats.recording = true;
        for (;;) {
          ready.wait(lock, [&] {return stop_requested || !queue.empty();});
          if (queue.empty()) break;
          auto batch = std::move(queue);
          queue.clear();
          queued_bytes = 0;
          lock.unlock();
          std::uint64_t bytes = 0;
          for (const auto & packet : batch) bytes += writer.Write(packet);
          lock.lock();
          stats.recorded_messages += batch.size();
          stats.recorded_bytes += bytes;
          // Deallocation of packet buffers must not hold up UDP reception.
          lock.unlock();
          batch.clear();
          lock.lock();
        }
        lock.unlock();
        writer.Finish();
        lock.lock();
      } catch (const std::exception & error) {
        if (!lock.owns_lock()) lock.lock();
        stats.record_error = error.what();
      }
      queue.clear();
      queued_bytes = 0;
      stats.record_busy = stats.recording = stats.record_stopping = false;
    }
  }

  void Stop()
  {
    quit.store(true, std::memory_order_relaxed);
    if (receiver.joinable()) receiver.join();
    sockets.clear();
    {
      std::lock_guard<std::mutex> lock(mutex);
      shutting_down = stop_requested = true;
      stats.connected = false;
    }
    ready.notify_all();
    if (recorder.joinable()) recorder.join();
  }

  ~Impl() {Stop();}
};

DirectLiveSession::DirectLiveSession(const DirectLiveOptions & options) : impl_(std::make_unique<Impl>())
{
  impl_->Start(options);
}

DirectLiveSession::~DirectLiveSession() = default;

void DirectLiveSession::Disconnect() {impl_->Stop();}

std::unique_ptr<LiveFrame> DirectLiveSession::TakeLatest()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return std::move(impl_->latest);
}

LiveStats DirectLiveSession::Stats() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto result = impl_->stats;
  if (impl_->cloud_times.empty()) return result;
  result.cloud_age_ms = std::chrono::duration<double, std::milli>(Steady::now() - impl_->cloud_times.back()).count();
  if (impl_->cloud_times.size() > 1 && result.cloud_age_ms < 1000) {
    const auto span = std::chrono::duration<double>(impl_->cloud_times.back() - impl_->cloud_times.front()).count();
    if (span > 0) result.cloud_hz = (impl_->cloud_times.size() - 1) / span;
  }
  return result;
}

void DirectLiveSession::StartRecording(const fs::path & destination)
{
  if (destination.empty()) throw std::runtime_error("Choose a recording folder");
  const auto path = fs::absolute(destination) / kLivePcapFilename;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto & status = impl_->stats;
  if (!status.connected || !status.packets) throw std::runtime_error("Wait for LiDAR packets before recording");
  if (status.record_busy) throw std::runtime_error("A recording is already active or being saved");
  status.recording_path = path;
  status.record_error.clear();
  status.recorded_messages = status.recorded_bytes = status.record_dropped = 0;
  status.record_busy = true;
  impl_->stop_requested = false;
  impl_->ready.notify_one();
}

void DirectLiveSession::StopRecording()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->stats.record_busy) return;
  impl_->stop_requested = true;
  impl_->stats.record_stopping = true;
  impl_->ready.notify_one();
}
}  // namespace lidar_viewer
