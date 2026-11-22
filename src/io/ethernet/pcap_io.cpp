#include "lidar_viewer/pcap_io.hpp"
#include "ouster_packets.hpp"
#include "udp_pcap.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <locale>
#include <utility>

namespace lidar_viewer
{
struct PcapSource
{
  UdpRoute route;
  DirectLidarModel model;
  std::string calibration;
  std::shared_ptr<const OusterCalibration> ouster;
  std::shared_ptr<const HesaiAtCalibration> hesai_at;
};

namespace
{
constexpr std::uint32_t kRawIpv4 = 101;
constexpr std::size_t kPcapHeader = 24, kPcapRecord = 16, kIpHeader = 20, kUdpHeader = 8;
constexpr std::size_t kMaxPcapPayload = 65536 + 32;
constexpr std::size_t kPcapWindow = 256 * 1024;
constexpr const char * kPcapSourceVersion = "lidar-studio-pcap-source-v1";
constexpr std::uint64_t kFingerprintSeed = 14695981039346656037ull;

std::uint64_t FingerprintBytes(std::uint64_t hash, const std::uint8_t * bytes, std::size_t size)
{
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ull;
  }
  return hash;
}

std::runtime_error InvalidPcapSource()
{
  return std::runtime_error("PCAP source information is invalid or belongs to a different capture");
}

std::uint16_t Be16(const std::uint8_t * bytes)
{
  return static_cast<std::uint16_t>((bytes[0] << 8) | bytes[1]);
}

void PutBe16(std::uint8_t * bytes, std::uint16_t value)
{
  bytes[0] = static_cast<std::uint8_t>(value >> 8);
  bytes[1] = static_cast<std::uint8_t>(value);
}

void PutLe32(std::uint8_t * bytes, std::uint32_t value)
{
  for (unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

std::uint16_t IpChecksum(const std::uint8_t * header)
{
  unsigned sum = 0;
  for (std::size_t i = 0; i < kIpHeader; i += 2) sum += Be16(header + i);
  sum = (sum & 0xffffu) + (sum >> 16);
  return static_cast<std::uint16_t>(~((sum & 0xffffu) + (sum >> 16)));
}

struct CapturedUdp
{
  const std::uint8_t * data;
  std::size_t size;
  UdpRoute route;
  std::int64_t received_ns;
};

std::optional<CapturedUdp> ExtractUdp(const std::uint8_t * data, std::size_t size,
  std::uint32_t link, std::int64_t time)
{
  std::size_t offset = 0;
  std::uint16_t protocol = 0x0800;
  switch (link) {
    case 1: // Ethernet
      if (size < 14) return {};
      offset = 14; protocol = Be16(data + 12);
      break;
    case 113: // Linux cooked
      if (size < 16) return {};
      offset = 16; protocol = Be16(data + 14);
      break;
    case 276: // Linux cooked v2
      if (size < 20) return {};
      offset = 20; protocol = Be16(data);
      break;
    default: break; // raw IPv4
  }
  while ((protocol == 0x8100 || protocol == 0x88a8) && size >= offset + 4) {
    protocol = Be16(data + offset + 2);
    offset += 4;
  }
  if (protocol != 0x0800 || size - offset < kIpHeader) return {};
  const auto * ip = data + offset;
  if ((ip[0] >> 4) != 4 || ip[9] != 17 || (Be16(ip + 6) & 0x3fffu) != 0) return {};
  const std::size_t ip_header = (ip[0] & 0xf) * 4u;
  const std::size_t total = Be16(ip + 2);
  if (ip_header < kIpHeader || total < ip_header + kUdpHeader || total > size - offset) return {};
  const auto * udp = ip + ip_header;
  const std::size_t length = Be16(udp + 4);
  if (length < kUdpHeader || length > total - ip_header) return {};
  UdpRoute route;
  std::copy_n(ip + 12, 4, route.source_ip.begin());
  std::copy_n(ip + 16, 4, route.destination_ip.begin());
  route.source_port = Be16(udp);
  route.destination_port = Be16(udp + 2);
  return CapturedUdp{udp + kUdpHeader, length - kUdpHeader, route, time};
}

class PcapReader
{
public:
  explicit PcapReader(const fs::path & path,
    std::uint64_t read_ahead_end = std::numeric_limits<std::uint64_t>::max())
  : file_(path, std::ios::binary), size_(fs::file_size(path)), read_ahead_end_(std::min(size_, read_ahead_end))
  {
    std::array<std::uint8_t, kPcapHeader> header{};
    if (!file_.read(reinterpret_cast<char *>(header.data()), header.size())) {
      throw std::runtime_error("Could not read the PCAP header");
    }
    const auto magic = Be16(header.data());
    little_ = magic == 0xd4c3 || magic == 0x4d3c;
    const auto format = Read32(header.data());
    nanos_ = format == 0xa1b23c4du;
    if ((!nanos_ && format != 0xa1b2c3d4u) || Read16(header.data() + 4) != 2 ||
        Read16(header.data() + 6) != 4) {
      throw std::runtime_error("Expected a classic PCAP 2.4 file");
    }
    link_ = Read32(header.data() + 20) & 0xffffu;
    if (link_ != kRawIpv4 && link_ != 1 && link_ != 113 && link_ != 276 && link_ != 228) {
      throw std::runtime_error("PCAP requires Ethernet, IPv4 or Linux cooked packets");
    }
  }

  std::uint64_t Position() const {return window_offset_ + next_;}
  std::uint64_t Size() const {return size_;}

  void Seek(std::uint64_t offset)
  {
    if (offset < kPcapHeader || offset > size_ ||
        offset > static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max())) {
      throw std::runtime_error("Invalid PCAP frame offset");
    }
    if (offset >= window_offset_ && offset - window_offset_ <= end_) {
      next_ = static_cast<std::size_t>(offset - window_offset_);
      return;
    }
    file_.seekg(static_cast<std::streamoff>(offset));
    if (!file_) throw std::runtime_error("Could not seek to the PCAP frame");
    window_offset_ = offset;
    next_ = end_ = 0;
  }

  std::optional<CapturedUdp> Read()
  {
    if (!Ensure(kPcapRecord)) {
      throw std::runtime_error("Truncated PCAP record header");
    }
    const auto * header = window_->data() + next_;
    next_ += kPcapRecord;
    const auto captured = Read32(header + 8);
    if (captured > size_ - Position() || captured > Read32(header + 12)) {
      throw std::runtime_error("Truncated or invalid PCAP record");
    }
    if (captured > kMaxPcapPayload) {
      Seek(Position() + captured);
      return {};
    }
    const auto seconds = Read32(header);
    const auto fraction = Read32(header + 4);
    if (!Ensure(captured)) {
      throw std::runtime_error("Could not read the PCAP payload");
    }
    const auto * payload = window_->data() + next_;
    next_ += captured;
    if (fraction >= (nanos_ ? 1000000000u : 1000000u)) return {};
    const auto stamp = seconds * 1000000000ll + fraction * (nanos_ ? 1ll : 1000ll);
    // The UDP view is valid until the next read or seek.
    return ExtractUdp(payload, captured, link_, stamp);
  }

  // This is used once per index build. Frame reads only parse UDP as usual.
  std::uint64_t RecordFingerprint()
  {
    if (!Ensure(kPcapRecord)) throw InvalidPcapSource();
    const auto captured = Read32(window_->data() + next_ + 8);
    if (captured > kMaxPcapPayload || !Ensure(kPcapRecord + captured)) throw InvalidPcapSource();
    return FingerprintBytes(kFingerprintSeed, window_->data() + next_, kPcapRecord + captured);
  }

private:
  bool Ensure(std::size_t bytes)
  {
    const auto available = end_ - next_;
    if (available >= bytes) return true;
    const auto position = Position();
    const auto remaining = size_ - position;
    if (remaining < bytes) return false;
    // The stream is at the end of the window. Keep only the unread tail.
    auto * data = window_->data();
    if (available) std::memmove(data, data + next_, available);
    window_offset_ = position;
    next_ = 0;
    const auto ahead = position < read_ahead_end_ ? read_ahead_end_ - position : 0;
    // A frame may also need the packet that starts at its indexed end.
    const auto fill = std::max<std::uint64_t>(bytes, std::min<std::uint64_t>(ahead, window_->size()));
    const auto count = fill - available;
    file_.read(reinterpret_cast<char *>(data + available), static_cast<std::streamsize>(count));
    end_ = available + static_cast<std::size_t>(file_.gcount());
    // A concurrent truncation can cause a short read. Complete records already
    // in the window can still be used, and subsequent seeks can still succeed.
    if (file_.eof()) file_.clear(file_.rdstate() & std::ios::badbit);
    return end_ >= bytes;
  }

  std::uint16_t Read16(const std::uint8_t * bytes) const
  {
    return little_ ? static_cast<std::uint16_t>(bytes[0] | bytes[1] << 8) : Be16(bytes);
  }

  std::uint32_t Read32(const std::uint8_t * bytes) const
  {
    const std::uint32_t first = Read16(bytes), second = Read16(bytes + 2);
    return little_ ? first | (second << 16) : (first << 16) | second;
  }

  std::ifstream file_;
  using Window = std::array<std::uint8_t, kPcapWindow>;
  std::unique_ptr<Window> window_{new Window};
  std::uint64_t size_, read_ahead_end_, window_offset_{kPcapHeader};
  std::size_t next_{0}, end_{0};
  std::uint32_t link_{0};
  bool little_{false}, nanos_{false};
};

struct RecordedSource
{
  std::uint64_t point_offset{0}, fingerprint{0};
};

std::optional<RecordedSource> ReadRecordedSource(const fs::path & path, std::uint64_t capture_size)
{
  auto sidecar = path;
  sidecar += kPcapSourceSuffix;
  if (!fs::exists(sidecar)) return std::nullopt;
  if (!fs::is_regular_file(sidecar) || fs::file_size(sidecar) > 96) throw InvalidPcapSource();
  std::ifstream input(sidecar, std::ios::binary);
  input.imbue(std::locale::classic());
  std::string version;
  std::uint64_t size = 0;
  RecordedSource source;
  if (!(input >> version >> size >> source.point_offset >> source.fingerprint) ||
      version != kPcapSourceVersion || size != capture_size ||
      (source.point_offset && (source.point_offset < kPcapHeader || source.point_offset >= capture_size))) {
    throw InvalidPcapSource();
  }
  input >> std::ws;
  if (!input.eof()) throw InvalidPcapSource();
  return source;
}

FrameHandle FrameStart(const fs::path & path, const std::shared_ptr<const PcapSource> & source,
  std::uint64_t offset, std::size_t block, std::int64_t timestamp, std::size_t number)
{
  FrameHandle frame{path, InputFormat::LidarPcap, offset, 0,
    std::string(DirectLidarModelName(source->model)) + " #" + std::to_string(number + 1)};
  frame.bag_timestamp_ns = timestamp;
  frame.pcap_source = source;
  frame.pcap_start_block = static_cast<std::uint8_t>(block);
  return frame;
}
}  // namespace

fs::path FindLidarPcap(const fs::path & path)
{
  std::error_code error;
  if (path.extension() == ".pcap" && fs::is_regular_file(path, error)) return path;
  if (fs::is_directory(path, error) && fs::is_regular_file(path / kLivePcapFilename, error)) {
    return path / kLivePcapFilename;
  }
  return {};
}

std::vector<FrameHandle> IndexLidarPcap(const fs::path & path, InputLoadState * load)
{
  PcapReader reader(path);
  // A native capture keeps all raw datagrams in receive order. The source
  // offset selects the same point route and model that the live viewer used.
  const auto recorded_source = ReadRecordedSource(path, reader.Size());
  if (recorded_source) {
    const auto offset = recorded_source->point_offset;
    reader.Seek(offset ? offset : kPcapHeader);
    if (reader.RecordFingerprint() != recorded_source->fingerprint) throw InvalidPcapSource();
    if (!offset) throw std::runtime_error("The capture contains no packets from the selected LiDAR point stream");
  }
  auto calibration_path = path;
  calibration_path += kPcapCorrectionSuffix;
  std::string calibration;
  if (fs::is_regular_file(calibration_path)) calibration = ReadLidarCalibration(calibration_path);
  else {
    for (const auto * extension : {".json", ".dat", ".csv"}) {
      const auto candidate = fs::path(path).replace_extension(extension);
      if (!fs::is_regular_file(candidate)) continue;
      calibration = ReadLidarCalibration(candidate);
      break;
    }
  }
  std::shared_ptr<const OusterCalibration> ouster;
  std::shared_ptr<const HesaiAtCalibration> hesai_at;
  if (LooksLikeOusterMetadata(calibration)) ouster = std::make_shared<OusterCalibration>(calibration);
  else if (LooksLikeHesaiAtCalibration(calibration)) hesai_at = std::make_shared<HesaiAtCalibration>(calibration);

  std::shared_ptr<PcapSource> source;
  std::vector<FrameHandle> frames;
  LidarSequence sequence;
  LidarSweep sweep;
  LidarClock clock;
  std::uint64_t progress_at = reader.Position();
  auto publish = [&] {
    if (load) load->Publish({InputLoadStage::IndexingPcap, reader.Position(), reader.Size(), frames.size(), 0, 0, {}});
  };
  publish();
  std::string diagnostic;
  while (reader.Position() < reader.Size()) {
    const auto position = reader.Position();
    if (position >= progress_at) {
      publish();
      progress_at = position + 1024u * 1024u;
    }
    const auto udp = reader.Read();
    if (udp && source && !(source->route == udp->route)) continue;
    const auto packet = udp ? InspectLidarPacket(udp->data, udp->size, ouster.get(), hesai_at.get()) : std::nullopt;
    if (!packet) {
      if (recorded_source && !source) throw std::runtime_error("Could not decode the selected LiDAR packet in this capture");
      if (!source && udp && diagnostic.empty() && udp->route.destination_port == kDefaultLidarUdpPort)
        diagnostic = DescribeLidarPacket(udp->data, udp->size);
      continue;
    }
    if (!source) {
      LidarPacketDecoder decoder(packet->model, calibration, ouster.get(), hesai_at.get());
      source = std::make_shared<PcapSource>(PcapSource{udp->route, packet->model, std::move(calibration), ouster, hesai_at});
      frames.push_back(FrameStart(path, source, position, 0, clock.AtFrame(*packet, 0, udp->received_ns), 0));
    }
    if (source->model != packet->model || !sequence.Accept(packet->sequence)) continue;
    for (std::size_t block = 0; block < packet->blocks; ++block) {
      if (!sweep.Crossed(*packet, block)) continue;
      frames.back().pcap_end_offset = position;
      frames.back().pcap_end_block = static_cast<std::uint8_t>(block);
      frames.push_back(FrameStart(path, source, position, block, clock.AtFrame(*packet, block, udp->received_ns), frames.size()));
    }
  }
  if (frames.empty()) {
    throw std::runtime_error(diagnostic.empty() ?
      "No supported LiDAR UDP packets found. Ouster PCAP files require their matching metadata JSON." : diagnostic);
  }
  frames.back().pcap_end_offset = reader.Size();
  publish();
  return frames;
}

FrameData ReadLidarPcapFrame(const FrameHandle & handle)
{
  if (!handle.pcap_source) throw std::runtime_error("Missing PCAP frame source");
  PcapReader reader(handle.path, handle.pcap_end_offset);
  reader.Seek(handle.payload_offset);
  const auto & source = *handle.pcap_source;
  LidarPacketDecoder decoder(source.model, source.calibration, source.ouster.get(), source.hesai_at.get());
  LidarSequence sequence;
  FrameData frame;
  frame.points.reserve(128 * 1024);
  frame.frame_id = LidarFrameId(source.model);
  frame.timestamp_ns = handle.bag_timestamp_ns;
  while (reader.Position() < handle.pcap_end_offset ||
         (reader.Position() == handle.pcap_end_offset && handle.pcap_end_block > 0)) {
    const auto position = reader.Position();
    const auto udp = reader.Read();
    if (!udp || !(source.route == udp->route)) continue;
    const auto packet = InspectLidarPacket(udp->data, udp->size, source.ouster.get(), source.hesai_at.get());
    if (!packet || packet->model != source.model || !sequence.Accept(packet->sequence)) continue;
    const auto first = position == handle.payload_offset ? handle.pcap_start_block : 0u;
    const auto last = position == handle.pcap_end_offset ? handle.pcap_end_block : packet->blocks;
    if (first > last || last > packet->blocks) throw std::runtime_error("PCAP changed after indexing");
    for (auto block = first; block < last; ++block) decoder.AppendBlock(frame, udp->data, *packet, block);
  }
  return frame;
}

UdpPcapWriter::UdpPcapWriter(const fs::path & path, std::string_view calibration)
: path_(path), temporary_(path)
{
  fs::create_directories(path_.parent_path().parent_path());
  if (!fs::create_directory(path_.parent_path())) {
    throw std::runtime_error("Recording folder already exists");
  }
  temporary_ += ".part";
  stream_.exceptions(std::ios::failbit | std::ios::badbit);
  stream_.open(temporary_, std::ios::binary);
  std::array<std::uint8_t, kPcapHeader> header{};
  PutLe32(header.data(), 0xa1b23c4d);
  header[4] = 2; header[6] = 4;
  PutLe32(header.data() + 16, 65535);
  PutLe32(header.data() + 20, kRawIpv4);
  stream_.write(reinterpret_cast<const char *>(header.data()), header.size());
  next_offset_ = header.size();
  if (!calibration.empty()) {
    auto sidecar = path_;
    sidecar += kPcapCorrectionSuffix;
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(sidecar, std::ios::binary);
    output.write(calibration.data(), static_cast<std::streamsize>(calibration.size()));
    output.close();
  }
}

std::size_t UdpPcapWriter::Write(const LidarDatagram & packet)
{
  const auto ip_size = kIpHeader + kUdpHeader + packet.payload.size();
  std::array<std::uint8_t, kPcapRecord + kIpHeader + kUdpHeader> header{};
  PutLe32(header.data(), static_cast<std::uint32_t>(packet.received_ns / 1000000000ll));
  PutLe32(header.data() + 4, static_cast<std::uint32_t>(packet.received_ns % 1000000000ll));
  PutLe32(header.data() + 8, static_cast<std::uint32_t>(ip_size));
  PutLe32(header.data() + 12, static_cast<std::uint32_t>(ip_size));
  auto * ip = header.data() + kPcapRecord;
  ip[0] = 0x45; ip[8] = 64; ip[9] = 17;
  PutBe16(ip + 2, static_cast<std::uint16_t>(ip_size));
  std::copy(packet.route.source_ip.begin(), packet.route.source_ip.end(), ip + 12);
  // Socket capture preserves UDP payloads and endpoints. Other header fields
  // are reconstructed so the recording can be read by standard PCAP tools.
  std::copy(packet.route.destination_ip.begin(), packet.route.destination_ip.end(), ip + 16);
  PutBe16(ip + 10, IpChecksum(ip));
  auto * udp = ip + kIpHeader;
  PutBe16(udp, packet.route.source_port);
  PutBe16(udp + 2, packet.route.destination_port);
  PutBe16(udp + 4, static_cast<std::uint16_t>(kUdpHeader + packet.payload.size()));
  stream_.write(reinterpret_cast<const char *>(header.data()), header.size());
  stream_.write(reinterpret_cast<const char *>(packet.payload.data()), static_cast<std::streamsize>(packet.payload.size()));
  const bool first_point = packet.selected_point && !first_point_offset_;
  if (!packets_ || first_point) {
    source_fingerprint_ = FingerprintBytes(FingerprintBytes(kFingerprintSeed, header.data(), header.size()),
      packet.payload.data(), packet.payload.size());
  }
  if (first_point) first_point_offset_ = next_offset_;
  const auto bytes = header.size() + packet.payload.size();
  next_offset_ += bytes;
  ++packets_;
  return bytes;
}

void UdpPcapWriter::Finish()
{
  stream_.close();
  if (!packets_) {
    fs::remove(temporary_);
    throw std::runtime_error("Recording ended before any LiDAR packets were saved");
  }
  auto sidecar = path_;
  sidecar += kPcapSourceSuffix;
  std::ofstream source;
  source.exceptions(std::ios::failbit | std::ios::badbit);
  source.imbue(std::locale::classic());
  source.open(sidecar, std::ios::binary);
  source << kPcapSourceVersion << ' ' << next_offset_ << ' ' << first_point_offset_ << ' ' << source_fingerprint_ << '\n';
  source.close();
  fs::rename(temporary_, path_);
}
}  // namespace lidar_viewer
