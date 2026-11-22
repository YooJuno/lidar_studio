#include "lidar_packets.hpp"
#include "ouster_packets.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace lidar_viewer
{
namespace
{
using Model = DirectLidarModel;
constexpr float kRadians = kPi / 180.f;
constexpr std::size_t kVelodyneBlockBytes = 100;
constexpr std::size_t kLivoxHeaderBytes = 36;
constexpr std::size_t kAtBlockBytes = 3 + 128 * 4;
constexpr std::uint32_t kAtCircle = 360 * 25600;
constexpr std::uint64_t kNanosecondsPerHour = 3600000000000ull;

std::uint16_t Le16(const std::uint8_t * bytes)
{
  return static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8));
}

std::uint32_t Le32(const std::uint8_t * bytes)
{
  return Le16(bytes) | (static_cast<std::uint32_t>(Le16(bytes + 2)) << 16);
}

std::uint16_t Be16(const std::uint8_t * bytes)
{
  return static_cast<std::uint16_t>((bytes[0] << 8) | bytes[1]);
}

std::uint64_t HesaiTime(const std::uint8_t * utc, std::uint32_t microseconds)
{
  if (microseconds >= 1000000u) return 0;
  std::int64_t seconds = 0;
  if (utc[0] == 0) {
    for (std::size_t i = 2; i < 6; ++i) seconds = (seconds << 8) | utc[i];
  } else if (utc[0] >= 70 && utc[1] >= 1 && utc[1] <= 12 && utc[2] >= 1 && utc[2] <= 31 &&
      utc[3] < 24 && utc[4] < 60 && utc[5] <= 60) {
    std::tm time{};
    time.tm_year = utc[0]; time.tm_mon = utc[1] - 1; time.tm_mday = utc[2];
    time.tm_hour = utc[3]; time.tm_min = utc[4]; time.tm_sec = utc[5];
#ifdef _WIN32
    seconds = _mkgmtime64(&time);
#else
    seconds = timegm(&time);
#endif
  }
  return seconds > 0 ? static_cast<std::uint64_t>(seconds) * 1000000000ull + microseconds * 1000ull : 0;
}

std::int32_t Signed(std::uint16_t value)
{
  return value < 0x8000u ? value : static_cast<int>(value) - 0x10000;
}

std::int32_t Signed16(const std::uint8_t * bytes)
{
  return Signed(Le16(bytes));
}

std::int64_t Signed32(const std::uint8_t * bytes)
{
  const auto value = Le32(bytes);
  return value < 0x80000000u ? value : static_cast<std::int64_t>(value) - 0x100000000ll;
}

std::uint32_t AtEncoder(const std::uint8_t * block)
{
  return Le16(block) * 256u + block[2];
}

std::optional<LidarPacket> InspectAt128(const std::uint8_t * data, std::size_t size, const HesaiAtCalibration * calibration)
{
  if (size < 12 || data[0] != 0xee || data[1] != 0xff || data[2] != 4 || data[3] != 3 ||
      data[6] != 128 || data[7] != 2 || data[9] != 4) return {};
  constexpr std::size_t tail = 12 + 2 * kAtBlockBytes + 4;
  const bool sequence = (data[11] & 1) != 0;
  const std::size_t minimum = tail + 32 + (sequence ? 4 : 0) + 4;
  if (size != minimum && size != minimum + 32) return {};
  LidarPacket packet{Model::HesaiAt128, 2};
  if (sequence) packet.sequence = Le32(data + tail + 32);
  packet.sensor_timestamp_ns = HesaiTime(data + tail + 26, Le32(data + tail + 20));
  for (std::size_t block = 0; block < packet.blocks; ++block) {
    const auto encoder = AtEncoder(data + 12 + block * kAtBlockBytes);
    if (encoder >= kAtCircle) return {};
    packet.scan_ids[block] = calibration ? calibration->Sector(encoder) : HesaiAtCalibration::kOutsideSector;
  }
  return packet;
}

std::optional<LidarPacket> InspectXt(const std::uint8_t * data, std::size_t size)
{
  // The XT32M2X uses six blocks in the common XT 6.1 protocol.
  if (size < 12 || data[0] != 0xee || data[1] != 0xff || data[2] != 6 || data[3] != 1 ||
      (data[6] != 32 && data[6] != 16) || (data[7] != 8 && !(data[7] == 6 && data[6] == 32)) || data[9] != 4) return {};
  const std::size_t stride = 2 + 4 * data[6];
  const std::size_t tail = 12 + data[7] * stride;
  const bool has_sequence = (data[11] & 1) != 0;
  if (size != tail + 28 && (has_sequence || size != tail + 24)) return {};

  const auto model = data[7] == 6 ? Model::HesaiXt32M2X : data[6] == 32 ? Model::HesaiXt32 : Model::HesaiXt16;
  LidarPacket packet{model, data[7]};
  packet.motor_rpm = Le16(data + tail + 11);
  packet.sensor_timestamp_ns = HesaiTime(data + tail + 13, Le32(data + tail + 19));
  for (std::size_t i = 0; i < packet.blocks; ++i) {
    packet.azimuth[i] = Le16(data + 12 + i * stride);
    if (packet.azimuth[i] >= 36000) return {};
  }
  if (has_sequence) packet.sequence = Le32(data + tail + 24);
  return packet;
}

std::optional<LidarPacket> InspectVelodyne(const std::uint8_t * data, std::size_t size)
{
  if (size != 1206 || data[1204] < 0x37 || data[1204] > 0x39) return {};
  Model model;
  switch (data[1205]) {
    case 0x21: model = Model::VelodyneHdl32e; break;
    case 0x22: model = Model::VelodyneVlp16; break;
    case 0x24: model = Model::VelodynePuckHiRes; break;
    case 0x28: model = Model::VelodyneVlp32c; break;
    default: return {};
  }
  LidarPacket packet{model, 12};
  packet.dual_return = data[1204] == 0x39;
  const auto usec = Le32(data + 1200);
  // An hour-relative clock, with zero reserved here for unavailable timestamps.
  if (usec < 3600000000u) packet.sensor_timestamp_ns = usec * 1000ull + 1;
  for (std::size_t i = 0; i < packet.blocks; ++i) {
    const auto * block = data + i * kVelodyneBlockBytes;
    if (Le16(block) != 0xeeff) return {};
    packet.azimuth[i] = Le16(block + 2);
    if (packet.azimuth[i] >= 36000) return {};
  }
  return packet;
}

std::optional<LidarPacket> InspectMid360(const std::uint8_t * data, std::size_t size)
{
  if (size < kLivoxHeaderBytes || data[0] != 0 || Le16(data + 1) != size || data[11] > 2) return {};
  constexpr std::array<std::size_t, 4> point_bytes{0, 14, 8, 10};
  const auto type = data[10];
  const auto points = Le16(data + 5);
  if (type < 1 || type > 3 || points == 0 || points > 96 ||
      size != kLivoxHeaderBytes + points * point_bytes[type]) return {};
  LidarPacket packet{Model::LivoxMid360};
  packet.point_count = points;
  packet.data_type = type;
  packet.sensor_timestamp_ns = Le32(data + 28) | (static_cast<std::uint64_t>(Le32(data + 32)) << 32);
  return packet;
}

bool IsRobosenseM(Model model)
{
  return model == Model::RobosenseM1 || model == Model::RobosenseM2 || model == Model::RobosenseM3;
}

std::optional<LidarPacket> InspectRobosense(const std::uint8_t * data, std::size_t size)
{
  // MSOP signature and payload length for the ordinary M-series streams.
  if (size != 1210 && size != 1342 && size != 1448) return {};
  if (data[0] != 0x55 || data[1] != 0xaa || data[2] != 0x5a || data[3] != 0xa5) return {};
  LidarPacket packet(size == 1210 ? Model::RobosenseM1 : size == 1342 ? Model::RobosenseM2 : Model::RobosenseM3);
  packet.scan_ids[0] = Be16(data + 4);
  std::uint64_t seconds = 0;
  std::uint32_t usec = 0;
  for (std::size_t i = 10; i < 16; ++i) seconds = (seconds << 8) | data[i];
  for (std::size_t i = 16; i < 20; ++i) usec = (usec << 8) | data[i];
  if (seconds && seconds < 10000000000ull && usec < 1000000u)
    packet.sensor_timestamp_ns = seconds * 1000000000ull + usec * 1000ull;
  return packet;
}

std::size_t BeamCount(Model model)
{
  return model == Model::HesaiXt16 || model == Model::VelodyneVlp16 || model == Model::VelodynePuckHiRes ? 16 : 32;
}

BeamAngles DefaultAngles(Model model)
{
  BeamAngles angles;
  if (model == Model::HesaiXt32M2X) {
    for (std::size_t i = 0; i < BeamCount(model); ++i) angles.elevation[i] = 19.5f - 1.3f * static_cast<float>(i);
  } else if (IsHesaiXt(model)) {
    const float step = model == Model::HesaiXt16 ? 2.f : 1.f;
    for (std::size_t i = 0; i < BeamCount(model); ++i) angles.elevation[i] = 15.f - step * static_cast<float>(i);
  } else if (model == Model::VelodyneHdl32e) {
    for (std::size_t i = 0; i < 16; ++i) {
      angles.elevation[i * 2] = (static_cast<float>(i) * 4 - 92) / 3;
      angles.elevation[i * 2 + 1] = (static_cast<float>(i) * 4 - 28) / 3;
    }
  } else if (model == Model::VelodyneVlp32c) {
    // Factory VLP-32C firing order. Corrections are in degrees.
    angles.elevation = {-25,-1,-1.667f,-15.639f,-11.31f,0,-.667f,-8.843f,-7.254f,.333f,-.333f,-6.148f,
      -5.333f,1.333f,.667f,-4,-4.667f,1.667f,1,-3.667f,-3.333f,3.333f,2.333f,-2.667f,-3,7,4.667f,-2.333f,
      -2,15,10.333f,-1.333f};
    angles.azimuth = {-1.4f,4.2f,-1.4f,1.4f,-1.4f,1.4f,-4.2f,1.4f,-1.4f,4.2f,-1.4f,1.4f,-4.2f,1.4f,-4.2f,1.4f,
      -1.4f,4.2f,-1.4f,4.2f,-4.2f,1.4f,-1.4f,1.4f,-1.4f,1.4f,-1.4f,4.2f,-4.2f,1.4f,-1.4f,1.4f};
  } else if (model == Model::VelodyneVlp16 || model == Model::VelodynePuckHiRes) {
    const float spacing = model == Model::VelodynePuckHiRes ? 4.f / 3.f : 2.f;
    for (std::size_t i = 0; i < 8; ++i) {
      angles.elevation[i * 2] = (-7.5f + static_cast<float>(i)) * spacing;
      angles.elevation[i * 2 + 1] = (.5f + static_cast<float>(i)) * spacing;
    }
  }
  return angles;
}

BeamAngles ParseAngles(std::string_view csv, std::size_t channels)
{
  std::istringstream input{std::string(csv)};
  std::string line;
  BeamAngles angles;
  std::array<bool, 32> seen{};
  std::size_t rows = 0;
  while (std::getline(input, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    if (rows == 0 && line.find("Elevation") != std::string::npos && line.find("Azimuth") != std::string::npos) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    row.imbue(std::locale::classic());
    int id = 0;
    float elevation = 0, azimuth = 0;
    std::string extra;
    if (!(row >> id >> elevation >> azimuth) || row >> extra ||
        id < 1 || id > static_cast<int>(channels) || seen[id - 1] ||
        !std::isfinite(elevation) || std::abs(elevation) > 90.f ||
        !std::isfinite(azimuth) || std::abs(azimuth) > 180.f) {
      throw std::runtime_error("Invalid beam angle CSV row");
    }
    seen[id - 1] = true;
    angles.elevation[id - 1] = elevation;
    angles.azimuth[id - 1] = azimuth;
    ++rows;
  }
  if (rows != channels) throw std::runtime_error("Beam angle CSV must contain one row for every channel");
  return angles;
}

void AppendLivox(FrameData & frame, const std::uint8_t * data, const LidarPacket & packet)
{
  const auto * point = data + kLivoxHeaderBytes;
  for (std::size_t i = 0; i < packet.point_count; ++i) {
    KittiPoint xyz{};
    if (packet.data_type == 1) {
      xyz = {Signed32(point)*.001f, Signed32(point+4)*.001f, Signed32(point+8)*.001f, static_cast<float>(point[12])};
      point += 14;
    } else if (packet.data_type == 2) {
      xyz = {Signed16(point)*.01f, Signed16(point+2)*.01f, Signed16(point+4)*.01f, static_cast<float>(point[6])};
      point += 8;
    } else {
      const float range = Le32(point)*.001f;
      const float zenith = Le16(point+4)*.01f*kRadians, phi = Le16(point+6)*.01f*kRadians;
      xyz = {range*std::sin(zenith)*std::cos(phi), range*std::sin(zenith)*std::sin(phi),
        range*std::cos(zenith), static_cast<float>(point[8])};
      point += 10;
    }
    if (xyz.x != 0 || xyz.y != 0 || xyz.z != 0) frame.points.push_back(xyz);
  }
}

void AppendRobosense(FrameData & frame, const std::uint8_t * data, Model model)
{
  const bool polar = model == Model::RobosenseM1;
  const std::size_t blocks = model == Model::RobosenseM3 ? 5 : 25;
  const std::size_t channels = model == Model::RobosenseM3 ? 28 : 5;
  const std::size_t stride = polar ? 9 : 10;
  const float max_range = polar ? 200.f : model == Model::RobosenseM2 ? 250.f : 300.f;
  const float min_range = model == Model::RobosenseM3 ? .1f : .2f;
  const auto * block = data + 32;
  for (std::size_t i = 0; i < blocks; ++i, block += 2 + stride * channels) {
    const auto * point = block + 2;
    for (std::size_t channel = 0; channel < channels; ++channel, point += stride) {
      const float range = Be16(point) * .005f;
      if (range < min_range || range > max_range) continue;
      if (polar) {
        const float pitch = (static_cast<int>(Be16(point + 2)) - 32768) * (.01f * kRadians);
        const float yaw = (static_cast<int>(Be16(point + 4)) - 32768) * (.01f * kRadians);
        const float xy = range * std::cos(pitch);
        frame.points.push_back({xy * std::cos(yaw), xy * std::sin(yaw), range * std::sin(pitch),
          static_cast<float>(point[6])});
      } else {
        const float component = range / 32768.f;
        frame.points.push_back({Signed(Be16(point + 2)) * component, Signed(Be16(point + 4)) * component,
          Signed(Be16(point + 6)) * component, static_cast<float>(point[8])});
      }
    }
  }
}

float VelodyneBlockStep(const LidarPacket & packet, std::size_t block)
{
  const std::size_t advance = packet.dual_return ? 2 : 1;
  const auto first = block + advance < packet.blocks ? block : block - advance;
  const int step = (36000 + packet.azimuth[first + advance] - packet.azimuth[first]) % 36000;
  return step <= 200 ? step * .01f : 0.f;
}
}  // namespace

const char * DirectLidarModelName(DirectLidarModel model)
{
  switch (model) {
    case Model::HesaiXt32: return "Hesai XT32";
    case Model::HesaiXt16: return "Hesai XT16";
    case Model::HesaiXt32M2X: return "Hesai XT32M2X";
    case Model::HesaiAt128: return "Hesai AT128 / AT128P";
    case Model::VelodyneVlp16: return "Velodyne VLP-16 / Puck";
    case Model::VelodynePuckHiRes: return "Velodyne Puck Hi-Res";
    case Model::VelodyneHdl32e: return "Velodyne HDL-32E";
    case Model::VelodyneVlp32c: return "Velodyne VLP-32C";
    case Model::LivoxMid360: return "Livox Mid-360";
    case Model::OusterOs: return "Ouster OS0 / OS1 / OS2";
    case Model::RobosenseM1: return "RoboSense M1";
    case Model::RobosenseM2: return "RoboSense M2";
    case Model::RobosenseM3: return "RoboSense M3";
    case Model::Auto: return "Auto";
  }
  return "Unknown";
}

std::uint16_t DirectLidarDefaultPort(DirectLidarModel model)
{
  if (model == Model::LivoxMid360) return 56301;
  if (IsRobosenseM(model)) return 6699;
  return model == Model::OusterOs ? 7502 : kDefaultLidarUdpPort;
}

const char * LidarFrameId(DirectLidarModel model)
{
  switch (model) {
    case Model::HesaiXt32: return "pandar_xt32_native";
    case Model::HesaiXt16: return "pandar_xt16_native";
    case Model::HesaiXt32M2X: return "hesai_xt32m2x";
    case Model::HesaiAt128: return "hesai_at128";
    case Model::LivoxMid360: return "livox_mid360";
    case Model::OusterOs: return "ouster_os";
    case Model::RobosenseM1: return "robosense_m1";
    case Model::RobosenseM2: return "robosense_m2";
    case Model::RobosenseM3: return "robosense_m3";
    default: return "velodyne";
  }
}

bool IsHesaiXt(DirectLidarModel model)
{
  return model == Model::HesaiXt32 || model == Model::HesaiXt16 || model == Model::HesaiXt32M2X;
}

std::optional<LidarPacket> InspectLidarPacket(const std::uint8_t * data, std::size_t size,
  const OusterCalibration * ouster, const HesaiAtCalibration * at)
{
  if (ouster) return ouster->Inspect(data,size);
  if (at) return InspectAt128(data, size, at);
  if (auto packet = InspectXt(data, size)) return packet;
  if (auto packet = InspectAt128(data, size, nullptr)) return packet;
  if (auto packet = InspectVelodyne(data, size)) return packet;
  if (auto packet = InspectRobosense(data, size)) return packet;
  return InspectMid360(data, size);
}

std::string DescribeLidarPacket(const std::uint8_t * data, std::size_t size)
{
  if (size >= 8 && data[0] == 0xee && data[1] == 0xff) {
    return "Unsupported or invalid Hesai packet (UDP " + std::to_string(data[2]) + "." +
      std::to_string(data[3]) + ", " + std::to_string(data[6]) + " channels, " +
      std::to_string(size) + " bytes).";
  }
  if (size == 1206 && Le16(data) == 0xeeff) {
    return "Unsupported Velodyne packet (product ID " + std::to_string(data[1205]) + ").";
  }
  return "Received UDP data outside the supported LiDAR formats.";
}

bool LidarSequence::Accept(std::optional<std::uint32_t> sequence)
{
  if (sequence && previous) {
    const std::uint32_t advance = *sequence - *previous;
    if (advance == 0 || advance >= 0x80000000u) return false;
    missing += advance - 1;
  }
  previous = sequence;
  return true;
}

std::int64_t LidarClock::AtFrame(const LidarPacket & packet, std::size_t block, std::int64_t received_ns)
{
  const auto sensor = packet.model == Model::OusterOs ? packet.column_timestamps[block] : packet.sensor_timestamp_ns;
  if (!last_frame) {
    last_frame = received_ns;
  } else {
    const auto arrival_gap = static_cast<std::uint64_t>(std::max<std::int64_t>(0, received_ns - last_received));
    auto gap = arrival_gap;
    if (last_sensor && sensor) {
      const bool wraps = packet.model == Model::VelodyneVlp16 || packet.model == Model::VelodynePuckHiRes ||
        packet.model == Model::VelodyneHdl32e || packet.model == Model::VelodyneVlp32c;
      const auto elapsed = sensor > last_sensor ? sensor - last_sensor :
        wraps && sensor < last_sensor ? kNanosecondsPerHour - last_sensor + sensor : 0;
      // A clock reset or a synchronization jump must not stall playback.
      if (elapsed && elapsed <= arrival_gap + 5000000000ull) gap = elapsed;
    }
    const auto remaining = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() - last_frame);
    last_frame += static_cast<std::int64_t>(std::min(gap, remaining));
  }
  last_received = received_ns;
  last_sensor = sensor;
  return last_frame;
}

bool LidarSweep::Crossed(const LidarPacket & packet, std::size_t block)
{
  if (packet.model == Model::HesaiAt128) {
    const auto sector = packet.scan_ids[block];
    if (sector == HesaiAtCalibration::kOutsideSector) return false;
    const bool crossed = time_slice && sector != *time_slice;
    time_slice = sector;
    return crossed;
  }
  if (IsRobosenseM(packet.model)) {
    const auto sequence = packet.scan_ids[0];
    const bool crossed = time_slice && sequence < 50 && *time_slice > sequence + 100;
    if (crossed || !time_slice || sequence > *time_slice) time_slice = sequence;
    return crossed;
  }
  if (packet.model == Model::LivoxMid360 || packet.model == Model::OusterOs) {
    // Nonrepeating pattern. The clock in the packet also works when replay
    // traffic is sent back-to-back instead of at the original network rate.
    const auto slice = packet.model == Model::OusterOs ? packet.scan_ids[block] : packet.sensor_timestamp_ns / 100000000u;
    const bool crossed = time_slice && slice != *time_slice;
    time_slice = slice;
    return crossed;
  }
  const auto angle = packet.azimuth[block];
  const bool crossed = previous && (*previous > 18000 && angle < 18000) &&
    (36000 + angle - *previous < 18000);
  previous = angle;
  return crossed;
}

std::string ReadLidarCalibration(const fs::path & path)
{
  constexpr std::streamsize maximum = 1024 * 1024;
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) throw std::runtime_error("Could not open the LiDAR calibration file");
  const auto size = file.tellg();
  if (size <= 0 || size > maximum) throw std::runtime_error("Invalid LiDAR calibration file size");
  std::string calibration(static_cast<std::size_t>(size), '\0');
  file.seekg(0);
  if (!file.read(calibration.data(), size)) throw std::runtime_error("Could not read the LiDAR calibration file");
  return calibration;
}

bool LooksLikeHesaiAtCalibration(std::string_view bytes)
{
  return bytes.size() >= 2 && static_cast<unsigned char>(bytes[0]) == 0xee &&
    static_cast<unsigned char>(bytes[1]) == 0xff;
}

HesaiAtCalibration::HesaiAtCalibration(std::string_view bytes)
{
  const auto * data = reinterpret_cast<const std::uint8_t *>(bytes.data());
  if (!LooksLikeHesaiAtCalibration(bytes) || bytes.size() < 16 || data[2] != 1 || data[3] != 5 ||
      data[4] != kBeams || data[5] == 0 || data[5] > start_.size() ||
      data[6] == 0 || data[6] > data[5] || data[15] == 0 ||
      bytes.size() != 48 + 8 * data[5] + kBeams * (8 + 2 * kSamples)) {
    throw std::runtime_error("Hesai AT128 / AT128P needs its v1.5 angle correction .dat file");
  }
  const auto division = data[15];
  sectors_ = data[6];
  adjustment_unit_ = division * .01f;
  for (std::size_t i = 0; i < sectors_; ++i) {
    const auto start = static_cast<std::uint64_t>(Le32(data + 16 + 4 * i)) * division;
    const auto end = static_cast<std::uint64_t>(Le32(data + 16 + 4 * (data[5] + i))) * division;
    if (start >= kAtCircle || end >= kAtCircle || start == end)
      throw std::runtime_error("Invalid scan sector in the Hesai angle correction file");
    start_[i] = static_cast<std::uint32_t>(start);
    width_[i] = static_cast<std::uint32_t>((end + kAtCircle - start) % kAtCircle);
  }
  const auto * angle = data + 16 + data[5] * 8;
  for (std::size_t i = 0; i < kBeams; ++i) {
    azimuth_[i] = Signed32(angle + i * 4) * (division / 25600.f);
    elevation_[i] = Signed32(angle + (kBeams + i) * 4) * (division / 25600.f);
    if (std::abs(azimuth_[i]) > 180.f || std::abs(elevation_[i]) > 90.f)
      throw std::runtime_error("Invalid beam angles in the Hesai angle correction file");
  }
  const auto * adjustments = angle + kBeams * 8;
  std::memcpy(azimuth_adjustment_.data(), adjustments, azimuth_adjustment_.size());
  std::memcpy(elevation_adjustment_.data(), adjustments + azimuth_adjustment_.size(), elevation_adjustment_.size());
}

std::uint32_t HesaiAtCalibration::Sector(std::uint32_t encoder) const
{
  for (std::size_t i = 0; i < sectors_; ++i) {
    if ((encoder + kAtCircle - start_[i]) % kAtCircle < width_[i]) return static_cast<std::uint32_t>(i);
  }
  return kOutsideSector;
}

void HesaiAtCalibration::AppendBlock(FrameData & frame, const std::uint8_t * data,
  const LidarPacket & packet, std::size_t block) const
{
  const auto sector = packet.scan_ids[block];
  if (sector == kOutsideSector) return;
  const auto * body = data + 12 + block * kAtBlockBytes;
  const auto encoder = AtEncoder(body);
  const float angle = ((encoder + kAtCircle - start_[sector]) % kAtCircle) / 12800.f;
  constexpr unsigned sample_interval = 2 * 25600;
  const auto sample = encoder / sample_interval;
  const auto next = (sample + 1) % kSamples;
  const float fraction = static_cast<float>(encoder % sample_interval) / sample_interval;
  const auto * point = body + 3;
  for (std::size_t channel = 0; channel < kBeams; ++channel, point += 4) {
    const auto count = Le16(point);
    if (!count) continue;
    const auto offset = channel * kSamples;
    const auto adjustment = [&](const auto & values) {
      const float first = values[offset + sample];
      return adjustment_unit_ * (first + (values[offset + next] - first) * fraction);
    };
    const float yaw = (angle - azimuth_[channel] + adjustment(azimuth_adjustment_)) * kRadians;
    const float pitch = (elevation_[channel] + adjustment(elevation_adjustment_)) * kRadians;
    const float range = count * (data[9] * .001f);
    const float horizontal = range * std::cos(pitch);
    frame.points.push_back({horizontal * std::sin(yaw), horizontal * std::cos(yaw), range * std::sin(pitch),
      static_cast<float>(point[2])});
  }
}

LidarPacketDecoder::LidarPacketDecoder(Model model, std::string_view calibration, const OusterCalibration * ouster,
  const HesaiAtCalibration * at)
: angles_(calibration.empty() || !IsHesaiXt(model) ? DefaultAngles(model) : ParseAngles(calibration, BeamCount(model))),
  ouster_(ouster), at_(at)
{
  if (model == Model::OusterOs && !ouster_) throw std::runtime_error("Ouster requires its sensor metadata JSON");
  if (model == Model::HesaiAt128 && !at_) throw std::runtime_error("Choose the AT128 sensor's angle correction .dat file");
  if (!calibration.empty() && !IsHesaiXt(model) && !ouster_ && !at_)
    throw std::runtime_error("Angle CSV calibration is supported for the Hesai XT series");
  for (std::size_t i = 0; i < BeamCount(model); ++i) {
    cos_elevation_[i] = std::cos(angles_.elevation[i] * kRadians);
    sin_elevation_[i] = std::sin(angles_.elevation[i] * kRadians);
    cos_azimuth_[i] = std::cos(angles_.azimuth[i] * kRadians);
    sin_azimuth_[i] = std::sin(angles_.azimuth[i] * kRadians);
  }
}

void LidarPacketDecoder::UpdateHesaiAzimuth(Model model, int rpm)
{
  if (rpm == previous_rpm_) return;
  for (std::size_t i = 0; i < 32; ++i) {
    const float firing_us = 6.f + (model == Model::HesaiXt32M2X ? 2.888f * (i % 16) : 1.512f * i);
    const float offset = angles_.azimuth[i] + rpm * 6e-6f * firing_us;
    cos_azimuth_[i] = std::cos(offset * kRadians);
    sin_azimuth_[i] = std::sin(offset * kRadians);
  }
  previous_rpm_ = rpm;
}

void LidarPacketDecoder::AppendBlock(FrameData & frame, const std::uint8_t * data,
  const LidarPacket & packet, std::size_t block)
{
  if (frame.points.size() > kMaxLidarScanPoints - 256) {
    throw std::runtime_error("LiDAR scan exceeds the point limit. Check the sensor's scan timing.");
  }
  if (packet.model == Model::OusterOs) {
    ouster_->AppendColumn(frame,data,packet,block);
    return;
  }
  if (packet.model == Model::HesaiAt128) {
    at_->AppendBlock(frame, data, packet, block);
    return;
  }
  if (packet.model == Model::LivoxMid360) {
    AppendLivox(frame, data, packet);
    return;
  }
  if (IsRobosenseM(packet.model)) {
    AppendRobosense(frame,data,packet.model);
    return;
  }
  const bool hesai = IsHesaiXt(packet.model);
  const std::size_t beams = BeamCount(packet.model), samples = hesai ? beams : 32;
  const std::size_t stride = hesai ? 4 : 3;
  const auto * point = hesai ? data + 14 + block * (2 + stride * beams) :
    data + 4 + block * kVelodyneBlockBytes;
  const float distance_unit = hesai || packet.model == Model::VelodyneVlp32c ? .004f : .002f;
  const float azimuth_deg = packet.azimuth[block] * .01f;
  const float c = std::cos(azimuth_deg*kRadians), s = std::sin(azimuth_deg*kRadians);
  if (hesai) UpdateHesaiAzimuth(packet.model, packet.motor_rpm);
  const float step = !hesai && packet.model != Model::VelodyneHdl32e ? VelodyneBlockStep(packet, block) : 0.f;
  for (std::size_t i = 0; i < samples; ++i, point += stride) {
    const auto count = Le16(point);
    if (count == 0) continue;
    const auto channel = i % beams;
    const float distance = count * distance_unit;
    const float horizontal = distance * cos_elevation_[channel];
    float x = c * cos_azimuth_[channel] - s * sin_azimuth_[channel];
    float y = s * cos_azimuth_[channel] + c * sin_azimuth_[channel];
    if (step != 0) {
      const float fraction = beams == 16 ?
        (i / 16 * 55.296f + channel * 2.304f) / 110.592f :
        (channel / 2 * 2.304f) / 55.296f;
      const float angle = (azimuth_deg + angles_.azimuth[channel] + step * fraction) * kRadians;
      x = std::cos(angle);
      y = std::sin(angle);
    }
    frame.points.push_back({horizontal*x, -horizontal*y, distance*sin_elevation_[channel], static_cast<float>(point[2])});
  }
}
}  // namespace lidar_viewer
