#include "lidar_viewer/bag_io.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#ifdef LIDAR_VIEWER_HAS_ROSBAG
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/info.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#ifdef LIDAR_VIEWER_HAS_STATIC_TF
#include <tf2_msgs/msg/tf_message.hpp>
#endif
#endif

bool IsRos2BagInput(const fs::path & input)
{
  if (fs::is_directory(input)) {
    return fs::is_regular_file(input / "metadata.yaml");
  }
  return fs::is_regular_file(input) &&
    (input.extension() == ".mcap" || input.extension() == ".db3" ||
    input.filename() == "metadata.yaml");
}

bool RosbagInputAvailable()
{
#ifdef LIDAR_VIEWER_HAS_ROSBAG
  return true;
#else
  return false;
#endif
}

namespace
{
std::uint64_t Little64(const unsigned char * bytes)
{
  std::uint64_t value=0;
  for (int i=0;i<8;++i) value|=static_cast<std::uint64_t>(bytes[i])<<(8*i);
  return value;
}

std::optional<std::uint64_t> McapDuration(const fs::path & path)
{
  // MCAP v0 Footer and Statistics, https://mcap.dev/spec . Seek over every
  // schema/chunk index; do not read/decompress messages or scan unfinished bags.
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if (!file) return {};
  const auto size=file.tellg();
  if (size<45) return {};
  constexpr std::array<unsigned char,8> magic{0x89,'M','C','A','P','0','\r','\n'};
  std::array<unsigned char,8> header{};
  file.seekg(0);file.read(reinterpret_cast<char *>(header.data()),header.size());
  if (header!=magic) return {};
  const auto footer_at=static_cast<std::uint64_t>(size)-37;
  std::array<unsigned char,37> footer{};
  file.seekg(static_cast<std::streamoff>(footer_at));
  file.read(reinterpret_cast<char *>(footer.data()),footer.size());
  if (!file || footer[0]!=0x02 || Little64(footer.data()+1)!=20 ||
      !std::equal(magic.begin(),magic.end(),footer.begin()+29)) return {};
  auto offset=Little64(footer.data()+9);
  const auto offsets_start=Little64(footer.data()+17);
  const auto end=offsets_start ? offsets_start : footer_at;
  if (offset<8 || offset>=end || end>footer_at) return {};
  // A malformed summary must not create an unbounded loop or allocation.
  for (std::size_t records=0;offset<end && records<100000;++records) {
    if (end-offset<9) return {};
    std::array<unsigned char,9> record{};
    file.seekg(static_cast<std::streamoff>(offset));
    file.read(reinterpret_cast<char *>(record.data()),record.size());
    const auto length=Little64(record.data()+1);
    if (!file || length>end-offset-9) return {};
    if (record[0]==0x0b) {
      if (length<46) return {};
      std::array<unsigned char,46> stats{};
      file.read(reinterpret_cast<char *>(stats.data()),stats.size());
      if (!file || !Little64(stats.data())) return {};
      const auto first=Little64(stats.data()+26),last=Little64(stats.data()+34);
      return last>=first ? std::optional<std::uint64_t>(last-first) : std::nullopt;
    }
    offset+=9+length;
  }
  return {};
}
} // namespace

std::optional<std::uint64_t> ReadBagDurationNs(const fs::path & input)
{
  try {
    const auto path=input.filename()=="metadata.yaml" ? input.parent_path() : input;
    if (path.extension()==".mcap" && fs::is_regular_file(path)) return McapDuration(path);
#ifdef LIDAR_VIEWER_HAS_ROSBAG
    rosbag2_storage::BagMetadata metadata;
    if (fs::is_directory(path) && fs::is_regular_file(path/"metadata.yaml") &&
        fs::file_size(path/"metadata.yaml")<=4*1024*1024) {
      metadata=rosbag2_storage::MetadataIo().read_metadata(path.string());
    } else if (path.extension()==".db3" && fs::is_regular_file(path)) {
      metadata=rosbag2_cpp::Info().read_metadata(path.string(),"sqlite3");
    } else return {};
    if (!metadata.message_count || metadata.duration.count()<0) return {};
    return static_cast<std::uint64_t>(metadata.duration.count());
#else
    return {};
#endif
  } catch (const std::exception &) {return {};}
}

#ifdef LIDAR_VIEWER_HAS_ROSBAG
namespace
{
using PointCloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

fs::path BagPath(const fs::path & input)
{
  const fs::path path = input.filename() == "metadata.yaml" ? input.parent_path() : input;
  if (!IsRos2BagInput(path)) {
    throw std::runtime_error("Expected a ROS 2 bag directory, .mcap, or .db3: " + input.string());
  }
  return fs::canonical(path);
}

void OpenReader(rosbag2_cpp::Reader & reader, const fs::path & path)
{
  rosbag2_storage::StorageOptions storage;
  storage.uri = path.string();
  if (path.extension() == ".mcap") {
    storage.storage_id = "mcap";
  } else if (path.extension() == ".db3") {
    storage.storage_id = "sqlite3";
  }
  reader.open(storage);
  if (!reader.set_read_order({rosbag2_storage::ReadOrder::ReceivedTimestamp, false})) {
    throw std::runtime_error("Bag storage does not support timestamp-ordered reading: " + path.string());
  }
}

enum class PointFormat {PointCloud2, Livox};

struct PointTopic
{
  std::string name;
  PointFormat format;
  bool cdr;
};

PointTopic ChooseTopic(rosbag2_cpp::Reader & reader, const std::string & requested)
{
  std::vector<PointTopic> point_topics;
  for (const auto & entry : reader.get_all_topics_and_types()) {
    const bool cdr = entry.serialization_format == "cdr";
    if (entry.type == "sensor_msgs/msg/PointCloud2")
      point_topics.push_back({entry.name, PointFormat::PointCloud2, cdr});
    else if (entry.type == "livox_ros_driver2/msg/CustomMsg")
      point_topics.push_back({entry.name, PointFormat::Livox, cdr});
  }
  const auto checked = [](const PointTopic & topic) {
    if (!topic.cdr) throw std::runtime_error("Point-cloud topic must use ROS CDR serialization: " + topic.name);
    return topic;
  };
  if (!requested.empty()) {
    for (const auto & topic : point_topics) {
      if (topic.name == requested) return checked(topic);
    }
  } else {
    const PointTopic * standard = nullptr;
    std::size_t standard_count = 0;
    for (const auto & topic : point_topics) {
      if (topic.format != PointFormat::PointCloud2) continue;
      if (topic.name == "/b2/points") return checked(topic);
      standard = &topic;
      ++standard_count;
    }
    if (standard_count == 1) return checked(*standard);
    if (point_topics.size() == 1) return checked(point_topics.front());
  }
  std::ostringstream error;
  error << (requested.empty() ? "Choose a point-cloud topic using --topic." :
    "No compatible point-cloud topic '" + requested + "'. Use --topic to select one.");
  error << " Available PointCloud2 or Livox CustomMsg topics:";
  for (const auto & topic : point_topics) {
    error << ' ' << topic.name;
  }
  if (point_topics.empty()) {
    error << " (none)";
  }
  throw std::runtime_error(error.str());
}

void FilterTopic(rosbag2_cpp::Reader & reader, const std::string & topic)
{
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);
}

std::size_t ScalarSize(std::uint8_t type)
{
  switch (type) {
    case Field::INT8:
    case Field::UINT8: return 1;
    case Field::INT16:
    case Field::UINT16: return 2;
    case Field::INT32:
    case Field::UINT32:
    case Field::FLOAT32: return 4;
    case Field::FLOAT64: return 8;
    default: throw std::runtime_error("Unsupported PointField datatype: " + std::to_string(type));
  }
}

const Field * FindField(const PointCloud & cloud, const std::string & name, bool required)
{
  const Field * result = nullptr;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }
    if (result) {
      throw std::runtime_error("Duplicate PointCloud2 field: " + name);
    }
    if (field.count != 1 ||
      static_cast<std::uint64_t>(field.offset) + ScalarSize(field.datatype) > cloud.point_step)
    {
      throw std::runtime_error("PointCloud2 field exceeds point_step or is not scalar: " + name);
    }
    result = &field;
  }
  if (required && !result) {
    throw std::runtime_error("PointCloud2 is missing field: " + name);
  }
  return result;
}

template<typename T>
T ReadNumber(const std::uint8_t * bytes, bool swap)
{
  std::array<std::uint8_t, sizeof(T)> data{};
  std::memcpy(data.data(), bytes, sizeof(T));
  if (swap) {
    std::reverse(data.begin(), data.end());
  }
  T value;
  std::memcpy(&value, data.data(), sizeof(T));
  return value;
}

double ReadScalar(const std::uint8_t * point, const Field & field, bool swap)
{
  const auto * bytes = point + field.offset;
  switch (field.datatype) {
    case Field::INT8: return ReadNumber<std::int8_t>(bytes, false);
    case Field::UINT8: return ReadNumber<std::uint8_t>(bytes, false);
    case Field::INT16: return ReadNumber<std::int16_t>(bytes, swap);
    case Field::UINT16: return ReadNumber<std::uint16_t>(bytes, swap);
    case Field::INT32: return ReadNumber<std::int32_t>(bytes, swap);
    case Field::UINT32: return ReadNumber<std::uint32_t>(bytes, swap);
    case Field::FLOAT32: return ReadNumber<float>(bytes, swap);
    case Field::FLOAT64: return ReadNumber<double>(bytes, swap);
    default: throw std::runtime_error("Unsupported PointField scalar datatype");
  }
}

bool HostBigEndian()
{
  const std::uint16_t value = 1;
  std::uint8_t byte;
  std::memcpy(&byte, &value, 1);
  return byte == 0;
}

class LivoxCdr
{
public:
  explicit LivoxCdr(const rcl_serialized_message_t & message)
  {
    if (!message.buffer || message.buffer_length < 4 || message.buffer[0] != 0 || message.buffer[1] > 1) {
      throw std::runtime_error("Livox CustomMsg must use plain CDR1 serialization");
    }
    // Alignment is relative to the payload after the CDR encapsulation.
    payload_ = message.buffer + 4;
    size_ = message.buffer_length - 4;
    swap_ = (message.buffer[1] == 0) != HostBigEndian();
  }

  std::size_t Remaining() const {return size_ - offset_;}
  bool SwapBytes() const {return swap_;}

  const std::uint8_t * Take(std::size_t bytes, std::size_t alignment = 1)
  {
    const auto padding = (alignment - offset_ % alignment) % alignment;
    if (padding > Remaining() || bytes > Remaining() - padding)
      throw std::runtime_error("Truncated Livox CustomMsg payload");
    offset_ += padding;
    const auto * data = payload_ + offset_;
    offset_ += bytes;
    return data;
  }

  std::uint32_t ReadUint32() {return ReadNumber<std::uint32_t>(Take(4, 4), swap_);}

private:
  const std::uint8_t * payload_{};
  std::size_t size_{}, offset_{};
  bool swap_{};
};

FrameData DecodeLivoxCloud(const rcl_serialized_message_t & message)
{
  // Layout defined by Livox-SDK/livox_ros_driver2 CustomMsg and CustomPoint.
  // Read XYZ and reflectivity directly from the bag without a driver dependency.
  LivoxCdr cdr(message);
  cdr.Take(8, 4); // Header stamp. Playback and IMU use the bag receipt timestamp.
  const auto frame_id_size = cdr.ReadUint32();
  const auto * frame_id = cdr.Take(frame_id_size);
  if (!frame_id_size || frame_id[frame_id_size - 1] != 0)
    throw std::runtime_error("Invalid Livox CustomMsg frame ID");
  cdr.Take(8, 8); // timebase
  const auto point_num = cdr.ReadUint32();
  cdr.Take(4); // lidar_id and reserved bytes
  const std::size_t count = cdr.ReadUint32();
  if (point_num != count) throw std::runtime_error("Livox CustomMsg point counts disagree");

  // offset_time, XYZ, reflectivity, tag and line. CDR pads only before the
  // next point's 4-byte offset_time, so the last point needs just 19 bytes.
  constexpr std::size_t stride = 20, point_bytes = 19;
  FrameData frame;
  if (count > frame.points.max_size() ||
    (count && (cdr.Remaining() < point_bytes || (count - 1) > (cdr.Remaining() - point_bytes) / stride)))
  {
    throw std::runtime_error("Truncated or oversized Livox CustomMsg point array");
  }
  const auto * points = cdr.Take(count ? (count - 1) * stride + point_bytes : 0);
  frame.frame_id.assign(reinterpret_cast<const char *>(frame_id), frame_id_size - 1);
  frame.points.reserve(count);
  const bool swap = cdr.SwapBytes();
  for (std::size_t i = 0; i < count; ++i) {
    const auto * point = points + i * stride;
    const KittiPoint decoded{ReadNumber<float>(point + 4, swap), ReadNumber<float>(point + 8, swap),
      ReadNumber<float>(point + 12, swap), static_cast<float>(point[16])};
    if (std::isfinite(decoded.x) && std::isfinite(decoded.y) && std::isfinite(decoded.z))
      frame.points.push_back(decoded);
  }
  return frame;
}

void NormalizeLivoxImuAcceleration(std::vector<lidar_viewer::ImuObservation> & observations)
{
  // Livox Driver2 forwards acceleration in g from the sensor. Confirm the
  // units from the recording, since some publishers already convert to SI.
  constexpr double gravity = 9.80665;
  std::size_t valid = 0, near_one_g = 0;
  for (const auto & sample : observations) {
    if (!sample.acceleration_valid) continue;
    const auto & gyro = sample.angular_velocity;
    const double turn_rate = std::hypot(gyro.x, gyro.y, gyro.z);
    // Use stationary samples to distinguish units, as in gravity estimation.
    const bool gyro_usable = sample.gyro_valid && turn_rate < 20;
    if (gyro_usable && turn_rate >= .35) continue;
    const auto & acceleration = sample.acceleration;
    const double magnitude = std::hypot(acceleration.x, acceleration.y, acceleration.z);
    if (!std::isfinite(magnitude)) continue;
    ++valid;
    if (std::abs(magnitude - 1) < 1.5 / gravity) ++near_one_g;
  }
  if (near_one_g <= valid / 2) return;
  for (auto & sample : observations) {
    auto & acceleration = sample.acceleration;
    acceleration.x *= gravity;
    acceleration.y *= gravity;
    acceleration.z *= gravity;
  }
}

struct ReaderCache
{
  fs::path path;
  std::string topic;
  PointFormat format{PointFormat::PointCloud2};
  std::unique_ptr<rosbag2_cpp::Reader> reader;
};
}  // namespace

FrameData DecodePointCloud2(const PointCloud & cloud)
{
  FrameData frame;
  frame.frame_id = cloud.header.frame_id;
  if (cloud.header.stamp.sec>=0 && cloud.header.stamp.nanosec<1000000000u)
    frame.timestamp_ns=static_cast<std::int64_t>(cloud.header.stamp.sec)*1000000000+cloud.header.stamp.nanosec;
  const std::uint64_t count = static_cast<std::uint64_t>(cloud.width) * cloud.height;
  if (count == 0) {
    return frame;
  }
  const std::uint64_t packed_row = static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
  const std::uint64_t required_bytes = static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
  if (cloud.point_step == 0 || packed_row > cloud.row_step || required_bytes > cloud.data.size()) {
    throw std::runtime_error("Truncated or inconsistent PointCloud2 point_step/row_step/data layout");
  }
  const Field * x = FindField(cloud, "x", true);
  const Field * y = FindField(cloud, "y", true);
  const Field * z = FindField(cloud, "z", true);
  const Field * intensity = FindField(cloud, "intensity", false);
  if (!intensity) {
    intensity = FindField(cloud, "reflectivity", false);
  }
  if (count > frame.points.max_size()) {
    throw std::runtime_error("PointCloud2 point count exceeds addressable memory");
  }
  frame.points.reserve(static_cast<std::size_t>(count));
  const bool swap = cloud.is_bigendian != HostBigEndian();
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const auto * row_data = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const auto * point = row_data + static_cast<std::size_t>(column) * cloud.point_step;
      KittiPoint decoded{
        static_cast<float>(ReadScalar(point, *x, swap)),
        static_cast<float>(ReadScalar(point, *y, swap)),
        static_cast<float>(ReadScalar(point, *z, swap)),
        intensity ? static_cast<float>(ReadScalar(point, *intensity, swap)) : 0.0f
      };
      if (!std::isfinite(decoded.x) || !std::isfinite(decoded.y) || !std::isfinite(decoded.z)) {
        continue;
      }
      if (!std::isfinite(decoded.intensity)) {
        decoded.intensity = 0.0f;
      }
      frame.points.push_back(decoded);
    }
  }
  return frame;
}

std::vector<FrameHandle> ResolveBagFrames(
  const fs::path & input, const std::string & requested_topic, const std::string & requested_imu,
  lidar_viewer::InputLoadState * load)
{
  if (load) load->Stage(lidar_viewer::InputLoadStage::Opening);
  const fs::path path = BagPath(input);
  rosbag2_cpp::Reader reader;
  OpenReader(reader, path);
  const auto point_topic = ChooseTopic(reader, requested_topic);
  const auto & topic = point_topic.name;
  auto imu = std::make_shared<lidar_viewer::ImuTimeline>();
  std::unordered_map<std::string, std::uint64_t> counts;
  for (const auto & entry : reader.get_metadata().topics_with_message_count)
    counts[entry.topic_metadata.name] = entry.message_count;
  std::vector<std::string> imu_topics;
  std::string static_tf_topic;
  for (const auto & entry : reader.get_all_topics_and_types()) {
    if (entry.serialization_format != "cdr" || counts[entry.name] == 0) continue;
    if (entry.type == "sensor_msgs/msg/Imu") imu_topics.push_back(entry.name);
    if (entry.name == "/tf_static" && entry.type == "tf2_msgs/msg/TFMessage") static_tf_topic = entry.name;
  }
  std::sort(imu_topics.begin(), imu_topics.end());
  if (!requested_imu.empty()) {
    if (std::find(imu_topics.begin(), imu_topics.end(), requested_imu) == imu_topics.end())
      throw std::runtime_error("No nonempty CDR IMU topic: " + requested_imu);
    imu->topic = requested_imu;
  } else if (!imu_topics.empty()) {
    // This adapter explicitly flags the VN100's unavailable attitude, unlike
    // the raw driver which publishes a placeholder identity quaternion.
    const auto vn100 = std::find(imu_topics.begin(), imu_topics.end(), "/vn100/imu/data_ned");
    imu->topic = vn100 == imu_topics.end() ? imu_topics.front() : *vn100;
  }
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  if (!imu->topic.empty()) filter.topics.push_back(imu->topic);
#ifdef LIDAR_VIEWER_HAS_STATIC_TF
  if (!static_tf_topic.empty()) filter.topics.push_back(static_tf_topic);
#endif
  reader.set_filter(filter);
  lidar_viewer::InputLoadProgress progress;
  progress.stage=lidar_viewer::InputLoadStage::Indexing;
  progress.total_frames=counts[topic];
  for (const auto & selected : filter.topics) progress.total_messages+=counts[selected];
  if (load) load->Publish(progress);
  auto next_progress=std::chrono::steady_clock::now();
  std::vector<lidar_viewer::ImuObservation> observations;
  bool inconsistent_imu_frame = false;
  std::size_t malformed_imu_records = 0;
  std::vector<FrameHandle> frames;
  std::unordered_map<std::int64_t, std::size_t> duplicates;
  std::cout << "Indexing ROS 2 bag " << path << " topic " << topic
            << " (messages remain on disk)..." << std::endl;
  while (reader.has_next()) {
    if (load) load->CheckCancelled();
    if (load && std::chrono::steady_clock::now()>=next_progress) {
      load->Publish(progress);
      next_progress=std::chrono::steady_clock::now()+std::chrono::milliseconds(80);
    }
    const auto message = reader.read_next();
    ++progress.messages;
    if (message->topic_name == imu->topic && !imu->topic.empty()) {
      try {
        sensor_msgs::msg::Imu sample;
        rclcpp::SerializedMessage serialized(*message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::Imu>().deserialize_message(&serialized, &sample);
        if (imu->frame_id.empty()) imu->frame_id = sample.header.frame_id;
        if (imu->frame_id != sample.header.frame_id || sample.header.frame_id.empty()) inconsistent_imu_frame = true;
        lidar_viewer::ImuObservation observation;
        observation.timestamp_ns = message->recv_timestamp;
        observation.acceleration = {sample.linear_acceleration.x, sample.linear_acceleration.y, sample.linear_acceleration.z};
        observation.angular_velocity = {sample.angular_velocity.x, sample.angular_velocity.y, sample.angular_velocity.z};
        observation.orientation = {sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w};
        observation.acceleration_valid = sample.linear_acceleration_covariance[0] >= 0;
        observation.gyro_valid = sample.angular_velocity_covariance[0] >= 0;
        observation.orientation_valid = sample.orientation_covariance[0] >= 0;
        observation.world_ned = sample.header.frame_id.find("_ned") != std::string::npos;
        observations.push_back(observation);
        ++progress.imu_samples;
      } catch (const std::exception &) { ++malformed_imu_records; }
      continue;
    }
#ifdef LIDAR_VIEWER_HAS_STATIC_TF
    if (message->topic_name == static_tf_topic && !static_tf_topic.empty()) {
      try {
        tf2_msgs::msg::TFMessage transforms;
        rclcpp::SerializedMessage serialized(*message->serialized_data);
        rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&serialized, &transforms);
        for (const auto & transform : transforms.transforms) {
          const auto & q = transform.transform.rotation;
          imu->static_rotations.push_back({transform.header.frame_id, transform.child_frame_id, {q.x, q.y, q.z, q.w}});
        }
      } catch (const std::exception &) { /* An invalid optional TF never prevents reading LiDAR. */ }
      continue;
    }
#endif
    if (message->topic_name != topic) {
      continue;
    }
    FrameHandle handle;
    handle.path = path;
    handle.format = InputFormat::Ros2Bag;
    handle.bag_topic = topic;
    handle.bag_timestamp_ns = message->recv_timestamp;
    handle.bag_duplicate_index = duplicates[handle.bag_timestamp_ns]++;
    std::ostringstream label;
    label << path.filename().string() << " #" << std::setw(6) << std::setfill('0') << frames.size();
    handle.label = label.str();
    frames.push_back(std::move(handle));
    progress.frames=frames.size();
    if (frames.size() % 1000 == 0) {
      std::cout << "  indexed " << frames.size() << " point-cloud frames" << std::endl;
    }
  }
  if (frames.empty()) {
    throw std::runtime_error("Bag topic has no point-cloud messages: " + topic);
  }
  progress.stage=lidar_viewer::InputLoadStage::PreparingImu;
  if (load) load->Publish(progress);
  std::stable_sort(frames.begin(), frames.end(), [](const FrameHandle & a, const FrameHandle & b) {
    return a.bag_timestamp_ns < b.bag_timestamp_ns;
  });
  if (inconsistent_imu_frame) imu->status = "IMU frame_id is empty or changes within this bag";
  else {
    imu->received_timestamps.reserve(observations.size());
    for (const auto & sample : observations) imu->received_timestamps.push_back(sample.timestamp_ns);
    // These sensor-family-1 recordings contain a placeholder attitude with
    // zero covariance. Preserve genuine VN100 attitude when it is present.
    if (imu->topic == "/vn100/vectornav_driver_node/imu/data" && imu->frame_id == "vn100_link_ned" &&
      !observations.empty() && std::all_of(observations.begin(), observations.end(), [](const auto & sample) {
        return std::abs(sample.orientation.x) < 1e-8 && std::abs(sample.orientation.y) < 1e-8 &&
          std::abs(sample.orientation.z) < 1e-8 && std::abs(std::abs(sample.orientation.w) - 1) < 1e-8;
      })) {
      for (auto & sample : observations) sample.orientation_valid = false;
    }
    if (point_topic.format == PointFormat::Livox && imu->frame_id == "livox_frame" &&
      (imu->topic == "/livox/imu" || imu->topic.rfind("/livox/imu_", 0) == 0))
    {
      NormalizeLivoxImuAcceleration(observations);
    }
    imu->samples = lidar_viewer::EstimateImuGravity(observations);
    imu->status = imu->topic.empty() ? "No IMU topic recorded" : "No reliable IMU gravity estimate";
  }
  if (load) load->CheckCancelled();
  for (auto & handle : frames) handle.imu_timeline = imu;
  if (!imu->topic.empty()) std::cout << "IMU: " << imu->topic << " | " << imu->samples.size()
    << " usable gravity samples | " << malformed_imu_records << " malformed records skipped\n";
  std::cout << "Indexed " << frames.size() << " frames; no coordinate, clock, or intensity conversion applied."
            << std::endl;
  return frames;
}

static FrameData ReadBagFrame(const FrameHandle & handle,ReaderCache & cache)
{
  const fs::path path = BagPath(handle.path);
  if (!cache.reader || cache.path != path || cache.topic != handle.bag_topic) {
    auto reader = std::make_unique<rosbag2_cpp::Reader>();
    OpenReader(*reader, path);
    const auto topic = ChooseTopic(*reader, handle.bag_topic);
    FilterTopic(*reader, topic.name);
    cache = {path, topic.name, topic.format, std::move(reader)};
  }
  cache.reader->seek(handle.bag_timestamp_ns);
  std::size_t duplicate = 0;
  while (cache.reader->has_next()) {
    const auto message = cache.reader->read_next();
    if (message->topic_name != handle.bag_topic || message->recv_timestamp < handle.bag_timestamp_ns) {
      continue;
    }
    if (message->recv_timestamp != handle.bag_timestamp_ns) {
      break;
    }
    if (duplicate++ != handle.bag_duplicate_index) {
      continue;
    }
    FrameData frame;
    if (cache.format == PointFormat::Livox) {
      frame = DecodeLivoxCloud(*message->serialized_data);
    } else {
      PointCloud cloud;
      rclcpp::SerializedMessage serialized(*message->serialized_data);
      rclcpp::Serialization<PointCloud>().deserialize_message(&serialized, &cloud);
      frame = DecodePointCloud2(cloud);
    }
    frame.timestamp_ns=handle.bag_timestamp_ns;
    if (handle.imu_timeline)
      frame.imu = lidar_viewer::MatchFrameImu(*handle.imu_timeline, handle.bag_timestamp_ns, frame.frame_id);
    return frame;
  }
  throw std::runtime_error("Indexed bag frame is missing; input bag may have changed: " + handle.label);
}
FrameData LoadBagFrame(const FrameHandle & handle,bool independent_reader)
{
  if (independent_reader) {
    // One extra reusable reader on the export worker. Batch export must not
    // steal the playback reader's cursor or hold its seek/decode mutex.
    thread_local ReaderCache independent;
    return ReadBagFrame(handle,independent);
  }
  static ReaderCache shared;
  static std::mutex mutex;
  const std::lock_guard<std::mutex> guard(mutex);
  return ReadBagFrame(handle,shared);
}
#else
std::vector<FrameHandle> ResolveBagFrames(const fs::path &, const std::string &, const std::string &,
  lidar_viewer::InputLoadState *)
{
  throw std::runtime_error(
    "ROS 2 bag support is not compiled. Source /opt/ros/jazzy/setup.bash and reconfigure CMake "
    "with ROS bag support enabled; .bin loading remains available without ROS.");
}

FrameData LoadBagFrame(const FrameHandle &,bool)
{
  throw std::runtime_error("This viewer was built without ROS 2 bag support");
}
#endif
