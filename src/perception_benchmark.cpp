#include "lidar_viewer/algorithms/ground_removal.hpp"
#include "lidar_viewer/learned_detector.hpp"
#include "lidar_viewer/object_tracker.hpp"
#include "lidar_viewer/terraseg.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;
using lidar_viewer::algorithms::Detection;
constexpr float CandidateScore = .10f;
constexpr std::size_t MaxPoints = 4000000;
constexpr std::string_view ManifestHeader =
  "scene\tframe\ttimestamp_ns\tkeyframe\tcloud\tr00\tr01\tr02\tr10\tr11\tr12\tr20\tr21\tr22\ttx\tty\ttz";

struct TrackingProfile
{
  std::string name;
  lidar_viewer::TrackingParams params;
  std::unique_ptr<lidar_viewer::ObjectTracker> tracker;
};

struct Options
{
  fs::path manifest;
  fs::path output;
  fs::path python_root{fs::u8path(LIDAR_VIEWER_SOURCE_DIR) / "python"};
  std::string method;
  std::string model;
  float score{.50f};
  std::vector<TrackingProfile> profiles;
};

struct Scan
{
  std::string scene;
  std::string frame;
  std::uint64_t scene_number = 0;
  std::uint64_t sequence = 0;
  std::int64_t timestamp_ns = 0;
  bool keyframe = false;
  fs::path cloud;
  std::array<float, 9> rotation{};
  float height = 0;
};

struct Cloud
{
  std::vector<KittiPoint> points;
  std::size_t source_count;
};

void PrintUsage()
{
  std::cout <<
    "Usage: lidar_studio_benchmark --manifest PATH --output DIR\n"
    "       (--ground METHOD | --detector MODEL) [--python-root PATH]\n"
    "       [--score FLOAT] [--track NAME:HITS:MISSED_SECONDS]...\n\n"
    "Ground methods: fast_grid, gseg3d, patchworkpp, patchwork_classic (patchwork),\n"
    "                fugseg, grid, ransac, terraseg\n"
    "Detector models: tiny, short_range, base, transfusion\n"
    "Tracking defaults: score 0.50, profile studio:3:0.8\n"
    "Output must be a new or empty directory. The manifest uses nuScenes XYZI-ring.\n";
}

std::vector<std::string_view> Split(std::string_view line, char delimiter)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  for (auto end = line.find(delimiter); end != std::string_view::npos; end = line.find(delimiter, start)) {
    fields.push_back(line.substr(start, end - start));
    start = end + 1;
  }
  fields.push_back(line.substr(start));
  return fields;
}

template<typename Integer>
Integer ParseInteger(std::string_view text, const char * name)
{
  Integer value{};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size())
    throw std::invalid_argument(std::string("Invalid ") + name + ": " + std::string(text));
  return value;
}

float ParseFloat(std::string_view text, const char * name)
{
  std::istringstream input{std::string(text)};
  input.imbue(std::locale::classic());
  float value{};
  if (!(input >> std::noskipws >> value) || !input.eof() || !std::isfinite(value))
    throw std::invalid_argument(std::string("Invalid ") + name + ": " + std::string(text));
  return value;
}

bool IsNameCharacter(char c)
{
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
    c == '_' || c == '-';
}

bool IsHexCharacter(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

TrackingProfile ParseProfile(std::string_view value)
{
  const auto fields = Split(value, ':');
  if (fields.size() != 3 || fields[0].empty() ||
      !std::all_of(fields[0].begin(), fields[0].end(), IsNameCharacter))
    throw std::invalid_argument("Tracking profile must be NAME:HITS:MISSED_SECONDS");
  TrackingProfile profile;
  profile.name = fields[0];
  profile.params.confirmation_hits = ParseInteger<int>(fields[1], "track confirmation hits");
  profile.params.max_missed_s = ParseFloat(fields[2], "track timeout");
  if (profile.params.confirmation_hits < 1 || profile.params.confirmation_hits > 20 ||
      profile.params.max_missed_s < .05f || profile.params.max_missed_s > 10.f)
    throw std::invalid_argument("Tracking requires 1–20 confirmation hits and a timeout of 0.05–10 seconds");
  return profile;
}

Options ParseOptions(const std::vector<std::string> & arguments)
{
  Options options;
  std::unordered_set<std::string> flags;
  for (std::size_t i = 0; i < arguments.size(); i += 2) {
    const auto & flag = arguments[i];
    if (i + 1 == arguments.size() || arguments[i + 1].empty())
      throw std::invalid_argument("Missing value for " + flag);
    const auto & value = arguments[i + 1];
    if (flag != "--track" && !flags.insert(flag).second)
      throw std::invalid_argument("Repeated flag " + flag);
    if (flag == "--manifest") options.manifest = fs::u8path(value);
    else if (flag == "--output") options.output = fs::u8path(value);
    else if (flag == "--ground") options.method = value;
    else if (flag == "--detector") options.model = value;
    else if (flag == "--python-root") options.python_root = fs::u8path(value);
    else if (flag == "--score") options.score = ParseFloat(value, "tracking score");
    else if (flag == "--track") {
      auto profile = ParseProfile(value);
      if (std::any_of(options.profiles.begin(), options.profiles.end(),
          [&](const auto & existing) { return existing.name == profile.name; }))
        throw std::invalid_argument("Duplicate tracking profile " + profile.name);
      options.profiles.push_back(std::move(profile));
    } else throw std::invalid_argument("Unknown option " + flag);
  }
  if (options.manifest.empty() || options.output.empty() || options.method.empty() == options.model.empty())
    throw std::invalid_argument("Specify --manifest, --output and exactly one of --ground and --detector");
  if (!options.method.empty()) {
    if (options.method == "patchwork") options.method = "patchwork_classic";
    const std::array<std::string_view, 8> methods = {
      "fast_grid", "gseg3d", "patchworkpp", "patchwork_classic", "fugseg", "grid", "ransac", "terraseg"
    };
    if (std::find(methods.begin(), methods.end(), options.method) == methods.end())
      throw std::invalid_argument("Unknown ground method " + options.method);
    if (!options.profiles.empty() || flags.count("--score"))
      throw std::invalid_argument("Tracking options require --detector");
  } else {
    if (!lidar_viewer::algorithms::IsDetectorModel(options.model))
      throw std::invalid_argument("Unknown detector " + options.model);
    if (options.score < CandidateScore || options.score > 1.f)
      throw std::invalid_argument("Tracking score must be between 0.10 and 1.00");
    if (options.profiles.empty()) options.profiles.push_back(ParseProfile("studio:3:0.8"));
  }
  return options;
}

void TrimCr(std::string & line)
{
  if (!line.empty() && line.back() == '\r') line.pop_back();
}

std::vector<Scan> ReadManifest(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("Cannot read manifest " + path.u8string());
  TrimCr(line);
  if (line != ManifestHeader) throw std::runtime_error("Unexpected manifest header");
  std::vector<Scan> scans;
  std::unordered_set<std::string> scenes, frames;
  while (std::getline(input, line)) {
    TrimCr(line);
    try {
      const auto fields = Split(line, '\t');
      if (fields.size() != 17 || fields[0].empty() || fields[1].empty() ||
          !std::all_of(fields[0].begin(), fields[0].end(), IsNameCharacter) ||
          !std::all_of(fields[1].begin(), fields[1].end(), IsHexCharacter))
        throw std::invalid_argument("Invalid columns or scene/frame identifiers");
      Scan scan;
      scan.scene = fields[0];
      scan.frame = fields[1];
      scan.timestamp_ns = ParseInteger<std::int64_t>(fields[2], "timestamp");
      if (scan.timestamp_ns <= 0 || (fields[3] != "0" && fields[3] != "1"))
        throw std::invalid_argument("Invalid timestamp or keyframe flag");
      scan.keyframe = fields[3] == "1";
      scan.cloud = fs::u8path(fields[4]);
      if (!scan.cloud.is_absolute()) throw std::invalid_argument("Scan path must be absolute");
      for (std::size_t i = 0; i < scan.rotation.size(); ++i)
        scan.rotation[i] = ParseFloat(fields[5 + i], "rotation");
      // x/y positions stay relative to the sensor; the score reader uses
      // these translations when transforming predictions back to ego coordinates.
      ParseFloat(fields[14], "translation x");
      ParseFloat(fields[15], "translation y");
      scan.height = ParseFloat(fields[16], "translation z");
      if (scan.height <= 0) throw std::invalid_argument("Sensor height must be positive");
      if (!frames.insert(scan.frame).second) throw std::invalid_argument("Duplicate frame token");
      if (scans.empty() || scan.scene != scans.back().scene) {
        if (!scenes.insert(scan.scene).second) throw std::invalid_argument("Scene resumes after another scene");
      } else {
        const auto & previous = scans.back();
        if (scan.timestamp_ns <= previous.timestamp_ns)
          throw std::invalid_argument("Frames within one scene must be chronological");
        scan.sequence = previous.sequence + 1;
      }
      scan.scene_number = scenes.size();
      scans.push_back(std::move(scan));
    } catch (const std::exception & error) {
      throw std::runtime_error("Manifest line " + std::to_string(scans.size() + 2) + ": " + error.what());
    }
  }
  if (!input.eof()) throw std::runtime_error("Cannot finish reading manifest");
  if (scans.empty() || std::none_of(scans.begin(), scans.end(), [](auto & scan) { return scan.keyframe; }))
    throw std::runtime_error("Manifest contains no keyframes");
  return scans;
}

bool IsLittleEndian()
{
  const std::uint32_t one = 1;
  return *reinterpret_cast<const unsigned char *>(&one) == 1;
}

Cloud LoadScan(const Scan & scan, bool filter_missing)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "nuScenes needs IEEE float32");
  constexpr std::size_t stride = 5 * sizeof(float);
  std::ifstream input(scan.cloud, std::ios::binary | std::ios::ate);
  const auto size = input.tellg();
  if (size <= 0 || size > static_cast<std::streamoff>(MaxPoints * stride) || size % stride != 0)
    throw std::runtime_error("Invalid nuScenes scan size: " + scan.cloud.u8string());
  std::vector<float> raw(static_cast<std::size_t>(size) / sizeof(float));
  input.seekg(0);
  input.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(size));
  if (!input) throw std::runtime_error("Cannot read scan " + scan.cloud.u8string());
  if (!IsLittleEndian()) {
    for (auto & item : raw) {
      auto * bytes = reinterpret_cast<unsigned char *>(&item);
      std::reverse(bytes, bytes + sizeof(float));
    }
  }
  Cloud cloud{{}, raw.size() / 5};
  cloud.points.reserve(cloud.source_count);
  const auto & r = scan.rotation;
  for (std::size_t i = 0; i < raw.size(); i += 5) {
    const float x = raw[i], y = raw[i + 1], z = raw[i + 2], intensity = raw[i + 3];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(intensity))
      throw std::runtime_error("Non-finite input point");
    if (filter_missing && x == 0 && y == 0 && z == 0) continue;
    const KittiPoint point{
      r[0] * x + r[1] * y + r[2] * z,
      r[3] * x + r[4] * y + r[5] * z,
      r[6] * x + r[7] * y + r[8] * z + scan.height,
      intensity
    };
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::runtime_error("Non-finite aligned point");
    cloud.points.push_back(point);
  }
  return cloud;
}

struct PointHash
{
  std::size_t operator()(const KittiPoint & point) const
  {
    std::size_t hash = 0;
    for (const float value : {point.x, point.y, point.z, point.intensity})
      hash ^= std::hash<float>{}(value) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
    return hash;
  }
};

struct PointEqual
{
  bool operator()(const KittiPoint & a, const KittiPoint & b) const
  {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.intensity == b.intensity;
  }
};

std::vector<unsigned char> GroundMask(const std::vector<KittiPoint> & points,
  const lidar_viewer::algorithms::GroundRemovalResult & classified)
{
  if (classified.ground_points.size() + classified.non_ground_points.size() != points.size())
    throw std::runtime_error("Ground algorithm did not return one decision per point");
  struct Counts { std::size_t remaining = 0; unsigned char decisions = 0; };
  std::unordered_map<KittiPoint, Counts, PointHash, PointEqual> positions;
  positions.reserve(points.size());
  for (const auto & point : points) ++positions[point].remaining;

  const auto assign = [&](const std::vector<KittiPoint> & output, unsigned char decision) {
    for (const auto & point : output) {
      const auto position = positions.find(point);
      if (position == positions.end() || !position->second.remaining)
        throw std::runtime_error("Ground algorithm did not preserve the original scan points");
      --position->second.remaining;
      position->second.decisions |= decision;
    }
  };
  assign(classified.non_ground_points, 1);
  assign(classified.ground_points, 2);

  // Exact duplicate XYZI values cannot be disambiguated after partitioning.
  // If a backend disagrees within that group, record every member as unscored.
  std::vector<unsigned char> mask;
  mask.reserve(points.size());
  for (const auto & point : points) mask.push_back(positions.at(point).decisions - 1);
  return mask;
}

std::ofstream OpenOutput(const fs::path & path)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Cannot write " + path.u8string());
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  return output;
}

void PrintProgress(const std::vector<Scan> & scans, std::size_t index)
{
  if (index == 0 || (index + 1) % 100 == 0 || index + 1 == scans.size())
    std::cout << "Processed " << index + 1 << '/' << scans.size() << " scans" << std::endl;
}

void RunGround(const Options & options, const std::vector<Scan> & scans, std::ostream & timings)
{
  lidar_viewer::algorithms::GroundRemovalParams params;
  params.enabled = true;
  params.mode = options.method;
  std::unique_ptr<lidar_viewer::TerraSeg> terra;
  if (params.mode == "terraseg") {
    terra = std::make_unique<lidar_viewer::TerraSeg>(options.python_root);
    const auto status = terra->Status();
    if (!status.configured) throw std::runtime_error(status.message);
  }

  fs::create_directory(options.output / "masks");
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const auto & scan = scans[i];
    try {
      const auto cloud = LoadScan(scan, false);
      const auto & points = cloud.points;
      params.sequence_id = scan.scene_number;
      params.sensor_height_m = scan.height;
      double ms = 0;
      lidar_viewer::algorithms::GroundRemovalResult result;
      if (terra) {
        auto prediction = terra->Segment(points, params.gseg3d_ground_height_m, 0, {0, 0, scan.height}, ms);
        if (!prediction) throw std::runtime_error(terra->Status().message);
        result = std::move(*prediction);
      } else {
        const auto started = Clock::now();
        result = lidar_viewer::algorithms::RunGroundSegmentation(points, params);
        ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      }
      if (scan.keyframe) {
        const auto mask = GroundMask(points, result);
        auto output = OpenOutput(options.output / "masks" / (scan.frame + ".bin"));
        output.write(reinterpret_cast<const char *>(mask.data()), static_cast<std::streamsize>(mask.size()));
        output.close();
      }
      timings << scan.scene << '\t' << scan.frame << '\t' << points.size() << '\t' << ms
              << "\t\t" << (terra ? "CUDA" : "CPU") << '\n';
      PrintProgress(scans, i);
    } catch (const std::exception & error) {
      throw std::runtime_error("Ground frame " + scan.frame + ": " + error.what());
    }
  }
}

template<typename Ready>
void WaitForDetector(const lidar_viewer::LearnedDetector & detector, Ready ready, const char * activity)
{
  const auto deadline = Clock::now() + std::chrono::seconds(120);
  do {
    const auto status = detector.Status();
    if (status.failed || !status.configured) throw std::runtime_error(status.message);
    if (ready(status)) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (Clock::now() < deadline);
  throw std::runtime_error(std::string("Detector timed out ") + activity);
}

void WriteBox(std::ostream & output, const Detection & detection, float score)
{
  const auto & box = detection.oriented_bounds;
  output << detection.class_id << '\t' << score << '\t' << box.center.x << '\t' << box.center.y << '\t'
         << box.center.z << '\t' << box.half_length * 2 << '\t' << box.half_width * 2 << '\t'
         << box.max_z - box.min_z << '\t' << std::atan2(box.axis_x.y, box.axis_x.x);
}

void RunDetector(Options & options, const std::vector<Scan> & scans, std::ostream & timings)
{
  lidar_viewer::LearnedDetector detector;
  detector.Start(options.python_root, options.model);
  WaitForDetector(detector, [](const auto & status) { return status.ready; }, "at startup");
  const auto provider = detector.Status().provider;
  auto detections = OpenOutput(options.output / "detections.tsv");
  auto tracks = OpenOutput(options.output / "tracks.tsv");
  auto tracker_timings = OpenOutput(options.output / "tracking-timings.tsv");
  constexpr const char * box_columns = "class_id\tscore\tx\ty\tz\tlength\twidth\theight\tyaw";
  detections << "scene\tframe\t" << box_columns << '\n';
  tracks << "scene\tframe\tprofile\tid\t" << box_columns << "\tconfirmed\tpredicted\n";
  tracker_timings << "scene\tframe\tprofile\ttracking_ms\n";
  for (auto & profile : options.profiles) profile.tracker = std::make_unique<lidar_viewer::ObjectTracker>();
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const auto & scan = scans[i];
    try {
      auto cloud = LoadScan(scan, true);
      lidar_viewer::DetectorResult result;
      // The live detector has a latest-only input slot. Wait here before
      // submitting the next frame so each timed scan has a matching result.
      if (!cloud.points.empty()) {
        detector.Submit(i + 1, std::make_shared<const std::vector<KittiPoint>>(std::move(cloud.points)),
          CandidateScore, 0);
        std::optional<lidar_viewer::DetectorResult> response;
        WaitForDetector(detector, [&](const auto &) {
          response = detector.Poll();
          return response.has_value();
        }, "awaiting scan results");
        if (!response->error.empty()) throw std::runtime_error(response->error);
        if (response->revision != i + 1) throw std::runtime_error("Unexpected detector result revision");
        result = std::move(*response);
      }
      for (const auto & box : result.detections) {
        detections << scan.scene << '\t' << scan.frame << '\t';
        WriteBox(detections, box, box.confidence);
        detections << '\n';
      }
      for (auto & profile : options.profiles) {
        const auto & output = profile.tracker->Update(result.detections,
          {scan.scene_number, scan.sequence, scan.timestamp_ns, false}, profile.params, options.score);
        for (const auto & track : output) {
          tracks << scan.scene << '\t' << scan.frame << '\t' << profile.name << '\t' << track.id << '\t';
          // Match the viewer visibility rule for weak and coasting updates.
          WriteBox(tracks, track.detection, track.confirmed ? track.last_strong_confidence : track.detection.confidence);
          tracks << '\t' << track.confirmed << '\t' << track.predicted << '\n';
        }
        tracker_timings << scan.scene << '\t' << scan.frame << '\t' << profile.name << '\t'
                        << profile.tracker->Stats().processing_ms << '\n';
      }
      timings << scan.scene << '\t' << scan.frame << '\t' << cloud.source_count << "\t\t"
              << result.processing_ms << '\t' << provider << '\n';
      PrintProgress(scans, i);
    } catch (const std::exception & error) {
      throw std::runtime_error("Detector frame " + scan.frame + ": " + error.what());
    }
  }
  detections.close();
  tracks.close();
  tracker_timings.close();
}

int Benchmark(const std::vector<std::string> & arguments)
{
  try {
    if (arguments.size() == 1 && (arguments.front() == "--help" || arguments.front() == "-h")) {
      PrintUsage();
      return 0;
    }
    auto options = ParseOptions(arguments);
    const auto scans = ReadManifest(options.manifest);
    if (fs::exists(options.output) && !fs::is_empty(options.output))
      throw std::runtime_error("Output directory is not empty: " + options.output.u8string());
    fs::create_directories(options.output);
    auto timings = OpenOutput(options.output / "timings.tsv");
    timings << "scene\tframe\tpoint_count\tground_ms\tdetector_ms\tprovider\n";
    if (options.model.empty()) RunGround(options, scans, timings);
    else RunDetector(options, scans, timings);
    timings.close();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "LiDAR Studio benchmark: " << error.what() << '\n';
    return 1;
  }
}
}  // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t ** argv)
{
  std::vector<std::string> arguments;
  for (int i = 1; i < argc; ++i) arguments.push_back(fs::path(argv[i]).u8string());
  return Benchmark(arguments);
}
#else
int main(int argc, char ** argv)
{
  return Benchmark(std::vector<std::string>(argv + 1, argv + argc));
}
#endif
