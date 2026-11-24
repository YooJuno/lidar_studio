#include "lidar_viewer/benchmark_ui.hpp"

#include "lidar_viewer/config.hpp"
#include "lidar_viewer/file_dialog.hpp"
#include "lidar_viewer/platform.hpp"
#include "lidar_viewer/training_job.hpp"
#include "lidar_viewer/ui_theme.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lidar_viewer
{
namespace
{
namespace fs = std::filesystem;
using boost::property_tree::ptree;

struct BenchmarkRow
{
  std::string stage, method, metric, details, backend;
  std::optional<double> value, median, tail;
};

struct FailureExample
{
  std::string stage, method, scene, frame, description;
};

struct BreakdownRow
{
  std::string method;
  std::size_t support;
  std::vector<std::pair<std::string, std::optional<double>>> metrics;
  std::string counts;
};

using BreakdownScope = std::map<std::string, std::vector<BreakdownRow>>;
using BreakdownStage = std::map<std::string, BreakdownScope>;

struct ModelProof
{
  std::string title;
  std::map<std::string, std::string> hashes;
};

struct BenchmarkReport
{
  fs::path path;
  std::string version, split;
  std::size_t scans, keyframes, scenes, point_labelled;
  bool keyframes_only;
  double detector_score;
  std::vector<BenchmarkRow> rows;
  std::vector<FailureExample> cases;
  std::map<std::string, BreakdownStage> breakdowns;
  std::string selection_hash, comparison_hash, runner_hash, timing_host_hash, timing_host;
  bool cloud_payloads_hashed;
  std::vector<ModelProof> models;
};

std::optional<double> ReadMetric(const ptree & field)
{
  const auto value = field.get_value<std::string>();
  if (value == "null") return std::nullopt;
  const auto number = field.get_value<double>();
  if (!std::isfinite(number)) throw std::runtime_error("A report metric is not finite.");
  return number;
}

bool IsFingerprint(const std::string & value)
{
  return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
}

bool CanCompare(const BenchmarkReport & report)
{
  return IsFingerprint(report.selection_hash) && IsFingerprint(report.comparison_hash);
}

bool MatchingTimingHost(const BenchmarkReport & report, const BenchmarkReport & baseline)
{
  return IsFingerprint(report.timing_host_hash) && report.timing_host_hash == baseline.timing_host_hash;
}

std::string FormatCount(std::size_t count)
{
  auto text = std::to_string(count);
  for (auto pos = text.size(); pos > 3; ) {
    pos -= 3;
    text.insert(pos, 1, ',');
  }
  return text;
}

std::string CountLabel(const std::string & name)
{
  if (name == "Non-ground points removed") return "Obstacles lost";
  if (name == "Ground points missed") return "Missed ground";
  if (name == "Ambiguous points") return "Ambiguous";
  if (name == "Predicted false positives") return "Predicted FP";
  return name;
}

BenchmarkReport LoadReport(fs::path path)
{
  if (!fs::is_regular_file(path) || fs::file_size(path) > 16 * 1024 * 1024)
    throw std::runtime_error("Choose a benchmark.json report smaller than 16 MiB.");
  ptree root;
  std::ifstream file(path);
  boost::property_tree::read_json(file, root);
  if (root.get<std::string>("format") != "lidar-studio-perception-benchmark" || root.get<int>("schema") != 1)
    throw std::runtime_error("This benchmark report format is unsupported.");
  const auto & data = root.get_child("dataset");
  BenchmarkReport report{std::move(path), data.get<std::string>("version"), data.get<std::string>("split"),
    data.get<std::size_t>("scans"), data.get<std::size_t>("keyframes"), data.get_child("scenes").size(),
    data.get<std::size_t>("point_labelled_keyframes"), data.get<bool>("keyframes_only"),
    root.get<double>("settings.detector_score"), {}, {}, {}, data.get<std::string>("selection_sha256", ""),
    root.get<std::string>("settings.comparison_sha256", ""), root.get<std::string>("environment.runner_sha256", ""),
    root.get<std::string>("environment.timing_host_sha256", ""), root.get<std::string>("environment.timing_host", ""),
    data.get<bool>("cloud_payloads_hashed", false), {}};
  for (const auto & entry : root.get_child("rows")) {
    const auto & row = entry.second;
    report.rows.push_back({row.get<std::string>("stage"), row.get<std::string>("method"),
      row.get<std::string>("metric"), row.get<std::string>("details"), row.get<std::string>("backend", ""), ReadMetric(row.get_child("value")),
      ReadMetric(row.get_child("p50_ms")), ReadMetric(row.get_child("p95_ms"))});
  }
  if (const auto examples = root.get_child_optional("examples")) {
    for (const auto & entry : *examples) {
      const auto & row = entry.second;
      report.cases.push_back({row.get<std::string>("stage"), row.get<std::string>("method"),
        row.get<std::string>("scene"), row.get<std::string>("frame"), row.get<std::string>("description")});
    }
  }
  if (const auto breakdowns = root.get_child_optional("breakdowns")) {
    for (const auto & entry : *breakdowns) {
      const auto & row = entry.second;
      auto & group = report.breakdowns[row.get<std::string>("stage")][row.get<std::string>("scope")]
        [row.get<std::string>("label")];
      BreakdownRow detail{row.get<std::string>("method"), row.get<std::size_t>("support"), {}, {}};
      for (const auto & metric : row.get_child("metrics"))
        detail.metrics.emplace_back(metric.first, ReadMetric(metric.second));
      if (detail.metrics.empty() || detail.metrics.size() > 24)
        throw std::runtime_error("A report detail has an invalid number of metrics.");
      if (const auto counts = row.get_child_optional("counts")) {
        for (const auto & count : *counts) {
          if (!detail.counts.empty()) detail.counts += '\n';
          detail.counts += CountLabel(count.first) + " " + FormatCount(count.second.get_value<std::size_t>());
        }
      }
      group.push_back(std::move(detail));
    }
  }
  if (const auto models = root.get_child_optional("detector_models")) {
    for (const auto & model : *models) {
      ModelProof proof{model.second.get<std::string>("title", model.first), {}};
      if (const auto weights = model.second.get_child_optional("weights_sha256"))
        for (const auto & weight : *weights) proof.hashes.emplace(weight.first, weight.second.get_value<std::string>());
      report.models.push_back(std::move(proof));
    }
  }
  return report;
}

void TimingCell(int column, const std::optional<double> & value, const std::optional<double> & baseline)
{
  ImGui::TableSetColumnIndex(column);
  if (!value) {ImGui::TextDisabled("--"); return;}
  ImGui::Text("%.2f ms", *value);
  if (baseline) ImGui::TextDisabled("%+.2f ms", *value - *baseline);
}

struct DatasetPreset
{
  const char * label;
  const char * version;
  const char * split;
};

constexpr DatasetPreset kDatasets[]{
  {"Mini validation (81 keyframes)", "v1.0-mini", "mini_val"},
  {"Mini all (404 keyframes)", "v1.0-mini", "all"},
  {"Full validation", "v1.0-trainval", "val"},
};

const BenchmarkRow * MatchingRow(const BenchmarkReport * report, const BenchmarkRow & target)
{
  if (!report) return nullptr;
  const auto row = std::find_if(report->rows.begin(), report->rows.end(), [&](const auto & item) {
    return item.stage == target.stage && item.method == target.method && item.metric == target.metric;
  });
  return row == report->rows.end() ? nullptr : &*row;
}

std::optional<double> FindDetailValue(const BreakdownRow * row, const std::string & name)
{
  if (!row) return std::nullopt;
  const auto value = std::find_if(row->metrics.begin(), row->metrics.end(), [&](const auto & metric) {
    return metric.first == name;
  });
  return value == row->metrics.end() ? std::nullopt : value->second;
}

void ScoreCell(const std::optional<double> & value, const std::optional<double> & baseline = std::nullopt)
{
  if (!value) {ImGui::TextDisabled("--"); return;}
  ImGui::Text("%.2f%%", *value * 100.);
  if (baseline) ImGui::TextDisabled("%+.2f pp", (*value - *baseline) * 100.);
}

const char * DetailHeader(const std::string & metric)
{
  if (metric == "Pedestrian point loss") return "Person loss";
  if (metric == "Vehicle point loss") return "Vehicle loss";
  return metric.c_str();
}
}  // namespace

struct BenchmarkWorkspace::Impl
{
  struct Method { const char * id; const char * label; bool enabled; };
  enum class Pick {Dataset, Report, Baseline};
  FileDialog picker;
  Pick picking = Pick::Dataset;
  bool loading_baseline = false;
  std::future<BenchmarkReport> loading;
  std::optional<BenchmarkReport> report, baseline;
  std::string stage = "Ground", scope, label;
  std::string error, log;
  TrainingJob job;
  fs::path run;
  bool awaiting_result = false, cancelled = false, show_results = false;
  double last_poll = -1;
  int limit = 0;
  int dataset_preset = 0;
  bool hash_clouds = false;
  float score = .50f;
  std::array<char, 4096> dataset{};
#ifdef _WIN32
  std::array<char, 4096> python{{'p','y','t','h','o','n'}};
#else
  std::array<char, 4096> python{{'p','y','t','h','o','n','3'}};
#endif
  Method ground[9]{{"fast_grid", "Fast grid", true}, {"fugseg", "FugSeg", true},
    {"gseg3d", "GSeg3D", true}, {"patchworkpp", "Patchwork++", true},
    {"patchwork_classic", "Patchwork", false}, {"grid", "Grid", false},
    {"ransac", "RANSAC", false}, {"terraseg", "TerraSeg", false}, {"linefit", "LineFit", false}};
  Method detectors[4]{{"short_range", "AWML ShortRange", false}, {"tiny", "CenterPoint tiny", false},
    {"base", "CenterPoint base", false}, {"transfusion", "TransFusion", false}};
  bool compare_timeout = true;

  void Start()
  {
    const auto folder = fs::u8path(dataset.data());
    if (!fs::is_directory(folder)) throw std::runtime_error("Choose an extracted nuScenes folder.");
    const auto & preset = kDatasets[dataset_preset];
    const auto script = ProjectResourcePath("tools/benchmark_perception.py");
    if (!fs::is_regular_file(script)) throw std::runtime_error("The benchmark Python scripts are missing.");
    const auto parent = UserHome() / "LiDAR Studio Benchmarks";
    fs::create_directories(parent);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    run = parent / (std::string(preset.version) + "-" + preset.split + "-" + std::to_string(millis) + "-" + std::to_string(ProcessId()));
    std::vector<std::string> arguments{python.data(), "-X", "utf8", "-u", script.u8string(), "--dataset", folder.u8string(),
      "--runner", ExecutablePath().u8string(), "--studio-runner", "--python-root", ProjectResourcePath("python").u8string(),
      "--output", run.u8string(), "--version", preset.version, "--split", preset.split};
    if (limit > 0) arguments.insert(arguments.end(), {"--max-keyframes", std::to_string(limit)});
    if (hash_clouds) arguments.emplace_back("--hash-clouds");
    const auto add_methods = [&](const auto & methods, const char * flag) {
      bool selected = false;
      for (const auto & method : methods) {
        if (!method.enabled) continue;
        selected = true;
        arguments.insert(arguments.end(), {flag, method.id});
      }
      return selected;
    };
    const bool has_ground = add_methods(ground, "--ground");
    const bool has_detector = add_methods(detectors, "--detector");
    if (!has_ground) arguments.emplace_back("--no-ground");
    if (!has_ground && !has_detector) throw std::runtime_error("Select at least one method.");
    if (has_detector) {
      arguments.insert(arguments.end(), {"--score", std::to_string(score)});
      if (compare_timeout) arguments.insert(arguments.end(), {"--track", "studio:3:0.8", "--track", "strict:3:0.3"});
    }
    auto log_path = run;
    log_path += ".log";
    job.StartCommand(arguments, log_path);
    error.clear(); log.clear(); last_poll = -1;
    awaiting_result = true; cancelled = false;
  }

  void Poll()
  {
    try {
      job.Poll();
      if (awaiting_result && (!job.Running() || ImGui::GetTime() - last_poll >= .5)) {
        log = job.LogTail(8192);
        last_poll = ImGui::GetTime();
        if (!job.Running()) {
          awaiting_result = false;
          const auto file = run / "benchmark.json";
          if (!cancelled && fs::is_regular_file(file)) {
            loading_baseline = false;
            loading = std::async(std::launch::async, LoadReport, file);
          } else if (!cancelled) error = "Benchmark ended before writing the report. See the run log.";
        }
      }
      if (const auto result = picker.Poll()) {
        if (!result->error.empty()) throw std::runtime_error(result->error);
        if (!result->path.empty()) {
          error.clear();
          if (picking == Pick::Dataset) {
            const auto path = result->path.u8string();
            if (path.size() >= dataset.size()) throw std::runtime_error("Dataset path is too long.");
            dataset.fill(0);
            std::copy(path.begin(), path.end(), dataset.begin());
          } else {
            loading_baseline = picking == Pick::Baseline;
            loading = std::async(std::launch::async, LoadReport, result->path);
          }
        }
      }
      if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        if (loading_baseline) baseline = loading.get();
        else report = loading.get();
        show_results = true;
      }
    } catch (const std::exception & problem) {
      error = problem.what();
    }
  }
  void DrawRun(float scale);
  void DrawResults(float scale);
  void DrawSummary(float scale, const BenchmarkReport * compared);
  void DrawDetails(float scale, const BreakdownStage & details, const BenchmarkReport * compared);
  void DrawFailures(float scale);
};

void BenchmarkWorkspace::Impl::DrawRun(float scale)
{
  const bool busy = job.Running() || loading.valid() || picker.Active();
  ImGui::BeginDisabled(busy);
  ImGui::TextUnformatted("nuScenes dataset");
  ImGui::SetNextItemWidth(-100 * scale);
  ImGui::InputTextWithHint("##Dataset", "Extracted nuScenes folder", dataset.data(), dataset.size());
  ImGui::SameLine();
  if (ImGui::Button("Browse...")) {
    try { picking = Pick::Dataset; picker.OpenDirectory(UserHome()); }
    catch (const std::exception & problem) { error = problem.what(); }
  }
  ImGui::SetNextItemWidth(270 * scale);
  if (ImGui::BeginCombo("Split", kDatasets[dataset_preset].label)) {
    for (int i = 0; i < static_cast<int>(std::size(kDatasets)); ++i)
      if (ImGui::Selectable(kDatasets[i].label, i == dataset_preset)) dataset_preset = i;
    ImGui::EndCombo();
  }
  const auto draw_methods = [](const char * title, auto & methods) {
    ImGui::SeparatorText(title);
    if (ImGui::BeginTable(title, 3)) {
      for (auto & method : methods) {
        ImGui::TableNextColumn();
        ImGui::Checkbox(method.label, &method.enabled);
      }
      ImGui::EndTable();
    }
  };
  draw_methods("Ground segmentation", ground);
  draw_methods("Object detection", detectors);
  const bool track = std::any_of(std::begin(detectors), std::end(detectors), [](const auto & m) {return m.enabled;});
  ImGui::BeginDisabled(!track);
  ImGui::Checkbox("Compare track timeouts (0.8 s and 0.3 s)", &compare_timeout);
  ImGui::EndDisabled();
  if (ImGui::CollapsingHeader("Run settings")) {
    ImGui::InputText("Python executable", python.data(), python.size());
    ImGui::TextWrapped("Python requires NumPy and SciPy. Full validation also requires ijson and nuscenes-devkit.");
    ImGui::SetNextItemWidth(100 * scale);
    ImGui::InputInt("Keyframe limit (0 for entire split)", &limit, 0);
    limit = std::max(0, limit);
    ImGui::Checkbox("Hash point cloud files", &hash_clouds);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Verify raw scan bytes for baseline comparisons. Reads every selected scan once before evaluation.");
    ImGui::BeginDisabled(!track);
    ImGui::SetNextItemWidth(140 * scale);
    ImGui::InputFloat("Detector score", &score, .01f, .10f, "%.2f");
    score = std::isfinite(score) ? std::clamp(score, .10f, 1.f) : .50f;
    ImGui::EndDisabled();
  }
  if (ImGui::Button("Run comparison")) {
    try { Start(); }
    catch (const std::exception & problem) { error = problem.what(); }
  }
  ImGui::EndDisabled();
  if (job.Running()) {
    ImGui::SameLine();
    if (ImGui::Button("Cancel run")) { cancelled = true; job.Cancel(); }
  } else if (cancelled) { ImGui::SameLine(); ImGui::TextDisabled("Cancelled"); }
  if (!log.empty()) {
    if (ImGui::BeginChild("Run log", {0, 112 * scale}, ImGuiChildFlags_Borders)) {
      ImGui::TextUnformatted(log.c_str());
      if (job.Running()) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
  }
}

void BenchmarkWorkspace::Impl::DrawResults(float scale)
{
  ImGui::BeginDisabled(picker.Active() || loading.valid() || job.Running());
  if (ImGui::Button("Open report...")) {
    try { picking = Pick::Report; picker.OpenFile(report ? report->path.parent_path() : UserHome()); }
    catch (const std::exception & problem) { error = problem.what(); }
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(!report || !CanCompare(*report));
  if (ImGui::Button("Compare with baseline...")) {
    try { picking = Pick::Baseline; picker.OpenFile(report->path.parent_path()); }
    catch (const std::exception & problem) { error = problem.what(); }
  }
  if (report && !CanCompare(*report) && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Run a new benchmark to include data and protocol fingerprints.");
  ImGui::EndDisabled();
  ImGui::EndDisabled();
  if (report) {
    if (ImGui::GetContentRegionAvail().x > 480 * scale) ImGui::SameLine();
    if (ImGui::Button("Copy folder"))
      ImGui::SetClipboardText(report->path.parent_path().u8string().c_str());
  }
  if (loading.valid()) ImGui::TextDisabled("Reading benchmark report...");
  if (!report) {
    ImGui::TextWrapped("Open a benchmark.json from the perception benchmark to compare scores, processing time and errors.");
    return;
  }
  ImGui::TextWrapped("nuScenes %s / %s  |  %zu scenes  |  %zu scans  |  %zu scored keyframes",
    report->version.c_str(), report->split.c_str(), report->scenes, report->scans, report->keyframes);
  ImGui::TextWrapped("%zu with point labels  |  %s", report->point_labelled,
    report->keyframes_only ? "Keyframes only" : "Ordered replay with intermediate scans");

  const BenchmarkReport * compared = nullptr;
  if (baseline) {
    ImGui::TextWrapped("Baseline report from %s", baseline->path.parent_path().filename().u8string().c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear##Baseline")) baseline.reset();
    else if (!CanCompare(*baseline) || !CanCompare(*report))
      ImGui::TextWrapped("Deltas require both a data fingerprint and a scoring protocol fingerprint.");
    else if (report->selection_hash != baseline->selection_hash)
      ImGui::TextWrapped("Deltas unavailable because the data selections or scan hashing modes differ.");
    else if (report->comparison_hash != baseline->comparison_hash)
      ImGui::TextWrapped("Deltas unavailable because the scoring settings differ.");
    else {
      compared = &*baseline;
      ImGui::TextWrapped("Deltas use matching methods. Timing deltas require matching hardware, drivers and inference backends.");
    }
  }
  if (ImGui::CollapsingHeader("Reproducibility")) {
    if (IsFingerprint(report->selection_hash)) {
      ImGui::TextWrapped("Data selection fingerprint %s", report->selection_hash.c_str());
      ImGui::TextWrapped("Point cloud verification used %s", report->cloud_payloads_hashed ? "all scan bytes" : "scan metadata");
    }
    if (IsFingerprint(report->comparison_hash)) ImGui::TextWrapped("Scoring protocol fingerprint %s", report->comparison_hash.c_str());
    if (IsFingerprint(report->runner_hash)) ImGui::TextWrapped("Runner SHA-256 %s", report->runner_hash.c_str());
    if (!report->timing_host.empty()) ImGui::TextWrapped("Timing host %s", report->timing_host.c_str());
    for (const auto & model : report->models) {
      ImGui::TextWrapped("%s", model.title.c_str());
      ImGui::Indent();
      for (const auto & [file, digest] : model.hashes)
        ImGui::TextWrapped("%s  %s", file.c_str(), digest.c_str());
      ImGui::Unindent();
    }
    ImGui::TextWrapped("Timings and errors for each frame are saved alongside benchmark.json.");
  }

  std::vector<const char *> stages;
  for (const auto * name : {"Ground", "Detection", "Tracking"})
    if (std::any_of(report->rows.begin(), report->rows.end(), [name](const auto & row) {return row.stage == name;}))
      stages.push_back(name);
  if (stages.empty()) return;
  if (std::find(stages.begin(), stages.end(), stage) == stages.end()) stage = stages.front();
  ImGui::SetNextItemWidth(140 * scale);
  if (ImGui::BeginCombo("Stage", stage.c_str())) {
    for (const auto * name : stages)
      if (ImGui::Selectable(name, stage == name)) {stage = name; scope.clear(); label.clear();}
    ImGui::EndCombo();
  }
  const auto input = report->breakdowns.find(stage);
  if (input != report->breakdowns.end()) {
    if (ImGui::GetContentRegionAvail().x > 360 * scale) ImGui::SameLine();
    ImGui::SetNextItemWidth(140 * scale);
    if (input->second.count(scope) == 0) scope.clear();
    if (ImGui::BeginCombo("View", scope.empty() ? "Summary" : scope.c_str())) {
      if (ImGui::Selectable("Summary", scope.empty())) scope.clear();
      for (const auto * name : {"Overall", "Scene", "Class"})
        if (input->second.count(name) && ImGui::Selectable(name, scope == name)) {scope = name; label.clear();}
      ImGui::EndCombo();
    }
  } else scope.clear();
  if (stage != "Ground") ImGui::TextDisabled("Counts at detector score %.2f", report->detector_score);
  if (!scope.empty()) DrawDetails(scale, input->second, compared);
  else DrawSummary(scale, compared);
  DrawFailures(scale);
}

void BenchmarkWorkspace::Impl::DrawSummary(float scale, const BenchmarkReport * compared)
{
  const bool matching_host = compared && MatchingTimingHost(*report, *compared);
  if (!ImGui::BeginTable("##Summary", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
      ImGuiTableFlags_BordersInnerH)) return;
  ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthStretch, 1.7f);
  ImGui::TableSetupColumn("Score", ImGuiTableColumnFlags_WidthStretch, 1.5f);
  ImGui::TableSetupColumn("p50", ImGuiTableColumnFlags_WidthFixed, 76 * scale);
  ImGui::TableSetupColumn("p95", ImGuiTableColumnFlags_WidthFixed, 76 * scale);
  ImGui::TableSetupColumn("Errors", ImGuiTableColumnFlags_WidthStretch, 2.7f);
  ImGui::TableHeadersRow();
  for (const auto & row : report->rows) {
    if (row.stage != stage) continue;
    const auto * prior = MatchingRow(compared, row);
    const bool show_timing = prior && matching_host && !row.backend.empty() && row.backend == prior->backend;
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextWrapped("%s", row.method.c_str());
    if (!row.backend.empty()) {
      const char * backend = row.backend == "CUDAExecutionProvider" ? "CUDA" :
        row.backend == "CPUExecutionProvider" ? "CPU" : row.backend.c_str();
      ImGui::TextDisabled("%s", backend);
    }
    ImGui::TableSetColumnIndex(1);
    if (!row.metric.empty()) ImGui::TextWrapped("%s", row.metric.c_str());
    ScoreCell(row.value, prior ? prior->value : std::nullopt);
    TimingCell(2, row.median, show_timing ? prior->median : std::nullopt);
    TimingCell(3, row.tail, show_timing ? prior->tail : std::nullopt);
    ImGui::TableSetColumnIndex(4);
    ImGui::TextWrapped("%s", row.details.c_str());
  }
  ImGui::EndTable();
}

void BenchmarkWorkspace::Impl::DrawDetails(float scale, const BreakdownStage & details, const BenchmarkReport * compared)
{
  const auto & labels = details.at(scope);
  if (!labels.count(label)) label = labels.begin()->first;
  if (scope != "Overall" || labels.size() > 1) {
    ImGui::SetNextItemWidth(200 * scale);
    if (ImGui::BeginCombo(scope.c_str(), label.c_str())) {
      for (const auto & item : labels)
        if (ImGui::Selectable(item.first.c_str(), item.first == label)) label = item.first;
      ImGui::EndCombo();
    }
  }
  const auto & rows = labels.at(label);
  const std::vector<BreakdownRow> * baseline_rows = nullptr;
  if (compared) {
    const auto prior_stage = compared->breakdowns.find(stage);
    if (prior_stage != compared->breakdowns.end()) {
      const auto prior_scope = prior_stage->second.find(scope);
      if (prior_scope != prior_stage->second.end()) {
        const auto prior_label = prior_scope->second.find(label);
        if (prior_label != prior_scope->second.end()) baseline_rows = &prior_label->second;
      }
    }
  }
  const auto & metrics = rows.front().metrics;
  const bool show_counts = std::any_of(rows.begin(), rows.end(), [](const auto & row) {return !row.counts.empty();});
  const float line_height = ImGui::GetTextLineHeightWithSpacing();
  float height = line_height * 2;
  for (const auto & row : rows) {
    const auto count_lines = static_cast<float>(std::count(row.counts.begin(), row.counts.end(), '\n') + 1);
    height += line_height * (std::max(count_lines, baseline_rows ? 2.f : 1.f) + .5f);
  }
  height = std::min(320 * scale, height);
  if (!ImGui::BeginTable("##Details", static_cast<int>(metrics.size()) + 2 + show_counts,
      ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit |
      ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH, {0, height})) return;
  ImGui::TableSetupScrollFreeze(1, 1);
  ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthFixed, 122 * scale);
  const char * unit = stage == "Ground" ? "Points" : "Boxes";
  ImGui::TableSetupColumn(unit, ImGuiTableColumnFlags_WidthFixed, 58 * scale);
  for (const auto & metric : metrics) {
    const auto * header = DetailHeader(metric.first);
    ImGui::TableSetupColumn(header, ImGuiTableColumnFlags_WidthFixed,
      std::max(65 * scale, ImGui::CalcTextSize(header).x));
  }
  if (show_counts) ImGui::TableSetupColumn("Errors", ImGuiTableColumnFlags_WidthFixed, 163 * scale);
  ImGui::TableHeadersRow();
  for (const auto & row : rows) {
    const BreakdownRow * prior = nullptr;
    if (baseline_rows) {
      const auto match = std::find_if(baseline_rows->begin(), baseline_rows->end(), [&](const auto & item) {
        return item.method == row.method;
      });
      if (match != baseline_rows->end()) prior = &*match;
    }
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextWrapped("%s", row.method.c_str());
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted(FormatCount(row.support).c_str());
    for (std::size_t i = 0; i < metrics.size(); ++i) {
      ImGui::TableSetColumnIndex(static_cast<int>(i) + 2);
      ScoreCell(FindDetailValue(&row, metrics[i].first), FindDetailValue(prior, metrics[i].first));
    }
    if (show_counts) {
      ImGui::TableSetColumnIndex(static_cast<int>(metrics.size()) + 2);
      ImGui::TextWrapped("%s", row.counts.c_str());
    }
  }
  ImGui::EndTable();
}

void BenchmarkWorkspace::Impl::DrawFailures(float scale)
{
  const bool by_scene = scope == "Scene";
  const auto shown = [&](const FailureExample & example) {
    return example.stage == stage && (!by_scene || example.scene == label);
  };
  if (std::none_of(report->cases.begin(), report->cases.end(), shown) || !ImGui::CollapsingHeader("Failure examples")) return;
  if (!ImGui::BeginTable("##FailureCases", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) return;
  ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthStretch, 2.f);
  ImGui::TableSetupColumn("Scene", ImGuiTableColumnFlags_WidthStretch, 1.4f);
  ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthStretch, 3.f);
  ImGui::TableSetupColumn("Frame ID", ImGuiTableColumnFlags_WidthFixed, 90 * scale);
  ImGui::TableHeadersRow();
  int index = 0;
  for (const auto & example : report->cases) {
    if (!shown(example)) continue;
    ImGui::PushID(index++);
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextWrapped("%s", example.method.c_str());
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted(example.scene.c_str());
    ImGui::TableSetColumnIndex(2);
    ImGui::TextWrapped("%s", example.description.c_str());
    ImGui::TableSetColumnIndex(3);
    if (ImGui::SmallButton("Copy ID")) ImGui::SetClipboardText(example.frame.c_str());
    ImGui::PopID();
  }
  ImGui::EndTable();
}


BenchmarkWorkspace::BenchmarkWorkspace() : impl_(std::make_unique<Impl>()) {}
BenchmarkWorkspace::~BenchmarkWorkspace() = default;

void BenchmarkWorkspace::Draw(bool & open, float scale)
{
  auto & state = *impl_;
  state.Poll();
  if (!open) return;
  const ImVec2 display = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowSize({std::min(900.f * scale, display.x - 24.f * scale),
    std::min(720.f * scale, display.y - 24.f * scale)}, ImGuiCond_FirstUseEver);
  if (ImGui::Begin("Perception benchmark", &open, ImGuiWindowFlags_NoCollapse)) {
    if (!state.error.empty()) {
      ImGui::PushStyleColor(ImGuiCol_Text, ui::ErrorColor());
      ImGui::TextWrapped("%s", state.error.c_str());
      ImGui::PopStyleColor();
    }
    if (ImGui::BeginTabBar("Benchmark tabs")) {
      if (ImGui::BeginTabItem("Run")) {
        state.DrawRun(scale);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Results", nullptr, state.show_results ? ImGuiTabItemFlags_SetSelected : 0)) {
        state.show_results = false;
        state.DrawResults(scale);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
  }
  ImGui::End();
}
}  // namespace lidar_viewer
