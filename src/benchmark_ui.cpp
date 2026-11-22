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
#include <optional>
#include <stdexcept>
#include <string>
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

struct BenchmarkReport
{
  fs::path path;
  std::string version, split;
  std::size_t scans, keyframes, scenes, point_labelled;
  bool keyframes_only;
  std::vector<BenchmarkRow> rows;
  std::vector<FailureExample> cases;
};

std::optional<double> ReadMetric(const ptree & row, const char * key)
{
  const auto value = row.get<std::string>(key);
  if (value == "null") return std::nullopt;
  const auto number = row.get<double>(key);
  if (!std::isfinite(number)) throw std::runtime_error("A report metric is not finite.");
  return number;
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
    data.get<std::size_t>("point_labelled_keyframes"), data.get<bool>("keyframes_only"), {}, {}};
  for (const auto & entry : root.get_child("rows")) {
    const auto & row = entry.second;
    report.rows.push_back({row.get<std::string>("stage"), row.get<std::string>("method"),
      row.get<std::string>("metric"), row.get<std::string>("details"), row.get<std::string>("backend", ""), ReadMetric(row, "value"),
      ReadMetric(row, "p50_ms"), ReadMetric(row, "p95_ms")});
  }
  if (const auto examples = root.get_child_optional("examples")) {
    for (const auto & entry : *examples) {
      const auto & row = entry.second;
      report.cases.push_back({row.get<std::string>("stage"), row.get<std::string>("method"),
        row.get<std::string>("scene"), row.get<std::string>("frame"), row.get<std::string>("description")});
    }
  }
  return report;
}

void TimingCell(int column, const std::optional<double> & value)
{
  ImGui::TableSetColumnIndex(column);
  if (value) ImGui::Text("%.2f ms", *value);
  else ImGui::TextDisabled("--");
}
}  // namespace

struct BenchmarkWorkspace::Impl
{
  struct Method { const char * id; const char * label; bool enabled; };
  FileDialog picker;
  bool choose_dataset = false;
  std::future<BenchmarkReport> loading;
  std::optional<BenchmarkReport> report;
  std::string error, log;
  TrainingJob job;
  fs::path run;
  bool awaiting_result = false, cancelled = false, show_results = false;
  double last_poll = -1;
  int limit = 0;
  float score = .50f;
  std::array<char, 4096> dataset{};
#ifdef _WIN32
  std::array<char, 4096> python{{'p','y','t','h','o','n'}};
#else
  std::array<char, 4096> python{{'p','y','t','h','o','n','3'}};
#endif
  Method ground[8]{{"fast_grid", "Fast grid", true}, {"fugseg", "FugSeg", true},
    {"gseg3d", "GSeg3D", true}, {"patchworkpp", "Patchwork++", true},
    {"patchwork_classic", "Patchwork", false}, {"grid", "Grid", false},
    {"ransac", "RANSAC", false}, {"terraseg", "TerraSeg", false}};
  Method detectors[4]{{"short_range", "AWML ShortRange", false}, {"tiny", "CenterPoint tiny", false},
    {"base", "CenterPoint base", false}, {"transfusion", "TransFusion", false}};
  bool compare_timeout = true;

  void Start()
  {
    const auto folder = fs::u8path(dataset.data());
    if (!fs::is_directory(folder)) throw std::runtime_error("Choose an extracted nuScenes Mini folder.");
    const auto script = ProjectResourcePath("tools/benchmark_perception.py");
    if (!fs::is_regular_file(script)) throw std::runtime_error("The benchmark Python scripts are missing.");
    const auto parent = UserHome() / "LiDAR Studio Benchmarks";
    fs::create_directories(parent);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    run = parent / ("mini-val-" + std::to_string(millis) + "-" + std::to_string(ProcessId()));
    std::vector<std::string> arguments{python.data(), "-X", "utf8", "-u", script.u8string(), "--dataset", folder.u8string(),
      "--runner", ExecutablePath().u8string(), "--studio-runner", "--python-root", ProjectResourcePath("python").u8string(),
      "--output", run.u8string()};
    if (limit > 0) arguments.insert(arguments.end(), {"--max-keyframes", std::to_string(limit)});
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
          if (!cancelled && fs::is_regular_file(file))
            loading = std::async(std::launch::async, LoadReport, file);
          else if (!cancelled) error = "Benchmark ended before writing the report. See the run log.";
        }
      }
      if (const auto result = picker.Poll()) {
        if (!result->error.empty()) throw std::runtime_error(result->error);
        if (!result->path.empty()) {
          error.clear();
          if (choose_dataset) {
            const auto path = result->path.u8string();
            if (path.size() >= dataset.size()) throw std::runtime_error("Dataset path is too long.");
            dataset.fill(0);
            std::copy(path.begin(), path.end(), dataset.begin());
          } else loading = std::async(std::launch::async, LoadReport, result->path);
        }
      }
      if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        report = loading.get();
        show_results = true;
      }
    } catch (const std::exception & problem) {
      error = problem.what();
    }
  }
  void DrawRun(float scale);
  void DrawResults(float scale);
};

void BenchmarkWorkspace::Impl::DrawRun(float scale)
{
  const bool busy = job.Running() || loading.valid() || picker.Active();
  ImGui::BeginDisabled(busy);
  ImGui::TextUnformatted("nuScenes Mini dataset");
  ImGui::SetNextItemWidth(-100 * scale);
  ImGui::InputTextWithHint("##Dataset", "Folder with v1.0-mini and lidarseg", dataset.data(), dataset.size());
  ImGui::SameLine();
  if (ImGui::Button("Browse...")) {
    try { choose_dataset = true; picker.OpenDirectory(UserHome()); }
    catch (const std::exception & problem) { error = problem.what(); }
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
    ImGui::TextDisabled("Requires NumPy and SciPy in the selected Python.");
    ImGui::SetNextItemWidth(100 * scale);
    ImGui::InputInt("Keyframe limit (0 for all 81)", &limit, 0);
    limit = std::clamp(limit, 0, 81);
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
    try { choose_dataset = false; picker.OpenFile(report ? report->path.parent_path() : UserHome()); }
    catch (const std::exception & problem) { error = problem.what(); }
  }
  ImGui::EndDisabled();
  if (report) {
    ImGui::SameLine();
    if (ImGui::Button("Copy report folder"))
      ImGui::SetClipboardText(report->path.parent_path().u8string().c_str());
  }
  if (loading.valid()) ImGui::TextDisabled("Reading benchmark report...");
  if (const auto & result = report) {
    ImGui::Text("nuScenes %s / %s  |  %zu scenes  |  %zu scans  |  %zu scored keyframes",
      result->version.c_str(), result->split.c_str(), result->scenes, result->scans, result->keyframes);
    ImGui::TextDisabled("%zu with point labels  |  %s", result->point_labelled,
      result->keyframes_only ? "Keyframes only" : "Ordered replay with intermediate scans");
    ImGui::Spacing();
    for (const auto * stage : {"Ground", "Detection", "Tracking"}) {
      if (std::none_of(result->rows.begin(), result->rows.end(), [stage](const auto & row) {return row.stage == stage;})) continue;
      ImGui::PushID(stage);
      ImGui::SeparatorText(stage);
      if (ImGui::BeginTable("##Metrics", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg |
          ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthStretch, 2.f);
        ImGui::TableSetupColumn("Score", ImGuiTableColumnFlags_WidthStretch, 2.f);
        ImGui::TableSetupColumn("p50", ImGuiTableColumnFlags_WidthFixed, 64.f * scale);
        ImGui::TableSetupColumn("p95", ImGuiTableColumnFlags_WidthFixed, 64.f * scale);
        ImGui::TableSetupColumn("Errors", ImGuiTableColumnFlags_WidthStretch, 3.f);
        ImGui::TableHeadersRow();
        for (const auto & row : result->rows) {
          if (row.stage != stage) continue;
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextWrapped("%s", row.method.c_str());
          if (!row.backend.empty()) {
            const char * backend = row.backend == "CUDAExecutionProvider" ? "CUDA" :
              row.backend == "CPUExecutionProvider" ? "CPU" : row.backend.c_str();
            ImGui::TextDisabled("%s", backend);
          }
          ImGui::TableSetColumnIndex(1);
          if (row.value) ImGui::Text("%s %.1f%%", row.metric.c_str(), *row.value * 100.);
          else ImGui::TextDisabled("--");
          TimingCell(2, row.median);
          TimingCell(3, row.tail);
          ImGui::TableSetColumnIndex(4);
          ImGui::TextWrapped("%s", row.details.c_str());
        }
        ImGui::EndTable();
      }
      ImGui::PopID();
    }
    if (!result->cases.empty() && ImGui::CollapsingHeader("Failure examples")) {
      if (ImGui::BeginTable("##FailureCases", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Method", ImGuiTableColumnFlags_WidthStretch, 2.f);
        ImGui::TableSetupColumn("Scene", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthStretch, 3.f);
        ImGui::TableSetupColumn("Frame ID", ImGuiTableColumnFlags_WidthFixed, 90.f * scale);
        ImGui::TableHeadersRow();
        int index = 0;
        for (const auto & example : result->cases) {
          ImGui::PushID(index++);
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextWrapped("%s", example.method.c_str());
          ImGui::TextDisabled("%s", example.stage.c_str());
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
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Per-frame errors and timings are saved alongside the report.");
  } else {
    ImGui::TextWrapped("Open a benchmark.json from the perception benchmark to compare scores, processing time and errors.");
  }
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
