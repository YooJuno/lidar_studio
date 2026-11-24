# Perception benchmark

Replay the same nuScenes LiDAR scans through LiDAR Studio's ground, object detection and tracking backends. The benchmark processes every selected scan in timestamp order. A Studio report shows the comparisons and failure examples, and TSV files preserve the per-frame measurements and predictions.

## Data

Extract both archives into the same directory. Mini has 10 scenes and 404 labelled keyframes. The default `mini_val` split selects `scene-0103` and `scene-0916`, with 81 labelled keyframes. [nuScenes Mini archive](https://www.nuscenes.org/data/v1.0-mini.tgz) · [Mini LiDARSeg labels](https://www.nuscenes.org/data/nuScenes-lidarseg-mini-v1.0.tar.bz2) · [Dataset documentation](https://www.nuscenes.org/tutorials/nuscenes_lidarseg_panoptic_tutorial.html)

The evaluator reads `samples/LIDAR_TOP`, `sweeps/LIDAR_TOP`, `v1.0-mini` and `lidarseg/v1.0-mini`. Object-only evaluation uses the Mini archive. For `v1.0-trainval`, select `--version v1.0-trainval --split val` and install `nuscenes-devkit` and `ijson`. Observe the dataset's terms when using or sharing derived results. [nuScenes terms](https://www.nuscenes.org/terms-of-use)

## Run a comparison

In Studio, open **Viewer → Open → Perception benchmark… → Run**, choose the extracted nuScenes folder, select methods, and run the comparison. Results open automatically and are saved under `~/LiDAR Studio Benchmarks`. **Run settings** lets you choose a Python executable with NumPy and SciPy, limit the number of keyframes, and set the detection score. The Studio app can install AWML ShortRange and CenterPoint tiny. [Detector setup](guide.md#사람차량-인식--원본-tier-iv-모델)

For a scripted comparison, build the source checkout and use Python 3.10 or later. `--detector` uses the Studio detector installation and its execution provider.

```bash
cmake --build build --target lidar_studio_benchmark --parallel
python -m venv .venv-benchmark
.venv-benchmark/bin/python -m pip install numpy scipy
.venv-benchmark/bin/python tools/benchmark_perception.py \
  --runner build/lidar_studio_benchmark \
  --dataset /path/to/nuscenes \
  --output build/benchmark-mini-val \
  --ground fast_grid --ground fugseg --ground gseg3d --ground patchworkpp --ground linefit \
  --detector short_range --detector tiny \
  --track studio:3:0.8 --track strict:3:0.3
```

On Windows, use `.venv-benchmark\Scripts\python.exe` and the benchmark executable in the build configuration directory, such as `build\Release\lidar_studio_benchmark.exe`. Choose a new output directory for each run. `--scene scene-0103 --max-keyframes 5` runs a quick check. `--keyframes-only` runs inference at the 2 Hz label rate and changes the tracker input cadence. `--no-ground` runs only the selected detectors and tracking profiles. Run `--help` for options and supported algorithms.

To reopen a run, choose **Perception benchmark… → Results → Open report…** and select `benchmark.json`. Results include overall, scene and class breakdowns where applicable. Compare both missed detections and false positives at the selected operating score. Ground results include the fraction of pedestrian, vehicle and other non-ground points removed. `summary.tsv` is the compact table. Each method directory contains frame timings and errors. Object directories also contain detector boxes, tracks and track ID events. Ground directories contain byte masks in original point order for the labelled keyframes.

## Compare a baseline and candidate

Run the two builds on the same ordered split, with the same thresholds and tracker profiles. Use `--hash-clouds` to include point payloads in the selection fingerprint. The report always fingerprints the selected scans, labels, timestamps and calibration. It also records the scoring protocol, target executable hash, model weight identifiers and timing host. Select a baseline report in Studio to see matched score and latency deltas.

For automated checks, define limits suited to the deployment. For example, this command allows a 0.5 percentage point drop in the tracker score and a 10% latency increase, and sets a minimum person recall.

```bash
python3 tools/compare_perception_reports.py baseline/benchmark.json candidate/benchmark.json \
  --max-score-drop Tracking 'short_range / studio' 'Local IDF1' 0.5 \
  --max-p95-rise Detection short_range 'Local center AP' 10 \
  --min-breakdown Detection short_range Class pedestrian Recall 30 --json
```

The command exits with 0 for a passing comparison, 1 for an exceeded limit and 2 for incompatible reports or an unavailable requested metric. Timing deltas require the same host fingerprint and execution backend. Use comparable machine load and repeat runs near a performance limit. For deployment validation, choose thresholds on development scenes and check them on a separate labelled split representative of the installed sensors and operating conditions.

## Interpret results

| Stage | Protocol |
| --- | --- |
| Ground | Compare original point indices. `driveable_surface`, `sidewalk` and `terrain` are ground. Noise, ego-vehicle returns, `flat.other` and ambiguous duplicate predictions are ignored. Report IoU, precision, recall and the loss of non-ground, pedestrian and vehicle points. |
| Detection | Local class-wise AP using 101-point interpolation at 0.5, 1, 2 and 4 m horizontal center-distance gates. The 0.10 candidate floor applies before scoring. Operating-point counts use the selected threshold and a 2 m gate. Counts for pedestrian-free labelled keyframes are reported separately. |
| Tracking | Give each profile the same detector observations, including intermediate sweeps. Local IDF1 assigns whole tracks one-to-one per scene and class using a 2 m horizontal gate. Recall, ID changes, interruptions and unmatched predicted boxes are scored on labelled keyframes. The Studio tracker runs in sensor-relative coordinates. [Identity metrics paper](https://arxiv.org/abs/1609.01775) |
| Timing | Report p50, p95 and p99 per stage with the backend. The report records the first scan separately and omits the first three scans from percentiles when enough samples exist. All scans still contribute to accuracy and tracker state. Ground timing excludes mask export. Detector timing includes packing, inference and worker communication, excluding setup and queueing. |

Local detection AP and tracking metrics are labeled separately from official nuScenes leaderboard scores. The default Mini run is useful for integration checks and controlled comparisons on two scenes. For a broader evaluation, keep the class definitions and split fixed, record the weights and runtime, and add labelled clips from the sensors and environments in use. Use a sensor-rate replay with capture and rendering enabled to measure frame drops and scan-to-result latency. [Official detection protocol](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/eval/detection/README.md) · [Official tracking protocol](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/eval/tracking/README.md)

On all 404 label frames in nuScenes Mini, LineFit classified 1.37% of non-ground points as ground, compared with 5.49% for FugSeg. Ground IoU was 80.31% for LineFit and 93.30% for FugSeg. Try LineFit when keeping low obstacle points is important and the scene can tolerate more residual ground. Its range and vertical tolerance are adjustable in **Ground → Algorithm settings**. [LineFit implementation and paper](https://github.com/lorenwel/linefit_ground_segmentation)
