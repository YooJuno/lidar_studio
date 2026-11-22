# Code structure

## External algorithms

`third_party/` holds the pinned FugSeg, DFKI GSeg3D and KAIST Patchwork checkouts and dependency notices. The parent repository records each source commit. `git submodule update --init --recursive` restores those versions.

`cmake/third_party/` selects the vendor cores, pins Dear ImGui and generates the FugSeg and GSeg3D integration headers in the build tree. Studio's adapters convert inputs, map parameters and map results back to source points. [Source and license notices](../THIRD_PARTY_NOTICES.md)

Learned inference weights are managed by their installers. Object detection has a versioned model manifest in `python/lidar_detection/models.py`. The TerraSeg installer pins the official source and the worker verifies its checkpoint. Inference wrappers and worker protocols live in Studio source.

## Studio code

| Area | Responsibility |
| --- | --- |
| `src/main.cpp` | Application entry point. |
| `src/viewer/` | Window, playback, camera and frame processing. `panels/` holds workspace controls. `render/` holds OpenGL programs and the panorama cache and palette. |
| `src/io/` | Live input routing, ROS bag readers in `bag/`, direct sensors and PCAP in `ethernet/`, point cloud files in `pointcloud/`. |
| `src/alignment/` and `src/roi/` | IMU alignment, box operations and point exports. |
| `src/algorithms/ground/builtin/` | Studio's Fast grid, Grid and RANSAC implementations. |
| `src/algorithms/ground/upstream/` | Studio adapters for FugSeg, GSeg3D and Patchwork. |
| `src/algorithms/` | Ground dispatch and clustering, with optional GPU backends in `cuda/` and Kalman tracking in `tracking/`. |
| `src/inference/` and `python/lidar_detection/` | Model subprocess transport, object and TerraSeg lifecycles, detector preprocessing and decoding. |
| `src/datasets/` and `src/training/` | Dataset review, bundled example manifests and training jobs. |
| `src/benchmark/` and `tools/benchmark_perception.py` | Ordered offline replay, evaluation and report generation. The native runner calls the same ground, detection and tracking code as Studio. |
| `src/settings/` and `src/platform/` | Configuration, settings and system file dialogs. |

Shared C++ interfaces are under `include/lidar_viewer/`. Private headers stay next to their implementation.

When updating a vendor version, review the relevant integration header and adapter against the new commit. Run the algorithm tests and compare predictions on fixed scans before accepting the update. The benchmark guide covers repeatable runs and result interpretation. [Benchmark guide](benchmark.md)
