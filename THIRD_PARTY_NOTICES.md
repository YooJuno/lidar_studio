# Third-party code, models and example data

The root MIT license covers original LiDAR Studio code by Yoo Junho, except
files with their own license notices. It does not replace the terms of
third-party code, model weights, datasets or dataset-derived media.

## Integrated algorithms and detection code

| Component | License | Notices / source |
| --- | --- | --- |
| FugSeg | MIT | `third_party/fugseg/LICENSE` |
| GSeg3D | BSD-3-Clause | `third_party/ground_segmentation/LICENSE.txt` |
| Patchwork++ | BSD-2-Clause | `third_party/patchwork_plusplus/LICENSE` |
| Autoware-derived CenterPoint adapters | Apache-2.0 | Existing SPDX headers and `third_party/licenses/Autoware-CenterPoint-*` |
| AWML ShortRange models and adapter notices | Apache-2.0 | `third_party/licenses/AWML-CenterPoint-*` |
| Autoware TransFusion LiDAR | Apache-2.0 | [Official model and preprocessing reference](https://huggingface.co/AutowareFoundation/lidar_transfusion) |
| TerraSeg Small | Apache-2.0 (source), CC BY-NC-SA 4.0 (weights) | [TU Delft source](https://github.com/tudelft-iv/TerraSeg), [official model and training-data terms](https://huggingface.co/TedLentsch/TerraSeg) |

Files marked `SPDX-License-Identifier: Apache-2.0`, including the detection
preprocessing/postprocessing adapters and native helper, retain that license.
The third-party algorithm adapters retain their upstream attribution.

## Core build and runtime dependencies

PCL (BSD-3-Clause), Dear ImGui (MIT), GLFW (Zlib), GLEW (its BSD/MIT/SGI
notices), nanoflann (BSD-2-Clause), Eigen (MPL-2.0 and accompanying notices),
Boost (BSL-1.0), curl (curl license) and OpenSSL (Apache-2.0) retain their
upstream licenses. Bundled runtimes and Python packages retain their own
notices, including Python, NumPy, ONNX and ONNX Runtime.

Distributed packages include license texts in `third-party-licenses` (macOS)
or the corresponding resource directory. Runtime package notices remain in
the runtime's `*.dist-info/licenses` directories. The app's About window
provides a read-only license browser.

## Original pretrained models

tiny and AWML ShortRange weights retain their original Apache-2.0 model-card
and legal notices. Weights are not committed to the source repository. Mac
release bundles include them; the app can also download verified originals
into user storage. Model cards and training-data terms still apply. The
project's MIT license grants no additional rights over third-party data.

Autoware TransFusion LiDAR and TerraSeg Small weights are installed separately
from their pinned official releases. TerraSeg requires explicit acceptance of
its noncommercial model terms during setup.

## Examples and media

- KITTI-derived screenshots and demo recordings: CC BY-NC-SA 3.0, as described
  in `docs/images/README.md` and `docs/media/SOURCE.md`.
- UrbanV2X driving examples: CC BY-NC-SA 4.0, credited in `docs/examples.md`.
  The app downloads these separately; they are not licensed under MIT.
- Patchwork++ sample scans: retain the source repository's applicable terms
  and attribution described in `docs/examples.md`.

In particular, commercial permission for the original application code does
not remove noncommercial restrictions from example data or derived media.
