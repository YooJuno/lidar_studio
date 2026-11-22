# Feature preview recordings

Both previews use consecutive scans from **KITTI Raw** at **10 Hz** and a fixed camera in LiDAR Studio.

| Files | Source scans | Actual application settings |
| --- | --- | --- |
| `ground-segmentation.gif` / `.mp4` | 20–109, 90 frames, 9 s | FugSeg toggled off and on; objects and clustering off |
| `object-detection.gif` / `.mp4` | 82–153, 72 frames, 7.2 s | AWML ShortRange v2.0.0; Car and Bicycle; minimum score 0.60; IDs and motion trails on; FugSeg with Obstacles only; clustering off |

## Dataset and license

- **Dataset:** KITTI Raw, `2011_09_26_drive_0005_sync`, Velodyne HDL-64E. Scan indices are zero-based.
- **Authors:** Andreas Geiger, Philip Lenz, Christoph Stiller and Raquel Urtasun.
- **Reference:** *Vision meets Robotics: The KITTI Dataset*, International Journal of Robotics Research, 2013.
- **Official source:** [KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php), [copyright and citation](https://www.cvlibs.net/datasets/kitti/).
- **Rendered media license:** [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/).

The recordings show model output. The tracking demo uses the [official TIER IV AWML ShortRange base weights](https://github.com/tier4/AWML/blob/9e44a974aae03a196003950898dc909cd0c680ad/projects/CenterPoint/docs/CenterPoint-ShortRange/v2/base.md). Source scans and model weights are available separately. [Example datasets and setup](../examples.md).

## Capture and playback

The captures advance through the original sequence, waiting for each app result before capturing that scan. Each frame is visible for 100 ms. The fixed crop shows the native points, boxes, IDs and trails. The GIF palette is reduced for web playback. The recorded durations describe scan time.

### Ground segmentation

Captured from build `b291d9a`. The first 20 frames show the full cloud. Frames 20–44 use Obstacles only, frames 45–64 show the full cloud, and frames 65–89 use Obstacles only again. These frame numbers are relative to the clip.

The saved camera targets `(5, 0, 0.5)` m with yaw `-0.8` rad, pitch `1.15` rad and distance `65` m. The grid spacing is 5 m. Point size is 2 px. The GIF is 960 × 450 and the MP4 is 1120 × 524.

### Object detection and tracking

Captured from source revision [`23f4a09`](https://github.com/YooJuno/lidar_studio/tree/23f4a09599c2ce8200fffbe89ba9825bb587c08b). The tracker processes scans 60–81 to build its history before the clip starts. The visible bicycle retains ID **#4** across all 72 captured scans. Cyan and yellow trails follow the tracked positions in the LiDAR frame. `weak` identifies a lower-score observation that supports an existing ID, and `est.` identifies a predicted position through a brief miss.

In **Objects**, choose **AWML ShortRange**, enable **Car**, **Bicycle**, **IDs**, and **Tracking options → Motion trails**, and set **Min score** to 0.60. In **Ground**, use **FugSeg → Obstacles only**. The detector receives the full aligned scan, including points hidden in the view.

The 3D camera targets `(12.5, 0, 0.5)` m with yaw `-1.45` rad, pitch `1.13` rad and distance `29` m. The capture uses Uniform point color, 1.5 px size and no reference grid. The GIF and MP4 are both 1104 × 624.
