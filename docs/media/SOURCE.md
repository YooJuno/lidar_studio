# Feature preview recordings

The README has two focused previews, each captured from the **published LiDAR Studio app**. They use the same 90 consecutive scans, a fixed camera and the original 10 Hz sensor timeline. No source frames are repeated within a preview. Each GIF loops only after the complete nine-second sequence.

| Files | Actual application settings | What to look for |
| --- | --- | --- |
| `ground-segmentation.gif` / `.mp4` | FugSeg; actual classifier toggled OFF / ON; objects and clustering off | OFF shows the complete cloud. ON selects Obstacles only, removing the classified road while retaining obstacles. The camera remains fixed. |
| `object-detection.gif` / `.mp4` | Original Autoware CenterPoint tiny v4.1 weights; Car target; minimum score 0.60; tracking IDs on; FugSeg ground removal with Obstacles only; clustering off | Cyan vehicle boxes and persistent IDs over the non-ground points. A `predicted` label indicates a tracker prediction during a missed measurement. |

## Dataset and license

- **Dataset:** KITTI Raw, `2011_09_26_drive_0005_sync`, Velodyne HDL-64E.
- **Scans shown:** `0000000020.bin` through `0000000109.bin`, inclusive; zero-based indices 20–109.
- **Authors:** Andreas Geiger, Philip Lenz, Christoph Stiller and Raquel Urtasun.
- **Reference:** *Vision meets Robotics: The KITTI Dataset*, International Journal of Robotics Research, 2013.
- **Official source:** [KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php), [copyright and citation](https://www.cvlibs.net/datasets/kitti/).
- **License:** these dataset-derived GIF and MP4 files are provided under [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/). This does not change LiDAR Studio's software license.

The previews contain actual algorithm outputs, not ground-truth annotations or an accuracy benchmark. Detection uses the installed pretrained model; geometric clusters are not presented as learned detections. The model source revision is `494c8171def40bd36cc2feb323e0a5acbfab132b`, from [AutowareFoundation/lidar_centerpoint](https://huggingface.co/AutowareFoundation/lidar_centerpoint/tree/494c8171def40bd36cc2feb323e0a5acbfab132b). Only rendered media is included in the repository; raw scans and model weights are installed separately.

These KITTI previews are distinct from the in-app UrbanV2X downloads and the six-frame Patchwork++ quick preview. [Example datasets and setup](../examples.md).

## Capture and playback

Frames were captured in an isolated Xvfb session using the app built from commit `b291d9a`. Each frame is stepped in order and captured after its ground or learned-detection result is ready. The nine-second animation therefore follows **sensor time**, rather than the wall-clock time spent capturing PNGs. GPU initialization, screenshot overhead and display refresh are not processing-speed measurements.

The capture leaves points, classification colors, model boxes and labels unchanged. A fixed crop removes the repeated application header, playback bar and status bar so the relevant sidebar and point cloud remain readable. There are no camera cuts, orbiting, fades, synthetic boxes or acceleration effects. Both formats use 90 frames at 10 fps. GIF previews are 960 × 450 pixels; the higher-resolution MP4 files are 1120 × 524 pixels. GIF encoding reduces the palette; the MP4 preserves more detail.

To reproduce the views, open the KITTI sequence described in the [example guide](../examples.md), start at scan 20 and step through scan 109. Set **Uniform** point coloring and a 2 px point size. Ground uses a 5 m reference grid; the object preview hides the grid and shows only the Car target. The capture uses these saved camera values for both previews:

```ini
projection_mode=3d
camera_target_x=5
camera_target_y=0
camera_target_z=0.5
camera_yaw_rad=-0.8
camera_pitch_rad=1.15
camera_distance=65
```

For ground, start with **Classify ground** off. Turn it on at captured frame 20 and choose **Obstacles only**. Turn it off at frame 45 (the app restores **All points**), then turn it on and select **Obstacles only** again at frame 65. This produces OFF for 2 s, ON for 2.5 s, OFF for 2 s and ON for 2.5 s. For objects, leave **FugSeg** enabled and select **Obstacles only** in Ground. Install the optional models, choose **CenterPoint tiny**, set **Min score = 0.60**, select **Car**, enable **IDs** and hide motion trails and speed labels. Ground filtering changes the displayed points; the learned model still receives its normal full, aligned scan before ground filtering. Sensor-height alignment is the app's usual first-frame estimate; manual angles and translation are zero and IMU leveling is off.
