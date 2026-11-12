# Demo recording

`demo.mp4` and `demo.gif` are screen recordings of LiDAR Studio, captured in a private Xvfb session. They show the real public example download, playback and orbiting, ground/obstacle views, direct ROI editing and export settings. All views and edits use the downloaded point clouds. The MP4 runs at the recorded speed; the GIF preview is reduced in size and played at 1.5× speed.

The recording uses the six-frame quick preview. The current **Driving examples…** chooser also offers 300- and 600-frame UrbanV2X recordings; see [the example guide](../examples.md).

## Example data source

- **Distribution:** [Patchwork++ example scans](https://github.com/url-kaist/patchwork-plusplus/tree/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/data), Urban Robotics Lab, KAIST.
- **Pinned revision:** `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`.
- **Files:** `000000.bin` through `000005.bin`, binary float32 XYZI scans.
- **Upstream copyright and license:** [BSD 2-Clause](https://github.com/url-kaist/patchwork-plusplus/blob/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/LICENSE), reproduced below.

These public demonstration files are separate from the KITTI Raw drive 0005 sequence used for the static screenshots. Only the video and preview are included here; the app downloads the source scans on request and verifies their size and SHA-256 checksums.

The rendered demo recordings are shared under [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/). This applies to the recordings, not to LiDAR Studio's software or the upstream source license.

## Upstream notice

BSD 2-Clause License

Copyright (c) 2024, Urban Robotics Lab. @ KAIST

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
