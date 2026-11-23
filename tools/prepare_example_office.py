#!/usr/bin/env python3
"""Prepare the portable LiDAR-only Office1 example from the pinned source bag."""

from argparse import ArgumentParser
from hashlib import file_digest, sha256
from itertools import islice
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

from numpy import fromiter
from rosbags.highlevel import AnyReader


SOURCE_SHA256 = "570521eb09437403c5c12725694ffa675d7cd17e575a88de1261800249397e96"
START_MESSAGE = 1000
MESSAGES_PER_FRAME = 5
FRAMES = 300
CREDIT = """Elevator-LIO Office1 point cloud excerpt
Original data by Yifan Zhang, Yudong Huang, Yuchong Zhang, Changze Li,
Haoran Liu, Ming Yang and Tong Qin.
Source: https://huggingface.co/datasets/xiaofan0100/Elevator-LIO-Dataset
Revision: 9a0f13bd9f917e405cb4cb76c924f02774dc33d9 (Office1.bag)
Paper: https://arxiv.org/abs/2605.24495
License: Creative Commons Attribution 4.0 International
https://creativecommons.org/licenses/by/4.0/

LiDAR Studio selected /livox/lidar messages 1000 through 2499 (zero-based)
and combined each group of five consecutive messages into one playback frame
at approximately 10 Hz. The points keep their recorded XYZ and reflectivity
as little-endian float32 values. The filenames retain the header timestamp
of the first message in each frame. Per-point timing, IMU, poses and other
fields are omitted. No motion compensation, resampling or labels were applied.
"""


def add(archive, manifest, name, contents):
    entry = ZipInfo(name)
    entry.compress_type = ZIP_DEFLATED
    archive.writestr(entry, contents, compresslevel=3)
    manifest.write(f'  {{"{name}",{len(contents)},"{sha256(contents).hexdigest()}"}},\n')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("archive", type=Path)
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    with args.source.open("rb") as source:
        if file_digest(source, "sha256").hexdigest() != SOURCE_SHA256:
            parser.error("The input does not match the published Office1 bag")

    with AnyReader([args.source]) as reader, ZipFile(args.archive, "x") as archive, args.manifest.open("x") as manifest:
        connections = [item for item in reader.connections if item.topic == "/livox/lidar"]
        if len(connections) != 1:
            raise ValueError("Expected a single /livox/lidar topic")
        messages = islice(reader.messages(connections=connections), START_MESSAGE, None)
        for _ in range(FRAMES):
            frame = bytearray()
            start = None
            for _ in range(MESSAGES_PER_FRAME):
                connection, _, raw = next(messages)
                scan = reader.deserialize(raw, connection.msgtype)
                if start is None:
                    stamp = scan.header.stamp
                    start = stamp.sec * 1_000_000_000 + stamp.nanosec
                points = fromiter(
                    (value for p in scan.points for value in (p.x, p.y, p.z, p.reflectivity)),
                    dtype="<f4",
                    count=4 * scan.point_num,
                )
                frame.extend(points.tobytes())
            add(archive, manifest, f"{start}.bin", frame)
        add(archive, manifest, "ATTRIBUTION.txt", CREDIT.encode("utf-8"))
    with args.archive.open("rb") as archive:
        print(args.archive.name, args.archive.stat().st_size, file_digest(archive, "sha256").hexdigest())


if __name__ == "__main__":
    main()
