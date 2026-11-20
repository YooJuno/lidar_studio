#!/usr/bin/env python3
"""Drive the standalone viewer with Mid-360 UDP over loopback under Xvfb."""

import argparse
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import tempfile
import time

from ui_controls import Controls
from ui_gl_smoke import X11, capture, changed_pixels


def mid360_packet(sequence):
    header = bytearray(36)
    struct.pack_into('<HHH', header, 1, 36 + 96 * 14, 10000, 96)
    struct.pack_into('<H', header, 7, sequence)
    header[10] = 1
    struct.pack_into('<Q', header, 28, 1_000_000_000 + sequence * 10_000_000)
    points = b''.join(struct.pack('<iiiBB', 3000 + sequence * 10 + x * 60,
                                 1000 + y * 90, 10 * x, 76, 0x21)
                      for x in range(12) for y in range(8))
    return header + points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='direct-live-', dir=args.output.resolve()))
    log_path = output / 'viewer.log'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as first, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as second:
        first.bind(('127.0.0.1', 0))
        second.bind(('127.0.0.1', 0))
        points_port, imu_port = first.getsockname()[1], second.getsockname()[1]
    env = dict(os.environ, XDG_CONFIG_HOME=str(output / 'config'), LIDAR_VIEWER_UI_TRACE='1')
    x11 = X11()
    process = None
    try:
        with log_path.open('wb') as log, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            process = subprocess.Popen([str(args.viewer.resolve()), '--udp-port', str(points_port),
                                        '--aux-udp-port', str(imu_port), '--lidar-ip', '127.0.0.1',
                                        '--record-folder', str(output / 'recordings'), '--2d'],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)

            def wait_for(predicate, description):
                deadline = time.monotonic() + 12
                while not predicate():
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError(f'{description}. See {log_path}')
                    time.sleep(.05)

            def wait_for_log(part):
                wait_for(lambda: part in log_path.read_text(errors='replace'), f'Missing trace {part!r}')

            def send(first, last):
                for sequence in range(first, last):
                    sender.sendto(mid360_packet(sequence), ('127.0.0.1', points_port))
                    time.sleep(.002)

            wait_for(x11.viewer_window, 'Viewer did not open')
            window = x11.viewer_window()
            x11.focus(window)
            x11.resize(window, 1600, 960)
            controls = Controls(x11, window, log_path)
            controls.click('open_live')
            controls.click('live_direct_model')
            controls.rect('live_model_mid360')
            x11.key(window, 'Escape')  # Leave the model selector on Auto.
            controls.click('live_connect')
            wait_for_log('Direct LiDAR listening: Auto')
            send(0, 40)
            wait_for_log('Direct LiDAR detected: Livox Mid-360')
            wait_for_log('Live first frame:')
            x11.key(window, 'space')
            wait_for_log('Live display paused')
            time.sleep(.3)
            before = capture(x11, window, output / '01-paused.png')
            controls.click('live_record')
            wait_for_log('Live recording:')
            imu = bytearray(60)
            struct.pack_into('<HHH', imu, 1, len(imu), 0, 1)
            struct.pack_into('<Q', imu, 28, 1_423_000_000)
            struct.pack_into('<6f', imu, 36, 0.125, -0.25, 0.5, 0.75, -0.5, 9.81)
            sender.sendto(imu, ('127.0.0.1', imu_port))
            send(40, 120)
            during = capture(x11, window, output / '02-recording.png')
            if changed_pixels(before, during, (395, 200, 1510, 740)):
                raise AssertionError('Recording changed the frozen point cloud')
            controls.click('live_record')
            wait_for_log('Saved LiDAR capture:')
            path, = (output / 'recordings').glob('*/lidar.pcap')
            raw = path.read_bytes()
            if imu not in raw or mid360_packet(76) not in raw:
                raise AssertionError('PCAP lost original Mid-360 point data or IMU bytes')
            controls.click('open_live')
            controls.click('live_open_capture')
            wait_for_log('Opened point cloud:')
            opened = re.search(r'Opened point cloud:.*\| (\d+) frames', log_path.read_text(errors='replace'))
            if not opened or int(opened[1]) < 6:
                raise AssertionError('Recorded scans did not reopen in the viewer')
            capture(x11, window, output / '03-replayed.png')
            print(f'Mid-360 direct live, frozen display, raw points + IMU and {opened[1]} reopened frames. {output}')
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        x11.close()


if __name__ == '__main__':
    main()
