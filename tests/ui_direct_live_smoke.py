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
    env = dict(os.environ, XDG_CONFIG_HOME=str(output / 'config'), LIDAR_VIEWER_UI_TRACE='1',
               LIDAR_VIEWER_CATALOG_TRACE='1')
    x11 = X11()
    process = None
    try:
        with (log_path.open('wb') as log, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
              socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other_source):
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

            def read_log():
                return log_path.read_text(errors='replace')

            def wait_for_log(part, after=0):
                wait_for(lambda: part in read_log()[after:], f'Missing trace {part!r}')

            def send(first, last, source=sender):
                for sequence in range(first, last):
                    source.sendto(mid360_packet(sequence), ('127.0.0.1', points_port))
                    time.sleep(.002)

            wait_for(x11.viewer_window, 'Viewer did not open')
            window = x11.viewer_window()
            x11.focus(window)
            x11.resize(window, 1600, 960)
            controls = Controls(x11, window, log_path)
            controls.click('open_live')
            controls.rect('live_direct_model')
            capture(x11, window, output / '00-setup.png')
            controls.click('live_direct_model')
            controls.rect('live_model_mid360')
            x11.key(window, 'Escape')  # Leave the model selector on Auto.
            controls.click('live_connect')
            wait_for_log('Direct LiDAR listening: Auto')
            controls.rect('live_waiting_settings')
            capture(x11, window, output / '00-waiting.png')
            send(0, 40)
            wait_for_log('Direct LiDAR detected: Livox Mid-360')
            wait_for_log('Live first frame:')
            x11.key(window, 'space')
            wait_for_log('Live display paused')
            controls.click('live_freeze')
            wait_for_log('Live display resumed')
            after_resume = len(read_log())
            controls.click('live_freeze')
            wait_for_log('Live display paused', after=after_resume)
            _, _, frame_log = read_log().partition('Live first frame:')
            if re.search(r'^UI live_waiting_settings ', frame_log, re.M):
                raise AssertionError('Live settings action remained over the displayed point cloud')
            time.sleep(.3)
            before = capture(x11, window, output / '01-paused.png')
            imu = bytearray(60)
            struct.pack_into('<HHH', imu, 1, len(imu), 0, 1)
            struct.pack_into('<Q', imu, 28, 1_423_000_000)
            struct.pack_into('<6f', imu, 36, 0.125, -0.25, 0.5, 0.75, -0.5, 9.81)

            # Auxiliary packets alone still produce a raw capture, with no replay action.
            auxiliary_log = len(read_log())
            controls.click('live_record')
            wait_for_log('Live recording:', after=auxiliary_log)
            sender.sendto(imu, ('127.0.0.1', imu_port))
            time.sleep(.25)
            controls.click('live_record')
            wait_for_log('Saved LiDAR capture:', after=auxiliary_log)
            controls.rect('live_capture_no_points', after=auxiliary_log)
            if re.search(r'^UI live_open_recording ', read_log()[auxiliary_log:], re.M):
                raise AssertionError('An auxiliary-only capture offered a point cloud replay')
            raw_only_path, = (output / 'recordings').glob('*/lidar.pcap')
            if imu not in raw_only_path.read_bytes():
                raise AssertionError('Auxiliary-only capture lost the original IMU bytes')

            mixed_log = len(read_log())
            controls.click('live_record')
            wait_for_log('Live recording:', after=mixed_log)
            # Valid packets from a different port arrive first. Keep their bytes, but replay the live stream.
            send(300, 330, other_source)
            sender.sendto(imu, ('127.0.0.1', imu_port))
            send(40, 120)
            during = capture(x11, window, output / '02-recording.png')
            if changed_pixels(before, during, (395, 200, 1510, 740)):
                raise AssertionError('Recording changed the frozen point cloud')
            controls.click('live_record')
            wait_for_log('Saved LiDAR capture:', after=mixed_log)
            path, = set((output / 'recordings').glob('*/lidar.pcap')) - {raw_only_path}
            raw = path.read_bytes()
            if any(packet not in raw for packet in (imu, mid360_packet(76), mid360_packet(300))):
                raise AssertionError('PCAP lost original point or IMU bytes')
            width, height = 1024, 720
            x11.resize(window, width, height)
            compact_log = len(read_log())
            for action in ('live_freeze', 'live_record', 'live_open_recording'):
                left, top, right, bottom = controls.rect(action, after=compact_log)
                if not (0 <= left < right <= width and 0 <= top < bottom <= height):
                    raise AssertionError(f'{action} is clipped at {width}x{height}')
            capture(x11, window, output / '03-saved-compact.png')
            controls.click('live_open_recording')
            wait_for_log('Opened point cloud:')
            opened = re.search(r'Opened point cloud:.*\| (\d+) frames', read_log())
            if not opened or int(opened[1]) < 6:
                raise AssertionError('The saved capture did not replay the selected live stream')
            capture(x11, window, output / '04-replayed.png')

            reconnect_log = len(read_log())
            controls.click('open_live')
            controls.rect('live_connect', after=reconnect_log)
            controls.click('live_connect')
            wait_for_log('Direct LiDAR listening: Auto', after=reconnect_log)
            controls.rect('live_waiting_settings', after=reconnect_log)
            disconnect_log = len(read_log())
            controls.click('live_waiting_settings')
            controls.rect('live_connect', after=disconnect_log)
            controls.click('live_connect')
            wait_for_log('Live disconnected', after=disconnect_log)
            controls.rect('source_selector', after=disconnect_log)
            controls.click('source_selector')
            row = re.compile(r'^Source row: "' + re.escape(str(path.parent.resolve())) +
                             r'" \| select ([\d.-]+) ([\d.-]+) .* \| available 1$', re.M)

            def catalog_row():
                matches = row.findall(read_log()[disconnect_log:])
                if matches:
                    x, y = map(float, matches[-1])
                    if 0 <= x < width and 0 <= y < height:
                        return round(x), round(y)

            wait_for(catalog_row, 'Saved PCAP missing from Browse recordings')
            capture(x11, window, output / '05-recording-list.png')
            x11.click(window, *catalog_row())
            wait_for_log('Opened point cloud:', after=disconnect_log)
            print(f'Mid-360 live setup, frozen display, compact transport, auxiliary-only save and mixed-port capture '
                  f'and {opened[1]} reopened frames from the live shortcut and recording list. {output}')
    finally:
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        x11.close()


if __name__ == '__main__':
    main()
