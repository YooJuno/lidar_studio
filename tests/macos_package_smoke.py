#!/usr/bin/env python3
"""Verify extracted ZIP/DMG apps on macOS; report Gatekeeper separately."""
import argparse
import json
import os
from pathlib import Path
import platform
import plistlib
import struct
import subprocess
import sys
import tempfile
import time

from macos_gl_probe import lacks_runner_opengl, probe_library


def run(*args, **kwargs):
    result = subprocess.run(args, text=True, capture_output=True, timeout=90, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{args[0]} failed ({result.returncode}):\n{result.stdout}\n{result.stderr}')
    return result


def check_app(app, workspace, require_gui=False):
    if app.name != 'LiDAR Studio.app':
        raise RuntimeError('Installed app must be named LiDAR Studio.app')
    info = plistlib.loads((app / 'Contents/Info.plist').read_bytes())
    if info.get('CFBundleDisplayName') != 'LiDAR Studio':
        raise RuntimeError('macOS display name must be LiDAR Studio')
    executable = app / 'Contents/MacOS' / info['CFBundleExecutable']
    if not os.access(executable, os.X_OK):
        raise RuntimeError('Packaged app executable is missing or not executable')
    run('codesign', '--verify', '--deep', '--strict', '--verbose=2', str(app))
    signature = run('codesign', '--display', '--verbose=4', str(app)).stderr
    assessment = subprocess.run(['spctl', '--assess', '--type', 'execute', '--verbose=4', str(app)],
                                text=True, capture_output=True, timeout=90)
    if 'Signature=adhoc' not in signature and assessment.returncode:
        raise RuntimeError('Developer ID app failed Gatekeeper: ' + assessment.stderr)
    if assessment.returncode:
        print('Gatekeeper: not accepted for downloaded distribution (ad-hoc, not notarized).', flush=True)
    else:
        print('Gatekeeper: accepted.', flush=True)
    minimum = info.get('LSMinimumSystemVersion')
    if not minimum:
        raise RuntimeError('Info.plist must declare LSMinimumSystemVersion')
    if tuple(map(int, minimum.split('.'))) > tuple(map(int, platform.mac_ver()[0].split('.'))):
        raise RuntimeError('App requires a newer macOS than this runner')
    # Reject accidental dependencies on the build runner's Homebrew installation.
    binaries = [executable, *app.rglob('*.dylib')]
    for binary in binaries:
        if binary.is_symlink():
            continue
        dependencies = run('otool', '-L', str(binary)).stdout.splitlines()[1:]
        for line in dependencies:
            name = line.strip().split(' (', 1)[0]
            if name.startswith(('/opt/', '/Users/', '/usr/local/')):
                raise RuntimeError(f'Unbundled dependency in {binary.name}: {name}')
    env = dict(os.environ, XDG_CONFIG_HOME=str(workspace / 'config'), XDG_DATA_HOME=str(workspace / 'data'),
               DYLD_LIBRARY_PATH='', DYLD_FALLBACK_LIBRARY_PATH=str(workspace / 'no-external-libraries'),
               LIDAR_VIEWER_UI_TRACE='1')
    help_text = run(str(executable), '--help', cwd=workspace, env=env).stdout
    if 'Usage: lidar_studio' not in help_text:
        raise RuntimeError('Packaged app did not start')
    fixture = workspace / 'points.bin'
    fixture.write_bytes(b''.join(struct.pack('ffff', x * .1, y * .1, 0., .5)
                                 for x in range(-15, 16) for y in range(-15, 16)))
    inspect = run(str(executable), str(fixture), '--inspect', cwd=workspace, env=env)
    if '961' not in inspect.stdout + inspect.stderr:
        raise RuntimeError('Packaged app could not load the point-cloud fixture')
    graphics = probe_library(next((app / "Contents/Frameworks").glob("libglfw.3.dylib")))
    if not require_gui and lacks_runner_opengl(graphics):
        gui_status = "unverified: runner has no native OpenGL pixel format"
        print("GUI NOT VERIFIED: independent GLFW 3.3/4.1 probes failed on this runner.", flush=True)
    else:
        log_path = workspace / 'gui.log'
        with log_path.open('w') as log:
            process = subprocess.Popen([str(executable), str(fixture)], cwd=workspace, env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 25
                while 'UI open_input' not in log_path.read_text(errors='replace'):
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('Packaged GUI did not render: ' + log_path.read_text(errors='replace')[-6000:])
                    time.sleep(.1)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)
        gui_status = "passed"
    return dict(app=app.name, version=info['CFBundleShortVersionString'], minimum_macos=minimum,
                architecture=platform.machine(), signature='ad-hoc' if 'Signature=adhoc' in signature else 'Developer ID',
                gatekeeper_accepted=assessment.returncode == 0, runtime='passed', gui=gui_status)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packages', type=Path, required=True)
    parser.add_argument('--require-gui', action='store_true', help='Fail if the runner cannot render OpenGL')
    args = parser.parse_args()
    args.packages = args.packages.resolve()
    if sys.platform != 'darwin':
        parser.error('Run this verification on a macOS runner')
    packages = sorted([*args.packages.glob('*.zip'), *args.packages.glob('*.dmg')])
    if len(packages) != 2:
        raise RuntimeError('Expected a ZIP and DMG macOS package')
    for package in packages:
        with tempfile.TemporaryDirectory(prefix='lidar-macos-package-') as temporary:
            workspace = Path(temporary)
            if package.suffix == '.zip':
                run('ditto', '-x', '-k', str(package.resolve()), str(workspace / 'extracted'))
            else:
                mount = workspace / 'mounted'
                run('hdiutil', 'attach', '-readonly', '-nobrowse', '-mountpoint', str(mount), str(package.resolve()))
                try:
                    apps = list(mount.glob('*.app'))
                    if len(apps) != 1:
                        raise RuntimeError('DMG must contain exactly one app')
                    run('ditto', str(apps[0]), str(workspace / 'extracted' / apps[0].name))
                finally:
                    run('hdiutil', 'detach', str(mount))
            apps = list((workspace / 'extracted').rglob('*.app'))
            if len(apps) != 1:
                raise RuntimeError('Package must contain exactly one app')
            print(json.dumps(dict(package=package.name, **check_app(apps[0], workspace, args.require_gui))), flush=True)


if __name__ == '__main__':
    main()
