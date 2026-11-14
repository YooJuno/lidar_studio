#!/usr/bin/env python3
"""Verify extracted ZIP/DMG apps on macOS; report Gatekeeper separately."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import plistlib
import re
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

from macos_gl_probe import lacks_runner_opengl, probe_library


def run(*args, **kwargs):
    result = subprocess.run(args, text=True, capture_output=True, timeout=90, **kwargs)
    if result.returncode:
        raise RuntimeError(f'{args[0]} failed ({result.returncode}):\n{result.stdout}\n{result.stderr}')
    return result


def check_detector(app, executable, workspace, env):
    assets=app/'Contents/Resources/detector'
    if not (assets/'runtime/bin/python3').is_file() or not (assets/'manifest.json').is_file():
        raise RuntimeError('Offline Mac detector runtime is missing')
    if not os.access(assets/'runtime/bin/python3',os.X_OK):
        raise RuntimeError('Bundled Python lost its executable permissions during packaging')
    # A real outdoor scan used by the public sample, with pinned bytes.
    cloud=workspace/'driving.bin'
    url='https://raw.githubusercontent.com/url-kaist/patchwork-plusplus/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/data/000000.bin'
    with urllib.request.urlopen(url,timeout=120) as source:
        cloud.write_bytes(source.read())
    if hashlib.sha256(cloud.read_bytes()).hexdigest()!='bf272996d5b6d25cc5589e1089137cb20a98b63bd4823a7fea5631b359f6d68c':
        raise RuntimeError('Point-cloud fixture checksum mismatch')
    config=Path(env['XDG_CONFIG_HOME'])/'lidar_viewer'
    profiles=config/'profiles';profiles.mkdir(parents=True,exist_ok=True)
    if (config/'detector.cfg').exists():
        raise RuntimeError('Packaged detector test must start without user model setup')
    for variant in ['tiny','short_range']:
        (profiles/'default.cfg').write_text(f'detection_model={variant}\ndetection_min_score=0.25\ndetection_classes=31\n')
        result=run(str(executable),str(cloud),'--sensor-height','1.73','--inspect','--detect',cwd=workspace,env=env)
        output=result.stdout+result.stderr
        match=re.search(r'Detector: '+variant+r' \| CPUExecutionProvider \| .*? \| (\d+) boxes',output)
        if not match or int(match.group(1))<1:
            raise RuntimeError('Original bundled model produced no valid boxes on the real sample: '+output)
        if (config/'detector.cfg').exists():
            raise RuntimeError('Built-in inference unexpectedly required a model configuration file')
        print(f'Offline packaged {variant}: CPU inference on 124668 real points, {match.group(1)} detections',flush=True)
        # Running an import or inference must never add pyc or fused files to
        # the signed app, including when launched from a read-only DMG.
        run('codesign','--verify','--deep','--strict',str(app))


def check_app(app, workspace, require_gui=False, require_detector=False):
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
    if require_detector:
        check_detector(app,executable,workspace,env)
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
                gatekeeper_accepted=assessment.returncode == 0, runtime='passed', gui=gui_status,
                learned_detection='both original models passed' if require_detector else 'not checked')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packages', type=Path, required=True)
    parser.add_argument('--require-gui', action='store_true', help='Fail if the runner cannot render OpenGL')
    parser.add_argument('--require-detector', action='store_true', help='Require offline inference using both original bundled models')
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
            print(json.dumps(dict(package=package.name, **check_app(apps[0], workspace, args.require_gui,args.require_detector))), flush=True)


if __name__ == '__main__':
    main()
