#!/usr/bin/env python3
"""Check the plist using CMake's actual bundle parser on any CI platform."""
import argparse
from pathlib import Path
import plistlib
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmake', default=shutil.which('cmake'))
    args = parser.parse_args()
    template = Path(__file__).resolve().parents[1] / 'cmake/macos/Info.plist.in'
    with tempfile.TemporaryDirectory(prefix='lidar-macos-metadata-') as temporary:
        root = Path(temporary)
        app = root / 'LiDAR Studio.app'
        executable = app / 'Contents/MacOS/lidar_studio'
        executable.parent.mkdir(parents=True)
        executable.touch()
        script = root / 'verify.cmake'
        script.write_text(f'''
include(BundleUtilities)
set(MACOSX_BUNDLE_EXECUTABLE_NAME lidar_studio)
set(MACOSX_BUNDLE_GUI_IDENTIFIER io.github.yoojuno.lidar)
set(MACOSX_BUNDLE_BUNDLE_NAME "LiDAR Studio")
set(MACOSX_BUNDLE_SHORT_VERSION_STRING 1.2.3)
set(MACOSX_BUNDLE_BUNDLE_VERSION 1.2.3)
set(CMAKE_OSX_DEPLOYMENT_TARGET 15.0)
configure_file([==[{template.as_posix()}]==] [==[{(app / 'Contents/Info.plist').as_posix()}]==])
get_bundle_main_executable([==[{app.as_posix()}]==] result)
if(NOT result STREQUAL [==[{executable.as_posix()}]==])
  message(FATAL_ERROR "BundleUtilities could not locate the packaged executable: ${{result}}")
endif()
''')
        subprocess.run([args.cmake, '-P', str(script)], check=True)
        info = plistlib.loads((app / 'Contents/Info.plist').read_bytes())
        assert info['LSMinimumSystemVersion'] == '15.0'
        assert info['CFBundleExecutable'] == 'lidar_studio'
        assert info['CFBundleDisplayName'] == 'LiDAR Studio'
        assert info['CFBundleShortVersionString'] == '1.2.3'
        print('macOS metadata: valid plist and CMake bundle parser passed')


if __name__ == '__main__':
    main()
