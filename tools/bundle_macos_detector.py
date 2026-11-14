#!/usr/bin/env python3
"""Build the relocatable, offline CPU detector shipped inside the Mac app."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from lidar_detection.models import MODELS

PYTHON_URL = 'https://github.com/astral-sh/python-build-standalone/releases/download/20261110/cpython-3.12.16%2B20261110-aarch64-apple-darwin-install_only_stripped.tar.gz'
PYTHON_SHA256 = '2f11b8aa3726bdbd87dd9eb76568412faddf4f47e26913fa45d6a2e424972acd'
REQUIREMENTS = ['numpy==2.2.6', 'onnx==1.20.1', 'onnxruntime==1.23.2',
                'protobuf==7.36.2', 'flatbuffers==25.12.19', 'sympy==1.14.0',
                'mpmath==1.3.0', 'ml_dtypes==0.5.4', 'typing_extensions==4.16.0',
                'packaging==26.3', 'coloredlogs==15.0.1', 'humanfriendly==10.0']


def checksum(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def download(url, target, expected):
    if target.is_file() and checksum(target) == expected:
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + '.download')
    try:
        print('Downloading:', target.name, flush=True)
        request = urllib.request.Request(url, headers={'User-Agent': 'LiDAR-Studio-detector-bundle'})
        with urllib.request.urlopen(request, timeout=120) as source, partial.open('wb') as output:
            shutil.copyfileobj(source, output, 1024 * 1024)
        if checksum(partial) != expected:
            raise RuntimeError('Checksum mismatch: ' + target.name)
        partial.replace(target)
    finally:
        partial.unlink(missing_ok=True)


def build(destination):
    if sys.platform != 'darwin' or platform.machine() != 'arm64':
        raise RuntimeError('Build this runtime on an Apple Silicon Mac')
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    runtime = destination / 'runtime'
    with tempfile.TemporaryDirectory(prefix='lidar-detector-python-') as temporary:
        temporary = Path(temporary)
        archive = temporary / 'python.tar.gz'
        download(PYTHON_URL, archive, PYTHON_SHA256)
        with tarfile.open(archive) as source:
            source.extractall(temporary, filter='data')
        if runtime.exists():
            shutil.rmtree(runtime)
        (temporary / 'python').rename(runtime)
    python = runtime / 'bin/python3'
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PIP_DISABLE_PIP_VERSION_CHECK='1')
    subprocess.run([str(python), '-E', '-s', '-B', '-m', 'pip', 'install', '--no-cache-dir',
                    '--only-binary=:all:', *REQUIREMENTS], env=env, check=True)
    for variant, spec in MODELS.items():
        model = destination / 'models' / spec.folder
        for name, url, expected in spec.files:
            download(url, model / name, expected)
        subprocess.run([str(python), '-E', '-s', '-B', '-c',
                        'import sys;sys.path.insert(0,sys.argv[1]);from lidar_detection.centerpoint import CenterPoint;'
                        'm=CenterPoint(sys.argv[2],"cpu",sys.argv[3]);print("Validated bundled model:",m.spec.title,m.provider)',
                        str(ROOT / 'python'), str(model), variant], env=env, check=True)
    (destination / 'manifest.json').write_text(json.dumps(dict(
        python='3.12.16', python_source=PYTHON_URL, python_sha256=PYTHON_SHA256,
        requirements=REQUIREMENTS, models={name: dict(folder=m.folder, files={n: h for n, _, h in m.files})
                                         for name, m in MODELS.items()}), indent=2))
    # Imports in the app also use -B; no interpreter cache writes may invalidate
    # the read-only, signed application bundle.
    for cache in runtime.rglob('__pycache__'):
        shutil.rmtree(cache)
    print('Offline Mac detector prepared:', destination, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    build(parser.parse_args().output)
