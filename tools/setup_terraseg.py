#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Install the optional TerraSeg Small model for non-commercial use."""

import argparse
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent.parent
WORKER = ROOT / "python" / "terraseg_worker.py"
sys.path.insert(0, str(WORKER.parent))
from terraseg_worker import CHECKPOINT_BYTES, CHECKPOINT_NAME, MODEL_REVISION, verify_checkpoint

SOURCE = "https://github.com/tudelft-iv/TerraSeg.git"
SOURCE_REVISION = "7a8df87b81d726f15be377ba08992bc1d8b5b6fe"
MODEL_PAGE = "https://huggingface.co/TedLentsch/TerraSeg"
MODEL_URL = f"{MODEL_PAGE}/resolve/{MODEL_REVISION}/{CHECKPOINT_NAME}"


def user_directory(variable, default):
    directory = Path(os.environ.get(variable, default))
    return (directory if directory.is_absolute() else default) / "lidar_viewer"


def run(*args, capture=False, env=None):
    result = subprocess.run(args, check=True, text=True, env=env, stdout=subprocess.PIPE if capture else None)
    return result.stdout.strip() if capture else None


def prepare_upstream(git, data):
    source = data / f"source-{SOURCE_REVISION[:12]}"
    if not source.exists():
        with tempfile.TemporaryDirectory(prefix="source-", dir=data) as temporary:
            checkout = Path(temporary) / "checkout"
            run(git, "init", "--quiet", checkout)
            run(git, "-C", checkout, "fetch", "--quiet", "--depth=1", SOURCE, SOURCE_REVISION)
            run(git, "-C", checkout, "checkout", "--quiet", "--detach", SOURCE_REVISION)
            checkout.rename(source)
    revision = run(git, "-C", source, "rev-parse", "HEAD", capture=True)
    status = run(git, "-C", source, "status", "--porcelain", "--untracked-files=no", capture=True)
    if revision != SOURCE_REVISION or status:
        raise RuntimeError("The cached TerraSeg source differs from the pinned upstream revision")
    return source


def prepare_checkpoint(data):
    target = data / CHECKPOINT_NAME
    try:
        verify_checkpoint(target)
        return target
    except (OSError, ValueError):
        print(f"Downloading {CHECKPOINT_NAME} from {MODEL_PAGE}", flush=True)

    with tempfile.NamedTemporaryFile(prefix="weights-", suffix=".part", dir=data, delete=False) as temp:
        path = Path(temp.name)
    try:
        with urlopen(MODEL_URL, timeout=90) as stream, path.open("wb") as output:
            size = 0
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                size += len(chunk)
                if size > CHECKPOINT_BYTES:
                    raise RuntimeError("TerraSeg checkpoint exceeds its expected size")
                output.write(chunk)
        verify_checkpoint(path)
        path.replace(target)
    finally:
        path.unlink(missing_ok=True)
    return target


def publish_config(config, python, checkpoint):
    if any("\n" in str(path) or "\r" in str(path) for path in (python, checkpoint)):
        raise ValueError("Newlines in TerraSeg runtime paths are unsupported")
    config.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix="terraseg-", dir=config, delete=False) as temp:
        temporary = Path(temp.name)
        temp.write(f"python={python}\ncheckpoint={checkpoint}\n")
    try:
        temporary.replace(config / "terraseg.cfg")
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--accept-noncommercial", action="store_true",
                        help="Accept the official checkpoint's CC BY-NC-SA 4.0 terms")
    args = parser.parse_args()
    if not args.accept_noncommercial:
        parser.error(f"TerraSeg weights use CC BY-NC-SA 4.0. See {MODEL_PAGE} and pass --accept-noncommercial.")
    if sys.platform != "linux" or platform.machine() != "x86_64":
        parser.error("The pinned TerraSeg runtime supports Linux x86_64 with an NVIDIA CUDA GPU")

    git, uv = shutil.which("git"), shutil.which("uv")
    if not git or not uv:
        parser.error("Install git and uv before preparing TerraSeg (https://docs.astral.sh/uv/getting-started/installation/)")
    import fcntl

    config = user_directory("XDG_CONFIG_HOME", Path.home() / ".config")
    data = user_directory("XDG_DATA_HOME", Path.home() / ".local/share") / "terraseg"
    data.mkdir(parents=True, exist_ok=True)
    try:
        with (data / "setup.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            print("Preparing the locked TerraSeg CUDA runtime. The first install downloads several GB.", flush=True)
            source = prepare_upstream(git, data)
            runtime = data / f"runtime-{SOURCE_REVISION[:12]}"
            # Use the official committed lock (including hashes and CUDA wheels)
            # without resolving to new versions when upstream metadata changes.
            run(uv, "sync", "--project", source, "--frozen", "--no-dev", "--no-editable",
                "--package", "terraseg", "--python", "3.10", env=dict(os.environ, UV_PROJECT_ENVIRONMENT=str(runtime)))
            checkpoint = prepare_checkpoint(data)
            python = runtime / "bin/python"
            run(python, "-I", "-B", WORKER, "--checkpoint", checkpoint, "--check")
            publish_config(config, python, checkpoint)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"TerraSeg setup failed: {error}", file=sys.stderr)
        return 1
    print(f"TerraSeg Small is ready. Configuration: {config / 'terraseg.cfg'}", flush=True)
    print(f"Weights: CC BY-NC-SA 4.0 · {MODEL_PAGE}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
