#!/usr/bin/env python3
"""Exercise a published ZIP example, its replay profile and cache cleanup under Xvfb."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from ui_controls import Controls
from ui_gl_smoke import X11, capture


EXAMPLES = {
    "os0": ("ouster-os0-urban-v1", 201, "example-os0"),
    "os1": ("ouster-os1-urban-v1", 258, "example-os1"),
    "os2": ("ouster-os2-urban-v1", 201, "example-os2"),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dataset", choices=EXAMPLES, required=True)
    args = parser.parse_args()
    cache_name, frames, profile = EXAMPLES[args.dataset]
    args.output.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=args.dataset + "-", dir=args.output.resolve()))
    repo = Path(__file__).resolve().parents[1]
    viewer = output / "build/lidar_studio"
    viewer.parent.mkdir()
    shutil.copy2(args.viewer.resolve(), viewer)
    shutil.copytree(repo / "config", output / "config")
    cache = output / "data/lidar_viewer/examples" / cache_name
    log = output / "viewer.log"
    env = dict(os.environ, XDG_CONFIG_HOME=str(output / "settings"),
               XDG_DATA_HOME=str(output / "data"), LIDAR_VIEWER_UI_TRACE="1")
    x11 = X11()
    subprocess.run(["xrdb", "-merge"], input="Xft.dpi: 96\n", text=True, check=True)
    with log.open("wb") as stream:
        process = subprocess.Popen([str(viewer)], cwd=output, env=env, stdout=stream, stderr=subprocess.STDOUT)

    def wait(condition, seconds=100):
        deadline = time.monotonic() + seconds
        while not condition():
            if process.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError(log.read_text(errors="replace")[-2400:])
            time.sleep(.05)

    def saw(message):
        return message in log.read_text(errors="replace")

    window = None
    try:
        wait(lambda: x11.viewer_window() is not None, 15)
        window = x11.viewer_window()
        x11.focus(window)
        x11.resize(window, 1600, 960)
        controls = Controls(x11, window, log)
        controls.click("open_input")
        controls.rect("open_input_file")
        controls.rect("open_input_folder")
        assert not cache.exists(), "Opening the chooser started a download"
        controls.click("example_sensor")
        controls.click("example_" + args.dataset)
        controls.click("example_start")
        wait(lambda: saw(f" | {frames} frames"))
        assert saw("Example ready | cached 0 / 1"), "Initial open did not download the archive"
        assert not list(cache.glob("*.zip")) and not list(cache.glob("*.part"))
        assert (cache / "lidar.pcap").is_file() and (cache / "lidar.json").is_file()
        capture(x11, window, output / "playing.png")
        controls.click("tab_Align")
        controls.chord("s")
        saved = output / "settings/lidar_viewer/profiles" / (profile + ".cfg")
        wait(saved.is_file, 5)
        timestamps = {p.name: p.stat().st_mtime_ns for p in cache.iterdir()}
        controls.click("open_input")
        controls.click("example_start")
        wait(lambda: saw("Example ready | cached 1 / 1"))
        wait(lambda: log.read_text(errors="replace").count("Opened point cloud:")>=2)
        assert timestamps == {p.name: p.stat().st_mtime_ns for p in cache.iterdir()}
        (cache / "keep.txt").write_text("user file")
        controls.click("open_input")
        controls.click("example_delete")
        wait(lambda: saw("Example deleted | " + cache_name))
        assert saw("Example unloaded | " + cache_name)
        assert [p.name for p in cache.iterdir()] == ["keep.txt"]
        print(f"{args.dataset}: {frames} playable frames, profile, cached reopen and deletion passed ({output})")
    except Exception:
        if window and process.poll() is None:
            capture(x11, window, output / "failure.png")
        print(f"Diagnostics: {output}")
        raise
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=8)
        x11.close()


if __name__ == "__main__":
    main()
