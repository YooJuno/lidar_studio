#!/usr/bin/env python3
"""Run the actual LiDAR Studio perception backends on labelled nuScenes scans."""

import argparse
from collections import deque
import csv
import hashlib
import heapq
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path

from nuscenes_benchmark_data import prepare


GROUND = ("fast_grid", "gseg3d", "fugseg", "patchworkpp", "patchwork_classic", "grid", "ransac", "terraseg")
DETECTORS = ("short_range", "tiny", "base", "transfusion")
DEFAULT_GROUND = ("fast_grid", "fugseg", "gseg3d", "patchworkpp")
DEFAULT_TRACKING = "studio:3:0.8"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Extracted nuScenes data and optional LiDARSeg labels")
    parser.add_argument("--runner", type=Path, required=True, help="Built lidar_studio_benchmark executable")
    parser.add_argument("--studio-runner", action="store_true", help="Use the benchmark backend in the Studio executable")
    parser.add_argument("--output", type=Path, required=True, help="New directory for the report and frame data")
    parser.add_argument("--version", choices=("v1.0-mini", "v1.0-trainval"), default="v1.0-mini")
    parser.add_argument("--split", choices=("mini_val", "mini_train", "all", "val"), help="Defaults to mini_val or val")
    parser.add_argument("--scene", action="append", default=[], help="Limit to a named scene in this split; repeatable")
    parser.add_argument("--max-keyframes", type=int, help="Limit the scored prefix for a smoke test")
    parser.add_argument("--keyframes-only", action="store_true", help="Process 2 Hz labelled keyframes only, including tracking")
    parser.add_argument("--ground", choices=GROUND, action="append", help="Ground method; repeat to compare. Defaults to four CPU methods")
    parser.add_argument("--no-ground", action="store_true", help="Run detection and tracking without ground evaluation")
    parser.add_argument("--detector", choices=DETECTORS, action="append", default=[], help="Installed detector; repeat to compare")
    parser.add_argument("--track", action="append", default=[], metavar="NAME:HITS:SECONDS", help="Tracker configuration; repeat to compare on identical detections")
    parser.add_argument("--score", type=float, default=0.50, help="Common detection and tracking threshold; default 0.50")
    parser.add_argument("--python-root", type=Path, help="Directory containing detector_worker.py for the native runner")
    args = parser.parse_args()
    if args.no_ground and args.ground:
        parser.error("--no-ground cannot be combined with --ground")
    args.ground = [] if args.no_ground else list(dict.fromkeys(args.ground or DEFAULT_GROUND))
    args.detector = list(dict.fromkeys(args.detector))
    if not args.ground and not args.detector:
        parser.error("Select a ground method or detector")
    if args.track and not args.detector:
        parser.error("--track requires --detector")
    if not math.isfinite(args.score) or not 0.10 <= args.score <= 1:
        parser.error("--score must be from 0.10 to 1.00")
    if args.max_keyframes is not None and args.max_keyframes < 1:
        parser.error("--max-keyframes must be positive")
    args.split = args.split or ("mini_val" if args.version == "v1.0-mini" else "val")
    if args.detector and not args.track:
        args.track = [DEFAULT_TRACKING]
    return args


def digest(path):
    hashed = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            hashed.update(chunk)
    return hashed.hexdigest()


def run_backend(args, stage, method, directory, manifest):
    directory.mkdir()
    command = [str(args.runner.resolve())]
    if args.studio_runner:
        command.append("--benchmark-backend")
    command.extend(("--manifest", str(manifest), "--output", str(directory),
                    "--ground" if stage == "ground" else "--detector", method))
    if args.python_root:
        command.extend(("--python-root", str(args.python_root.resolve())))
    if stage == "objects":
        command.extend(("--score", str(args.score)))
        for profile in args.track:
            command.extend(("--track", profile))
    log = directory.with_suffix(".log")
    with log.open("w", encoding="utf-8") as output:
        print(f"Running {stage} / {method} ({log})", flush=True)
        tail = deque(maxlen=6)
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                              encoding="utf-8", errors="replace",
                              creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0) as child:
            for line in child.stdout:
                output.write(line)
                output.flush()
                tail.append(line)
                if line.startswith("Processed "):
                    print(f"{method} · {line.rstrip()}", flush=True)
            code = child.wait()
    if code:
        raise RuntimeError(f"Native runner exited with {code}. {''.join(tail).strip()}")


def ui_rows(runs):
    rows = []

    def add(stage, name, metric, value, timing, details):
        rows.append({"stage": stage, "method": name, "metric": metric, "value": value,
                     "p50_ms": timing.get("p50_ms"), "p95_ms": timing.get("p95_ms"),
                     "p99_ms": timing.get("p99_ms"), "backend": ", ".join(timing.get("providers", [])),
                     "details": details})

    for run in runs:
        name = run["method"]
        if run["status"] != "ok":
            add("Ground" if run["stage"] == "ground" else "Detection", name, "", None, {}, run["error"])
            continue
        result = run["result"]
        if run["stage"] == "ground":
            points = result["overall"]
            add("Ground", name, "Ground IoU", points.get("iou"), result["timing"],
                f"{points['false_ground']:,} non-ground points removed | {points['missed_ground']:,} ground points left")
            continue
        detection = result["detection"]
        person = detection["classes"].get("pedestrian", {})
        add("Detection", name, "Local center AP", detection.get("mean_ap"), result["timing"],
            f"Pedestrian {person.get('fp', 0):,} false positives | {person.get('fn', 0):,} misses at the selected score")
        for profile, track in result["tracking"].items():
            add("Tracking", f"{name} / {profile}", "Recall", track.get("recall"), track["timing"],
                f"{track['id_switches']:,} ID switches | {track['fragmentations']:,} fragments | {track['coasting_fp']:,} predicted false positives")
    return rows


def failure_examples(output, runs):
    """Save a short list for review; complete frame-level counts stay in the TSVs."""
    cases = []

    def collect(file, method, stage, column, description, predicate):
        with file.open(encoding="utf-8", newline="") as source:
            rows = (row for row in csv.DictReader(source, delimiter="\t") if predicate(row) and int(row[column]) > 0)
            for row in heapq.nlargest(3, rows, key=lambda item: int(item[column])):
                name = f"{method} / {row['profile']}" if "profile" in row else method
                cases.append({"stage": stage, "method": name, "scene": row["scene"], "frame": row["frame"],
                              "description": f"{int(row[column]):,} {description}"})

    def any_frame(_):
        return True

    for run in runs:
        if run["status"] != "ok":
            continue
        method, folder = run["method"], output / run["folder"]
        if run["stage"] == "ground":
            collect(folder / "ground-errors.tsv", method, "Ground", "false_ground", "non-ground points removed", any_frame)
            continue
        collect(folder / "object-errors.tsv", method, "Detection", "fn", "pedestrians missed",
                lambda row: row["class"] == "pedestrian")
        collect(folder / "tracking-errors.tsv", method, "Tracking", "id_switches", "ID switches",
                any_frame)
    return cases


def write_report(output, report):
    columns = ("stage", "method", "metric", "value", "p50_ms", "p95_ms", "p99_ms", "backend", "details")
    with (output / "summary.tsv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns, delimiter="\t")
        writer.writeheader()
        writer.writerows(report["rows"])
    temporary = output / "benchmark.json.tmp"
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(output / "benchmark.json")


def main():
    sys.stdout.reconfigure(errors="backslashreplace")
    sys.stderr.reconfigure(errors="backslashreplace")
    args = parse_args()
    try:
        from nuscenes_benchmark_metrics import evaluate_ground, evaluate_objects
    except ModuleNotFoundError as error:
        print(f"Missing {error.name}. Install the benchmark dependencies with: python -m pip install numpy scipy", file=sys.stderr)
        return 1

    output = args.output.resolve()
    try:
        if not args.runner.is_file():
            raise ValueError(f"Native runner is missing: {args.runner}")
        output.mkdir(parents=True, exist_ok=False)
        manifest = output / "frames.tsv"
        data = prepare(args.dataset.resolve(), args.version, args.split, manifest,
                       scenes=tuple(args.scene), max_keyframes=args.max_keyframes,
                       keyframes_only=args.keyframes_only)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Benchmark setup failed: {error}", file=sys.stderr)
        return 1

    labelled = sum(frame.labels is not None for frame in data.keyframes)
    scenes = list(dict.fromkeys(frame.scene for frame in data.frames))
    print(f"{len(data.frames)} input scans, {len(data.keyframes)} box-labelled keyframes, "
          f"{labelled} point-labelled keyframes, {len(scenes)} scenes", flush=True)

    runs = []
    for stage, methods in (("ground", args.ground), ("objects", args.detector)):
        for method in methods:
            directory = output / f"{stage}-{method}"
            run = {"stage": stage, "method": method, "folder": directory.name}
            try:
                if stage == "ground" and labelled != len(data.keyframes):
                    raise ValueError("Install the matching nuScenes LiDARSeg expansion to score ground accuracy.")
                run_backend(args, stage, method, directory, manifest)
                result = evaluate_ground(data, directory) if stage == "ground" else evaluate_objects(data, directory, args.score)
                run.update(status="ok", result=result)
            except (OSError, ValueError, RuntimeError) as error:
                run.update(status="failed", error=str(error))
                print(f"{stage} / {method}: {error}", file=sys.stderr, flush=True)
            runs.append(run)

    rows = ui_rows(runs)
    report = {
        "format": "lidar-studio-perception-benchmark", "schema": 1,
        "dataset": {"root": str(data.root), "version": args.version, "split": args.split, "scenes": scenes,
                    "scans": len(data.frames), "keyframes": len(data.keyframes), "point_labelled_keyframes": labelled,
                    "keyframes_only": args.keyframes_only, "max_keyframes": args.max_keyframes},
        "settings": {"detector_score": args.score, "candidate_floor": 0.10, "tracking": args.track,
                     "replay": "ordered offline; every selected scan", "tracking_coordinates": "sensor-relative"},
        "environment": {"platform": platform.platform(), "processor": platform.processor(),
                        "runner": str(args.runner.resolve()), "runner_sha256": digest(args.runner)},
        "rows": rows, "examples": failure_examples(output, runs), "runs": runs,
    }
    write_report(output, report)
    print(f"Report: {output / 'benchmark.json'}", flush=True)
    return int(any(run["status"] != "ok" for run in runs))


if __name__ == "__main__":
    sys.exit(main())
