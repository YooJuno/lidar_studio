#!/usr/bin/env python3
"""Run the actual LiDAR Studio perception backends on labelled nuScenes scans."""

import argparse
from collections import deque
import csv
from dataclasses import astuple
import hashlib
import heapq
import json
import math
import os
import platform
import runpy
import shutil
import subprocess
import sys
from pathlib import Path

from nuscenes_benchmark_data import prepare


GROUND = ("fast_grid", "gseg3d", "fugseg", "patchworkpp", "patchwork_classic", "grid", "ransac", "terraseg", "linefit")
DETECTORS = ("short_range", "tiny", "base", "transfusion")
DEFAULT_GROUND = ("fast_grid", "fugseg", "gseg3d", "patchworkpp")
DEFAULT_TRACKING = "studio:3:0.8"
COMPARISON_PROTOCOL = "lidar-studio-nuscenes-local-v2"


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
    parser.add_argument("--hash-clouds", action="store_true", help="Hash every selected point file for byte-verified comparisons")
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


def hash_file(hashed, path):
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            hashed.update(chunk)


def digest(path):
    hashed = hashlib.sha256()
    hash_file(hashed, path)
    return hashed.hexdigest()


def hash_record(hashed, record):
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    hashed.update(encoded + b"\n")


def record_digest(record):
    hashed = hashlib.sha256()
    hash_record(hashed, record)
    return hashed.hexdigest()


def selection_digest(dataset, hash_clouds):
    hashed = hashlib.sha256()
    hash_record(hashed, (dataset.version, hash_clouds, dataset.lidarseg_categories))
    for frame in dataset.frames:
        hash_record(hashed, (
            frame.scene, frame.frame, frame.sample, frame.timestamp_ns, frame.keyframe, frame.point_count,
            frame.ego_rotation, frame.ego_translation, frame.sensor_rotation, frame.sensor_translation,
            digest(frame.labels) if frame.labels is not None else None,
            [astuple(annotation) for annotation in frame.annotations],
            digest(frame.cloud) if hash_clouds else None,
        ))
    return hashed.hexdigest()


def timing_host():
    processor = platform.processor() or platform.machine()
    if platform.system() == "Linux":
        try:
            with Path("/proc/cpuinfo").open(encoding="utf-8") as source:
                processor = next((line.partition(":")[2].strip() for line in source
                                  if line.startswith("model name")), processor)
        except OSError:
            pass
    accelerators = ""
    nvidia = shutil.which("nvidia-smi")
    if nvidia:
        try:
            query = [nvidia, "--query-gpu=name,driver_version", "--format=csv,noheader"]
            result = subprocess.run(query, capture_output=True, text=True, timeout=3, check=False)
            if result.returncode == 0:
                accelerators = "; ".join(result.stdout.splitlines())
        except (OSError, subprocess.TimeoutExpired):
            pass
    parts = [part for part in (platform.platform(), processor, accelerators) if part]
    # Include the host identity in the digest without displaying the hostname.
    host_sha256 = record_digest((platform.node(), parts, os.cpu_count()))
    return " | ".join(parts), host_sha256


def model_manifest(args):
    if not args.detector:
        return {}
    root = args.python_root or Path(__file__).resolve().parent.parent / "python"
    path = root / "lidar_detection/models.py"
    if not path.is_file():
        return {}
    models = runpy.run_path(str(path))["MODELS"]
    return {
        name: {"title": models[name].title,
               "weights_sha256": {filename: sha256 for filename, _, sha256 in models[name].files
                                  if filename.endswith(".onnx")}}
        for name in args.detector if name in models
    }


def breakdown_rows(runs):
    rows = []

    def add(stage, method, scope, label, support, metrics, counts):
        rows.append({"stage": stage, "method": method, "scope": scope, "label": label,
                     "support": support, "metrics": metrics, "counts": counts})

    def details(stage, method, overall, classes, scenes):
        groups = (("Overall", {"All": overall}), ("Class", classes), ("Scene", scenes))
        for scope, values in groups:
            for name, stats in values.items():
                metric = "ap" if stage == "Detection" else "idf1"
                metrics = {"AP" if stage == "Detection" else "IDF1": stats[metric],
                           "Precision": stats["precision"], "Recall": stats["recall"]}
                counts = {"False positives": stats["fp"], "Missed boxes": stats["fn"]}
                if stage == "Tracking":
                    counts.update({"ID switches": stats["id_switches"], "Fragments": stats["fragmentations"],
                                   "Predicted false positives": stats["coasting_fp"]})
                add(stage, method, scope, name, stats["ground_truth"], metrics, counts)

    def ratio(count, support):
        return count / support if support else None

    for run in runs:
        if run["status"] != "ok":
            continue
        method, result = run["method"], run["result"]
        if run["stage"] == "ground":
            groups = (("Overall", {"All": result["overall"]}), ("Scene", result["scenes"]))
            for scope, values in groups:
                for name, stats in values.items():
                    support = sum(stats[key] for key in ("true_ground", "false_ground", "missed_ground", "true_non_ground"))
                    metrics = {"IoU": stats["iou"], "Precision": stats["precision"], "Recall": stats["recall"],
                               "Obstacle loss": stats["lost_non_ground_rate"],
                               "Pedestrian point loss": ratio(stats["removed_pedestrian_points"], stats["pedestrian_points"]),
                               "Vehicle point loss": ratio(stats["removed_vehicle_points"], stats["vehicle_points"])}
                    counts = {"Non-ground points removed": stats["false_ground"],
                              "Ground points missed": stats["missed_ground"],
                              "Ambiguous points": stats["ambiguous_prediction_points"]}
                    add("Ground", method, scope, name, support, metrics, counts)
            continue
        detection = result["detection"]
        details("Detection", method, detection["overall"], detection["classes"], detection["scenes"])
        for profile, track in result["tracking"].items():
            details("Tracking", f"{method} / {profile}", track, track["classes"], track["scenes"])
    return rows


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
            recall = track["recall"]
            recall_label = f"{recall:.1%} recall" if recall is not None else "No labelled targets"
            add("Tracking", f"{name} / {profile}", "Local IDF1", track["idf1"], track["timing"],
                f"{recall_label} | {track['id_switches']:,} ID switches | {track['fragmentations']:,} fragments | {track['coasting_fp']:,} predicted false positives")
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
        print("Fingerprinting the selection and labels...", flush=True)
        selection_sha256 = selection_digest(data, args.hash_clouds)
        models = model_manifest(args)
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
    host_label, host_sha256 = timing_host()
    settings = {"protocol": COMPARISON_PROTOCOL, "detector_score": args.score,
                "candidate_floor": 0.10, "tracking": sorted(args.track),
                "replay": "ordered offline; every selected scan", "tracking_coordinates": "sensor-relative"}
    settings["comparison_sha256"] = record_digest(settings)
    report = {
        "format": "lidar-studio-perception-benchmark", "schema": 1,
        "dataset": {"root": str(data.root), "version": args.version, "split": args.split, "scenes": scenes,
                    "scans": len(data.frames), "keyframes": len(data.keyframes), "point_labelled_keyframes": labelled,
                    "keyframes_only": args.keyframes_only, "max_keyframes": args.max_keyframes,
                    "selection_sha256": selection_sha256, "cloud_payloads_hashed": args.hash_clouds},
        "settings": settings,
        "environment": {"platform": platform.platform(), "processor": platform.processor(),
                        "timing_host_sha256": host_sha256, "timing_host": host_label,
                        "runner": str(args.runner.resolve()), "runner_sha256": digest(args.runner)},
        "detector_models": models, "rows": rows, "breakdowns": breakdown_rows(runs),
        "examples": failure_examples(output, runs), "runs": runs,
    }
    write_report(output, report)
    print(f"Report: {output / 'benchmark.json'}", flush=True)
    return int(any(run["status"] != "ok" for run in runs))


if __name__ == "__main__":
    sys.exit(main())
