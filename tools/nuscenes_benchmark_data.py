"""Prepare ordered nuScenes LiDAR scans and their published benchmark labels."""

from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Iterable


# The official mini split is small enough to use without the Python devkit.
MINI_TRAIN = (
    "scene-0061", "scene-0553", "scene-0655", "scene-0757",
    "scene-0796", "scene-1077", "scene-1094", "scene-1100",
)
MINI_VAL = ("scene-0103", "scene-0916")
MANIFEST_COLUMNS = (
    "scene", "frame", "timestamp_ns", "keyframe", "cloud",
    "r00", "r01", "r02", "r10", "r11", "r12", "r20", "r21", "r22", "tx", "ty", "tz",
)
SCAN_POINT_BYTES = 20  # Little-endian float32 x, y, z, intensity, ring.
FRAME_TOKEN = re.compile(r"[0-9a-f]{32}")
Vec3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class Annotation:
    translation: Vec3
    size: Vec3  # Width, length, height in nuScenes order.
    rotation: Quaternion  # w, x, y, z, in global coordinates.
    category_name: str
    instance_token: str
    num_lidar_pts: int
    num_radar_pts: int
    visibility_token: str
    visibility: str


@dataclass(frozen=True, slots=True)
class Frame:
    scene: str
    frame: str  # LIDAR_TOP sample_data token.
    sample: str  # Associated nuScenes sample token.
    timestamp_ns: int
    keyframe: bool
    cloud: Path
    labels: Path | None
    point_count: int
    ego_rotation: Quaternion
    ego_translation: Vec3
    sensor_rotation: Quaternion
    sensor_translation: Vec3
    annotations: tuple[Annotation, ...]


@dataclass(frozen=True, slots=True)
class BenchmarkDataset:
    root: Path
    version: str
    split: str
    frames: tuple[Frame, ...]
    keyframes: tuple[Frame, ...]
    lidarseg_categories: dict[int, str]


def _records(table: Path):
    try:
        with table.open(encoding="utf-8") as source:
            if table.stat().st_size <= 64 * 1024 * 1024:
                data = json.load(source)
                if not isinstance(data, list):
                    raise ValueError("expected an array of records")
                yield from data
            else:
                # Full trainval tables contain millions of records. Filter them
                # as we read to keep peak memory proportional to the selection.
                try:
                    import ijson
                except ImportError as exc:
                    raise ValueError("install ijson to read large nuScenes metadata tables") from exc
                yield from ijson.items(source.buffer, "item", use_float=True)
    except (OSError, ValueError) as exc:
        raise ValueError(f"Cannot read {table}: {exc}") from exc


def _index(records: Iterable[dict], key: str, table: str) -> dict:
    result = {}
    for row in records:
        token = row[key]
        if not isinstance(token, str) or not token or token in result:
            raise ValueError(f"Invalid or duplicate {key} {token!r} in {table}")
        result[token] = row
    return result


def _scene_names(version: str, split: str) -> tuple[str, ...]:
    if version == "v1.0-mini":
        mini = {"mini_train": MINI_TRAIN, "mini_val": MINI_VAL, "all": MINI_TRAIN + MINI_VAL}
        if split not in mini:
            raise ValueError("v1.0-mini requires mini_train, mini_val or all")
        return mini[split]
    if version == "v1.0-trainval" and split == "val":
        try:
            from nuscenes.utils.splits import create_splits_scenes
        except ImportError as exc:
            raise ValueError("install nuscenes-devkit for the official trainval validation split") from exc
        return tuple(create_splits_scenes()["val"])
    raise ValueError("Use v1.0-mini or v1.0-trainval val with public ground truth")


def _sample_chain(scene: dict, samples: dict) -> list[str]:
    ordered = []
    seen = set()
    token = scene["first_sample_token"]
    while token:
        row = samples.get(token)
        previous = ordered[-1] if ordered else ""
        if (
            row is None or token in seen
            or row["scene_token"] != scene["token"] or row["prev"] != previous
        ):
            raise ValueError(f"Broken sample chain in {scene['name']} at {token}")
        ordered.append(token)
        seen.add(token)
        token = row["next"]
    if len(ordered) != scene["nbr_samples"] or not ordered or ordered[-1] != scene["last_sample_token"]:
        raise ValueError(f"Incomplete keyframe list in {scene['name']}")
    return ordered


def _vector(values, size: int, name: str) -> tuple[float, ...]:
    if not isinstance(values, list) or len(values) != size:
        raise ValueError(f"{name} must contain {size} numbers")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"Non-finite value in {name}")
    return result


def _quaternion(values, name: str) -> Quaternion:
    q = _vector(values, 4, name)
    norm = math.hypot(*q)
    if not math.isclose(norm, 1.0, abs_tol=0.001):
        raise ValueError(f"Non-unit quaternion in {name}")
    return tuple(value / norm for value in q)


def _rotation_matrix(q: Quaternion) -> tuple[float, ...]:
    w, x, y, z = q
    return (
        1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y),
        2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x),
        2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y),
    )


def _pose(row: dict, name: str) -> tuple[Quaternion, Vec3]:
    return _quaternion(row["rotation"], name), _vector(row["translation"], 3, name)


def _blob(root: Path, filename: str) -> Path:
    relative = Path(filename)
    if relative.is_absolute() or relative.drive or ".." in relative.parts:
        raise ValueError(f"Unsafe nuScenes blob path {filename!r}")
    path = root / relative
    if any(value in str(path) for value in "\t\r\n"):
        raise ValueError(f"Tabs and line breaks are not supported in the manifest path {path}")
    if not path.is_file():
        raise ValueError(f"Missing nuScenes data file: {path}")
    return path


def _load_annotations(
    metadata: Path, selected_samples: set[str], categories: dict,
) -> dict[str, tuple[Annotation, ...]]:
    instances = _index(_records(metadata / "instance.json"), "token", "instance")
    visibility = _index(_records(metadata / "visibility.json"), "token", "visibility")
    groups = defaultdict(list)
    for row in _records(metadata / "sample_annotation.json"):
        if row["sample_token"] not in selected_samples:
            continue
        rotation, translation = _pose(row, f"annotation {row['token']}")
        size = _vector(row["size"], 3, f"annotation size {row['token']}")
        if min(size) <= 0:
            raise ValueError(f"Non-positive box size in annotation {row['token']}")
        instance = instances[row["instance_token"]]
        category = categories[instance["category_token"]]["name"]
        visibility_token = row["visibility_token"]
        groups[row["sample_token"]].append(Annotation(
            translation, size, rotation, category, row["instance_token"],
            int(row["num_lidar_pts"]), int(row["num_radar_pts"]), visibility_token,
            visibility[visibility_token]["level"] if visibility_token else "",
        ))
    if not groups:
        raise ValueError("The selected samples have no public 3D annotation records")
    return {sample: tuple(rows) for sample, rows in groups.items()}


def _lidar_rows(
    metadata: Path, samples_by_scene: dict[str, list[str]], keyframes_only: bool,
) -> tuple[list[tuple[str, dict]], dict]:
    sensors = _index(_records(metadata / "sensor.json"), "token", "sensor")
    calib = _index((
        row for row in _records(metadata / "calibrated_sensor.json")
        if sensors[row["sensor_token"]]["channel"] == "LIDAR_TOP"
    ), "token", "LIDAR_TOP calibration")
    sample_to_scene = {sample: scene for scene, samples in samples_by_scene.items() for sample in samples}
    rows_by_scene = defaultdict(list)
    for row in _records(metadata / "sample_data.json"):
        if row["sample_token"] in sample_to_scene and row["calibrated_sensor_token"] in calib:
            if row["is_key_frame"] or not keyframes_only:
                rows_by_scene[sample_to_scene[row["sample_token"]]].append(row)
    result = []
    for scene, samples in samples_by_scene.items():
        rows = sorted(rows_by_scene[scene], key=lambda row: row["timestamp"])
        key_indices = [i for i, row in enumerate(rows) if row["is_key_frame"]]
        if [rows[i]["sample_token"] for i in key_indices] != samples:
            raise ValueError(f"Missing, repeated or out-of-order LIDAR_TOP keyframes in {scene}")
        rows = rows[key_indices[0]:key_indices[-1] + 1]
        for earlier, later in zip(rows, rows[1:]):
            if earlier["timestamp"] >= later["timestamp"]:
                raise ValueError(f"Non-increasing LIDAR_TOP sensor timestamps in {scene}")
            if not keyframes_only and (
                earlier["next"] != later["token"] or later["prev"] != earlier["token"]
            ):
                raise ValueError(f"Missing linked LIDAR_TOP sweep in {scene} at {later['token']}")
        result.extend((scene, row) for row in rows)
    return result, calib


def _lidarseg_categories(categories: dict) -> dict[int, str]:
    indexed = {}
    names = set()
    for row in categories.values():
        index, name = row["index"], row["name"]
        if type(index) is not int or not 0 <= index <= 255 or index in indexed or name in names:
            raise ValueError(f"Invalid or repeated LiDARSeg category {index!r}, {name!r}")
        indexed[index] = name
        names.add(name)
    return indexed


def _make_frames(
    root: Path, metadata: Path, samples_by_scene: dict, keyframes_only: bool,
) -> tuple[tuple[Frame, ...], dict[int, str]]:
    rows, calib = _lidar_rows(metadata, samples_by_scene, keyframes_only)
    sensor_poses = {token: _pose(row, f"calibration {token}") for token, row in calib.items()}
    ego_needed = {row["ego_pose_token"] for _, row in rows}
    ego_poses = _index((
        row for row in _records(metadata / "ego_pose.json") if row["token"] in ego_needed
    ), "token", "ego_pose")
    selected_samples = {row["sample_token"] for _, row in rows if row["is_key_frame"]}
    categories = _index(_records(metadata / "category.json"), "token", "category")
    annotations = _load_annotations(metadata, selected_samples, categories)
    label_table = metadata / "lidarseg.json"
    label_index = None
    lidarseg_categories = {}
    if label_table.exists():
        lidarseg_categories = _lidarseg_categories(categories)
        keyframes = {row["token"] for _, row in rows if row["is_key_frame"]}
        label_index = _index((
            row for row in _records(label_table) if row["sample_data_token"] in keyframes
        ), "sample_data_token", "lidarseg")
    frames = []
    seen = set()
    for scene, row in rows:
        token = row["token"]
        if not FRAME_TOKEN.fullmatch(token) or token in seen:
            raise ValueError(f"Invalid or duplicate LIDAR_TOP token {token!r}")
        seen.add(token)
        if type(row["timestamp"]) is not int or row["timestamp"] <= 0:
            raise ValueError(f"Invalid LIDAR_TOP µs timestamp in {token}")
        cloud = _blob(root, row["filename"])
        point_count, remainder = divmod(cloud.stat().st_size, SCAN_POINT_BYTES)
        if not point_count or remainder:
            raise ValueError(f"Invalid nuScenes 5-float point records: {cloud}")
        labels = None
        if row["is_key_frame"] and label_index is not None:
            if token not in label_index:
                raise ValueError(f"LiDARSeg table has no ground truth for {scene} keyframe {token}")
            labels = _blob(root, label_index[token]["filename"])
            if labels.stat().st_size != point_count:
                raise ValueError(f"LiDARSeg label count differs from {point_count} points in {cloud}")
        ego_rotation, ego_translation = _pose(ego_poses[row["ego_pose_token"]], f"ego pose {token}")
        sensor_rotation, sensor_translation = sensor_poses[row["calibrated_sensor_token"]]
        frames.append(Frame(
            scene, token, row["sample_token"], row["timestamp"] * 1000,
            row["is_key_frame"], cloud, labels, point_count, ego_rotation, ego_translation,
            sensor_rotation, sensor_translation,
            annotations.get(row["sample_token"], ()) if row["is_key_frame"] else (),
        ))
    return tuple(frames), lidarseg_categories


def prepare(
    dataset_root: Path, version: str, split: str, manifest: Path,
    scenes: tuple[str, ...] = (), max_keyframes: int | None = None, keyframes_only: bool = False,
) -> BenchmarkDataset:
    """Validate and write a scan manifest. Prediction code never reads labels."""
    allowed = set(_scene_names(version, split))
    if not set(scenes) <= allowed:
        raise ValueError(f"Scenes outside {split}: {', '.join(sorted(set(scenes)-allowed))}")
    if max_keyframes is not None and max_keyframes < 1:
        raise ValueError("max_keyframes must be positive")
    root = Path(dataset_root).resolve()
    metadata = root / version
    try:
        available = _index(_records(metadata / "scene.json"), "name", "scene")
        names = sorted(set(scenes) or allowed)
        missing = set(names) - available.keys()
        if missing:
            raise ValueError(f"Missing {split} scene metadata: {', '.join(sorted(missing))}")
        scene_tokens = {available[name]["token"] for name in names}
        samples = _index((
            row for row in _records(metadata / "sample.json") if row["scene_token"] in scene_tokens
        ), "token", "sample")
        samples_by_scene = {}
        remaining = max_keyframes
        for name in names:
            selected = _sample_chain(available[name], samples)[:remaining]
            samples_by_scene[name] = selected
            if remaining is not None:
                remaining -= len(selected)
                if not remaining:
                    break
        frames, lidarseg_categories = _make_frames(root, metadata, samples_by_scene, keyframes_only)
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Invalid or incomplete nuScenes metadata under {metadata}: {exc}") from exc
    with Path(manifest).open("w", encoding="utf-8", newline="") as output:
        output.write("\t".join(MANIFEST_COLUMNS) + "\n")
        for frame in frames:
            fields = (frame.scene, frame.frame, frame.timestamp_ns, int(frame.keyframe), frame.cloud)
            transform = (*_rotation_matrix(frame.sensor_rotation), *frame.sensor_translation)
            output.write("\t".join(map(str, (*fields, *transform))) + "\n")
    keyframes = tuple(row for row in frames if row.keyframe)
    return BenchmarkDataset(root, version, split, frames, keyframes, lidarseg_categories)
