"""Score LiDAR Studio outputs against the selected nuScenes keyframes.

Object metrics are local comparisons, with matching rules recorded in the report.
The module accepts the ordered dataset manifest and the native worker's TSVs.
"""

from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
from itertools import groupby
import math
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment


CLASS_RANGES = {"car": 50.0, "bus": 50.0, "truck": 50.0, "pedestrian": 40.0, "bicycle": 40.0}
NATIVE_CLASSES = tuple(CLASS_RANGES)
ANNOTATION_CLASSES = {
    "vehicle.car": "car",
    "vehicle.bus.bendy": "bus",
    "vehicle.bus.rigid": "bus",
    "vehicle.truck": "truck",
    "vehicle.bicycle": "bicycle",
    "human.pedestrian.adult": "pedestrian",
    "human.pedestrian.child": "pedestrian",
    "human.pedestrian.construction_worker": "pedestrian",
    "human.pedestrian.police_officer": "pedestrian",
}
GROUND_CLASSES = ("flat.driveable_surface", "flat.sidewalk", "flat.terrain")
IGNORED_CLASSES = ("noise", "flat.other", "vehicle.ego")
AP_DISTANCES_M = (0.5, 1.0, 2.0, 4.0)
CENTER_GATE_M = 2.0
CANDIDATE_MIN = 0.10
TIMING_WARMUP_SCANS = 3
TIMING_COLUMNS = ("scene", "frame", "point_count", "ground_ms", "detector_ms", "provider")
BOX_COLUMNS = ("class_id", "score", "x", "y", "z", "length", "width", "height", "yaw")
GROUND_COLUMNS = (
    "true_ground", "false_ground", "missed_ground", "true_non_ground", "ignored_points",
    "ambiguous_prediction_points", "pedestrian_points", "removed_pedestrian_points",
    "vehicle_points", "removed_vehicle_points",
)


def _ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def _read_rows(path, columns):
    with path.open(encoding="utf-8", newline="") as source:
        rows = csv.DictReader(source, delimiter="\t")
        if rows.fieldnames != list(columns):
            raise ValueError(f"Unexpected TSV schema in {path}")
        for row in rows:
            if None in row or any(value is None for value in row.values()):
                raise ValueError(f"Invalid row {rows.line_num} in {path}")
            yield row


def _write_rows(path, columns, rows):
    with path.open("w", encoding="utf-8", newline="") as target:
        writer = csv.writer(target, delimiter="\t", lineterminator="\n")
        writer.writerow(columns)
        writer.writerows(rows)


def _nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"Expected a finite nonnegative value, received {value!r}")
    return number


def _timing_summary(values):
    if not values:
        raise ValueError("No timing samples")
    warmup = TIMING_WARMUP_SCANS if len(values) > TIMING_WARMUP_SCANS else 0
    timed = values[warmup:]
    p50, p95, p99 = np.percentile(timed, [50, 95, 99])
    return {
        "count": len(timed),
        "first_scan_ms": values[0],
        "warmup_scans_excluded": warmup,
        "mean_ms": float(np.mean(timed)),
        "p50_ms": float(p50),
        "p95_ms": float(p95),
        "p99_ms": float(p99),
        "max_ms": max(timed),
    }


def _require_frame(frames, row):
    frame = frames.get(row["frame"])
    if frame is None or frame.scene != row["scene"]:
        raise ValueError(f"Unexpected frame {row['scene']}/{row['frame']}")
    return frame


def _stage_timing(dataset, run_dir, column):
    frames = {frame.frame: frame for frame in dataset.frames}
    times = {}
    providers = set()
    for row in _read_rows(run_dir / "timings.tsv", TIMING_COLUMNS):
        frame = _require_frame(frames, row)
        if frame.frame in times or int(row["point_count"]) != frame.point_count:
            raise ValueError(f"Duplicate timing or point count mismatch for {frame.frame}")
        times[frame.frame] = _nonnegative(row[column])
        if row["provider"]:
            providers.add(row["provider"])
    if times.keys() != frames.keys():
        raise ValueError(f"Missing {len(frames.keys() - times.keys())} frame timings for {column}")
    summary = _timing_summary([times[frame.frame] for frame in dataset.frames])
    summary["providers"] = sorted(providers)
    return summary


def _ground_summary(counts):
    good = counts["true_ground"]
    lost = counts["false_ground"]
    missed = counts["missed_ground"]
    return {
        **{key: counts[key] for key in GROUND_COLUMNS},
        "iou": _ratio(good, good + lost + missed),
        "precision": _ratio(good, good + lost),
        "recall": _ratio(good, good + missed),
        "lost_non_ground_rate": _ratio(lost, lost + counts["true_non_ground"]),
    }


def evaluate_ground(dataset, run_dir):
    run_dir = Path(run_dir)
    timing = _stage_timing(dataset, run_dir, "ground_ms")
    classes = dataset.lidarseg_categories
    missing = set(GROUND_CLASSES + IGNORED_CLASSES) - set(classes.values())
    if missing:
        raise ValueError(f"Dataset label mapping is missing {', '.join(sorted(missing))}")
    if not all(0 <= index <= 255 for index in classes):
        raise ValueError("LiDARSeg mapping must use byte values")
    known = np.zeros(256, dtype=bool)
    known[list(classes)] = True

    def make_mask(predicate):
        table = np.zeros(256, dtype=bool)
        for index, name in classes.items():
            table[index] = predicate(name)
        return table

    gt_ground = make_mask(lambda name: name in GROUND_CLASSES)
    ignored = make_mask(lambda name: name in IGNORED_CLASSES)
    pedestrians = make_mask(lambda name: name.startswith("human.pedestrian."))
    vehicles = make_mask(lambda name: name.startswith("vehicle.") and name != "vehicle.ego")
    per_scene = defaultdict(Counter)
    rows = []
    for frame in dataset.keyframes:
        if frame.labels is None:
            raise ValueError(f"Missing LiDARSeg labels for {frame.scene}/{frame.frame}")
        labels = np.fromfile(frame.labels, dtype=np.uint8)
        prediction = np.fromfile(run_dir / "masks" / f"{frame.frame}.bin", dtype=np.uint8)
        if len(labels) != frame.point_count or len(prediction) != frame.point_count:
            raise ValueError(f"Point, label or prediction counts disagree for {frame.frame}")
        if not known[labels].all() or (prediction > 2).any():
            raise ValueError(f"Unknown semantic class or invalid prediction mask for {frame.frame}")

        ambiguous = prediction == 2
        valid = ~(ignored[labels] | ambiguous)
        ground = gt_ground[labels] & valid
        obstacles = valid & ~ground
        predicted = prediction == 1
        pedestrian = pedestrians[labels] & valid
        vehicle = vehicles[labels] & valid
        masks = (
            ground & predicted, obstacles & predicted, ground & ~predicted, obstacles & ~predicted,
            ~valid, ambiguous, pedestrian, pedestrian & predicted, vehicle, vehicle & predicted,
        )
        counts = Counter({key: int(np.count_nonzero(mask)) for key, mask in zip(GROUND_COLUMNS, masks)})
        per_scene[frame.scene].update(counts)
        rows.append((frame.scene, frame.frame, *(counts[key] for key in GROUND_COLUMNS)))

    _write_rows(run_dir / "ground-errors.tsv", ("scene", "frame", *GROUND_COLUMNS), rows)
    return {
        "ground_classes": list(GROUND_CLASSES),
        "ignored_classes": list(IGNORED_CLASSES),
        "overall": _ground_summary(sum(per_scene.values(), Counter())),
        "scenes": {scene: _ground_summary(counts) for scene, counts in per_scene.items()},
        "timing": timing,
    }


@dataclass(frozen=True)
class Candidate:
    location: tuple[float, float]
    score: float
    identity: str = ""
    coasting: bool = False


@dataclass(frozen=True)
class GroundTruth:
    location: tuple[float, float]
    identity: str


def _ego_box_center(frame, x, y, z):
    # Worker rotates LiDAR axes into ego axes, applies vertical mounting
    # translation and keeps the horizontal origin at the LiDAR.
    sx, sy, _ = frame.sensor_translation
    local = np.array((x + sx, y + sy, z))
    w, *axis = frame.ego_rotation
    axis = np.array(axis)
    cross = 2 * np.cross(axis, local)
    global_position = local + w * cross + np.cross(axis, cross) + frame.ego_translation
    return tuple(global_position[:2])


def _candidate(frame, row, track=False):
    class_index = int(row["class_id"])
    if not 0 <= class_index < len(NATIVE_CLASSES):
        raise ValueError(f"Invalid native detector class {class_index}")
    category = NATIVE_CLASSES[class_index]
    score = _nonnegative(row["score"])
    x, y, z = (float(row[key]) for key in ("x", "y", "z"))
    if score > 1 or not all(map(math.isfinite, (x, y, z))):
        raise ValueError(f"Invalid box score or position for {frame.frame}")
    if track and (not row["id"] or row["predicted"] not in ("0", "1")):
        raise ValueError(f"Invalid track ID or predicted flag for {frame.frame}")
    candidate = Candidate(_ego_box_center(frame, x, y, z), score, row.get("id", ""), row.get("predicted") == "1")
    return category, candidate


def _within_range(frame, location, category):
    return math.dist(location, frame.ego_translation[:2]) < CLASS_RANGES[category]


def _read_candidates(dataset, run_dir):
    frames = {frame.frame: frame for frame in dataset.frames}
    detections = defaultdict(list)
    tracks = defaultdict(list)
    for row in _read_rows(run_dir / "detections.tsv", ("scene", "frame", *BOX_COLUMNS)):
        frame = _require_frame(frames, row)
        if frame.keyframe:
            category, candidate = _candidate(frame, row)
            if _within_range(frame, candidate.location, category):
                detections[frame.frame, category].append(candidate)
    seen = set()
    for row in _read_rows(run_dir / "tracks.tsv", ("scene", "frame", "profile", "id", *BOX_COLUMNS, "confirmed", "predicted")):
        frame = _require_frame(frames, row)
        key = (frame.frame, row["profile"], row["id"])
        if key in seen or row["confirmed"] not in ("0", "1"):
            raise ValueError(f"Duplicate track or invalid confirmed flag for {frame.frame}")
        seen.add(key)
        if frame.keyframe and row["confirmed"] == "1":
            category, candidate = _candidate(frame, row, track=True)
            if _within_range(frame, candidate.location, category):
                tracks[frame.frame, row["profile"], category].append(candidate)
    return detections, tracks


def _ground_truth(dataset):
    truth = defaultdict(list)
    for frame in dataset.keyframes:
        for annotation in frame.annotations:
            category = ANNOTATION_CLASSES.get(annotation.category_name)
            location = annotation.translation[:2]
            if category and annotation.num_lidar_pts > 0 and _within_range(frame, location, category):
                truth[frame.frame, category].append(GroundTruth(location, annotation.instance_token))
    return truth


def _greedy_matches(candidates, targets, gate):
    """Score-ranked detections claim their nearest unclaimed target."""
    available = set(range(len(targets)))
    for candidate in sorted(candidates, key=lambda box: -box.score):
        distance, nearest = min(
            ((math.dist(candidate.location, targets[index].location), index) for index in available),
            default=(math.inf, -1),
        )
        if distance < gate:
            available.remove(nearest)
            yield candidate, distance
        else:
            yield candidate, None


def _average_precision(pairs, support):
    if support == 0:
        return None
    if not pairs:
        return 0.0
    pairs = sorted(pairs, key=lambda item: -item[0])
    scores, positives = np.asarray(pairs).T
    end = np.append(np.flatnonzero(scores[:-1] != scores[1:]), len(scores) - 1)
    true_positives = positives.cumsum()[end]
    recall = true_positives / support
    precision = true_positives / (end + 1)
    envelope = np.maximum.accumulate(precision[::-1])[::-1]
    indices = np.searchsorted(recall, np.linspace(0, 1, 101))
    return float(np.append(envelope, 0.0)[indices].mean())


def _object_summary(counts):
    tp, fp, fn = (counts[key] for key in ("tp", "fp", "fn"))
    return {
        "tp": tp, "fp": fp, "fn": fn, "ground_truth": tp + fn,
        "precision": _ratio(tp, tp + fp), "recall": _ratio(tp, tp + fn),
        "mean_center_error_m": _ratio(counts["center_error_m"], tp),
    }


def _detection_summary(counts, pairs):
    summary = _object_summary(counts)
    ap = {str(gate): _average_precision(pairs[gate], summary["ground_truth"]) for gate in AP_DISTANCES_M}
    valid_ap = [value for value in ap.values() if value is not None]
    return {
        **summary,
        "ap": sum(valid_ap) / len(valid_ap) if valid_ap else None,
        "ap_by_distance_m": ap,
    }


def _mean_class_ap(classes):
    defined = [scores["ap"] for scores in classes.values() if scores["ap"] is not None]
    return sum(defined) / len(defined) if defined else None


def _score_detections(dataset, run_dir, detections, truth, score_threshold):
    classes = {}
    scene_classes = defaultdict(dict)
    scene_totals = defaultdict(Counter)
    frames_by_scene = [(scene, tuple(frames)) for scene, frames in groupby(dataset.keyframes, key=lambda f: f.scene)]
    rows = []
    negative_frames = negative_fp = 0
    for category in CLASS_RANGES:
        pairs = defaultdict(list)
        counts = Counter()
        for scene, frames in frames_by_scene:
            scene_counts = Counter()
            scene_pairs = defaultdict(list)
            for frame in frames:
                candidates = detections[frame.frame, category]
                targets = truth[frame.frame, category]
                selected = 0
                distances = []
                for gate in AP_DISTANCES_M:
                    for candidate, distance in _greedy_matches(candidates, targets, gate):
                        scene_pairs[gate].append((candidate.score, distance is not None))
                        # At the operating score, candidates are a prefix of the AP
                        # ranks and have the same greedy associations at this gate.
                        if gate == CENTER_GATE_M and candidate.score >= score_threshold:
                            selected += 1
                            if distance is not None:
                                distances.append(distance)
                tp, fp, fn = len(distances), selected - len(distances), len(targets) - len(distances)
                scene_counts.update(tp=tp, fp=fp, fn=fn, center_error_m=sum(distances))
                negative = category == "pedestrian" and not any(
                    annotation.category_name.startswith("human.pedestrian.") for annotation in frame.annotations
                )
                if negative:
                    negative_frames += 1
                    negative_fp += fp
                rows.append((scene, frame.frame, category, tp, fp, fn, int(negative)))
            counts.update(scene_counts)
            scene_totals[scene].update(scene_counts)
            scene_classes[scene][category] = _detection_summary(scene_counts, scene_pairs)
            for gate in AP_DISTANCES_M:
                pairs[gate].extend(scene_pairs[gate])
        classes[category] = _detection_summary(counts, pairs)
    _write_rows(run_dir / "object-errors.tsv", ("scene", "frame", "class", "tp", "fp", "fn", "pedestrian_negative_frame"), rows)
    mean_ap = _mean_class_ap(classes)
    return {
        "mean_ap": mean_ap,
        "overall": {**_object_summary(sum(scene_totals.values(), Counter())), "ap": mean_ap},
        "classes": classes,
        "scenes": {scene: {**_object_summary(counts), "ap": _mean_class_ap(scene_classes[scene])}
                   for scene, counts in scene_totals.items()},
        "pedestrian_negatives": {
            "keyframes": negative_frames,
            "false_positives": negative_fp,
            "false_positives_per_100_keyframes": _ratio(100 * negative_fp, negative_frames),
        },
    }


def _tracking_timing(dataset, run_dir):
    frames = {frame.frame: frame for frame in dataset.frames}
    times = defaultdict(dict)
    for row in _read_rows(run_dir / "tracking-timings.tsv", ("scene", "frame", "profile", "tracking_ms")):
        frame = _require_frame(frames, row)
        profile = row["profile"]
        if not profile or frame.frame in times[profile]:
            raise ValueError(f"Missing profile or duplicate track timing for {frame.frame}")
        times[profile][frame.frame] = _nonnegative(row["tracking_ms"])
    if not times or any(profile_times.keys() != frames.keys() for profile_times in times.values()):
        raise ValueError("Missing tracking profile or incomplete per-frame timings")
    return {profile: _timing_summary([profile_times[frame.frame] for frame in dataset.frames])
            for profile, profile_times in times.items()}


def _track_distances(candidates, targets):
    a = np.array([candidate.location for candidate in candidates])
    b = np.array([target.location for target in targets])
    return np.linalg.norm(a.reshape(-1, 1, 2) - b.reshape(1, -1, 2), axis=2)


def _track_matches(candidates, targets, previous_matches, distances):
    if distances.size == 0:
        return []
    # Keep the prior labelled frame's valid identity matches through crowded
    # crossings. A nearer centre alone does not establish an ID switch.
    candidate_ids = {candidate.identity: i for i, candidate in enumerate(candidates)}
    matches = []
    used_candidates, used_targets = set(), set()
    for j, target in enumerate(targets):
        i = candidate_ids.get(previous_matches.get(target.identity))
        if i is not None and i not in used_candidates and distances[i, j] < CENTER_GATE_M:
            matches.append((i, j, float(distances[i, j])))
            used_candidates.add(i)
            used_targets.add(j)
    remaining_candidates = [i for i in range(len(candidates)) if i not in used_candidates]
    remaining_targets = [j for j in range(len(targets)) if j not in used_targets]
    remaining_distances = distances[np.ix_(remaining_candidates, remaining_targets)]
    # Prioritize the number of in-gate matches, then minimize total distance.
    outside = (min(len(candidates), len(targets)) + 1) * CENTER_GATE_M
    rows, cols = linear_sum_assignment(np.where(remaining_distances < CENTER_GATE_M, remaining_distances, outside))
    matches.extend((remaining_candidates[i], remaining_targets[j], float(remaining_distances[i, j]))
                   for i, j in zip(rows, cols) if remaining_distances[i, j] < CENTER_GATE_M)
    return matches


def _identity_true_positives(overlap):
    """Find the best one-to-one trajectory assignment over an entire scene."""
    if not overlap:
        return 0
    track_ids = {track for track, _ in overlap}
    target_ids = {target for _, target in overlap}
    track_rows = {track: i for i, track in enumerate(track_ids)}
    target_cols = {target: j for j, target in enumerate(target_ids)}
    hits = np.zeros((len(track_ids), len(target_ids)), dtype=np.int64)
    for (track, target), count in overlap.items():
        hits[track_rows[track], target_cols[target]] = count
    rows, cols = linear_sum_assignment(hits, maximize=True)
    return int(hits[rows, cols].sum())


def _tracking_summary(counts):
    positives = counts["idtp"]
    predictions = counts["tp"] + counts["fp"]
    targets = counts["tp"] + counts["fn"]
    return {
        **_object_summary(counts),
        **{key: counts[key] for key in ("id_switches", "fragmentations", "coasting_fp")},
        "idtp": positives, "idfp": predictions - positives, "idfn": targets - positives,
        "idf1": _ratio(2 * positives, predictions + targets),
    }


def _score_tracks(dataset, run_dir, tracks, truth, score_threshold):
    timings = _tracking_timing(dataset, run_dir)
    if any(profile not in timings for _, profile, _ in tracks):
        raise ValueError("Track outputs reference an unknown profile")
    scores = {}
    rows = []
    events = []
    for profile, timing in timings.items():
        classes = {category: Counter() for category in CLASS_RANGES}
        scenes = defaultdict(Counter)
        last_match = {}
        previous_frame_matches = {}
        missed = set()
        identity_overlaps = defaultdict(Counter)
        for frame in dataset.keyframes:
            for category in CLASS_RANGES:
                candidates = [box for box in tracks[frame.frame, profile, category] if box.score >= score_threshold]
                targets = truth[frame.frame, category]
                context = (frame.scene, category)
                distances = _track_distances(candidates, targets)
                # IDF1 assigns full trajectories independently of the frame-wise matches.
                identity_overlaps[context].update((candidates[i].identity, targets[j].identity)
                                                  for i, j in zip(*np.nonzero(distances < CENTER_GATE_M)))
                matches = _track_matches(candidates, targets, previous_frame_matches.get(context, {}), distances)
                previous_frame_matches[context] = {targets[j].identity: candidates[i].identity for i, j, _ in matches}
                matched_candidates = {i for i, _, _ in matches}
                matched_targets = {j for _, j, _ in matches}
                counts = Counter(
                    tp=len(matches), fp=len(candidates) - len(matches), fn=len(targets) - len(matches),
                    center_error_m=sum(distance for _, _, distance in matches),
                    coasting_fp=sum(box.coasting for i, box in enumerate(candidates) if i not in matched_candidates),
                )
                for i, j, _ in matches:
                    target_id = targets[j].identity
                    key = (frame.scene, target_id)
                    previous_id = last_match.get(key)
                    current_id = candidates[i].identity
                    if previous_id is not None and previous_id != current_id:
                        counts["id_switches"] += 1
                        events.append((frame.scene, frame.frame, profile, category, "id_switch", target_id, previous_id, current_id))
                    if key in missed:
                        counts["fragmentations"] += 1
                        missed.remove(key)
                        events.append((frame.scene, frame.frame, profile, category, "reacquired", target_id, previous_id, current_id))
                    last_match[key] = current_id
                for j, target in enumerate(targets):
                    key = (frame.scene, target.identity)
                    if j not in matched_targets and key in last_match:
                        missed.add(key)
                classes[category].update(counts)
                scenes[frame.scene].update(counts)
                values = ("tp", "fp", "fn", "id_switches", "fragmentations", "coasting_fp")
                rows.append((frame.scene, frame.frame, profile, category, *(counts[key] for key in values)))
        for (scene, category), overlap in identity_overlaps.items():
            hits = _identity_true_positives(overlap)
            classes[category]["idtp"] += hits
            scenes[scene]["idtp"] += hits
        scores[profile] = {
            **_tracking_summary(sum(classes.values(), Counter())),
            "classes": {category: _tracking_summary(counts) for category, counts in classes.items()},
            "scenes": {scene: _tracking_summary(counts) for scene, counts in scenes.items()},
            "timing": {**timing, "providers": ["CPU"]},
        }

    _write_rows(run_dir / "tracking-errors.tsv", (
        "scene", "frame", "profile", "class", "tp", "fp", "fn", "id_switches", "fragmentations", "coasting_fp",
    ), rows)
    _write_rows(run_dir / "tracking-events.tsv", (
        "scene", "frame", "profile", "class", "event", "ground_truth_id", "previous_track_id", "track_id",
    ), events)
    return scores


def evaluate_objects(dataset, run_dir, score_threshold):
    run_dir = Path(run_dir)
    if not CANDIDATE_MIN <= score_threshold <= 1:
        raise ValueError(f"Object score threshold must be {CANDIDATE_MIN}–1")
    timing = _stage_timing(dataset, run_dir, "detector_ms")
    detections, tracks = _read_candidates(dataset, run_dir)
    truth = _ground_truth(dataset)
    return {
        "protocol": "Local LiDAR Studio metrics on nuScenes annotations",
        "case_policy": "Only keyframes; ground truth requires at least one LiDAR return; class ranges measured from ego center",
        "class_range_m": CLASS_RANGES,
        "detection_candidate_minimum_score": CANDIDATE_MIN,
        "detection_ap": "101-point interpolated AP after class-wise score-ranked nearest matching; undefined without ground truth",
        "detection_ap_match_distances_m": list(AP_DISTANCES_M),
        "operating_score_threshold": score_threshold,
        "operating_match_distance_m": CENTER_GATE_M,
        "tracking_policy": "Confirmed tracks including predictions; preserve prior valid matches, then maximize remaining in-gate matches and minimize distance",
        "identity_policy": "IDF1 from a global one-to-one track assignment within each scene and class, using the local center gate",
        "detection": _score_detections(dataset, run_dir, detections, truth, score_threshold),
        "tracking": _score_tracks(dataset, run_dir, tracks, truth, score_threshold),
        "timing": timing,
    }
