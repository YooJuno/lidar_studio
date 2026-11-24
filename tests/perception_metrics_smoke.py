#!/usr/bin/env python3
"""Check identity matching, timing samples, and benchmark selection fingerprints."""

from collections import Counter
from dataclasses import replace
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from nuscenes_benchmark_metrics import (
    Candidate, GroundTruth, _identity_true_positives, _timing_summary, _track_distances, _track_matches,
    _tracking_summary,
)
from benchmark_perception import selection_digest
from nuscenes_benchmark_data import BenchmarkDataset, Frame


targets = [GroundTruth((0, 0), "a"), GroundTruth((1, 0), "b")]
crossing = [Candidate((0.6, 0), 0.8, "1"), Candidate((0.4, 0), 0.8, "2")]
previous = {"a": "1", "b": "2"}

distances = _track_distances(crossing, targets)
assert {(i, j) for i, j, _ in _track_matches(crossing, targets, previous, distances)} == {(0, 0), (1, 1)}
assert {(i, j) for i, j, _ in _track_matches(crossing, targets, {}, distances)} == {(0, 1), (1, 0)}

replacement = [Candidate((3.1, 0), 0.8, "1"), Candidate((1, 0), 0.8, "2"), Candidate((0, 0), 0.8, "3")]
assert {(i, j) for i, j, _ in _track_matches(replacement, targets, previous, _track_distances(replacement, targets))} == {(1, 1), (2, 0)}

# Two IDs split one trajectory. Per-frame recall is perfect, but only the
# longer of the two can match the reference identity.
idtp = _identity_true_positives({("1", "person"): 3, ("2", "person"): 2})
summary = _tracking_summary(Counter(tp=5, idtp=idtp))
assert (summary["idtp"], summary["idfp"], summary["idfn"], summary["idf1"]) == (3, 2, 2, 0.6)
assert _identity_true_positives({("1", "a"): 4, ("1", "b"): 3, ("2", "a"): 3}) == 6
assert _identity_true_positives({}) == 0 and _tracking_summary(Counter())["idf1"] is None

timing = _timing_summary([100, 50, 10, 4, 6])
assert timing["first_scan_ms"] == 100 and timing["warmup_scans_excluded"] == 3
assert timing["p50_ms"] == 5 and timing["count"] == 2
assert _timing_summary([1])["p50_ms"] == 1

with TemporaryDirectory() as folder:
    root = Path(folder)
    location = (0., 0., 0.)
    rotation = (1., 0., 0., 0.)
    frame = Frame("scene-0001", "1", "2", 1, True, root / "scan.bin", root / "label.bin", 1,
                  rotation, location, rotation, location, ())
    frame.cloud.write_bytes(bytes(20))
    frame.labels.write_bytes(b"\x01")
    dataset = BenchmarkDataset(root, "v1.0-mini", "all", (frame,), (frame,), {0: "noise", 1: "terrain"})
    other_cloud = root / "other-scan.bin"
    other_cloud.write_bytes(frame.cloud.read_bytes())
    relocated = replace(frame, cloud=other_cloud)
    comparison = replace(dataset, root=root / "other-root", frames=(relocated,), keyframes=(relocated,))
    assert selection_digest(dataset, True) == selection_digest(comparison, True)
    other_cloud.write_bytes(b"\x01" + bytes(19))
    assert selection_digest(dataset, True) != selection_digest(comparison, True)
    assert selection_digest(dataset, False) == selection_digest(comparison, False)
    expected = selection_digest(dataset, False)
    frame.labels.write_bytes(b"\x00")
    assert expected != selection_digest(dataset, False)
