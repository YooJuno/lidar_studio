#!/usr/bin/env python3
"""A stable track crossing must not be scored as an ID switch."""

from collections import Counter
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from nuscenes_benchmark_metrics import (
    Candidate, GroundTruth, _identity_true_positives, _track_distances, _track_matches, _tracking_summary,
)


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
