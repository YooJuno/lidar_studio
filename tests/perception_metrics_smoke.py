#!/usr/bin/env python3
"""A stable track crossing must not be scored as an ID switch."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from nuscenes_benchmark_metrics import Candidate, GroundTruth, _track_matches


targets = [GroundTruth((0, 0), "a"), GroundTruth((1, 0), "b")]
crossing = [Candidate((0.6, 0), 0.8, "1"), Candidate((0.4, 0), 0.8, "2")]
previous = {"a": "1", "b": "2"}

assert {(i, j) for i, j, _ in _track_matches(crossing, targets, previous)} == {(0, 0), (1, 1)}
assert {(i, j) for i, j, _ in _track_matches(crossing, targets, {})} == {(0, 1), (1, 0)}

replacement = [Candidate((3.1, 0), 0.8, "1"), Candidate((1, 0), 0.8, "2"), Candidate((0, 0), 0.8, "3")]
assert {(i, j) for i, j, _ in _track_matches(replacement, targets, previous)} == {(1, 1), (2, 0)}
