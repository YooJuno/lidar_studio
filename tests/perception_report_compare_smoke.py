#!/usr/bin/env python3
"""Check benchmark compatibility and regression gates using local reports."""

from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory


TOOL = Path(__file__).resolve().parents[1] / "tools" / "compare_perception_reports.py"
GROUND = ("Ground", "fugseg", "Ground IoU")
LOSS = ("Ground", "fugseg", "Overall", "All", "Pedestrian point loss")
RECALL = ("Detection", "tiny", "Class", "Pedestrian", "Recall")


def compare(folder, baseline, candidate, code, *options):
    paths = [folder / name for name in ("baseline.json", "candidate.json")]
    for path, report in zip(paths, (baseline, candidate)):
        path.write_text(json.dumps(report, allow_nan=False), encoding="utf-8")
    process = subprocess.run([sys.executable, str(TOOL), *paths, "--json", *options],
                             capture_output=True, text=True, check=False)
    assert process.returncode == code, (process.returncode, process.stdout, process.stderr)
    return json.loads(process.stdout)


def main():
    baseline = {
        "format": "lidar-studio-perception-benchmark",
        "schema": 1,
        "dataset": {"selection_sha256": "a" * 64},
        "settings": {"comparison_sha256": "b" * 64},
        "environment": {"timing_host_sha256": "c" * 64},
        "rows": [{"stage": GROUND[0], "method": GROUND[1], "metric": GROUND[2],
                  "value": .81, "p95_ms": 10, "backend": "CPU"}],
        "breakdowns": [
            {"stage": LOSS[0], "method": LOSS[1], "scope": LOSS[2], "label": LOSS[3],
             "support": 488, "metrics": {LOSS[4]: .02}},
            {"stage": RECALL[0], "method": RECALL[1], "scope": RECALL[2], "label": RECALL[3],
             "support": 73, "metrics": {RECALL[4]: .80}},
        ],
    }
    candidate = deepcopy(baseline)
    candidate["rows"][0].update(value=.78, p95_ms=12)
    with TemporaryDirectory(prefix="lidar-compare-reports-") as temporary:
        folder = Path(temporary)
        accepted = compare(folder, baseline, candidate, 0,
                           "--max-score-drop", *GROUND, "3", "--max-p95-rise", *GROUND, "20",
                           "--max-breakdown", *LOSS, "2", "--min-breakdown", *RECALL, "80")
        row = accepted["rows"][0]
        assert accepted["status"] == "passed" and len(accepted["gates"]) == 4
        assert all(gate["status"] == "passed" for gate in accepted["gates"])
        assert math.isclose(row["score_delta_pp"], -3) and math.isclose(row["p95_rise_percent"], 20)

        failed = compare(folder, baseline, candidate, 1,
                         "--max-score-drop", *GROUND, "2.9", "--max-p95-rise", *GROUND, "19.9",
                         "--max-breakdown", *LOSS, "1.9", "--min-breakdown", *RECALL, "80.1")
        assert failed["status"] == "regressed" and all(gate["status"] == "failed" for gate in failed["gates"])

        for section, fingerprint in (("dataset", "selection_sha256"), ("settings", "comparison_sha256")):
            incompatible = deepcopy(candidate)
            incompatible[section][fingerprint] = "d" * 64
            invalid = compare(folder, baseline, incompatible, 2)
            assert invalid["status"] == "invalid" and invalid["rows"] == []
            del incompatible[section][fingerprint]
            assert "Missing or invalid" in compare(folder, baseline, incompatible, 2)["errors"][0]

        wrong_host = deepcopy(candidate)
        wrong_host["environment"]["timing_host_sha256"] = "d" * 64
        other_backend = deepcopy(candidate)
        other_backend["rows"][0]["backend"] = "CUDA"
        unknown_host = deepcopy(candidate)
        del unknown_host["environment"]["timing_host_sha256"]
        for report, status in ((wrong_host, "host_mismatch"), (other_backend, "backend_mismatch"),
                               (unknown_host, "missing_host_fingerprint")):
            quality_only = compare(folder, baseline, report, 0, "--max-score-drop", *GROUND, "3")
            assert quality_only["rows"][0]["timing_status"] == status
            assert quality_only["rows"][0]["p95_delta_ms"] is None
            invalid = compare(folder, baseline, report, 2, "--max-p95-rise", *GROUND, "50")
            assert invalid["gates"][0]["status"] == "unavailable"

        incomplete = deepcopy(candidate)
        incomplete["rows"].append({**candidate["rows"][0], "method": "linefit"})
        union = compare(folder, baseline, incomplete, 0)
        assert union["rows"][-1]["status"] == "missing_baseline"
        invalid = compare(folder, baseline, incomplete, 2,
                          "--max-score-drop", "Ground", "linefit", "Ground IoU", "10")
        assert invalid["gates"][0]["status"] == "unavailable"
        incomplete["breakdowns"][0]["metrics"][LOSS[4]] = None
        assert compare(folder, baseline, incomplete, 2, "--max-breakdown", *LOSS, "10")["gates"][0]["status"] == "unavailable"

        zero = deepcopy(baseline)
        zero["rows"][0]["p95_ms"] = 0
        invalid = compare(folder, zero, candidate, 2, "--max-p95-rise", *GROUND, "100")
        assert invalid["rows"][0]["timing_status"] == "undefined_relative_rise"
    print("Perception report compatibility and regression gates passed")


if __name__ == "__main__":
    main()
