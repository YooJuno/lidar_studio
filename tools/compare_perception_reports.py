#!/usr/bin/env python3
"""Compare two LiDAR Studio benchmark reports and optionally enforce regression limits."""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys


FORMAT = "lidar-studio-perception-benchmark"
RowKey = tuple[str, str, str]
BreakdownKey = tuple[str, str, str, str, str]
SCOPES = ("Overall", "Scene", "Class")


@dataclass(frozen=True)
class Row:
    score: float | None
    p95: float | None
    backend: str
    details: str


@dataclass(frozen=True)
class Report:
    selection: str
    protocol: str
    host: str | None
    rows: dict[RowKey, Row]
    breakdowns: dict[BreakdownKey, float | None]


def fingerprint(report, section, field, *, required=True):
    table = report.get(section, {})
    if not isinstance(table, dict):
        raise ValueError(f"{section} must be an object")
    value = table.get(field)
    if value is None and not required:
        return None
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise ValueError(f"Missing or invalid {section}.{field}; use a fresh benchmark report")
    return value


def measurement(value, name, upper=math.inf):
    if value is None:
        return None
    try:
        if type(value) in (int, float) and math.isfinite(value) and 0 <= value <= upper:
            return float(value)
    except OverflowError:
        pass
    raise ValueError(f"Invalid {name}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Repeated JSON key {key}")
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError(f"Invalid JSON constant {value}")


def breakdown_measurements(document):
    breakdowns = document.get("breakdowns", [])
    if not isinstance(breakdowns, list):
        raise ValueError("Expected benchmark breakdown rows")
    indexed = {}
    for row in breakdowns:
        if not isinstance(row, dict):
            raise ValueError("Expected a breakdown row object")
        key = tuple(row.get(name) for name in ("stage", "method", "scope", "label"))
        metrics = row.get("metrics")
        if (not all(isinstance(part, str) and part.strip() for part in key)
                or key[2] not in SCOPES or not isinstance(metrics, dict)):
            raise ValueError("Invalid breakdown stage, method, scope, label or metrics")
        for metric, value in metrics.items():
            index = (*key, metric)
            if not metric.strip() or index in indexed:
                raise ValueError(f"Invalid or repeated breakdown metric {index}")
            indexed[index] = measurement(value, "breakdown metric", 1)
    return indexed


def read_report(path):
    try:
        with path.open(encoding="utf-8") as stream:
            document = json.load(stream, object_pairs_hook=unique_object, parse_constant=invalid_constant)
        if (not isinstance(document, dict) or document.get("format") != FORMAT
                or type(document.get("schema")) is not int or document["schema"] != 1):
            raise ValueError("Expected a schema 1 LiDAR Studio benchmark report")
        selection = fingerprint(document, "dataset", "selection_sha256")
        protocol = fingerprint(document, "settings", "comparison_sha256")
        host = fingerprint(document, "environment", "timing_host_sha256", required=False)
        rows = document.get("rows")
        if not isinstance(rows, list) or not rows:
            raise ValueError("Expected benchmark result rows")
        indexed = {}
        for entry in rows:
            if not isinstance(entry, dict):
                raise ValueError("Expected a result row object")
            stage, method, metric = (entry.get(field) for field in ("stage", "method", "metric"))
            if not all(isinstance(part, str) and part.strip() for part in (stage, method)) or not isinstance(metric, str):
                raise ValueError("Invalid stage, method or metric")
            key = (stage, method, metric)
            if key in indexed:
                raise ValueError(f"Repeated {stage} / {method} / {metric} row")
            score = measurement(entry.get("value"), "score", 1)
            p95 = measurement(entry.get("p95_ms"), "p95 timing")
            backend = entry.get("backend", "")
            details = entry.get("details", "")
            if not isinstance(backend, str) or not isinstance(details, str):
                raise ValueError("Invalid result backend or details")
            if not metric.strip() and (score is not None or p95 is not None):
                raise ValueError("Measured row is missing its metric name")
            indexed[key] = Row(score, p95, backend, details)
        return Report(selection, protocol, host, indexed, breakdown_measurements(document))
    except (OSError, ValueError) as error:
        raise ValueError(f"{path}: {error}") from error


def compare_row(key, before, after, baseline_host, candidate_host):
    stage, method, metric = key
    row = {
        "stage": stage, "method": method, "metric": metric,
        "status": "matched",
        "baseline_score": before.score if before else None,
        "candidate_score": after.score if after else None,
        "score_delta_pp": None,
        "baseline_p95_ms": before.p95 if before else None,
        "candidate_p95_ms": after.p95 if after else None,
        "p95_delta_ms": None,
        "p95_rise_percent": None,
        "timing_status": "missing_measurement",
        "baseline_backend": before.backend if before else None,
        "candidate_backend": after.backend if after else None,
    }
    if before is None or after is None:
        row["status"] = "missing_baseline" if before is None else "missing_candidate"
        if not metric:
            row["details"] = (before or after).details
        return row
    if before.score is not None and after.score is not None:
        row["score_delta_pp"] = (after.score - before.score) * 100
    if not metric:
        row["details"] = f"Baseline: {before.details} | Candidate: {after.details}"
    if before.p95 is None or after.p95 is None:
        return row
    if not baseline_host or not candidate_host:
        row["timing_status"] = "missing_host_fingerprint"
    elif baseline_host != candidate_host:
        row["timing_status"] = "host_mismatch"
    elif not before.backend or not after.backend:
        row["timing_status"] = "missing_backend"
    elif before.backend != after.backend:
        row["timing_status"] = "backend_mismatch"
    else:
        row["p95_delta_ms"] = after.p95 - before.p95
        relative = (after.p95 / before.p95 - 1) * 100 if before.p95 else None
        if relative is not None and math.isfinite(relative):
            row["p95_rise_percent"] = relative
            row["timing_status"] = "comparable"
        else:
            row["timing_status"] = "undefined_relative_rise"
    return row


def compare(baseline, candidate):
    if baseline.selection != candidate.selection:
        raise ValueError("Dataset selection fingerprints differ. Use the same dataset, frames, calibration and labels.")
    if baseline.protocol != candidate.protocol:
        raise ValueError("Scoring protocol fingerprints differ. Use the same evaluation settings.")
    return {
        key: compare_row(key, baseline.rows.get(key), candidate.rows.get(key), baseline.host, candidate.host)
        for key in sorted(baseline.rows.keys() | candidate.rows.keys())
    }


def parse_gates(args):
    gates = {}
    for flag, name, entries in (
        ("--max-score-drop", "score_drop_pp", args.max_score_drop),
        ("--max-p95-rise", "p95_rise_percent", args.max_p95_rise),
        ("--min-breakdown", "minimum_percent", args.min_breakdown),
        ("--max-breakdown", "maximum_percent", args.max_breakdown),
    ):
        for *parts, limit in entries:
            key = tuple(parts)
            if not all(part.strip() for part in key):
                raise ValueError(f"{flag} requires nonempty row identifiers")
            is_breakdown = len(key) == 5
            if is_breakdown and key[2] not in SCOPES:
                raise ValueError(f"{flag} scope must be Overall, Scene or Class")
            try:
                threshold = measurement(float(limit), flag, 100 if is_breakdown else math.inf)
            except ValueError as error:
                bound = "[0, 100]" if is_breakdown else "finite and nonnegative"
                raise ValueError(f"{flag} limit must be {bound}") from error
            if (key, name) in gates:
                raise ValueError(f"Repeated {flag} for {' / '.join(key)}")
            gates[key, name] = threshold
    return gates


def evaluate_gates(rows, candidate, limits):
    evaluated = []
    for (key, kind), threshold in limits.items():
        description = {"stage": key[0], "method": key[1], "metric": key[-1]}
        if len(key) == 5:
            description.update(scope=key[2], label=key[3])
            value = candidate.breakdowns.get(key)
            measured = None if value is None else value * 100
            detail = "Candidate breakdown metric is missing or undefined."
        else:
            row = rows.get(key)
            if row is None or row["status"] != "matched":
                measured, detail = None, "Row is missing from at least one report."
            elif kind == "score_drop_pp":
                delta = row["score_delta_pp"]
                measured, detail = -delta if delta is not None else None, "Score is missing."
            else:
                measured, detail = row["p95_rise_percent"], row["timing_status"]
        status = "unavailable"
        if measured is not None:
            violates = measured < threshold if kind == "minimum_percent" else measured > threshold
            at_limit = math.isclose(measured, threshold, rel_tol=1e-12, abs_tol=1e-10)
            status = "failed" if violates and not at_limit else "passed"
            detail = ""
        evaluated.append({**description, "gate": kind, "limit": threshold, "measured": measured,
                          "status": status, "detail": detail})
    return evaluated


def print_text(result):
    print(f"Comparison {result['status']}")
    for error in result.get("errors", []):
        print(error, file=sys.stderr)
    for row in result["rows"]:
        label = f"{row['stage']} / {row['method']} / {row['metric'] or 'unavailable'}"
        if row["status"] != "matched":
            print(f"{label}    {row['status'].replace('_', ' ')}")
        else:
            score = row["score_delta_pp"]
            timing = row["p95_delta_ms"]
            relative = row["p95_rise_percent"]
            score_text = "score unavailable" if score is None else f"score {score:+.3f} pp"
            timing_text = (f"p95 {row['timing_status'].replace('_', ' ')}" if timing is None else f"p95 {timing:+.3f} ms")
            if relative is not None:
                timing_text += f" ({relative:+.2f}%)"
            print(f"{label}    {score_text}    {timing_text}")
        if row.get("details"):
            print(f"  {row['details']}")
    for gate in result["gates"]:
        label = " / ".join(gate[key] for key in ("stage", "method", "scope", "label", "metric") if key in gate)
        units = "pp" if gate["gate"] == "score_drop_pp" else "%"
        limit = "min" if gate["gate"] == "minimum_percent" else "max"
        measured = gate["measured"]
        measurement_text = gate["detail"] if measured is None else f"{measured:.4g} {units} ({limit} {gate['limit']:g} {units})"
        print(f"{gate['status'].upper()}    {label}    {gate['gate'].replace('_', ' ')}    {measurement_text}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path, help="Baseline benchmark.json")
    parser.add_argument("candidate", type=Path, help="Candidate benchmark.json")
    parser.add_argument("--json", action="store_true", help="Write a machine-readable comparison to stdout")
    parser.add_argument("--max-score-drop", nargs=4, action="append", default=[],
                        metavar=("STAGE", "METHOD", "METRIC", "PP"),
                        help="Gate the maximum score drop in absolute percentage points. Repeat for additional rows.")
    parser.add_argument("--max-p95-rise", nargs=4, action="append", default=[],
                        metavar=("STAGE", "METHOD", "METRIC", "PERCENT"),
                        help="Gate the maximum p95 latency rise as a percent of baseline. Repeat for additional rows.")
    for flag, limit in (("--min-breakdown", "minimum"), ("--max-breakdown", "maximum")):
        parser.add_argument(flag, nargs=6, action="append", default=[],
                            metavar=("STAGE", "METHOD", "SCOPE", "LABEL", "METRIC", "PERCENT"),
                            help=f"Gate a {limit} candidate breakdown percentage. Scope is Overall, Scene or Class.")
    parser.epilog = "Exit 0 for a valid passing comparison, 1 for exceeded limits, 2 for invalid reports or unavailable gates."
    args = parser.parse_args(argv)
    result = {
        "format": "lidar-studio-perception-comparison", "schema": 1, "status": "compared",
        "baseline": str(args.baseline), "candidate": str(args.candidate), "rows": [], "gates": [], "errors": [],
    }
    try:
        baseline, candidate = (read_report(path) for path in (args.baseline, args.candidate))
        rows = compare(baseline, candidate)
        result["selection_sha256"] = baseline.selection
        result["comparison_sha256"] = baseline.protocol
        result["rows"] = list(rows.values())
        result["gates"] = evaluate_gates(rows, candidate, parse_gates(args))
        if not any(row["score_delta_pp"] is not None or row["p95_delta_ms"] is not None for row in rows.values()):
            raise ValueError("The reports have no comparable score or timing rows.")
        if any(gate["status"] == "unavailable" for gate in result["gates"]):
            raise ValueError("At least one requested regression gate could not be evaluated.")
        if any(gate["status"] == "failed" for gate in result["gates"]):
            result["status"] = "regressed"
        elif result["gates"]:
            result["status"] = "passed"
    except ValueError as error:
        result["status"] = "invalid"
        result["errors"].append(str(error))
    if args.json:
        print(json.dumps(result, ensure_ascii=True, allow_nan=False, indent=2))
    else:
        print_text(result)
    return {"compared": 0, "passed": 0, "regressed": 1, "invalid": 2}[result["status"]]


if __name__ == "__main__":
    sys.exit(main())
