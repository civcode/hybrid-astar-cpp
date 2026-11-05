#!/usr/bin/env python3
"""Manually compare weighted grid-only versus combined articulated A*.

Run inside the project's Docker test image. No network, packages, scheduling,
or GitHub Actions are required by this script.
"""

import argparse
import csv
import io
import math
import statistics
import subprocess
import sys

STRATEGIES = ("grid-weighted", "weighted")
DEFAULT_WEIGHTS = "1.0,1.1,1.2,1.3,1.4,1.5"


def parse_weights(raw):
    try:
        weights = [float(token.strip()) for token in raw.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("weights must be comma-separated numbers") from exc
    if not weights or any(not math.isfinite(w) or not (0.0 < w <= 3.0) for w in weights):
        raise argparse.ArgumentTypeError("each heuristic weight must be finite and in (0, 3]")
    if len(set(weights)) != len(weights):
        raise argparse.ArgumentTypeError("duplicate weights are not supported")
    return weights


def run_once(binary, mode, weight):
    command = [binary, mode, format(weight, ".12g")]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode not in (0, 2):
        raise RuntimeError(
            f"benchmark failed (exit {completed.returncode}): {command!r}\n"
            f"stdout: {completed.stdout}\nstderr: {completed.stderr}"
        )
    rows = list(csv.DictReader(io.StringIO(completed.stdout)))
    if len(rows) != 1:
        raise RuntimeError(f"expected one CSV benchmark row, got {len(rows)}: {completed.stdout!r}")
    row = rows[0]
    if row.get("mode") != mode or not math.isclose(
        float(row["heuristic_weight"]), weight, rel_tol=0.0, abs_tol=1e-9
    ):
        raise RuntimeError(f"benchmark returned mismatched mode or weight: {row!r}")
    success = row["status"] == "0" and row["validated"] == "1"
    if (completed.returncode == 0) != success:
        raise RuntimeError(f"inconsistent benchmark status/validation/exit code: {row!r}")
    return row


def median(rows, field):
    return statistics.median(float(row[field]) for row in rows) if rows else None


def format_value(value):
    return "" if value is None else f"{value:.3f}".rstrip("0").rstrip(".")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", default="./build-docker/hybrid_astar_articulated_warehouse_bench",
                        help="benchmark executable inside the Docker container")
    parser.add_argument("--weights", type=parse_weights, default=parse_weights(DEFAULT_WEIGHTS),
                        help=f"comma-separated weights (default: {DEFAULT_WEIGHTS})")
    parser.add_argument("--repeats", type=int, default=3,
                        help="repetitions per mode and weight (default: 3)")
    parser.add_argument("--raw", action="store_true",
                        help="output per-run CSV instead of aggregated comparison CSV")
    args = parser.parse_args(argv)
    if args.repeats < 1:
        parser.error("--repeats must be >= 1")

    records = []
    for weight in args.weights:
        for mode in STRATEGIES:
            for repetition in range(1, args.repeats + 1):
                print(f"Running {mode} at weight={weight:g} ({repetition}/{args.repeats})",
                      file=sys.stderr, flush=True)
                row = run_once(args.benchmark, mode, weight)
                records.append((mode, weight, repetition, row))

    if args.raw:
        columns = ["repetition", *records[0][3].keys()]
        writer = csv.DictWriter(sys.stdout, fieldnames=columns)
        writer.writeheader()
        for _, _, repetition, row in records:
            writer.writerow({"repetition": repetition, **row})
        return 0

    columns = ["mode", "heuristic_weight", "runs", "success", "validated",
               "mean_ms", "median_ms", "min_ms", "max_ms", "median_expanded",
               "median_body_checks", "median_drive_m", "median_plan_cost_m",
               "median_stationary_s", "median_coupled_s"]
    writer = csv.DictWriter(sys.stdout, fieldnames=columns)
    writer.writeheader()
    for weight in args.weights:
        for mode in STRATEGIES:
            rows = [row for m, w, _, row in records if m == mode and w == weight]
            valid = [row for row in rows if row["status"] == "0" and row["validated"] == "1"]
            times = [float(row["elapsed_ms"]) for row in rows]
            writer.writerow({
                "mode": mode,
                "heuristic_weight": format(weight, ".12g"),
                "runs": len(rows),
                "success": sum(row["status"] == "0" for row in rows),
                "validated": len(valid),
                "mean_ms": format_value(statistics.mean(times)),
                "median_ms": format_value(statistics.median(times)),
                "min_ms": format_value(min(times)),
                "max_ms": format_value(max(times)),
                "median_expanded": format_value(median(rows, "expanded")),
                "median_body_checks": format_value(median(rows, "search_body_checks")),
                "median_drive_m": format_value(median(valid, "drive_m")),
                "median_plan_cost_m": format_value(median(valid, "plan_cost_m")),
                "median_stationary_s": format_value(median(valid, "stationary_s")),
                "median_coupled_s": format_value(median(valid, "coupled_s")),
            })
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
