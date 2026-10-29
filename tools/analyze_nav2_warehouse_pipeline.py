#!/usr/bin/env python3
"""Summarize paired Nav2 warehouse margin pipeline CSV.

Uses only stdlib. One CSV row per case/margin, with p50/p95 over timed repeats.
Failed inflated plans remain included in success-rate calculations.
"""
import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


def percentile(xs, fraction):
    if not xs:
        return math.nan
    values = sorted(xs)
    return values[max(0, min(len(values) - 1, math.ceil(fraction * len(values)) - 1))]


def number(row, field):
    return float(row[field])


def summarize(rows):
    bycase = {}
    for row in rows:
        key = int(row["case_id"])
        margin = round(number(row, "margin_m"), 3)
        if margin not in (0.0, 0.1):
            raise ValueError(f"Unsupported margin {margin}; expected 0 or 0.1")
        if row["map_cells"] != "1684044" or abs(number(row, "resolution_m") - 0.03) > 1e-9:
            raise ValueError("CSV is not from the full-resolution 1006x1674 Nav2 warehouse")
        if number(row, "discovery_expansions") < 30000:
            raise ValueError("Discovered baseline path has fewer than 30k expansions")
        pair = bycase.setdefault(key, {})
        if margin in pair:
            raise ValueError(f"Duplicate case/margin: {key}/{margin}")
        pair[margin] = row
    if not bycase:
        raise ValueError("No benchmark rows")
    for key, variants in bycase.items():
        if set(variants) != {0.0, 0.1}:
            raise ValueError(f"Missing paired variant for case {key}")
        a, b = variants[0.0], variants[0.1]
        for f in ("start_x_m", "start_y_m", "start_yaw_rad", "goal_x_m",
                  "goal_y_m", "goal_yaw_rad", "draw_index", "seed", "model", "connector"):
            if a[f] != b[f]:
                raise ValueError(f"Paired route differs for case {key}, field {f}")
    print(f"Validated {len(bycase)} fully matched routes; map=1006x1674 (1,684,044 cells), "
          f"resolution=0.03m; discovery required >=30k expansions at zero margin")
    print("Note: summary timings are per-route p50s; per-route p95s are shown separately.")
    for margin in (0.0, 0.1):
        records = [pairs[margin] for _, pairs in sorted(bycase.items())]
        successes = [r for r in records if r["status"] == "Success"]
        print(f"\nMARGIN {margin:.2f} m: {len(successes)}/{len(records)} valid plans; "
              f"statuses={dict(Counter(r['status'] for r in records))}")
        for k in ("search_p50_ms", "refine_p50_ms", "total_p50_ms",
                  "search_p95_ms", "refine_p95_ms", "total_p95_ms", "expanded",
                  "grid_fallbacks", "grid_settlements", "search_pose_checks",
                  "exact_collision_pose_calls", "refine_accepted", "refine_exact_pose_checks"):
            data = [number(r, k) for r in records]
            print(f"  {k}: median={statistics.median(data):.2f} "
                  f"p95_across_paths={percentile(data, .95):.2f}")
        both = [number(r, "refined_length_m") for r in successes]
        if both:
            print(f"  refined_length_m (successful only): median={statistics.median(both):.2f}")

    pairs = [variants for _, variants in sorted(bycase.items())]
    double = [p for p in pairs if p[0.0]["status"] == p[0.1]["status"] == "Success"]
    inflated_failures = [p for p in pairs if p[0.1]["status"] != "Success"]
    print(f"\nPAIRED outcomes: both_success={len(double)}/{len(pairs)}, "
          f"inflated_failure={len(inflated_failures)}/{len(pairs)}")
    if double:
        for field in ("search_p50_ms", "refine_p50_ms", "total_p50_ms",
                      "expanded", "grid_fallbacks", "refined_length_m",
                      "refine_exact_pose_checks"):
            deltas = [number(p[0.1], field) - number(p[0.0], field) for p in double]
            print(f"  delta inflated-minus-off {field}: "
                  f"median={statistics.median(deltas):+.2f}, "
                  f"p95={percentile(deltas, .95):+.2f}, "
                  f"faster={sum(v < 0 for v in deltas)}/{len(deltas)}")

    print("\nPOTENTIAL BOTTLENECK INDICATORS (diagnostics, not causal proofs)")
    for margin in (0.0, .1):
        records = [p[margin] for p in pairs if p[margin]["status"] == "Success"]
        if not records:
            continue
        med = lambda field: statistics.median(number(r, field) for r in records)
        ratio = lambda a, b: (med(a) / med(b)) if med(b) else 0.0
        print(f"  margin {margin:.2f}: search_share_of_total ~ "
              f"{100 * ratio('search_p50_ms', 'total_p50_ms'):.1f}%; "
              f"refinement_share ~ {100 * ratio('refine_p50_ms', 'total_p50_ms'):.1f}%")
        print(f"    grid settlements median={med('grid_settlements'):.0f} "
              f"(default cap=100000); grid fallbacks median={med('grid_fallbacks'):.0f}")
        print(f"    collision pose calls median={med('exact_collision_pose_calls'):.0f}; "
              f"search motion calls median={med('search_motion_calls'):.0f}; "
              f"pruned-by-key median={med('pruned_by_key'):.0f}")
        if any(int(r["profile_enabled"]) for r in records):
            for k in ("search_motion_hotpath_ms", "exact_collision_hotpath_ms",
                      "grid_queue_hotpath_ms", "open_queue_hotpath_ms",
                      "label_lookup_hotpath_ms", "label_write_hotpath_ms",
                      "hash_hotpath_ms"):
                print(f"    {k}: median={med(k):.2f}")
    print("\nTiming caveats: search/refine wall times cover the complete computation "
          "but exclude map loading, discovery, browser and JSON serialization. "
          "Hotpath times come from the final timed repetition, include profiling overhead, "
          "and are nested (e.g. collision within motion): DO NOT ADD THEM TOGETHER.")
    print("Follow-ups: if search dominates, test 100k vs 250k vs 500k grid-settlement caps; "
          "if collision dominates, examine broad-phase disc/empty-region counters; "
          "if refinement dominates, study accepted/attempted/check budgets.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()
    with args.csv_file.open(newline="", encoding="utf-8") as f:
        summarize(list(csv.DictReader(f)))


if __name__ == "__main__":
    main()
