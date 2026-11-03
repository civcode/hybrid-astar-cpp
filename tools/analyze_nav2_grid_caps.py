#!/usr/bin/env python3
"""Compare 100k/250k/500k obstacle-grid settlement budgets on identical poses.

Inputs must be three original Nav2 warehouse benchmark CSVs generated with
--collision-mode fastpath, --replay-csv SAME_FILE and no collision work
instrumentation for low-overhead timing. Results retain failed paths.
"""
import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


def med(rows, key):
    return statistics.median(float(row[key]) for row in rows)


def evaluate(paths):
    cases = {}
    for path in paths:
        with Path(path).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise ValueError(f"{path}: no results")
        caps = {int(row.get("grid_settlement_cap", "0")) for row in rows}
        if len(caps) != 1 or next(iter(caps)) not in (100000, 250000, 500000):
            raise ValueError(f"{path}: expected one accepted heuristic budget")
        cap = next(iter(caps))
        if cap in cases:
            raise ValueError(f"Duplicate grid budget {cap}")
        bycase = {}
        for row in rows:
            if row.get("collision_checker") != "fastpath":
                raise ValueError("Run with --collision-mode fastpath")
            if int(row["map_cells"]) != 1684044 or abs(float(row["resolution_m"]) - .03) > 1e-8:
                raise ValueError("This comparison is for full-resolution Nav2 warehouse only")
            if int(row["discovery_expansions"]) < 30000:
                raise ValueError("Non-qualifying scenario")
            key = (int(row["case_id"]), round(float(row["margin_m"]), 3))
            if key[1] not in (0.0, 0.1) or key in bycase:
                raise ValueError("Missing or repeated route/margin")
            bycase[key] = row
        cases[cap] = bycase
    if set(cases) != {100000, 250000, 500000}:
        raise ValueError("Provide exactly one file for 100k, 250k and 500k")
    keys = set(cases[100000])
    if not keys or any(set(rows) != keys for rows in cases.values()):
        raise ValueError("Different scenarios in heuristic-cap runs")
    for key in keys:
        reference = cases[100000][key]
        for rows in cases.values():
            row = rows[key]
            for field in ("start_x_m", "start_y_m", "start_yaw_rad",
                          "goal_x_m", "goal_y_m", "goal_yaw_rad",
                          "seed", "draw_index", "model", "connector"):
                if row[field] != reference[field]:
                    raise ValueError(f"Scenario {key} differs in {field}")
    print(f"Matched {len(keys)//2} routes × two safety margins, same full Nav2 warehouse.")
    for margin in (0.0, 0.1):
        print(f"\nMARGIN {margin:.2f} m")
        for cap in (100000, 250000, 500000):
            selected = [row for (id, m), row in sorted(cases[cap].items()) if m == margin]
            successes = [row for row in selected if row["status"] == "Success"]
            saturated = sum(int(row["grid_settlements"]) >= cap for row in selected)
            print(f"  grid_cap={cap:6d} success={len(successes)}/{len(selected)} "
                  f"search_median={med(selected, 'search_p50_ms'):.2f} ms "
                  f"total_median={med(selected, 'total_p50_ms'):.2f} ms "
                  f"expanded_median={med(selected, 'expanded'):.0f} "
                  f"fallback_median={med(selected, 'grid_fallbacks'):.0f} "
                  f"saturated={saturated}/{len(selected)}")
        baseline = cases[100000]
        for cap in (250000, 500000):
            # Preserve failures in the success rate above; compute paired
            # latency deltas only for cases where both searches succeeded.
            matched = [(baseline[key], cases[cap][key]) for key in sorted(keys)
                       if key[1] == margin and
                       baseline[key]["status"] == cases[cap][key]["status"] == "Success"]
            if not matched:
                print(f"  vs100k cap={cap}: no mutually successful paths")
                continue
            for field in ("search_p50_ms", "total_p50_ms", "expanded",
                          "exact_collision_pose_calls", "grid_fallbacks"):
                delta = [float(after[field])-float(before[field]) for before, after in matched]
                print(f"  cap {cap} minus 100k {field}: paired median={statistics.median(delta):+.2f}; "
                      f"faster_or_fewer={sum(x<0 for x in delta)}/{len(delta)}")
    print("\nHeuristic settlements are part of search time. Larger budgets may cost "
          "more even if fallbacks and expansions decrease. The reference baseline "
          "and new caps were run on matched exported pose coordinates; do not "
          "confuse this with a Nav2 Smac planning benchmark.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_files", type=Path, nargs=3)
    args = parser.parse_args()
    evaluate(args.csv_files)


if __name__ == "__main__":
    main()
