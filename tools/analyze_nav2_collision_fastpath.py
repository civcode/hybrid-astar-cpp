#!/usr/bin/env python3
"""Report paired legacy-vs-fastpath collision results on the full Nav2 warehouse.

Expected input: four rows per route (0/0.10 m x legacy/fastpath), generated
by hybrid_astar_nav2_warehouse_pipeline_benchmark_test --collision-mode both.
Runs without pandas, ROS or a live network connection.
"""
import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    return values[max(0, math.ceil(fraction * len(values)) - 1)]


def number(row, name):
    return float(row[name])


def summarize(rows):
    groups = {}
    for row in rows:
        margin = round(number(row, "margin_m"), 3)
        mode = row.get("collision_checker")
        if margin not in (0.0, 0.1) or mode not in ("legacy", "fastpath"):
            raise ValueError("Expected both checker modes at 0 and 0.10 m")
        if int(row["map_cells"]) != 1684044 or abs(number(row, "resolution_m") - .03) > 1e-9:
            raise ValueError("Expected original 1006x1674 Nav2 warehouse at 0.03 m/cell")
        if number(row, "discovery_expansions") < 30000:
            raise ValueError("Baseline discovery did not reach 30000 expansions")
        key = int(row["case_id"])
        slots = groups.setdefault(key, {})
        if (margin, mode) in slots:
            raise ValueError(f"Duplicate route/margin/checker: {key}/{margin}/{mode}")
        slots[margin, mode] = row

    if not groups:
        raise ValueError("No rows found")
    for key, slots in groups.items():
        if set(slots) != {
            (0.0, "legacy"), (0.0, "fastpath"),
            (0.1, "legacy"), (0.1, "fastpath")
        }:
            raise ValueError(f"Missing checker/margin combination for route {key}")
        reference = slots[0.0, "legacy"]
        for row in slots.values():
            for field in (
                "start_x_m", "start_y_m", "start_yaw_rad",
                "goal_x_m", "goal_y_m", "goal_yaw_rad",
                "seed", "draw_index", "model", "connector",
                "grid_settlement_cap", "collect_collision_work"
            ):
                if row[field] != reference[field]:
                    raise ValueError(f"Different {field} on paired route {key}")
        for margin in (0.0, 0.1):
            old, new = slots[margin, "legacy"], slots[margin, "fastpath"]
            for field in (
                "status", "expanded", "generated", "pruned_by_key",
                "search_pose_checks", "final_pose_checks", "refine_status",
                "refine_attempted", "refine_accepted", "refine_exact_pose_checks"
            ):
                if old[field] != new[field]:
                    raise ValueError(f"Geometry/work mismatch route {key}, {margin}, {field}")
            for field in ("raw_length_m", "refined_length_m", "raw_cost_m", "refined_cost_m"):
                if abs(number(old, field) - number(new, field)) > 1e-7:
                    raise ValueError(f"Path changed route {key}, {margin}, {field}")

    print(f"Validated {len(groups)} routes x two margins x two collision checkers, "
          "with unchanged search work and path geometry")
    print("Same-map results: unprofiled search/refinement/total timings are comparable "
          "within the same build and CPU; hotpath counters are nested.")
    for margin in (0.0, 0.1):
        old = [g[margin, "legacy"] for _, g in sorted(groups.items())]
        fast = [g[margin, "fastpath"] for _, g in sorted(groups.items())]
        statuses = dict(Counter(r["status"] for r in fast))
        print(f"\nMARGIN {margin:.2f} m: statuses={statuses}; "
              f"checker geometry is identical on every route")
        for field in ("search_p50_ms", "refine_p50_ms", "total_p50_ms"):
            a = [number(row, field) for row in old]
            b = [number(row, field) for row in fast]
            changes = [after - before for before, after in zip(a, b)]
            pct = [(after / before - 1) * 100 for before, after in zip(a, b) if before > 0]
            print(
                f"  {field}: legacy median={statistics.median(a):.2f} ms, "
                f"fastpath median={statistics.median(b):.2f} ms; "
                f"paired median delta={statistics.median(changes):+.2f} ms, "
                f"median change={statistics.median(pct):+.1f}% "
                f"(faster on {sum(v < 0 for v in changes)}/{len(changes)}); "
                f"paired p95 delta={percentile(changes, .95):+.2f} ms"
                if pct else
                f"  {field}: both median 0 ms on failed plans"
            )
        for field in ("collision_candidate_cells", "collision_occupied_cells",
                      "collision_sat_tests", "collision_exact_distance_tests",
                      "collision_inflated_aabb_rejections",
                      "collision_inflated_axis_rejections"):
            if int(old[0].get("collect_collision_work", "0")):
                print(
                    f"  {field}: legacy median={statistics.median(number(r, field) for r in old):,.0f}, "
                    f"fastpath median={statistics.median(number(r, field) for r in fast):,.0f}"
                )
        if int(fast[0].get("profile_enabled", "0")):
            for field in ("exact_collision_hotpath_ms", "search_motion_hotpath_ms"):
                print(
                    f"  {field}: legacy median={statistics.median(number(r, field) for r in old):.2f} ms, "
                    f"fastpath median={statistics.median(number(r, field) for r in fast):.2f} ms"
                )
    caps = {int(g[0.0, "legacy"]["grid_settlement_cap"]) for g in groups.values()}
    if len(caps) == 1:
        cap = next(iter(caps))
        saturated = sum(int(g[0.0, "legacy"]["grid_settlements"]) >= cap for g in groups.values())
        print(f"\nGRID HEURISTIC: {saturated}/{len(groups)} baseline routes hit "
              f"{cap:,} settlement cap. Reuse --replay-csv to compare "
              "--grid-settlements 100000, 250000, 500000 as separate experiments.")
    print("A/B interpretation: broadphase correctness is checked inside the C++ "
          "benchmark before rows are emitted. This analyzer additionally verifies "
          "that search work, refinement counters and paths remain unchanged.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()
    with args.csv_file.open(newline="", encoding="utf-8") as stream:
        summarize(list(csv.DictReader(stream)))


if __name__ == "__main__":
    main()
