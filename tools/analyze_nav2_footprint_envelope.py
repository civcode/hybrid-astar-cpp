#!/usr/bin/env python3
"""Analyze matched 70x50 sharp vs 50x30+0.10m rounded Nav2 footprints.

Rows come from hybrid_astar_nav2_footprint_envelope_benchmark_test.
Only stdlib. Do NOT compare timing deltas from different CPUs or builds.
Search and refinement timings are instrument-free. Detailed per-search
collision counters and collision-only microbenchmarks are separate passes.
"""
import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path

SHARP = "large_rectangle_70x50"
ROUND = "small_rounded_50x30_margin10"
REQUIRED = {SHARP, ROUND}


def med(seq):
    return statistics.median(seq)


def p95(seq):
    xs = sorted(seq)
    return xs[max(0, math.ceil(.95 * len(xs)) - 1)]


def num(row, field):
    return float(row[field])


def close(a, b, atol=1e-7):
    return math.isclose(a, b, abs_tol=atol, rel_tol=1e-9)


def compare(rows):
    paired = {}
    for row in rows:
        key = int(row["case_id"])
        variant = row["variant"]
        if variant not in REQUIRED:
            raise ValueError("Unexpected footprint variant: " + variant)
        if int(row["map_cells"]) != 1684044 or not close(num(row, "resolution_m"), .03):
            raise ValueError("Expected original 1006x1674 Nav2 warehouse at 0.03m/cell")
        if int(row["discovery_expansions"]) < 30000:
            raise ValueError("Route does not meet original 30k+ expansion qualification")
        if row["model"] != "differential":
            raise ValueError("Experiment must use identical differential-drive dynamics")
        if not close(num(row, "envelope_length_m"), .70) or not close(
                num(row, "envelope_width_m"), .50):
            raise ValueError("Two comparison shapes must share 70x50cm extents")
        expected = (.70, .50, 0) if variant == SHARP else (.50, .30, .10)
        for field, value in zip(("body_length_m", "body_width_m", "margin_m"), expected):
            if not close(num(row, field), value):
                raise ValueError(f"Invalid footprint setting for {variant}: {field}")
        variants = paired.setdefault(key, {})
        if variant in variants:
            raise ValueError(f"Duplicate case={key}, variant={variant}")
        variants[variant] = row
    if not paired:
        raise ValueError("No benchmark rows")
    for case, variants in paired.items():
        if set(variants) != REQUIRED:
            raise ValueError(f"Missing paired geometry for case {case}")
        a, b = variants[SHARP], variants[ROUND]
        for field in (
            "start_x_m", "start_y_m", "start_yaw_rad", "goal_x_m",
            "goal_y_m", "goal_yaw_rad", "draw_index", "seed",
            "map_cells", "grid_cap", "model", "repeats", "micro_poses"
        ):
            if a[field] != b[field]:
                raise ValueError(f"Case {case}: compared different {field} values")
        if num(a, "micro_poses") < 100:
            raise ValueError("Too few matched microbenchmark poses")
        # Do NOT demand identical search expansions or paths. Rounded corners
        # and physical rectangle corners are genuinely different geometries.
        for row in (a, b):
            for field in ("search_p50_ms", "refine_p50_ms", "total_p50_ms",
                          "micro_p50_us_per_pose", "micro_p95_us_per_pose"):
                if not math.isfinite(num(row, field)) or num(row, field) < 0:
                    raise ValueError("Missing, negative, or nonfinite timings")
    return paired


def summarize(rows):
    pairs = compare(rows)
    print(f"Validated {len(pairs)} full-map matched cases; >=30k baseline expansions.")
    print("SHARP = 70x50cm rectangle, margin 0; ROUNDED = 50x30cm rectangle +0.10m.")
    print("Both have 70x50cm outer extents, but the rounded corners differ.")
    sharp = [p[SHARP] for _, p in sorted(pairs.items())]
    rounded = [p[ROUND] for _, p in sorted(pairs.items())]
    for title, selected in (("SHARP", sharp), ("ROUNDED", rounded)):
        status = dict(Counter(r["status"] for r in selected))
        print(f"\n{title}: status={status}")
        for field in (
            "search_p50_ms", "refine_p50_ms", "total_p50_ms",
            "expanded", "exact_collision_pose_calls", "raw_length_m",
            "micro_p50_us_per_pose", "micro_p95_us_per_pose"
        ):
            values = [num(r, field) for r in selected]
            print(f"  {field}: median={med(values):.3f}, p95_across_cases={p95(values):.3f}")
        calls = [num(r, "exact_collision_pose_calls") for r in selected]
        for field in (
            "work_candidate_cells", "work_occupied_cells",
            "work_sat_tests", "work_exact_distance_tests",
            "work_aabb_rejections", "work_axis_rejections"
        ):
            total = [num(r, field) for r in selected]
            per_pose = [n / d for n, d in zip(total, calls) if d > 0]
            print(f"  {field}: median_total={med(total):,.0f}; "
                  f"median_per_planning_pose_check={med(per_pose) if per_pose else 0:.3f}")
        micro = [num(r, "micro_poses") for r in selected]
        for field in (
            "micro_candidate_cells", "micro_occupied_cells",
            "micro_sat_tests", "micro_exact_distance_tests",
            "micro_aabb_rejections", "micro_axis_rejections"
        ):
            per_pose = [num(r, field)/count for r, count in zip(selected, micro)]
            print(f"  {field}: median_per_fixed_pose={med(per_pose):.3f}")
        collision_fraction = [
            num(r, "micro_collisions") / count for r, count in zip(selected, micro)
        ]
        print(f"  fixed_pose_collision_fraction: median={med(collision_fraction):.3%}")

    succeeded = [(a, b) for a, b in zip(sharp, rounded)
                 if a["status"] == "Success" and b["status"] == "Success"]
    changed_status = sum(a["status"] != b["status"] for a, b in zip(sharp, rounded))
    changed_micro_collisions = sum(a["micro_collisions"] != b["micro_collisions"]
                                   for a, b in zip(sharp, rounded))
    print(f"\nPAIRED outcomes: both_success={len(succeeded)}/{len(sharp)}, "
          f"different_status={changed_status}/{len(sharp)}, "
          f"different_fixed_pose_collision_counts={changed_micro_collisions}/{len(sharp)}.")
    print("Total planning time and expansions may change because the two rounded/sharp "
          "footprints are NOT identical collision sets.")
    if succeeded:
        for field in (
            "search_p50_ms", "refine_p50_ms", "total_p50_ms",
            "expanded", "exact_collision_pose_calls", "refined_length_m"
        ):
            deltas = [num(b, field) - num(a, field) for a, b in succeeded]
            print(f"  rounded_minus_sharp {field}: median={med(deltas):+.3f}, "
                  f"p95={p95(deltas):+.3f}, rounded_lower={sum(v < 0 for v in deltas)}/{len(deltas)}")
    # The direct collision-only micro uses the SAME randomly generated poses
    # on the SAME map. It isolates checker work from changes in search work,
    # but different corner geometry can still change early collision exits.
    differences = [num(b, "micro_p50_us_per_pose") -
                   num(a, "micro_p50_us_per_pose")
                   for a, b in zip(sharp, rounded)]
    ratios = [num(b, "micro_p50_us_per_pose") /
              num(a, "micro_p50_us_per_pose")
              for a, b in zip(sharp, rounded)
              if num(a, "micro_p50_us_per_pose") > 0]
    print(f"\nCOLLISION-ONLY matched-pose test: rounded_minus_sharp "
          f"median={med(differences):+.3f} us/check; "
          f"median_time_ratio={med(ratios) if ratios else float('nan'):.3f}x "
          f"(rounded faster on {sum(v < 0 for v in differences)}/{len(differences)} cases).")
    print("Use the fixed-pose candidate/occupied-cell and exact-distance test "
          "counts above to distinguish extra scanned cells from extra computation.")
    print("Caveats: microbenchmark poses are sampled uniformly, not from planner "
          "trajectories. Full-pipeline work counts were collected in a separate "
          "untimed pass. Reported microseconds/check include early collision exits.")
    print("These results compare collision implementations and shapes in our planner, "
          "not Nav2 Smac's runtime.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    args = parser.parse_args()
    with args.csv_file.open(newline="", encoding="utf-8") as stream:
        summarize(list(csv.DictReader(stream)))


if __name__ == "__main__":
    main()
