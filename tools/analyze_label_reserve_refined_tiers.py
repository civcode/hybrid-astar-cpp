#!/usr/bin/env python3
"""Validate matched calibration-route tier v2 timings and bucket-array tradeoffs.

This does not infer new timing results. Run the manually invoked C++ benchmark
with PROFILE_HOTPATHS=OFF and pass its CSV here.
"""
import argparse
import csv
import math
from collections import defaultdict
from statistics import median

CAPACITIES = {13: 524288, 14: 655360, 15: 786432, 17: 1048576}
WORK = (
    "status", "expanded", "generated", "peak_labels", "baseline_rehashes",
    "label_finds", "label_find_hits", "label_insertions", "label_updates",
    "pruned_by_key", "exact_pose_calls", "search_motion_calls",
)
FIXED = ("model", "geometry", "seed", "repeats", "fast_empty_aabb",
         "map_fingerprint", "tier_series_version")


def number(row, field):
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"Nonfinite {field}: {row[field]}")
    return value


def saving(a, b):
    if a <= 0:
        raise ValueError("Search timings must be positive")
    return 100.0 * (a - b) / a


def analyze(filename, expected_routes):
    with open(filename, encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("No benchmark data")
    if len(rows) != expected_routes * len(CAPACITIES):
        raise ValueError(f"Expected {expected_routes*len(CAPACITIES)} rows, "
                         f"got {len(rows)}")
    groups = defaultdict(dict)
    for row in rows:
        tier = int(row["tier_index"])
        if tier not in CAPACITIES:
            raise ValueError(f"Unexpected tier {tier}")
        if int(row["tier_series_version"]) != 2:
            raise ValueError("Only tier-series version 2 is supported")
        if int(row["label_reserve"]) != CAPACITIES[tier]:
            raise ValueError(f"Incorrect capacity for tier {tier}")
        if row["status"] != "Success":
            raise ValueError(f"Unsuccessful sampled route {row['sample']}")
        if int(row["rehashes"]) != 0:
            raise ValueError(f"Tier {tier} rehashed on route {row['sample']}")
        if number(row, "search_p50_ms") <= 0:
            raise ValueError("Search time must be positive")
        key = (row["sample"], row["model"], row["geometry"], row["seed"])
        if tier in groups[key]:
            raise ValueError(f"Duplicate tier {tier} for {key}")
        groups[key][tier] = row
    if len(groups) != expected_routes:
        raise ValueError(f"Expected {expected_routes} route groups, got {len(groups)}")
    for key, matched in groups.items():
        if set(matched) != set(CAPACITIES):
            raise ValueError(f"Missing tiers for {key}")
        first = matched[13]
        for tier, value in matched.items():
            for field in WORK + FIXED:
                if value[field] != first[field]:
                    raise ValueError(f"Tier {tier} changed {field} on {key}")
    all_seeds = {r["seed"] for r in rows}
    all_maps = {r["map_fingerprint"] for r in rows}
    all_geometry = {(r["model"], r["geometry"]) for r in rows}
    if len(all_seeds) != 1 or len(all_maps) != 1 or len(all_geometry) != 1:
        raise ValueError("Mixed map, seed or robot geometry; split inputs")
    if sorted(int(k[0]) for k in groups) != list(range(expected_routes)):
        raise ValueError("Sample IDs are not a complete 0..N-1 sequence")
    cases = list(groups.values())
    print(f"Verified {len(rows)} successful rows: {len(cases)} matched four-tier "
          "comparisons, all zero-rehash and identical search work.")
    print(f"Map fingerprint: {next(iter(all_maps))}; seed: "
          f"{next(iter(all_seeds))}; model/geometry: {next(iter(all_geometry))}")
    print("Bucket memory estimates cover unordered_map bucket pointers only, "
          "not node allocations or total process RSS.\n")
    print(" Tier | Labels   | Median search ms | Median saving vs 13 | "
          "Wins vs 13 | Wins vs 17 | Median bucket MiB | Delta vs 13")
    for tier in CAPACITIES:
        values = [case[tier] for case in cases]
        ms = [number(r, "search_p50_ms") for r in values]
        savings13 = [saving(number(c[13], "search_p50_ms"),
                            number(c[tier], "search_p50_ms")) for c in cases]
        wins13 = sum(number(c[tier], "search_p50_ms") <
                     number(c[13], "search_p50_ms") for c in cases)
        wins17 = sum(number(c[tier], "search_p50_ms") <
                     number(c[17], "search_p50_ms") for c in cases)
        bucket = [number(v, "bucket_bytes_estimate")/(1024*1024) for v in values]
        delta = [(number(c[tier], "bucket_bytes_estimate") -
                  number(c[13], "bucket_bytes_estimate"))/(1024*1024)
                 for c in cases]
        print(f" {tier:4d} | {CAPACITIES[tier]:8d} | {median(ms):16.3f} | "
              f"{median(savings13):+18.2f}% | "
              f"{wins13:2d}/{len(cases):2d}      | "
              f"{wins17:2d}/{len(cases):2d}      | "
              f"{median(bucket):17.2f} | {median(delta):+.2f} MiB")
    print("\nAdditional matched comparisons:")
    for a, b in ((13, 14), (14, 15), (14, 17), (15, 17)):
        pct = [saving(number(c[a], "search_p50_ms"),
                      number(c[b], "search_p50_ms")) for c in cases]
        wins = sum(number(c[b], "search_p50_ms") <
                   number(c[a], "search_p50_ms") for c in cases)
        print(f"  {a} -> {b}: median {median(pct):+.2f}% faster in tier {b}; "
              f"{wins}/{len(cases)} wins")
    hard = sorted(cases, key=lambda c: int(c[13]["peak_labels"]), reverse=True)[:5]
    print("\nFive highest-label routes (sample, labels, tier13/14/15/17 ms):")
    for c in hard:
        print("  " + c[13]["sample"] + ", " + c[13]["peak_labels"] + ", " +
              "/".join(f"{number(c[t], 'search_p50_ms'):.3f}" for t in CAPACITIES))
    print("\nInterpretation: choose a memory/speed balance only after reviewing "
          "paired timing and real-world RSS. Zero rehashes alone is not a speed result.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--expected-routes", type=int, default=32)
    args = p.parse_args()
    if args.expected_routes <= 0:
        p.error("--expected-routes must be positive")
    analyze(args.csv_file, args.expected_routes)
