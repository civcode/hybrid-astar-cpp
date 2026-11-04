#!/usr/bin/env python3
"""Summarize the matched 0/65536/131072/262144 warehouse label-reserve sweep.

Production-speed conclusions require PROFILE_HOTPATHS=OFF. Timed results are
not available until the user runs the manually invoked C++ benchmark.
"""
import argparse
import csv
from collections import defaultdict
from statistics import median

RESERVES = (0, 65536, 131072, 262144)
WORK_FIELDS = (
    "status", "expanded", "generated", "exact_pose_calls",
    "search_motion_calls", "label_finds", "label_find_hits",
    "label_insertions", "label_updates", "pruned_by_key",
    "stale_open_pops",
)


def value(row, column):
    return float(row[column])


def change_pct(before, after):
    if before <= 0:
        raise ValueError("Cannot compute percentage from nonpositive baseline")
    return 100.0 * (before - after) / before


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file")
    parser.add_argument("--expected-routes", type=int, default=10,
                        help="Expected number of distinct route IDs; use 2 for a smoke run")
    parser.add_argument("--allow-profiled", action="store_true",
                        help="Inspect instrumented runs; NOT suitable for speed selection")
    args = parser.parse_args()
    if args.expected_routes <= 0:
        parser.error("Expected route count must be positive")
    with open(args.csv_file, newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        parser.error("No benchmark rows in input CSV")
    if not args.allow_profiled and any(int(r["profile_enabled"]) != 0 for r in rows):
        raise ValueError("Profiled timings cannot select a production reservation size; "
                         "rerun with PROFILE_HOTPATHS=OFF")
    groups = defaultdict(dict)
    geometry_names = set()
    route_ids = set()
    for row in rows:
        reserve = int(row["label_reserve"])
        if reserve not in RESERVES:
            raise ValueError(f"Unexpected reserve: {reserve}")
        if row["status"] != "Success":
            raise ValueError(f"Unsuccessful reserve={reserve}, case={row['case_id']}: "
                             f"{row['status']}")
        if int(row["prefix_enabled"]) != 1:
            raise ValueError("Prefix setting must remain ON in all variants")
        key = (row["model"], row["geometry"], row["case_id"])
        if reserve in groups[key]:
            raise ValueError(f"Duplicate case/reservation: {key}, {reserve}")
        groups[key][reserve] = row
        geometry_names.add((row["model"], row["geometry"]))
        route_ids.add(row["case_id"])
    if len(route_ids) != args.expected_routes:
        raise ValueError(f"Found {len(route_ids)} route IDs, expected {args.expected_routes}")
    if len(rows) != len(groups) * len(RESERVES):
        raise ValueError("Row count does not match complete quadruples")
    by_geometry = defaultdict(list)
    for key, matched in groups.items():
        if set(matched) != set(RESERVES):
            raise ValueError(f"Missing reserves in {key}: {set(RESERVES)-set(matched)}")
        base = matched[0]
        for reserve, variant in matched.items():
            for field in WORK_FIELDS + ("fast_empty_aabb", "prefix_enabled", "repeats"):
                if variant[field] != base[field]:
                    raise ValueError(f"Reservation changed {field} in {key}: "
                                     f"0={base[field]}, {reserve}={variant[field]}")
        by_geometry[(key[0], key[1])].append(matched)
    expected_pairs = args.expected_routes * len(geometry_names)
    if len(groups) != expected_pairs:
        raise ValueError(f"Expected {expected_pairs} quadruples, found {len(groups)}")
    print(f"Verified {len(rows)} rows: {len(groups)} identical-work, "
          f"four-way route/geometry comparisons")
    if args.allow_profiled:
        print("WARNING: profiling overhead is enabled; do not select sizes from latency data")
    for group_name, matched_cases in [("ALL GEOMETRIES", list(groups.values()))] + [
        (f"{m} / {g}", cases) for (m, g), cases in sorted(by_geometry.items())
    ]:
        print(f"\n{group_name} ({len(matched_cases)} comparisons)")
        print(" Reserve | Median search ms | Median paired saving | Wins vs 0"
              " | Median rehashes | Median bucket MiB | Bucket delta vs 0")
        for reserve in RESERVES:
            values = [p[reserve] for p in matched_cases]
            paired_savings = [change_pct(value(p[0], "search_p50_ms"),
                                         value(p[reserve], "search_p50_ms"))
                              for p in matched_cases]
            wins = sum(value(p[reserve], "search_p50_ms") <
                       value(p[0], "search_p50_ms") for p in matched_cases)
            bucket_mib = [value(r, "bucket_bytes_estimate") / (1024*1024)
                          for r in values]
            delta_mib = [(value(p[reserve], "bucket_bytes_estimate") -
                          value(p[0], "bucket_bytes_estimate")) / (1024*1024)
                         for p in matched_cases]
            print(f" {reserve:7d} | "
                  f"{median(value(r, 'search_p50_ms') for r in values):16.3f} | "
                  f"{median(paired_savings):+19.2f}% | "
                  f"{wins:2d}/{len(matched_cases):2d}      | "
                  f"{median(value(r, 'label_rehashes') for r in values):14.1f} | "
                  f"{median(bucket_mib):17.2f} | "
                  f"{median(delta_mib):+17.2f} MiB")
        print(" Wins against 262144:",
              ", ".join(f"{r}: "
                        f"{sum(value(p[r], 'search_p50_ms') <
                               value(p[262144], 'search_p50_ms')
                               for p in matched_cases)}/{len(matched_cases)}"
                        for r in (0, 65536, 131072)))
    print("\nBucket estimates exclude separate unordered_map node allocations, "
          "allocator overhead, and other process memory.")
    print("Do not enable a universal or adaptive reserve without testing small maps "
          "and short searches as well as the warehouse workloads.")


if __name__ == "__main__":
    main()
