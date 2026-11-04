#!/usr/bin/env python3
"""Summarize deterministic full-map clearance OFF/exact/EDT warehouse replays."""
import argparse
import csv
from collections import defaultdict
from statistics import median


def number(row, key):
    return float(row[key])


def integer(row, key):
    return int(row[key])


def pct(before, after):
    return 100 * (before - after) / before if before else 0.0


def med(values):
    return f"{median(values):.2f}" if values else "N/A"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file")
    args = parser.parse_args()
    with open(args.csv_file, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        parser.error("Input CSV has no rows")

    groups = defaultdict(dict)
    for row in rows:
        key = (row["model"], row["geometry"], row["case_id"])
        variant = row["variant"]
        if variant in groups[key]:
            raise ValueError(f"Duplicate variant {variant} for {key}")
        groups[key][variant] = row
    expected = {"off", "exact", "edt"}
    by_geometry = defaultdict(list)
    for key, triple in sorted(groups.items()):
        if set(triple) != expected:
            raise ValueError(f"Expected {expected} for {key}, got {set(triple)}")
        off, exact, edt = (triple[n] for n in ("off", "exact", "edt"))
        # Identical objectives must not change the search or its result.
        for col in (
            "status", "expanded", "generated", "pruned_by_key",
            "exact_collision_calls", "search_motion_calls",
            "clearance_samples", "clearance_queries",
            "raw_sections", "quality_poses", "refine_status",
        ):
            if exact[col] != edt[col]:
                raise ValueError(f"EDT mismatch for {key}: {col} {exact[col]} != {edt[col]}")
        for col in (
            "raw_length_m", "raw_cost_m", "raw_clearance_cost_m",
            "min_clearance_m", "p10_clearance_m",
            "refined_length_m", "refined_cost_m",
        ):
            if abs(number(exact, col) - number(edt, col)) > 2e-5:
                raise ValueError(f"EDT changed {col} on {key}")
        if integer(edt, "clearance_queries") != (
            integer(edt, "edt_zero_cost_bypasses") +
            integer(edt, "exact_clearance_fallbacks")
        ):
            raise ValueError(f"EDT work counters do not reconcile for {key}")
        if integer(exact, "edt_zero_cost_bypasses") != 0:
            raise ValueError(f"Exact-mode EDT bypasses are nonzero for {key}")
        by_geometry[(key[0], key[1])].append((off, exact, edt))

    print("Validated:", len(groups), "matched route/geometry triples; exact and EDT outcomes identical")
    for (model, geometry), triples in sorted(by_geometry.items()):
        print(f"\n{model} / {geometry}: {len(triples)} route triples")
        success = [t for t in triples if all(r["status"] == "Success" for r in t)]
        print("  All-three success:", len(success), "/", len(triples))
        for idx, variant in enumerate(("off", "exact", "edt")):
            series = [t[idx] for t in triples]
            print(
                f"  {variant:5s} median search={med([number(r, 'search_p50_ms') for r in series])} ms,"
                f" expansions={med([integer(r, 'expanded') for r in series])},"
                f" collision checks={med([integer(r, 'exact_collision_calls') for r in series])}"
            )
        print("  Median exact->EDT search saving:",
              med([pct(number(t[1], "search_p50_ms"),
                       number(t[2], "search_p50_ms")) for t in triples]), "%")
        print("  EDT median build cost:", med([number(t[2], "edt_build_ms") for t in triples]), "ms")
        print("  EDT median storage:",
              med([number(t[2], "edt_storage_bytes") / (1024*1024) for t in triples]), "MiB")
        ratios = [
            100 * integer(t[2], "edt_zero_cost_bypasses") /
            integer(t[2], "clearance_queries")
            for t in triples if integer(t[2], "clearance_queries")
        ]
        print("  Median certified zero-penalty samples:", med(ratios), "%")
        if success:
            for field, label in (
                ("expanded", "expansion"),
                ("exact_collision_calls", "exact collision-call"),
            ):
                print(f"  Median OFF->EDT {label} saving:",
                      med([pct(number(t[0], field), number(t[2], field))
                           for t in success]), "%")
            for field, label in (
                ("raw_length_m", "raw path length"),
                ("min_clearance_m", "minimum inflated clearance"),
                ("p10_clearance_m", "10th-percentile inflated clearance"),
            ):
                print(f"  {label} OFF/EDT median:",
                      med([number(t[0], field) for t in success]), "/",
                      med([number(t[2], field) for t in success]), "m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
