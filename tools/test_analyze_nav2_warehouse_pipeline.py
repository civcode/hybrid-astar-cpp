#!/usr/bin/env python3
"""Offline sanity checks: paired pipeline reporting must preserve failures."""
import contextlib
import io

from analyze_nav2_warehouse_pipeline import summarize


def row(margin, status="Success", case_id="0"):
    fields = dict(
        case_id=case_id,
        margin_m=margin,
        status=status,
        map_cells="1684044",
        resolution_m="0.03",
        discovery_expansions="30000",
        start_x_m="-12.52", start_y_m="20.53", start_yaw_rad="0.0",
        goal_x_m="13.07", goal_y_m="-18.33", goal_yaw_rad="0.0",
        draw_index="42", seed="20261029", model="differential", connector="native",
        search_p50_ms="1200.0", refine_p50_ms="25.0", total_p50_ms="1225.0",
        search_p95_ms="1300.0", refine_p95_ms="27.0", total_p95_ms="1327.0",
        expanded="40000", grid_fallbacks="2000", grid_settlements="100000",
        search_pose_checks="100000", exact_collision_pose_calls="100000",
        refine_accepted="6", refine_exact_pose_checks="7000",
        refined_length_m="69.0", search_motion_calls="300000",
        pruned_by_key="250000", profile_enabled="0",
    )
    return fields


def main():
    a = row("0.000000")
    b = row("0.100000", status="NoPath")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        summarize([a, b])
    text = buf.getvalue()
    assert "Validated 1 fully matched routes" in text
    assert "MARGIN 0.10 m: 0/1 valid plans" in text
    assert "inflated_failure=1/1" in text
    assert "POTENTIAL BOTTLENECK INDICATORS" in text
    for modified in (
        {**b, "goal_x_m": "1.0"},
        {**b, "discovery_expansions": "29999"},
        {**b, "map_cells": "344128"},
    ):
        try:
            summarize([a, modified])
        except ValueError:
            pass
        else:
            raise AssertionError("Expected invalid / unpaired benchmark data to be rejected")
    # All-success paired delta must report positive slowdown.
    b = row("0.100000")
    b["search_p50_ms"] = "1400.0"
    with contextlib.redirect_stdout(buf := io.StringIO()):
        summarize([a, b])
    assert "delta inflated-minus-off search_p50_ms: median=+200.00" in buf.getvalue()
    print("NAV2 PIPELINE ANALYZER TEST PASS")


if __name__ == "__main__":
    main()
