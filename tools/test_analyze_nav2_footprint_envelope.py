#!/usr/bin/env python3
"""Offline regression for matched sharp/rounded footprint analyzer."""
import contextlib
import io

from analyze_nav2_footprint_envelope import SHARP, ROUND, compare, summarize


def make(variant):
    sharp = variant == SHARP
    return dict(
        case_id="0", draw_index="57", seed="20261029",
        variant=variant, model="differential",
        map_cells="1684044", resolution_m=".03", grid_cap="100000",
        start_x_m="1.0", start_y_m="5.0", start_yaw_rad="0.4",
        goal_x_m="12", goal_y_m="-10", goal_yaw_rad="-1",
        discovery_expansions="45000",
        body_length_m=".70" if sharp else ".50",
        body_width_m=".50" if sharp else ".30",
        margin_m="0" if sharp else ".10",
        envelope_length_m=".70", envelope_width_m=".50",
        status="Success", repeats="5",
        search_p50_ms="1000" if sharp else "1100",
        search_p95_ms="1050" if sharp else "1150",
        refine_p50_ms="20", refine_p95_ms="22",
        total_p50_ms="1020" if sharp else "1120",
        total_p95_ms="1072" if sharp else "1172",
        expanded="44000" if sharp else "46000",
        generated="80000", search_pose_checks="500000",
        exact_collision_pose_calls="600000",
        grid_settlements="100000", grid_fallbacks="40000",
        raw_length_m="38", refine_status="Improved",
        refine_attempted="160", refine_accepted="5",
        refine_exact_pose_checks="7000", refined_length_m="37",
        work_candidate_cells="4500000" if sharp else "5500000",
        work_occupied_cells="1000000" if sharp else "1300000",
        work_sat_tests="1000000" if sharp else "750000",
        work_exact_distance_tests="0" if sharp else "120000",
        work_aabb_rejections="0" if sharp else "350000",
        work_axis_rejections="0" if sharp else "200000",
        micro_poses="20000", micro_collisions="15000" if sharp else "14800",
        micro_p50_us_per_pose=".65" if sharp else ".81",
        micro_p95_us_per_pose=".67" if sharp else ".84",
        micro_candidate_cells="600000" if sharp else "650000",
        micro_occupied_cells="200000" if sharp else "250000",
        micro_sat_tests="200000" if sharp else "100000",
        micro_exact_distance_tests="0" if sharp else "60000",
        micro_aabb_rejections="0" if sharp else "90000",
        micro_axis_rejections="0" if sharp else "60000",
    )


def main():
    a, b = make(SHARP), make(ROUND)
    with contextlib.redirect_stdout(output := io.StringIO()):
        summarize([a, b])
    text = output.getvalue()
    assert "Validated 1 full-map matched cases" in text
    assert "different_fixed_pose_collision_counts=1/1" in text
    assert "rounded_minus_sharp total_p50_ms: median=+100.000" in text
    assert "median_time_ratio=1.246x" in text
    assert "work_exact_distance_tests" in text
    assert "micro_exact_distance_tests" in text

    for bad in (
        [a], [a, dict(a)], # missing or duplicate
        [a, {**b, "start_x_m": "2"}], # unpaired poses
        [a, {**b, "map_cells": "344128"}], # wrong map
        [a, {**b, "margin_m": "0"}], # mismatched geometry
        [a, {**b, "envelope_width_m": ".70"}],
        [a, {**b, "micro_poses": "19999"}], # not same micro poses
        [a, {**b, "model": "ackermann"}],
        [a, {**b, "discovery_expansions": "29999"}],
    ):
        try:
            compare(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Invalid data unexpectedly accepted: {bad!r}")
    # Geometry may legitimately change success status and search expansions.
    lost = {**b, "status": "NoPath", "expanded": "49000"}
    assert len(compare([a, lost])) == 1
    print("NAV2 FOOTPRINT ENVELOPE ANALYZER PASS")


if __name__ == "__main__":
    main()
