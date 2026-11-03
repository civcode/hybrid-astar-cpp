#!/usr/bin/env python3
"""Offline regression checks for paired legacy/optimized CSV analysis."""
import contextlib
import io

from analyze_nav2_collision_fastpath import summarize


def row(margin, mode):
    return dict(
        case_id="0", margin_m=f"{margin:.6f}", collision_checker=mode,
        map_cells="1684044", resolution_m="0.030000",
        discovery_expansions="30000", draw_index="10", seed="20261029",
        model="differential", connector="native",
        start_x_m="-10", start_y_m="-19", start_yaw_rad="0.5",
        goal_x_m="5", goal_y_m="10", goal_yaw_rad="0",
        grid_settlement_cap="100000", collect_collision_work="1",
        profile_enabled="0", grid_settlements="100000",
        status="Success", expanded="50000", generated="100000",
        pruned_by_key="200000", search_pose_checks="500000",
        final_pose_checks="5000", refine_status="WorkBudgetExceeded",
        refine_attempted="160", refine_accepted="6",
        refine_exact_pose_checks="20000",
        raw_length_m="48", refined_length_m="47",
        raw_cost_m="50", refined_cost_m="49",
        search_p50_ms="1400" if mode == "legacy" else "900",
        refine_p50_ms="20", total_p50_ms="1420" if mode == "legacy" else "920",
        collision_candidate_cells="400000", collision_occupied_cells="100000",
        collision_sat_tests="100000" if mode == "legacy" else "1000",
        collision_exact_distance_tests="100000" if mode == "legacy" else "1000",
        collision_inflated_aabb_rejections="0" if mode == "legacy" else "90000",
        collision_inflated_axis_rejections="0" if mode == "legacy" else "9000",
    )


def main():
    rows = [row(0, "legacy"), row(0, "fastpath"),
            row(.1, "legacy"), row(.1, "fastpath")]
    with contextlib.redirect_stdout(buffer := io.StringIO()):
        summarize(rows)
    text = buffer.getvalue()
    assert "Validated 1 routes x two margins x two collision checkers" in text
    assert "paired median delta=-500.00 ms" in text
    assert "collision_inflated_aabb_rejections" in text
    assert "GRID HEURISTIC: 1/1" in text
    for modified in (
        [*rows, dict(rows[3])],  # duplicate
        [rows[0], rows[1], rows[2]],  # incomplete
        [rows[0], rows[1], rows[2], {**rows[3], "expanded": "50001"}],
        [rows[0], rows[1], rows[2], {**rows[3], "raw_length_m": "48.2"}],
        [rows[0], rows[1], rows[2], {**rows[3], "start_x_m": "7"}],
    ):
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                summarize(modified)
        except ValueError:
            pass
        else:
            raise AssertionError("Bad paired checker CSV was not rejected")
    print("NAV2 COLLISION FASTPATH ANALYZER PASS")


if __name__ == "__main__":
    main()
