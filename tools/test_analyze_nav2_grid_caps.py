#!/usr/bin/env python3
"""Verify heuristic-cap report matches routes and retains all three budgets."""
import contextlib
import csv
import io
import pathlib
import tempfile

from analyze_nav2_grid_caps import evaluate


def main():
    with tempfile.TemporaryDirectory() as temp:
        paths = []
        for cap in (100000, 250000, 500000):
            path = pathlib.Path(temp) / f"grid-{cap}.csv"
            rows = []
            for margin in (0.0, 0.1):
                rows.append({
                    "case_id": "0", "margin_m": f"{margin:.6f}",
                    "grid_settlement_cap": str(cap), "collision_checker": "fastpath",
                    "map_cells": "1684044", "resolution_m": ".03",
                    "discovery_expansions": "80000",
                    "seed": "20261029", "draw_index": "37",
                    "model": "differential", "connector": "native",
                    "start_x_m": "11", "start_y_m": "-13", "start_yaw_rad": "1",
                    "goal_x_m": "-7", "goal_y_m": "19", "goal_yaw_rad": "2",
                    "status": "Success", "grid_settlements": str(cap),
                    "search_p50_ms": str(1400 - cap / 1000),
                    "total_p50_ms": str(1430 - cap / 1000),
                    "expanded": str(150000 - cap // 100),
                    "grid_fallbacks": str(200000 - cap // 10),
                    "exact_collision_pose_calls": "2500000",
                })
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            paths.append(path)
        with contextlib.redirect_stdout(output := io.StringIO()):
            evaluate(paths)
        result = output.getvalue()
        assert "Matched 1 routes" in result
        assert "grid_cap=100000" in result
        assert "grid_cap=500000" in result
        assert "cap 250000 minus 100k search_p50_ms" in result

        # No duplicate or mismatched capacity files.
        try:
            evaluate((paths[0], paths[0], paths[2]))
        except ValueError:
            pass
        else:
            raise AssertionError("Duplicated grid cap accepted")
    print("NAV2 GRID CAP ANALYZER PASS")


if __name__ == "__main__":
    main()
