"""Standalone stdlib regression checks for the Docker weight-sweep driver."""

import csv
import io
from pathlib import Path
import importlib.util
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "articulated_heuristic_weight_sweep.py"

FAKE_BENCHMARK = '''#!/usr/bin/env python3
import sys
mode, weight = sys.argv[1], float(sys.argv[2])
print("mode,status,validated,expanded,generated,search_body_checks,elapsed_ms,stationary_s,coupled_s,drive_m,swept_free,grid_settlements,heuristic_weight,plan_cost_m,goal_xy_error_m,goal_yaw_error_rad,goal_gamma_error_rad")
print(f"{mode},0,1,{1000 if mode == 'weighted' else 2000},5000,6000,{20 if mode == 'weighted' else 30},1,10,{20 if mode == 'weighted' else 21},200,100,{weight},{25 if mode == 'weighted' else 24},0.1,0.02,0.01")
'''


class WeightSweepTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.benchmark = Path(self.dir.name) / "fake_benchmark"
        self.benchmark.write_text(FAKE_BENCHMARK, encoding="utf-8")
        self.benchmark.chmod(0o755)

    def run_sweep(self, *args):
        return subprocess.run([sys.executable, str(SCRIPT),
                               "--benchmark", str(self.benchmark), *args],
                              text=True, capture_output=True, check=False)

    def test_summary_contains_matched_modes_and_path_cost(self):
        proc = self.run_sweep("--weights", "1.0,1.5", "--repeats", "2")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        rows = list(csv.DictReader(io.StringIO(proc.stdout)))
        self.assertEqual(len(rows), 4)
        self.assertEqual([(x["mode"], x["heuristic_weight"]) for x in rows],
                         [("grid-weighted", "1"), ("weighted", "1"),
                          ("grid-weighted", "1.5"), ("weighted", "1.5")])
        for row in rows:
            self.assertEqual(row["runs"], "2")
            self.assertEqual(row["validated"], "2")
        self.assertEqual(rows[0]["median_plan_cost_m"], "24")
        self.assertEqual(rows[1]["median_plan_cost_m"], "25")
        self.assertEqual(rows[0]["median_expanded"], "2000")
        self.assertEqual(rows[1]["median_expanded"], "1000")

    def test_raw_rows(self):
        proc = self.run_sweep("--weights", "1.5", "--repeats", "2", "--raw")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        rows = list(csv.DictReader(io.StringIO(proc.stdout)))
        self.assertEqual(len(rows), 4)
        self.assertEqual([x["repetition"] for x in rows], ["1", "2", "1", "2"])
        self.assertTrue(all(x["validated"] == "1" for x in rows))

    def test_invalid_weights_rejected(self):
        spec = importlib.util.spec_from_file_location("sweep", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for value in ("nan", "inf", "-1", "0", "3.1", "1.2,oops", "1,1"):
            with self.subTest(value=value):
                with self.assertRaises(module.argparse.ArgumentTypeError):
                    module.parse_weights(value)
        proc = self.run_sweep("--weights", "nan")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("error", proc.stderr)

    def test_invalid_repeat_count_rejected(self):
        proc = self.run_sweep("--repeats", "0")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("--repeats must be", proc.stderr)


if __name__ == "__main__":
    unittest.main()
