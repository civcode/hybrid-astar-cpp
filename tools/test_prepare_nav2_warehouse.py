#!/usr/bin/env python3
"""Offline integration checks for the generated Nav2 warehouse assets."""
import pathlib
import subprocess
import sys
import tempfile

from prepare_nav2_warehouse import (
    TARGET_WIDTH, TARGET_HEIGHT, read_pgm,
    conservative_resample, yaml_metadata,
)


def main():
    # Fixture: a 700x620 P5 map with all three occupancy states.
    width, height = 700, 620
    cells = bytearray([254]) * (width * height)
    cells[0] = 0                       # blocked upper-left source
    cells[width * 120 + 200] = 205    # unknown source
    cells[width * 400 + 500] = 0      # blocked lower-right source
    source = f"P5\n# test source\n{width} {height}\n255\n".encode() + cells
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        fixture = root / "fixture.pgm"
        fixture.write_bytes(source)
        destination = root / "prepared"
        subprocess.run(
            [sys.executable,
             str(pathlib.Path(__file__).with_name("prepare_nav2_warehouse.py")),
             "--source", str(fixture), "--output-dir", str(destination)],
            check=True, capture_output=True, text=True,
        )
        full = destination / "nav2_warehouse_full.pgm"
        reduced = destination / "nav2_warehouse_344128.pgm"
        assert full.read_bytes() == source, "full map must preserve all source bytes"
        sw, sh, original = read_pgm(full.read_bytes())
        rw, rh, smaller = read_pgm(reduced.read_bytes())
        assert (sw, sh) == (700, 620)
        assert (rw, rh) == (TARGET_WIDTH, TARGET_HEIGHT)
        assert rw * rh == 344128 and len(smaller) == 344128
        assert smaller == conservative_resample(original, sw, sh)
        assert 0 in smaller and 205 in smaller and 254 in smaller
        for path in (full, reduced):
            yaml = path.with_suffix(".yaml").read_text()
            assert "resolution: 0.03" in yaml
            assert "origin: [-15.1, -25.0, 0]" in yaml
            assert "mode: trinary" in yaml
            assert "image: " + path.name in yaml
        manifest = (destination / "nav2_warehouse_manifest.json").read_text()
        assert '"cells": 344128' in manifest
        assert '"cells": 434000' in manifest
    print("NAV2 WAREHOUSE MAP GENERATOR PASS")


if __name__ == "__main__":
    main()
