#!/usr/bin/env python3
"""Fetch a pinned Nav2 warehouse PGM and prepare two ROS-compatible demo maps.

No ImageMagick, Pillow, or ROS dependency. Original raster bytes are retained;
the 344128-cell raster is occupancy-conservatively resampled to 608x566.

The downsample has deliberately unchanged 0.03 m/cell resolution, so its
physical coverage is SMALLER. It is not the historical Nav2 Smac benchmark.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil
import urllib.request

NAV2_COMMIT = "4584491f67cda337d25cd1d99edaeb8b4f45a4d9"
NAV2_BLOB = "7ba02265299dda31028f39d4a40d056a656b2a83"
SOURCE_URL = (
    "https://raw.githubusercontent.com/ros-navigation/navigation2/"
    + NAV2_COMMIT + "/nav2_bringup/maps/warehouse.pgm"
)
RESOLUTION = 0.03
ORIGIN = (-15.1, -25.0, 0.0)
TARGET_WIDTH, TARGET_HEIGHT = 608, 566
assert TARGET_WIDTH * TARGET_HEIGHT == 344128


def read_pgm(data: bytes) -> tuple[int, int, bytes]:
    """Read 8-bit binary P5 or ASCII P2 with comment/whitespace handling."""
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(data):
            if data[index] in b" \t\r\n":
                index += 1
            elif data[index] == 35:  # '#'
                newline = data.find(b"\n", index)
                index = len(data) if newline < 0 else newline + 1
            else:
                break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        if index == start:
            raise ValueError("Truncated PGM header")
        result = data[start:index]
        if index < len(data):
            # A CRLF header counts as one separator. Do not consume any
            # additional whitespace, which may be the first raster byte.
            if data[index] == 13 and index + 1 < len(data) and data[index + 1] == 10:
                index += 2
            else:
                index += 1
        return result

    magic = token()
    if magic not in (b"P5", b"P2"):
        raise ValueError("Only 8-bit P5 or ASCII P2 PGM is supported")
    width, height, maxval = int(token()), int(token()), int(token())
    if width < 1 or height < 1 or width * height > 20_000_000 or maxval != 255:
        raise ValueError("Invalid PGM dimensions or maxval")
    count = width * height
    if magic == b"P5":
        raster = data[index:]
        if len(raster) != count:
            raise ValueError("PGM raster length differs from header dimensions")
    else:
        # ASCII PGM source is supported for local fixtures, not expected upstream.
        payload = data[index:].split()
        if len(payload) != count:
            raise ValueError("Invalid ASCII PGM pixel count")
        raster = bytes(int(p) for p in payload)
    return width, height, raster


def conservative_resample(pixels: bytes, width: int, height: int) -> bytes:
    """Any occupied source cell blocks its target; unknown also propagates.

    Uniformly scale to FIT the 608x566 grid, preserving the source aspect
    ratio without stretching. Unused letterbox cells remain unknown.
    The source's ROS trinary thresholds are occupied >0.65, free <0.1,
    negate=false: grayscale 0..88 occupied, 89..229 unknown, 230..255 free.
    """
    scale = min(TARGET_WIDTH / width, TARGET_HEIGHT / height)
    fitted_width = max(1, min(TARGET_WIDTH, round(width * scale)))
    fitted_height = max(1, min(TARGET_HEIGHT, round(height * scale)))
    left = (TARGET_WIDTH - fitted_width) // 2
    top = (TARGET_HEIGHT - fitted_height) // 2
    output = bytearray([205]) * (TARGET_WIDTH * TARGET_HEIGHT)

    # Downsample each occupied source cell into its corresponding output cell.
    # A source blocked cell wins over unknown and free; unknown wins over free.
    x_map = [left + min(fitted_width - 1, (2 * x + 1) * fitted_width // (2 * width))
             for x in range(width)]
    y_map = [top + min(fitted_height - 1, (2 * y + 1) * fitted_height // (2 * height))
             for y in range(height)]
    # Mark covered cells free first; letterbox margins remain unknown.
    for y in range(top, top + fitted_height):
        offset = y * TARGET_WIDTH
        output[offset + left:offset + left + fitted_width] = b"\xfe" * fitted_width
    for y in range(height):
        dst = y_map[y] * TARGET_WIDTH
        src = y * width
        for x, target_x in enumerate(x_map):
            value = pixels[src + x]
            cell = dst + target_x
            if value <= 89:  # occupied threshold 0.65, including conservative boundary
                output[cell] = 0
            elif value < 230 and output[cell] != 0:
                output[cell] = 205
    return bytes(output)


def yaml_metadata(filename: str) -> str:
    return (
        "# Derived from ros-navigation/navigation2 nav2_bringup/maps/warehouse.yaml\n"
        + "image: " + filename + "\n"
        + "mode: trinary\n"
        + "resolution: 0.03\n"
        + "origin: [-15.1, -25.0, 0]\n"
        + "negate: 0\n"
        + "occupied_thresh: 0.65\n"
        + "free_thresh: 0.1\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=pathlib.Path,
        default=pathlib.Path(__file__).resolve().parent.parent / "apps/demo/maps",
        help="Generated PGM, ROS YAML, and manifest destination",
    )
    parser.add_argument(
        "--source", type=pathlib.Path,
        help="Use a local PGM instead of downloading (useful for offline fixtures)",
    )
    args = parser.parse_args()
    if args.source:
        data = args.source.read_bytes()
    else:
        request = urllib.request.Request(
            SOURCE_URL, headers={"User-Agent": "hybrid-astar-cpp-map-preparation/1.0"}
        )
        with urllib.request.urlopen(request, timeout=90) as response:
            data = response.read(20_000_001)
        if len(data) > 20_000_000:
            raise ValueError("Downloaded Nav2 PGM exceeds 20 MB limit")
        blob_sha = hashlib.sha1(
            f"blob {len(data)}".encode("ascii") + bytes([0]) + data
        ).hexdigest()
        if blob_sha != NAV2_BLOB:
            raise ValueError(
                f"Upstream map blob changed: expected {NAV2_BLOB}, got {blob_sha}"
            )
    width, height, pixels = read_pgm(data)
    if width < TARGET_WIDTH or height < TARGET_HEIGHT:
        raise ValueError("Source raster too small for 608x566 downsample")
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)

    full_pgm = output / "nav2_warehouse_full.pgm"
    full_pgm.write_bytes(data)  # original bytes, unchanged
    small_pgm = output / "nav2_warehouse_344128.pgm"
    small_pgm.write_bytes(
        f"P5\n{TARGET_WIDTH} {TARGET_HEIGHT}\n255\n".encode("ascii")
        + conservative_resample(pixels, width, height)
    )
    for filename in (full_pgm, small_pgm):
        filename.with_suffix(".yaml").write_text(
            yaml_metadata(filename.name), encoding="utf-8"
        )
    manifest = {
        "upstream_repository": "ros-navigation/navigation2",
        "upstream_commit": NAV2_COMMIT,
        "upstream_git_blob": NAV2_BLOB if not args.source else None,
        "source_file": "nav2_bringup/maps/warehouse.pgm",
        "resolution_m_per_cell": RESOLUTION,
        "origin": ORIGIN,
        "resampling": "aspect-preserving fit, blocked/unknown conservative pooling",
        "note": "344128-cell map has smaller physical extent at fixed 0.03 m/cell",
        "maps": {
            "nav2_warehouse_full": {
                "width": width, "height": height, "cells": width * height
            },
            "nav2_warehouse_344128": {
                "width": TARGET_WIDTH, "height": TARGET_HEIGHT,
                "cells": TARGET_WIDTH * TARGET_HEIGHT
            },
        },
    }
    (output / "nav2_warehouse_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Nav2 original: {width} x {height} = {width * height:,} cells")
    print(f"Nav2 reduced:  {TARGET_WIDTH} x {TARGET_HEIGHT} = 344,128 cells")
    print(f"Both at 0.03 m/cell; saved to {output}")


if __name__ == "__main__":
    main()
