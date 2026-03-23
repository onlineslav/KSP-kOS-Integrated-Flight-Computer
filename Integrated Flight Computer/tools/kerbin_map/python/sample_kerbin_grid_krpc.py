#!/usr/bin/env python3
"""Sample a body's lat/lon grid from KSP via kRPC.

Outputs NumPy arrays suitable for high-resolution offline map rendering.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

try:
    import krpc
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Missing dependency: krpc. Install with `python -m pip install -r requirements.txt`."
    ) from exc


def lat_for_row(row: int, height: int) -> float:
    return 90.0 - (row + 0.5) * 180.0 / float(height)


def lon_for_col(col: int, width: int) -> float:
    return -180.0 + (col + 0.5) * 360.0 / float(width)


def format_hms(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sample body terrain/biomes via kRPC.")
    p.add_argument("--host", default="127.0.0.1", help="kRPC host (default: 127.0.0.1)")
    p.add_argument("--rpc-port", type=int, default=50000, help="kRPC RPC port")
    p.add_argument("--stream-port", type=int, default=50001, help="kRPC stream port")
    p.add_argument("--client-name", default="ifc_kerbin_map_sampler", help="kRPC client name")
    p.add_argument("--body", default="Kerbin", help="Body name to sample")
    p.add_argument("--width", type=int, default=4096, help="Output grid width")
    p.add_argument("--height", type=int, default=2048, help="Output grid height")
    p.add_argument("--bedrock", action="store_true", help="Also sample bedrock height")
    p.add_argument("--biome", action="store_true", help="Also sample biome index grid")
    p.add_argument("--out-dir", default="output", help="Output directory")
    p.add_argument("--out-stem", default="kerbin", help="Output filename stem")
    p.add_argument(
        "--progress-rows",
        type=int,
        default=16,
        help="Print progress every N rows",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Connecting to kRPC at {args.host}:{args.rpc_port}/{args.stream_port} ...")
    conn = krpc.connect(
        name=args.client_name,
        address=args.host,
        rpc_port=args.rpc_port,
        stream_port=args.stream_port,
    )

    bodies = conn.space_center.bodies
    if args.body not in bodies:
        print(f"ERROR: body '{args.body}' not found. Available: {', '.join(sorted(bodies.keys()))}")
        return 1
    body = bodies[args.body]

    width = int(args.width)
    height = int(args.height)
    if width <= 0 or height <= 0:
        print("ERROR: width/height must be > 0")
        return 1

    print(f"Sampling body={body.name} at {width} x {height} ({width * height:,} points)")
    print(f"Options: bedrock={args.bedrock} biome={args.biome}")

    surface = np.zeros((height, width), dtype=np.float32)
    bedrock = np.zeros((height, width), dtype=np.float32) if args.bedrock else None
    biome_idx = np.zeros((height, width), dtype=np.uint16) if args.biome else None
    biome_names: list[str] = []
    biome_lookup: dict[str, int] = {}

    start = time.time()
    total = width * height
    last_progress = 0.0

    for row in range(height):
        lat = lat_for_row(row, height)
        for col in range(width):
            lon = lon_for_col(col, width)

            surface[row, col] = body.surface_height(lat, lon)

            if bedrock is not None:
                bedrock[row, col] = body.bedrock_height(lat, lon)

            if biome_idx is not None:
                biome_name = body.biome_at(lat, lon)
                idx = biome_lookup.get(biome_name)
                if idx is None:
                    idx = len(biome_names)
                    biome_lookup[biome_name] = idx
                    biome_names.append(biome_name)
                biome_idx[row, col] = idx

        if (row % args.progress_rows) == 0 or row == (height - 1):
            done = (row + 1) * width
            elapsed = time.time() - start
            rate = done / max(elapsed, 1e-6)
            eta = (total - done) / max(rate, 1e-6)
            now = time.time()
            if now - last_progress >= 0.2:
                print(
                    f"row {row + 1:>5}/{height}  "
                    f"{(100.0 * done / total):6.2f}%  "
                    f"{rate:9.1f} pts/s  ETA {format_hms(eta)}"
                )
                last_progress = now

    surface_path = out_dir / f"{args.out_stem}_surface_height.npy"
    np.save(surface_path, surface)
    print(f"Wrote {surface_path}")

    bedrock_path = None
    if bedrock is not None:
        bedrock_path = out_dir / f"{args.out_stem}_bedrock_height.npy"
        np.save(bedrock_path, bedrock)
        print(f"Wrote {bedrock_path}")

    biome_path = None
    if biome_idx is not None:
        biome_path = out_dir / f"{args.out_stem}_biome_idx.npy"
        np.save(biome_path, biome_idx)
        print(f"Wrote {biome_path}")

    meta = {
        "body": body.name,
        "width": width,
        "height": height,
        "timestamp_unix": time.time(),
        "surface_file": surface_path.name,
        "bedrock_file": bedrock_path.name if bedrock_path else "",
        "biome_file": biome_path.name if biome_path else "",
        "biome_names": biome_names,
        "notes": "Equirectangular grid, lon in [-180,180], lat in [90,-90].",
    }
    meta_path = out_dir / f"{args.out_stem}_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"Wrote {meta_path}")

    elapsed = time.time() - start
    print(f"Done in {format_hms(elapsed)} ({elapsed:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
