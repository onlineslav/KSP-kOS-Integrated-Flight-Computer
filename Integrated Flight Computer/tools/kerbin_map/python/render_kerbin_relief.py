#!/usr/bin/env python3
"""Render a shaded relief Kerbin map from sampled terrain grids."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Render shaded equirectangular map PNG.")
    p.add_argument("--surface", required=True, help="Path to *_surface_height.npy")
    p.add_argument("--bedrock", default="", help="Optional *_bedrock_height.npy for ocean depth")
    p.add_argument("--out", required=True, help="Output PNG path")
    p.add_argument("--land-max", type=float, default=-1.0, help="Land normalization max (m); <=0 uses p99")
    p.add_argument("--ocean-max-depth", type=float, default=6000.0, help="Depth for full ocean color scale (m)")
    p.add_argument("--sun-azimuth", type=float, default=315.0, help="Hillshade sun azimuth (deg)")
    p.add_argument("--sun-altitude", type=float, default=45.0, help="Hillshade sun altitude (deg)")
    p.add_argument("--z-scale", type=float, default=1.0, help="Vertical exaggeration for hillshade")
    p.add_argument("--shade-strength", type=float, default=0.35, help="0..1 hillshade blend strength")
    return p.parse_args()


def interp_palette(t: np.ndarray, stops: list[tuple[float, tuple[int, int, int]]]) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    pos = np.array([s[0] for s in stops], dtype=np.float32)
    cols = np.array([s[1] for s in stops], dtype=np.float32) / 255.0
    out = np.zeros((*t.shape, 3), dtype=np.float32)
    for c in range(3):
        out[..., c] = np.interp(t, pos, cols[:, c])
    return out


def hillshade(elev: np.ndarray, azimuth_deg: float, altitude_deg: float, z_scale: float) -> np.ndarray:
    dy, dx = np.gradient(elev.astype(np.float32) * float(z_scale))
    slope = np.pi / 2.0 - np.arctan(np.hypot(dx, dy))
    aspect = np.arctan2(-dx, dy)

    az = np.deg2rad(azimuth_deg)
    alt = np.deg2rad(altitude_deg)

    shade = (
        np.sin(alt) * np.sin(slope)
        + np.cos(alt) * np.cos(slope) * np.cos(az - aspect)
    )
    return np.clip((shade + 1.0) * 0.5, 0.0, 1.0).astype(np.float32)


def main() -> int:
    args = parse_args()

    surface = np.load(args.surface).astype(np.float32)
    if surface.ndim != 2:
        raise SystemExit("Surface array must be 2D.")

    bedrock = None
    if args.bedrock:
        bedrock = np.load(args.bedrock).astype(np.float32)
        if bedrock.shape != surface.shape:
            raise SystemExit("Bedrock shape must match surface shape.")

    land_mask = surface > 0.0

    if args.land_max > 0:
        land_max = float(args.land_max)
    else:
        land_vals = surface[land_mask]
        land_max = float(np.percentile(land_vals, 99.0)) if land_vals.size else 1.0
        land_max = max(land_max, 1.0)

    land_t = np.clip(surface / land_max, 0.0, 1.0)
    land_palette = [
        (0.00, (91, 110, 59)),   # lowland olive
        (0.20, (112, 132, 73)),
        (0.40, (127, 110, 76)),  # foothill tan
        (0.65, (141, 129, 109)), # highland
        (0.85, (170, 170, 170)),
        (1.00, (236, 236, 236)), # peaks
    ]
    land_rgb = interp_palette(land_t, land_palette)

    if bedrock is not None:
        ocean_depth = np.clip(-bedrock, 0.0, float(args.ocean_max_depth))
        ocean_t = np.clip(ocean_depth / float(args.ocean_max_depth), 0.0, 1.0)
    else:
        ocean_t = np.full(surface.shape, 0.55, dtype=np.float32)

    ocean_palette = [
        (0.00, (46, 119, 174)),  # shallow
        (0.45, (30, 92, 150)),
        (1.00, (12, 45, 94)),    # deep
    ]
    ocean_rgb = interp_palette(ocean_t, ocean_palette)

    rgb = np.where(land_mask[..., None], land_rgb, ocean_rgb)

    terrain_for_shade = surface if bedrock is None else np.where(land_mask, surface, bedrock)
    shade = hillshade(
        terrain_for_shade,
        azimuth_deg=float(args.sun_azimuth),
        altitude_deg=float(args.sun_altitude),
        z_scale=float(args.z_scale),
    )

    s = np.clip(float(args.shade_strength), 0.0, 1.0)
    shade_mult = (1.0 - s) + s * shade
    rgb = np.clip(rgb * shade_mult[..., None], 0.0, 1.0)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    img = (rgb * 255.0).astype(np.uint8)
    Image.fromarray(img, mode="RGB").save(out_path)
    print(f"Wrote {out_path} ({surface.shape[1]}x{surface.shape[0]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
