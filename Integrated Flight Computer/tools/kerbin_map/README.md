# Kerbin Map Toolchain (Offline, High-Resolution)

## Quick Start (viewer only)

If beacons/plates CSVs are already exported, just run the viewer:

```powershell
cd "f:\Kerbal Space Program\Ships\Script\Integrated Flight Computer\tools\kerbin_map\web"
python -m http.server 8080
```

Open `http://localhost:8080`.

To re-export nav data from kOS first (e.g. after adding new beacons/plates):

```kerboscript
RUNPATH("0:/Integrated Flight Computer/tools/kerbin_map/kos/export_ifc_nav_for_web.ks").
```

Then start the server above.

---

This folder provides an end-to-end local pipeline to build your own Kerbin map
and overlay IFC waypoints/approach routes in a browser.

## What you get

1. `python/sample_kerbin_grid_krpc.py`
- Samples Kerbin with kRPC (`surface_height`, optional `bedrock_height`, optional `biome_at`)
- Outputs NumPy grids at any resolution you choose (4k/8k/16k, etc.)

2. `python/render_kerbin_relief.py`
- Renders a shaded relief equirectangular PNG from sampled height data
- Designed for web-map use (lat/lon projection)

3. `kos/export_ifc_nav_for_web.ks`
- Exports live IFC `NAV_BEACON_DB` and `PLATE_IDS` data to CSV for overlay

4. `web/`
- Browser viewer with pan/zoom
- Waypoint markers + labels
- Plate route overlay selection

## Prereqs

- KSP running with kRPC server active (for map sampling)
- Python 3.10+ on your desktop
- Packages in `python/requirements.txt`

Install packages:

```powershell
cd "f:\Kerbal Space Program\Ships\Script\Integrated Flight Computer\tools\kerbin_map\python"
python -m pip install -r requirements.txt
```

## Step 1: Sample Kerbin with kRPC

From `tools/kerbin_map/python`:

```powershell
python sample_kerbin_grid_krpc.py --body Kerbin --width 8192 --height 4096 --bedrock --biome --out-dir ..\web\data\generated --out-stem kerbin_8k
```

Outputs:
- `..\web\data\generated\kerbin_8k_surface_height.npy`
- `..\web\data\generated\kerbin_8k_bedrock_height.npy` (if `--bedrock`)
- `..\web\data\generated\kerbin_8k_biome_idx.npy` (if `--biome`)
- `..\web\data\generated\kerbin_8k_meta.json`

## Step 2: Render web basemap PNG

```powershell
python render_kerbin_relief.py `
  --surface ..\web\data\generated\kerbin_8k_surface_height.npy `
  --bedrock ..\web\data\generated\kerbin_8k_bedrock_height.npy `
  --out ..\web\data\kerbin_map.png
```

Default web viewer expects `web/data/kerbin_map.png`.

## Step 3: Export IFC beacons and plates from kOS

In kOS terminal:

```kerboscript
RUNPATH("0:/Integrated Flight Computer/tools/kerbin_map/kos/export_ifc_nav_for_web.ks").
```

Outputs:
- `0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_beacons.csv`
- `0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_plates.csv`

## Step 4: Run viewer

```powershell
cd "f:\Kerbal Space Program\Ships\Script\Integrated Flight Computer\tools\kerbin_map\web"
python -m http.server 8080
```

Open:
- `http://localhost:8080`

## Notes

- Projection is equirectangular (lat/lon), matching IFC coordinates.
- 8k works well on most systems. 16k may require large RAM/VRAM.
- If map image is missing, the viewer still loads and shows overlays.
