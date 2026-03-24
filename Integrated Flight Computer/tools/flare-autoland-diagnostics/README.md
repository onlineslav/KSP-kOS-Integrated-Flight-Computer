# Flare and Autoland Diagnostics Notebook

This notebook is a flare-focused analysis tool for IFC logs.

## What it does

- Loads a selected log (or auto-selects latest `ifc_log_*.csv`)
- Handles IFC footer rows like `# end T+...`
- Builds flare/touchdown derived metrics
- Plots:
  - Full-flight overview (heights, VS, IAS, throttle)
  - Flare/touchdown zoom (height, VS, FPA, pitch/AoA)
  - TECS energy diagnostics (Et/Eb errors, estimated energy levels, gamma decomposition, throttle channel)
  - Vertical-acceleration authority margin (`required up-accel` vs `measured dVS/dt`)
  - Control authority diagnostics (tracking errors, authority latch, throttle floor)
  - Elevator authority channels when available:
    - `flare_pitch_in_cmd`
    - `elev_defl_avg_deg`
    - `elev_defl_max_deg`
    - `elev_defl_n`
- Produces automatic issue flags (early sink arrest, ballooning, persistent tracking miss)

## How to use

1. Open the notebook:
   - `Integrated Flight Computer/tools/flare-autoland-diagnostics/flare_autoland_diagnostics.ipynb`
2. In the `User config` cell:
   - Set `USE_LATEST_LOG = True` to auto-pick newest log, or
   - Set `USE_LATEST_LOG = False` and choose `LOG_FILE_PATH`.
3. Run all cells top-to-bottom.

## Notes

- Older logs without the new elevator columns still work; those plots/checks are skipped automatically.
- Thresholds in `User config` are intended to be tuned per aircraft.
