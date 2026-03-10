# X10-F Takeoff Tuning Loop (IFC + VR Probe)

This is the full tuning playbook for deriving and converging takeoff values with FAR + IFC.

Pipeline used:

`boot/ifc_testflight_bootloader.ks -> Integrated Flight Computer/tests/takeoff_vr_probe.ks -> RUN_TAKEOFF_IFC() -> phase_takeoff.ks`

## What You Edit

Primary config:

- `Integrated Flight Computer/aircraft/x10_f_cfg.ks`
  - `"v_r"`
  - `"v2"`
  - `"takeoff_pitch_tgt"`
  - `"takeoff_climb_fpa"`
  - `"a_crit"`

Probe assumptions:

- `Integrated Flight Computer/tests/takeoff_vr_probe.ks`
  - `VRP_ALPHA_CRIT_DEG` (fallback only; probe now auto-reads aircraft `a_crit` when available)
  - `VRP_LOF_FACTOR_ASSUMED` (default `1.08`)

## Part A: Record FAR/VAB Data Explicitly (Before First Run)

In SPH/VAB with FAR static analysis, set the exact takeoff configuration first:

- takeoff mass (fuel/loadout at brake release)
- takeoff flap setting
- gear down
- runway altitude/atmosphere close to test runway
- analysis Mach near expected rotate speed (for X10-F usually around `0.25-0.30`)

Write down these values exactly:

| Symbol | What to record | Units | Source |
|---|---|---|---|
| `m` | Mass at brake release | `kg` | FAR/VAB craft mass (convert from tons if needed) |
| `rho` | Air density at runway | `kg/m^3` | FAR atmosphere readout (or Kerbin sea-level approximation) |
| `S` | Effective wing/reference area | `m^2` | FAR static analysis |
| `CLmax_TO` | Max lift coefficient in takeoff config | unitless | FAR `CL vs AoA` curve |
| `alpha_CLmax` | AoA at `CLmax_TO` | `deg` | FAR `CL vs AoA` curve |
| `alpha_crit` | Chosen critical AoA | `deg` | FAR stall behavior / curve break |
| `M_to` | Analysis Mach used | unitless | FAR static analysis setup |

Notes:

- If mass is shown in tons, convert using `kg = tons * 1000`.
- For KSC low altitude, `rho` is often close to sea-level Kerbin values, but use FAR's number when available.
- If FAR gives slightly different `CLmax` vs Mach, use the more conservative (lower `CLmax`) value.

## Part B: Compute Theoretical First Guess

Use these equations:

- `W = m * g` with `g = 9.80665`
- `Vs_TO = sqrt((2 * W) / (rho * S * CLmax_TO))`

Initial speed seeds:

- `V_LOF_seed = 1.08 * Vs_TO`
- `v_r_seed = 1.03 to 1.08 * Vs_TO` (pick midpoint first: `1.05 * Vs_TO`)
- `v2_seed = 1.15 to 1.25 * Vs_TO` (pick midpoint first: `1.20 * Vs_TO`)

Initial rotation AoA target method:

1. Compute required lift coefficient at liftoff seed speed:
- `CL_req = (2 * W) / (rho * V_LOF_seed^2 * S)`

2. From FAR `CL vs AoA`, read `alpha_req` where `CL ~= CL_req`.

3. Choose a safe initial rotation AoA command:
- `alpha_rot_seed = min(alpha_req + 0.5 to 1.0, alpha_crit - 3.0)`

4. Map to IFC:
- set `"takeoff_pitch_tgt"` close to this seed (clamp to practical range, usually `8-12 deg` for first pass)

Write initial values into `x10_f_cfg.ks`:

- `"v_r"` = `v_r_seed`
- `"v2"` = `v2_seed`
- `"takeoff_pitch_tgt"` = `alpha_rot_seed` (practical clamped value)
- `"takeoff_climb_fpa"` = `8.0` (good default starting point)
- `"a_crit"` = `alpha_crit`

Probe behavior:

- Probe reads `a_crit` from `ACTIVE_AIRCRAFT` automatically during init.
- `VRP_ALPHA_CRIT_DEG` is only a fallback if cfg `a_crit` is missing/invalid.

## Part C: One Iteration Test Loop (Repeat)

1. Keep test conditions controlled
- same runway
- same mass/loadout
- same flap/gear setup
- similar weather/atmosphere

2. Run pipeline
- start `boot/ifc_testflight_bootloader.ks`
- choose runway
- arm once

3. Capture probe outputs
- terminal summary:
  - `Rotate` IAS/AoA
  - `Liftoff` IAS/AoA/VS
  - `Conservative Vs_TO est`
  - `Suggested next V_R`
- csv:
  - `0:/Integrated Flight Computer/logs/vr_probe_<stamp>.csv` (or `_N` suffix if file exists)

4. Update config for next run

- `v_r`:
  - start from probe `Suggested next V_R`
  - limit each change to about `+/- 3 to 5 m/s`

- `v2`:
  - if post-liftoff AoA high / mushy climb: `+2 to +5 m/s`
  - if speed excessive and climb too flat: `-1 to -3 m/s`

- `takeoff_pitch_tgt`:
  - if delayed rotate/liftoff: `+0.5 to +1.0 deg`
  - if AoA spikes near critical: `-0.5 to -1.0 deg`

- `takeoff_climb_fpa`:
  - if speed decays after liftoff: `-0.5 to -1.0 deg`
  - if climb too shallow with extra speed: `+0.5 deg`

5. Repeat until stable
- run at least 3 consistent passes after last parameter change

## Part D: Convergence Criteria (Done Condition)

Stop tuning when all are true for at least 3 runs:

- smooth rotation near commanded `v_r`
- liftoff shortly after rotation
- liftoff AoA margin to critical maintained:
  - `AoA_liftoff <= a_crit - (2 to 4 deg)`
- stable early climb (no porpoise, no sink-back)
- climb speed tracks near `v2`

## Part E: Quick Run Log Template

Use this per run:

| Run | m (kg) | CLmax_TO | a_crit | v_r cmd | v2 cmd | Rotate IAS/AoA | Liftoff IAS/AoA | Probe next v_r | Changes for next run |
|---|---:|---:|---:|---:|---:|---|---|---:|---|

This makes reversions and trend tracking much easier.

---

# Experimental Data - X10-F

| Symbol | Determined Value | Units |
|---|---|---|
| `m` | 99025 | `kg` |
| `rho` | 1.225 | `kg/m^3` |
| `S` | 180 | `m^2` |
| `CLmax_TO` | 0.8 | unitless |
| `alpha_CLmax` | 45 | `deg` |
| `M_to` | 0.3 | unitless |

## Derived Results From These Inputs

Assumptions/constants used:

- `g = 9.80665 m/s^2`
- `W = m * g`
- `Vs_TO = sqrt((2 * W)/(rho * S * CLmax_TO))`

Computed:

- `W = 971,081 N`
- `Vs_TO = 104.93 m/s`
- `V_LOF_seed = 1.08 * Vs_TO = 113.33 m/s`
- `v_r_seed range = 1.03 to 1.08 * Vs_TO = 108.08 to 113.33 m/s`
- `v_r_seed(mid) = 1.05 * Vs_TO = 110.18 m/s`
- `v2_seed range = 1.15 to 1.25 * Vs_TO = 120.67 to 131.16 m/s`
- `v2_seed(mid) = 1.20 * Vs_TO = 125.92 m/s`
- `CL_req at V_LOF_seed = (2 * W)/(rho * V_LOF_seed^2 * S) = 0.686`

## alpha_crit Determination From alpha_CLmax

Since you requested deriving `alpha_crit` from `alpha_CLmax` and no separate stall-break AoA was provided:

- `alpha_crit (provisional) = alpha_CLmax = 45 deg`

Use this as a first-pass placeholder only. Replace it later with measured in-flight stall-break behavior if you gather better data.

## Initial Values To Set Now

Set in `Integrated Flight Computer/aircraft/x10_f_cfg.ks`:

- `"a_crit"` = `45.0`
- `"v_r"` = `110.2`
- `"v2"` = `125.9`
- `"takeoff_pitch_tgt"` = `12.0`
- `"takeoff_climb_fpa"` = `8.0`

Why `takeoff_pitch_tgt = 12.0`:

- Theoretical AoA from the CL curve may imply much higher angles with this dataset.
- For first IFC takeoff runs, a practical clamp is safer; `12 deg` is a stable starting command, then tune by probe results.

Set in `Integrated Flight Computer/tests/takeoff_vr_probe.ks`:

- `VRP_ALPHA_CRIT_DEG = 45.0` (optional fallback; auto-read from cfg is preferred)
- `VRP_LOF_FACTOR_ASSUMED = 1.08` (keep as-is for now)

## Run Log (Start Here)

| Run | m (kg) | CLmax_TO | a_crit | v_r cmd | v2 cmd | pitch tgt | climb fpa | Rotate IAS/AoA | Liftoff IAS/AoA | Probe next v_r | Changes for next run |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---|
| 1 | 99025 | 0.8 | 45.0 | 110.2 | 125.9 | 12.0 | 8.0 |  |  |  | baseline seed run |
