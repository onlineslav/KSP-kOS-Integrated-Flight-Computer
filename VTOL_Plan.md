# VTOL Assist — MVP Implementation Plan

## Overview

AMO is the mode/framework. VTOL Assist is a **module** within it, alongside Ground
Steering Assist. It is also callable by the autopilot (future).

The MVP covers a **4-pod aircraft** (front-left, front-right, rear-left, rear-right)
with one IR servo per pod. The geometry-based mixing matrix handles 3-pod and N-pod
layouts without code changes — the mixing coefficients are derived from part positions.

---

## Tag Conventions

Apply these tags in the KSP editor:

| Tag | Part | Convention |
|-----|------|------------|
| `vtol_eng_1` | Front-left engine part | Numbered left→right, front→back |
| `vtol_eng_2` | Front-right engine part | |
| `vtol_eng_3` | Rear-left engine part | |
| `vtol_eng_4` | Rear-right engine part | |
| IR group `vtol_srv_1` | Front-left servo group | Group name matches engine number |
| IR group `vtol_srv_2` | Front-right servo group | |
| IR group `vtol_srv_3` | Rear-left servo group | |
| IR group `vtol_srv_4` | Rear-right servo group | |

---

## Aircraft Config Keys (`*_cfg.ks`)

```kerboscript
"has_vtol"               = TRUE
"vtol_eng_tag_prefix"    = "vtol_eng"   // → vtol_eng_1, _2, _3, _4
"vtol_srv_group_prefix"  = "vtol_srv"   // → IR group names vtol_srv_1, _2, _3, _4
"vtol_hover_angle"       = 90           // servo degrees for hover (engines pointing down)
"vtol_cruise_angle"      = 0            // servo degrees for cruise (engines pointing forward)
"vtol_yaw_gain"          = 8            // degrees differential servo tilt per unit yaw input (override)
"vtol_vs_kp"             = 0.30         // VS hold proportional gain (override)
"vtol_vs_ki"             = 0.04         // VS hold integral gain (override)
"vtol_max_vs"            = 8.0          // m/s max commanded VS (override)
"vtol_alt_kp"            = 0.40         // alt-hold: m/s VS per meter error (override)
```

---

## File Changes

| File | Change |
|------|--------|
| `lib/ifc_amo_vtol.ks` | **NEW** — all VTOL logic |
| `lib/ifc_constants.ks` | Add VTOL tuning constants after AMO section |
| `lib/ifc_state.ks` | Add VTOL globals + IFC_INIT_STATE resets |
| `lib/ifc_amo.ks` | Refactor `AMO_TICK_PREARM` → dispatcher; extract `_AMO_GSTEER_TICK` |
| `ifc_main.ks` | Add `RUNPATH` for `ifc_amo_vtol.ks` |
| `vtol_test.ks` | **NEW** — standalone test script (root Scripts dir) |

---

## Control Law

### Pilot Input Mapping (AMO/pre-arm mode)

| Axis | Role |
|------|------|
| Main throttle (0–100%) | VS command: 50% = hold, 0% = max descent, 100% = max climb |
| Pitch stick | Fore/aft differential thrust |
| Roll stick | Left/right differential thrust |
| Yaw stick | Differential pod servo tilt (left pods vs right pods toe in/out) |

Physical throttle is **locked to 1.0** in VTOL mode. All collective control is via
per-engine thrust limits. The pilot's throttle axis is re-mapped to VS command.

### Geometry (computed once at discovery)

```
lateral_m[i]  = VDOT(eng_pos[i] - CoM, SHIP:FACING:STARVECTOR)  // + = right
long_m[i]     = VDOT(eng_pos[i] - CoM, SHIP:FACING:FOREVECTOR)  // + = forward

roll_mix[i]   = -lateral_m[i] / max_lat * VTOL_ROLL_GAIN
pitch_mix[i]  = +long_m[i]    / max_lng * VTOL_PITCH_GAIN
yaw_srv_mix[i]= SIGN(lateral_m[i])   // -1=left pod, 0=centerline, +1=right pod
```

For the 4-pod layout:

| Pod | roll_mix | pitch_mix | yaw_srv_mix |
|-----|----------|-----------|-------------|
| Front-Left  | +gain | +gain | −1 |
| Front-Right | −gain | +gain | +1 |
| Rear-Left   | +gain | −gain | −1 |
| Rear-Right  | −gain | −gain | +1 |

### Per-engine throttle limit (each tick)

```
thr[i] = CLAMP(collective + roll_cmd * roll_mix[i] + pitch_cmd * pitch_mix[i], 0, 1)
```

### VS hold (PI controller)

```
vs_cmd      = (pilot_throttle - 0.5) * 2.0 * max_vs
vs_err      = vs_cmd - SHIP:VERTICALSPEED
vs_integral += CLAMP(vs_integral + vs_err * dt, -VS_INTEGRAL_LIM, VS_INTEGRAL_LIM)
collective  = CLAMP(hover_collective + vs_err * VS_KP + vs_integral * VS_KI, 0, 1)
```

`hover_collective` self-learns: slowly tracks `collective` when `|vs_err| < 0.5 m/s`.

### Yaw via differential servo tilt

```
srv_angle[i] = hover_angle + yaw_cmd * yaw_gain * yaw_srv_mix[i]
```

Left pods tilt forward (angle decreases), right pods tilt backward when yaw_cmd > 0.
This creates a horizontal force couple → yaw torque. Centerline pods unchanged.

---

## Module API (ifc_amo_vtol.ks)

| Function | Description |
|----------|-------------|
| `VTOL_DISCOVER()` | Find engines + servos by tag, compute geometry. Safe to call every tick (cached). |
| `VTOL_TICK_PREARM()` | AMO tick — reads pilot inputs, applies mixing. Called from `AMO_TICK_PREARM`. |
| `VTOL_TICK(roll, pitch, yaw, thr)` | Autopilot tick — caller provides all commands. |
| `VTOL_RELEASE()` | Restore engine limits, stop servos, clear active flag. |
| `VTOL_RESET()` | Full state clear (for re-arm or restart). |

---

## AMO Dispatcher Refactor

`AMO_TICK_PREARM` is refactored from a single monolithic function into a dispatcher:

```
FUNCTION AMO_TICK_PREARM {
  // Ground Steering module
  IF _AMO_ENABLED() AND NOT _AMO_HAS_NWS() AND _AMO_ON_GROUND() {
    _AMO_GSTEER_TICK().   // extracted from old AMO_TICK_PREARM body
  } ELSE {
    AMO_RELEASE().
  }
  // VTOL Assist module
  VTOL_TICK_PREARM().
}
```

Each module has its own enable guard and can run independently.

---

## Test Script (vtol_test.ks)

- Loads minimum module stack (constants → state → helpers → autobrake → amo → vtol)
- Accepts hardcoded or config-file aircraft config with `has_vtol = TRUE`
- Establishes `LOCK THROTTLE TO THROTTLE_CMD`, sets `THROTTLE_CMD = 1.0`
- Calls `VTOL_DISCOVER()`, displays geometry table (mixing coefficients per engine)
- Runs 50 Hz loop: calls `VTOL_TICK_PREARM()`, displays live state
- `ON ABORT`: calls `VTOL_RELEASE()`, unlocks throttle, exits

---

## Out of Scope (MVP)

- Kill Horizontal Velocity maneuver
- Transition automation (VTOL → cruise blending)
- Autopilot integration (phase_vtol.ks)
- GUI buttons
- Trim offset computation (assumes symmetric CoM placement)
- Display integration in ifc_display.ks

---

## Future Work

1. **KHV** — tilt pods to oppose horizontal velocity vector, hold until speed < threshold
2. **Transition** — `vtol_fraction` (0=cruise, 1=hover) blends servo angle + throttle mode
3. **Autopilot phase** — `phase_vtol.ks` with `VTOL_TICK(roll_cmd, pitch_cmd, ...)` from guidance
4. **GUI** — KHV button, alt-hold toggle, hover-collective trim adjust
5. **Trim solver** — linear solve for zero-torque throttle distribution at asymmetric CoM
