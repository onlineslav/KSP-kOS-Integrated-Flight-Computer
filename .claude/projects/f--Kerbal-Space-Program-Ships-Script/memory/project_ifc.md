---
name: IFC project overview
description: Integrated Flight Computer codebase structure, architecture patterns, and active work (spaceplane ascent guidance)
type: project
---

The project is a kOS-scripted Integrated Flight Computer for Kerbal Space Program spaceplanes.
Working directory: `f:\Kerbal Space Program\Ships\Script\`
Main entry: `Integrated Flight Computer/ifc_main.ks`

**Architecture:** Phase-based state machine. Phases: TAKEOFF, CRUISE, APPROACH, FLARE, TOUCHDOWN, ROLLOUT, ASCENT (stub), REENTRY.
`phase_ascent.ks` is a complete stub — implementing it is the active project.

**Key patterns:**
- `@LAZYGLOBAL OFF` on every file
- All state in `ifc_state.ks`, all constants in `ifc_constants.ks`
- `AC_PARAM(key, fallback, guard)` for per-aircraft config with -1 sentinel
- `SET_PHASE()` / `SET_SUBPHASE()` for transitions (never write IFC_PHASE directly)
- `THROTTLE_CMD` global + `LOCK THROTTLE TO THROTTLE_CMD` (set once at start)
- `desired_steering` global + `LOCK STEERING TO desired_steering` (phase functions write each cycle)
- `IFC_ACTUAL_DT` = real measured loop dt from `TIME:SECONDS` (never hardcode Δt)
- FAR and AA wrapped behind `FAR_AVAILABLE` / `AA_AVAILABLE` guards
- CSV telemetry via `LOGGER_WRITE()`, rate-limited at `IFC_CSV_LOG_PERIOD = 2.0s`

**Active work:** Implementing full spaceplane ascent-to-orbit guidance per `spaceplane_guidance_paper_v3.md`.
Plan:
1. New `lib/ifc_ascent_state.ks` — frame transforms, dual EMA state estimator, J_ab/J_rk valuation
2. Rewrite `phases/phase_ascent.ks` — 5 sub-phases: AB_CORRIDOR, AB_ZOOM, ROCKET_SUSTAIN, ROCKET_CLOSEOUT, CIRCULARISE
3. New globals in `ifc_state.ks`, new constants in `ifc_constants.ks`
4. Ascent params added to `aircraft_template.ks`

**Why:** User wants measurement-driven (not scheduled) ascent guidance grounded in specific orbital energy per kg propellant (J metric), to work across vehicle designs with minimal per-vehicle tuning.

**How to apply:** When working on ascent code, read spaceplane_guidance_paper_v3.md for the math. Follow the §9.1 build order — validate frame transforms and state estimator before writing any control law.
