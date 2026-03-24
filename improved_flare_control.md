# Improved Flare Control Plan

## Goal
Build a robust flare controller that consistently achieves target touchdown sink rate without hard touchdowns, while respecting aircraft authority limits and runway geometry.

## Scope
- Replace heuristic flare behavior with a layered controller.
- Keep compatibility with existing IFC phases (`APPROACH -> FLARE -> TOUCHDOWN -> ROLLOUT`).
- Use existing AA Director integration, but improve guidance, authority awareness, and diagnostics.

## Design Principles
1. Separate guidance from control:
   - Guidance decides desired vertical path.
   - Control tracks that path with pitch/AoA/throttle under constraints.
2. Prioritize sink-rate safety over speed bleed in late flare.
3. Detect authority limits explicitly and respond early.
4. Use runway-relative main gear height for all flare/touchdown decisions.
5. Tune in staged increments with measurable acceptance metrics.

## Target Architecture
### 1) Flare Mode Manager
- Add explicit flare submodes:
  - `FLARE_CAPTURE`: transition from ILS descent to flare profile.
  - `FLARE_TRACK`: track sink/flight-path target.
  - `ROUNDOUT`: final sink arrest near touchdown.
  - `TOUCHDOWN_CONFIRM`: robust handoff to touchdown phase.
- Gate mode transitions with hysteresis and debounce.

### 2) Vertical Guidance Generator
- Define reference sink profile as a function of remaining runway-relative height:
  - `w_ref(h)` from entry sink to target touchdown sink.
- Add smooth terminal shape in roundout (no late abrupt commands).
- Remove/disable late-flare "extra sink for speed bleed" behavior.

### 3) VS/FPA Error Feedback
- Compute:
  - `vs_err = w_ref - vs_actual`
  - `fpa_err = gamma_ref - gamma_actual`
- Command flare trajectory with feedback:
  - `gamma_cmd = gamma_ref + K_vs * vs_err + K_fpa * fpa_err`
- Add clamp/rate limit to `gamma_cmd` with anti-windup handling.

### 4) Pitch Command Mapping
- Convert path command to pitch command with AoA estimate:
  - `theta_cmd = gamma_cmd + alpha`
- Keep pitch rate limiter.
- Track and log `theta_cmd - theta_actual` as pitch authority indicator.

### 5) Throttle Coordination
- Add flare-specific throttle policy:
  - Maintain minimum throttle floor in flare for control authority.
  - Avoid full-idle collapse when large sink-rate error persists.
- Optionally blend throttle floor down as touchdown confidence increases.

### 6) Authority/Feasibility Monitor
- Compute required upward acceleration from current state:
  - `a_req = (vs^2 - vs_target^2) / (2*h)` for `h > threshold`.
- Detect authority-limited condition if all true:
  - large `vs_err`,
  - growing `pitch_err`,
  - persistent negative `fpa_err`,
  - low thrust floor hit or AoA near limit.
- Trigger mitigation:
  - earlier roundout,
  - increased throttle floor,
  - optional go-around flag (future).

## Implementation Roadmap
### Phase 0: Instrumentation (already partly done)
- Keep/extend flare diagnostics in CSV:
  - `flare_vs_err`, `flare_fpa_err`, `flare_pitch_err`, `flare_req_up_a`, `flare_thr_floor`.
- Add optional elevator-deflection telemetry probe (tagged parts/modules).

### Phase 1: Safe Baseline Refactor
- Refactor `phase_autoland.ks` flare section into helper functions:
  - `compute_flare_refs()`
  - `compute_gamma_cmd()`
  - `compute_theta_cmd()`
  - `run_flare_throttle_policy()`
- Preserve current behavior first, only code-structure change.

### Phase 2: Guidance + Feedback Upgrade
- Introduce VS/FPA feedback law.
- Disable late-flare speed-bleed sink bias by default.
- Keep conservative clamps and rate limits.

### Phase 3: Throttle Floor and Authority Recovery
- Add configurable flare throttle floor and recovery logic.
- Add authority-limited mitigation branch based on diagnostics.

### Phase 4: Roundout and Touchdown Handoff Hardening
- Improve roundout schedule and touchdown debounce using main gear metrics.
- Ensure touchdown spoilers/reversers do not interfere with flare control before valid touchdown.

### Phase 5: Aircraft Tuning Package (X10-F first)
- Tune gains and limits for X10-F.
- Lock baseline values once repeated landings meet criteria.

## Config Additions (Proposed)
- `flare_vs_kp`
- `flare_fpa_kp`
- `flare_cmd_fpa_min`
- `flare_cmd_fpa_max`
- `flare_min_throttle`
- `flare_min_throttle_agl_blend`
- `flare_authority_vs_err_trigger`
- `flare_authority_pitch_err_trigger`
- `flare_authority_recovery_gain`
- `flare_disable_speed_bleed` (boolean)

## Test Plan
1. Deterministic test flights:
   - Same approach geometry, windless conditions, multiple repeats.
2. Entry condition sweep:
   - Vary flare entry sink and IAS.
3. Failure-mode tests:
   - Low-authority low-speed/high-drag cases.
4. Metrics to track:
   - touchdown `vs_ms`,
   - max `flare_pitch_err`,
   - max `flare_fpa_err`,
   - time in authority-limited state,
   - bounce incidence.

## Acceptance Criteria
- Touchdown sink rate within target envelope (configurable) on repeated runs.
- No hard-impact touchdown in nominal test profile.
- No sustained divergence where `vs_err` grows in final 10 m.
- Stable mode transitions with no flare/ground phase oscillation.

## Immediate Next Step
Implement Phase 1 refactor in `Integrated Flight Computer/phases/phase_autoland.ks`, keeping behavior unchanged, then run one regression test to confirm no functional break before Phase 2 tuning changes.

