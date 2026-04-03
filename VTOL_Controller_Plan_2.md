# VTOL Controller Plan 2 — Complete Redesign from First Principles

**Document status:** Design specification  
**Replaces:** `VTOL_Plan.md` (MVP scaffold — that plan is complete and implemented)  
**Goal:** Identify every architectural gap in the current `ifc_amo_vtol.ks` implementation,
specify the correct control cascade from first principles, and provide an exhaustive
implementation roadmap that can be followed without reference to any other document.

---

## Table of Contents

1. [Executive Summary and Motivation](#1-executive-summary-and-motivation)
2. [Aircraft Configuration and Physical Geometry](#2-aircraft-configuration-and-physical-geometry)
3. [Coordinate Frames and Sign Conventions](#3-coordinate-frames-and-sign-conventions)
4. [Plant Model](#4-plant-model)
   - 4.1 Translational Dynamics
   - 4.2 Rotational Dynamics
   - 4.3 Engine Model (Spool Lag)
   - 4.4 Nacelle/Servo Model
   - 4.5 Atmosphere and Aerodynamics (simplified)
5. [State Estimation and Sensor Model](#5-state-estimation-and-sensor-model)
6. [Control Cascade Architecture — Overview](#6-control-cascade-architecture--overview)
7. [Layer 4 — Rate Control and Actuator Allocation (Innermost)](#7-layer-4--rate-control-and-actuator-allocation-innermost)
   - 7.1 Rate Loop Equations
   - 7.2 Angular Acceleration Damping (D Term)
   - 7.3 Physical Moment Allocation
   - 7.4 Thrust Limiter Conversion
   - 7.5 Saturation and Priority Handling
   - 7.6 Yaw Rate Channel
8. [Layer 3 — Attitude Hold](#8-layer-3--attitude-hold)
   - 8.1 Attitude Error Computation
   - 8.2 Rate Commands from Attitude Error
   - 8.3 Integrator Management
9. [Layer 2 — Velocity and Collective](#9-layer-2--velocity-and-collective)
   - 9.1 Vertical Channel (Altitude / VS Hold)
   - 9.2 Geometry-Correct Collective
   - 9.3 Horizontal Velocity Channel
   - 9.4 Attitude Commands from Horizontal Velocity Error
10. [Layer 1 — Position / Path Guidance](#10-layer-1--position--path-guidance)
    - 10.1 Hover-at-Point
    - 10.2 Kill Horizontal Velocity (KHV)
    - 10.3 Path Following (future)
11. [Spool Lag Compensation (Cross-Cutting)](#11-spool-lag-compensation-cross-cutting)
    - 11.1 Why Spool Lag Breaks Naive Control
    - 11.2 Command Pre-Filtering
    - 11.3 Smith Predictor for the Collective Channel
    - 11.4 Gain Scheduling
    - 11.5 Anti-Windup with Lag Awareness
12. [Transition to Forward Flight](#12-transition-to-forward-flight)
    - 12.1 Scheduling Variable
    - 12.2 Nacelle Angle Schedule
    - 12.3 Collective-to-Throttle Handoff
    - 12.4 Attitude Command Law Blending
    - 12.5 Critical Transition Dynamics
13. [Modes and State Machine](#13-modes-and-state-machine)
14. [Diagnosis of the Current Implementation](#14-diagnosis-of-the-current-implementation)
    - 14.1 What is Already Correct
    - 14.2 Missing: D Term on Rate Error
    - 14.3 Missing: Geometry-Correct Collective
    - 14.4 Missing: Physical Moment Allocation
    - 14.5 Missing: Velocity and Position Loops
    - 14.6 Missing: Spool Lag Compensation in the Rate Loop
15. [New Constants Required](#15-new-constants-required)
16. [New State Variables Required](#16-new-state-variables-required)
17. [Module Contract Changes](#17-module-contract-changes)
18. [Phased Implementation Plan](#18-phased-implementation-plan)
19. [Test Plan](#19-test-plan)
20. [Risk Register](#20-risk-register)

---

## 1. Executive Summary and Motivation

The current VTOL controller (`ifc_amo_vtol.ks`) was built to the MVP specification in
`VTOL_Plan.md`. That specification deliberately deferred several architectural components
to keep the first implementation manageable. Flight testing has shown that the MVP is
not reliably stable, primarily because:

1. **There is no D term on angular rate error.** The inner rate loop uses only a
   proportional gain on rate error. With no damping on angular acceleration, the aircraft
   can build up angular momentum during the spool lag period before any correction arrives.

2. **Collective is not geometry-corrected.** The vertical-speed PI controller targets
   total throttle, but the actual vertical thrust is `T_total × cos(φ) × cos(θ)`. In any
   off-level attitude (bank or pitch excursion during stabilization), the altitude
   controller undershoots and the aircraft sinks.

3. **The mixing matrix uses normalized dimensionless coefficients, not physical moments.**
   This means the "gains" in the allocation are entangled with aircraft geometry and must
   be re-tuned whenever the aircraft changes. More critically, there is no principled way
   to prioritize collective over attitude, or to handle saturation gracefully.

4. **No horizontal velocity hold.** The aircraft has no way to maintain position over a
   ground point without constant pilot input on roll/pitch.

5. **Spool lag compensation exists only in the vertical (VS) channel.** The horizontal
   differential channels — which share the same engines — have no lag-aware feedforward
   or gain scheduling.

The goal of this plan is to rebuild the controller from the rigid-body equations of
motion upward, specifying each layer of the cascade precisely, and identifying exactly
what code changes are needed to implement the correct architecture.

---

## 2. Aircraft Configuration and Physical Geometry

### 2.1 Physical Configuration

The target aircraft has four engine nacelles in a "quadcopter" layout:

```
                          FORWARD
                     _______________
     vtol_eng_1 →  /               \ ← vtol_eng_2
     (Front-Left) /                 \ (Front-Right)
                 /         CoM       \
                 \                   /
     vtol_eng_3 → \                 / ← vtol_eng_4
     (Rear-Left)   \_______________/ (Rear-Right)
                          AFT
```

Each nacelle is mounted at the tip of a wing (engines 1 and 2) or at the tip of the
horizontal stabilizer (engines 3 and 4). Each nacelle has an independent IR servo that
rotates the thrust axis from vertical (hover, ~90°) to horizontal (forward flight, ~0°).
The rotation axis of the servo is lateral (left-right), so tilting the nacelle forward or
backward changes how much thrust points down vs. forward.

### 2.2 Geometric Parameters (computed at VTOL_DISCOVER)

For each engine i, the following are computed once and stored:

| Symbol | Definition | Sign | kOS expression |
|--------|-----------|------|----------------|
| `x_i` | longitudinal distance from CoM | + = forward | `VDOT(eng_pos - SHIP:POSITION, SHIP:FACING:FOREVECTOR)` |
| `y_i` | lateral distance from CoM | + = starboard | `VDOT(eng_pos - SHIP:POSITION, SHIP:FACING:STARVECTOR)` |
| `T_max_i` | maximum rated thrust (kN) | + | `eng:MAXTHRUST` |
| `k_up_i` | spool-up rate constant (1/s) | + | from engine DB |
| `k_dn_i` | spool-down rate constant (1/s) | + | from engine DB |

For the symmetric 4-pod layout the expected values are:
- `x_1 = x_2 = +x_fwd` (engines 1 and 2 are forward of CoM)
- `x_3 = x_4 = -x_aft` (engines 3 and 4 are aft of CoM)
- `y_1 = y_3 = -y_arm` (left/port engines)
- `y_2 = y_4 = +y_arm` (right/starboard engines)

The total moment arm for roll is `2 × y_arm`, and for pitch it is `x_fwd + x_aft`.

### 2.3 Moment of Inertia (estimated, not measured)

kOS does not expose `SHIP:MOI`. The controller must work without it. All rate and
moment calculations use **normalized, dimensionless** units that absorb 1/I implicitly
into the gains. This is acceptable for an empirically-tuned controller. If MOI were
known, it would allow computing physical moments in kN·m and decoupling gains from
inertia completely.

---

## 3. Coordinate Frames and Sign Conventions

All sign conventions used in this document and in the controller code:

### 3.1 Body Frame

Axes are defined by `SHIP:FACING`:

| Axis | kOS vector | Positive direction |
|------|-----------|-------------------|
| X (forward) | `SHIP:FACING:FOREVECTOR` | nose direction |
| Y (right/starboard) | `SHIP:FACING:STARVECTOR` | right wing |
| Z (up) | `SHIP:FACING:TOPVECTOR` | top of aircraft |

### 3.2 Angular Rates (body frame components)

Extracted from `SHIP:ANGULARVEL` (world-frame rad/s) by projecting:

| Rate | Sign convention | kOS expression |
|------|----------------|----------------|
| `p` (roll rate) | positive = roll right (starboard down) | `-VDOT(SHIP:ANGULARVEL, SHIP:FACING:FOREVECTOR)` |
| `q` (pitch rate) | positive = nose up | `VDOT(SHIP:ANGULARVEL, SHIP:FACING:STARVECTOR)` |
| `r` (yaw rate) | positive = nose right | `-VDOT(SHIP:ANGULARVEL, SHIP:FACING:TOPVECTOR)` |

Note: `SHIP:ANGULARVEL` is in world (inertial) frame. The dot products with body
vectors give the components along each body axis. The sign on FOREVECTOR is negative
because angular velocity around the forward axis with the right-hand rule gives
nose-up when thumbs point forward, but roll-right means the angular velocity vector
points rearward from the perspective of an observer looking forward.

**Verification rule:** When the aircraft is level and rolling right (starboard going down),
`p` must be positive. Test in-game before finalizing gains.

### 3.3 Euler Angles

| Angle | Symbol | Positive | kOS derivation |
|-------|--------|---------|---------------|
| Roll (bank) | φ | right wing down | `ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1))` |
| Pitch | θ | nose up | `90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR)` |
| Heading/Yaw | ψ | clockwise from North | `GET_COMPASS_HDG()` |

### 3.4 Moments (torques)

| Moment | Symbol | Positive effect | Physical unit (normalized) |
|--------|--------|----------------|---------------------------|
| Roll moment | L | rolls aircraft right | `[normalized, absorbed into Kp_roll]` |
| Pitch moment | M | pitches nose up | `[normalized, absorbed into Kp_pitch]` |
| Yaw moment | N | yaws nose right | `[via servo differential]` |

The roll moment produced by differential engine thrust is:
```
L = Σ T_i_vertical × (-y_i)
```
where `T_i_vertical` is the vertical component of engine i's thrust, and `y_i` is its
lateral position. Negative sign because positive roll is starboard-down, and a port engine
(y_i < 0) producing more thrust pushes the port side up, rolling starboard down (positive).

The pitch moment is:
```
M = Σ T_i_vertical × x_i
```
Forward engines (x_i > 0) producing more thrust push the nose up (positive pitch). ✓

---

## 4. Plant Model

The plant model describes how the aircraft actually behaves — the physical equations
of motion that the controller must invert to produce desired motion.

### 4.1 Translational Dynamics

In the body-fixed vertical coordinate (NED frame, Z positive downward — but kOS uses
Z positive upward, so we use ENU):

```
m × ẍ = T_fwd - D_fwd             [forward, m/s²]
m × ÿ = T_lat - D_lat             [lateral, m/s²]
m × z̈ = T_vert - m×g - D_vert    [vertical, m/s²]
```

Where:
- `T_fwd, T_lat, T_vert` = components of total thrust vector in world frame (kN)
- `D_fwd, D_lat, D_vert` = aerodynamic drag components (kN, dominated by forward drag)
- `m` = `SHIP:MASS` (t)
- `g` = 9.80665 m/s² at sea level on Kerbin (use `CONSTANT:G × SHIP:BODY:MASS / SHIP:BODY:RADIUS^2` for precision)

For the hover case with nacelles at 90°, `T_fwd ≈ 0`, `T_lat ≈ 0`, and:
```
T_vert = Σ T_i    (sum of all engine thrusts, all pointing down → up on aircraft)
z̈ = T_vert / m - g
```

**Geometry-correction for off-level attitude** — This is critical and currently missing:

When the aircraft is banked by φ and pitched by θ, the engines still point along the
aircraft's Z axis (approximately), but that axis is no longer vertical. The actual vertical
component of total thrust is:

```
T_vert_actual = T_total × cos(φ) × cos(θ)
```

For equilibrium hover: `T_total = m × g / (cos(φ) × cos(θ))`.

At φ = 15° and θ = 5°: `cos(15°) × cos(5°) ≈ 0.961`, so the aircraft needs ~4% more
total thrust than at level. If the VS controller commands `T_total = m×g` (level
assumption), the aircraft will descend at an accelerating rate during any attitude
excursion.

### 4.2 Rotational Dynamics

Euler's equations (body frame, assuming principal axes approximately aligned with body):

```
I_xx × ṗ = L - (I_zz - I_yy) × q × r    [roll]
I_yy × q̇ = M - (I_xx - I_zz) × p × r    [pitch]
I_zz × ṙ = N - (I_yy - I_xx) × p × q    [yaw]
```

The cross-coupling terms `(I_zz - I_yy) × q × r` etc. are called gyroscopic or
inertial coupling. For small rates (hover), these are second-order small and can be
neglected. For the transition phase at higher rates, they matter.

In hover with small angles, the linearized model is:

```
I_xx × ṗ = L_engine              [roll, from thrust differential]
I_yy × q̇ = M_engine              [pitch, from thrust differential]
I_zz × ṙ = N_servo               [yaw, from nacelle differential tilt]
```

This is the model the rate loop must invert. Since I_xx, I_yy, I_zz are unknown,
the inversion absorbs them into empirical gains.

### 4.3 Engine Model (Spool Lag)

Each engine has a first-order lag response to throttle commands. This is the single
most important physical constraint on the controller:

```
dT_i/dt = k_i(cmd) × (T_cmd_i - T_i_actual)
```

Where `k_i` depends on direction:
- `k_i = k_up_i` when `T_cmd_i > T_i_actual` (spooling up)
- `k_i = k_dn_i` when `T_cmd_i < T_i_actual` (spooling down)

Typical values for KSP jet engines (from the engine DB):
- `k_up ≈ 0.2 – 0.5 /s` → τ_up ≈ 2–5 seconds
- `k_dn ≈ 0.2 – 0.4 /s` → τ_dn ≈ 2.5–5 seconds

**This lag is applied to the effective throttle seen by each engine**, which is:
```
throttle_eff_i = THROTTLE_CMD × thrust_limiter_i
```

So when the VTOL controller changes a thrust limiter, the engine's actual output
follows with the spool lag. This is NOT unique to the collective (VS hold) channel —
it applies equally to all differential corrections for roll, pitch, and altitude.

**Time-to-settle formula (from engine model code):**
```
lag_s = -LN(0.01 / |T_cmd - T_actual|) / k_eff
```
This gives the time in seconds to reach 99% of the commanded value. For a 10% differential
command change with k_up = 0.3: `lag_s ≈ -LN(0.01/0.1)/0.3 ≈ 7.7 seconds`.

### 4.4 Nacelle / Servo Model

Each nacelle servo is modeled as a first-order lag:

```
dα_i/dt = k_srv × (α_cmd_i - α_i_actual)
```

The servo speed `k_srv` (effectively 1/τ_srv) depends on the `max speed` field written
to the IR servo. Typical values: servo position settles in 1–3 seconds for a 90° change.

**Key difference from engine model:** Servo lag is much shorter than spool lag. For the
same nominal angular position change, the nacelle responds ~3–5× faster than engine
thrust. This makes nacelle angle the preferred actuator for fast (yaw) control.

The nacelle angle α is defined as:
- α = 90°: nacelle points straight down (full hover)
- α = 0°: nacelle points straight forward (full cruise)
- α > 90°: possible for differential corrections, but mechanically limited

### 4.5 Atmosphere and Aerodynamics (hover mode)

In hover and low-speed flight, aerodynamic drag is:
```
D_fwd ≈ C_D × ½ × ρ × V² × A_ref
```

At hover speeds (< 10 m/s), drag is less than 5% of thrust for most aircraft and can
be treated as a disturbance that the integral terms absorb. For transition at higher
speeds (>40 m/s), drag becomes significant and must be modeled.

FAR aerodynamics (when `FAR_AVAILABLE`) provide forces via `ADDONS:FAR:AEROFORCE`.
These must be rotated from body frame to inertial frame using `SHIP:FACING` before use.

---

## 5. State Estimation and Sensor Model

### 5.1 Available kOS Measurements

| State | kOS expression | Notes |
|-------|---------------|-------|
| Roll angle φ | `ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1))` | degrees |
| Pitch angle θ | `90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR)` | degrees |
| Heading ψ | `GET_COMPASS_HDG()` | degrees, 0–360 |
| Roll rate p | `-VDOT(SHIP:ANGULARVEL, SHIP:FACING:FOREVECTOR) × 180/π` | deg/s |
| Pitch rate q | `VDOT(SHIP:ANGULARVEL, SHIP:FACING:STARVECTOR) × 180/π` | deg/s |
| Yaw rate r | `-VDOT(SHIP:ANGULARVEL, SHIP:FACING:TOPVECTOR) × 180/π` | deg/s |
| Vertical speed | `SHIP:VERTICALSPEED` | m/s, + = up |
| Altitude AGL | `GET_AGL()` | m, uses terrain |
| Airspeed | `GET_IAS()` | m/s |
| Ground velocity | `SHIP:VELOCITY:SURFACE` | m/s, inertial-ish |
| Mass | `SHIP:MASS` | tonnes |
| Engine thrust | `eng:THRUST` | kN (actual) |
| Engine spool state | `EM_GET_SPOOL_AT(i)` | normalized 0–1 |
| Nacelle position | `srv_mod:GETFIELD("current position")` | degrees |

### 5.2 Filtering Strategy

**Angular rates** are subject to high-frequency vibration noise from engines and gear.
All rate signals must be passed through an EMA filter before use in the D term:

```
p_filt = α_rate × p_raw + (1 - α_rate) × p_filt_prev
```

Recommended `α_rate = 0.6 – 0.8` (balance between noise rejection and lag).
Lower alpha = smoother but adds lag. The D term from a noisy unfiltered rate is
worse than no D term at all.

**Angular acceleration** (for the D term in the rate loop) is the derivative of filtered
rate:
```
p_dot = (p_filt - p_filt_prev) / IFC_ACTUAL_DT
```
This must also be filtered (α_accel = 0.3–0.5) as it amplifies noise significantly.

**Vertical speed** from `SHIP:VERTICALSPEED` is already somewhat filtered by the physics
engine but still contains noise. Apply an EMA with α_vs = 0.5.

**Engine spool states** from `EM_GET_SPOOL_AT(i)` are already model outputs (low noise).
Use directly.

### 5.3 CoM Shift Awareness

As fuel burns, the CoM shifts. The geometric moment arms `x_i`, `y_i` computed at
`VTOL_DISCOVER` become stale. For long hover operations:
- The adaptive trim integrator (already implemented) compensates for pitch CoM shift
- A periodic re-computation of engine positions (every 60 seconds) would improve accuracy
- This is deferred to a future enhancement

---

## 6. Control Cascade Architecture — Overview

The controller is organized as four concentric loops. Each outer loop runs at the same
50 Hz tick rate as the inner loops, but has lower bandwidth (achieved by smaller gains
and explicit rate-limiting). The key design rule is:

> **Each layer's bandwidth must be at least 3–5× lower than the layer inside it.**

This separation ensures that the inner loop has "settled" before the outer loop
computes its next command, which is the condition required for cascade stability.

### 6.1 Layer Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 1: Position / Path (0.05–0.1 Hz bandwidth)           │
│  Input:  desired hover position, waypoint, or KHV           │
│  Output: desired horizontal velocity commands [vN_cmd, vE_cmd]│
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Layer 2: Velocity + Collective (0.2–0.5 Hz bandwidth)      │
│  Input:  [vN_cmd, vE_cmd] + [vs_cmd or alt_cmd]             │
│  Output: desired attitude angles [φ_cmd, θ_cmd]             │
│          + geometry-correct collective T_total_cmd           │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Layer 3: Attitude Hold (0.5–2 Hz bandwidth)                │
│  Input:  [φ_cmd, θ_cmd, ψ_cmd]                              │
│  Output: desired angular rate commands [p_cmd, q_cmd, r_cmd] │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Layer 4: Rate Control + Allocation (50 Hz, fastest)        │
│  Input:  [p_cmd, q_cmd, r_cmd] + T_total_cmd                │
│  Output: per-engine thrust limiters + per-nacelle angles    │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
                    Physical Actuators:
                    - 4× thrust limiters (spool lag 2–5 s)
                    - 4× IR servo angles (servo lag ~1–2 s)
```

### 6.2 Bandwidth Targets

Given engine spool time constant τ_spool ≈ 3–5 seconds (k ≈ 0.2–0.33 /s):

| Layer | Bandwidth target | Period | Justification |
|-------|-----------------|--------|---------------|
| Rate loop (L4) | 0.05–0.10 Hz | 10–20 s | Limited by spool lag; P+D on rate |
| Attitude (L3) | 0.02–0.05 Hz | 20–50 s | 3–5× slower than L4 |
| Velocity (L2) | 0.005–0.01 Hz | 100–200 s | 3–5× slower than L3 |
| Position (L1) | 0.002–0.005 Hz | 200–500 s | 3–5× slower than L2 |

These are very slow compared to a typical rotorcraft. This is the fundamental physical
constraint of using slow-spool jet engines. **The system must be designed to accept
slow response rather than fight the physics.**

The yaw channel (servo-driven) can achieve higher bandwidth (0.2–0.5 Hz) because
servo lag is much shorter than spool lag. Yaw and the other axes should be tuned
independently.

### 6.3 What Happens Without This Separation

If the attitude loop (L3) tries to command rate changes faster than the spool lag allows
thrust to respond (L4), the following happens:
1. L3 commands a rate change
2. L4 commands a thrust differential
3. The engines spool slowly — no immediate response
4. L3 sees no response, grows its error, drives commands harder
5. 3–5 seconds later, the commanded thrust arrives — but L3 has over-commanded
6. Overshoot in the opposite direction, then oscillation

The bandwidth separation prevents this by ensuring L3 never asks for rate changes
faster than L4 can reliably deliver.

---

## 7. Layer 4 — Rate Control and Actuator Allocation (Innermost)

Layer 4 runs every tick at 50 Hz. It receives rate commands from Layer 3 and collective
from Layer 2, and produces per-engine thrust limiter commands and per-nacelle angles.

### 7.1 Rate Loop Equations

Given:
- `p_cmd` = desired roll rate (deg/s), from Layer 3
- `q_cmd` = desired pitch rate (deg/s), from Layer 3
- `r_cmd` = desired yaw rate (deg/s), from Layer 3

Measured (filtered):
- `p_filt` = filtered actual roll rate (deg/s)
- `q_filt` = filtered actual pitch rate (deg/s)
- `r_filt` = filtered actual yaw rate (deg/s)

Rate errors:
```
p_err = p_cmd - p_filt
q_err = q_cmd - q_filt
r_err = r_cmd - r_filt
```

**Normalized moment commands** (dimensionless, scaled to [-1, 1]):
```
L_cmd = Kp_roll  × p_err  +  Kd_roll  × (-p_dot_filt)
M_cmd = Kp_pitch × q_err  +  Kd_pitch × (-q_dot_filt)
```

Where `p_dot_filt` and `q_dot_filt` are filtered angular accelerations.
The negative sign on the D term: if the aircraft is already rolling right (positive p),
`p_dot_filt` could be positive (accelerating), and we want to resist that with a
negative moment → `-Kd × p_dot` is negative, correctly opposing the acceleration.

Clamp:
```
L_cmd = CLAMP(L_cmd, -VTOL_RATE_CMD_ROLL_MAX, VTOL_RATE_CMD_ROLL_MAX)
M_cmd = CLAMP(M_cmd, -VTOL_RATE_CMD_PITCH_MAX, VTOL_RATE_CMD_PITCH_MAX)
```

### 7.2 Angular Acceleration Damping (the D Term — Currently Missing)

This is the most important missing element. Without it, the rate loop is a pure P
controller. With a slow actuator (spool lag), a pure P controller allows angular
momentum to build unchecked until the commanded thrust eventually arrives — by
which time it's too much and the aircraft overshoots.

The angular acceleration D term:
```
p_dot_raw   = (p_filt - p_filt_prev) / IFC_ACTUAL_DT
p_dot_filt  = α_accel × p_dot_raw + (1 - α_accel) × p_dot_filt_prev
```

**Implementation note:** `p_dot` can be very large on the first tick (if there's a
discontinuity). Guard with a clamp:
```
p_dot_filt = CLAMP(p_dot_filt, -VTOL_RATE_ACCEL_CLAMP, VTOL_RATE_ACCEL_CLAMP)
```

The D term gain `Kd_roll` should be tuned to approximately:
```
Kd_roll ≈ τ_spool × Kp_roll / 2
```
This places the zero of the PD controller at 1/(2τ_spool), providing damping in the
same frequency range as the spool lag's pole.

### 7.3 Physical Moment Allocation

**This replaces the current mixing matrix approach.**

The current approach:
```
limit[i] = collective × (1 + trim) + roll_cmd × roll_mix[i] + pitch_cmd × pitch_mix[i]
```
This is a normalized linear combination where `roll_mix[i]` and `pitch_mix[i]` are
dimensionless coefficients derived from geometry. While functional, it has two
problems:
1. The collective and differential terms are coupled — changing `collective` changes
   the base from which `roll_mix` acts.
2. There is no principled priority ordering: if saturation occurs, there is no
   guarantee that collective is preserved over attitude.

**The correct approach uses additive decomposition in physical units (or consistently
normalized units), with collective as the base and moments as additive corrections:**

```
// Step 1: Baseline thrust command per engine (collective, equal distribution)
T_base = T_total_cmd / N_engines    // normalized 0-1: VTOL_COLLECTIVE / N_engines

// Step 2: Roll differential correction
//   To produce moment L_cmd (positive = roll right), we need:
//   MORE thrust on port engines (y_i < 0), LESS on starboard (y_i > 0)
//   delta_T_roll_i = L_cmd × roll_authority_i
//   roll_authority_i = -y_i / sum_i(y_i²)  [physically: minimizes ΔT magnitude]
//
//   For the symmetric 4-pod layout:
//   delta_T_roll_i = L_cmd × (-y_i) / (2 × y_arm²)

// Step 3: Pitch differential correction
//   To produce moment M_cmd (positive = nose up):
//   MORE thrust on forward engines (x_i > 0), LESS on aft (x_i < 0)
//   delta_T_pitch_i = M_cmd × x_i / sum_i(x_i²)
//
//   For the symmetric 4-pod layout:
//   delta_T_pitch_i = M_cmd × x_i / (2 × (x_fwd² + x_aft²)/2) 
//                   = M_cmd × x_i / (x_fwd² + x_aft²)

// Step 4: Sum
T_cmd_i = T_base + delta_T_roll_i + delta_T_pitch_i
```

**For the symmetric 4-pod layout, expanding:**

```
T_cmd_FL = VTOL_COLLECTIVE/4 + L_cmd/(4 × y_arm) + M_cmd/(2 × (x_fwd + x_aft))
T_cmd_FR = VTOL_COLLECTIVE/4 - L_cmd/(4 × y_arm) + M_cmd/(2 × (x_fwd + x_aft))
T_cmd_AL = VTOL_COLLECTIVE/4 + L_cmd/(4 × y_arm) - M_cmd/(2 × (x_fwd + x_aft))
T_cmd_AR = VTOL_COLLECTIVE/4 - L_cmd/(4 × y_arm) - M_cmd/(2 × (x_fwd + x_aft))
```

Verification:
- `T_FL + T_FR + T_AL + T_AR = VTOL_COLLECTIVE` ✓ (differential sums to zero)
- Roll moment: `(T_FL + T_AL) × y_arm - (T_FR + T_AR) × y_arm = L_cmd/2 × y_arm × 2/y_arm = L_cmd` ✓
- Pitch moment: `(T_FL + T_FR) × x_fwd - (T_AL + T_AR) × x_aft ≈ M_cmd` (approx for equal x_fwd ≈ x_aft)

**In the general (asymmetric) case**, pre-compute the allocation matrix at VTOL_DISCOVER:

```
// Allocation matrix A (3 × N_engines):
// Row 0: [1, 1, ..., 1]                     (collective)
// Row 1: [-y_1, -y_2, ..., -y_N]            (roll moment)
// Row 2: [x_1, x_2, ..., x_N]              (pitch moment)

// Minimum-norm pseudoinverse A_pinv = A^T × (A × A^T)^(-1)
// T_cmd_vec = A_pinv × [T_total_cmd; L_cmd; M_cmd]
```

For 4 engines (overdetermined — 3 constraints, 4 unknowns), the pseudoinverse gives the
minimum-thrust-deviation solution. Compute A_pinv once at VTOL_DISCOVER and cache it.

kOS does not have built-in matrix operations, so for the symmetric 4-pod case, use the
closed-form solution above. For asymmetric layouts, implement a simple 3×4 pseudoinverse
using the known rank-3 structure.

**Priority when saturated:**

If any `T_cmd_i > 1.0` or `T_cmd_i < 0` after adding roll/pitch corrections, there is
a conflict. Priority rule (in order):
1. Preserve total collective (don't let the aircraft drop)
2. Preserve the larger-magnitude of roll or pitch command
3. Scale down the smaller-magnitude command

Implementation: if the solution is infeasible, find the maximum `α ∈ [0,1]` such that
`T_cmd_i = T_base + α × (delta_T_roll_i + delta_T_pitch_i)` satisfies `T_cmd_i ∈ [floor, 1]`
for all i. This is the binary search approach already in the current `_VTOL_ALLOC_SHIFT_BOUNDS`
function. The key change is that the inputs to that function are now physically derived
moment commands, not scaled mixing coefficients.

### 7.4 Thrust Limiter Conversion

The per-engine thrust command `T_cmd_i` (normalized 0–1, where 1.0 = full engine thrust)
is converted to a thrust limiter:

```
IF THROTTLE_CMD > 0.001 {
  thrust_limiter_i = CLAMP(T_cmd_i / THROTTLE_CMD, 0, 1)
} ELSE {
  thrust_limiter_i = 0
}
```

Note: `THROTTLE_CMD = 1.0` in VTOL mode (the physical throttle is locked high; the pilot
throttle axis is re-mapped to VS command). Therefore `thrust_limiter_i = T_cmd_i` in
VTOL mode. This simplification is valid only when THROTTLE_CMD is known to be 1.0.

### 7.5 Saturation and Priority Handling

**Engine limit floor:** Never set any engine to 0 when in flight. A minimum floor
(e.g., 15% or `VTOL_ENGINE_LIMIT_FLOOR`) prevents complete engine shutdown during
aggressive corrections. The floor also ensures the engine stays warm and can spool up
quickly.

**Limit span diagnostics:** Track `max(limits) - min(limits)` as `VTOL_DIAG_LIMIT_SPAN`.
A value > 0.7 indicates the controller is operating near saturation and should be logged.

**Clamp counts:** Track `VTOL_DIAG_CLAMP_HIGH_COUNT` and `VTOL_DIAG_CLAMP_LOW_COUNT`.
More than one saturated engine per tick is a sign that the attitude or rate commands
exceed what the actuators can deliver at the current collective level.

### 7.6 Yaw Rate Channel

Yaw control uses nacelle angle differential. For a desired yaw rate `r_cmd`:

```
r_err = r_cmd - r_filt
yaw_nacelle_cmd = Kp_yaw × r_err + Kd_yaw × (-r_dot_filt)
```

The yaw differential is applied via servo angles:
```
// yaw_srv_mix[i]: -1 for port (left) nacelles, +1 for starboard (right) nacelles
α_cmd_i = α_collective_i + yaw_nacelle_cmd × VTOL_YAW_SRV_GAIN × yaw_srv_mix[i]
```

When `yaw_nacelle_cmd > 0` (yaw nose right):
- Starboard (right) nacelles: α decreases (tilt more forward) → creates forward thrust on right
- Port (left) nacelles: α increases (tilt more backward) → creates backward thrust on left
- Net couple: forward force on right, backward on left → yaw right ✓

**Yaw command limits:**
```
yaw_nacelle_cmd = CLAMP(yaw_nacelle_cmd, -VTOL_YAW_CMD_MAX, VTOL_YAW_CMD_MAX)
```

**Yaw bandwidth:** Servo lag (~0.5–2s) allows the yaw channel to achieve 2–3× the
bandwidth of the roll/pitch channels. The yaw D term `Kd_yaw` should be tuned to
`τ_servo × Kp_yaw / 2`, analogous to the roll/pitch D term.

---

## 8. Layer 3 — Attitude Hold

Layer 3 takes desired attitude angles and produces rate commands for Layer 4.

### 8.1 Attitude Error Computation

```
phi_err   = phi_cmd - phi_actual       [roll, degrees]
theta_err = theta_cmd - theta_actual   [pitch, degrees]
psi_err   = WRAP_180(psi_cmd - psi_actual)  [yaw, degrees, wrap to ±180]
```

Sign check: if `phi_err > 0` (commanded roll is to the right of current), the
aircraft needs to roll right → `p_cmd` should be positive ✓

**Small-angle approximation:** For hover operations within ±30° of level, the
approximation θ̇ ≈ q and φ̇ ≈ p is valid. Outside this range, the full kinematic
equations should be used, but this is beyond the scope of hover stabilization.

### 8.2 Rate Commands from Attitude Error

**Proportional only (no integral in L3):**

```
p_cmd = Kp_att_roll  × phi_err      [deg/s]
q_cmd = Kp_att_pitch × theta_err    [deg/s]
r_cmd = Kp_att_yaw   × psi_err      [deg/s]
```

**Why no integral in Layer 3?**

Steady-state attitude errors are handled by Layer 2's velocity integrators:
- If the aircraft has a persistent pitch offset, the velocity loop will adjust the
  commanded pitch attitude until the error is zero.
- An integral in L3 would compete with L2's integral and create an unnecessary
  double-integration problem.

**Rate command limits (the critical pre-filter for spool lag):**

```
p_cmd = CLAMP(p_cmd, -VTOL_ATT_P_CMD_MAX_DEGS, VTOL_ATT_P_CMD_MAX_DEGS)
q_cmd = CLAMP(q_cmd, -VTOL_ATT_Q_CMD_MAX_DEGS, VTOL_ATT_Q_CMD_MAX_DEGS)
r_cmd = CLAMP(r_cmd, -VTOL_ATT_R_CMD_MAX_DEGS, VTOL_ATT_R_CMD_MAX_DEGS)
```

The rate limit values must satisfy:
```
VTOL_ATT_P_CMD_MAX_DEGS < VTOL_RATE_CMD_ROLL_MAX / Kp_roll
```
to prevent L3 from saturating L4. If L3 commands a rate that exceeds what L4 can
produce (given the spool lag), L4 saturates and L3's integral would wind up.
Since L3 has no integral, this only causes clipping — which is acceptable.

**Lag-aware scaling of rate limits** (already partially implemented):

When `em_lag_s` is large (engines spooling), reduce the rate command limits:
```
lag_scale = CLAMP(VTOL_ATT_GAIN_LAG_REF_S / em_lag_s, min_scale, 1.0)
p_cmd = CLAMP(p_cmd, -VTOL_ATT_P_CMD_MAX_DEGS × lag_scale, ...)
```

### 8.3 Integrator Management

Layer 3 has no integrators itself. The state that must be reset on mode changes:
- The attitude error history (used only to detect upset)
- The rate filter states (`p_filt`, `q_filt`, `r_filt`) should NOT be reset when
  entering attitude hold — they represent real aircraft motion.

**Attitude command seeding:** When the pilot engages attitude hold, seed `phi_cmd` and
`theta_cmd` from the current aircraft attitude to prevent a step input.

---

## 9. Layer 2 — Velocity and Collective

Layer 2 is the most complex layer. It handles two distinct jobs:
1. **Vertical:** translating VS/altitude commands into a geometry-correct collective thrust
2. **Horizontal:** translating velocity commands into attitude commands

### 9.1 Vertical Channel (Altitude / VS Hold)

This is a refinement of the existing `_VTOL_VS_HOLD` function.

The current code:
```
collective = hover_collective + vs_err × Kp + integral × Ki
```

The correct geometry-aware version:

```
// Target vertical acceleration from VS error
az_cmd = vs_kp × (vs_cmd - vs_actual) + vs_integral × vs_ki   [m/s²]

// Convert to required total thrust, geometry-corrected
// Note: at hover, cos(phi) × cos(theta) ≈ 1 for small angles
// At 15° bank: ≈ 0.96 (need 4% more thrust)
cos_att = COS(phi_actual) × COS(theta_actual)
IF ABS(cos_att) < 0.2 { SET cos_att TO 0.2. }  // floor: 78° max tilt

T_total_cmd = SHIP:MASS × (9.806 + az_cmd) / cos_att   [kN]

// Convert to normalized collective [0, 1]
T_total_max = sum of all engine max thrusts [kN]
VTOL_COLLECTIVE = CLAMP(T_total_cmd / T_total_max, 0, VTOL_COLLECTIVE_MAX)
```

The geometry correction `1/cos(phi)×cos(theta)` ensures that when the aircraft is in
an attitude excursion (e.g., roll stabilization applied while flying), the collective
automatically increases to compensate for the reduced vertical component. **This
eliminates the "sink when rolling" problem.**

**VS command from throttle:**
```
vs_cmd = (p_thr - 0.5) × 2.0 × VTOL_MAX_VS    [m/s]
```
No change from current — this mapping is correct.

**Alt-hold:**
```
IF VTOL_ALT_HOLD {
  vs_cmd = CLAMP((VTOL_ALT_CMD - GET_AGL()) × alt_kp, -VTOL_MAX_VS, VTOL_MAX_VS)
}
```

### 9.2 Geometry-Correct Collective — Implementation Details

The geometry correction introduces a nonlinearity when the aircraft is in a large
attitude excursion. This can cause "thrust hunting" if the attitude correction produces
large φ swings. Guard:

1. Low-pass filter `cos_att` to avoid throttle chattering during rapid attitude changes:
   ```
   cos_att_filt = α_cos × cos_att_raw + (1 - α_cos) × cos_att_filt_prev
   ```
   Use `α_cos = 0.3` (slow, to prevent the collective from chasing attitude oscillations).

2. Cap the correction: never command more than 150% of hover collective due to geometry
   correction alone. If the aircraft is banked beyond ~50°, the VTOL system cannot
   maintain altitude regardless — flag an UPSET condition.

### 9.3 Horizontal Velocity Channel

This is the new component needed for hover-at-point and KHV.

**Inputs:**
- `vN_cmd` = desired northward velocity (m/s)
- `vE_cmd` = desired eastward velocity (m/s)
- `vN_actual` = `VDOT(SHIP:VELOCITY:SURFACE, north_vec)` [m/s]
- `vE_actual` = `VDOT(SHIP:VELOCITY:SURFACE, east_vec)` [m/s]

**North and east vectors in kOS:**
```
LOCAL north_vec IS VCRS(SHIP:UP:VECTOR, VCRS(SHIP:BODY:POSITION, SHIP:UP:VECTOR):NORMALIZED):NORMALIZED.
LOCAL east_vec  IS VCRS(SHIP:UP:VECTOR, north_vec).
```
Alternatively, use `IFC_NORTH_VEC` and `IFC_UP_VEC` if they are already computed in
the main loop.

**Velocity PI controller:**
```
vN_err = vN_cmd - vN_actual
vE_err = vE_cmd - vE_actual

// Integrate velocity error (very slow, for steady winds and CoM offsets)
IF |vN_err| < VTOL_VEL_INT_DEADBAND { vel_int_N += vN_err × dt }
IF |vE_err| < VTOL_VEL_INT_DEADBAND { vel_int_E += vE_err × dt }
vel_int_N = CLAMP(vel_int_N, -VTOL_VEL_INT_LIM, VTOL_VEL_INT_LIM)
vel_int_E = CLAMP(vel_int_E, -VTOL_VEL_INT_LIM, VTOL_VEL_INT_LIM)

ax_cmd = Kp_vel × vN_err + Ki_vel × vel_int_N   [m/s²]
ay_cmd = Kp_vel × vE_err + Ki_vel × vel_int_E   [m/s²]
```

**Rate-limit horizontal acceleration commands:**
```
ax_cmd = CLAMP(ax_cmd, -VTOL_MAX_HORIZ_ACCEL, VTOL_MAX_HORIZ_ACCEL)
ay_cmd = CLAMP(ay_cmd, -VTOL_MAX_HORIZ_ACCEL, VTOL_MAX_HORIZ_ACCEL)
```
`VTOL_MAX_HORIZ_ACCEL` limits how aggressively the aircraft pitches/banks to chase
velocity, which prevents large attitude excursions during KHV.

### 9.4 Attitude Commands from Horizontal Velocity Error

The geometry relating horizontal acceleration to attitude angles (for a hover vehicle):

```
// To accelerate forward (ax_cmd > 0):
//   The aircraft must pitch nose-forward (theta < 0 in our convention where + = nose up)
//   a_forward = T_total/m × sin(-theta) ≈ T_total/m × (-theta_rad) for small angles
//   => theta_cmd = -arctan(ax_cmd / (g + az_cmd))  [radians → degrees]

// To accelerate rightward (ay_cmd > 0):
//   The aircraft must bank right (phi > 0)
//   a_lateral = T_total/m × sin(phi) ≈ T_total/m × phi_rad for small angles
//   => phi_cmd = arctan(ay_cmd / (g + az_cmd))  [radians → degrees]

// In kOS (degrees):
LOCAL az_cmd IS (VTOL_COLLECTIVE × T_total_max / SHIP:MASS) - 9.806.
LOCAL denom IS MAX(0.5, 9.806 + az_cmd).  // floor: avoid divide by near-zero on aggressive descent
LOCAL theta_cmd IS -ARCTAN(ax_cmd / denom) × (180 / CONSTANT:PI)
LOCAL phi_cmd   IS  ARCTAN(ay_cmd / denom) × (180 / CONSTANT:PI)
```

Clamp attitude commands to safe limits:
```
theta_cmd = CLAMP(theta_cmd, -VTOL_MAX_FWD_PITCH, VTOL_MAX_FWD_PITCH)
phi_cmd   = CLAMP(phi_cmd,   -VTOL_MAX_BANK,       VTOL_MAX_BANK)
```

**In pilot-control (AMO) mode**, the attitude commands come from the pilot stick
directly, and the velocity loop is inactive. The connection is:

```
IF VTOL_VEL_HOLD_ACTIVE {
  theta_cmd = theta_cmd_vel_loop     // from velocity loop
  phi_cmd   = phi_cmd_vel_loop
} ELSE {
  theta_cmd = p_pitch × VTOL_MAX_FWD_PITCH   // pilot stick → angle command
  phi_cmd   = p_roll  × VTOL_MAX_BANK
}
```

**Converting from body-aligned attitude to world-aligned attitude command:**

The pitch and roll computed above are in terms of world-frame acceleration. The attitude
command sent to Layer 3 must be in the body frame Euler angles. For small angles at
zero heading, these are the same. For arbitrary heading, transform via:

```
// Desired force vector in world frame: [ax_cmd, ay_cmd, g + az_cmd]
// The attitude command is the direction of this vector
theta_cmd_world = -ARCTAN(ax_cmd / (g + az_cmd))   // pitch = tilt in N direction
phi_cmd_world   =  ARCTAN(ay_cmd / (g + az_cmd))   // roll = tilt in E direction
```

At hover, the heading doesn't change the vertical plane, so this is correct. For
forward-flight transitions where heading matters, the transformation to body-frame
Euler angles requires multiplication by the heading rotation matrix.

---

## 10. Layer 1 — Position / Path Guidance

Layer 1 is the outermost and slowest loop. It converts a desired position (or mode
like KHV) into velocity commands for Layer 2.

### 10.1 Hover-at-Point

**Input:** `VTOL_TARGET_LAT`, `VTOL_TARGET_LNG` (geographic coordinates)

```
// Position error in meters (approximate, valid within ~500 m)
LOCAL target_geo IS LATLNG(VTOL_TARGET_LAT, VTOL_TARGET_LNG).
LOCAL dist_m IS GEO_DISTANCE(SHIP:GEOPOSITION, target_geo).
LOCAL bearing IS GEO_BEARING(SHIP:GEOPOSITION, target_geo).

// Convert to N/E displacement
LOCAL err_N IS dist_m × COS(bearing)   [meters]
LOCAL err_E IS dist_m × SIN(bearing)   [meters]

// P+I control: position error → velocity command
LOCAL vN_cmd IS Kp_pos × err_N + Ki_pos × pos_int_N
LOCAL vE_cmd IS Kp_pos × err_E + Ki_pos × pos_int_E
vN_cmd = CLAMP(vN_cmd, -VTOL_MAX_HORIZ_SPEED, VTOL_MAX_HORIZ_SPEED)
vE_cmd = CLAMP(vE_cmd, -VTOL_MAX_HORIZ_SPEED, VTOL_MAX_HORIZ_SPEED)
```

The integral term removes steady-state offset due to wind. However, the integrator
must only run when the aircraft is close to the target (within `VTOL_POS_INT_RADIUS`)
to prevent windup during approach.

**Approach speed schedule:** Reduce velocity command as the aircraft nears the target:
```
vN_cmd = vN_cmd × MIN(1.0, dist_m / VTOL_POS_CAPTURE_RADIUS)
```
This creates a smooth deceleration as the aircraft arrives at the target.

### 10.2 Kill Horizontal Velocity (KHV)

KHV is a simpler version of hover-at-point that ignores position and targets zero
surface velocity:

```
vN_cmd = 0    // target: zero surface velocity in N direction
vE_cmd = 0    // target: zero surface velocity in E direction
```

The velocity loop (Layer 2) does the rest. Once surface velocity is below
`VTOL_KHV_CAPTURE_MPS`, engage alt-hold and set KHV as completed.

KHV is the prerequisite for hover-at-point: fly KHV first to stop, then activate
hover-at-point to hold position.

### 10.3 Path Following (Future)

Not implemented in this plan. The architecture accommodates it: path following simply
computes `[vN_cmd, vE_cmd]` from the cross-track error and along-track progress,
feeding Layer 2 the same way hover-at-point does.

---

## 11. Spool Lag Compensation (Cross-Cutting)

This section documents the spool lag problem and all mitigation strategies in full.
Understanding this section is essential for correct gain tuning.

### 11.1 Why Spool Lag Breaks Naive Control

Consider the rate loop with a proportional gain only (the current state):

**Scenario:** Aircraft develops a 5 deg/s roll rate error (rolling right unexpectedly).

1. Rate loop computes: `L_cmd = Kp_roll × 5 = 0.15` (positive = commands roll-right
   corrective moment, wait, this corrects by increasing port engines)
2. Engine throttle command increases port engines, decreases starboard engines by ~3%
3. Engines begin spooling at k_up = 0.3 /s
4. After 1 second: 26% of the commanded differential has been achieved → 0.78% thrust
   differential → tiny corrective moment
5. The 5 deg/s rate has continued for 1 second → 5 degrees of bank has developed
6. The 5-degree bank generates additional gravitational roll torque (gravity component:
   `g × sin(5°) ≈ 0.85 m/s²` laterally → destabilizing torque)
7. The rate loop now sees a growing error, commands more correction, which takes more
   spool time...

Without the D term on angular acceleration: there is no signal that the rate error
is persisting or growing. The P term acts on instantaneous error, but by the time the
correction arrives, the error has grown.

**With the D term on angular acceleration:**

If the aircraft is building up roll rate at 2 deg/s², `p_dot = +2 deg/s²`.
The D term contributes `-Kd × 2 = -0.04` (opposing the acceleration).
This acts immediately (D term is computed from sensor data, not actuator state), and
while it doesn't change how fast the engines spool, it pre-emptively sends a correction
command that accounts for the developing rate, buying the spool response time to
work against the actual developing error rather than the lagged past error.

### 11.2 Command Pre-Filtering

**The most important single technique for spool lag tolerance.**

For any commanded signal `cmd` going to an actuator with lag τ, the maximum useful
rate of change of the command is:

```
|d(cmd)/dt| ≤ 1 / τ   [normalized units per second]
```

If you change the command faster than this, the actuator cannot track and the error
keeps growing. The actuator essentially ignores the detail of the fast-changing command.

**Implementation:** Apply a slew rate limiter to all commands entering the rate loop:

```
// Command slew for roll
LOCAL roll_cmd_slew_max IS VTOL_RATE_CMD_SLEW_PER_S × lag_scale
LOCAL roll_step IS roll_cmd_slew_max × IFC_ACTUAL_DT
SET L_cmd TO CLAMP(L_cmd, VTOL_L_CMD_PREV - roll_step, VTOL_L_CMD_PREV + roll_step)
SET VTOL_L_CMD_PREV TO L_cmd
```

Similarly for pitch. The lag scale reduces the slew rate when spool lag is large:
```
lag_scale = CLAMP(VTOL_RATE_SLEW_LAG_REF_S / em_lag_s, VTOL_RATE_SLEW_MIN_SCALE, 1.0)
```

**Effect:** This prevents the rate loop from toggling between positive and negative
large commands during the spool lag period (which causes the "bang-bang" oscillation
that makes the aircraft thrash without settling).

### 11.3 Smith Predictor for the Collective Channel

The VS hold PI controller regulates `vs_actual` around `vs_cmd`. But the actuator
(engine collective thrust) has a spool lag of τ seconds. The standard PI controller
"sees" the error too late.

A Smith Predictor restructures the loop to close feedback around the predicted future
output rather than the current output:

```
// Engine model: first-order lag
// Predicted collective after dt with current spool state:
T_filt_filt = T_filt_filt + (VTOL_COLLECTIVE - T_filt_filt) × (1/τ) × dt
// T_filt_filt is the predicted actual collective at any future time

// Predicted vertical speed in lag_s seconds:
az_predicted = (T_filt_filt × T_total_max / SHIP:MASS) - 9.806
vs_predicted = vs_actual + az_predicted × em_lag_s

// Close the feedback loop on vs_predicted, not vs_actual:
vs_err = vs_cmd - vs_predicted    // error against predicted future state
integral_update = vs_err × dt     // integrator sees the prediction, not the measurement
```

This allows the integrator to "look ahead" and stop accumulating error before the
commanded thrust arrives. Without it, the integral continues to ramp up during the
spool lag period and causes overshoot.

**Note:** The current code approximates this with `VTOL_COLLECTIVE_EFF_EST` and
feedforward inversion. The Smith Predictor formulation above is more theoretically
correct and should replace the current approach.

### 11.4 Gain Scheduling

All gains must be scheduled as a function of current spool lag. The principle:

> When actuator bandwidth is reduced (high lag), reduce controller gains proportionally
> to maintain the same gain margin.

For any gain `K`:
```
K_effective = K_nominal × CLAMP(LAG_REF_S / em_lag_s, min_scale, 1.0)
```

Where `LAG_REF_S` is the lag value at which the nominal gain was tuned (typically the
steady-state lag at hover collective). When `em_lag_s > LAG_REF_S`, gains reduce.
When `em_lag_s ≤ LAG_REF_S` (fully spooled), gains are at full value.

**Gain scheduling must apply to all loops, not just VS hold:**

| Loop | Gains to schedule | Current state |
|------|-----------------|---------------|
| Rate loop (L4) | `Kp_roll`, `Kd_roll`, `Kp_pitch`, `Kd_pitch` | **MISSING** |
| Attitude loop (L3) | `Kp_att_roll`, `Kp_att_pitch` | Partially done |
| VS hold (L2 vert) | `Kp_vs`, `Ki_vs` | Done |
| Rate cmd slew | slew rate limits | Done (partial) |

**The most impactful missing item:** Gain scheduling in the rate loop (L4) does not
currently exist. When spool lag is high (engines just started or heavily throttled up),
the rate loop gains should reduce by the same factor as the VS hold gains.

### 11.5 Anti-Windup with Lag Awareness

All integrators must use lag-aware anti-windup to prevent the "windup during spool lag,
overshoot on engagement" failure mode.

Standard anti-windup condition (saturation only):
```
IF cmd_saturated AND err_same_sign_as_integral { FREEZE integral }
```

Extended lag-aware anti-windup:
```
IF cmd_saturated OR alloc_limited OR (em_lag_s > AW_LAG_S AND eff_err > AW_EFF_ERR_MIN) {
  integral × = (1 - unwind_rate × dt)    // exponential decay
} ELSE {
  integral += err × dt
}
```

The `eff_err` condition (effective collective error: `|VTOL_COLLECTIVE - VTOL_COLLECTIVE_EFF_EST|`)
catches the case where the integrator is driving a command that the engines have not
yet delivered. This prevents the integral from building up during a spool-up transient.

---

## 12. Transition to Forward Flight

Transition from hover to forward flight is the most complex maneuver in the flight
envelope. This section specifies the architecture, not the implementation (which
is in a future phase).

### 12.1 Scheduling Variable

The primary scheduling variable is the **nacelle collective angle** `α_avg`:
```
α_avg = average of all nacelle α_est[i]    [degrees, 0 = forward, 90 = hover]
```

This is preferable to airspeed as the scheduling variable because:
1. It directly controls the ratio of hover-to-cruise force
2. It is commanded (predictable), while airspeed is a consequence (lagged)
3. It handles the case where the aircraft hovers at speed (cross-wind)

A secondary blend variable is `hover_blend = sin(α_avg)`, ranging from 1.0 (pure
hover) to 0.0 (pure cruise), with a smooth cosine transition.

### 12.2 Nacelle Angle Schedule

The nacelle collective angle commands all four servos to the same target angle (modulo
yaw differential). During transition:

```
α_cmd_i = α_collective + yaw_differential[i] + pitch_differential[i]
```

Where `α_collective` is the flight-phase commanded angle:
- Hover: `α_collective = vtol_hover_angle` (typically 90°)
- Transition: commanded between 90° and 0° on a schedule
- Cruise: `α_collective = vtol_cruise_angle` (typically 0°)

**The transition schedule** is a function of airspeed (with `α_avg` as the command):

```
IF IAS < VTOL_TRANS_START_IAS {
  α_collective = vtol_hover_angle    // stay in hover
} ELSE IF IAS > VTOL_TRANS_END_IAS {
  α_collective = vtol_cruise_angle   // fully transitioned
} ELSE {
  LOCAL t IS (IAS - VTOL_TRANS_START_IAS) / (VTOL_TRANS_END_IAS - VTOL_TRANS_START_IAS)
  α_collective = vtol_hover_angle + t × (vtol_cruise_angle - vtol_hover_angle)
}
```

The slew rate on `α_collective` must be limited to the servo capability:
```
α_cmd_slew_max = VTOL_NACELLE_SLEW_DEGS_PER_S × IFC_ACTUAL_DT
```

### 12.3 Collective-to-Throttle Handoff

In hover (`α_avg = 90°`), the physical throttle is locked at 1.0 and all collective
control is via thrust limiters.

In cruise (`α_avg = 0°`), the thrust limiters should be at 1.0 (equal) and collective
is controlled via `THROTTLE_CMD` through the autothrottle system.

During transition, there is a handoff between these two control modes. The blend:

```
// hover_blend = sin(α_avg), ranges 1.0 (hover) to 0.0 (cruise)
THROTTLE_CMD = hover_blend × 1.0 + (1 - hover_blend) × AT_THROTTLE_CMD
thrust_limiter_i (collective component) = 
  hover_blend × VTOL_COLLECTIVE + (1 - hover_blend) × 1.0
```

The transition also affects the vertical vs. forward thrust split:
```
T_vertical_i = T_total_i × SIN(α_est_i)   // lift component
T_forward_i  = T_total_i × COS(α_est_i)   // forward thrust component
```

As α → 0°, the vertical component vanishes. The aircraft must be generating aerodynamic
lift to compensate. This requires sufficient airspeed to generate lift at the current
pitch angle.

**Minimum airspeed constraint:** Do not command α < α_min until aerodynamic lift
exceeds the thrust reduction. For a first approximation:
```
lift = 0.5 × rho × IAS² × Cl × wing_area
lift_fraction = lift / (SHIP:MASS × 9.806)
α_min = ARCCOS(CLAMP(lift_fraction, 0, 1))   // [degrees]
α_collective = MAX(α_min, commanded_α)
```

### 12.4 Attitude Command Law Blending

In hover, pitch attitude commands horizontal acceleration (Section 9.4):
```
theta_cmd_hover = -ARCTAN(ax_cmd / g)   // nose-down to go forward
```

In cruise, pitch attitude sets angle of attack for lift:
```
theta_cmd_cruise = target_pitch_for_speed   // from autothrottle / airspeed hold
```

The blend:
```
theta_cmd = hover_blend × theta_cmd_hover + (1 - hover_blend) × theta_cmd_cruise
```

At intermediate nacelle angles, the aircraft is partially a helicopter (thrust
vectoring controls position) and partially a fixed-wing (pitch controls AoA for lift).
The blend provides a smooth transition between these modes.

### 12.5 Critical Transition Dynamics

**Energy conservation:** During transition, the nacelle tilts forward, converting
hover (vertical) thrust into cruise (forward) thrust. The aircraft must accelerate
forward fast enough that aerodynamic lift builds up before vertical thrust drops too much.
If the nacelle tips forward too quickly at low airspeed, the aircraft will descend.

**Collective feedforward during transition:** Apply a collective feedforward that
anticipates the vertical thrust reduction:
```
T_collective_transition_ff = VTOL_COLLECTIVE / SIN(MAX(α_avg, 0.1°))
```
This increases the collective command as nacelles tilt forward, compensating for the
reduced vertical component of thrust.

**Roll control during transition:** As α approaches 0°, the roll moment arm from
thrust differential becomes zero (nacelles pointing forward, no vertical component to
create a roll couple). Roll control must transition to aerodynamic surfaces (ailerons)
or to servo-based rolling. This is a hard constraint: the VTOL roll controller must
disengage before `α < α_roll_min`.

---

## 13. Modes and State Machine

The VTOL controller operates in the following modes. Mode transitions are triggered
by pilot inputs or automated conditions:

```
┌──────────┐     arm      ┌────────────┐
│ INACTIVE │──────────────▶│   HOVER    │
│          │              │  STABILIZE │
└──────────┘              └─────┬──────┘
                                │ enable vel hold
                    ┌───────────▼──────────┐
                    │     HOVER HOLD        │
                    │  (vel loop active)    │
                    └───────────┬──────────┘
                                │ activate KHV
                    ┌───────────▼──────────┐
                    │  KILL HORIZ VEL       │
                    │  (target vel = 0)     │
                    └───────────┬──────────┘
                                │ KHV complete
                    ┌───────────▼──────────┐
                    │  HOVER AT POINT       │
                    │  (pos loop active)    │
                    └───────────┬──────────┘
                                │ transition command
                    ┌───────────▼──────────┐
                    │   TRANSITION         │
                    │  (nacelle schedule)   │
                    └───────────┬──────────┘
                                │ α < 10°
                    ┌───────────▼──────────┐
                    │  FORWARD FLIGHT       │
                    │  (AT + IFC phases)    │
                    └──────────────────────┘
```

**Per mode, which layers are active:**

| Mode | L4 Rate | L3 Attitude | L2 Velocity | L1 Position |
|------|---------|-------------|-------------|-------------|
| HOVER STABILIZE | ✓ | ✓ (hold level) | ✓ (vert only) | ✗ |
| HOVER HOLD | ✓ | ✓ | ✓ (vert + horiz) | ✗ |
| KHV | ✓ | ✓ | ✓ | target=0 |
| HOVER AT POINT | ✓ | ✓ | ✓ | ✓ |
| TRANSITION | ✓ | ✓ (blend) | ✓ (blend) | ✗ |
| FORWARD FLIGHT | ✗ (AT) | ✗ (AA) | ✗ | ✗ (IFC) |

---

## 14. Diagnosis of the Current Implementation

This section maps the idealized architecture from Sections 6–11 against the current
`ifc_amo_vtol.ks` code, identifying exactly what is correct and what is missing.

### 14.1 What is Already Correct

**Inner rate loop structure (L4):**
The code in `VTOL_TICK_PREARM` (lines ~1290–1390) implements:
```
rate_cmd = att_err × att2rate_kp + integral × att2rate_ki    // L3
cmd = (rate_cmd - rate_actual) × rate_kp                      // L4 P term
```
This is the correct cascade structure (angle → rate command → rate error → command).

**Spool lag gain scheduling:**
All of `level_roll_kp`, `level_roll_kd`, `level_roll_ki`, `roll_att2rate_kp`, and
`roll_rate_kp` are scheduled against `em_lag_s` via `level_gain_scale`. ✓

**VS hold PI with anti-windup:**
The `_VTOL_VS_HOLD` function correctly identifies saturation, alloc-limited, and
lag-limited conditions and applies integral windup prevention. ✓

**Feedforward inversion on collective:**
The `ff_enabled` branch in `_VTOL_VS_HOLD` inverts the first-order engine model to
pre-compensate for spool lag. This is a partial Smith Predictor. ✓

**Adaptive trim:**
`_VTOL_ADAPT_TRIM` slowly integrates pitch and roll attitude to update engine base
offsets. This handles CoM shifts and asymmetric thrust. ✓

**Allocation solver:**
`_VTOL_ALLOC_SHIFT_BOUNDS` and the binary search for `alpha` handle the case where
full differential is not feasible under engine limits. ✓

**Upset detection:**
The VTOL_UPSET_ACTIVE flag and associated hysteresis logic correctly identifies large
attitude/rate excursions and applies conservative rate-only damping. ✓

### 14.2 Missing: D Term on Rate Error (Highest Priority)

**Where:** Layer 4 rate loop.

**Current code (lines ~1301, ~1354):**
```kerboscript
LOCAL roll_d_term IS -roll_rate_degs * roll_rate_kp.
LOCAL roll_unsat IS (roll_rate_cmd - roll_rate_degs) * roll_rate_kp.
```

`roll_unsat` = `rate_cmd × Kp - rate_actual × Kp`.

The `roll_d_term` is computed as a diagnostic value but it IS effectively the same as
the Kp term when `rate_cmd = 0`. There is no separate gain for rate derivative (angular
acceleration). The rate error is `(rate_cmd - rate_actual)`, and the command is
`rate_error × rate_kp`. There is no `p_dot_filt × Kd` term anywhere.

**What to add:**
- Track `p_filt_prev`, `q_filt_prev` per-tick
- Compute `p_dot = (p_filt - p_filt_prev) / dt` with an EMA filter
- Add `roll_d_contribution = -p_dot_filt × Kd_roll_accel` to the roll command
- Add separate constants `VTOL_RATE_KD_ROLL_ACCEL` and `VTOL_RATE_KD_PITCH_ACCEL`
- Add per-aircraft config keys `"vtol_rate_kd_roll_accel"` and `"vtol_rate_kd_pitch_accel"`

**New state variables required:**
- `VTOL_RATE_P_FILT_PREV`, `VTOL_RATE_Q_FILT_PREV`
- `VTOL_RATE_P_DOT_FILT`, `VTOL_RATE_Q_DOT_FILT`

### 14.3 Missing: Geometry-Correct Collective (High Priority)

**Where:** `_VTOL_VS_HOLD` function.

**Current code (line ~848):**
```kerboscript
LOCAL unclamped IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp_use + VTOL_VS_INTEGRAL * vs_ki_use.
```

The `VTOL_HOVER_COLLECTIVE` term is the learned steady-state collective. The PI
controller adjusts around it. However, there is no correction for the current attitude.
When the aircraft is banked or pitched, the vertical thrust is reduced and the PI must
integrate down (error correction) rather than feedforward-correcting.

**What to add:**
- Compute `cos_att_filt = alpha_cos × cos(phi) × cos(theta) + (1-alpha_cos) × cos_att_filt_prev`
- Divide the total collective output by `cos_att_filt` before clamping:
  ```kerboscript
  LOCAL cos_att IS CLAMP(cos_att_filt, 0.3, 1.0).
  LOCAL collective_geom_corrected IS CLAMP(target_collective / cos_att, 0, coll_max).
  ```
- The `cos_att_filt` EMA should use α_cos ≈ 0.15 (very slow, to prevent chasing
  attitude oscillations in the altitude channel)

**New state variables required:**
- `VTOL_COS_ATT_FILT` (init to 1.0 in IFC_INIT_STATE)

### 14.4 Missing: Physical Moment Allocation (Medium Priority)

**Where:** `_VTOL_APPLY_ENGINES` function.

**Current allocation:**
```kerboscript
LOCAL diff_i IS diff_scale × (roll_cmd × VTOL_ROLL_MIX[i] + pitch_cmd × VTOL_PITCH_MIX[i]).
```

`VTOL_ROLL_MIX[i]` and `VTOL_PITCH_MIX[i]` are precomputed as:
```kerboscript
VTOL_ROLL_MIX:ADD(CLAMP(-lat / max_lat, -1, 1) × t_scale × roll_gain)
VTOL_PITCH_MIX:ADD(CLAMP(pitch_mix_sign × lng / max_lng, -1, 1) × t_scale × pitch_gain)
```

This normalizes by `max_lat` and `max_lng` respectively, so the coefficients are
dimensionless (range [-gain, +gain]). The gains `vtol_roll_gain` and `vtol_pitch_gain`
absorb the geometry scaling.

**The problem:** Because `roll_gain` and `pitch_gain` are separate from the physical
moment arms, there is no guarantee that:
1. Full roll authority equals full pitch authority in terms of moment
2. The correct priority ordering holds when both channels saturate simultaneously

**What to add (Phase 2 change):**
- Pre-compute physical moment arms `arm_roll` (m) and `arm_pitch` (m) at VTOL_DISCOVER
- Store `VTOL_ARM_ROLL_M` and `VTOL_ARM_PITCH_M` as state
- In `_VTOL_APPLY_ENGINES`, compute differential in physical units:
  ```kerboscript
  LOCAL delta_T_roll_i IS L_cmd × (-y_i) / (2 × VTOL_ARM_ROLL_M²)
  LOCAL delta_T_pitch_i IS M_cmd × x_i / (2 × VTOL_ARM_PITCH_M²)
  ```
  Where `L_cmd` and `M_cmd` are the outputs of the rate loop in physical-normalized units.

Note: This requires re-scaling `L_cmd` and `M_cmd` from the rate loop to match the
physical unit convention. A clean way to do this is to express moment commands in
"fraction of max differential" (which is what the current code does) and compute the
arm lengths only to determine priority weighting, not the command magnitudes.

### 14.5 Missing: Velocity and Position Loops (Medium Priority — Future Phase)

The current code has no horizontal velocity feedback. In pilot (AMO) mode, the pilot's
stick commands differential thrust directly, interpreted as pitch/roll attitude. There
is no way for the code to:
- Arrest horizontal drift automatically
- Hold position over a ground point
- Execute a KHV stop

This requires implementing Sections 9.3, 9.4, and 10.1–10.2.

**New state variables required:**
- `VTOL_VEL_INT_N`, `VTOL_VEL_INT_E` (velocity integrators)
- `VTOL_POS_INT_N`, `VTOL_POS_INT_E` (position integrators)
- `VTOL_TARGET_LAT`, `VTOL_TARGET_LNG`, `VTOL_TARGET_ALT`
- `VTOL_VEL_HOLD_ACTIVE`, `VTOL_POS_HOLD_ACTIVE`, `VTOL_KHV_ACTIVE`
- `VTOL_VN_CMD`, `VTOL_VE_CMD` (velocity commands, for telemetry)

### 14.6 Missing: Rate Loop Gain Scheduling in the D Term (Low Priority)

**Current code:** The gain scheduling `level_gain_scale` applies to `roll_att2rate_kp`
and `roll_rate_kp`, but there is no D term gain to schedule. Once the D term is added
(Section 14.2), `Kd_roll_accel` must also be included in the gain scheduling:
```kerboscript
SET kd_roll_accel_use TO kd_roll_accel × CLAMP(level_gain_scale, kd_min_scale, 1.0).
```

---

## 15. New Constants Required

All new constants go in `ifc_constants.ks`. Naming prefix: `VTOL_`.

### Layer 4 Rate Loop Additions

```kerboscript
GLOBAL VTOL_RATE_KD_ROLL_ACCEL      IS 0.0.  // D term gain on roll angular acceleration
GLOBAL VTOL_RATE_KD_PITCH_ACCEL     IS 0.0.  // D term gain on pitch angular acceleration
GLOBAL VTOL_RATE_P_ALPHA            IS 0.7.  // EMA alpha for roll rate filter
GLOBAL VTOL_RATE_Q_ALPHA            IS 0.7.  // EMA alpha for pitch rate filter
GLOBAL VTOL_RATE_PDOT_ALPHA         IS 0.4.  // EMA alpha for roll accel filter
GLOBAL VTOL_RATE_QDOT_ALPHA         IS 0.4.  // EMA alpha for pitch accel filter
GLOBAL VTOL_RATE_ACCEL_CLAMP_DEGS2  IS 300.0. // max angular accel to avoid D-term spike on init
```

### Layer 2 Geometry Correction Additions

```kerboscript
GLOBAL VTOL_COS_ATT_ALPHA     IS 0.12.  // EMA alpha for attitude cosine filter (very slow)
GLOBAL VTOL_COS_ATT_FLOOR     IS 0.25.  // minimum cos(phi)×cos(theta) (= max ~75° tilt)
```

### Layer 2 Horizontal Velocity Loop (for Phase 2)

```kerboscript
GLOBAL VTOL_VEL_KP              IS 0.15.  // m/s² per m/s velocity error
GLOBAL VTOL_VEL_KI              IS 0.005. // m/s² per (m/s × s) velocity integral
GLOBAL VTOL_VEL_INT_LIM         IS 2.0.   // m/s × s — integral clamp
GLOBAL VTOL_VEL_INT_DEADBAND    IS 0.5.   // m/s — don't integrate when within deadband
GLOBAL VTOL_MAX_HORIZ_ACCEL     IS 1.5.   // m/s² max commanded horizontal acceleration
GLOBAL VTOL_MAX_HORIZ_SPEED     IS 8.0.   // m/s max horizontal speed command from pos loop
GLOBAL VTOL_MAX_FWD_PITCH       IS 15.0.  // deg max forward pitch command for horizontal accel
GLOBAL VTOL_MAX_BANK            IS 15.0.  // deg max bank command for horizontal accel
```

### Layer 1 Position Loop (for Phase 3)

```kerboscript
GLOBAL VTOL_POS_KP              IS 0.08.  // (m/s) per meter position error
GLOBAL VTOL_POS_KI              IS 0.002. // (m/s) per (m × s)
GLOBAL VTOL_POS_INT_LIM         IS 3.0.   // m × s — integral clamp
GLOBAL VTOL_POS_INT_RADIUS      IS 50.0.  // m — only integrate within this radius
GLOBAL VTOL_POS_CAPTURE_RADIUS  IS 10.0.  // m — approach deceleration radius
GLOBAL VTOL_KHV_CAPTURE_MPS     IS 0.5.   // m/s — KHV success threshold
```

### Transition Constants (for Phase 4)

```kerboscript
GLOBAL VTOL_TRANS_START_IAS     IS 30.0.  // m/s — begin nacelle tilt at this speed
GLOBAL VTOL_TRANS_END_IAS       IS 80.0.  // m/s — fully tilted at this speed
GLOBAL VTOL_NACELLE_SLEW_DPS    IS 5.0.   // deg/s max nacelle angle change rate
GLOBAL VTOL_NACELLE_ALPHA_MIN   IS 10.0.  // deg floor for nacelle angle (never point fully forward unless aero lift available)
```

---

## 16. New State Variables Required

All new state variables go in `ifc_state.ks` under a `// --- VTOL Phase 2+ ---` section
and must be reset in `IFC_INIT_STATE()`.

### Phase 1 (D term and geometry correction)

```kerboscript
// Rate filter states
GLOBAL VTOL_RATE_P_FILT       IS 0.0.   // filtered roll rate (deg/s)
GLOBAL VTOL_RATE_Q_FILT       IS 0.0.   // filtered pitch rate (deg/s)
GLOBAL VTOL_RATE_P_FILT_PREV  IS 0.0.   // previous filtered roll rate
GLOBAL VTOL_RATE_Q_FILT_PREV  IS 0.0.   // previous filtered pitch rate
GLOBAL VTOL_RATE_P_DOT_FILT   IS 0.0.   // filtered roll angular acceleration (deg/s²)
GLOBAL VTOL_RATE_Q_DOT_FILT   IS 0.0.   // filtered pitch angular acceleration (deg/s²)

// Attitude geometry correction
GLOBAL VTOL_COS_ATT_FILT      IS 1.0.   // filtered cos(phi)×cos(theta) for collective

// Moment commands (for telemetry and anti-windup)
GLOBAL VTOL_L_CMD_PREV        IS 0.0.   // previous roll moment command (for slew limiting)
GLOBAL VTOL_M_CMD_PREV        IS 0.0.   // previous pitch moment command (for slew limiting)
```

### Phase 2 (Horizontal velocity hold)

```kerboscript
GLOBAL VTOL_VEL_INT_N         IS 0.0.   // northward velocity integral
GLOBAL VTOL_VEL_INT_E         IS 0.0.   // eastward velocity integral
GLOBAL VTOL_VN_ACTUAL         IS 0.0.   // filtered north velocity (m/s)
GLOBAL VTOL_VE_ACTUAL         IS 0.0.   // filtered east velocity (m/s)
GLOBAL VTOL_VN_CMD            IS 0.0.   // commanded north velocity (m/s)
GLOBAL VTOL_VE_CMD            IS 0.0.   // commanded east velocity (m/s)
GLOBAL VTOL_VEL_HOLD_ACTIVE   IS FALSE. // velocity hold mode enabled
GLOBAL VTOL_KHV_ACTIVE        IS FALSE. // kill horizontal velocity mode active
GLOBAL VTOL_PHI_CMD           IS 0.0.   // roll command from velocity loop (deg)
GLOBAL VTOL_THETA_CMD         IS 0.0.   // pitch command from velocity loop (deg)
```

### Phase 3 (Position hold)

```kerboscript
GLOBAL VTOL_POS_INT_N         IS 0.0.   // northward position integral
GLOBAL VTOL_POS_INT_E         IS 0.0.   // eastward position integral
GLOBAL VTOL_TARGET_LAT        IS 0.0.   // target hover latitude (deg)
GLOBAL VTOL_TARGET_LNG        IS 0.0.   // target hover longitude (deg)
GLOBAL VTOL_TARGET_ALT        IS 0.0.   // target hover altitude AGL (m)
GLOBAL VTOL_POS_HOLD_ACTIVE   IS FALSE. // position hold mode enabled
```

### Phase 4 (Transition)

```kerboscript
GLOBAL VTOL_NACELLE_ALPHA_CMD IS 90.0.  // collective nacelle angle command (deg, 90=hover)
GLOBAL VTOL_NACELLE_ALPHA_EST IS 90.0.  // estimated actual nacelle angle (averaged)
GLOBAL VTOL_HOVER_BLEND       IS 1.0.   // 1.0 = full hover, 0.0 = full cruise
GLOBAL VTOL_TRANS_ACTIVE      IS FALSE. // transition mode active
```

### Diagnostic additions (Phase 1)

```kerboscript
GLOBAL VTOL_DIAG_P_DOT_FILT   IS 0.0.
GLOBAL VTOL_DIAG_Q_DOT_FILT   IS 0.0.
GLOBAL VTOL_DIAG_ROLL_D_ACCEL IS 0.0.
GLOBAL VTOL_DIAG_PITCH_D_ACCEL IS 0.0.
GLOBAL VTOL_DIAG_COS_ATT_FILT IS 1.0.
GLOBAL VTOL_DIAG_COLL_BEFORE_CORR IS 0.0.
GLOBAL VTOL_DIAG_COLL_AFTER_CORR  IS 0.0.
```

---

## 17. Module Contract Changes

### ifc_amo_vtol.ks — Updated Contracts

**New reads:**
- `SHIP:ANGULARVEL` (already read; now also used for filtered rate + derivative)
- `SHIP:MASS` (new, for geometry-correct collective)
- `SHIP:VELOCITY:SURFACE` (new, for velocity hold in Phase 2)
- `IFC_NORTH_VEC`, `IFC_UP_VEC` (used in Phase 2 — must be set in main loop)

**New writes (to ifc_state.ks globals):**
- `VTOL_RATE_P_FILT`, `VTOL_RATE_Q_FILT`, `VTOL_RATE_P_FILT_PREV`, `VTOL_RATE_Q_FILT_PREV`
- `VTOL_RATE_P_DOT_FILT`, `VTOL_RATE_Q_DOT_FILT`
- `VTOL_COS_ATT_FILT`
- `VTOL_L_CMD_PREV`, `VTOL_M_CMD_PREV`
- All Phase 2 state variables when Phase 2 is implemented

**Function contracts (changes to existing functions):**

`_VTOL_VS_HOLD(thr_input)`:
- Now reads `phi_actual` and `theta_actual` (passed as parameters or read from cache)
- Applies geometry correction via `VTOL_COS_ATT_FILT`
- Returns geometry-corrected collective (breaking change: callers see different value)

`VTOL_TICK_PREARM()` internal wings-level controller:
- Now computes `p_dot_filt` and `q_dot_filt` from filtered rate differences
- Adds D term: `roll_d_accel = -p_dot_filt × kd_roll_accel_use`
- The final roll command becomes: `roll_unsat + roll_d_accel`

**New function: `_VTOL_UPDATE_RATE_FILTERS(ang_vel, starvec, forevec, upvec)`**
- Computes `p_filt`, `q_filt`, `p_dot_filt`, `q_dot_filt` from `_angvel`
- Must be called once at the top of each tick, before the rate loop
- Reads `VTOL_RATE_P_ALPHA`, `VTOL_RATE_PDOT_ALPHA` (configurable per aircraft)

### ifc_state.ks

- All new globals listed in Section 16 must be added under clearly labelled sections
- `IFC_INIT_STATE()` must reset each new global to its default
- Group by phase: `// --- VTOL Phase 1 (D term, geometry correction) ---`, etc.

### ifc_constants.ks

- All new constants listed in Section 15 added after the existing VTOL section
- Group by phase with comments

---

## 18. Phased Implementation Plan

The implementation is split into four phases, each buildable and testable independently.
**Phase 1 is the priority** — it addresses the two highest-impact missing components.

---

### Phase 1: D Term + Geometry Correction (Immediate)

**Goal:** Stabilize the aircraft and fix the altitude-sink-during-roll problem.

**Files changed:** `ifc_amo_vtol.ks`, `ifc_state.ks`, `ifc_constants.ks`

**Step 1.1 — Add rate filter state and update function**

In `ifc_state.ks`, add (under `// --- VTOL Phase 1 ---`):
```kerboscript
GLOBAL VTOL_RATE_P_FILT      IS 0.0.
GLOBAL VTOL_RATE_Q_FILT      IS 0.0.
GLOBAL VTOL_RATE_P_FILT_PREV IS 0.0.
GLOBAL VTOL_RATE_Q_FILT_PREV IS 0.0.
GLOBAL VTOL_RATE_P_DOT_FILT  IS 0.0.
GLOBAL VTOL_RATE_Q_DOT_FILT  IS 0.0.
GLOBAL VTOL_L_CMD_PREV       IS 0.0.
GLOBAL VTOL_M_CMD_PREV       IS 0.0.
GLOBAL VTOL_COS_ATT_FILT     IS 1.0.
// diag
GLOBAL VTOL_DIAG_ROLL_D_ACCEL  IS 0.0.
GLOBAL VTOL_DIAG_PITCH_D_ACCEL IS 0.0.
GLOBAL VTOL_DIAG_COS_ATT_FILT  IS 1.0.
GLOBAL VTOL_DIAG_COLL_BEFORE_CORR IS 0.0.
GLOBAL VTOL_DIAG_COLL_AFTER_CORR  IS 0.0.
```

Reset in `IFC_INIT_STATE()`:
```kerboscript
SET VTOL_RATE_P_FILT      TO 0.
SET VTOL_RATE_Q_FILT      TO 0.
SET VTOL_RATE_P_FILT_PREV TO 0.
SET VTOL_RATE_Q_FILT_PREV TO 0.
SET VTOL_RATE_P_DOT_FILT  TO 0.
SET VTOL_RATE_Q_DOT_FILT  TO 0.
SET VTOL_L_CMD_PREV       TO 0.
SET VTOL_M_CMD_PREV       TO 0.
SET VTOL_COS_ATT_FILT     TO 1.
// diag
SET VTOL_DIAG_ROLL_D_ACCEL  TO 0.
SET VTOL_DIAG_PITCH_D_ACCEL TO 0.
SET VTOL_DIAG_COS_ATT_FILT  TO 1.
SET VTOL_DIAG_COLL_BEFORE_CORR TO 0.
SET VTOL_DIAG_COLL_AFTER_CORR  TO 0.
```

**Step 1.2 — Add a `_VTOL_UPDATE_RATE_FILTERS` function in `ifc_amo_vtol.ks`**

```kerboscript
FUNCTION _VTOL_UPDATE_RATE_FILTERS {
  PARAMETER ang_vel, starvec, forevec.

  LOCAL deg_per_rad IS 180 / CONSTANT:PI.
  LOCAL p_raw IS -VDOT(ang_vel, forevec) * deg_per_rad.
  LOCAL q_raw IS  VDOT(ang_vel, starvec) * deg_per_rad.

  LOCAL alpha_p IS _AMO_CFG_NUM("vtol_rate_p_alpha", VTOL_RATE_P_ALPHA, 0.01).
  LOCAL alpha_q IS _AMO_CFG_NUM("vtol_rate_q_alpha", VTOL_RATE_Q_ALPHA, 0.01).
  LOCAL alpha_dot IS _AMO_CFG_NUM("vtol_rate_pdot_alpha", VTOL_RATE_PDOT_ALPHA, 0.01).
  LOCAL accel_clamp IS _AMO_CFG_NUM("vtol_rate_accel_clamp_degs2", VTOL_RATE_ACCEL_CLAMP_DEGS2, 0.1).

  // Save previous before updating
  SET VTOL_RATE_P_FILT_PREV TO VTOL_RATE_P_FILT.
  SET VTOL_RATE_Q_FILT_PREV TO VTOL_RATE_Q_FILT.

  // EMA filter on raw rates
  SET VTOL_RATE_P_FILT TO VTOL_RATE_P_FILT + (p_raw - VTOL_RATE_P_FILT) * alpha_p.
  SET VTOL_RATE_Q_FILT TO VTOL_RATE_Q_FILT + (q_raw - VTOL_RATE_Q_FILT) * alpha_q.

  // Angular acceleration (rate derivative)
  LOCAL p_dot_raw IS 0.
  LOCAL q_dot_raw IS 0.
  IF IFC_ACTUAL_DT > 0.001 {
    SET p_dot_raw TO (VTOL_RATE_P_FILT - VTOL_RATE_P_FILT_PREV) / IFC_ACTUAL_DT.
    SET q_dot_raw TO (VTOL_RATE_Q_FILT - VTOL_RATE_Q_FILT_PREV) / IFC_ACTUAL_DT.
  }
  SET p_dot_raw TO CLAMP(p_dot_raw, -accel_clamp, accel_clamp).
  SET q_dot_raw TO CLAMP(q_dot_raw, -accel_clamp, accel_clamp).

  // EMA filter on acceleration
  SET VTOL_RATE_P_DOT_FILT TO VTOL_RATE_P_DOT_FILT + (p_dot_raw - VTOL_RATE_P_DOT_FILT) * alpha_dot.
  SET VTOL_RATE_Q_DOT_FILT TO VTOL_RATE_Q_DOT_FILT + (q_dot_raw - VTOL_RATE_Q_DOT_FILT) * alpha_dot.
}
```

**Step 1.3 — Call `_VTOL_UPDATE_RATE_FILTERS` at the top of `VTOL_TICK_PREARM`**

Add immediately after caching `_angvel`, `_starvec`, `_forevec` (around line 1092):
```kerboscript
_VTOL_UPDATE_RATE_FILTERS(_angvel, _starvec, _forevec).
```

**Step 1.4 — Add D term to the roll and pitch rate loops**

In the roll auto-feedback block (around line 1287), after the existing `roll_unsat` computation:
```kerboscript
LOCAL kd_roll_accel IS _AMO_CFG_NUM("vtol_rate_kd_roll_accel", VTOL_RATE_KD_ROLL_ACCEL, 0).
SET kd_roll_accel TO kd_roll_accel * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
LOCAL roll_d_accel IS -VTOL_RATE_P_DOT_FILT * kd_roll_accel.
SET roll_unsat TO roll_unsat + roll_d_accel.
SET roll_cmd TO CLAMP(roll_unsat, -roll_cap, roll_cap).
SET VTOL_DIAG_ROLL_D_ACCEL TO roll_d_accel.
```

Apply the same pattern to the pitch block:
```kerboscript
LOCAL kd_pitch_accel IS _AMO_CFG_NUM("vtol_rate_kd_pitch_accel", VTOL_RATE_KD_PITCH_ACCEL, 0).
SET kd_pitch_accel TO kd_pitch_accel * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
LOCAL pitch_d_accel IS VTOL_RATE_Q_DOT_FILT * kd_pitch_accel.
SET pitch_unsat TO pitch_unsat + pitch_d_accel.
SET pitch_cmd TO CLAMP(pitch_unsat, -pitch_cap, pitch_cap).
SET VTOL_DIAG_PITCH_D_ACCEL TO pitch_d_accel.
```

Sign convention check for pitch:
- If aircraft is pitching nose-up at increasing rate: q > 0 and q_dot > 0
- We want the D term to resist this: the correction should push nose DOWN
- In the current code, positive `pitch_unsat` pushes nose UP (via pitch_mix)
- So we want: `pitch_d_accel = -(-q_dot_filt × kd_pitch)` ... let's trace more carefully.

The current `pitch_unsat` is:
```kerboscript
LOCAL pitch_unsat IS (pitch_rate_degs - pitch_rate_cmd) * pitch_rate_kp.
```
Where `pitch_rate_degs = VDOT(ang_vel, starvec) × deg_per_rad` = `q` (positive = nose up).
And `pitch_rate_cmd` is the rate command from the attitude loop.
And the final `pitch_cmd = CLAMP(pitch_unsat, -pitch_cap, pitch_cap)` is the differential command.

When `pitch_rate_degs > pitch_rate_cmd` (pitching nose-up too fast), `pitch_unsat > 0`.
Positive `pitch_cmd` → increases pitch → **wrong, this would accelerate the error**!

**Wait, there is a sign error in the existing pitch rate formula.** Let me re-read:

From line ~1345:
```kerboscript
LOCAL pitch_rate_cmd_p IS -pitch_err * pitch_att2rate_kp.
```
`pitch_err = -pitch_ang` (line ~1344), and `pitch_att2rate_kp > 0`.
So for nose-up attitude (pitch_ang > 0): `pitch_err < 0` → `pitch_rate_cmd_p > 0`.
This means positive rate command when nose is up... which means "increase nose-up rate"?

That seems wrong, but looking further at line ~1355:
```kerboscript
LOCAL pitch_unsat IS (pitch_rate_degs - pitch_rate_cmd) * pitch_rate_kp.
```
If we're pitching nose-up (q > 0) and rate_cmd is 0:
- `pitch_unsat = q × Kp > 0` → positive pitch command.
- Positive pitch command via `pitch_mix_sign=+1` increases front engines → more nose-up → WRONG direction!

This is a bug in the existing code's pitch axis convention. The sign of `pitch_d_accel` to add must
account for this existing convention. Until the pitch sign convention is audited and fixed,
the D term should be:
```kerboscript
LOCAL pitch_d_accel IS VTOL_RATE_Q_DOT_FILT × kd_pitch_accel.
```
(Same sign as the existing `pitch_d_term` diagnostic which is `+pitch_rate_degs × pitch_rate_kp`).

**NOTE: The pitch axis sign convention in the existing code must be audited and documented before
implementing the D term.** This is a separate task (Step 1.5 below).

**Step 1.5 — Audit and document pitch axis sign convention (prerequisite to D term)**

Before adding the D term, verify in flight (or in the Jupyter notebook diagnostic) that
the existing pitch correction correctly arrests a nose-up attitude excursion. Log:
- `phi_actual`, `theta_actual`
- `pitch_err`, `pitch_rate_cmd`, `pitch_rate_degs`
- `pitch_unsat`, `pitch_cmd`

If nose-up attitude (theta > 0) results in `pitch_cmd > 0` when `pitch_mix_sign = +1`,
then increasing front engine thrust = more nose-up = positive feedback. That's wrong.

Expected correct behavior: nose-up → negative pitch_cmd (reduce front engines, nose down).

Update vtol_test.ks to log these values and compare against the Jupyter notebook plots.

**Step 1.6 — Add geometry correction to `_VTOL_VS_HOLD`**

At the top of `_VTOL_VS_HOLD`, after reading `coll_max`:
```kerboscript
// Update attitude geometry correction filter
LOCAL phi_rad IS ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1)) * CONSTANT:PI / 180.
LOCAL theta_rad IS (90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR)) * CONSTANT:PI / 180.
LOCAL cos_att_raw IS COS(phi_rad * 180/CONSTANT:PI) * COS(theta_rad * 180/CONSTANT:PI).
// kOS trig takes degrees
LOCAL cos_phi IS CLAMP(VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR), -1, 1).  // faster: dot with UP
LOCAL cos_att_meas IS MAX(VTOL_COS_ATT_FLOOR, ABS(cos_phi)).
LOCAL cos_att_alpha IS _AMO_CFG_NUM("vtol_cos_att_alpha", VTOL_COS_ATT_ALPHA, 0.001).
SET VTOL_COS_ATT_FILT TO VTOL_COS_ATT_FILT + (cos_att_meas - VTOL_COS_ATT_FILT) * cos_att_alpha.
```

Note: `VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR)` = cos of the angle between the ship's
"up" axis and the world "up" direction = cos(bank) × cos(pitch) for typical attitudes.
This is geometrically correct and avoids the two-ARCSIN path.

Then, after computing `target_collective` (before the slew limiter):
```kerboscript
SET VTOL_DIAG_COLL_BEFORE_CORR TO target_collective.
LOCAL cos_use IS CLAMP(VTOL_COS_ATT_FILT, VTOL_COS_ATT_FLOOR, 1.0).
SET target_collective TO CLAMP(target_collective / cos_use, 0, coll_max).
SET VTOL_DIAG_COLL_AFTER_CORR TO target_collective.
SET VTOL_DIAG_COS_ATT_FILT TO VTOL_COS_ATT_FILT.
```

**Step 1.7 — Add new constants to `ifc_constants.ks`**

After the existing VTOL section:
```kerboscript
// --- VTOL Phase 1 additions (D term, geometry correction) ---
GLOBAL VTOL_RATE_KD_ROLL_ACCEL      IS 0.0.    // start at 0; tune upward in flight
GLOBAL VTOL_RATE_KD_PITCH_ACCEL     IS 0.0.    // start at 0; tune upward in flight
GLOBAL VTOL_RATE_P_ALPHA            IS 0.70.
GLOBAL VTOL_RATE_Q_ALPHA            IS 0.70.
GLOBAL VTOL_RATE_PDOT_ALPHA         IS 0.35.
GLOBAL VTOL_RATE_QDOT_ALPHA         IS 0.35.
GLOBAL VTOL_RATE_ACCEL_CLAMP_DEGS2  IS 300.0.
GLOBAL VTOL_COS_ATT_ALPHA           IS 0.12.
GLOBAL VTOL_COS_ATT_FLOOR           IS 0.25.
```

**Step 1.8 — Add to `vtol_test.ks` config**

```kerboscript
"vtol_rate_kd_roll_accel",    0.0,   // tune: start here, increase toward 0.04
"vtol_rate_kd_pitch_accel",   0.0,   // tune: start here, increase toward 0.04
"vtol_rate_p_alpha",          0.65,
"vtol_rate_q_alpha",          0.65,
"vtol_rate_pdot_alpha",       0.30,
"vtol_rate_qdot_alpha",       0.30,
"vtol_rate_accel_clamp_degs2", 250.0,
"vtol_cos_att_alpha",         0.10,
"vtol_cos_att_floor",         0.25,
```

---

### Phase 2: Horizontal Velocity Hold and KHV (Next after Phase 1 stable)

**Goal:** Enable the aircraft to hold position and automatically stop horizontal drift.

**Files changed:** `ifc_amo_vtol.ks`, `ifc_state.ks`, `ifc_constants.ks`

**Step 2.1 — Implement `_VTOL_VEL_HOLD` function**

This function replaces the direct pilot-stick-to-attitude path when VTOL_VEL_HOLD_ACTIVE.

Inputs: `p_roll` (pilot roll), `p_pitch` (pilot pitch)
Outputs: `phi_cmd`, `theta_cmd` (attitude commands to L3)

When `VTOL_KHV_ACTIVE = TRUE`: `vN_cmd = vE_cmd = 0`
When `VTOL_VEL_HOLD_ACTIVE = TRUE`: `vN_cmd, vE_cmd` from pilot (rate-scaled)
When both FALSE: pass pilot stick through as direct attitude commands

**Step 2.2 — Compute world-frame velocity components**

Add helper `_VTOL_GET_SURFACE_VEL`:
```kerboscript
FUNCTION _VTOL_GET_SURFACE_VEL {
  // Returns LEXICON("vN", vN, "vE", vE) in m/s
  LOCAL surf IS SHIP:VELOCITY:SURFACE.
  LOCAL n_vec IS IFC_NORTH_VEC.     // must be cached in main loop
  LOCAL e_vec IS VCRS(SHIP:UP:VECTOR, n_vec):NORMALIZED.
  RETURN LEXICON("vN", VDOT(surf, n_vec), "vE", VDOT(surf, e_vec)).
}
```

**Step 2.3 — Integrate velocity PI into Layer 2**

The velocity error drives `ax_cmd`, `ay_cmd`, which then drive `theta_cmd`, `phi_cmd`
(Section 9.3–9.4).

The output `theta_cmd` and `phi_cmd` from the velocity loop are sent to Layer 3 in place
of the pilot pitch and roll inputs when `VTOL_VEL_HOLD_ACTIVE`.

**Step 2.4 — Add KHV trigger**

When the pilot presses a designated button (AG or menu), set:
```kerboscript
SET VTOL_KHV_ACTIVE TO TRUE.
SET VTOL_VEL_INT_N TO 0.
SET VTOL_VEL_INT_E TO 0.
```

KHV completes when `|VTOL_VN_ACTUAL| < VTOL_KHV_CAPTURE_MPS` and
`|VTOL_VE_ACTUAL| < VTOL_KHV_CAPTURE_MPS`.

---

### Phase 3: Hover-at-Point (After Phase 2 stable)

**Goal:** Lock the aircraft to a GPS coordinate while holding altitude.

Depends on Phase 2 (velocity hold as the inner loop).

**Step 3.1 — Implement `_VTOL_POS_HOLD` function**

Takes `VTOL_TARGET_LAT`, `VTOL_TARGET_LNG`, computes position error in meters (via
`GEO_DISTANCE`, `GEO_BEARING`), runs PI controller, outputs `vN_cmd`, `vE_cmd` to the
velocity loop.

**Step 3.2 — Arm hover-at-point**

On activation, capture current position:
```kerboscript
SET VTOL_TARGET_LAT TO SHIP:GEOPOSITION:LAT.
SET VTOL_TARGET_LNG TO SHIP:GEOPOSITION:LNG.
SET VTOL_POS_HOLD_ACTIVE TO TRUE.
SET VTOL_KHV_ACTIVE TO FALSE.
```

**Step 3.3 — Add to display**

Show `VTOL_TARGET_LAT/LNG`, current position error (m), and velocity commands in the
VTOL display section.

---

### Phase 4: Transition to Forward Flight (After Phase 3 stable)

**Goal:** Smooth, stable transition from hover to forward flight.

This is the most complex phase and requires:
1. Nacelle collective angle command (`VTOL_NACELLE_ALPHA_CMD`)
2. Altitude hold via geometry-corrected collective (already in Phase 1)
3. Blended attitude command law (Section 12.4)
4. Handoff from thrust-limiter collective to THROTTLE_CMD control (Section 12.3)
5. Minimum airspeed constraint on nacelle tilt (Section 12.2)

**Step 4.1 — Separate per-nacelle angle command into two components:**
```
α_cmd_i = α_collective + α_yaw_diff[i] + α_pitch_diff[i]
```
The `α_collective` is the new scheduler-driven command.
The `α_yaw_diff[i]` is the existing yaw differential.
The `α_pitch_diff[i]` allows differential tilt between forward/aft pairs for pitch correction
(only effective when α is away from 90°).

**Step 4.2 — Transition scheduler state machine:**

Triggered by: pilot mode command, or auto-transition above `VTOL_TRANS_START_IAS`.

States: `VTOL_TRANS_NONE`, `VTOL_TRANS_ACCEL`, `VTOL_TRANS_TIPPING`,
`VTOL_TRANS_CRUISE`, `VTOL_TRANS_RETURN`.

During `VTOL_TRANS_TIPPING`, nacelle angle is commanded from 90° toward 0° with
the minimum-airspeed constraint enforced.

**Step 4.3 — Collective-to-throttle handoff:**

As `VTOL_HOVER_BLEND` decreases from 1.0 to 0.0:
- `THROTTLE_CMD` transitions from the locked 1.0 to control by AT system
- Thrust limiter collective reduces from `VTOL_COLLECTIVE` toward 1.0 (uniform)

---

## 19. Test Plan

### 19.1 Pre-Flight Verification (vtol_test.ks and Jupyter Notebook)

Before any flight test of Phase 1 changes, verify in the diagnostic notebook:

**T1: Rate filter response test**
- While stationary, manually log 10 seconds of raw `ANGULARVEL` and `VTOL_RATE_P_FILT`
- Verify EMA is smoothing noise but not adding excessive lag (< 0.5s delay visible)
- Verify `VTOL_RATE_P_DOT_FILT` is reasonable (< 50 deg/s² at rest)

**T2: Geometry correction sanity check**
- With aircraft static at 0° bank: `VTOL_COS_ATT_FILT` should converge to ~1.0
- Command 15° bank manually: `VTOL_COS_ATT_FILT` should converge to cos(15°) ≈ 0.966
- With correction: collective should increase by ~4% to maintain altitude ✓

**T3: D term sign verification (prerequisite — must pass before enabling Kd > 0)**
- Hover at stable altitude
- Introduce a roll perturbation (gently bump pitch stick to create a roll)
- Log `phi_actual`, `VTOL_RATE_P_DOT_FILT`, `VTOL_DIAG_ROLL_D_ACCEL`, `roll_cmd`
- Verify: when p_dot > 0 (rolling right, accelerating), `roll_d_accel < 0`
  (opposing the acceleration) ✓

**T4: Pitch sign audit (must pass before enabling any pitch D term)**
- Create a nose-up attitude excursion (brief pitch stick pulse)
- Log `theta_actual`, `pitch_err`, `pitch_rate_cmd`, `pitch_unsat`, `pitch_cmd`
- Verify: nose-up → pitch_cmd → correction brings nose down (not further up)
- If sign is wrong: identify where the sign convention flips and document the fix

### 19.2 Flight Test Sequence

**Phase 1 flight tests:**

**FT1-A: Geometry correction isolated test (Kd = 0)**
- Enable geometry correction (`vtol_cos_att_alpha = 0.10`)
- Hover at 20 m AGL, stable
- Command 10° bank for 5 seconds, release
- Expected: no noticeable altitude drop during bank; altitude recovers quickly
- Without fix: aircraft descends 2–5 m during 10° bank excursion

**FT1-B: D term introduction (incremental)**
- Set `vtol_rate_kd_roll_accel = 0.01` and `vtol_rate_kd_pitch_accel = 0.01`
- Hover at 20 m AGL
- Introduce roll perturbation, observe damping
- Increase gain by 0.005 until either: (a) oscillation appears, or (b) noticeable
  improvement in damping with no oscillation
- Maximum usable gain: below oscillation threshold by ~30% margin
- Typical expected range: 0.02–0.08 (depends on spool time constant)

**FT1-C: Combined test**
- Hover at 20 m AGL with both corrections active
- Test: gentle 5-second forward flight (10° pitch), return to hover
- Expected: altitude maintained within ±3 m throughout (vs. current ±8–15 m)

**Phase 2 flight tests:**

**FT2-A: KHV test**
- Fly forward at 5 m/s at 20 m AGL
- Activate KHV
- Expected: aircraft decelerates and stabilizes within 30 seconds, within ±2 m/s
  of zero ground speed

**FT2-B: Hover-at-point test**
- Activate hover-at-point at current location
- Walk the aircraft 20 m away manually (pilot input override), release
- Expected: aircraft returns to original position within 30–60 seconds

**Phase 4 flight tests:**

**FT4-A: Short transition test**
- At 30 m AGL, fly forward to 35 m/s IAS
- Activate transition
- Monitor altitude: should not deviate more than ±5 m during transition
- Monitor nacelle angle: should smoothly tilt from 90° to ~40° by the time IAS = 60 m/s

**FT4-B: Full transition**
- Transition to full forward flight
- Verify: AT system takes over throttle control
- Verify: IFC phases (cruise, approach) are available after transition

### 19.3 Regression Tests (after any change)

After any change to `ifc_amo_vtol.ks`:
1. Run `vtol_test.ks` and confirm VTOL_DISCOVER succeeds (correct engine count and arm lengths)
2. Verify zero-altitude hover is stable (no drift, no oscillation) for 60 seconds
3. Verify gentle pilot inputs produce expected responses (no reversed controls)
4. Verify ABORT handler releases VTOL correctly (no throttle lock after abort)

---

## 20. Risk Register

| ID | Risk | Probability | Impact | Mitigation |
|----|------|------------|--------|-----------|
| R1 | Pitch D term sign is wrong, causing positive feedback | Medium | High | Must complete Step 1.5 (sign audit) before enabling D term in pitch |
| R2 | `cos_att` filter too slow — oscillation in altitude during roll stabilization | Medium | Medium | Increase `vtol_cos_att_alpha` if altitude oscillates |
| R3 | `cos_att` filter too fast — altitude chases attitude oscillation | Low | Medium | Reduce `vtol_cos_att_alpha` to decouple channels |
| R4 | D term amplifies rate noise — high-frequency oscillation | Medium | High | Start with `Kd = 0`, increase slowly; verify `VTOL_RATE_P_DOT_FILT` is smooth before enabling |
| R5 | Rate filter adds lag — slower attitude response | Low | Low | If response is too slow, increase `VTOL_RATE_P_ALPHA` toward 1.0 |
| R6 | Velocity loop interferes with trim — constant attitude offset | Medium | Medium | Reset velocity integrators when engaging vel-hold; seed `vel_int` from current vel error |
| R7 | Transition collective feedforward overestimates — hard pitch-up at transition | Medium | High | Start nacelle tilt very slowly; monitor pitch deviation during FT4-A |
| R8 | Pitch sign convention mismatch found during audit — requires sign flip that breaks existing behavior | Medium | High | Add sign-flip flag to config (`vtol_pitch_sign_corrected`) to allow staged rollout |
| R9 | `VTOL_RATE_P_FILT_PREV` retains stale value across mode changes — spike on re-engagement | Low | Medium | Reset `PREV` values equal to `FILT` values on mode engagement |
| R10 | kOS `SHIP:ANGULARVEL` returns world-frame while code treats as body-frame | Low | High | Double-check the dot product decomposition in T3 sign audit test — if body-frame rates match intuitive behavior, convention is confirmed correct |

---

## Appendix A: Gain Tuning Guide

### A.1 Tuning Order

Always tune from inner to outer:
1. Rate loop D term gains (`Kd_roll_accel`, `Kd_pitch_accel`)
2. Rate loop P gain (`roll_rate_kp`, `pitch_rate_kp`)
3. Attitude loop P gain (`roll_att2rate_kp`, `pitch_att2rate_kp`)
4. Attitude loop I gain (`roll_att2rate_ki`, `pitch_att2rate_ki`)
5. VS hold PI (`vtol_vs_kp`, `vtol_vs_ki`)
6. Velocity loop PI (Phase 2)
7. Position loop P (Phase 3)

### A.2 Rate Loop D Term Starting Point

Estimated D term gain from physics:
```
Kd_estimate ≈ τ_spool × Kp_rate / 2
```

For `Kp_rate = 0.030` and `τ_spool ≈ 3.5s`:
```
Kd_estimate ≈ 3.5 × 0.030 / 2 ≈ 0.05
```

But the D term here is on angular acceleration (deg/s²), not angular velocity:
```
D contribution = -Kd × p_dot
```
For typical roll disturances (p_dot ~ 5 deg/s²), this contributes ≈ -0.25 to the command.
Compare with P contribution at 5 deg/s rate error: `P = 0.030 × 5 = 0.15`.

The D term should be roughly 2× the P term at the expected disturbance frequency.
Start at `Kd_roll_accel = 0.02` and increase until oscillation onset, then back off 30%.

### A.3 Geometry Correction Scale Factor

With `cos_att_floor = 0.25` (75° max tilt before floor kicks in), the maximum collective
boost from geometry correction is `1 / 0.25 = 4×`. This is extreme but only reached at
75° bank, which is already an UPSET condition. In normal hover operations (< 20° tilt):
```
1 / cos(20°) ≈ 1.064   // 6.4% collective increase at 20° bank
```
This is well within the engine's headroom and should cause no issues.

### A.4 Bandwidth Separation Verification

After tuning, verify bandwidth separation by doing a step input test:

1. Command a step in attitude (via pilot stick): 10° roll, then release
2. Measure time for roll rate to peak → this is approximately the rise time of the rate loop
3. Measure time for roll attitude to return to zero → this is the rise time of the attitude loop
4. The ratio should be > 3 (attitude loop > 3× slower than rate loop)

If the ratio is < 2, the attitude P gain is too high relative to the rate gains — reduce
`roll_att2rate_kp`.

---

## Appendix B: kOS Implementation Notes

### B.1 SHIP:ANGULARVEL Convention

`SHIP:ANGULARVEL` is in the **world (inertial) frame**. To extract body-frame components:

```kerboscript
LOCAL p IS -VDOT(SHIP:ANGULARVEL, SHIP:FACING:FOREVECTOR).  // roll rate, deg/s after ×(180/π)
LOCAL q IS  VDOT(SHIP:ANGULARVEL, SHIP:FACING:STARVECTOR).  // pitch rate
LOCAL r IS -VDOT(SHIP:ANGULARVEL, SHIP:FACING:TOPVECTOR).   // yaw rate
```

These must be read only once per tick and stored in locals. `SHIP:ANGULARVEL` and
`SHIP:FACING:*` each traverse the vessel part tree — calling them multiple times per
tick is expensive.

### B.2 COS in kOS

kOS `COS()` and `SIN()` take degrees, not radians. Verify this in all geometry
calculations. The `CONSTANT:PI` is available for converting.

### B.3 VDOT with Normalized Vectors

`SHIP:UP:VECTOR` is already normalized. `SHIP:FACING:TOPVECTOR` is already normalized.
`VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR)` is the cosine of the angle between them
without any further normalization needed. This is the fastest way to compute `cos(φ)×cos(θ)`.

### B.4 LIST vs. LEXICON for Per-Engine Data

The current code uses kOS `LIST()` for per-engine arrays (VTOL_ROLL_MIX, etc.). New per-
engine state (VTOL_RATE_P_FILT for each engine is not needed — only two scalars). The
rate filter state is aircraft-level, not per-engine. Use `GLOBAL` scalars, not lists.

### B.5 Config Key Naming for Phase 1 Additions

All new config keys follow the existing pattern `"vtol_<subsystem>_<param>"`:
- `"vtol_rate_kd_roll_accel"` — rate loop D term roll
- `"vtol_rate_kd_pitch_accel"` — rate loop D term pitch
- `"vtol_cos_att_alpha"` — geometry correction EMA rate
- `"vtol_cos_att_floor"` — minimum cos(att) for geometry correction

These must be added to `vtol_test.ks` aircraft config with initial values of 0 for
all gains, to ensure the aircraft behaves identically to pre-Phase-1 until each gain
is explicitly set non-zero.

---

## Appendix C: Development Phases with Embedded Verification Steps

This appendix provides a step-by-step development sequence where every code change is
immediately followed by a concrete verification procedure. **Each verification step must
pass before the next code change is made.** This prevents the common failure mode of
making multiple changes at once and then being unable to isolate which one broke things.

**Conventions:**
- Steps are `P<phase>.<step>` — a discrete code change
- Verification steps are `P<phase>.<step>.V<n>` — a testable check of that exact change
- **GATE** — a blocking prerequisite; work stops until the gate passes
- Expected values are given as ranges, not exact numbers — real hardware has variation
- "Compare to baseline" means compare the named CSV channels against the pre-phase
  reference log using the Jupyter diagnostic notebook

**Before starting Phase 1:** Record a 60-second stable hover CSV log labeled
`baseline_pre_phase1.csv`. This is the regression reference for all subsequent steps.

---

### Phase 1: D Term + Geometry Correction

**Goal:** Increase hover damping and eliminate altitude loss during attitude excursions.  
**Failure mode if skipped:** Underdamped oscillation from spool lag; aircraft sinks
during any roll or pitch correction.  
**Dependencies:** None — builds on the current codebase as-is.

---

#### P1.1 — Add state variables and constants (no behavior change)

**Code changes:**
1. Add Phase 1 globals to `ifc_state.ks` (see Section 16, Phase 1 list)
2. Reset all new globals in `IFC_INIT_STATE()` to defaults
3. Add Phase 1 constants to `ifc_constants.ks` (Section 15 rate loop and geometry sections)
4. **Set all new gain constants to 0.0** — this is critical; a non-zero default would
   immediately change behavior before verification
5. Add new config keys to `vtol_test.ks` with 0.0 values for all new gains

**P1.1.V1 — Load without error (on pad, engines off)**

Procedure: Run `vtol_test.ks`.

Pass: Script loads. `VTOL_DISCOVER()` completes with correct engine count. No kOS
runtime error of any kind.  
Fail: Any error → check new variable names against CLAUDE.md reserved names list
(common culprits: `LOG`, `LIST`, `STATUS`, single-letter names).

**P1.1.V2 — No behavioral change from baseline**

Procedure: Hover at 20 m AGL for 60 seconds. Record CSV as `p1.1_check.csv`.

Pass: `VTOL_COLLECTIVE`, `VTOL_DIAG_ROLL_ERR`, `VTOL_DIAG_PITCH_ERR` match baseline
within ±5% RMS. New globals exist but are 0.0 / 1.0 since no code uses them yet.  
Fail: Any behavioral change → a newly added global name shadows an existing one in the
VTOL_TICK_PREARM scope; run a diff of state globals and audit for accidental collision.

---

#### P1.2 — Implement `_VTOL_UPDATE_RATE_FILTERS` and wire it in

**Code changes:**
1. Write `_VTOL_UPDATE_RATE_FILTERS(ang_vel, starvec, forevec)` (Section 18, Step 1.2)
2. Call it at the top of `VTOL_TICK_PREARM`, immediately after the facing-vector cache block
3. The function writes only to filter state globals — no control outputs change yet

**P1.2.V1 — Filter sanity on the pad (engines on, throttle at idle)**

Procedure: Run the armed VTOL loop for 30 seconds while stationary. Check
`VTOL_RATE_P_FILT`, `VTOL_RATE_Q_FILT` from the display or CSV.

Pass: Both values within ±3 deg/s (idle engine vibration). No linear drift. Values
are stable.  
Fail: Values > ±10 deg/s at idle → alpha is too high; reduce `vtol_rate_p_alpha` by 0.1.  
Fail: Diverging values → the projection `VDOT(SHIP:ANGULARVEL, SHIP:FACING:FOREVECTOR)`
is being called on each tick with a stale facing vector; confirm the function uses the
cached `_forevec` and `_starvec` passed as parameters, not fresh suffix reads.

**P1.2.V2 — Angular acceleration is bounded**

Procedure: Continue the same 30-second stationary run. Read `VTOL_RATE_P_DOT_FILT`.

Pass: `|VTOL_RATE_P_DOT_FILT| < 20 deg/s²` while stationary at idle.  
Fail: Spikes > 100 deg/s² → confirm `IFC_ACTUAL_DT > 0.001` guard is inside
`_VTOL_UPDATE_RATE_FILTERS` before the division; also confirm `VTOL_RATE_ACCEL_CLAMP_DEGS2`
is applied to the raw derivative before the EMA.

**P1.2.V3 — Filters respond to a known input**

Procedure: Hover at 10 m AGL. Hold pitch stick forward for 2 seconds, release.

Pass: `VTOL_RATE_Q_FILT` rises (negative or positive depending on convention) during
forward pitch and returns near 0 when stick is released and aircraft levels. The filter
tracks the general trend without oscillation.  
Fail: Filter does not respond → the starvector projection is incorrect; verify against
Section 3.2 sign convention table and confirm with a known direction.

**P1.2.V4 — No behavioral change from P1.1 baseline**

Procedure: Record 60-second hover as `p1.2_check.csv`. Compare to `p1.1_check.csv`.

Pass: Matches within ±5% RMS (filter writes state but nothing reads it yet).  
Fail: Any change → the new function accidentally writes to a variable that the existing
control loop reads; search for the collision.

---

#### P1.3 — GATE: Pitch Axis Sign Audit

**This is a hard gate. P1.4 (D term) cannot be implemented until this gate passes.**

Purpose: Confirm the exact sign of the corrective action when the aircraft is in a nose-up
or nose-down excursion. The D term must have the same sign as the corrective (stabilizing)
action, not the error-growing action.

**P1.3.V1 — Code trace for a nose-up excursion**

Procedure (no flight required — trace the code manually):

1. Assume: `theta_actual = +5°` (nose up), no pilot input, `pitch_rate_cmd = 0`
2. `pitch_ang = +5`
3. `pitch_err = -pitch_ang = -5`
4. `pitch_rate_cmd_p = -pitch_err × kp = +5 × kp` (positive)
5. `pitch_rate_cmd = +positive` (let's say +4.5 deg/s commanded)
6. If aircraft has no current rate: `pitch_rate_degs = 0`
7. `pitch_unsat = (0 - 4.5) × rate_kp = -4.5 × 0.032 = -0.144`
8. `pitch_cmd = -0.144` (negative)
9. With `pitch_mix_sign = +1`: `pitch_mix_FL > 0`, `pitch_cmd < 0`
   → `diff_FL = pitch_cmd × pitch_mix_FL < 0` → forward engine DECREASES
10. Forward engine decreasing → front of aircraft thrust reduces → nose pitches DOWN ✓

Document: The sign is correct. The double-negative through `pitch_err = -pitch_ang` and
`pitch_unsat = (rate_actual - rate_cmd)` (rate_actual minus rate_cmd, not vice versa)
produces the correct stabilizing direction.

Pass: The trace in steps 1–10 is consistent and ends with "nose pitches DOWN" ✓.  
Fail: Any step in the trace contradicts the expected direction → there is an existing
sign error; document which step and in which direction before proceeding to V2.

**P1.3.V2 — Flight confirmation**

Procedure: Hover at 15 m AGL. Apply a brief nose-up impulse (half-second stick pull), release.

Pass: Aircraft returns to level within 5 seconds without pilot input. The trace from V1
is confirmed as stabilizing.  
Fail: Aircraft continues pitching up (diverges) after stick release → the sign error
found in V1 is real; the fix must be identified and applied before continuing. **Log
the exact gain values and conditions at time of failure for analysis.**

**P1.3.V3 — D term sign derivation (required output of this gate)**

Based on the confirmed sign convention from V1/V2, derive and document the correct sign
for the D term addition:

Given: `pitch_unsat = (rate_actual - rate_cmd) × Kp`

When aircraft is pitching nose-up and accelerating: `q > 0`, `q_dot > 0`
- The D term should RESIST the acceleration — add a correction that tends to REDUCE `pitch_cmd`
- Since more negative `pitch_cmd` → lower front engines → nose down (the corrective direction):
  - We want `pitch_d_accel < 0` when `q_dot > 0`
  - Therefore: `pitch_d_accel = -q_dot_filt × Kd_pitch_accel` (negative sign)
- Final: `pitch_unsat += pitch_d_accel = -q_dot_filt × Kd`

For roll: same logic with `p` and `p_dot`:
- `roll_d_accel = -p_dot_filt × Kd_roll_accel`

Record these derivations in the code as a comment block immediately above the D term
addition. This is the permanent documentation of the sign convention.

Gate passes when V1 and V2 both confirm correct direction AND the sign derivation in
V3 is documented in the code comment.

---

#### P1.4 — Add the D term to the rate loops

**Code changes (only after P1.3 gate passes):**
1. In the roll auto-feedback block: add `roll_d_accel = -VTOL_RATE_P_DOT_FILT × kd_roll`
   and add it to `roll_unsat` before the cap (Section 18, Step 1.4)
2. In the pitch auto-feedback block: add `pitch_d_accel = -VTOL_RATE_Q_DOT_FILT × kd_pitch`
   (the sign comes from the P1.3.V3 derivation)
3. Write `VTOL_DIAG_ROLL_D_ACCEL` and `VTOL_DIAG_PITCH_D_ACCEL`
4. All new gains start at 0.0 in `vtol_test.ks`

**P1.4.V1 — Zero-gain deploy: no behavioral change**

Procedure: Hover 60 seconds with Kd gains at 0.0. Compare to P1.2 baseline.

Pass: No behavioral change. `VTOL_DIAG_ROLL_D_ACCEL` and `VTOL_DIAG_PITCH_D_ACCEL`
both read 0.0 as expected.  
Fail: Any behavioral change with Kd=0 → the code has a logic error outside the gain
multiplication; the D term path must be purely additive with zero effect at zero gain.

**P1.4.V2 — D term sign is correct in flight**

Procedure: Hover stable. Introduce a brief right roll impulse (0.2-second stick pulse).
While aircraft is rolling right and rate is rising (`p > 0`, `p_dot > 0`):

Pass: `VTOL_DIAG_ROLL_D_ACCEL < 0` (opposing the roll acceleration).  
Fail: `VTOL_DIAG_ROLL_D_ACCEL > 0` → sign is inverted in code; negate the computation.

**P1.4.V3 — Incremental Kd tuning (roll axis)**

Procedure: Increase `vtol_rate_kd_roll_accel` from 0.0 in steps of 0.005.

After each step:
1. Hover for 30 seconds with the new gain
2. Introduce a 5-degree roll perturbation
3. Measure settling time (time from perturbation to |phi| < 1°)

Pass at each step: Settling time decreases OR remains equal AND no new oscillation is
visible. Record settling time vs. gain in a table.

Stop when: Oscillation appears at gain Kg. Set final gain to `0.7 × Kg`.

Expected outcome: Final gain in range 0.02–0.06. Settling time reduced from baseline
(typically > 8 seconds) to < 5 seconds with correct D gain.

Repeat identically for pitch axis using `vtol_rate_kd_pitch_accel`.

**P1.4.V4 — Both axes simultaneously**

Procedure: With both D gains set to tuned values from V3:
1. Hover 2 minutes — watch for any slow-growing oscillation (period > 10 seconds)
2. Command 10° forward pitch for 5 seconds, return to hover
3. Command 10° bank for 5 seconds, return to hover

Pass: No oscillation in any 2-minute hover. Attitude returns to level within 5 seconds
of releasing stick. Altitude deviation during attitude inputs < 5 m.  
Fail: Slow oscillation (period 10–30 seconds) → outer-loop (attitude PI) is interfering
with the D term; reduce `roll_att2rate_kp` by 20% and retry.

---

#### P1.5 — Add geometry correction to collective

**Code changes:**
1. Compute and EMA-filter `cos(phi) × cos(theta)` inside `_VTOL_VS_HOLD` using
   `VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR)` (fastest, most accurate single dot product)
2. Divide `target_collective` by the filtered cosine before the slew limiter
3. Write `VTOL_DIAG_COLL_BEFORE_CORR`, `VTOL_DIAG_COLL_AFTER_CORR`, `VTOL_DIAG_COS_ATT_FILT`
4. Start with `vtol_cos_att_alpha = 0.0` — filter stays at initial value 1.0 and the
   division by 1.0 changes nothing

**P1.5.V1 — Zero-alpha deploy: no behavioral change**

Procedure: Hover 60 seconds with `vtol_cos_att_alpha = 0.0`. Compare to P1.4 baseline.

Pass: No behavioral change. `VTOL_DIAG_COS_ATT_FILT` reads exactly 1.0.
`COLL_BEFORE_CORR` equals `COLL_AFTER_CORR`.  
Fail: Any behavioral change → the filter initialization is not 1.0; confirm
`VTOL_COS_ATT_FILT` is reset to 1.0 (not 0.0) in `IFC_INIT_STATE()`.

**P1.5.V2 — Filter responds to a known attitude**

Procedure:
1. Enable filter: `vtol_cos_att_alpha = 0.10`
2. Hover at 10 m AGL, stable level
3. Command a 15° bank and hold for 20 seconds while logging

Pass:
- `VTOL_DIAG_COS_ATT_FILT` decreases from 1.0 toward `cos(15°) ≈ 0.966`
- It approaches but does not overshoot 0.966
- `COLL_AFTER_CORR > COLL_BEFORE_CORR` during the bank (about 3.5% higher)

Fail: Filter does not decrease → `VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR)` is not
the right quantity; verify by checking that the value is 1.0 when level and ~0.96 at 15°.

**P1.5.V3 — Altitude improvement during a bank**

Procedure:
1. Geometry correction enabled at `alpha = 0.10`
2. Hover at 20 m AGL with alt-hold active
3. Command a 15° bank, hold 10 seconds, return to level
4. Record altitude deviation from the log

Pass: Altitude deviation during bank < 3 m (compared to > 6 m without correction).  
Fail: No improvement → verify the division `target_collective / cos_use` is placed
AFTER the PI computation and BEFORE the slew limiter; if placed before the PI, the
integrator fights the correction.

**P1.5.V4 — Stability under combined D term + geometry correction**

Procedure: Both Phase 1 changes active (Kd gains from P1.4, cos_att from P1.5). Run:
1. 2-minute stable hover: no oscillation anywhere
2. Forward 10 m at 3 m/s, hold 5 seconds, return to hover, hold 5 seconds
3. Sidestep 5 m, hold, return

Pass: No oscillation. Altitude within ±3 m throughout. Aircraft returns to level in < 5 s.
Record final log `p1_complete.csv` as the new baseline for Phase 2.  
Fail: New altitude oscillation → `cos_att_alpha` is too fast; reduce to 0.05.

**Phase 1 complete gate:** All of P1.1–P1.5 pass. `p1_complete.csv` recorded.

---

### Phase 2: Horizontal Velocity Hold and KHV

**Goal:** Enable the aircraft to stop itself (KHV) and hold position over a point.  
**Dependencies:** Phase 1 complete (`p1_complete.csv` exists as baseline).

---

#### P2.1 — Add velocity state and implement surface velocity helper

**Code changes:**
1. Add Phase 2 state globals to `ifc_state.ks` and reset in `IFC_INIT_STATE()`
2. Add velocity constants to `ifc_constants.ks`
3. Implement `_VTOL_GET_SURFACE_VEL()` function
4. Call it each tick and store results in `VTOL_VN_ACTUAL`, `VTOL_VE_ACTUAL`
   (read-only — not yet used in control)

**P2.1.V1 — Velocity readout on the ground**

Procedure: Stationary on pad, run armed loop for 10 seconds.

Pass: `VTOL_VN_ACTUAL` and `VTOL_VE_ACTUAL` both within ±0.5 m/s (small Kerbin rotation
contribution is expected).  
Fail: Values > 5 m/s on stationary pad → `IFC_NORTH_VEC` is zero or uninitialized;
confirm the main loop computes and stores this before VTOL tick runs.

**P2.1.V2 — Velocity readout tracks known forward motion**

Procedure:
1. Hover at 10 m AGL
2. Fly north at approximately 5 m/s (by pitch input) for 10 seconds, return to hover

Pass: `VTOL_VN_ACTUAL` rises to approximately 5 m/s during northward flight and returns
near 0 at hover. `VTOL_VE_ACTUAL` stays < 1 m/s during straight north flight.  
Fail: `VTOL_VN_ACTUAL` shows expected magnitude but wrong direction → north/east vectors
are swapped; reverse the cross-product order in `_VTOL_GET_SURFACE_VEL`.

**P2.1.V3 — No behavioral change**

Procedure: Record 60-second hover as `p2.1_check.csv`. Compare to Phase 1 baseline.

Pass: Matches within ±5% on all control channels.  
Fail: Any control change → velocity readout is somehow affecting control; find the write path.

---

#### P2.2 — Implement velocity hold in passthrough mode

**Code changes:**
1. Write `_VTOL_VEL_HOLD(p_pitch, p_roll)` — when `VTOL_VEL_HOLD_ACTIVE = FALSE`,
   returns pilot pitch/roll unchanged (passthrough)
2. When active, computes `theta_cmd` and `phi_cmd` from velocity error (Sections 9.3–9.4)
3. Add `VTOL_KHV_ACTIVE` flag: when TRUE, sets `vN_cmd = vE_cmd = 0`
4. Wire the function into the roll/pitch command path (inactive by default)

**P2.2.V1 — Passthrough mode is lossless**

Procedure: With `VTOL_VEL_HOLD_ACTIVE = FALSE`, run full flight sequence from Phase 1.

Pass: Behavior identical to `p1_complete.csv` baseline.  
Fail: Any behavioral change → the passthrough path modifies the signal; trace the exact
variable that changed and find where the new code touches it.

**P2.2.V2 — No step input on engagement**

Procedure:
1. Hover stable
2. Set `VTOL_VEL_HOLD_ACTIVE = TRUE` with `vN_cmd = vE_cmd = 0` while already hovering

Pass: No jerk or sudden attitude change on engagement. Pitch and roll remain within
±2° of their pre-engagement value.  
Fail: Aircraft jerks on engagement → initial attitude command from vel loop doesn't match
current attitude; seed `VTOL_THETA_CMD = theta_actual` and `VTOL_PHI_CMD = phi_actual`
at engagement.

**P2.2.V3 — KHV decelerates the aircraft**

Procedure:
1. Fly forward at 5 m/s
2. Set `VTOL_KHV_ACTIVE = TRUE`
3. Log velocity for 90 seconds

Pass: `|VTOL_VN_ACTUAL|` reduces to < 1 m/s within 60 seconds. No oscillation in
velocity that grows over time.  
Fail: Velocity reduces to ~2 m/s and stops → the velocity integrator is needed; set
`vtol_vel_ki` from 0 to 0.003 and retry.  
Fail: Velocity oscillates with increasing amplitude → P gain too high; reduce
`vtol_vel_kp` by 30%.

**P2.2.V4 — Attitude limits are respected during KHV**

Procedure: Fly at 8 m/s, activate KHV.

Pass: Maximum pitch during deceleration does not exceed `VTOL_MAX_FWD_PITCH` (15°).  
Fail: Aircraft exceeds 15° pitch → `ax_cmd` is not being clamped before the `arctan`
conversion; verify `CLAMP(ax_cmd, -VTOL_MAX_HORIZ_ACCEL, VTOL_MAX_HORIZ_ACCEL)`.

**P2.2.V5 — Altitude maintained during KHV**

Procedure: Same as V3. Log altitude throughout.

Pass: Altitude deviation < 5 m during the entire KHV stop.  
Fail: Altitude drops during deceleration pitch → confirm Phase 1 geometry correction
is active (`VTOL_DIAG_COS_ATT_FILT` should show < 1.0 during the nose-down deceleration).

---

#### P2.3 — Implement hover-at-point

**Code changes:**
1. Implement `_VTOL_POS_HOLD()`: reads `VTOL_TARGET_LAT/LNG`, computes distance and
   bearing via `GEO_DISTANCE` / `GEO_BEARING`, runs PI to output `vN_cmd`, `vE_cmd`
2. Add position state globals and constants
3. Default: `VTOL_POS_HOLD_ACTIVE = FALSE`

**P2.3.V1 — Position error measurement accuracy**

Procedure:
1. Record current position, set as target
2. Manually fly exactly 20 m east (use heading 090°, watch distance on display)
3. Read the displayed position error

Pass: Error reads within 10% of 20 m. Bearing to target is approximately 270° (west).  
Fail: Error is wrong magnitude → `GEO_DISTANCE` is returning radians, not meters; multiply
the result by `SHIP:BODY:RADIUS` if needed.  
Fail: Bearing is 90° off → the bearing formula has a 90-degree convention error.

**P2.3.V2 — Aircraft returns from an offset**

Procedure:
1. Hover at target location, activate position hold
2. Manually push aircraft 25 m east using brief stick input, then release
3. Wait 90 seconds

Pass: Aircraft returns to within 5 m of original target without overshoot.  
Fail: Overshoots and oscillates → position P gain too high; reduce `VTOL_POS_KP` by 30%.  
Fail: Stops 15 m away → the position integrator isn't converging; verify integrator is
only running when within `VTOL_POS_INT_RADIUS` of the target.

**P2.3.V3 — Disturbance rejection**

Procedure:
1. Hover at hold point
2. Apply three brief stick pulses in random directions, one every 15 seconds
3. Wait 30 seconds after the last pulse

Pass: Aircraft returns to within 5 m of hold point after each disturbance.  
Fail: Each disturbance creates a new permanent offset → position integrator is not
running; confirm `VTOL_POS_INT_N/E` are updating when within the integration radius.

**Phase 2 complete gate:** All P2.1–P2.3 pass. Record `p2_complete.csv` as the new baseline.

---

### Phase 3: Physical Moment Allocation Refactor

**Goal:** Replace normalized mixing-matrix allocation with physically-derived per-engine
moment allocation. This is an internal refactor — external hover behavior should be
equivalent after gain re-tuning.  
**Dependencies:** Phase 2 complete.

---

#### P3.1 — Pre-compute moment arms at VTOL_DISCOVER

**Code changes:**
1. In `VTOL_DISCOVER`, compute `VTOL_ARM_ROLL_M` = largest `|y_i|` across all engines
2. Compute `VTOL_ARM_PITCH_M` = largest `|x_i|` across all engines
3. Store in state globals, log to discovery event

**P3.1.V1 — Arm lengths match physical aircraft**

Procedure: Run `VTOL_DISCOVER` on the pad. Check logged arm values.

Pass: `VTOL_ARM_ROLL_M` is within 10% of the measured distance from CoM to wingtip
nacelle (measured in the KSP editor). `VTOL_ARM_PITCH_M` is within 10% of the
CoM-to-nacelle fore-aft distance.  
Fail: Arm reads as 0 → FOREVECTOR/STARVECTOR projection is using world-frame instead
of body-frame; confirm the offset is computed using the body-frame vectors cached at
discovery time.

---

#### P3.2 — Implement symmetric closed-form allocation with feature flag

**Code changes:**
1. Add config flag `vtol_physical_alloc_enabled` (default FALSE)
2. When FALSE: use existing mixing matrix (unchanged path)
3. When TRUE: use closed-form allocation from Section 7.3
4. The flag allows A/B testing in the same flight session

**P3.2.V1 — Old path unchanged with flag OFF**

Procedure: Set flag to FALSE, hover 60 seconds. Compare to Phase 2 baseline.

Pass: No behavioral change. The new code path is never executed.  
Fail: Any change → the flag check itself modifies a global; inspect the branch.

**P3.2.V2 — New allocation produces same moment for a reference input**

Procedure (offline calculation before flight):
1. Choose a specific roll command (e.g., roll_cmd = 0.2) and collective (0.5)
2. Calculate old path engine limits manually using the mixing matrix
3. Calculate new path engine limits using the closed-form formula
4. They should produce the same roll moment (verify: `(T_FL + T_AL) - (T_FR + T_AR)` is
   the same in both cases after appropriate scaling)

Pass: Moments match within 5%.  
Fail: Moments differ → the arm normalization convention is different between old and new
paths; compute the scaling factor that maps between them.

**P3.2.V3 — Hover quality is equivalent after gain re-tuning**

Procedure:
1. Enable flag, re-tune rate gains so hover behavior looks similar to baseline
2. Hover 60 seconds

Pass: `VTOL_DIAG_LIMIT_SPAN` (engine limit span) is similar to baseline at the same
collective. No new oscillation.  
Fail: Limit span significantly different → the per-engine authority has changed; re-scale
the moment commands to match.

**Phase 3 complete gate:** All P3.1–P3.2 pass with gains re-tuned and documented.

---

### Phase 4: Transition to Forward Flight

**Goal:** Smooth, stable transition from hover to forward flight.  
**Dependencies:** Phase 3 complete. Stable hover and hover-at-point required.

---

#### P4.1 — Nacelle collective angle command

**Code changes:**
1. Add `VTOL_NACELLE_ALPHA_CMD` state global, initialized to `vtol_hover_angle`
2. Modify `_VTOL_APPLY_SERVOS` to use `VTOL_NACELLE_ALPHA_CMD` instead of `hover_angle`
3. Add slew limiter on `VTOL_NACELLE_ALPHA_CMD` at rate `VTOL_NACELLE_SLEW_DPS`

**P4.1.V1 — Manual nacelle command works**

Procedure: Hover stable. Manually set `VTOL_NACELLE_ALPHA_CMD = 80` in code.

Pass: All servos move to 80°. Yaw differential still produces expected yaw response (servos deviate from 80° symmetrically).  
Fail: Servos don't move → `_VTOL_APPLY_SERVOS` is reading the old hardcoded `hover_angle`; confirm the variable substitution.

**P4.1.V2 — Slew rate is respected**

Procedure: Command a step from 90° to 70° while hovering.

Pass: Servos move at approximately `VTOL_NACELLE_SLEW_DPS` deg/s, not instantaneously.
It takes about `20 / VTOL_NACELLE_SLEW_DPS` seconds to reach 70°.  
Fail: Nacelles jump immediately → slew limiter is applied after the servo write, not before.

---

#### P4.2 — Collective feedforward for nacelle tilt

**Code changes:**
1. When nacelles are not at hover angle, boost collective by `1 / sin(VTOL_NACELLE_ALPHA_CMD)`
   to compensate for reduced vertical thrust component
2. Apply only when `sin(VTOL_NACELLE_ALPHA_CMD) > 0.1` (don't apply at very small angles)

**P4.2.V1 — Altitude held during nacelle tilt**

Procedure: With alt-hold active, tilt nacelles from 90° to 70° over 10 seconds.

Pass: Altitude deviation < 5 m throughout the 20° tilt.  
Fail: Altitude drops significantly → feedforward is not applied before the slew limiter;
it must be in the VS-hold output path, applied to the collective AFTER the PI and slew.

---

#### P4.3 — Airspeed-based transition schedule

**Code changes:**
1. Implement schedule: `VTOL_NACELLE_ALPHA_CMD` driven by IAS between
   `VTOL_TRANS_START_IAS` and `VTOL_TRANS_END_IAS`
2. Add `VTOL_TRANS_ACTIVE` flag to enable/disable the schedule
3. Add minimum airspeed constraint (Section 12.2)

**P4.3.V1 — Schedule is linear in IAS**

Procedure: Record `VTOL_NACELLE_ALPHA_CMD` and IAS during a forward acceleration from
0 to 100 m/s IAS.

Pass: `VTOL_NACELLE_ALPHA_CMD` decreases linearly from 90° at `TRANS_START_IAS` to 0°
at `TRANS_END_IAS`. No jumps or discontinuities.  
Fail: Step in schedule → the IAS variable has an outlier; add a rate-limiter on the IAS
reading used for the schedule (low-pass filter IAS for scheduling purposes only).

**P4.3.V2 — Minimum airspeed constraint prevents over-tilting at low speed**

Procedure:
1. Enable transition at an airspeed below `VTOL_TRANS_START_IAS` (e.g., commanded at 0 m/s)

Pass: Nacelles do not tilt below `α_min` calculated from current aerodynamic lift fraction.
The constraint log shows the aircraft is constrained.  
Fail: Nacelles tilt to 0° at zero airspeed → minimum airspeed constraint is not applied;
add `SET VTOL_NACELLE_ALPHA_CMD TO MAX(VTOL_NACELLE_ALPHA_CMD, alpha_min).`

**P4.3.V3 — Full transition flight test**

Procedure:
1. Start hover at 200 m AGL
2. Fly forward, activate transition at 35 m/s IAS
3. Accelerate to 100 m/s IAS
4. Log altitude, nacelle angle, collective, `THROTTLE_CMD`

Pass:
- Altitude deviation < ±10 m throughout
- Nacelle angle smoothly decreases from 90° to 0°
- `THROTTLE_CMD` transitions from 1.0 toward AT-commanded value
- Aircraft reaches cruise mode with normal AT active

Fail: Altitude drops > 10 m → increase collective feedforward gain or reduce nacelle
slew rate.  
Fail: Aircraft pitches up uncontrollably during transition → attitude command blending
from Section 12.4 is missing; the hover pitch command must be blended out as hover_blend
decreases.

**P4.3.V4 — Return transition (cruise to hover)**

Procedure: Decelerate from cruise to hover by reversing the nacelle schedule.

Pass: Altitude deviation < ±10 m. Nacelles return to 90°. Collective re-takes control
from AT. Pilot can stabilize in hover on the first attempt.  
Fail: Altitude climbs uncontrollably during tilt-back → the collective feedforward was
not reversed; the transition feedforward must apply symmetrically in both directions.

**Phase 4 complete gate:** All P4.1–P4.3 pass. Full transition cycle (hover → cruise →
hover) completes without pilot intervention to maintain altitude.

---

### Phase Completion Summary

| Phase | Primary change | Measurable pass criterion | Gate log file |
|-------|---------------|--------------------------|---------------|
| 1 | D term + geometry correction | Altitude during 15° bank < 3 m; roll settling < 5 s | `p1_complete.csv` |
| 2 | Velocity hold + KHV | KHV stop in < 60 s; position hold within 5 m | `p2_complete.csv` |
| 3 | Physical moment allocation | Equivalent hover after re-tuning; saturation preserves collective | `p3_complete.csv` |
| 4 | Transition | Full hover→cruise→hover with altitude deviation < 10 m | `p4_complete.csv` |

**Between phases:** Always compare the new baseline log to the previous-phase log before
declaring a phase complete. A phase introduces new improvements; if the new log shows
regression on the previous phase's pass criterion, the regression must be found and fixed
before calling the phase done.

---

*End of VTOL Controller Plan 2*
