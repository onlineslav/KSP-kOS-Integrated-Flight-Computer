# Measurement-Driven Ascent Guidance for Dual-Mode Spaceplanes
### A Design Framework for kOS and Ferram Aerospace Research — Revised Edition
*Kerbal Space Program Advanced Guidance Series*

---

## Abstract

This paper presents a design framework for an energy-optimal ascent guidance system for dual-mode spaceplanes operating in Kerbal Space Program, using the kOS scripting environment and Ferram Aerospace Research (FAR) aerodynamics. The core proposal is a measurement-driven controller that compares the instantaneous usefulness of air-breathing and rocket propulsion modes in terms of specific orbital energy gained per unit of propellant consumed, using live force data available from the kOS-FAR interface.

Unlike conventional approaches based on fixed pitch schedules, altitude thresholds, or dynamic pressure triggers, the system described here grounds every decision in a physically meaningful metric derived from first principles of orbital mechanics. The controller is structured as a layered architecture: a frame-consistent state estimator feeds a dual-channel mode valuation system, which drives a state machine governing five explicit flight phases. Guidance outputs are derived from orbital error terms rather than hardcoded trajectory profiles, enabling robust performance across a wide range of vehicle designs with minimal per-vehicle tuning.

This revised edition incorporates several improvements arising from peer review: a propellant equivalency weighting factor to account for the different mission value of liquid fuel versus liquid fuel/oxidizer; a corrected drag estimation strategy for J_rk during the zoom-climb phase; explicit aerodynamic gain scheduling for the pitch controller; derivative-on-measurement PID implementation to prevent transition kick; structural separation of valuation-signal and control-signal smoothing parameters; a dedicated low-pass filter on the drag term inside the rocket value estimator; an is_spooling flag to manage the mixed-propulsion transition window; and a formal distinction between engine thermal regime limits and true flameout events.

---

## 1. Introduction

### 1.1 The Problem

Spaceplanes equipped with dual-mode propulsion — engines capable of operating in either air-breathing (open-cycle) or oxidizer-fed (closed-cycle) mode — present a fundamentally different ascent optimisation problem than conventional rockets. The air-breathing phase allows the vehicle to exploit atmospheric oxygen as the oxidiser, dramatically improving specific impulse and therefore the quantity of propellant required to reach orbit. However, this advantage degrades with altitude as atmospheric density falls, and at some point the switch to closed-cycle rocket operation becomes energetically favourable.

The central question is not simply "when should the vehicle switch modes?" but rather two coupled questions: what does the optimal air-breathing trajectory look like, and how should the transition to rocket propulsion be executed in order to arrive at a target circular orbit with minimum propellant expenditure?

Existing approaches to this problem in the Kerbal Space Program community typically rely on one of three methods: manually flown ascent profiles developed through trial and error; fixed pitch schedules embedded in kOS scripts that command specific pitch angles at specific Mach numbers or altitudes; or dynamic pressure triggers that initiate mode transitions when a threshold q value is reached. All three approaches share a common weakness — they are tuned to a specific vehicle and break silently when applied to a different design.

### 1.2 The Opportunity

The combination of kOS and Ferram Aerospace Research creates an unusually rich sensing environment for a game-based flight controller. FAR exposes the total aerodynamic force vector, drag coefficient, lift coefficient, dynamic pressure, Mach number, and angle of attack. kOS provides vessel-level and engine-level thrust, available thrust at arbitrary atmospheric pressure, mass flow, fuel flow, specific impulse, and full orbital state. Together, these data sources are sufficient to implement a controller grounded not in scheduled heuristics but in online estimation of the physical quantities that actually determine energetic optimality.

This paper describes how to exploit that sensing capability to build a guidance system whose core decision-making is derived from orbital mechanics and practical control engineering, implemented in a form that is robust to the noise, frame alignment issues, and vehicle diversity inherent in the KSP environment.

### 1.3 Scope and Assumptions

The design described here assumes a KSP installation running kOS and FAR, with the kOS-Ferram interface available via the `ADDONS:FAR` namespace. The vehicle is assumed to have separately controllable air-breathing and rocket engines, and to be capable of sustained flight in both modes within the relevant portions of the atmosphere. The target is a circular low Kerbin orbit, though the framework generalises to any target orbital altitude.

This paper does not address takeoff and initial climb to air-breathing engagement speed, nor does it address rendezvous or deorbit. The guidance begins at the point where the air-breathing engines are producing useful thrust and the vehicle is climbing toward its optimal ascent corridor.

---

## 2. Theoretical Foundation

### 2.1 Specific Orbital Energy

The appropriate figure of merit for ascent optimisation is specific orbital energy, defined as:

```
E = v²/2 - μ/r
```

where *v* is the magnitude of the inertial velocity vector, *μ* is the gravitational parameter of Kerbin, and *r* is the distance from Kerbin's centre. This quantity captures both the kinetic and potential components of the vehicle's energy in a way that is directly relevant to orbit insertion — a vehicle reaches orbit when its specific orbital energy equals the energy of the target circular orbit.

The time rate of change of specific orbital energy along a powered trajectory is:

```
Ė = v · (T + F_aero) / m
```

where **v** is the inertial velocity vector, **T** is the total thrust vector, **F_aero** is the total aerodynamic force vector, and *m* is the vehicle mass. This expression follows from differentiating the energy equation and substituting the equations of motion. Its significance is that it directly quantifies how quickly the vehicle is accumulating orbital energy — the quantity that determines when orbit is achieved.

Crucially, this formulation is frame-dependent in a way that demands care: the dot products must be computed in a consistent inertial frame. Projecting vectors from different frames before combining them produces a corrupted signal that degrades silently.

### 2.2 Energy Efficiency of Propulsion

The key insight for mode comparison is to normalise the energy gain rate by the propellant consumption rate. Define the mode value *J* as:

```
J = Ė / ṁ_prop = [v · (T + F_aero) / m] / ṁ_prop
```

where *ṁ_prop* is the total propellant mass flow rate in the current mode. This quantity answers the question: how much useful orbital energy am I gaining per kilogram of propellant burned right now? When J_ab (the air-breathing value) exceeds J_rk (the rocket value), continuing in air-breathing mode is energetically preferable. When J_rk exceeds J_ab, switching to rocket mode produces more orbital energy per kilogram of propellant.

This framing resolves the mode selection problem at its correct level of abstraction. It avoids the common mistake of using altitude, dynamic pressure, or Mach number as primary switch variables — these are correlates of the underlying energetics, not the energetics themselves. A dynamic pressure threshold can be derived as an approximation to the J_ab = J_rk condition under simplifying assumptions, but those assumptions break down at the margins, which is precisely where the switch decision matters most.

### 2.3 Force Projection Along the Velocity Vector

The energy rate equation requires the along-track component of the net force. Since kOS provides thrust as a scalar (or per-engine) and FAR provides the aerodynamic force as a vector in the ship body frame, the correct procedure is:

- Define a canonical inertial frame for all computations.
- Express the thrust vector in that frame using the vessel's attitude quaternion.
- Transform the FAR aerodynamic force vector from the ship raw frame into the same inertial frame.
- Project both vectors onto the unit inertial velocity vector.
- Sum the projections to obtain the along-track net force.

The along-track net force is the physically meaningful quantity for energy rate computation. The cross-track components contribute to trajectory shaping (turning) but do not directly accumulate orbital energy along the current velocity direction. Using the full force magnitude rather than the projection introduces an error that depends on the vehicle's angle of attack and the alignment between thrust and velocity — errors that are small in cruise but potentially significant during the zoom-climb phase where the vehicle is pitched up aggressively.

### 2.4 The Optimal Air-Breathing Corridor

Optimal control analysis of the air-breathing ascent phase, under idealised assumptions, suggests that the energy-optimal trajectory follows a path of approximately constant dynamic pressure:

```
q = ½ρv²
```

The intuition is that air-breathing engine performance (thrust, specific impulse, intake mass flow) scales strongly with dynamic pressure. Flying at constant *q* keeps the engine near its performance optimum while gaining both altitude and speed along the path of increasing inertial velocity. As the vehicle accelerates, altitude must increase proportionally to maintain constant *q* as atmospheric density falls.

However, this result is a useful prior rather than an inviolable law. The constant-q result assumes smooth atmospheric density profiles, constant engine efficiency, and no thermal or structural constraints. In practice, the optimal trajectory deviates from constant *q* when heating limits the allowable angle of attack, when engine efficiency varies non-monotonically with Mach number, or when the vehicle's mass fraction evolution changes the optimal trade-off between drag and climb rate. The controller should treat the constant-q corridor as a soft attractor, not a hard reference to track.

### 2.5 The Zoom Climb

As the air-breathing mode approaches its useful limit — either because dynamic pressure has fallen below the threshold where J_ab > J_rk, or because thermal or structural constraints are approaching — it is advantageous to trade residual kinetic energy for altitude before transitioning to rocket mode. This zoom climb raises the apoapsis, reducing the atmospheric burden on the early rocket phase and improving the efficiency of the subsequent powered ascent.

The benefit of the zoom is trajectory-integrated, not reducible to a simple formula. It depends on the atmospheric density profile encountered during the rocket phase, the thrust-to-weight ratio of the rocket engines, and the geometry of the orbital insertion. The controller should treat the zoom as an explicit flight phase with its own entry and exit conditions, rather than as a smooth transition embedded in the corridor-following logic.

### 2.6 Rocket Ascent Optimality

Once in rocket mode, formal optimal control theory — in particular the Pontryagin minimum principle — suggests that the fuel-optimal thrust direction should initially act to raise apoapsis efficiently, then transition to a near-horizontal burn to accumulate orbital velocity as the vehicle approaches the target altitude. In a pure vacuum, this yields the Bilinear Tangent Steering law. In an atmosphere, the coupled aero-gravity problem becomes analytically intractable.

The practical implementation described in this paper does not attempt to solve the formal atmospheric optimal control problem. Instead, it uses an error-driven pitch law — driving apoapsis error and time-to-apoapsis toward their targets — as a well-understood engineering heuristic that approximates the optimal solution in the operational regime of interest. This is standard practice in atmospheric ascent guidance and does not undermine the theoretical grounding of the system; it simply acknowledges that the correct engineering response to an analytically intractable problem is a validated approximation, not a pretended closed-form solution.

The gravity loss term:

```
ΔV_grav = ∫ g sin(γ) dt
```

is minimised by keeping the flight path angle *γ* small. This is a terminal condition of the optimal solution, not a control objective to pursue from the beginning of the rocket phase. Attempting to flatten the trajectory prematurely — before apoapsis is sufficiently established — produces a trajectory that skims the upper atmosphere at low altitude, accumulating gravity and drag losses.

---

## 3. System Architecture

### 3.1 Layered Design

The guidance system is organised into four distinct layers, each with a clearly defined responsibility and a well-specified interface to adjacent layers. This separation is not merely organisational — it is a correctness requirement. The failure modes that most commonly destroy guidance systems of this class arise from collapsing these layers, allowing downstream logic to operate on unvalidated inputs or causing upstream estimation to be contaminated by guidance decisions.

| Layer | Name | Inputs | Outputs |
|-------|------|--------|---------|
| 1 | State Estimator | Raw kOS / FAR sensors | Smoothed, frame-consistent state vector |
| 2 | Mode Valuator | State vector | J_ab, J_rk, estimator validity |
| 3 | Phase Logic | Mode values, orbital state | Current flight phase, transition decisions |
| 4 | Guidance Output | Current phase, orbital errors | Thrust direction, throttle, engine mode |

Each layer is permitted to consume outputs only from the layer immediately below it. No layer may read raw sensor data other than Layer 1, and no layer may command vehicle outputs other than Layer 4. This invariant makes it possible to validate each layer independently and to diagnose failures by layer rather than by global symptom.

### 3.2 The Frame Invariant

A single constraint governs all vector operations in the system: **no value function may consume vectors from mixed reference frames.** This is not a guideline but a hard invariant, enforced by design. It is stated here prominently because frame misalignment is the most operationally dangerous failure mode in this class of system. The symptoms — subtly wrong value estimates, attitude-dependent instability, apparent tuning sensitivity — resemble controller weakness rather than a fundamental data error, and can waste significant debugging time before the root cause is identified.

The system adopts the orbital (inertial) frame as its canonical computation frame. All vectors produced by Layer 1 are expressed in this frame before being passed to Layer 2. The required transformations are:

- **FAR aerodynamic force vector:** `ADDONS:FAR:AEROFORCE` is in the ship body frame (`SHIP:RAW`). Transform to orbital frame using the vessel orientation quaternion.
- **Thrust vector:** kOS provides thrust magnitude per engine. Construct the thrust vector in the ship frame from each engine's facing vector, then rotate to orbital frame using the same orientation quaternion.
- **Velocity vector:** `SHIP:VELOCITY:ORBIT` is already in the orbital frame and requires no transformation.

All three transforms should be computed once per update cycle and stored as frame-consistent quantities. Downstream functions receive only the transformed vectors and are structurally prohibited from performing their own frame operations.

### 3.3 State Machine

The flight is partitioned into five explicit phases, each with defined entry conditions, active control objectives, and exit conditions:

| Phase | Entry Condition | Control Objective | Exit Condition |
|-------|----------------|-------------------|----------------|
| AB_CORRIDOR | Air-breathing engaged | Maintain q corridor, maximise J_ab | J_ab declining, zoom warranted |
| AB_ZOOM | Zoom criteria met | Raise apoapsis, preserve AB usefulness | J_rk > J_ab sustained, or limits reached |
| ROCKET_SUSTAIN | Mode switch committed | Raise apoapsis to target | Apoapsis within target band |
| ROCKET_CLOSEOUT | Apoapsis established | Accumulate horizontal velocity | Orbital velocity achieved |
| CIRCULARISE | Near apoapsis | Circularisation burn | Orbit closed |

The state machine enforces an important structural property: phase transitions are one-way within a flight (with the exceptions noted in the edge cases section). There is no reversion from ROCKET_SUSTAIN to AB_ZOOM, for example. This prevents oscillation between phases and ensures that the persistence logic for mode switching is not undermined by phase reversals.

---

## 4. Layer 1 — State Estimation

### 4.1 Design Principles

The state estimator is the root of truth for the entire system. Every decision made in Layers 2 through 4 is only as valid as the quantities produced here. The correct build order — validate the estimator first, build everything else second — follows directly from this dependency structure.

The estimator must satisfy three requirements. First, all output quantities must be expressed in the canonical orbital frame. Second, all derivative quantities must be computed from smoothed signals, not from frame-to-frame differences. Third, every output quantity must carry an associated validity flag indicating whether it is trustworthy enough to drive downstream decisions.

### 4.2 Raw Inputs and Sources

| Quantity | kOS / FAR Source | Notes |
|----------|-----------------|-------|
| Inertial velocity vector | `SHIP:VELOCITY:ORBIT` | Already in orbital frame |
| Surface velocity vector | `SHIP:VELOCITY:SURFACE` | For aero control reference |
| Vessel mass | `SHIP:MASS` | Updated each cycle |
| Altitude | `SHIP:ALTITUDE` | Above sea level |
| Apoapsis altitude | `SHIP:ORBIT:APOAPSIS` | Orbital frame |
| Time to apoapsis | `ETA:APOAPSIS` | Used for trajectory shaping |
| Dynamic pressure | `ADDONS:FAR:DYNPRES` | Cross-check with q = ½ρv² |
| Mach number | `ADDONS:FAR:MACH` | Engine performance reference; also used for gain scheduling |
| Angle of attack | `ADDONS:FAR:AOA` | Controllability assessment |
| Aerodynamic force vector | `ADDONS:FAR:AEROFORCE` | In SHIP:RAW frame — must transform |
| Drag coefficient | `ADDONS:FAR:CD` | Supplementary diagnostic |
| Vessel orientation | `SHIP:FACING` | Quaternion for frame transforms |
| AB engine thrust | Per `ENGINE:THRUST` (filtered) | Sum over AB engines |
| AB mass flow | Per `ENGINE:MASSFLOW` | Sum over AB engines |
| Rocket thrust (available) | `ENGINE:AVAILABLETHRUSTAT(pressure)` | For J_rk estimation |
| Rocket mass flow (estimated) | `ENGINE:MAXTHRUST / ENGINE:ISP / g0` | For J_rk estimation |
| Liquid Fuel mass | `SHIP:LIQUIDFUEL` | Separate tracking for propellant equivalency |
| Oxidizer mass | `SHIP:OXIDIZER` | Separate tracking for propellant equivalency |

### 4.3 Smoothing Strategy: Valuation Signals vs. Control Signals

A critical distinction that must be made explicit in the estimator design is the difference between **valuation signals** and **control signals**. These two categories have fundamentally different smoothing requirements, and conflating them produces a system that is either sluggish in control or noisy in valuation.

**Control signals** — quantities that feed directly into pitch commands and throttle outputs — must respond quickly to avoid divergence. Lag in a control signal means the vehicle overshoots its target state before the controller reacts. Control signals should use a relatively high EMA coefficient (α = 0.3–0.5) or a short rolling window (0.5–1.0 seconds).

**Valuation signals** — quantities that feed into J_ab and J_rk — drive mode-switching decisions, not instantaneous vehicle commands. The cost of a wrong mode decision over two seconds is substantially lower than the cost of a wrong pitch command over two seconds. Valuation signals can therefore tolerate more lag in exchange for greater stability. They should use a lower EMA coefficient (α = 0.1–0.2) or a longer rolling window (1.5–3.0 seconds).

The following table specifies the signal category and recommended smoothing for each key quantity:

| Signal | Category | EMA α | Window | Notes |
|--------|----------|-------|--------|-------|
| Along-track aero force (for pitch) | Control | 0.4 | 0.5 s | Fast response needed |
| Dynamic pressure (for pitch bias) | Control | 0.35 | 0.75 s | q tracking |
| AoA | Control | 0.4 | 0.5 s | Stability-critical |
| Along-track aero force (for J_ab) | Valuation | 0.15 | 2.0 s | Separate from control copy |
| Along-track drag (for J_rk) | Valuation | 0.10 | 3.0 s | Dedicated filter — see §5.3 |
| Specific orbital energy Ė | Valuation | 0.15 | 2.0 s | Mode comparison input |
| Apoapsis altitude | Valuation | 0.20 | 1.5 s | Phase transition input |
| Mass flow (AB) | Valuation | 0.20 | 1.5 s | J_ab denominator |
| Mass flow (rocket, estimated) | Valuation | 0.20 | 1.5 s | J_rk denominator |

The along-track aerodynamic force therefore exists in **two separate smoothed copies** inside the state estimator: one for control use (fast, responsive) and one for valuation use (slow, stable). These must be maintained and passed separately; downstream functions must not substitute one for the other.

For a quantity *x*, the smoothed value *x_s* is updated each cycle as:

```
x_s ← α · x_raw + (1 - α) · x_s
```

Derivative quantities such as the rate of change of specific orbital energy and the rate of change of apoapsis altitude should be computed as windowed differences over the specified window length, not as instantaneous frame deltas.

### 4.4 Specific Orbital Energy Computation

The specific orbital energy is computed directly from the orbital state:

```
E = |v_orb|²/2 - μ / (R_Kerbin + altitude)
```

where *v_orb* is the magnitude of the orbital velocity vector, *μ* is Kerbin's gravitational parameter (3.5316 × 10¹² m³/s²), and *R_Kerbin* is Kerbin's equatorial radius (600,000 m). The rate of change of E is then estimated from the smoothed windowed difference of the E time series, not from the force equation (which is used separately as a cross-check and for the J computation).

This dual computation — *E* from the orbital state, *Ė* from the force projection — provides a consistency check. If the two estimates diverge significantly, it is a signal that either the force projection is corrupted (likely a frame error) or the orbital state computation contains an error. Both should be logged at high frequency during initial validation.

### 4.5 Estimator Validity

The estimator maintains a validity state with three levels: VALID, DEGRADED, and INVALID. The downstream consequences of each state are explicit and defined before the system is implemented, not discovered during debugging.

| Validity State | Entry Condition | J_ab / J_rk Behaviour | Transition Logic |
|---------------|----------------|----------------------|-----------------|
| VALID | All checks pass | Normal computation | Normal persistence logic |
| DEGRADED | One check failing | Widen persistence window ×2 | Conservative: require larger margin |
| INVALID | Multiple checks failing or AoA oscillating | Freeze J values at last valid | Lock to simpler mode; block new transitions |

Specific validity checks include:

- **AoA oscillation check:** if the angle of attack signal is oscillating with amplitude greater than a threshold (e.g., 3 degrees) over a 2-second window, controllability is compromised and force projections are unreliable.
- **Force balance plausibility:** the computed Ė from force projection should agree with the windowed orbital E derivative to within a tolerance. Persistent disagreement indicates a frame or transform error.
- **Thrust plausibility:** summed engine thrust should be consistent with observed vehicle acceleration in the near-prograde direction.
- **Signal variance check:** if the variance of the aerodynamic force vector exceeds a threshold over a short window, the aero sensing is unreliable (possibly due to rapid attitude changes or simulation artefacts).

---

## 5. Layer 2 — Mode Valuation

### 5.1 The Core Metric

The mode valuation layer computes two scalars — J_ab and J_rk — representing the energetic usefulness of air-breathing and rocket propulsion respectively. These are not abstract scores but physically grounded estimates of specific orbital energy gain rate per unit propellant mass flow, with penalty multipliers for conditions that compromise either the accuracy of the estimate or the long-term utility of remaining in the current mode.

All inputs to this layer come from the valuation-smoothed copies of the state vector, not the control-smoothed copies. This is a hard requirement — substituting control-signal copies would inject high-frequency noise into the mode comparison.

### 5.2 Air-Breathing Mode Value

The primary J_ab computation uses the live force projection:

```
J_ab = [|v_orb| · (T_ab_along + F_aero_along_val) / m] / ṁ_ab · P_q · P_heat · P_ctrl · P_traj
```

where T_ab_along is the along-track component of the air-breathing thrust vector, F_aero_along_val is the **valuation-smoothed** along-track aerodynamic force component, *ṁ_ab* is the air-breathing propellant mass flow (liquid fuel only), and the P terms are penalty multipliers bounded in [0, 1].

#### 5.2.1 Penalty Terms

**P_q — Dynamic pressure penalty:** Applies when dynamic pressure falls below the lower bound of the optimal corridor or rises above the structural limit. The corridor bounds are vehicle-specific parameters representing physical limits on the vehicle's structural and thermal integrity. P_q degrades smoothly from 1.0 at the centre of the corridor to 0 at either boundary.

**P_heat — Thermal penalty:** Applies when the surface heating rate or a proxy for it (such as *q · v³*, proportional to aerodynamic heating) approaches the vehicle's thermal tolerance. P_heat provides a soft warning before a hard structural failure condition is reached.

**P_ctrl — Controllability penalty:** This is an epistemic penalty as much as a dynamic one. When the vehicle cannot stably realise commanded attitudes — indicated by AoA oscillation, control surface saturation, or pitch error exceeding a threshold — the force projections themselves become untrustworthy. P_ctrl should therefore be treated as a validity gate: when it falls below a threshold, it triggers DEGRADED or INVALID estimator status rather than merely reducing J_ab.

**P_traj — Trajectory penalty:** Applies when the current trajectory is no longer producing acceptable apoapsis growth. If the vehicle is flying flat and fast in air-breathing mode but apoapsis altitude is stagnant or declining, the trajectory is not serving the mission objective. This penalty prevents the local optimality of the J metric from being exploited in ways that are globally suboptimal — a vehicle can achieve excellent J_ab values by flying fast at low altitude while failing to build apoapsis, leaving the rocket phase with a severe gravity-turn problem.

#### 5.2.2 Why Penalty Terms are Not Optional

The core metric J_ab = Ė / ṁ is a local optimality condition. Local optimality does not guarantee global optimality. The penalty structure is the mechanism by which local decisions are kept consistent with the mission objective. P_traj in particular is not a safety layer — it is a structural requirement of the optimisation.

### 5.3 Rocket Mode Value Estimate

J_rk is inherently an estimate because the vehicle is not yet in rocket mode when the comparison is being made. Two important refinements distinguish this estimate from a naive implementation.

#### 5.3.1 Propellant Equivalency Weighting

In KSP, air-breathing engines consume pure liquid fuel (LF), while rocket engines consume a liquid fuel and oxidizer mixture (LFO). Because 1 kg of LF and 1 kg of LFO do not have equal scarcity value for the overall mission — a vehicle may have abundant LF reserves but limited oxidizer — a naive comparison of ṁ across modes can misrepresent the true mission cost of staying in each mode.

The corrected J_rk includes a propellant equivalency weight w_prop:

```
J_rk = [|v_orb| · (T_rk_avail - D_rk_est) / m] / (ṁ_rk_est · w_prop) · P_atm · P_traj_rk
```

where w_prop is computed from remaining reserves:

```
w_prop = 1 + k_prop · max(0, (LFO_fraction_remaining - LF_fraction_remaining))
```

Here, LFO_fraction_remaining and LF_fraction_remaining are the remaining propellant as fractions of their respective maximum tank capacities, and k_prop is a tuning coefficient (nominally 0.5). When oxidizer is scarcer relative to liquid fuel than their proportional burn fractions suggest, w_prop > 1, making J_rk appear more expensive and biasing the controller toward staying in air-breathing mode longer. When reserves are balanced, w_prop ≈ 1 and the correction vanishes. This correction is most significant for spaceplanes with drop tanks or asymmetric fuel-to-oxidizer tank ratios.

#### 5.3.2 Drag Estimation During Zoom Climb

A conservative drag assumption for J_rk — using the current measured drag as the rocket-phase drag estimate — is appropriate during level corridor flight, where drag is relatively stable. However, during the zoom-climb phase, this assumption breaks down. The vehicle is actively decelerating and climbing; drag is falling rapidly. Using the high-q corridor drag value to discount J_rk during the zoom artificially suppresses the rocket mode's apparent value at exactly the moment the switch should be triggering, causing the controller to hold onto a dying air-breathing mode too long.

The corrected D_rk_est uses a drag derivative projection during the zoom phase:

```
D_rk_est = max(D_current + dD/dt · t_lookahead, D_altitude_floor)
```

where dD/dt is the smoothed drag rate of change (which is negative during zoom climb), t_lookahead is a short projection horizon (nominally 3–5 seconds), and D_altitude_floor is an altitude-based lower bound on drag derived from the standard atmosphere at the current Mach number. The altitude floor prevents the projection from producing unphysically small drag estimates if the drag derivative is temporarily noisy. During corridor flight (not zoom), D_rk_est reverts to D_current with its normal conservative value.

#### 5.3.3 Dedicated Drag Filter for J_rk

FAR's aerodynamic calculations are particularly sensitive to Mach-transonic wave drag spikes, dynamic pressure fluctuations, and minor angle-of-attack changes. If the drag term inside J_rk is smoothed with the same parameters as the drag term used for pitch control, transonic spikes will inject high-frequency noise into J_rk, causing the J_ab < J_rk trigger condition to flip prematurely during single update cycles.

The drag term used inside J_rk must therefore receive a **dedicated, heavier low-pass filter** separate from all other drag signals in the system. The recommended parameters are α = 0.10 and a 3.0-second rolling window, as specified in the smoothing table in Section 4.3. This is a longer window than any other signal in the system. The cost is increased lag in the drag estimate used for mode comparison; the benefit is that transonic noise cannot cause a premature mode switch. This is the correct trade-off because the drag term in J_rk is not used for any real-time control purpose — it exists solely to assess the long-term value of switching modes.

### 5.4 Structural Separation: Mode Value vs. Trajectory Value

A critical architectural requirement is that the mode comparison (J_ab vs. J_rk) and the trajectory-shaping decision (corridor vs. zoom) must be implemented as separate, non-interacting computations. This separation prevents the most common failure mode in integrated guidance systems: a controller that interprets "J_ab is still high" as justification for maintaining the current trajectory, missing the fact that the current trajectory is no longer optimal even within air-breathing mode.

The zoom-climb decision, in particular, must be made independently of the mode comparison. The zoom may be warranted even when J_ab still substantially exceeds J_rk — specifically, when J_ab is declining and apoapsis is still below the target for rocket takeover. The correct action is to enter zoom phase while still in air-breathing mode, using the remaining profitable portion of the air-breathing operation to shape the trajectory for a more efficient rocket phase.

If mode comparison and trajectory shaping are collapsed into a single scalar, this situation cannot be distinguished from "J_ab is high, continue corridor flight," and the zoom phase is entered late or missed entirely.

---

## 6. Layer 3 — Phase Logic

### 6.1 Transition Logic and Persistence

Every phase transition in the state machine requires three conditions to be satisfied simultaneously: the primary condition (such as J_rk > J_ab) must be true; the primary condition must have been true continuously for a minimum persistence window (nominally 2–4 seconds); and the estimator validity must be at least DEGRADED (transitions are blocked in INVALID state).

The persistence window serves two functions. It prevents chattering — rapid oscillation between phases due to signal noise near a threshold boundary — and it creates a minimum commitment time that gives each phase the opportunity to produce observable effects.

The persistence window is state-dependent. When the value differential J_rk - J_ab is large and stable, a short persistence window (2 seconds) is appropriate. When the differential is near the margin threshold, a longer window (4–6 seconds) is required. This adaptive persistence is implemented as a linear interpolation between the minimum and maximum window lengths based on the magnitude of the differential relative to a confidence band.

Hysteresis is also applied: once a transition is committed, the reverse condition must exceed a larger threshold before a reverse transition is considered. Since the state machine is one-way for most transitions, hysteresis primarily applies within the zoom phase.

### 6.2 AB_CORRIDOR Phase

In AB_CORRIDOR, the guidance objective is to follow the optimal air-breathing trajectory while maximising the measured J_ab. The control output is a pitch bias relative to the surface prograde vector, adjusted to keep the vehicle within the dynamic pressure corridor.

The pitch bias is the weighted sum of several error terms:

- **Dynamic pressure error:** deviation from the target *q*, converted to a pitch correction via a proportional controller. The gain should be sized so that a 10% q deviation produces a modest pitch correction (e.g., 2–3 degrees).
- **Apoapsis growth rate error:** if apoapsis is not growing at an acceptable rate, add a positive pitch bias to steepen the climb, preventing the vehicle from flying too flat in pursuit of speed.
- **Heating margin:** if the thermal proxy is approaching the limit, add a small positive pitch bias to reduce the heating rate.

The total pitch correction is bounded to prevent the vehicle from exceeding its maximum angle of attack. The FAR AoA reading provides direct feedback for this limit.

Throttle should be held at maximum throughout the corridor phase. Fuel conservation in this phase is better achieved through trajectory shaping than throttle reduction.

### 6.3 AB_ZOOM Phase

Entry to the zoom phase is triggered when: J_ab is declining (negative derivative over the valuation smoothing window), current dynamic pressure is at or below the lower bound of the optimal corridor, apoapsis altitude is below the target for rocket takeover, and no hard limits have been breached.

The zoom objective is to convert the remaining useful portion of the air-breathing envelope into apoapsis altitude. The pitch target rises at a controlled rate — nominally 0.5 to 1.0 degrees per second — while AoA is monitored and climb rate is reduced if the AoA limit is approached.

The zoom phase has four exit conditions, all checked continuously:

- **J_rk > J_ab for the persistence window:** the primary mode switch condition has been met.
- **Air-breathing thrust has collapsed:** the engines are no longer producing meaningful thrust (intake starvation, flameout, or regime boundary — see Section 8.1 for the distinction).
- **AoA or controllability limit reached:** the vehicle cannot maintain the commanded attitude without violating stability constraints.
- **Apoapsis target reached:** the apoapsis has risen sufficiently that the rocket phase can execute cleanly.

On exit from zoom, the mode switch is committed regardless of which exit condition triggered. There is no reversion to AB_CORRIDOR from the zoom phase.

### 6.4 ROCKET_SUSTAIN Phase

The ROCKET_SUSTAIN phase begins at mode switch. The rocket engines are ignited, the air-breathing engines are shut down, and guidance transitions to orbital error-driven control.

#### 6.4.1 The is_spooling Flag

The transition between air-breathing and rocket modes is not instantaneous. Air-breathing engines have a spool-down period during which they continue producing diminishing thrust and consuming liquid fuel for several seconds after deactivation. During this window, the vehicle is in a mixed-propulsion state: both ṁ_ab and ṁ_rk are non-zero simultaneously.

This mixed-propulsion window invalidates the J comparison. Running J_ab vs. J_rk during spool-down produces a meaningless signal that could trigger spurious transitions. Simultaneously, the shifting thrust centre and changing total thrust magnitude during spool-down make the pitch controller dynamically incorrect if it is operating on assumptions about the propulsion state that no longer hold.

The `is_spooling` flag addresses both problems. It is set True when ROCKET_SUSTAIN is entered and remains True until the air-breathing thrust ratio falls below a threshold (nominally < 5% of rated thrust). While `is_spooling` is True:

- The J comparison is **suspended entirely**. No mode transitions can be triggered from a value comparison during spool-down.
- The pitch controller's **authority is softened** — gain scaling reduces the commanded pitch rate to prevent PID overreaction to the shifting thrust centre.
- The guidance objective reverts to a simple prograde hold until the spool-down completes and the controller can re-establish confidence in its force estimates.

Once `is_spooling` clears, normal ROCKET_SUSTAIN guidance resumes with a fresh integrator state.

#### 6.4.2 Normal ROCKET_SUSTAIN Guidance

The primary control objective is to raise apoapsis to the target altitude. The pitch command is derived from apoapsis error and time-to-apoapsis:

- If apoapsis is well below target and time-to-apoapsis is long, pitch up modestly to raise apoapsis more aggressively.
- If apoapsis is approaching target and time-to-apoapsis is short, shallow the pitch to begin accumulating horizontal velocity.
- If apoapsis overshoots, shallow immediately and accept the extra altitude as margin.

The key principle is that flight path angle is an output of this control system, not an input. The controller does not target γ = 0; it targets apoapsis altitude and time-to-apoapsis, and the resulting flight path angle is whatever those objectives require.

Throttle should be at maximum during ROCKET_SUSTAIN unless thermal or structural limits require reduction.

### 6.5 ROCKET_CLOSEOUT Phase

Entry to ROCKET_CLOSEOUT occurs when apoapsis altitude is within the target band. The primary objective shifts from raising apoapsis to accumulating horizontal orbital velocity while keeping apoapsis within the target band.

The pitch command targets the surface horizon or a small negative flight path angle, subject to the constraint that apoapsis remains above the lower bound of the target band. The phase exits to CIRCULARISE when time-to-apoapsis falls below a threshold and orbital velocity is within range of the available circularisation delta-V:

```
ΔV_circ = √(μ / r_ap) - v_ap
```

### 6.6 CIRCULARISE Phase

The vehicle coasts to apoapsis, then executes a prograde burn to raise periapsis to match apoapsis. This phase does not require the force-projection architecture of the earlier phases — it is a standard impulsive burn at a point, well-documented in the kOS community.

If apoapsis is below approximately 70 km for Kerbin on arrival, the periapsis target should be set above the atmosphere to produce a stable orbit for a secondary circularisation burn, rather than attempting to circularise into a decaying trajectory.

---

## 7. Layer 4 — Guidance Output

### 7.1 Control Variables

The guidance output layer translates phase objectives into vehicle commands. The control variables available in kOS are: pitch (via `LOCK STEERING`), throttle (via `LOCK THROTTLE`), and engine mode activation (via engine part modules). The guidance output layer is the only layer permitted to write to these variables.

### 7.2 Steering Reference

In all air-breathing phases (AB_CORRIDOR and AB_ZOOM), the steering reference is the surface velocity vector (surface prograde) plus a pitch bias. Using surface prograde as the base ensures that the vehicle's angle of attack relative to the airstream is controlled, which is the relevant quantity for aerodynamic force management.

In all rocket phases, the steering reference transitions to the orbital velocity vector (orbital prograde) plus a pitch bias. The transition at mode switch should be smooth — a sudden jump in the steering reference can cause a transient that corrupts short-window value estimates and triggers false validity degradation. The steering reference should be linearly interpolated between surface and orbital prograde over a 3–5 second window at the mode switch point.

### 7.3 Aerodynamic Gain Scheduling

Dynamic pressure during a spaceplane ascent varies by more than an order of magnitude — from approximately 5 kPa in the high-altitude corridor to 60+ kPa near the structural limit. A PID pitch controller tuned for q = 40 kPa will be sluggish at q = 5 kPa and potentially unstable at q = 60 kPa. Fixed-gain PID is therefore not appropriate for this application.

The guidance output layer must implement **aerodynamic gain scheduling**: the proportional and derivative gains of all pitch controllers are scaled inversely with dynamic pressure (or equivalently, with Mach number as a proxy):

```
K_p_effective = K_p_base / (q / q_ref)
K_d_effective = K_d_base / (q / q_ref)
```

where q_ref is the reference dynamic pressure at which the base gains were tuned (typically the target corridor centre q), and K_p_base and K_d_base are the tuned nominal gains. The integral gain is typically held constant or scheduled separately, as integral action responds to accumulated error rather than instantaneous dynamics.

In practice, gain scheduling should be validated by logging the effective gains and pitch response at several points across the ascent envelope — at least one measurement in the low-q climb, one in the corridor, and one in the high-q region just below the structural limit.

### 7.4 Anti-Windup and Derivative-on-Measurement

Any PID-style controller used for pitch targeting must address two implementation hazards that are particularly acute at phase transitions.

**Integrator anti-windup:** During the zoom phase, the pitch integrator accumulates a large positive value while the vehicle is pitched up. If unclamped, the transition to rocket mode produces an initial pitch overshoot as the integrator unwinds. The recommended approach is to clamp the integrator output to the physical limits of the control variable before it is applied, and to perform a controlled integrator reset at each phase transition.

**Derivative-on-measurement, not on error:** When the state machine switches phases, the setpoint changes instantaneously. If the derivative term is computed on the error signal (setpoint minus measurement), a step-change in the setpoint produces a derivative spike — a "derivative kick" — that commands a violent maximum-deflection pitch manoeuvre at exactly the moment of phase transition. This is prevented by computing the derivative term on the **measurement alone** (negative of the measurement derivative), not on the error:

```
D_term = -K_d · d(measurement)/dt
```

This produces zero derivative contribution at the instant of a setpoint step, since the measurement has not yet changed. The integrator then winds up gradually to close the new error, giving smooth transient behaviour.

Both anti-windup and derivative-on-measurement must be implemented in every pitch control loop in the system. Neither is optional.

### 7.5 Engine Mode Management

Switching between air-breathing and rocket modes requires care beyond simply activating and deactivating engines:

- **Air-breathing engine spool-down:** some air-breathing engine designs have non-instantaneous shutdown, producing residual thrust for several seconds. The `is_spooling` flag in Section 6.4.1 is the primary mechanism for managing this window.
- **Rocket engine startup transient:** rocket engines typically produce below-rated thrust during ignition. The mode switch timing should ensure rocket engines reach rated thrust before air-breathing thrust falls significantly, maintaining total thrust continuity.
- **Intake management:** intake area should be closed or reduced at mode switch to reduce drag. This is a vehicle-specific action that should be encapsulated in a transition routine called by the state machine.

---

## 8. Edge Cases and Failure Modes

### 8.1 Engine Regime Boundary vs. True Flameout

A distinction that the original framework did not make explicit is the difference between an **engine thermal regime boundary** and a **true flameout**. These are different physical events with different appropriate responses, and conflating them produces incorrect controller behaviour.

A **thermal regime boundary** is a predictable, design-specified point where the air-breathing engine's thrust drops precipitously because the operating envelope has been reached — for RAPIER-type engines in KSP, this typically occurs near Mach 4.5–5.0 where thermal flux peaks and thrust falls sharply. This is not a failure; it is the designed edge of the operating envelope. For vehicles where this boundary is known and repeatable, it should be treated as a **scheduled zoom-entry criterion**: the controller should begin monitoring J_ab decline and zoom readiness as the vehicle approaches this regime, rather than reacting reactively when thrust has already collapsed. The transition is planned, not emergency.

A **true flameout** is an unplanned thrust loss caused by intake starvation at altitude, operating outside the Mach envelope, or thermal damage. It is detected by monitoring the ratio of actual thrust to commanded thrust — if this ratio falls below a threshold (e.g., 0.5) for more than one update cycle and the vehicle is not near a known regime boundary, a flameout is declared.

On flameout detection, the state machine transitions immediately to AB_ZOOM if apoapsis is below target, or directly to ROCKET_SUSTAIN if apoapsis is sufficient. The persistence window requirement is waived for flameout-triggered transitions — the condition is unambiguous and irreversible. This is one of the two exceptions to the "transitions require persistence" rule (the other being the fuel exhaustion floor in Section 8.6).

### 8.2 Estimator Invalidity During Critical Phase

If the estimator transitions to INVALID state during AB_CORRIDOR — caused by severe attitude oscillation or unexpected aero behaviour — the guidance system cannot trust its value comparison. The appropriate response is to freeze the current mode, reduce pitch aggressiveness to stabilise the vehicle, and wait for the estimator to recover to DEGRADED or VALID before resuming.

If the estimator is INVALID at the time of a scheduled mode switch (triggered by flameout or another external condition), the switch proceeds but J_rk is estimated from the last known valid values. A flag is set to indicate that the rocket phase is proceeding on degraded guidance, which may warrant an earlier-than-optimal circularisation burn to avoid accumulating trajectory error.

### 8.3 Apoapsis Overshoot During Zoom

If the zoom phase is too aggressive, the apoapsis may rise above target before the mode switch is committed. This is generally preferable to falling short, but it wastes propellant.

The zoom controller should monitor apoapsis altitude continuously and reduce the pitch target rate if apoapsis is rising faster than a threshold rate. If apoapsis reaches the target during the zoom phase before J_ab has fallen below J_rk, the zoom-exit condition "apoapsis target reached" triggers and the mode switch is committed early.

### 8.4 Low Thrust-to-Weight in Rocket Phase

Spaceplanes often have relatively low rocket TWR because most engine mass is in the air-breathing powerplant. If rocket TWR at mode switch is near 1.0, gravity losses will be severe.

The guidance system should estimate effective rocket TWR from queryable rocket thrust and current vehicle mass. If TWR is below a threshold (e.g., 1.3), the zoom phase should be used more aggressively to pre-establish a higher apoapsis before rocket takeover. The P_traj_rk penalty in J_rk partially accounts for this, but for very low TWR vehicles an explicit check before committing to mode switch is warranted.

### 8.5 Hypersonic Controllability Loss

At high Mach numbers (typically above Mach 6–8 depending on the vehicle), aerodynamic centre shift and reduced control surface effectiveness can compromise controllability. The P_ctrl penalty and AoA oscillation validity check are the primary guards.

A secondary guard is warranted: if pitch error exceeds a threshold for more than a few seconds, the guidance system should reduce its demanded pitch deviation from prograde, accepting a suboptimal trajectory in exchange for stable, measurable flight conditions. A vehicle flying stably at a suboptimal trajectory produces valid measurements; a vehicle oscillating at an optimal commanded attitude produces corrupted measurements and physically unrealised commands.

### 8.6 Fuel Exhaustion During Air-Breathing Phase

The guidance system should monitor fuel margin and apply an additional penalty to J_ab when fuel reserves fall below a threshold — not because the engine is less efficient, but because the rocket phase requires propellant.

A fuel margin check — estimated remaining rocket delta-V versus required circularisation delta-V plus margin — provides a hard floor on the mode switch decision independent of the J comparison. If remaining fuel cannot close the orbit from the current trajectory, the mode switch is committed immediately. This is the second exception to the "transitions require persistence" rule.

### 8.7 Reentry Abort

If the guidance system determines — from negative orbital energy, suborbital trajectory, or rapidly falling apoapsis — that orbit cannot be achieved with remaining propellant, an abort state should be entered. Rocket engines are fired at maximum thrust in the most energy-efficient direction available, the J comparison is suspended, and trajectory tracking targets the highest achievable apoapsis with remaining propellant.

---

## 9. Implementation Plan

### 9.1 Development Sequence

The implementation follows the layered dependency structure strictly. Building the guidance output layer before the state estimator is validated is a category error.

| Step | Task | Validation Method |
|------|------|-------------------|
| 1 | Implement frame transforms; log raw vs. transformed vectors | Visual inspection in known attitude (prograde flight) |
| 2 | Implement state estimator with dual smoothing (valuation + control) | Verify each signal category behaves per spec; variance and lag both meet targets |
| 3 | Validate Ė from force projection vs. dE/dt from orbital state | Should agree to within ~5% in stable flight; divergence indicates transform error |
| 4 | Implement J_ab and J_rk with propellant equivalency and drag projection | Log during manual flight; verify J_ab > J_rk in corridor, J_rk > J_ab in thin air |
| 5 | Implement estimator validity logic | Induce controllability stress; verify validity degrades correctly |
| 6 | Implement state machine and phase logic (no guidance output) | Log phase state during manual flight; verify transitions trigger at physically meaningful moments |
| 7 | Implement guidance output for AB_CORRIDOR with gain scheduling | Verify q tracking and apoapsis growth; verify gain scheduling reduces oscillation at q extremes |
| 8 | Implement AB_ZOOM phase | Verify zoom entry, smooth pitch rise, all exit conditions |
| 9 | Implement ROCKET_SUSTAIN with is_spooling logic | Verify clean transition, no pitch kick at switch, spool-down window handled correctly |
| 10 | Implement ROCKET_CLOSEOUT and CIRCULARISE | Verify apoapsis targeting, trajectory flattening, orbit closure |
| 11 | Validate derivative-on-measurement and anti-windup | Log pitch commands at each phase transition; verify no transient spikes |
| 12 | Edge case validation | Deliberately trigger each case in Section 8 and verify correct response |

### 9.2 The update_state() Specification

Before writing any kOS code, the `update_state()` function should be fully specified in a document that defines every input, every transform, every smoothed output (with its category — valuation or control), every derivative, every validity check, and every downstream function gated on validity status. The document must explicitly state:

- The canonical frame and all required transforms.
- For each smoothed quantity: its category (valuation or control), EMA α, rolling window length, and units.
- Which downstream functions consume the control copy versus the valuation copy of shared signals.
- The validity check thresholds and their consequences.
- Explicit SI units on every quantity.

Mismatch between assumed and actual units is as insidious as frame misalignment. Velocity in metres per second, not Kerbin units. Force in Newtons. Mass in kilograms. Energy in Joules per kilogram. Frame in orbital (inertial).

### 9.3 Logging and Diagnostics

The implementation should log at minimum:

- All raw sensor values, before smoothing, at the full update rate.
- Both smoothed copies (valuation and control) of all dual-tracked signals.
- J_ab, J_rk, w_prop, and the differential, every cycle.
- D_rk_est and D_current separately, to validate the drag projection logic.
- Estimator validity state and the specific check that last changed it.
- Current phase, time in current phase, and is_spooling flag.
- Pitch command, actual pitch, pitch error, and effective gain (to validate gain scheduling).
- Along-track thrust and aero components separately.
- Liquid fuel and oxidizer fractions separately.

These logs enable post-flight analysis without requiring anomalies to be reproducible in real time.

### 9.4 Per-Vehicle Parameters

The design goal is to minimise per-vehicle tuning. The following parameters are expected to require vehicle-specific values:

- **Maximum dynamic pressure (structural limit):** a physical property, deterministic from the vehicle's design.
- **Maximum angle of attack at peak Mach:** readable from FAR in test flights.
- **Thermal tolerance:** from design specification or progressive heating tests.
- **Target q for corridor centre:** typically 30–60 kPa for scramjet-capable designs.
- **Target apoapsis altitude for rocket takeover:** a mission parameter, not a vehicle parameter.
- **Engine thermal regime boundary Mach (for RAPIER-type engines):** the Mach number at which the engine approaches its thermal velocity limit, used to trigger proactive zoom readiness. This is a predictable engine characteristic, not a tuning parameter.
- **k_prop (propellant equivalency coefficient):** nominally 0.5; adjust for vehicles with extreme fuel-to-oxidizer tank ratio asymmetry.

No pitch schedules, no altitude thresholds, and no arbitrary transition scores appear in this list.

---

## 10. kOS Implementation Considerations

The preceding sections describe what the guidance system must do. This section describes kOS-specific behaviours, limitations, and failure modes that will determine whether it can be made to do those things reliably. Many of these are not obvious from reading the kOS documentation and only become apparent when a script that is correct on paper misbehaves in flight. They are collected here so that they can be designed around rather than discovered.

### 10.1 Execution Rate, Physics Ticks, and the IPU Setting

KSP's physics engine runs at a fixed 50 Hz — one physics tick every 0.02 seconds. kOS does not run once per physics tick. Instead, it executes a number of instructions per update equal to the **IPU (Instructions Per Update)** setting, which defaults to 200 and is configurable in the kOS settings file. When the script requires more than IPU instructions to complete a single iteration of its main loop, the loop begins to lag behind the physics engine. The script still runs, but its update rate drops below 50 Hz, and computed derivatives and EMA updates become incorrectly timed because the code assumes a fixed Δt that it is no longer achieving.

For a guidance system of the complexity described in this paper — with dual smoothing tracks, vector transforms, J computations, and state machine evaluation — 200 IPU will almost certainly be insufficient. The IPU setting should be raised to at least 2000, and ideally benchmarked empirically: log the actual wall-clock time between main loop iterations and verify it matches the intended update period. If it does not, either raise IPU further or identify and optimise the most expensive operations in the loop.

All timing-sensitive computations — EMA updates, rolling window derivatives, persistence window timers — must use `TIME:SECONDS` rather than assuming a fixed Δt. The correct pattern is:

```
set dt to TIME:SECONDS - last_time.
set last_time to TIME:SECONDS.
```

Using a hardcoded Δt constant is a silent correctness error that will cause smoothing and derivative computations to be wrong whenever the loop runs faster or slower than assumed.

### 10.2 The `wait 0` Requirement

Every main loop in a kOS script must include `wait 0` at the end of each iteration. Without it, kOS will not yield execution back to KSP's physics engine, and the game will freeze or crash. `wait 0` yields for one physics tick regardless of IPU. This is not optional and is distinct from `wait 0.1` (which yields for approximately 0.1 seconds of game time). For the guidance loop, `wait 0` is almost always correct — it gives the physics engine one tick to advance the simulation while the script prepares the next iteration.

A corollary: any blocking operation inside the main loop (file I/O, heavy part list iteration, or a nested computation that exceeds IPU) will delay the `wait 0` and effectively skip physics ticks from the guidance system's perspective. This is a performance problem that manifests as guidance lag and is distinct from the IPU problem — it occurs even with a high IPU setting if a single loop iteration contains genuinely expensive operations.

### 10.3 LOCK Statement Evaluation Frequency and Cost

`LOCK STEERING` and `LOCK THROTTLE` are evaluated by kOS every physics tick, regardless of the IPU setting or main loop update rate. This is beneficial for steering — the vehicle responds to guidance commands at full physics rate rather than at the (potentially slower) script loop rate. However, it has a cost: if the locked expression is computationally expensive, it runs every physics tick and consumes IPU budget outside the main loop.

The correct pattern for this guidance system is to compute the desired steering direction in the main loop and store it in a variable, then lock steering to that variable:

```
set desired_steering to <computed direction>.
lock steering to desired_steering.
```

This means the lock expression itself is trivially cheap (a variable lookup), while the computation happens at the main loop rate where it can be controlled. Never lock steering directly to a complex expression involving vector transforms and PID computations — it will execute that entire expression every physics tick at full cost.

### 10.4 Reference Frame Conversions

kOS exposes vectors in several different frames that are not interchangeable without explicit rotation. The most important for this system are:

- **`SHIP:VELOCITY:ORBIT`** — velocity in the inertial (orbital) frame. This is the canonical frame for the guidance system and requires no transformation.
- **`SHIP:VELOCITY:SURFACE`** — velocity relative to the rotating surface frame. Subtract the planet's rotation to recover the inertial velocity, or use ORBIT directly.
- **`ADDONS:FAR:AEROFORCE`** — aerodynamic force vector in the **ship body frame** (`SHIP:RAW`), where the positive X axis points right along the vessel, positive Y points up from the vessel, and positive Z points backward. This requires a rotation to the orbital frame before use.
- **`SHIP:FACING`** — returns a `Direction` type, not a `Vector`. Extract the rotation quaternion via `SHIP:FACING:FOREVECTOR`, `SHIP:FACING:TOPVECTOR`, and `SHIP:FACING:STARVECTOR` to build the rotation matrix needed to transform body-frame vectors to the orbital frame.

The rotation from ship body frame to orbital frame is:

```
// v_orbital = R * v_body
// where R has columns [SHIP:FACING:STARVECTOR, SHIP:FACING:TOPVECTOR, -SHIP:FACING:FOREVECTOR]
```

Note the sign on the forward vector: kOS's `FOREVECTOR` points in the direction the nose faces, which is the negative-Z direction in the standard body frame convention used by AEROFORCE. Getting this sign wrong produces a drag vector that points forward rather than aft — a silent error that makes the vehicle appear to be thrusting from aerodynamic forces.

This transform should be computed once per main loop iteration and stored. Every downstream vector operation in that cycle uses the stored rotation. Recomputing it inside individual functions is both wasteful and a risk for introducing subtle inconsistencies if the vessel's attitude changes between calls within the same loop iteration.

### 10.5 Orbital State Edge Cases

Several kOS orbital state suffixes return undefined, negative, or misleading values in specific conditions that are reachable during a spaceplane ascent:

- **`SHIP:ORBIT:APOAPSIS`** returns a negative value when the vehicle is on a suborbital trajectory with periapsis below the surface. The guidance system must guard against negative apoapsis values — a negative apoapsis is not a small positive apoapsis, and passing it directly into apoapsis error computations will produce inverted control outputs.

- **`ETA:APOAPSIS`** behaves unexpectedly when the vehicle is nearly circular (apoapsis and periapsis close together) or when the orbit is hyperbolic. It can return very large values or values that jump discontinuously. The state machine should validate that `ETA:APOAPSIS` is within a physically plausible range before using it to compute time-to-apoapsis errors.

- **`SHIP:ORBIT:APOAPSIS`** and **`ETA:APOAPSIS`** are both computed from the two-body Keplerian orbit, which ignores atmospheric drag. At low altitudes with significant drag, the actual apoapsis will be lower than the Keplerian prediction. This discrepancy grows with atmospheric density and vehicle drag coefficient. The guidance system should treat these values as upper-bound estimates in the lower corridor and account for the discrepancy when setting the apoapsis target for rocket takeover — the actual apoapsis at mode switch will be somewhat lower than the Keplerian value predicted during the zoom phase.

- **`SHIP:MASS`** is updated by kOS once per physics tick. It accurately reflects propellant consumption but lags behind instantaneous mass flow by up to one tick. For J computations this is acceptable; for time-critical staging logic it should be noted.

### 10.6 Engine Identification and Part Tagging

The guidance system must separately aggregate air-breathing engines and rocket engines to compute ṁ_ab, ṁ_rk, T_ab, and T_rk. Iterating `SHIP:ENGINES` every cycle and filtering by engine type is both expensive (part list iteration traverses the full vessel hierarchy) and fragile (it depends on engine naming conventions that differ across mods and vehicles).

The correct approach is **part tagging**: assign kOS tags to all air-breathing engines and all rocket engines during the initialisation phase of the script, before the guidance loop begins. kOS tags are persistent part properties that survive between script runs. The initialisation routine should:

1. Iterate `SHIP:ENGINES` once at startup.
2. Classify each engine by inspecting its `ENGINE:ISPAT(0)` (sea-level Isp) versus `ENGINE:ISPAT(1)` (vacuum Isp) ratio — or, more reliably for RAPIER-type engines, by reading the engine module name via `PART:GETMODULE`.
3. Store the result as two lists: `ab_engines` and `rk_engines`.
4. Reference only these cached lists during the guidance loop.

For RAPIER-type engines specifically, mode switching is performed via the engine's part module:

```
engine:GETMODULE("ModuleEnginesAJEJet"):DOACTION("Toggle Mode", true).
```

The exact module name depends on the engine mod being used. This should be validated in the initialisation routine — if the expected module is absent, the initialisation should fail loudly rather than silently operating with the wrong mode assumption.

Never use engine mode string comparisons (e.g., comparing `engine:MODE` to `"Air Breathing"`) as the primary switch logic. Engine mode strings are localisation-dependent and can differ between KSP versions and mods. Use module availability and thrust ratio as the authoritative mode signal instead.

### 10.7 Division by Zero and Guard Conditions

Several computations in the guidance system involve division by quantities that can be zero or near-zero in legitimate flight conditions:

- **Mass flow denominators in J_ab and J_rk:** At engine startup and shutdown, mass flow passes through zero. Division by a near-zero ṁ produces astronomically large J values that will immediately trigger mode transitions. All J computations must guard against this: if ṁ_ab or ṁ_rk is below a minimum threshold (e.g., 0.01 kg/s), the corresponding J value should be set to zero rather than computed.

- **Velocity magnitude in force projection:** At very low speeds (near the beginning of the guidance engagement), `|v_orb|` can be small enough that numerical errors in the unit vector computation produce erratic projections. The force projection routine should return zero if `|v_orb|` is below a minimum threshold (e.g., 50 m/s).

- **`ETA:APOAPSIS` in time-to-apoapsis error terms:** If ETA:APOAPSIS is zero (vehicle is exactly at apoapsis) or negative (returning from above apoapsis), the error term inverts. Guard against ETA:APOAPSIS ≤ 0 before computing apoapsis-proximity terms.

- **Dynamic pressure in gain scheduling:** The gain scheduling formula divides by q. At the very start of the guidance engagement where q is near zero, this produces infinite gains. The gain scheduler must clamp q to a minimum value (e.g., q_ref / 10) to prevent gain runaway at low dynamic pressure.

None of these are exotic edge cases — all of them will be encountered during routine ascent. Each division by a potentially-zero quantity in the implementation should have an explicit guard.

### 10.8 SAS Interaction with LOCK STEERING

kOS's `LOCK STEERING` takes control of the vessel's attitude. KSP's built-in SAS also controls attitude. When both are active simultaneously, they fight each other, producing oscillation that will be misread by the AoA validity check as a controllability problem, triggering DEGRADED or INVALID estimator state. The result is a guidance system that perpetually thinks it has a controllability problem that it has actually caused itself.

The script must explicitly disable SAS immediately before engaging `LOCK STEERING`, and must not re-enable it while locked steering is active. The initialisation sequence should include:

```
SAS off.
lock steering to desired_steering.
```

When the script exits — either normally at orbit insertion or via an abort path — it should unlock steering and restore SAS to a safe state. Unexpected script termination (runtime error, out-of-fuel abort, manual intervention) that leaves steering locked and SAS disabled will leave the vehicle uncontrolled. A `ON ABORT` handler should unlock steering as its first action.

### 10.9 kOS PIDLoop and the Steering Manager

kOS provides a built-in `PIDLoop` type and a built-in steering manager that implements its own internal PID for translating the locked steering direction into control surface deflections. This internal steering manager has its own gains and is separate from any PID controllers the script implements for pitch targeting.

Two consequences follow. First, the script's pitch bias commands are inputs to the steering manager's own control loop — the actual control surface deflections are determined by the steering manager, not directly by the script. This means the effective pitch response is the composition of the script's outer PID and the steering manager's inner PID. The gains of the outer loop (the script's pitch controller) must be tuned with the steering manager's dynamics in mind, not as if the script had direct authority over control surfaces.

Second, kOS exposes `STEERINGMANAGER:PITCHPID`, `STEERINGMANAGER:YAWPID`, and `STEERINGMANAGER:ROLLPID` as accessible PIDLoop objects whose gains can be set from the script. For the gain scheduling requirement in Section 7.3, it is possible to directly adjust the steering manager's internal gains rather than (or in addition to) adjusting the outer pitch bias loop. This gives finer control over the effective system response but also means that gain scheduling logic must consider both loops and their interaction.

The steering manager also has a `MAXSTOPPINGTIME` setting that limits how aggressively it will manoeuvre to reach the commanded direction. For a spaceplane at high dynamic pressure, the default value may be too aggressive and cause overshoots; at low dynamic pressure, it may be too conservative and cause sluggish response. This parameter should be included in gain scheduling alongside the PID gains.

### 10.10 WHEN...THEN Triggers for Interrupt-Style Events

The guidance system has two events that require immediate response regardless of where the main loop is in its cycle: engine flameout (Section 8.1) and the fuel exhaustion floor (Section 8.6). Polling these conditions inside the main loop means they are checked once per loop iteration, which at 10 Hz may be 100 ms late. For a vehicle at high dynamic pressure, 100 ms of undetected flameout can produce meaningful trajectory deviation.

kOS's `WHEN...THEN` construct provides interrupt-style triggers that are evaluated every physics tick independently of the main loop:

```
when (thrust_ratio < 0.5) then {
    set flameout_detected to true.
    preserve. // keep the trigger active for re-evaluation
}
```

Flameout detection and fuel exhaustion monitoring should be implemented as `WHEN...THEN` triggers rather than main-loop polling. The trigger sets a flag; the main loop reads the flag and executes the state transition. This separates the detection timing (physics tick rate) from the response logic (main loop rate) without requiring the transition logic itself to run inside the trigger body, which should be kept minimal.

Note that `WHEN...THEN` triggers share the IPU budget with the main loop. Heavy computation inside a trigger body will steal IPU from the main loop. Triggers should set flags only and return immediately.

### 10.11 File I/O Performance and Log Buffering

kOS's `LOG` command writes to disk synchronously. In the logging strategy described in Section 9.3, logging all signals at the full update rate means a disk write every main loop iteration. On systems where disk I/O is slow relative to the physics tick rate, this can cause the main loop to stall waiting for the write to complete, effectively reducing the guidance update rate.

The recommended approach is **buffered logging**: accumulate log entries in a kOS list within the main loop, and flush the buffer to disk every N iterations (e.g., every 10 iterations, corresponding to roughly 1-second log intervals on a 10 Hz loop). The raw sensor data in the buffer is still captured at full rate; only the disk write is deferred. This decouples the guidance update rate from disk I/O performance.

During initial validation (Step 1–3 in the development sequence), full-rate synchronous logging may be acceptable because the loop is intentionally simple. As the full system is assembled in Steps 4–12, buffered logging should replace synchronous writes.

### 10.12 Suborbital Trajectory Handling During Guidance Engagement

When the guidance system first engages — at the beginning of the air-breathing corridor — the vehicle is on a suborbital trajectory with negative specific orbital energy. Several quantities that the guidance system computes will be in unusual states:

- `SHIP:ORBIT:APOAPSIS` may be below the current altitude or negative.
- `ETA:APOAPSIS` may be very short or behave erratically.
- The specific orbital energy E will be negative.
- J_rk estimated from available rocket thrust may be undefined if rocket engines have not been primed.

The estimator validity system (Section 4.5) will likely report DEGRADED at engagement if these edge values trigger plausibility checks. This is correct behaviour — the system should not attempt mode comparisons until the vehicle is in a reasonable state. However, it means the AB_CORRIDOR phase must be capable of operating under DEGRADED validity without attempting mode transitions, relying instead on simpler corridor-following objectives (q targeting, apoapsis growth) until the estimator recovers to VALID.

The initialisation sequence should explicitly initialise all smoothing buffers, rolling windows, and EMA states with physically plausible starting values rather than zero. A zero-initialised EMA for orbital energy will produce a large artificial Ė signal on the first few iterations as the filter converges from zero toward the actual value. Pre-seeding the EMA with the first measured value eliminates this startup transient.

---

## 11. Conclusion

The guidance system described in this paper represents a substantive departure from conventional KSP autopilot design. Its central innovation is the replacement of scheduled heuristics — pitch tables, altitude triggers, dynamic pressure thresholds — with a measurement-driven mode comparison grounded in specific orbital energy and normalised by propellant consumption.

This revised edition has strengthened the framework in four areas. First, the propellant equivalency correction ensures that the mode comparison reflects the true mission cost of each propellant type, not just its mass. Second, the separation of valuation-signal and control-signal smoothing makes explicit a distinction that was implicit before — these two categories have different noise tolerance requirements and must be tracked separately throughout the system. Third, the dedicated heavy low-pass filter on the drag term inside J_rk insulates the mode comparison from FAR's transonic noise, which would otherwise cause premature mode-switch triggers. Fourth, the is_spooling flag and the thermal regime boundary distinction convert two previously implicit and fragile assumptions into explicit, tested mechanisms.

The system's operational correctness continues to rest on the same four implementation properties identified in the original edition:

- Clean force projection in a consistent frame, enforced as a hard invariant.
- Stable state estimation with explicit separation of valuation and control smoothing.
- Hard structural separation between mode selection and trajectory shaping.
- Conservative switching logic with adaptive persistence, hysteresis, and the is_spooling guard.

If these properties hold, the system will perform robustly across a wide range of vehicle designs with minimal per-vehicle tuning. The tuning set consists entirely of physical vehicle properties — structural limits, thermal tolerance, engine regime boundaries — not controller parameters. A guidance system configured from physics, not from trial-and-error schedules, is one that transfers across vehicles without re-tuning.

The most important principle remains unchanged from the first edition: the primary signal is measured energy gain per propellant along the velocity vector, and everything else is scaffolding. The revisions in this edition have made that scaffolding sturdier. The foundation is the same.
