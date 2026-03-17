# IFC Aircraft Flight Testing Guide

A structured test sequence to determine all parameters required for an IFC aircraft configuration file (`*_cfg.ks`). Tests are ordered from ground to air and back, matching the natural flow of a flight.

---

## Test Card Overview

| # | Test | Parameters Determined |
|---|------|-----------------------|
| 1 | Engine Spool & Pre-Takeoff | `takeoff_throttle`, spool timing |
| 2 | Takeoff Ground Roll | `v_r`, takeoff yaw/steering gains |
| 3 | Rotation & Liftoff | `takeoff_pitch_tgt`, `v2`, `takeoff_climb_fpa` |
| 4 | Climb-Out & Flap Retraction | `vfe_*`, flap detent distances |
| 5 | Stall — Clean Configuration | Reference Vs (no flaps) |
| 6 | Stall — Landing Configuration | `vs0`, `a_crit` |
| 7 | Maneuvering Limits | `aa_max_aoa`, `aa_max_g`, `aa_max_bank`, `aa_max_sideslip`, `aa_max_side_g` |
| 8 | Approach Speed | `v_app`, `v_ref`, speed schedule shaping |
| 9 | Gear Extension | `gear_down_agl` |
| 10 | Flare | `flare_agl`, `flare_touchdown_vs`, flare tuning |
| 11 | Rollout | `rollout_brake_max_ias`, yaw/nose-hold tuning |

---

## Test 1 — Engine Spool & Pre-Takeoff Check

**Objective:** Verify engine spool behaviour and determine the brake-hold parameters before the takeoff roll begins.

### Procedure
1. Apply full parking brake. Set throttle to `takeoff_throttle` (start with `1.0`).
2. Observe thrust rising. Note how long it takes from throttle input to reach stable rated thrust.
3. Watch for any stall/engine start requiring a STAGE input (`takeoff_autostage`).

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `takeoff_throttle` | Throttle fraction at brake release | Use 1.0 unless engine overspeeds; reduce only if needed |
| `takeoff_engine_spool_timeout_s` | Max wait time (s) for engines to spool | Time from throttle input to declared "ready" |
| `takeoff_spool_thrust_frac` | Fraction of max thrust required before brake release | Observe at what thrust fraction engines stabilise |
| `takeoff_spool_steady_dknps` | Thrust rate-of-change (kN/s) threshold for "steady" | Measure the plateau rate from telemetry |
| `takeoff_spool_steady_hold_s` | Time thrust must remain steady before release | Count seconds thrust stays within threshold |
| `takeoff_flap_settle_s` | Wait time after final takeoff-flap step before release | Observe how long flaps take to reach detent |
| `takeoff_min_avail_thrust` | kN threshold considered "engines lit" | Note available thrust after spool but before full power |

### Notes
- If autostage is needed set `takeoff_autostage = 1` and `takeoff_stage_max_attempts` accordingly.
- For jets with long spool times, increase `takeoff_engine_spool_timeout_s`.

---

## Test 2 — Takeoff Ground Roll & Directional Control

**Objective:** Determine rotate speed (V_R) and tune centerline/yaw control for a straight, stable ground roll.

### Procedure
1. Release brakes at full power. Note IAS continuously.
2. Allow aircraft to accelerate without pilot input. Observe any heading drift.
3. On repeat runs, experiment with rudder/nosewheel inputs to hold centerline.
4. Note the IAS at which the nose naturally wants to lift, and the IAS at which the mains definitively leave the runway.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `v_r` | Rotate speed (m/s IAS) | IAS at which you apply back-pressure to rotate; aircraft should leave ground positively |
| `takeoff_flap detent` | Flap setting for takeoff | Use `flaps_detent_approach` or `flaps_detent_climb`; choose the detent that gives best lift/drag |
| `takeoff_loc_kp` | Centerline correction gain | Tune until heading correction is proportional and not oscillatory |
| `takeoff_loc_guard_m` | Cap on localizer error used for steering | Set to a realistic runway-width guard (~120–140 m) |
| `takeoff_steer_max_corr` | Max steering heading correction (deg) | Limit to prevent overcorrection at speed |
| `takeoff_steer_hdg_rate_kd` | Heading-rate damping for steering | Increase if nose weaves left-right during roll |
| `takeoff_yaw_kp` | Rudder gain per deg heading error | Tune: small value for docile aircraft, larger for nose-heavy |
| `takeoff_yaw_kd` | Rudder damping per deg/s heading rate | Increase if rudder oscillates |
| `takeoff_yaw_max_cmd` | Max rudder command magnitude | Clamp to prevent overcorrection |
| `takeoff_yaw_start_ias` | IAS where rudder assist begins (m/s) | Set to the speed where rudder becomes effective |
| `takeoff_yaw_full_ias` | IAS where rudder assist reaches full gain | Set to ~50–70% of V_R |
| `takeoff_yaw_min_scale` | Rudder authority floor at start of roll | Raise if aircraft swings wildly at low speed |
| `takeoff_yaw_sign` | +1 or −1 depending on axis convention | Verify rudder corrects in the right direction; invert if backwards |
| `takeoff_dir_max_corr` | Max heading correction in rotate/climb | Limit heading hunting after liftoff |

---

## Test 3 — Rotation, Liftoff & Initial Climb

**Objective:** Determine the pitch target at rotation, V2 climb speed, and climb FPA. These set how aggressively the aircraft climbs away after V_R.

### Procedure
1. At V_R, apply back-pressure (or let IFC rotate). Time and observe the pitch response.
2. Note the pitch angle at which the aircraft cleanly lifts off without tail-strike or buffet.
3. Establish a climb and allow IAS to stabilise. Adjust FPA until speed holds steady near target.
4. Note the AGL at which handling is clean enough to transition out of takeoff phase.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `v_r` | Rotate speed — confirmed from Test 2 | Cross-check: mains leave runway at or just above this IAS |
| `v2` | Initial climb speed target (m/s IAS) | Speed that gives a positive FPA without AoA warnings; typically 1.1–1.2 × Vs0 |
| `takeoff_pitch_tgt` | Pitch target commanded at rotation (deg) | Pitch angle that allows liftoff without over-rotation or tail-strike |
| `takeoff_climb_fpa` | Climb FPA for climb-out phase (deg) | Steepest climb angle that holds V2 without underspeed |
| `takeoff_pitch_slew_dps` | Rate at which pitch target is slewed (deg/s) | Slow = gentle rotation; increase for aircraft needing a snap rotation |
| `takeoff_rotate_pitch_kp` | Pitch command per deg error while on wheels | Tune so nose rises smoothly to `takeoff_pitch_tgt` |
| `takeoff_rotate_pitch_ff` | Baseline back-pressure during rotation | Add if aircraft is reluctant to rotate |
| `takeoff_rotate_pitch_max_cmd` | Max pitch command while on wheels | Clamp to prevent over-rotation |
| `takeoff_rotate_pitch_slew_per_s` | Pitch command slew rate on ground | Increase for snappy rotation; decrease for heavy aircraft |
| `takeoff_airborne_agl` | AGL threshold for airborne detection (m) | Height at which mains clearly leave ground; typically 2–5 m |
| `takeoff_airborne_min_vs` | Min vertical speed to confirm airborne (m/s) | Small positive value (0.5–1.0 m/s) to avoid false triggers |
| `takeoff_done_agl` | AGL to end takeoff phase (m) | Height at which the aircraft is established in clean climb; typically 300 m |
| `takeoff_climb_min_throttle` | Throttle floor during climb-out | Prevent sink-back if autothrottle reduces power too aggressively |
| `takeoff_climb_spd_thr_gain` | Throttle trim per m/s of (V2 − IAS) | Tune to maintain V2 on speed during climb-out |
| `takeoff_climb_fpa_spd_gain` | FPA reduction per m/s below V2 (speed protection) | Flatten pitch to regain energy if speed drops below V2 |
| `takeoff_climb_fpa_min` | Minimum climb FPA under speed protection (deg) | Floor to prevent level-off or descent during protection |
| `takeoff_aoa_protect_frac` | Fraction of `a_crit` to trigger AoA protection | Default 0.85; reduce if aircraft is sensitive near stall |
| `takeoff_aoa_fpa_gain` | FPA pull-down per deg AoA above threshold | How aggressively to unload pitch when nearing stall AoA |

---

## Test 4 — Climb & Flap Retraction Schedule

**Objective:** Determine the maximum speeds for each flap detent (Vfe), and the distances from the destination threshold at which each detent is scheduled during approach.

### Procedure
1. Climb to a safe altitude (>2,000 m AGL) with each flap detent selected in turn.
2. Accelerate slowly. Record the IAS at which buffet, structural strain, or FAR warnings appear for each detent. Set `vfe_*` values just below these limits with margin.
3. Retract flaps fully. Note cruise handling with flaps up.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `vfe_climb` | Max IAS for climb flap detent (m/s) | Maximum speed that FAR allows without flap damage/buffet at detent 1 |
| `vfe_approach` | Max IAS for approach flap detent (m/s) | Maximum speed at detent 2 |
| `vfe_landing` | Max IAS for landing flap detent (m/s) | Maximum speed at detent 3 (full) |
| `flaps_detent_up` | Detent index: fully retracted | Typically 0 |
| `flaps_detent_climb` | Detent index: climb / manoeuvre | Typically 1 |
| `flaps_detent_approach` | Detent index: takeoff / descent | Typically 2 |
| `flaps_detent_landing` | Detent index: full landing | Typically 3 |
| `flaps_max_detent` | Highest valid detent index | Match to number of FAR detents minus 1 |
| `flaps_climb_km` | Distance from threshold to allow climb detent (km) | Leave at default (45 km) unless aircraft needs earlier extension |
| `flaps_approach_km` | Distance to deploy approach flaps (km) | Typically 25–30 km |
| `flaps_landing_km` | Distance to deploy full landing flaps (km) | Typically 6–10 km |
| `ag_flaps_step_up` | Action group for FAR flap detent step-up | Verify with FAR UI; typically AG9 |
| `ag_flaps_step_down` | Action group for FAR flap detent step-down | Typically AG10 |

---

## Test 5 — Stall Speed, Clean Configuration

**Objective:** Establish the aircraft's clean (flaps up) stall speed as a reference. This is not a direct CFG parameter but informs `v_r`, `v2`, and `aa_max_aoa`.

### Procedure
1. Climb to >3,000 m AGL, level off, flaps up, gear up.
2. Reduce throttle to idle. Slowly decelerate at 1–2 m/s per second.
3. Note the IAS at first buffet, then at full stall break.
4. Record FAR's displayed AoA at stall.

### Values to Note (for reference)

| Value | Description |
|-------|-------------|
| Vs_clean | IAS at stall buffet onset (no flaps) |
| AoA at stall (clean) | Inform upper bound for `a_crit` |

---

## Test 6 — Stall Speed, Landing Configuration & Critical AoA

**Objective:** Determine stall speed in landing configuration (`vs0`) and FAR's critical AoA (`a_crit`). These feed the IFC's AoA protection system.

### Procedure
1. Climb to >3,000 m AGL. Deploy full landing flaps and gear.
2. Reduce throttle to idle. Decelerate at 1 m/s per second.
3. Note IAS at stall onset. This is `vs0`.
4. Throughout all stall tests, note the maximum AoA seen in FAR's overlay before the stall break. Set `a_crit` to that value.
5. Cross-check: IFC AoA protection triggers at `a_crit × APP_AOA_PROTECT_FRAC` (default 0.90).

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `vs0` | Stall speed, full landing config (m/s IAS) | IAS at clean stall break with gear and full flaps extended |
| `a_crit` | Critical AoA from FAR (deg) | FAR-displayed AoA at stall; 0 disables AoA protection |

### Notes
- Set `a_crit = 0` initially if you have not yet characterised the AoA at stall; re-run this test once FAR data is available.
- `Vapp` should be at least 1.3 × `vs0`; `v_ref` at least 1.23 × `vs0`.

---

## Test 7 — Maneuvering Limits (AA FBW Moderators)

**Objective:** Determine safe operational limits for the AtmosphereAutopilot fly-by-wire moderators. These cap the IFC's authority during all flight phases.

### Procedure
1. Climb to a safe altitude. Perform a series of pull-ups, rolls, and sideslip manoeuvres.
2. Observe FAR's structural stress gauge and the AoA indicator.
3. Establish limits below the structural limits with reasonable margin.
4. Bank: roll to progressively steeper angles in level turns. Note the bank at which speed/altitude bleed becomes unacceptable.

### Parameters to Record

| Parameter | Default | Description | How to Determine |
|-----------|---------|-------------|-----------------|
| `aa_max_aoa` | 12 deg | Max AoA allowed by FBW | Set below `a_crit` with margin; typically `a_crit × 0.75–0.85` |
| `aa_max_g` | 3.5 G | Max load factor | Structural limit from FAR; set conservatively below airframe redline |
| `aa_max_bank` | 35 deg | Max bank angle during approach | Bank angle that allows level turn without excessive sink; use −1 for default |
| `aa_max_sideslip` | 5 deg | Max sideslip angle | Test by applying rudder alone; limit where FAR shows significant yaw stress |
| `aa_max_side_g` | 1.5 G | Max lateral G | Limit lateral load during sideslip or rolling manoeuvres |

### Notes
- Leave all values at −1 to use global defaults unless the aircraft diverges from normal behaviour.
- Spaceplanes with high max-AoA designs (e.g., X10-F) may need `aa_max_aoa` raised to 18–22 deg.

---

## Test 8 — Approach Speeds

**Objective:** Determine `v_app` and `v_ref`, and tune the approach-speed intercept schedule. These are the most critical parameters for a stabilised ILS approach.

### Procedure
1. Configure the aircraft for landing (flaps, gear up for initial approach).
2. Fly a series of level passes at various IAS values with full landing flaps and gear down.
3. Note the IAS that gives a manageable AoA (ideally 5–12 deg) and positive control.
4. Set `v_ref` = 1.3 × `vs0` as a starting point; adjust upward if control is marginal.
5. Set `v_app` = `v_ref` + 10–15 m/s for a normal approach with energy margin.
6. Fly a full ILS approach with these values and observe speed stability from FAF to flare.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `v_ref` | Threshold crossing / short-final speed (m/s IAS) | 1.3 × `vs0`; adjust upward for heavy aircraft or gusty conditions |
| `v_app` | Approach target speed from FAF to flare (m/s IAS) | `v_ref` + 10–15 m/s; trim until AoA is in comfortable range on final |
| `app_spd_intercept_gain` | Multiplier applied to (Vapp − Vref) for intercept speed addition | Default 0.60; raise if aircraft captures the intercept speed slowly |
| `app_spd_intercept_min_add` | Minimum speed addition above Vapp during intercept (m/s) | Typically 4–6 m/s; sets floor above Vapp before capture |
| `app_spd_intercept_max_add` | Maximum speed addition during intercept (m/s) | Typically 8–12 m/s; prevents excessive speed before capture |
| `app_short_final_agl` | AGL (m) where Vapp begins blending toward Vref | Height at which you want to start decelerating to threshold speed |
| `app_speed_tgt_slew_per_s` | Speed target slew rate (m/s/s) | Rate of speed target change; lower = smoother but slower response |
| `app_short_final_cap` | Cap speed to short-final schedule even before LOC/GS capture | Set 1 for aircraft that need aggressive deceleration on short final |
| `ag_spoilers_arm` | AG to arm in-flight spoilers on approach | Test spoiler effect: use AG number or 0 if none |
| `app_spoiler_arm_km` | Distance from threshold to arm spoilers (km) | Arm early enough to stabilise; 0 = disabled |

### Notes
- On Kerbin, IAS and airspeed readings match at sea level. Verify approach AoA is below `a_crit × 0.9` at `v_ref`.
- For high-speed spaceplanes (X10-class), Vapp may be 120–150 m/s with correspondingly large Vapp − Vref deltas.

---

## Test 9 — Gear Extension Altitude

**Objective:** Confirm the gear-down altitude is high enough to give full extension before short final, but not so high as to add drag for the whole approach.

### Procedure
1. Fly an approach at `v_app`. Time gear extension from the moment of selection to full down-and-locked.
2. Choose `gear_down_agl` such that gear is confirmed down at least 500 m before the runway threshold (horizontally), accounting for descent rate.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `gear_down_agl` | AGL (m) at which IFC auto-extends gear | 200–500 m AGL is typical; set 0 for manual gear management |

---

## Test 10 — Flare Characterisation

**Objective:** Tune the flare manoeuvre from `flare_agl` through to touchdown. The goal is a smooth sink rate reduction ending in a controlled touchdown at `v_ref`.

### Procedure
1. Fly a full approach. Observe the AGL readout on the HUD.
2. Manually initiate the flare at various AGL heights to find the natural flare gate.
3. Aim for a touchdown sink rate of −0.2 to −0.5 m/s (firm but not hard). Adjust `flare_touchdown_vs`.
4. Check for balloon tendency (VS goes positive during flare). Tune `flare_balloon_vs_trigger` and `flare_balloon_fpa_push` to suppress.
5. Adjust `flare_pitch_rate_min/max` to match how quickly the aircraft responds to pitch input at approach speed.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `flare_agl` | AGL (m) to begin flare | Height at which smooth roundout can reach touchdown in the remaining runway slope; typically 20–130 m |
| `flare_touchdown_vs` | Target sink rate at wheel contact (m/s, negative) | Desired touchdown firmness; −0.05 to −0.3 m/s is gentle, −0.5 m/s is firm |
| `flare_ias_to_vs_gain` | Extra sink commanded per m/s above Vref during flare | Bleed energy during flare; increase if flare floats excessively |
| `flare_roundout_agl` | AGL (m) where final roundout blend begins | Height to start the final micro-level-off before touchdown |
| `flare_roundout_strength` | 0..1 blend strength in roundout zone | 1.0 = fully command touchdown VS; reduce if touchdown is abrupt |
| `flare_balloon_vs_trigger` | VS (m/s) above which anti-balloon push activates | Small positive value (0.05–0.15 m/s); triggers if aircraft floats upward |
| `flare_balloon_fpa_push` | Extra nose-down FPA applied in balloon recovery (deg) | 1.0–1.5 deg is typical; increase if ballooning is severe |
| `flare_pitch_rate_min` | Minimum FPA change rate (deg/s) at low IAS | Controls how slowly the flare arrests sink rate |
| `flare_pitch_rate_max` | Maximum FPA change rate (deg/s) at high IAS | Controls aggressiveness of flare at approach speed |
| `touchdown_confirm_s` | Debounce time (s) before FLARE → TOUCHDOWN commit | Increase if IFC prematurely declares touchdown on a rough runway |
| `touchdown_confirm_max_abs_vs` | Max |VS| (m/s) allowed when committing to touchdown | Prevents touchdown trigger during a float; typically 2–3 m/s |

### Notes
- `flare_agl` for large/fast aircraft should be much higher (100–130 m) to allow adequate pitch response time.
- Global defaults (`FLARE_AGL_M = 25 m`) are suitable for light aircraft. Override per-aircraft as needed.

---

## Test 11 — Rollout & Deceleration

**Objective:** Tune the post-touchdown ground roll: directional control, nose-hold, wheel braking, and spoiler/reverser deployment.

### Procedure
1. Land and allow the IFC to transition to rollout. Note at what IAS any tendency to veer left or right appears.
2. Observe nose attitude after mains contact. Note whether the nose drops abruptly (needs `rollout_nose_hold_cmd`) or pitches up (reduce command).
3. Test wheel braking: apply brakes gently above expected `rollout_brake_max_ias` and note any instability (shimmy, veer, tip-over tendency).
4. Confirm yaw assist corrects heading drift toward centreline. Tune gain/sign if overcorrection or wrong-direction inputs occur.

### Parameters to Record

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `rollout_brake_max_ias` | Max IAS (m/s) to allow wheel brakes | Highest IAS at which braking is stable; increase once landing gear is confirmed robust |
| `rollout_yaw_assist_ias` | IAS (m/s) below which rudder heading-hold begins | Typically from touchdown IAS down; set high for fast aircraft |
| `rollout_yaw_kp` | Rudder gain per deg heading error | Tune: small value (0.02–0.05) for gradual correction |
| `rollout_yaw_slew_per_s` | Max yaw command rate change per second | Limit abrupt rudder inputs; 1.0–2.5 is typical |
| `rollout_yaw_fade_ias` | IAS (m/s) below which yaw assist fades to zero | Match nosewheel steering takeover speed |
| `rollout_yaw_max_cmd` | Maximum rudder command magnitude | Clamp to prevent over-correction; 0.2–0.4 range |
| `rollout_yaw_sign` | +1 or −1 to flip rudder sign | Verify correction direction; invert if rudder fights heading |
| `rollout_steer_min_blend` | Minimum blend toward runway heading (0–1) | Higher value forces earlier alignment to runway; 0.08–0.15 typical |
| `rollout_nose_hold_cmd` | Aft-stick command to resist nose-drop after mains contact | Increase if nosewheel slams down; 0 = disabled |
| `rollout_nose_release_ias` | IAS (m/s) to begin releasing nose-hold | Below this speed the nose-hold gradually fades; typically 20–60 m/s |
| `rollout_nose_hold_min_s` | Minimum time to keep full nose-hold (s) | Keep through the initial touchdown transient |
| `rollout_nose_min_ref_deg` | Minimum pitch reference to hold after mains contact (deg) | Prevent immediate nose-down below this pitch |
| `rollout_nose_target_pitch_deg` | Final pitch target once nose lowering is allowed (deg) | Typically 0–1 deg; prevents digging nosewheel in |
| `rollout_nose_target_slew_dps` | Pitch target slew rate while lowering nose (deg/s) | Controls how slowly the pitch target is lowered |
| `rollout_pitch_hold_kp` | Gain: pitch command per deg error | Tune for stability; 0.08–0.12 is typical |
| `rollout_pitch_max_cmd` | Max closed-loop pitch command | Clamp to prevent abrupt nose oscillation |
| `rollout_pitch_max_down_cmd` | Max nose-down pitch command | Separate limit to protect nosewheel from slam |
| `rollout_pitch_slew_per_s` | Pitch command slew rate on rollout | Controls response speed; 1.2–2.0 typical |
| `rollout_touchdown_settle_s` | Time to hold TOUCHDOWN phase before entering ROLLOUT | Allow gear loads to settle; 0.3–0.6 s typical |
| `bounce_recovery_agl_m` | AGL (m) above which a re-airborne detection is a bounce | Typically 2–4 m; raise for large gear compression travel |
| `bounce_recovery_min_vs` | Min upward VS (m/s) to count as a real bounce | 0.6–1.0 m/s; prevents false trigger from gear oscillation |
| `bounce_recovery_confirm_s` | Debounce time for bounce detection (s) | Typically 0.3 s |
| `bounce_recovery_max_s` | Window after touchdown in which bounce recovery is active (s) | Typically 4–6 s |
| `ag_spoilers` | AG to deploy spoilers/airbrakes on touchdown | Verify spoilers deploy correctly; 0 = not equipped |
| `ag_thrust_rev` | AG for reverse thrust on touchdown | Verify reverse activates after mains contact; 0 = not equipped |
| `ag_drogue` | AG for drogue chute on touchdown | Set to AG number or 0 if no drogue |

---

## Quick-Reference: Parameter → CFG Key Mapping

| Flight characteristic | CFG key(s) |
|-----------------------|------------|
| Rotate speed | `v_r` |
| V2 climb speed | `v2` |
| Rotation pitch target | `takeoff_pitch_tgt` |
| Climb FPA | `takeoff_climb_fpa` |
| Stall speed, landing config | `vs0` |
| Critical AoA | `a_crit` |
| Approach speed | `v_app` |
| Threshold speed | `v_ref` |
| Flap speed limits | `vfe_climb`, `vfe_approach`, `vfe_landing` |
| Gear extension altitude | `gear_down_agl` |
| Flare trigger height | `flare_agl` |
| Target touchdown sink rate | `flare_touchdown_vs` |
| Brake speed limit | `rollout_brake_max_ias` |
| FBW AoA limit | `aa_max_aoa` |
| FBW G limit | `aa_max_g` |
| FBW bank limit | `aa_max_bank` |
| FBW sideslip limit | `aa_max_sideslip` |

---

## Typical Test Flight Profile

```
Takeoff roll ──► Rotation ──► Climb-out ──► Cruise altitude
                                               │
                                    Stall tests (clean + landing config)
                                    Maneuvering limits
                                               │
                              ◄── Descend to approach altitude
ILS approach ──► Short final ──► Flare ──► Touchdown ──► Rollout ──► Full stop
```

Run at least **three complete approaches** before finalising flare and rollout parameters. Log data using the IFC logger (`ifc_log_*.csv`) and review sink rate, IAS, and AoA traces to confirm values are consistent across runs.
