@LAZYGLOBAL OFF.

// ============================================================
// phase_ascent.ks  -  Integrated Flight Computer
// Spaceplane ascent-to-orbit guidance.
//
// Five sub-phases (one-way state machine):
//   AB_CORRIDOR  →  AB_ZOOM  →  RKT_SUSTAIN  →  RKT_CLOSEOUT  →  CIRCULARISE
//
// Depends on:
//   lib/ifc_ascent_state.ks   (ASC_STATE_INIT, ASC_STATE_UPDATE)
//   lib/ifc_constants.ks      (SUBPHASE_ASC_*, ASC_* constants)
//   lib/ifc_state.ks          (ASC_* globals, THROTTLE_CMD, etc.)
//   lib/ifc_helpers.ks        (MOVE_TOWARD, CLAMP, IFC_SET_ALERT)
//   lib/ifc_aa.ks             (AA_SET_DIRECTOR)
//
// Per-aircraft parameters consumed (all optional; constants are fallbacks):
//   ascent_q_target     Pa     corridor centre dynamic pressure
//   ascent_q_max        Pa     structural limit
//   ascent_q_min        Pa     corridor lower bound
//   ascent_heat_limit   Pa·m/s aerodynamic heating proxy limit
//   ascent_k_prop       –      propellant equivalency coefficient
//   ascent_aoa_limit    deg    maximum angle of attack
//   ascent_regime_mach  –      Mach at which AB regime boundary begins
//   ascent_zoom_target_m    m  target apoapsis before rocket mode switch
//   ascent_apoapsis_target_m m final orbit apoapsis
// ============================================================

// ── Flight-path-angle helpers ─────────────────────────────

// Surface (aerodynamic) flight path angle in degrees.
FUNCTION _ASC_SURF_FPA {
  LOCAL spd IS SHIP:AIRSPEED.
  IF spd < 1 { RETURN 0. }
  RETURN ARCSIN(CLAMP(SHIP:VERTICALSPEED / spd, -1, 1)).
}

// Orbital flight path angle in degrees.
FUNCTION _ASC_ORB_FPA {
  LOCAL v_orb IS SHIP:VELOCITY:ORBIT.
  IF v_orb:MAG < ASC_MIN_VORB { RETURN 0. }
  RETURN ARCSIN(CLAMP(VDOT(v_orb:NORMALIZED, SHIP:UP:VECTOR), -1, 1)).
}

// Compass bearing of the orbital prograde vector (deg, 0 = north).
FUNCTION _ASC_ORB_HDG {
  LOCAL v_orb IS SHIP:VELOCITY:ORBIT.
  LOCAL up_hat IS SHIP:UP:VECTOR.
  LOCAL horz IS VXCL(up_hat, v_orb).
  IF horz:MAG < 1 { RETURN SHIP:HEADING. }
  LOCAL nrth IS HEADING(0, 0):FOREVECTOR.
  LOCAL east IS HEADING(90, 0):FOREVECTOR.
  RETURN WRAP_360(ARCTAN2(VDOT(horz, east), VDOT(horz, nrth))).
}

// ── Dynamic-pressure gain scheduling ─────────────────────
//
// Returns a scale factor that amplifies pitch gains at low q and
// attenuates them at high q, keeping control authority proportional
// to aerodynamic responsiveness across the ascent envelope.
//
//   K_eff = K_base * (q_ref / q)   →   returned by this function
//
FUNCTION _ASC_Q_GAIN_SCALE {
  LOCAL q_dyn IS MAX(ASC_Q_CTRL, ASC_MIN_Q_GAIN).
  RETURN CLAMP(ASC_Q_REF / q_dyn, 0.3, 4.0).
}

// Ground-safe heading source.
// During ascent we hold a locked reference heading, captured at phase init.
FUNCTION _ASC_SURFACE_HDG {
  RETURN ASC_HDG_REF.
}

// Blend from locked ascent heading to orbital prograde heading.
FUNCTION _ASC_BLEND_HDG {
  PARAMETER blend.
  LOCAL b IS CLAMP(blend, 0, 1).
  LOCAL delta IS WRAP_180(_ASC_ORB_HDG() - ASC_HDG_REF).
  RETURN WRAP_360(ASC_HDG_REF + delta * b).
}

// Smooth surface-FPA reference before sending it to AA.
FUNCTION _ASC_SURF_FPA_SMOOTH {
  PARAMETER dt.
  LOCAL raw IS _ASC_SURF_FPA().
  LOCAL rate_dps IS 8.0.
  SET ASC_SURF_FPA_FILT TO MOVE_TOWARD(ASC_SURF_FPA_FILT, raw, rate_dps * MAX(dt, 0.01)).
  RETURN ASC_SURF_FPA_FILT.
}

// Rate-limit heading commands to avoid abrupt bank transients.
FUNCTION _ASC_HDG_SMOOTH {
  PARAMETER target_hdg, dt.
  LOCAL max_rate_dps IS 12.0.
  LOCAL err IS WRAP_180(target_hdg - ASC_HDG_CMD_FILT).
  LOCAL step_lim IS max_rate_dps * MAX(dt, 0.01).
  SET err TO CLAMP(err, -step_lim, step_lim).
  SET ASC_HDG_CMD_FILT TO WRAP_360(ASC_HDG_CMD_FILT + err).
  RETURN ASC_HDG_CMD_FILT.
}

FUNCTION _ASC_SET_THR_CMD {
  PARAMETER target, dt, slew_per_s.
  LOCAL slew IS MAX(slew_per_s, 0.05).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, CLAMP(target, 0, 1), slew * MAX(dt, 0.01)).
}

// Arms AB->rocket mode-switch logic only after the vehicle is clearly in ascent.
// Prevents premature commit while still on takeoff roll / very early climb.
FUNCTION _ASC_SWITCH_ARMED {
  IF ASC_AB_ENGINES:LENGTH = 0 { RETURN TRUE. }
  IF SHIP:AIRSPEED >= 120 { RETURN TRUE. }
  IF ALT:RADAR >= 1000 { RETURN TRUE. }
  IF SHIP:VERTICALSPEED >= 25 { RETURN TRUE. }
  RETURN FALSE.
}

// ── Mode switch persistence ───────────────────────────────
//
// Returns TRUE once J_rk > J_ab has been continuously satisfied
// for an adaptive persistence window.  The window is shorter when
// the differential is large and wider when the margin is thin.
// Validity state DEGRADED doubles the window; INVALID blocks entirely.
//
FUNCTION _ASC_CHECK_PERSIST {
  IF ASC_ESTIMATOR_VALIDITY = ASC_INVALID {
    SET ASC_PERSIST_UT TO -1.
    RETURN FALSE.
  }

  // Block planned transitions while q exceeds structural limit.
  // Emergency exits (flameout, fuel floor) bypass this function.
  IF ASC_Q_CTRL > ASC_Q_MAX_CACHED {
    SET ASC_PERSIST_UT TO -1.
    RETURN FALSE.
  }

  LOCAL diff IS ASC_J_RK - ASC_J_AB.
  IF diff <= 0 {
    SET ASC_PERSIST_UT TO -1.
    RETURN FALSE.
  }

  // Adaptive window: smaller when differential is large and confident.
  LOCAL j_ref IS MAX(ABS(ASC_J_AB), ABS(ASC_J_RK)).
  IF j_ref < 1 { SET j_ref TO 1. }
  LOCAL conf IS CLAMP(diff / (j_ref * ASC_PERSIST_CONF_BAND), 0, 1).
  LOCAL win  IS ASC_PERSIST_MAX_S - conf * (ASC_PERSIST_MAX_S - ASC_PERSIST_MIN_S).
  IF ASC_ESTIMATOR_VALIDITY = ASC_DEGRADED { SET win TO win * 2. }

  IF ASC_PERSIST_UT < 0 { SET ASC_PERSIST_UT TO TIME:SECONDS. }
  RETURN (TIME:SECONDS - ASC_PERSIST_UT) >= win.
}

// ── Engine management ─────────────────────────────────────

FUNCTION _ASC_IGNITE_ROCKETS {
  LOCAL i IS 0.
  UNTIL i >= ASC_RK_ENGINES:LENGTH {
    LOCAL eng IS ASC_RK_ENGINES[i].
    // For multimode engines (RAPIER-class), force secondary mode (rocket).
    IF eng:MULTIMODE {
      IF eng:HASSUFFIX("AUTOSWITCH") { SET eng:AUTOSWITCH TO FALSE. }
      IF eng:HASSUFFIX("PRIMARYMODE") AND eng:PRIMARYMODE {
        SET eng:PRIMARYMODE TO FALSE.
      }
    }
    IF NOT eng:IGNITION { eng:ACTIVATE(). }
    SET i TO i + 1.
  }
}

FUNCTION _ASC_ENGINE_IN_RK {
  PARAMETER eng.
  LOCAL k IS 0.
  UNTIL k >= ASC_RK_ENGINES:LENGTH {
    IF ASC_RK_ENGINES[k] = eng { RETURN TRUE. }
    SET k TO k + 1.
  }
  RETURN FALSE.
}

FUNCTION _ASC_SHUTDOWN_AB {
  // Iterate backward so we can safely remove dual-mode engines from AB tracking.
  LOCAL i IS ASC_AB_ENGINES:LENGTH - 1.
  UNTIL i < 0 {
    LOCAL eng IS ASC_AB_ENGINES[i].
    IF _ASC_ENGINE_IN_RK(eng) {
      // Dual-mode engine now belongs to rocket guidance; keep it running.
      ASC_AB_ENGINES:REMOVE(i).
    } ELSE {
      eng:SHUTDOWN().
    }
    SET i TO i - 1.
  }
}

// Commit the AB→rocket mode switch.  Called from CORRIDOR or ZOOM on any exit
// condition.  Pitch bias is intentionally NOT reset to avoid an FPA kick at
// the transition; ROCKET_SUSTAIN slews it toward its new target smoothly.
FUNCTION _ASC_COMMIT_ROCKET {
  PARAMETER reason.
  IFC_SET_ALERT("ASC: " + reason + "  →  RKT SUSTAIN").
  _ASC_IGNITE_ROCKETS().
  _ASC_SHUTDOWN_AB().
  SET ASC_IS_SPOOLING    TO TRUE.
  SET ASC_PERSIST_UT     TO -1.
  SET ASC_FLAMEOUT       TO FALSE.
  SET ASC_STEER_BLEND    TO 0.
  SET ASC_BLEND_START_UT TO TIME:SECONDS.
  SET_SUBPHASE(SUBPHASE_ASC_ROCKET_SUSTAIN).
}

// ── Flameout detection ────────────────────────────────────
//
// Sets ASC_FLAMEOUT = TRUE when AB thrust has unexpectedly collapsed
// below a fraction of available thrust and we are not near the known
// engine regime boundary (where the decline is expected and scheduled).
//
FUNCTION _ASC_CHECK_FLAMEOUT {
  IF ASC_AB_ENGINES:LENGTH = 0 { RETURN. }

  LOCAL regime_mach IS ASC_REGIME_MACH_CACHED.
  // Near the regime boundary: thrust decline is scheduled, not a failure.
  IF TELEM_ASC_MACH >= regime_mach * ASC_FLAMEOUT_MACH_GUARD_FRAC { RETURN. }

  LOCAL t_act   IS 0.
  LOCAL t_avail IS 0.
  LOCAL i IS 0.
  UNTIL i >= ASC_AB_ENGINES:LENGTH {
    LOCAL eng IS ASC_AB_ENGINES[i].
    IF eng:IGNITION {
      SET t_act   TO t_act   + eng:THRUST.
      SET t_avail TO t_avail + eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
    }
    SET i TO i + 1.
  }

  IF t_avail < 0.1 { RETURN. }
  IF (t_act / t_avail) < ASC_FLAMEOUT_THR_RATIO {
    SET ASC_FLAMEOUT TO TRUE.
  }
}

// ── AB_CORRIDOR sub-phase ─────────────────────────────────
//
// Control objective: maintain dynamic pressure corridor, build apoapsis.
//
// Pitch bias is a gain-scheduled sum of:
//   – q error term:         nose-up when q high, nose-down when q low
//   – apoapsis rate term:   extra nose-up if apo not growing fast enough
//
// Exit conditions (in priority order):
//   1. Flameout detected → ZOOM immediately (no persistence required)
//   2. J_rk > J_ab for persistence window AND apo ≥ zoom target → rocket
//   3. J_rk > J_ab for persistence window AND apo < zoom target  → ZOOM
//   4. q below lower corridor bound AND apo below zoom target    → ZOOM
//
FUNCTION _ASC_CORRIDOR {
  PARAMETER dt.

  LOCAL q_tgt      IS ASC_Q_TARGET_CACHED.
  LOCAL zoom_apo   IS ASC_ZOOM_APO_TARGET_CACHED.
  LOCAL aoa_lim    IS ASC_AOA_LIMIT_CACHED.
  LOCAL fpa_max    IS ASC_CORRIDOR_FPA_MAX_CACHED.
  LOCAL q_scale    IS _ASC_Q_GAIN_SCALE().

  // --- Pitch bias: q-corridor term ----------------------------------
  // For a spaceplane ascent corridor:
  //   q_err > 0 (q too high)  -> pitch up to climb out of dense air.
  //   q_err < 0 (q too low)   -> relax pitch to rebuild dynamic pressure.
  LOCAL q_err IS ASC_Q_CTRL - q_tgt.
  LOCAL q_bias IS q_err * ASC_Q_KP * q_scale.

  // --- Pitch bias: apoapsis growth rate term ------------------------
  LOCAL elapsed IS MAX(TIME:SECONDS - ASC_APO_PREV_UT, 0.5).
  LOCAL apo_rate IS (ASC_APO_VAL - ASC_APO_PREV) / elapsed.
  LOCAL apo_bias IS (ASC_APO_RATE_TGT - apo_rate) * ASC_APO_RATE_KP * q_scale.

  // --- Pitch bias: vertical-speed feedforward -----------------------
  // Anticipates altitude overshoot by adding a nose-down term proportional
  // to upward VS.  Activates when climbing; zero or negative VS = no effect.
  // This prevents the aircraft from climbing several km past the q-target
  // altitude before the reactive q-corridor term catches up.
  // Tapered linearly to zero by ASC_VS_FF_ALT_CUTOFF: useful at low altitude
  // where q-overshoot is a risk; at high altitude q falls naturally and
  // applying this feedforward drives the vehicle into a dive.
  LOCAL vs_alt_scale IS 1 - CLAMP(SHIP:ALTITUDE / ASC_VS_FF_ALT_CUTOFF, 0, 1).
  LOCAL vs_ff IS -CLAMP(SHIP:VERTICALSPEED, 0, 500) * ASC_VS_KP * vs_alt_scale.

  LOCAL bias_cmd IS CLAMP(q_bias + apo_bias + vs_ff, ASC_PITCH_BIAS_MIN, ASC_PITCH_BIAS_MAX).
  SET ASC_PITCH_BIAS TO MOVE_TOWARD(ASC_PITCH_BIAS, bias_cmd,
                                    ASC_PITCH_SLEW_DPS * dt).

  // --- Steering: surface prograde + pitch bias ----------------------
  LOCAL surf_fpa IS _ASC_SURF_FPA_SMOOTH(dt).
  LOCAL q_max IS ASC_Q_MAX_CACHED.
  LOCAL fpa_floor IS ASC_CORRIDOR_FPA_MIN.
  // Safety guard: never command a descent while still in the low-altitude
  // climb-out segment unless apoapsis is already well above the zoom target.
  IF ALT:RADAR < ASC_CORRIDOR_GUARD_AGL_M AND SHIP:ORBIT:APOAPSIS < zoom_apo * ASC_CORRIDOR_GUARD_APO_FRAC {
    SET fpa_floor TO 2.0.
  }
  LOCAL fpa_cmd  IS CLAMP(surf_fpa + ASC_PITCH_BIAS, fpa_floor, fpa_max).
  // AoA guard in corridor: stop adding pitch when approaching the limit.
  IF ASC_AOA_CTRL > aoa_lim {
    SET fpa_cmd TO MIN(fpa_cmd, surf_fpa - 1.0).
  } ELSE IF ASC_AOA_CTRL > aoa_lim * 0.85 {
    SET fpa_cmd TO MIN(fpa_cmd, surf_fpa + 0.5).
  }
  LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(_ASC_SURFACE_HDG(), dt).
  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  // q-relief throttle schedule: reduce thrust when corridor is exceeded.
  // This prevents rapid low-altitude q runaway and helps avoid dive-recovery
  // command saturation in AB corridor.
  LOCAL thr_cmd IS 1.0.
  IF ASC_Q_CTRL > q_tgt {
    LOCAL span IS MAX(q_max - q_tgt, q_tgt * 0.5).
    LOCAL over IS CLAMP((ASC_Q_CTRL - q_tgt) / span, 0, 1).
    SET thr_cmd TO 1.0 - over * 0.45.
  }
  IF ASC_Q_CTRL > q_max {
    SET thr_cmd TO MIN(thr_cmd, 0.35).
  }
  _ASC_SET_THR_CMD(CLAMP(thr_cmd, 0.35, 1.0), dt, 0.8).

  IF NOT _ASC_SWITCH_ARMED() {
    SET ASC_PERSIST_UT TO -1.
    SET ASC_FLAMEOUT TO FALSE.
    RETURN.
  }

  // --- Exit 1: flameout --------------------------------------------
  _ASC_CHECK_FLAMEOUT().
  IF ASC_FLAMEOUT {
    IFC_SET_ALERT("ASC: AB flameout in corridor  →  ZOOM").
    SET ASC_PERSIST_UT TO -1.
    SET_SUBPHASE(SUBPHASE_ASC_AB_ZOOM).
    RETURN.
  }

  // --- Exit 1b: fuel floor — not enough propellant to circularise --
  IF _ASC_CHECK_FUEL_FLOOR() {
    _ASC_COMMIT_ROCKET("fuel floor corridor").
    RETURN.
  }

  // --- Mach floor: planned mode transitions require approaching regime boundary ---
  // Emergency exits (flameout, fuel floor above) bypass this gate.
  // Also gates exit 4 (q-low corridor floor) since it falls below this RETURN.
  IF TELEM_ASC_MACH < ASC_REGIME_MACH_CACHED * ASC_SWITCH_MACH_FLOOR_FRAC {
    SET ASC_PERSIST_UT TO -1.
    RETURN.
  }

  // --- Exit 2/3: J_rk > J_ab persistence met ----------------------
  IF _ASC_CHECK_PERSIST() {
    SET ASC_PERSIST_UT TO -1.
    IF SHIP:ORBIT:APOAPSIS >= zoom_apo {
      // Apoapsis already high enough — go straight to rocket mode.
      _ASC_COMMIT_ROCKET("J_rk>J_ab corridor direct").
    } ELSE {
      // Zoom to raise apoapsis before committing.
      IFC_SET_ALERT("ASC: J_rk>J_ab  →  ZOOM").
      SET_SUBPHASE(SUBPHASE_ASC_AB_ZOOM).
    }
    RETURN.
  }

  // --- Exit 4: q below lower corridor bound + apo below zoom target -
  LOCAL q_zoom_lim IS q_tgt * ASC_Q_ZOOM_ENTRY.
  IF ASC_Q_CTRL < q_zoom_lim AND SHIP:ORBIT:APOAPSIS < zoom_apo
     AND ASC_ESTIMATOR_VALIDITY <> ASC_INVALID {
    IFC_SET_ALERT("ASC: q below corridor low  →  ZOOM").
    SET_SUBPHASE(SUBPHASE_ASC_AB_ZOOM).
    RETURN.
  }

  // --- Exit 5: AB speed ceiling ----------------------------------------
  // When orbital energy rate (Ė_orb) has been persistently near zero, the
  // AB engines have reached their thrust-drag equilibrium and can no longer
  // accelerate the vehicle toward orbit.  Commit to rockets regardless of
  // the J comparison — staying in AB mode makes no further progress.
  IF ASC_EDOT_VAL < ASC_CEIL_EDOT_THR AND ASC_ESTIMATOR_VALIDITY <> ASC_INVALID {
    IF ASC_CEIL_PERSIST_UT < 0 { SET ASC_CEIL_PERSIST_UT TO TIME:SECONDS. }
    ELSE IF TIME:SECONDS - ASC_CEIL_PERSIST_UT >= ASC_CEIL_PERSIST_S {
      IFC_SET_ALERT("ASC: AB speed ceiling  →  ZOOM").
      SET ASC_CEIL_PERSIST_UT TO -1.
      SET_SUBPHASE(SUBPHASE_ASC_AB_ZOOM).
      RETURN.
    }
  } ELSE {
    SET ASC_CEIL_PERSIST_UT TO -1.
  }
}

// ── AB_ZOOM sub-phase ─────────────────────────────────────
//
// Control objective: pitch up at a controlled rate to convert residual
// kinetic energy into apoapsis altitude before the mode switch.
//
// Pitch rises at ASC_ZOOM_PITCH_RATE deg/s toward ASC_ZOOM_PITCH_MAX.
// Two guards prevent over-pitching:
//   – AoA guard: stop pitching if approaching the AoA limit
//   – Apo growth guard: hold pitch if apoapsis is rising too fast
//
// Exit conditions (all commit the mode switch):
//   1. Apoapsis reaches target
//   2. J_rk > J_ab for persistence window
//   3. AB flameout / thrust collapse
//   4. AoA sustained at limit with DEGRADED or INVALID estimator
//
FUNCTION _ASC_ZOOM {
  PARAMETER dt.

  LOCAL aoa_lim    IS ASC_AOA_LIMIT_CACHED.
  LOCAL zoom_apo   IS ASC_ZOOM_APO_TARGET_CACHED.

  // --- Pitch rise ---------------------------------------------------
  LOCAL pitch_target IS ASC_PITCH_BIAS + ASC_ZOOM_PITCH_RATE * dt.
  SET pitch_target TO MIN(pitch_target, ASC_ZOOM_PITCH_MAX).

  // AoA guard: hold pitch if approaching limit.
  IF ASC_AOA_CTRL > aoa_lim * 0.85 {
    SET pitch_target TO ASC_PITCH_BIAS. // hold
  }

  // Apo growth guard: stop pitching if rising too fast near target.
  LOCAL elapsed IS MAX(TIME:SECONDS - ASC_APO_PREV_UT, 0.5).
  LOCAL apo_rate IS (ASC_APO_VAL - ASC_APO_PREV) / elapsed.
  IF apo_rate > ASC_ZOOM_APO_RATE_MAX AND ASC_APO_VAL > zoom_apo * 0.80 {
    SET pitch_target TO ASC_PITCH_BIAS. // hold
  }

  SET ASC_PITCH_BIAS TO MOVE_TOWARD(ASC_PITCH_BIAS, pitch_target,
                                    ASC_ZOOM_PITCH_RATE * dt).

  // --- Steering: surface prograde + rising pitch bias ---------------
  LOCAL surf_fpa IS _ASC_SURF_FPA_SMOOTH(dt).
  LOCAL fpa_cmd  IS CLAMP(surf_fpa + ASC_PITCH_BIAS, ASC_ZOOM_FPA_MIN, ASC_ZOOM_FPA_MAX).
  LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(_ASC_SURFACE_HDG(), dt).
  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _ASC_SET_THR_CMD(1.0, dt, 0.9).

  IF NOT _ASC_SWITCH_ARMED() {
    SET ASC_PERSIST_UT TO -1.
    SET ASC_FLAMEOUT TO FALSE.
    RETURN.
  }

  // --- Exit 1: apoapsis target reached (waived persistence) ---------
  IF SHIP:ORBIT:APOAPSIS >= zoom_apo {
    _ASC_COMMIT_ROCKET("apo target reached").
    RETURN.
  }

  // --- Exit 2: J_rk > J_ab persistence met -------------------------
  IF _ASC_CHECK_PERSIST() {
    _ASC_COMMIT_ROCKET("J_rk>J_ab zoom").
    RETURN.
  }

  // --- Exit 3: flameout (persistence waived — unambiguous event) ----
  _ASC_CHECK_FLAMEOUT().
  IF ASC_FLAMEOUT {
    _ASC_COMMIT_ROCKET("AB flameout").
    RETURN.
  }

  // --- Exit 3b: fuel floor -------------------------------------------
  IF _ASC_CHECK_FUEL_FLOOR() {
    _ASC_COMMIT_ROCKET("fuel floor zoom").
    RETURN.
  }

  // --- Exit 4: AoA limit with reduced controllability ---------------
  IF ASC_AOA_CTRL >= aoa_lim AND ASC_ESTIMATOR_VALIDITY <> ASC_VALID {
    _ASC_COMMIT_ROCKET("AoA limit").
    RETURN.
  }
}

// ── ROCKET_SUSTAIN sub-phase ──────────────────────────────
//
// Control objective: raise apoapsis to the target altitude.
//
// Two operating modes:
//
//   is_spooling = TRUE (immediately after mode switch):
//     AB engines are winding down.  Hold a simple surface→orbital
//     prograde blend, no aggressive pitch authority, no J comparison.
//     Clears when AB thrust ratio falls below ASC_SPOOL_THR_THRESH.
//
//   is_spooling = FALSE (normal guidance):
//     Pitch law: apoapsis error drives a nose-up bias that decreases
//     as ETA:APOAPSIS shortens (flattening the trajectory as apoapsis
//     approaches to reduce gravity losses).
//
// Steering blend: surface prograde → orbital prograde over ASC_STEER_BLEND_S.
// Pitch bias is SLEWED not stepped, preventing FPA kick at mode switch.
//
FUNCTION _ASC_ROCKET_SUSTAIN {
  PARAMETER dt.

  LOCAL final_apo IS ASC_FINAL_APO_TARGET_CACHED.

  // --- Update steering blend ----------------------------------------
  IF ASC_BLEND_START_UT >= 0 {
    LOCAL elapsed IS TIME:SECONDS - ASC_BLEND_START_UT.
    SET ASC_STEER_BLEND TO CLAMP(elapsed / ASC_STEER_BLEND_S, 0, 1).
  }

  // --- Spool-down management ----------------------------------------
  IF ASC_IS_SPOOLING {
    LOCAL t_ab_now   IS 0.
    LOCAL t_ab_rated IS 0.
    LOCAL i IS 0.
    UNTIL i >= ASC_AB_ENGINES:LENGTH {
      LOCAL eng IS ASC_AB_ENGINES[i].
      SET t_ab_now   TO t_ab_now   + eng:THRUST.
      SET t_ab_rated TO t_ab_rated + eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
      SET i TO i + 1.
    }
    // Spool complete when AB thrust has decayed or engines are truly off.
    IF t_ab_rated < 0.1 OR (t_ab_now / t_ab_rated) < ASC_SPOOL_THR_THRESH {
      SET ASC_IS_SPOOLING TO FALSE.
      IFC_SET_ALERT("ASC: spool complete  —  rocket guidance active").
    } ELSE {
      // Prograde hold during spool-down; reduced pitch authority.
      LOCAL srf_fpa IS _ASC_SURF_FPA_SMOOTH(dt).
      LOCAL orb_fpa IS _ASC_ORB_FPA().
      LOCAL fpa_blend IS srf_fpa * (1 - ASC_STEER_BLEND) + orb_fpa * ASC_STEER_BLEND.
      LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(_ASC_BLEND_HDG(ASC_STEER_BLEND), dt).
      AA_SET_DIRECTOR(hdg_cmd, fpa_blend).
      SET TELEM_AA_HDG_CMD TO hdg_cmd.
      SET TELEM_AA_FPA_CMD TO fpa_blend.
      SET TELEM_LOC_CORR   TO 0.
      SET TELEM_GS_CORR    TO 0.
      _ASC_SET_THR_CMD(1.0, dt, 1.2).
      RETURN.
    }
  }

  // --- Normal ROCKET_SUSTAIN pitch law ------------------------------
  //
  // pitch_target = apo_error * KP * eta_factor
  //
  //   apo_error  > 0 → apoapsis below target → pitch up
  //   eta_factor   → 1.0 when ETA is long, 0.0 when ETA is short
  //               (flattens trajectory as apoapsis approaches)
  //
  LOCAL apo_err    IS final_apo - SHIP:ORBIT:APOAPSIS.
  LOCAL eta_factor IS CLAMP(ETA:APOAPSIS / ASC_ETA_FLATTEN_S, 0, 1).
  LOCAL bias_cmd   IS CLAMP(apo_err * ASC_APO_KP, 0, ASC_PITCH_MAX_SUSTAIN) * eta_factor.

  // Slew — never step — to avoid FPA kick (pitch bias may be large on entry).
  SET ASC_PITCH_BIAS TO MOVE_TOWARD(ASC_PITCH_BIAS, bias_cmd,
                                    ASC_PITCH_SLEW_DPS * dt).

  // Blended steering: surface → orbital prograde.
  LOCAL srf_fpa IS _ASC_SURF_FPA_SMOOTH(dt).
  LOCAL orb_fpa IS _ASC_ORB_FPA().
  LOCAL fpa_base IS srf_fpa * (1 - ASC_STEER_BLEND) + orb_fpa * ASC_STEER_BLEND.
  LOCAL fpa_cmd  IS CLAMP(fpa_base + ASC_PITCH_BIAS, -15.0, ASC_PITCH_MAX_SUSTAIN).

  LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(_ASC_BLEND_HDG(ASC_STEER_BLEND), dt).
  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _ASC_SET_THR_CMD(1.0, dt, 1.0).

  // --- Exit: apoapsis at or above lower band edge -------------------
  // One-sided check: trigger as soon as apo reaches (target - band).
  // Catches both the normal case (apo settling in band) and the fast-growth
  // case where apo blows through the narrow two-sided window in one cycle.
  IF SHIP:ORBIT:APOAPSIS >= final_apo - ASC_APO_BAND_M {
    IFC_SET_ALERT("ASC: apo on target  →  RKT CLOSEOUT").
    SET ASC_PITCH_BIAS TO 0.
    SET ASC_STEER_BLEND TO 1.0. // fully orbital from here
    SET_SUBPHASE(SUBPHASE_ASC_CLOSEOUT).
    RETURN.
  }
}

// ── ROCKET_CLOSEOUT sub-phase ─────────────────────────────
//
// Control objective: accumulate horizontal orbital velocity while
// keeping apoapsis inside the target band.
//
// Pitch targets orbital prograde (FPA → 0).  If apoapsis dips below the
// lower band edge, a temporary guard bias pitches up to restore it.
//
FUNCTION _ASC_ROCKET_CLOSEOUT {
  PARAMETER dt.

  LOCAL final_apo IS ASC_FINAL_APO_TARGET_CACHED.
  LOCAL apo_low   IS final_apo - ASC_APO_BAND_M.

  SET ASC_STEER_BLEND TO 1.0.
  SET ASC_PITCH_BIAS  TO 0.

  // Orbital prograde FPA (target: 0, but guard can override).
  LOCAL orb_fpa IS _ASC_ORB_FPA().

  // Apoapsis guard: brief nose-up if apo dips below the band.
  LOCAL apo_guard IS 0.
  IF SHIP:ORBIT:APOAPSIS < apo_low {
    LOCAL deficit IS apo_low - SHIP:ORBIT:APOAPSIS.
    SET apo_guard TO CLAMP(deficit * ASC_APO_KP * 2, 0, ASC_APO_GUARD_MAX_DEG).
  }

  LOCAL fpa_cmd IS CLAMP(orb_fpa + apo_guard, -5.0, 10.0).
  LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(_ASC_BLEND_HDG(ASC_STEER_BLEND), dt).
  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  // Cut thrust if apoapsis has overshot the upper band edge — coast rather
  // than adding more energy.  Engine will re-light in CIRCULARISE if needed.
  LOCAL closeout_thr IS 1.0.
  IF SHIP:ORBIT:APOAPSIS > final_apo + ASC_APO_BAND_M {
    SET closeout_thr TO 0.0.
  }
  _ASC_SET_THR_CMD(closeout_thr, dt, 0.8).

  // --- Exit: near apoapsis, ready to coast and circularise ----------
  IF ETA:APOAPSIS < ASC_CIRC_ETA_S {
    IFC_SET_ALERT("ASC: near apoapsis  →  CIRCULARISE").
    SET_SUBPHASE(SUBPHASE_ASC_CIRCULARISE).
    RETURN.
  }
}

// ── CIRCULARISE sub-phase ─────────────────────────────────
//
// Control objective: close the orbit.
//
// Two modes:
//   Coast  (ETA:APOAPSIS > 5 s) — throttle off, hold orbital prograde.
//   Burn   (ETA:APOAPSIS ≤ 5 s) — full throttle, orbital prograde.
//
// Exit when periapsis exceeds ASC_CIRC_PERI_TGT.
//
FUNCTION _ASC_CIRCULARISE {
  PARAMETER dt.

  LOCAL orb_fpa IS _ASC_ORB_FPA().
  LOCAL orb_hdg IS _ASC_ORB_HDG().

  LOCAL hdg_cmd IS _ASC_HDG_SMOOTH(orb_hdg, dt).
  AA_SET_DIRECTOR(hdg_cmd, orb_fpa).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO orb_fpa.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  IF ETA:APOAPSIS > 5 {
    _ASC_SET_THR_CMD(0.0, dt, 1.5).   // coasting
    RETURN.
  }

  _ASC_SET_THR_CMD(1.0, dt, 1.5).   // burning

  // --- Exit: orbit closed -------------------------------------------
  IF SHIP:ORBIT:PERIAPSIS >= ASC_CIRC_PERI_TGT {
    IFC_SET_ALERT("ASC: orbit closed  —  DONE").
    _ASC_SET_THR_CMD(0.0, dt, 2.0).
    SET_PHASE(PHASE_DONE).
    RETURN.
  }
}

// ── Telemetry export ──────────────────────────────────────

FUNCTION _ASC_UPDATE_TELEM {
  SET TELEM_ASC_PITCH_BIAS TO ASC_PITCH_BIAS.
  SET TELEM_ASC_BLEND      TO ASC_STEER_BLEND.
  SET TELEM_ASC_SPOOLING   TO ASC_IS_SPOOLING.
}

// ── Main entry point ──────────────────────────────────────
//
// Called every cycle from the IFC main loop while IFC_PHASE = PHASE_ASCENT.
// On the first call, runs ASC_STATE_INIT() to classify engines, store
// propellant capacities, and pre-seed all EMA filter states.
// Every subsequent call runs the full estimator update then dispatches
// to the active sub-phase handler.
//
FUNCTION RUN_ASCENT {
  // One-time initialisation on first call.
  IF NOT ASC_INITIALIZED {
    ASC_STATE_INIT().
    LOCAL hdg_now IS GET_COMPASS_HDG().
    SET ASC_HDG_REF TO hdg_now.
    // If ascent follows takeoff, keep the runway azimuth as the heading reference.
    IF ABS(WRAP_180(hdg_now - TO_RWY_HDG)) <= 25 {
      SET ASC_HDG_REF TO TO_RWY_HDG.
    }
    SET ASC_HDG_CMD_FILT  TO ASC_HDG_REF.
    SET ASC_SURF_FPA_FILT TO _ASC_SURF_FPA().
    SET ASC_PITCH_BIAS     TO 0.
    SET ASC_STEER_BLEND    TO 0.
    SET ASC_BLEND_START_UT TO -1.
    SET ASC_IS_SPOOLING    TO FALSE.
    SET ASC_FLAMEOUT       TO FALSE.
    SET ASC_PERSIST_UT     TO -1.
    SET_SUBPHASE(SUBPHASE_ASC_AB_CORRIDOR).

    // Physics-tick-rate flameout detector.
    // Reads only pre-computed globals — no AC_PARAM or function calls allowed
    // inside WHEN...THEN conditions (they run at physics rate, not main loop rate).
    // ASC_AB_THR_RATIO is updated each main-loop cycle in ASC_STATE_UPDATE.
    // ASC_REGIME_MACH_CACHED is set once in ASC_STATE_INIT.
    // No PRESERVE: once fired, the main loop handles the transition and the
    // trigger is no longer needed.
    WHEN ASC_AB_ENGINES:LENGTH > 0
     AND ASC_AB_THR_RATIO < ASC_FLAMEOUT_THR_RATIO
     AND TELEM_ASC_MACH < ASC_REGIME_MACH_CACHED * ASC_FLAMEOUT_MACH_GUARD_FRAC
     AND (SHIP:AIRSPEED > 120 OR ALT:RADAR > 1000 OR SHIP:VERTICALSPEED > 25)
     AND IFC_PHASE = PHASE_ASCENT
     AND (IFC_SUBPHASE = SUBPHASE_ASC_AB_CORRIDOR
          OR IFC_SUBPHASE = SUBPHASE_ASC_AB_ZOOM)
    THEN {
      SET ASC_FLAMEOUT TO TRUE.
    }
  }

  // Per-cycle state estimator + mode valuation update.
  ASC_STATE_UPDATE(IFC_ACTUAL_DT).

  // Sub-phase dispatch.
  LOCAL sp IS IFC_SUBPHASE.
  IF      sp = SUBPHASE_ASC_AB_CORRIDOR    { _ASC_CORRIDOR(IFC_ACTUAL_DT).       }
  ELSE IF sp = SUBPHASE_ASC_AB_ZOOM        { _ASC_ZOOM(IFC_ACTUAL_DT).           }
  ELSE IF sp = SUBPHASE_ASC_ROCKET_SUSTAIN { _ASC_ROCKET_SUSTAIN(IFC_ACTUAL_DT). }
  ELSE IF sp = SUBPHASE_ASC_CLOSEOUT       { _ASC_ROCKET_CLOSEOUT(IFC_ACTUAL_DT). }
  ELSE IF sp = SUBPHASE_ASC_CIRCULARISE    { _ASC_CIRCULARISE(IFC_ACTUAL_DT).    }

  _ASC_UPDATE_TELEM().
}
