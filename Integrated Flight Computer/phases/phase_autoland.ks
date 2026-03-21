@LAZYGLOBAL OFF.

// ============================================================
// phase_autoland.ks  -  Integrated Flight Computer
//
// Handles post-glideslope phases:
//   PHASE_FLARE      - arrest sink rate, idle throttle
//   PHASE_TOUCHDOWN  - detect ground contact, deploy brakes/spoilers
//   PHASE_ROLLOUT    - decelerate to a stop on runway heading
// ============================================================

// ── Public entry point ────────────────────────────────────
FUNCTION RUN_AUTOLAND {
  IF IFC_PHASE = PHASE_FLARE {
    _RUN_FLARE().
  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN {
    _RUN_TOUCHDOWN().
  } ELSE IF IFC_PHASE = PHASE_ROLLOUT {
    _RUN_ROLLOUT().
  }
}

FUNCTION _DEPLOY_TOUCHDOWN_SPOILERS {
  LOCAL ag_sp IS 0.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_spoilers") {
    SET ag_sp TO ACTIVE_AIRCRAFT["ag_spoilers"].
  }
  IF ag_sp > 0 { TRIGGER_AG(ag_sp, TRUE). }
}

// ─────────────────────────────────────────────────────────
// BOUNCE RECOVERY helper
// Detects a post-touchdown bounce and transitions back to FLARE.
//
// Parameters (all resolved by the caller from AC_PARAM or constants):
//   br_agl_m     - m AGL above which aircraft is considered airborne
//   br_min_vs    - m/s minimum upward VS to count as a real bounce
//   br_confirm_s - s airborne conditions must persist before recovery triggers
//   br_max_s     - s after phase entry where bounce recovery is still checked
//   reset_init   - TRUE when called from PHASE_TOUCHDOWN (resets TOUCHDOWN_INIT_DONE
//                  so the one-time handoff sequence re-runs on re-landing)
//
// Returns TRUE when recovery was triggered; caller must RETURN immediately.
// ─────────────────────────────────────────────────────────
FUNCTION _CHECK_BOUNCE_RECOVERY {
  PARAMETER br_agl_m, br_min_vs, br_confirm_s, br_max_s, reset_init.
  LOCAL agl IS GET_AGL().
  LOCAL flare_h IS GET_RUNWAY_REL_HEIGHT().
  LOCAL airborne IS PHASE_ELAPSED() < br_max_s AND
                   SHIP:STATUS <> "LANDED" AND
                   agl > br_agl_m AND
                   SHIP:VERTICALSPEED > br_min_vs.
  IF airborne {
    IF BOUNCE_RECOVERY_START_UT < 0 { SET BOUNCE_RECOVERY_START_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - BOUNCE_RECOVERY_START_UT >= br_confirm_s {
      BRAKES OFF.
      UNLOCK WHEELSTEERING.
      IF reset_init { SET TOUCHDOWN_INIT_DONE TO FALSE. }
      SET TOUCHDOWN_CANDIDATE_UT      TO -1.
      SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
      SET BOUNCE_RECOVERY_START_UT    TO -1.
      SET ROLLOUT_PITCH_CMD_PREV      TO 0.
      LOCAL pitch_now IS GET_PITCH().
      SET ROLLOUT_PITCH_REF_DEG TO pitch_now.
      SET ROLLOUT_PITCH_TGT_DEG TO pitch_now.
      SET SHIP:CONTROL:YAW   TO 0.
      SET SHIP:CONTROL:ROLL  TO 0.
      SET SHIP:CONTROL:PITCH TO 0.
      LOCAL entry_ias IS MAX(GET_IAS(), 10).
      SET FLARE_PITCH_CMD TO ARCTAN(SHIP:VERTICALSPEED / entry_ias).
      SET FLARE_ENTRY_VS  TO CLAMP(SHIP:VERTICALSPEED, FLARE_MIN_ENTRY_SINK_VS, -0.05).
      SET FLARE_ENTRY_AGL TO MAX(flare_h, 1).
      SET IFC_ALERT_TEXT TO "BOUNCE recovery: returning to FLARE (" + ROUND(flare_h, 1) + " m rw)".
      SET IFC_ALERT_UT   TO TIME:SECONDS.
      SET_PHASE(PHASE_FLARE).
      RETURN TRUE.
    }
  } ELSE {
    SET BOUNCE_RECOVERY_START_UT TO -1.
  }
  RETURN FALSE.
}

// ─────────────────────────────────────────────────────────
// FLARE
// Sink-rate targeting with AoA ceiling.
//
// As runway-relative height decreases from FLARE_ENTRY_AGL toward 0, the target
// vertical speed is linearly interpolated from the entry sink
// rate to TOUCHDOWN_VS (-0.3 m/s).  That target VS is
// converted to a flight path angle and fed to AA Director.
// AoA is protected by AA's own MAXAOA FBW limiter.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLARE {
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE   TO 0.
  SET TELEM_RO_YAW_GATE    TO 0.
  SET TELEM_RO_PITCH_TGT   TO 0.
  SET TELEM_RO_PITCH_ERR   TO 0.
  SET TELEM_RO_PITCH_FF    TO 0.
  LOCAL agl    IS GET_RUNWAY_REL_HEIGHT().
  LOCAL ias    IS MAX(GET_IAS(), 10).  // floor prevents divide-by-zero at very low speed
  LOCAL vs_now IS SHIP:VERTICALSPEED.

  // Per-aircraft flare overrides.
  // flare_touchdown_vs is a negative value (e.g. -0.05) with -1 as sentinel.
  // AC_PARAM cannot express "< 0 AND <> -1", so this one stays inline.
  LOCAL flare_touchdown_vs IS TOUCHDOWN_VS.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flare_touchdown_vs")
      AND ACTIVE_AIRCRAFT["flare_touchdown_vs"] <> -1
      AND ACTIVE_AIRCRAFT["flare_touchdown_vs"] < 0 {
    SET flare_touchdown_vs TO ACTIVE_AIRCRAFT["flare_touchdown_vs"].
  }
  LOCAL flare_vs_gain           IS AC_PARAM("flare_ias_to_vs_gain",       FLARE_IAS_TO_VS_GAIN,     0).
  LOCAL flare_balloon_vs        IS AC_PARAM("flare_balloon_vs_trigger",   FLARE_BALLOON_VS_TRIGGER, 0).
  LOCAL flare_balloon_push      IS AC_PARAM("flare_balloon_fpa_push",     FLARE_BALLOON_FPA_PUSH,   0).
  LOCAL flare_rate_min          IS AC_PARAM("flare_pitch_rate_min",       FLARE_PITCH_RATE_MIN,     0.001).
  LOCAL flare_rate_max          IS AC_PARAM("flare_pitch_rate_max",       FLARE_PITCH_RATE_MAX,     0.001).
  LOCAL flare_roundout_agl      IS AC_PARAM("flare_roundout_agl",         FLARE_ROUNDOUT_AGL_M,     0).
  LOCAL flare_roundout_strength IS MIN(AC_PARAM("flare_roundout_strength", FLARE_ROUNDOUT_STRENGTH, 0), 1).
  LOCAL touchdown_confirm_s     IS AC_PARAM("touchdown_confirm_s",         TOUCHDOWN_CONFIRM_S,      0.001).
  LOCAL touchdown_confirm_max_abs_vs IS AC_PARAM("touchdown_confirm_max_abs_vs", TOUCHDOWN_CONFIRM_MAX_ABS_VS, 0.001).
  // Vref: used for both energy-compensation sink and autothrottle target.
  LOCAL vref IS AC_PARAM("v_ref", ACTIVE_V_APP - 10, 0).
  IF flare_rate_max < flare_rate_min { SET flare_rate_max TO flare_rate_min. }

  // Progress: 0 = just entered flare, 1 = at ground level.
  LOCAL frac IS CLAMP(1 - agl / FLARE_ENTRY_AGL, 0, 1).

  // Interpolate target sink rate from entry VS down to touchdown target.
  LOCAL base_vs IS FLARE_ENTRY_VS + (flare_touchdown_vs - FLARE_ENTRY_VS) * frac.

  // Energy compensation: if still fast versus Vref, command extra sink.
  // Fade speed-driven extra sink out as we approach touchdown:
  // early flare can still shed energy, but the last part prioritizes a soft touchdown.
  LOCAL speed_excess IS MAX(ias - vref, 0).
  LOCAL speed_sink_blend IS CLAMP(1 - frac, 0, 1).
  LOCAL tgt_vs IS base_vs - speed_excess * flare_vs_gain * speed_sink_blend.

  // Final roundout: in the last few meters, blend sink target quickly
  // toward flare_touchdown_vs to reduce hard wheel contact.
  IF flare_roundout_agl > 0 AND agl < flare_roundout_agl AND flare_roundout_strength > 0 {
    LOCAL round_frac IS CLAMP(1 - agl / flare_roundout_agl, 0, 1) * flare_roundout_strength.
    SET tgt_vs TO tgt_vs + (flare_touchdown_vs - tgt_vs) * round_frac.
  }

  SET tgt_vs TO CLAMP(tgt_vs, FLARE_MAX_SINK_VS, -0.05).

  // Convert target VS to flight path angle (degrees).
  LOCAL tgt_fpa IS ARCTAN(tgt_vs / ias).

  // Anti-balloon recovery: if we climb in flare, bias nose down immediately.
  // Floor the cap at the entry glideslope angle so the push can't cascade
  // into a dive steeper than what we entered the flare with.
  IF vs_now > flare_balloon_vs {
    LOCAL balloon_floor IS ARCTAN(FLARE_ENTRY_VS / MAX(ias, 1)).
    SET tgt_fpa TO MAX(MIN(tgt_fpa, FLARE_PITCH_CMD - flare_balloon_push), balloon_floor).
  }

  // AoA ceiling is handled by AA's own MAXAOA FBW limiter — no IFC-level
  // guard here.  A separate guard would freeze the flare FPA at the approach
  // descent angle any time approach AoA is near the limit, preventing the
  // flare from ever arresting sink rate.  AA is the correct backstop.

  // Speed-aware flare response: faster at high IAS, gentler at low IAS.
  LOCAL rate_span IS MAX(FLARE_RATE_HIGH_IAS - FLARE_RATE_LOW_IAS, 1).
  LOCAL rate_blend IS CLAMP((ias - FLARE_RATE_LOW_IAS) / rate_span, 0, 1).
  LOCAL flare_rate IS flare_rate_min + (flare_rate_max - flare_rate_min) * rate_blend.

  // Smooth the FPA transition at flare_rate deg/s.
  SET FLARE_PITCH_CMD TO MOVE_TOWARD(
    FLARE_PITCH_CMD, tgt_fpa,
    flare_rate * IFC_ACTUAL_DT
  ).

  AA_SET_DIRECTOR(ACTIVE_RWY_HDG, FLARE_PITCH_CMD).
  SET TELEM_AA_HDG_CMD   TO ACTIVE_RWY_HDG.
  SET TELEM_AA_FPA_CMD   TO FLARE_PITCH_CMD.
  SET TELEM_FLARE_TGT_VS TO tgt_vs.
  SET TELEM_FLARE_FRAC   TO frac.

  // Flare autothrottle: continue cascade targeting Vref to prevent stall
  // from IAS loss that idle throttle causes on high-drag aircraft.
  // Minimum throttle is 0 (no forced idle floor) so the aircraft can still
  // reduce thrust naturally when above Vref.
  AT_RUN_SPEED_HOLD(vref, 0, 1).

  // Transition: touchdown.
  // If KSP reports LANDED, commit immediately (no debounce) so flare ends
  // the moment the wheels touch.  Keep the fallback path debounced.
  IF SHIP:STATUS = "LANDED" {
    _DEPLOY_TOUCHDOWN_SPOILERS().
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO GET_PITCH().
    SET TOUCHDOWN_INIT_DONE         TO FALSE.
    SET TOUCHDOWN_CANDIDATE_UT      TO -1.
    SET BOUNCE_RECOVERY_START_UT    TO -1.
    SET_PHASE(PHASE_TOUCHDOWN).
    RETURN.
  }

  // Fallback touchdown path: debounced for low-altitude near-ground cases.
  LOCAL td_fallback IS agl < TOUCHDOWN_FALLBACK_AGL_M AND vs_now <= TOUCHDOWN_FALLBACK_MAX_VS.
  IF td_fallback {
    IF TOUCHDOWN_CANDIDATE_UT < 0 { SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - TOUCHDOWN_CANDIDATE_UT >= touchdown_confirm_s {
      IF ABS(vs_now) > touchdown_confirm_max_abs_vs {
        // Ignore high-rate rebound spikes; wait for a stable ground contact.
        SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS.
        RETURN.
      }
      // Capture pre-impact pitch while still in flare; this is a better
      // nose-hold reference than waiting until touchdown transients begin.
      _DEPLOY_TOUCHDOWN_SPOILERS().
      SET TOUCHDOWN_CAPTURE_PITCH_DEG TO GET_PITCH().
      SET TOUCHDOWN_INIT_DONE         TO FALSE.
      SET TOUCHDOWN_CANDIDATE_UT      TO -1.
      SET BOUNCE_RECOVERY_START_UT    TO -1.
      SET_PHASE(PHASE_TOUCHDOWN).
    }
  } ELSE {
    SET TOUCHDOWN_CANDIDATE_UT TO -1.
  }
}

// ─────────────────────────────────────────────────────────
// TOUCHDOWN
// Deploy spoilers/reversers, then hand over to ground-roll logic.
// Switch AA from Director to standard FBW, then command rollout inputs.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_TOUCHDOWN {
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE   TO 0.
  SET TELEM_RO_YAW_GATE    TO 0.
  SET THROTTLE_CMD TO 0.

  LOCAL nose_hold_cmd              IS AC_PARAM("rollout_nose_hold_cmd",          ROLLOUT_NOSE_HOLD_CMD,          -0.5).
  LOCAL rollout_pitch_hold_kp      IS AC_PARAM("rollout_pitch_hold_kp",          ROLLOUT_PITCH_HOLD_KP,          0).
  LOCAL rollout_pitch_max_cmd      IS MIN(AC_PARAM("rollout_pitch_max_cmd",      ROLLOUT_PITCH_MAX_CMD,          0.001), 1).
  LOCAL rollout_pitch_max_down_cmd IS MIN(AC_PARAM("rollout_pitch_max_down_cmd", ROLLOUT_PITCH_MAX_DOWN_CMD,     0), rollout_pitch_max_cmd).
  LOCAL rollout_pitch_slew_per_s   IS AC_PARAM("rollout_pitch_slew_per_s",       ROLLOUT_PITCH_SLEW_PER_S,       0.001).
  // Use rollout nose target immediately during TOUCHDOWN to put the nose gear down.
  LOCAL touchdown_nose_target_pitch IS AC_PARAM("rollout_nose_target_pitch_deg", ROLLOUT_NOSE_TARGET_PITCH_DEG, -0.5).
  LOCAL touchdown_settle_s         IS AC_PARAM("rollout_touchdown_settle_s",     TOUCHDOWN_SETTLE_S,             0.001).
  LOCAL br_agl_m                   IS AC_PARAM("bounce_recovery_agl_m",         BOUNCE_RECOVERY_AGL_M,          0.001).
  LOCAL br_min_vs                  IS AC_PARAM("bounce_recovery_min_vs",        BOUNCE_RECOVERY_MIN_VS,         0.001).
  LOCAL br_confirm_s               IS AC_PARAM("bounce_recovery_confirm_s",     BOUNCE_RECOVERY_CONFIRM_S,      0.001).
  LOCAL br_max_s                   IS AC_PARAM("bounce_recovery_max_s",         BOUNCE_RECOVERY_MAX_S,          0.001).

  // One-time touchdown handoff work.
  IF NOT TOUCHDOWN_INIT_DONE {
    // Spoilers (action group from aircraft config).
    _DEPLOY_TOUCHDOWN_SPOILERS().

    // Reverse thrust (action group from aircraft config).
    LOCAL ag_tr IS 0.
    IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_thrust_rev") { SET ag_tr TO ACTIVE_AIRCRAFT["ag_thrust_rev"]. }
    IF ag_tr > 0 { TRIGGER_AG(ag_tr, TRUE). }

    // Drogue chute (action group from aircraft config).
    LOCAL ag_dr IS 0.
    IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_drogue") { SET ag_dr TO ACTIVE_AIRCRAFT["ag_drogue"]. }
    IF ag_dr > 0 { TRIGGER_AG(ag_dr, TRUE). }

    BRAKES OFF.

    // Keep AA in standard FBW mode for rollout damping.
    AA_RESTORE_FBW().
    UNLOCK STEERING.

    // Clear accumulated control inputs from the flare.
    SET SHIP:CONTROL:YAW   TO 0.
    SET SHIP:CONTROL:ROLL  TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    SET ROLLOUT_YAW_CMD_PREV   TO 0.
    SET ROLLOUT_PITCH_CMD_PREV TO 0.
    SET ROLLOUT_PITCH_REF_DEG TO touchdown_nose_target_pitch.
    SET ROLLOUT_PITCH_TGT_DEG TO ROLLOUT_PITCH_REF_DEG.

    // Capture touchdown heading and start wheelsteering from that heading.
    // We'll blend from touchdown heading -> runway heading as speed decays.
    SET ROLLOUT_ENTRY_HDG TO GET_COMPASS_HDG().
    LOCK WHEELSTEERING TO ROLLOUT_ENTRY_HDG.
    SET IFC_ALERT_TEXT TO "ROLLOUT armed: hdg " + ROUND(ROLLOUT_ENTRY_HDG, 1)
        + " IAS " + ROUND(GET_IAS(), 1) + " m/s".
    SET IFC_ALERT_UT   TO TIME:SECONDS.

    SET TOUCHDOWN_INIT_DONE TO TRUE.
  }

  // Closed-loop touchdown pitch command:
  // drive directly toward rollout nose target; suppress positive nose-up feedforward.
  LOCAL pitch_now IS GET_PITCH().
  LOCAL pitch_err IS ROLLOUT_PITCH_REF_DEG - pitch_now.
  LOCAL td_pitch_ff IS MIN(nose_hold_cmd, 0).
  LOCAL td_pitch_target IS td_pitch_ff + pitch_err * rollout_pitch_hold_kp.
  SET td_pitch_target TO CLAMP(td_pitch_target, -rollout_pitch_max_down_cmd, rollout_pitch_max_cmd).
  LOCAL td_pitch_cmd IS MOVE_TOWARD(
    ROLLOUT_PITCH_CMD_PREV,
    td_pitch_target,
    rollout_pitch_slew_per_s * IFC_ACTUAL_DT
  ).
  SET ROLLOUT_PITCH_CMD_PREV TO td_pitch_cmd.
  SET SHIP:CONTROL:PITCH TO td_pitch_cmd.
  SET TELEM_RO_PITCH_TGT TO ROLLOUT_PITCH_REF_DEG.
  SET TELEM_RO_PITCH_ERR TO pitch_err.
  SET TELEM_RO_PITCH_FF  TO td_pitch_ff.

  // Debounced bounce recovery (reset_init=TRUE so the handoff sequence re-arms on re-landing).
  IF _CHECK_BOUNCE_RECOVERY(br_agl_m, br_min_vs, br_confirm_s, br_max_s, TRUE) { RETURN. }

  // Hold briefly in TOUCHDOWN to absorb contact transients before rollout.
  IF PHASE_ELAPSED() >= touchdown_settle_s {
    SET TOUCHDOWN_INIT_DONE         TO FALSE.
    SET TOUCHDOWN_CANDIDATE_UT      TO -1.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
    SET BOUNCE_RECOVERY_START_UT    TO -1.
    SET_PHASE(PHASE_ROLLOUT).
  }
}

// ─────────────────────────────────────────────────────────
// ROLLOUT
// Ground-roll controller.
// - Keeps wheelsteering gentle at high speed by blending heading command:
//   touchdown heading -> runway heading as IAS decreases.
// - Delays/limits wheel brakes at high speed to avoid violent swerve.
// - Uses small rudder assist only at lower speeds.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_ROLLOUT {
  LOCAL ias IS GET_IAS().

  // Thrust reverser throttle management.
  // If the aircraft has reversers (ag_thrust_rev > 0), command full throttle
  // while above the deactivation speed, then cut reversers and throttle once
  // below it.  Only toggle the action group once (ROLLOUT_REV_DEACTIVATED flag).
  LOCAL ag_tr IS 0.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_thrust_rev") {
    SET ag_tr TO ACTIVE_AIRCRAFT["ag_thrust_rev"].
  }
  LOCAL thr_rev_deact IS AC_PARAM("thr_rev_deact_ias", THR_REV_DEACT_IAS, 0).
  IF ag_tr > 0 AND NOT ROLLOUT_REV_DEACTIVATED {
    IF ias > thr_rev_deact {
      SET THROTTLE_CMD TO 1.
    } ELSE {
      TRIGGER_AG(ag_tr, FALSE).
      SET ROLLOUT_REV_DEACTIVATED TO TRUE.
      SET THROTTLE_CMD TO 0.
    }
  } ELSE {
    SET THROTTLE_CMD TO 0.
  }

  LOCAL brake_max_ias              IS AC_PARAM("rollout_brake_max_ias",          ROLLOUT_BRAKE_MAX_IAS,          0.001).
  // rollout_brake_delay_s: per-aircraft override for the brake enable delay.
  LOCAL brake_delay_s              IS AC_PARAM("rollout_brake_delay_s",          ROLLOUT_BRAKE_DELAY_S,          0).
  LOCAL yaw_assist_ias             IS AC_PARAM("rollout_yaw_assist_ias",         ROLLOUT_YAW_ASSIST_IAS,         0.001).
  // roll_assist_ias: explicit 0 from config disables IFC roll assist, so guard = 0.
  LOCAL roll_assist_ias            IS AC_PARAM("rollout_roll_assist_ias",        ROLLOUT_ROLL_ASSIST_IAS,        0).
  LOCAL steer_min_blend            IS MIN(AC_PARAM("rollout_steer_min_blend",    ROLLOUT_STEER_MIN_BLEND,        0), 1).
  LOCAL rollout_yaw_kp             IS AC_PARAM("rollout_yaw_kp",                 KP_ROLLOUT_YAW,                 0).
  LOCAL rollout_yaw_slew_per_s     IS AC_PARAM("rollout_yaw_slew_per_s",         ROLLOUT_YAW_SLEW_PER_S,         0).
  LOCAL rollout_yaw_fade_ias       IS AC_PARAM("rollout_yaw_fade_ias",           ROLLOUT_YAW_FADE_IAS,           0.001).
  LOCAL rollout_yaw_max_cmd        IS MIN(AC_PARAM("rollout_yaw_max_cmd",        ROLLOUT_YAW_MAX_CMD,            0), 1).
  LOCAL nose_hold_cmd              IS AC_PARAM("rollout_nose_hold_cmd",          ROLLOUT_NOSE_HOLD_CMD,          -0.5).
  LOCAL nose_release_ias           IS AC_PARAM("rollout_nose_release_ias",       ROLLOUT_NOSE_RELEASE_IAS,       0.001).
  LOCAL nose_hold_min_s            IS AC_PARAM("rollout_nose_hold_min_s",        ROLLOUT_NOSE_HOLD_MIN_S,        0).
  // rollout_nose_target_pitch_deg: -1 is sentinel, so guard = -0.5 to accept 0 and positive.
  LOCAL rollout_nose_target_pitch  IS AC_PARAM("rollout_nose_target_pitch_deg",  ROLLOUT_NOSE_TARGET_PITCH_DEG,  -0.5).
  LOCAL rollout_nose_target_slew   IS AC_PARAM("rollout_nose_target_slew_dps",   ROLLOUT_NOSE_TARGET_SLEW_DPS,   0.001).
  LOCAL rollout_pitch_hold_kp      IS AC_PARAM("rollout_pitch_hold_kp",          ROLLOUT_PITCH_HOLD_KP,          0).
  LOCAL rollout_pitch_max_cmd      IS MIN(AC_PARAM("rollout_pitch_max_cmd",      ROLLOUT_PITCH_MAX_CMD,          0.001), 1).
  LOCAL rollout_pitch_max_down_cmd IS MIN(AC_PARAM("rollout_pitch_max_down_cmd", ROLLOUT_PITCH_MAX_DOWN_CMD,     0), rollout_pitch_max_cmd).
  LOCAL rollout_pitch_slew_per_s   IS AC_PARAM("rollout_pitch_slew_per_s",       ROLLOUT_PITCH_SLEW_PER_S,       0.001).
  LOCAL br_agl_m                   IS AC_PARAM("bounce_recovery_agl_m",         BOUNCE_RECOVERY_AGL_M,          0.001).
  LOCAL br_min_vs                  IS AC_PARAM("bounce_recovery_min_vs",        BOUNCE_RECOVERY_MIN_VS,         0.001).
  LOCAL br_confirm_s               IS AC_PARAM("bounce_recovery_confirm_s",     BOUNCE_RECOVERY_CONFIRM_S,      0.001).
  LOCAL br_max_s                   IS AC_PARAM("bounce_recovery_max_s",         BOUNCE_RECOVERY_MAX_S,          0.001).
  // yaw_sign: -1 or 1 only; AC_PARAM cannot express "any non-zero value". Keep inline.
  LOCAL yaw_sign IS -1.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_sign")
      AND ACTIVE_AIRCRAFT["rollout_yaw_sign"] <> 0 {
    SET yaw_sign TO ACTIVE_AIRCRAFT["rollout_yaw_sign"].
  }

  // Debounced bounce recovery (no TOUCHDOWN_INIT_DONE reset needed — ROLLOUT already cleared it).
  IF _CHECK_BOUNCE_RECOVERY(br_agl_m, br_min_vs, br_confirm_s, br_max_s, FALSE) { RETURN. }

  // Build a centerline-aware steering target from localizer error.
  LOCAL steer_target_hdg IS ACTIVE_RWY_HDG.
  LOCAL loc_corr IS 0.
  IF ACTIVE_ILS_ID <> "" {
    LOCAL dev IS _COMPUTE_ILS_DEVIATIONS().
    IF dev:HASKEY("loc") AND dev:HASKEY("dist") {
      LOCAL loc_m  IS dev["loc"].
      LOCAL dist_m IS dev["dist"].
      IF ABS(loc_m) <= ROLLOUT_LOC_MAX_ERR_M AND
         dist_m <= ROLLOUT_LOC_MAX_AHEAD_M AND
         dist_m >= -ROLLOUT_LOC_MAX_PAST_M {
        SET loc_corr TO CLAMP(-loc_m * KP_ROLLOUT_LOC, -MAX_ROLLOUT_LOC_CORR, MAX_ROLLOUT_LOC_CORR).
        SET steer_target_hdg TO WRAP_360(ACTIVE_RWY_HDG + loc_corr).
      }
    }
  }

  // Blend steering command as speed drops.
  LOCAL steer_span IS MAX(ROLLOUT_STEER_START_IAS - ROLLOUT_STEER_FULL_IAS, 1).
  LOCAL steer_blend_base IS CLAMP((ROLLOUT_STEER_START_IAS - ias) / steer_span, 0, 1).
  LOCAL steer_blend IS CLAMP(MAX(steer_blend_base, steer_min_blend), 0, 1).
  LOCAL hdg_delta IS WRAP_180(steer_target_hdg - ROLLOUT_ENTRY_HDG).
  LOCAL steer_hdg IS WRAP_360(ROLLOUT_ENTRY_HDG + hdg_delta * steer_blend).
  SET ROLLOUT_STEER_HDG TO steer_hdg.  // expose for telemetry logger
  LOCK WHEELSTEERING TO steer_hdg.
  SET TELEM_STEER_BLEND TO steer_blend.
  SET TELEM_RO_LOC_CORR TO loc_corr.

  // Wheel brakes: delayed and speed-gated to reduce high-speed instability.
  IF PHASE_ELAPSED() > brake_delay_s AND ias <= brake_max_ias {
    BRAKES ON.
  } ELSE {
    BRAKES OFF.
  }

  // Rudder assist starts near touchdown, ramps up as speed decays.
  // hdg_err is computed here (outside the IAS gate) so telemetry always sees it.
  LOCAL hdg_err IS WRAP_180(GET_COMPASS_HDG() - steer_hdg).
  SET TELEM_RO_HDG_ERR TO hdg_err.
  LOCAL yaw_gate IS 0.
  LOCAL yaw_scale IS 0.
  LOCAL yaw_cmd_target IS 0.
  IF SHIP:STATUS = "LANDED" {
    IF ias <= yaw_assist_ias {
      LOCAL yaw_span IS MAX(yaw_assist_ias - ROLLOUT_STEER_FULL_IAS, 1).
      // Guard against large heading-wrap spikes on touchdown.
      IF ABS(hdg_err) <= ROLLOUT_YAW_ERR_GUARD_DEG {
        LOCAL yaw_scale_high IS CLAMP((yaw_assist_ias - ias) / yaw_span, 0, 1).
        LOCAL yaw_fade_span IS MAX(rollout_yaw_fade_ias - ROLLOUT_DONE_IAS, 1).
        LOCAL yaw_scale_low IS CLAMP((ias - ROLLOUT_DONE_IAS) / yaw_fade_span, 0, 1).
        SET yaw_scale TO yaw_scale_high * yaw_scale_low.
        SET yaw_cmd_target TO (hdg_err * rollout_yaw_kp * yaw_sign) * yaw_scale.
      } ELSE {
        SET yaw_gate TO 2.
      }
    } ELSE {
      SET yaw_gate TO 1.
    }
  } ELSE {
    // Airborne or gear unload: suppress rudder assist to avoid feeding oscillation.
    SET yaw_gate TO 3.
  }
  SET yaw_cmd_target TO CLAMP(yaw_cmd_target, -rollout_yaw_max_cmd, rollout_yaw_max_cmd).
  SET TELEM_RO_YAW_SCALE TO yaw_scale.
  SET TELEM_RO_YAW_GATE  TO yaw_gate.
  SET TELEM_RO_YAW_TGT   TO yaw_cmd_target.
  LOCAL yaw_cmd IS MOVE_TOWARD(
    ROLLOUT_YAW_CMD_PREV,
    yaw_cmd_target,
    rollout_yaw_slew_per_s * IFC_ACTUAL_DT
  ).
  SET ROLLOUT_YAW_CMD_PREV TO yaw_cmd.
  SET SHIP:CONTROL:YAW TO yaw_cmd.

  // Nose gear protection:
  // 1) Hold touchdown pitch attitude with feedback right after landing.
  // 2) Once stable and slow enough, ramp target pitch down gradually.
  LOCAL desired_pitch_deg IS ROLLOUT_PITCH_REF_DEG.
  IF SHIP:STATUS = "LANDED" AND PHASE_ELAPSED() >= nose_hold_min_s AND ias <= nose_release_ias {
    SET desired_pitch_deg TO rollout_nose_target_pitch.
  }
  SET ROLLOUT_PITCH_TGT_DEG TO MOVE_TOWARD(
    ROLLOUT_PITCH_TGT_DEG,
    desired_pitch_deg,
    rollout_nose_target_slew * IFC_ACTUAL_DT
  ).

  LOCAL pitch_cmd_ff IS 0.
  IF SHIP:STATUS = "LANDED" AND ABS(nose_hold_cmd) > 0 {
    IF PHASE_ELAPSED() < nose_hold_min_s OR ias > nose_release_ias {
      SET pitch_cmd_ff TO nose_hold_cmd.
    } ELSE {
      LOCAL nose_span IS MAX(nose_release_ias - ROLLOUT_DONE_IAS, 1).
      LOCAL nose_scale IS CLAMP((ias - ROLLOUT_DONE_IAS) / nose_span, 0, 1).
      SET pitch_cmd_ff TO nose_hold_cmd * nose_scale.
    }
  }

  LOCAL pitch_now IS GET_PITCH().
  LOCAL pitch_err IS ROLLOUT_PITCH_TGT_DEG - pitch_now.
  LOCAL pitch_cmd_target IS pitch_cmd_ff + pitch_err * rollout_pitch_hold_kp.
  SET pitch_cmd_target TO CLAMP(pitch_cmd_target, -rollout_pitch_max_down_cmd, rollout_pitch_max_cmd).
  LOCAL pitch_cmd IS MOVE_TOWARD(
    ROLLOUT_PITCH_CMD_PREV,
    pitch_cmd_target,
    rollout_pitch_slew_per_s * IFC_ACTUAL_DT
  ).
  SET ROLLOUT_PITCH_CMD_PREV TO pitch_cmd.
  SET SHIP:CONTROL:PITCH TO pitch_cmd.
  SET TELEM_RO_PITCH_TGT TO ROLLOUT_PITCH_TGT_DEG.
  SET TELEM_RO_PITCH_ERR TO pitch_err.
  SET TELEM_RO_PITCH_FF  TO pitch_cmd_ff.

  // Roll assist: keep wings level during rollout (also speed-scaled).
  // If AA FBW is active, do not inject roll input from IFC to avoid
  // controller fighting and rollout oscillation.
  LOCAL bank IS VECTORANGLE(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR) - 90.
  LOCAL roll_cmd IS 0.
  LOCAL use_ifc_roll_assist IS NOT (AA_AVAILABLE AND AA_FBW_ON).
  SET TELEM_RO_ROLL_ASSIST TO 0.
  IF use_ifc_roll_assist AND ias <= roll_assist_ias {
    LOCAL roll_span IS MAX(roll_assist_ias - ROLLOUT_STEER_FULL_IAS, 1).
    LOCAL roll_scale IS CLAMP((roll_assist_ias - ias) / roll_span, 0, 1).
    SET roll_cmd TO (-bank * KP_ROLLOUT_ROLL) * roll_scale.
    SET TELEM_RO_ROLL_ASSIST TO 1.
  }
  SET SHIP:CONTROL:ROLL TO roll_cmd.

  IF ias < ROLLOUT_DONE_IAS {
    BRAKES ON.
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    SET ROLLOUT_YAW_CMD_PREV        TO 0.
    SET ROLLOUT_PITCH_CMD_PREV      TO 0.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
    SET BOUNCE_RECOVERY_START_UT    TO -1.
    SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
    SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
    UNLOCK WHEELSTEERING.
    UNLOCK THROTTLE.
    SET_PHASE(PHASE_DONE).
  }
}
