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

// ─────────────────────────────────────────────────────────
// FLARE
// Sink-rate targeting with AoA ceiling.
//
// As AGL decreases from FLARE_ENTRY_AGL toward 0, the target
// vertical speed is linearly interpolated from the entry sink
// rate to TOUCHDOWN_VS (-0.3 m/s).  That target VS is
// converted to a flight path angle and fed to AA Director.
// If FAR AoA approaches MAX_FLARE_AOA the pitch-up is frozen.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLARE {
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE TO 0.
  SET TELEM_RO_YAW_GATE TO 0.
  SET TELEM_RO_PITCH_TGT TO 0.
  SET TELEM_RO_PITCH_ERR TO 0.
  SET TELEM_RO_PITCH_FF TO 0.
  LOCAL ac  IS ACTIVE_AIRCRAFT.
  LOCAL agl IS GET_AGL().
  LOCAL ias IS MAX(GET_IAS(), 10).  // floor prevents divide-by-zero at very low speed
  LOCAL vs_now IS SHIP:VERTICALSPEED.

  // Per-aircraft flare overrides (optional).
  LOCAL flare_touchdown_vs   IS TOUCHDOWN_VS.
  LOCAL flare_vs_gain        IS FLARE_IAS_TO_VS_GAIN.
  LOCAL flare_balloon_vs     IS FLARE_BALLOON_VS_TRIGGER.
  LOCAL flare_balloon_push   IS FLARE_BALLOON_FPA_PUSH.
  LOCAL flare_rate_min       IS FLARE_PITCH_RATE_MIN.
  LOCAL flare_rate_max       IS FLARE_PITCH_RATE_MAX.
  LOCAL flare_roundout_agl   IS FLARE_ROUNDOUT_AGL_M.
  LOCAL flare_roundout_strength IS FLARE_ROUNDOUT_STRENGTH.
  IF ac:HASKEY("flare_touchdown_vs") AND ac["flare_touchdown_vs"] <> -1 AND ac["flare_touchdown_vs"] < 0 {
    SET flare_touchdown_vs TO ac["flare_touchdown_vs"].
  }
  IF ac:HASKEY("flare_ias_to_vs_gain") AND ac["flare_ias_to_vs_gain"] >= 0 {
    SET flare_vs_gain TO ac["flare_ias_to_vs_gain"].
  }
  IF ac:HASKEY("flare_balloon_vs_trigger") AND ac["flare_balloon_vs_trigger"] >= 0 {
    SET flare_balloon_vs TO ac["flare_balloon_vs_trigger"].
  }
  IF ac:HASKEY("flare_balloon_fpa_push") AND ac["flare_balloon_fpa_push"] >= 0 {
    SET flare_balloon_push TO ac["flare_balloon_fpa_push"].
  }
  IF ac:HASKEY("flare_pitch_rate_min") AND ac["flare_pitch_rate_min"] > 0 {
    SET flare_rate_min TO ac["flare_pitch_rate_min"].
  }
  IF ac:HASKEY("flare_pitch_rate_max") AND ac["flare_pitch_rate_max"] > 0 {
    SET flare_rate_max TO ac["flare_pitch_rate_max"].
  }
  IF ac:HASKEY("flare_roundout_agl") AND ac["flare_roundout_agl"] >= 0 {
    SET flare_roundout_agl TO ac["flare_roundout_agl"].
  }
  IF ac:HASKEY("flare_roundout_strength") AND ac["flare_roundout_strength"] >= 0 {
    SET flare_roundout_strength TO CLAMP(ac["flare_roundout_strength"], 0, 1).
  }
  IF flare_rate_max < flare_rate_min { SET flare_rate_max TO flare_rate_min. }

  // Progress: 0 = just entered flare, 1 = at ground level.
  LOCAL frac IS CLAMP(1 - agl / FLARE_ENTRY_AGL, 0, 1).

  // Interpolate target sink rate from entry VS down to touchdown target.
  LOCAL base_vs IS FLARE_ENTRY_VS + (flare_touchdown_vs - FLARE_ENTRY_VS) * frac.

  // Energy compensation: if still fast versus Vref, command extra sink.
  LOCAL vref IS ACTIVE_V_APP - 10.
  IF ac:HASKEY("v_ref") { SET vref TO ac["v_ref"]. }
  LOCAL speed_excess IS MAX(ias - vref, 0).
  // Fade speed-driven extra sink out as we approach touchdown:
  // early flare can still shed energy, but the last part prioritizes a soft touchdown.
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
  IF vs_now > flare_balloon_vs {
    SET tgt_fpa TO MIN(tgt_fpa, FLARE_PITCH_CMD - flare_balloon_push).
  }

  // AoA ceiling: if FAR is available and AoA is near the limit,
  // stop pitching up (freeze tgt_fpa at current commanded value).
  IF FAR_AVAILABLE {
    LOCAL aoa IS GET_AOA().
    IF aoa > MAX_FLARE_AOA * 0.85 {
      SET tgt_fpa TO MIN(tgt_fpa, FLARE_PITCH_CMD).
    }
  }

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

  // Idle throttle during flare.
  SET THROTTLE_CMD TO 0.

  // Transition: confirmed touchdown (debounced).
  LOCAL td_fallback IS agl < TOUCHDOWN_FALLBACK_AGL_M AND vs_now <= TOUCHDOWN_FALLBACK_MAX_VS.
  IF SHIP:STATUS = "LANDED" OR td_fallback {
    IF TOUCHDOWN_CANDIDATE_UT < 0 { SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - TOUCHDOWN_CANDIDATE_UT >= TOUCHDOWN_CONFIRM_S {
      // Capture pre-impact pitch while still in flare; this is a better
      // nose-hold reference than waiting until touchdown transients begin.
      SET TOUCHDOWN_CAPTURE_PITCH_DEG TO GET_PITCH().
      SET TOUCHDOWN_INIT_DONE TO FALSE.
      SET TOUCHDOWN_CANDIDATE_UT TO -1.
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
  SET TELEM_RO_YAW_SCALE TO 0.
  SET TELEM_RO_YAW_GATE TO 0.
  SET THROTTLE_CMD TO 0.
  LOCAL nose_hold_cmd IS ROLLOUT_NOSE_HOLD_CMD.
  LOCAL nose_min_ref_deg IS ROLLOUT_NOSE_MIN_REF_DEG.
  LOCAL rollout_pitch_hold_kp IS ROLLOUT_PITCH_HOLD_KP.
  LOCAL rollout_pitch_max_cmd IS ROLLOUT_PITCH_MAX_CMD.
  LOCAL rollout_pitch_slew_per_s IS ROLLOUT_PITCH_SLEW_PER_S.
  LOCAL touchdown_settle_s IS TOUCHDOWN_SETTLE_S.
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_hold_cmd") {
    SET nose_hold_cmd TO ACTIVE_AIRCRAFT["rollout_nose_hold_cmd"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_min_ref_deg") AND ACTIVE_AIRCRAFT["rollout_nose_min_ref_deg"] >= 0 {
    SET nose_min_ref_deg TO ACTIVE_AIRCRAFT["rollout_nose_min_ref_deg"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_hold_kp") AND ACTIVE_AIRCRAFT["rollout_pitch_hold_kp"] >= 0 {
    SET rollout_pitch_hold_kp TO ACTIVE_AIRCRAFT["rollout_pitch_hold_kp"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_max_cmd") AND ACTIVE_AIRCRAFT["rollout_pitch_max_cmd"] > 0 {
    SET rollout_pitch_max_cmd TO CLAMP(ACTIVE_AIRCRAFT["rollout_pitch_max_cmd"], 0, 1).
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_slew_per_s") AND ACTIVE_AIRCRAFT["rollout_pitch_slew_per_s"] > 0 {
    SET rollout_pitch_slew_per_s TO ACTIVE_AIRCRAFT["rollout_pitch_slew_per_s"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_touchdown_settle_s") AND ACTIVE_AIRCRAFT["rollout_touchdown_settle_s"] > 0 {
    SET touchdown_settle_s TO ACTIVE_AIRCRAFT["rollout_touchdown_settle_s"].
  }

  // One-time touchdown handoff work.
  IF NOT TOUCHDOWN_INIT_DONE {
    // Spoilers (action group from aircraft config).
    LOCAL ag_sp IS ACTIVE_AIRCRAFT["ag_spoilers"].
    IF ag_sp > 0 { TRIGGER_AG(ag_sp, TRUE). }

    // Reverse thrust (action group from aircraft config).
    LOCAL ag_tr IS ACTIVE_AIRCRAFT["ag_thrust_rev"].
    IF ag_tr > 0 { TRIGGER_AG(ag_tr, TRUE). }

    // Drogue chute (action group from aircraft config).
    LOCAL ag_dr IS ACTIVE_AIRCRAFT["ag_drogue"].
    IF ag_dr > 0 { TRIGGER_AG(ag_dr, TRUE). }

    BRAKES OFF.

    // Keep AA in standard FBW mode for rollout damping.
    AA_RESTORE_FBW().
    UNLOCK STEERING.

    // Clear accumulated control inputs from the flare.
    SET SHIP:CONTROL:YAW   TO 0.
    SET SHIP:CONTROL:ROLL  TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    SET ROLLOUT_YAW_CMD_PREV TO 0.
    SET ROLLOUT_PITCH_CMD_PREV TO 0.
    LOCAL captured_pitch IS TOUCHDOWN_CAPTURE_PITCH_DEG.
    IF captured_pitch = 0 { SET captured_pitch TO GET_PITCH(). }
    SET ROLLOUT_PITCH_REF_DEG TO MAX(captured_pitch, nose_min_ref_deg).
    SET ROLLOUT_PITCH_TGT_DEG TO ROLLOUT_PITCH_REF_DEG.

    // Capture touchdown heading and start wheelsteering from that heading.
    // We'll blend from touchdown heading -> runway heading as speed decays.
    SET ROLLOUT_ENTRY_HDG TO GET_COMPASS_HDG().
    LOCK WHEELSTEERING TO ROLLOUT_ENTRY_HDG.
    PRINT "  ROLLOUT armed: hdg " + ROUND(ROLLOUT_ENTRY_HDG, 1)
        + " deg, IAS " + ROUND(GET_IAS(), 1) + " m/s"
        + ", pitch_ref " + ROUND(ROLLOUT_PITCH_REF_DEG, 2) + " deg".

    SET TOUCHDOWN_INIT_DONE TO TRUE.
  }

  // Closed-loop touchdown pitch hold:
  // hold sampled pitch attitude with a small feedforward up-command.
  LOCAL pitch_now IS GET_PITCH().
  LOCAL pitch_err IS ROLLOUT_PITCH_REF_DEG - pitch_now.
  LOCAL td_pitch_ff IS nose_hold_cmd.
  LOCAL td_pitch_target IS td_pitch_ff + pitch_err * rollout_pitch_hold_kp.
  SET td_pitch_target TO CLAMP(td_pitch_target, -rollout_pitch_max_cmd, rollout_pitch_max_cmd).
  LOCAL td_pitch_cmd IS MOVE_TOWARD(
    ROLLOUT_PITCH_CMD_PREV,
    td_pitch_target,
    rollout_pitch_slew_per_s * IFC_ACTUAL_DT
  ).
  SET ROLLOUT_PITCH_CMD_PREV TO td_pitch_cmd.
  SET SHIP:CONTROL:PITCH TO td_pitch_cmd.
  SET TELEM_RO_PITCH_TGT TO ROLLOUT_PITCH_REF_DEG.
  SET TELEM_RO_PITCH_ERR TO pitch_err.
  SET TELEM_RO_PITCH_FF TO td_pitch_ff.

  // If we were not really on the wheels, return to flare.
  LOCAL agl IS GET_AGL().
  IF SHIP:STATUS <> "LANDED" AND agl > TOUCHDOWN_FALLBACK_AGL_M {
    BRAKES OFF.
    UNLOCK WHEELSTEERING.
    SET TOUCHDOWN_INIT_DONE TO FALSE.
    SET TOUCHDOWN_CANDIDATE_UT TO -1.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
    SET ROLLOUT_PITCH_CMD_PREV TO 0.
    SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
    SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
    // Clear any leftover manual inputs before re-entering flare director mode.
    SET SHIP:CONTROL:YAW TO 0.
    SET SHIP:CONTROL:ROLL TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    LOCAL entry_ias IS MAX(GET_IAS(), 10).
    SET FLARE_PITCH_CMD TO ARCTAN(SHIP:VERTICALSPEED / entry_ias).
    SET FLARE_ENTRY_VS  TO CLAMP(SHIP:VERTICALSPEED, FLARE_MIN_ENTRY_SINK_VS, -0.05).
    SET FLARE_ENTRY_AGL TO MAX(agl, 1).
    SET_PHASE(PHASE_FLARE).
    RETURN.
  }

  // Hold briefly in TOUCHDOWN to absorb contact transients before rollout.
  IF PHASE_ELAPSED() >= touchdown_settle_s {
    SET TOUCHDOWN_INIT_DONE TO FALSE.
    SET TOUCHDOWN_CANDIDATE_UT TO -1.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
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
  SET THROTTLE_CMD TO 0.
  LOCAL ias IS GET_IAS().
  LOCAL agl IS GET_AGL().
  LOCAL brake_max_ias IS ROLLOUT_BRAKE_MAX_IAS.
  LOCAL yaw_assist_ias IS ROLLOUT_YAW_ASSIST_IAS.
  LOCAL roll_assist_ias IS ROLLOUT_ROLL_ASSIST_IAS.
  LOCAL steer_min_blend IS ROLLOUT_STEER_MIN_BLEND.
  LOCAL rollout_yaw_kp IS KP_ROLLOUT_YAW.
  LOCAL rollout_yaw_slew_per_s IS ROLLOUT_YAW_SLEW_PER_S.
  LOCAL rollout_yaw_fade_ias IS ROLLOUT_YAW_FADE_IAS.
  LOCAL rollout_yaw_max_cmd IS ROLLOUT_YAW_MAX_CMD.
  LOCAL nose_hold_cmd IS ROLLOUT_NOSE_HOLD_CMD.
  LOCAL nose_release_ias IS ROLLOUT_NOSE_RELEASE_IAS.
  LOCAL nose_hold_min_s IS ROLLOUT_NOSE_HOLD_MIN_S.
  LOCAL rollout_nose_target_pitch_deg IS ROLLOUT_NOSE_TARGET_PITCH_DEG.
  LOCAL rollout_nose_target_slew_dps IS ROLLOUT_NOSE_TARGET_SLEW_DPS.
  LOCAL rollout_pitch_hold_kp IS ROLLOUT_PITCH_HOLD_KP.
  LOCAL rollout_pitch_max_cmd IS ROLLOUT_PITCH_MAX_CMD.
  LOCAL rollout_pitch_slew_per_s IS ROLLOUT_PITCH_SLEW_PER_S.
  LOCAL yaw_sign IS -1.
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_brake_max_ias") AND ACTIVE_AIRCRAFT["rollout_brake_max_ias"] > 0 {
    SET brake_max_ias TO ACTIVE_AIRCRAFT["rollout_brake_max_ias"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_assist_ias") AND ACTIVE_AIRCRAFT["rollout_yaw_assist_ias"] > 0 {
    SET yaw_assist_ias TO ACTIVE_AIRCRAFT["rollout_yaw_assist_ias"].
  }
  // Allow explicit 0 from aircraft config to disable IFC roll assist.
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_roll_assist_ias") AND ACTIVE_AIRCRAFT["rollout_roll_assist_ias"] >= 0 {
    SET roll_assist_ias TO ACTIVE_AIRCRAFT["rollout_roll_assist_ias"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_steer_min_blend") AND ACTIVE_AIRCRAFT["rollout_steer_min_blend"] >= 0 {
    SET steer_min_blend TO CLAMP(ACTIVE_AIRCRAFT["rollout_steer_min_blend"], 0, 1).
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_kp") AND ACTIVE_AIRCRAFT["rollout_yaw_kp"] >= 0 {
    SET rollout_yaw_kp TO ACTIVE_AIRCRAFT["rollout_yaw_kp"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_slew_per_s") AND ACTIVE_AIRCRAFT["rollout_yaw_slew_per_s"] >= 0 {
    SET rollout_yaw_slew_per_s TO ACTIVE_AIRCRAFT["rollout_yaw_slew_per_s"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_fade_ias") AND ACTIVE_AIRCRAFT["rollout_yaw_fade_ias"] > 0 {
    SET rollout_yaw_fade_ias TO ACTIVE_AIRCRAFT["rollout_yaw_fade_ias"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_max_cmd") AND ACTIVE_AIRCRAFT["rollout_yaw_max_cmd"] >= 0 {
    SET rollout_yaw_max_cmd TO CLAMP(ACTIVE_AIRCRAFT["rollout_yaw_max_cmd"], 0, 1).
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_hold_cmd") {
    SET nose_hold_cmd TO ACTIVE_AIRCRAFT["rollout_nose_hold_cmd"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_release_ias") AND ACTIVE_AIRCRAFT["rollout_nose_release_ias"] > 0 {
    SET nose_release_ias TO ACTIVE_AIRCRAFT["rollout_nose_release_ias"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_hold_min_s") AND ACTIVE_AIRCRAFT["rollout_nose_hold_min_s"] >= 0 {
    SET nose_hold_min_s TO ACTIVE_AIRCRAFT["rollout_nose_hold_min_s"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_target_pitch_deg") {
    SET rollout_nose_target_pitch_deg TO ACTIVE_AIRCRAFT["rollout_nose_target_pitch_deg"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_nose_target_slew_dps") AND ACTIVE_AIRCRAFT["rollout_nose_target_slew_dps"] > 0 {
    SET rollout_nose_target_slew_dps TO ACTIVE_AIRCRAFT["rollout_nose_target_slew_dps"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_hold_kp") AND ACTIVE_AIRCRAFT["rollout_pitch_hold_kp"] >= 0 {
    SET rollout_pitch_hold_kp TO ACTIVE_AIRCRAFT["rollout_pitch_hold_kp"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_max_cmd") AND ACTIVE_AIRCRAFT["rollout_pitch_max_cmd"] > 0 {
    SET rollout_pitch_max_cmd TO CLAMP(ACTIVE_AIRCRAFT["rollout_pitch_max_cmd"], 0, 1).
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_pitch_slew_per_s") AND ACTIVE_AIRCRAFT["rollout_pitch_slew_per_s"] > 0 {
    SET rollout_pitch_slew_per_s TO ACTIVE_AIRCRAFT["rollout_pitch_slew_per_s"].
  }
  IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_sign") AND ACTIVE_AIRCRAFT["rollout_yaw_sign"] <> 0 {
    SET yaw_sign TO ACTIVE_AIRCRAFT["rollout_yaw_sign"].
  }

  // Bounce recovery: if we lift off again shortly after touchdown, return to flare logic.
  IF PHASE_ELAPSED() < BOUNCE_RECOVERY_MAX_S AND SHIP:STATUS <> "LANDED" AND agl > BOUNCE_RECOVERY_AGL_M {
    BRAKES OFF.
    UNLOCK WHEELSTEERING.
    SET TOUCHDOWN_CANDIDATE_UT TO -1.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
    SET ROLLOUT_PITCH_CMD_PREV TO 0.
    SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
    SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
    // Clear any leftover manual inputs before re-entering flare director mode.
    SET SHIP:CONTROL:YAW TO 0.
    SET SHIP:CONTROL:ROLL TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    LOCAL entry_ias IS MAX(GET_IAS(), 10).
    SET FLARE_PITCH_CMD TO ARCTAN(SHIP:VERTICALSPEED / entry_ias).
    SET FLARE_ENTRY_VS  TO CLAMP(SHIP:VERTICALSPEED, FLARE_MIN_ENTRY_SINK_VS, -0.05).
    SET FLARE_ENTRY_AGL TO MAX(agl, 1).
    PRINT "  BOUNCE recovery: returning to FLARE (AGL " + ROUND(agl, 1) + " m)".
    SET_PHASE(PHASE_FLARE).
    RETURN.
  }

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
  IF PHASE_ELAPSED() > ROLLOUT_BRAKE_DELAY_S AND ias <= brake_max_ias {
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
  SET TELEM_RO_YAW_GATE TO yaw_gate.
  SET TELEM_RO_YAW_TGT TO yaw_cmd_target.
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
    SET desired_pitch_deg TO rollout_nose_target_pitch_deg.
  }
  SET ROLLOUT_PITCH_TGT_DEG TO MOVE_TOWARD(
    ROLLOUT_PITCH_TGT_DEG,
    desired_pitch_deg,
    rollout_nose_target_slew_dps * IFC_ACTUAL_DT
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
  SET pitch_cmd_target TO CLAMP(pitch_cmd_target, -rollout_pitch_max_cmd, rollout_pitch_max_cmd).
  LOCAL pitch_cmd IS MOVE_TOWARD(
    ROLLOUT_PITCH_CMD_PREV,
    pitch_cmd_target,
    rollout_pitch_slew_per_s * IFC_ACTUAL_DT
  ).
  SET ROLLOUT_PITCH_CMD_PREV TO pitch_cmd.
  SET SHIP:CONTROL:PITCH TO pitch_cmd.
  SET TELEM_RO_PITCH_TGT TO ROLLOUT_PITCH_TGT_DEG.
  SET TELEM_RO_PITCH_ERR TO pitch_err.
  SET TELEM_RO_PITCH_FF TO pitch_cmd_ff.

  // Roll assist: keep wings level during rollout (also speed-scaled).
  // If AA FBW is active, do not inject roll input from IFC to avoid
  // controller fighting and rollout oscillation.
  LOCAL bank IS 90 - VECTORANGLE(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR).
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
    SET ROLLOUT_YAW_CMD_PREV TO 0.
    SET ROLLOUT_PITCH_CMD_PREV TO 0.
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
    SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
    SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
    UNLOCK WHEELSTEERING.
    UNLOCK THROTTLE.
    SET_PHASE(PHASE_DONE).
  }
}
