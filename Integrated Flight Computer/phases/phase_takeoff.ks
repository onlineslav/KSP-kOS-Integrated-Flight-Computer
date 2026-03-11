@LAZYGLOBAL OFF.

// ============================================================
// phase_takeoff.ks  -  Integrated Flight Computer
//
// TAKEOFF phase state machine:
//   TO_PREFLIGHT   - one-time setup, auto-stage, thrust-ready gate
//   TO_GROUND_ROLL - throttle up, centerline hold with wheel + rudder assist
//   TO_ROTATE      - pitch ramp at V_R with debounced liftoff detection
//   TO_CLIMB       - climb-out on runway centerline until done AGL
// ============================================================

FUNCTION RUN_TAKEOFF {
  IF IFC_SUBPHASE = SUBPHASE_TO_PREFLIGHT {
    _RUN_TO_PREFLIGHT().
  } ELSE IF IFC_SUBPHASE = SUBPHASE_TO_GROUND_ROLL {
    _CHECK_TAKEOFF_FLAPS().
    _RUN_TO_GROUND_ROLL().
  } ELSE IF IFC_SUBPHASE = SUBPHASE_TO_ROTATE {
    _CHECK_TAKEOFF_FLAPS().
    _RUN_TO_ROTATE().
  } ELSE IF IFC_SUBPHASE = SUBPHASE_TO_CLIMB {
    _CHECK_TAKEOFF_FLAPS().
    _RUN_TO_CLIMB().
  }
}

FUNCTION _TO_GET_YAW_SIGN {
  LOCAL sign IS -1.
  IF ACTIVE_AIRCRAFT <> 0 {
    IF ACTIVE_AIRCRAFT:HASKEY("takeoff_yaw_sign") AND ACTIVE_AIRCRAFT["takeoff_yaw_sign"] <> 0 {
      SET sign TO ACTIVE_AIRCRAFT["takeoff_yaw_sign"].
    } ELSE IF ACTIVE_AIRCRAFT:HASKEY("rollout_yaw_sign") AND ACTIVE_AIRCRAFT["rollout_yaw_sign"] <> 0 {
      SET sign TO ACTIVE_AIRCRAFT["rollout_yaw_sign"].
    }
  }
  RETURN sign.
}

FUNCTION _TO_COMPUTE_LOC_DEV_M {
  IF ACTIVE_ILS_ID = "" { RETURN 0. }
  LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).
  IF NOT ils:HASKEY("ll") OR NOT ils:HASKEY("alt_asl") { RETURN 0. }

  LOCAL thr_pos   IS ils["ll"]:ALTITUDEPOSITION(ils["alt_asl"]).
  LOCAL rwy_right IS HEADING(TO_RWY_HDG + 90, 0):FOREVECTOR.
  LOCAL disp      IS SHIP:POSITION - thr_pos.
  RETURN VDOT(disp, rwy_right). // + = right of centerline
}

FUNCTION _TO_COMPUTE_STEER_HDG {
  PARAMETER max_corr_deg.
  LOCAL kp_loc    IS AC_PARAM("takeoff_loc_kp", KP_TAKEOFF_LOC, 0).
  LOCAL loc_guard IS AC_PARAM("takeoff_loc_guard_m", TAKEOFF_LOC_GUARD_M, 0.001).
  LOCAL loc_m     IS _TO_COMPUTE_LOC_DEV_M().

  // Guard against bad geometry spikes when craft is not on/near runway centerline.
  IF loc_m > loc_guard { SET loc_m TO loc_guard. }
  IF loc_m < -loc_guard { SET loc_m TO -loc_guard. }

  LOCAL loc_corr IS CLAMP(-loc_m * kp_loc, -max_corr_deg, max_corr_deg).
  SET TELEM_RO_LOC_CORR TO loc_corr.
  SET TELEM_LOC_CORR TO loc_corr.
  SET TELEM_GS_CORR TO 0.
  RETURN WRAP_360(TO_RWY_HDG + loc_corr).
}

FUNCTION _TO_COMPUTE_CLIMB_FPA_TGT {
  PARAMETER ias.
  LOCAL v2 IS AC_PARAM("v2", TAKEOFF_V2_DEFAULT, 0.001).
  LOCAL spd_err IS v2 - ias.

  LOCAL climb_fpa_base IS AC_PARAM("takeoff_climb_fpa", TAKEOFF_CLIMB_FPA, 0.001).
  LOCAL fpa_spd_gain IS AC_PARAM("takeoff_climb_fpa_spd_gain", TAKEOFF_CLIMB_FPA_SPD_GAIN, 0).
  LOCAL fpa_min IS AC_PARAM("takeoff_climb_fpa_min", TAKEOFF_CLIMB_FPA_MIN, 0.001).

  LOCAL climb_fpa_tgt IS climb_fpa_base.
  IF spd_err > 0 {
    SET climb_fpa_tgt TO MAX(climb_fpa_base - spd_err * fpa_spd_gain, fpa_min).
  }

  // AoA guard for early climb: if AoA approaches a_crit, pull FPA down.
  LOCAL aoa_crit IS -1.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("a_crit")
      AND ACTIVE_AIRCRAFT["a_crit"] > 0 {
    SET aoa_crit TO ACTIVE_AIRCRAFT["a_crit"].
  }
  IF aoa_crit > 0 {
    LOCAL aoa_warn_frac IS CLAMP(AC_PARAM("takeoff_aoa_protect_frac", 0.85, 0.01), 0.10, 0.99).
    LOCAL aoa_fpa_gain  IS AC_PARAM("takeoff_aoa_fpa_gain", 0.90, 0).
    LOCAL aoa_warn IS aoa_crit * aoa_warn_frac.
    LOCAL aoa_now IS GET_AOA().
    IF aoa_now > aoa_warn {
      LOCAL aoa_over IS aoa_now - aoa_warn.
      SET climb_fpa_tgt TO MAX(climb_fpa_tgt - aoa_over * aoa_fpa_gain, fpa_min).
    }
  }
  RETURN climb_fpa_tgt.
}

FUNCTION _TO_RUN_GROUND_CENTERLINE {
  PARAMETER steer_max_corr.

  LOCAL hdg_now IS GET_COMPASS_HDG().
  LOCAL hdg_rate IS WRAP_180(hdg_now - TO_HDG_PREV) / MAX(IFC_ACTUAL_DT, 0.01).
  SET TO_HDG_PREV TO hdg_now.

  LOCAL steer_hdg IS _TO_COMPUTE_STEER_HDG(steer_max_corr).
  LOCAL steer_rate_kd IS AC_PARAM("takeoff_steer_hdg_rate_kd", TAKEOFF_STEER_HDG_RATE_KD, 0).
  IF steer_rate_kd > 0 {
    LOCAL steer_corr IS WRAP_180(steer_hdg - TO_RWY_HDG) - hdg_rate * steer_rate_kd.
    SET steer_corr TO CLAMP(steer_corr, -steer_max_corr, steer_max_corr).
    SET steer_hdg TO WRAP_360(TO_RWY_HDG + steer_corr).
  }
  SET ROLLOUT_STEER_HDG TO steer_hdg.
  LOCK WHEELSTEERING TO steer_hdg.
  SET TELEM_STEER_BLEND TO 1.

  LOCAL hdg_err IS WRAP_180(hdg_now - steer_hdg).
  SET TELEM_RO_HDG_ERR TO hdg_err.

  LOCAL ias IS GET_IAS().
  LOCAL yaw_start IS AC_PARAM("takeoff_yaw_start_ias", TAKEOFF_YAW_START_IAS, 0).
  LOCAL yaw_full  IS AC_PARAM("takeoff_yaw_full_ias", TAKEOFF_YAW_FULL_IAS, 0.001).
  LOCAL yaw_min_scale IS CLAMP(AC_PARAM("takeoff_yaw_min_scale", TAKEOFF_YAW_MIN_SCALE, 0), 0, 1).
  LOCAL yaw_kp    IS AC_PARAM("takeoff_yaw_kp", KP_TAKEOFF_YAW, 0).
  LOCAL yaw_kd    IS AC_PARAM("takeoff_yaw_kd", KD_TAKEOFF_YAW, 0).
  LOCAL yaw_boost_err_deg IS AC_PARAM("takeoff_yaw_boost_err_deg", TAKEOFF_YAW_BOOST_ERR_DEG, 0.001).
  LOCAL yaw_boost_max IS AC_PARAM("takeoff_yaw_boost_max", TAKEOFF_YAW_BOOST_MAX, 0).
  LOCAL yaw_max   IS MIN(AC_PARAM("takeoff_yaw_max_cmd", TAKEOFF_YAW_MAX_CMD, 0), 1).
  LOCAL yaw_slew  IS AC_PARAM("takeoff_yaw_slew_per_s", TAKEOFF_YAW_SLEW_PER_S, 0.001).
  LOCAL yaw_sign  IS _TO_GET_YAW_SIGN().

  LOCAL yaw_gate IS 0.
  LOCAL yaw_scale IS 0.
  LOCAL yaw_tgt IS 0.
  IF SHIP:STATUS = "LANDED" {
    IF ABS(hdg_err) <= ROLLOUT_YAW_ERR_GUARD_DEG {
      LOCAL yaw_span IS MAX(yaw_full - yaw_start, 1).
      IF ias >= yaw_start {
        LOCAL yaw_ramp IS CLAMP((ias - yaw_start) / yaw_span, 0, 1).
        SET yaw_scale TO yaw_min_scale + (1 - yaw_min_scale) * yaw_ramp.
      } ELSE {
        SET yaw_gate TO 1.
        // Keep some rudder authority available from rollout start.
        SET yaw_scale TO yaw_min_scale.
      }
      LOCAL yaw_gain_scale IS 1.
      IF yaw_boost_max > 0 AND yaw_boost_err_deg > 0 {
        SET yaw_gain_scale TO 1 + MIN(ABS(hdg_err) / yaw_boost_err_deg, yaw_boost_max).
      }
      LOCAL yaw_err IS (hdg_err * yaw_kp + hdg_rate * yaw_kd) * yaw_gain_scale.
      SET yaw_tgt TO (yaw_err * yaw_sign) * yaw_scale.
    } ELSE {
      SET yaw_gate TO 2.
    }
  } ELSE {
    SET yaw_gate TO 3.
  }
  SET yaw_tgt TO CLAMP(yaw_tgt, -yaw_max, yaw_max).
  LOCAL yaw_cmd IS MOVE_TOWARD(
    TO_YAW_CMD_PREV,
    yaw_tgt,
    yaw_slew * IFC_ACTUAL_DT
  ).
  SET TO_YAW_CMD_PREV TO yaw_cmd.
  SET SHIP:CONTROL:YAW TO yaw_cmd.
  SET SHIP:CONTROL:ROLL TO 0.
  SET SHIP:CONTROL:PITCH TO 0.

  SET TELEM_RO_YAW_SCALE TO yaw_scale.
  SET TELEM_RO_YAW_GATE  TO yaw_gate.
  SET TELEM_RO_YAW_TGT   TO yaw_tgt.
}

FUNCTION _TO_TRY_AUTO_STAGE {
  LOCAL auto_stage IS AC_PARAM("takeoff_autostage", 1, 0).
  IF auto_stage <= 0 { RETURN. }
  IF GET_IAS() > 5 { RETURN. } // never auto-stage once already accelerating

  LOCAL min_avail IS AC_PARAM("takeoff_min_avail_thrust", TAKEOFF_MIN_AVAIL_THRUST, 0).
  IF SHIP:AVAILABLETHRUST >= min_avail { RETURN. }

  LOCAL max_attempts IS ROUND(AC_PARAM("takeoff_stage_max_attempts", TAKEOFF_STAGE_MAX_ATTEMPTS, 0)).
  IF TO_STAGE_ATTEMPTS >= max_attempts { RETURN. }

  LOCAL retry_s IS AC_PARAM("takeoff_stage_retry_s", TAKEOFF_STAGE_RETRY_S, 0).
  IF TIME:SECONDS - TO_LAST_STAGE_UT < retry_s { RETURN. }

  STAGE.
  SET TO_STAGE_ATTEMPTS TO TO_STAGE_ATTEMPTS + 1.
  SET TO_LAST_STAGE_UT TO TIME:SECONDS.
  SET IFC_ALERT_TEXT TO "TO autostage #" + TO_STAGE_ATTEMPTS
    + "  avail " + ROUND(SHIP:AVAILABLETHRUST, 1) + " kN".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION _RUN_TO_PREFLIGHT {
  LOCAL first_tick IS PHASE_ELAPSED() <= IFC_ACTUAL_DT * 1.5.
  LOCAL det_to IS ROUND(AC_PARAM("flaps_detent_takeoff", AC_PARAM("flaps_detent_approach", 2, 0), 0)).
  SET FLAPS_TARGET_DETENT TO det_to.
  _CHECK_TAKEOFF_FLAPS().

  BRAKES ON.
  GEAR ON.
  // One-time preflight setup. Keep this out of the per-cycle path so
  // throttle can spool up while brakes remain engaged.
  IF first_tick {
    IF DEFINED CAMERA {
      SET CAMERA:MODE TO "LOCKED".
    }
    SET THROTTLE_CMD TO 0.
    SET SHIP:CONTROL:YAW TO 0.
    SET SHIP:CONTROL:ROLL TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    SET TO_YAW_CMD_PREV TO 0.
    SET TO_PITCH_CMD_PREV TO 0.
    SET TO_HDG_PREV TO GET_COMPASS_HDG().
    SET TO_CLIMB_FPA_CMD TO 0.
    SET TO_SPOOL_PREV_AVAIL TO SHIP:AVAILABLETHRUST.
    SET TO_SPOOL_STABLE_UT TO -1.
    LOCK WHEELSTEERING TO TO_RWY_HDG.

    SET TELEM_AA_HDG_CMD TO TO_RWY_HDG.
    SET TELEM_AA_FPA_CMD TO 0.
    SET TELEM_RO_YAW_TGT TO 0.
    SET TELEM_RO_YAW_SCALE TO 0.
    SET TELEM_RO_YAW_GATE TO 1.
    SET TELEM_RO_HDG_ERR TO WRAP_180(GET_COMPASS_HDG() - TO_RWY_HDG).
    SET TELEM_RO_LOC_CORR TO 0.
    SET ROLLOUT_STEER_HDG TO TO_RWY_HDG.
    SET TELEM_STEER_BLEND TO 1.
  }

  _TO_TRY_AUTO_STAGE().

  LOCAL to_thr IS AC_PARAM("takeoff_throttle", 1.0, 0.001).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, to_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).

  IF PHASE_ELAPSED() < 2.0 { RETURN. }

  LOCAL min_avail IS AC_PARAM("takeoff_min_avail_thrust", TAKEOFF_MIN_AVAIL_THRUST, 0).
  LOCAL spool_timeout_s IS AC_PARAM("takeoff_engine_spool_timeout_s", TAKEOFF_ENGINE_SPOOL_TIMEOUT_S, 0).
  LOCAL spool_frac IS CLAMP(AC_PARAM("takeoff_spool_thrust_frac", TAKEOFF_SPOOL_THRUST_FRAC, 0), 0, 1).
  LOCAL steady_dknps IS AC_PARAM("takeoff_spool_steady_dknps", TAKEOFF_SPOOL_STEADY_DKNPS, 0).
  LOCAL steady_hold_s IS AC_PARAM("takeoff_spool_steady_hold_s", TAKEOFF_SPOOL_STEADY_HOLD_S, 0.001).
  LOCAL flap_settle_s IS AC_PARAM("takeoff_flap_settle_s", TAKEOFF_FLAP_SETTLE_S, 0).

  LOCAL req_avail IS min_avail.
  IF SHIP:MAXTHRUST > 0 {
    SET req_avail TO MAX(req_avail, SHIP:MAXTHRUST * spool_frac * to_thr).
  }

  LOCAL avail_now IS SHIP:AVAILABLETHRUST.
  LOCAL avail_rate IS ABS(avail_now - TO_SPOOL_PREV_AVAIL) / MAX(IFC_ACTUAL_DT, 0.01).
  SET TO_SPOOL_PREV_AVAIL TO avail_now.

  LOCAL throttle_ready IS to_thr <= 0 OR THROTTLE_CMD >= to_thr - 0.01.
  LOCAL thrust_stable_now IS avail_rate <= steady_dknps.
  IF throttle_ready AND thrust_stable_now {
    IF TO_SPOOL_STABLE_UT < 0 { SET TO_SPOOL_STABLE_UT TO TIME:SECONDS. }
  } ELSE {
    SET TO_SPOOL_STABLE_UT TO -1.
  }
  LOCAL thrust_stable IS TO_SPOOL_STABLE_UT >= 0 AND
                       TIME:SECONDS - TO_SPOOL_STABLE_UT >= steady_hold_s.
  LOCAL thrust_ready IS avail_now >= req_avail AND thrust_stable.
  LOCAL flaps_ready IS TRUE.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_up")
      AND ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_down")
      AND ACTIVE_AIRCRAFT["ag_flaps_step_up"] > 0
      AND ACTIVE_AIRCRAFT["ag_flaps_step_down"] > 0 {
    SET flaps_ready TO FLAPS_CURRENT_DETENT = FLAPS_TARGET_DETENT AND
      TIME:SECONDS - FLAPS_LAST_STEP_UT >= flap_settle_s.
  }

  IF ((throttle_ready AND thrust_ready) OR PHASE_ELAPSED() >= spool_timeout_s)
      AND flaps_ready {
    IF NOT thrust_ready {
      SET IFC_ALERT_TEXT TO "TO spool timeout: avail " + ROUND(avail_now, 1)
            + " kN (req " + ROUND(req_avail, 1) + ")".
      SET IFC_ALERT_UT   TO TIME:SECONDS.
    }
    SET THROTTLE_CMD TO to_thr.
    BRAKES OFF.
    SET_SUBPHASE(SUBPHASE_TO_GROUND_ROLL).
  }
}

FUNCTION _RUN_TO_GROUND_ROLL {
  _TO_TRY_AUTO_STAGE().

  LOCAL to_thr IS AC_PARAM("takeoff_throttle", 1.0, 0.001).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, to_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).

  LOCAL steer_max_corr IS AC_PARAM("takeoff_steer_max_corr", TAKEOFF_STEER_MAX_CORR, 0.001).
  _TO_RUN_GROUND_CENTERLINE(steer_max_corr).
  SET TELEM_AA_HDG_CMD TO ROLLOUT_STEER_HDG.
  SET TELEM_AA_FPA_CMD TO 0.

  LOCAL v_r IS AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001).
  SET ACTIVE_V_TGT TO v_r.
  IF GET_IAS() >= v_r {
    SET TO_ROTATE_PITCH_CMD TO GET_PITCH().
    SET TO_AIRBORNE_UT TO -1.
    SET TO_PITCH_CMD_PREV TO 0.
    SET TO_HDG_PREV TO GET_COMPASS_HDG().
    SET SHIP:CONTROL:YAW TO 0.
    SET TO_YAW_CMD_PREV TO 0.
    SET_SUBPHASE(SUBPHASE_TO_ROTATE).
  }
}

FUNCTION _RUN_TO_ROTATE {
  LOCAL to_thr IS AC_PARAM("takeoff_throttle", 1.0, 0.001).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, to_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).
  SET ACTIVE_V_TGT TO AC_PARAM("v2", TAKEOFF_V2_DEFAULT, 0.001).

  LOCAL pitch_tgt  IS AC_PARAM("takeoff_pitch_tgt", TAKEOFF_ROTATE_PITCH_TGT, 0.001).
  LOCAL pitch_slew IS AC_PARAM("takeoff_pitch_slew_dps", TAKEOFF_PITCH_SLEW_DPS, 0.001).
  SET TO_ROTATE_PITCH_CMD TO MOVE_TOWARD(
    TO_ROTATE_PITCH_CMD,
    pitch_tgt,
    pitch_slew * IFC_ACTUAL_DT
  ).

  LOCAL airborne_agl IS AC_PARAM("takeoff_airborne_agl", TAKEOFF_AIRBORNE_AGL_M, 0).
  LOCAL airborne_min_vs IS AC_PARAM("takeoff_airborne_min_vs", TAKEOFF_AIRBORNE_MIN_VS, 0).
  LOCAL agl_now IS GET_AGL().
  // Avoid control-mode thrash from LANDED/FLYING status flicker near liftoff.
  LOCAL on_ground_rotate IS agl_now <= airborne_agl + 1.

  IF on_ground_rotate {
    // Keep ground steering/yaw control active through rotation while on wheels.
    LOCAL steer_max_corr IS AC_PARAM("takeoff_steer_max_corr", TAKEOFF_STEER_MAX_CORR, 0.001).
    _TO_RUN_GROUND_CENTERLINE(steer_max_corr).

    // AA Director often under-commands pitch while still weight-on-wheels.
    // Use FBW damping plus direct pitch input until airborne.
    AA_RESTORE_FBW().
    LOCAL pitch_kp IS AC_PARAM("takeoff_rotate_pitch_kp", TAKEOFF_ROTATE_PITCH_KP, 0).
    LOCAL pitch_ff IS AC_PARAM("takeoff_rotate_pitch_ff", TAKEOFF_ROTATE_PITCH_FF, -0.5).
    LOCAL pitch_max_cmd IS MIN(AC_PARAM("takeoff_rotate_pitch_max_cmd", TAKEOFF_ROTATE_PITCH_MAX_CMD, 0), 1).
    LOCAL pitch_min_cmd IS MIN(AC_PARAM("takeoff_rotate_pitch_min_cmd", TAKEOFF_ROTATE_PITCH_MIN_CMD, 0), pitch_max_cmd).
    LOCAL pitch_cmd_slew IS AC_PARAM("takeoff_rotate_pitch_slew_per_s", TAKEOFF_ROTATE_PITCH_SLEW_PER_S, 0.001).
    LOCAL pitch_err IS TO_ROTATE_PITCH_CMD - GET_PITCH().
    LOCAL pitch_tgt_cmd IS pitch_ff + pitch_err * pitch_kp.
    IF pitch_err > 0 {
      SET pitch_tgt_cmd TO MAX(pitch_tgt_cmd, pitch_min_cmd).
    }
    SET pitch_tgt_cmd TO CLAMP(pitch_tgt_cmd, -0.10, pitch_max_cmd).
    LOCAL pitch_cmd IS MOVE_TOWARD(
      TO_PITCH_CMD_PREV,
      pitch_tgt_cmd,
      pitch_cmd_slew * IFC_ACTUAL_DT
    ).
    SET TO_PITCH_CMD_PREV TO pitch_cmd.
    SET SHIP:CONTROL:PITCH TO pitch_cmd.

    SET TELEM_AA_HDG_CMD TO ROLLOUT_STEER_HDG.
    SET TELEM_AA_FPA_CMD TO TO_ROTATE_PITCH_CMD.
    SET TELEM_RO_PITCH_TGT TO TO_ROTATE_PITCH_CMD.
    SET TELEM_RO_PITCH_ERR TO pitch_err.
    SET TELEM_RO_PITCH_FF TO pitch_ff.
  } ELSE {
    LOCAL hdg_cmd IS TO_RWY_HDG.
    AA_SET_DIRECTOR(hdg_cmd, TO_ROTATE_PITCH_CMD).
    SET TELEM_AA_HDG_CMD TO hdg_cmd.
    SET TELEM_AA_FPA_CMD TO TO_ROTATE_PITCH_CMD.

    SET SHIP:CONTROL:YAW TO 0.
    SET TO_YAW_CMD_PREV TO 0.
    SET SHIP:CONTROL:PITCH TO 0.
    SET TO_PITCH_CMD_PREV TO 0.
    SET TELEM_RO_YAW_TGT TO 0.
    SET TELEM_RO_YAW_SCALE TO 0.
    SET TELEM_RO_YAW_GATE TO 3.
    SET TELEM_RO_HDG_ERR TO WRAP_180(GET_COMPASS_HDG() - hdg_cmd).
    SET ROLLOUT_STEER_HDG TO hdg_cmd.
    SET TELEM_STEER_BLEND TO 1.
    SET TELEM_RO_LOC_CORR TO 0.
    SET TELEM_LOC_CORR TO 0.
    SET TELEM_RO_PITCH_TGT TO TO_ROTATE_PITCH_CMD.
    SET TELEM_RO_PITCH_ERR TO TO_ROTATE_PITCH_CMD - GET_PITCH().
    SET TELEM_RO_PITCH_FF TO 0.
  }

  LOCAL airborne IS agl_now > airborne_agl AND
                  SHIP:VERTICALSPEED > airborne_min_vs AND
                  SHIP:STATUS <> "LANDED".
  IF airborne {
    IF TO_AIRBORNE_UT < 0 { SET TO_AIRBORNE_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - TO_AIRBORNE_UT >= TAKEOFF_AIRBORNE_CONFIRM_S {
      UNLOCK WHEELSTEERING.
      GEAR OFF.
      LOCAL ias_now IS GET_IAS().
      LOCAL climb_fpa_tgt IS _TO_COMPUTE_CLIMB_FPA_TGT(ias_now).
      LOCAL climb_seed_max IS MAX(TO_ROTATE_PITCH_CMD, climb_fpa_tgt).
      // Seed climb command from rotate target/pitch, not instantaneous VS/IAS.
      SET TO_CLIMB_FPA_CMD TO CLAMP(GET_PITCH(), climb_fpa_tgt, climb_seed_max).
      SET SHIP:CONTROL:PITCH TO 0.
      SET TO_PITCH_CMD_PREV TO 0.
      SET_SUBPHASE(SUBPHASE_TO_CLIMB).
    }
  } ELSE {
    SET TO_AIRBORNE_UT TO -1.
  }
}

FUNCTION _RUN_TO_CLIMB {
  LOCAL v2 IS AC_PARAM("v2", TAKEOFF_V2_DEFAULT, 0.001).
  SET ACTIVE_V_TGT TO v2.

  LOCAL ias IS GET_IAS().
  LOCAL spd_err IS v2 - ias.

  LOCAL to_thr IS AC_PARAM("takeoff_throttle", 1.0, 0.001).
  LOCAL min_thr IS AC_PARAM("takeoff_climb_min_throttle", TAKEOFF_CLIMB_MIN_THR, 0).
  LOCAL thr_gain IS AC_PARAM("takeoff_climb_spd_thr_gain", TAKEOFF_CLIMB_SPD_THR_GAIN, 0).
  IF min_thr > to_thr { SET min_thr TO to_thr. }

  LOCAL raw_thr IS CLAMP(to_thr + spd_err * thr_gain, min_thr, to_thr).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).

  LOCAL climb_fpa_tgt IS _TO_COMPUTE_CLIMB_FPA_TGT(ias).
  LOCAL climb_fpa_slew IS AC_PARAM("takeoff_climb_fpa_slew_dps", TAKEOFF_CLIMB_FPA_SLEW_DPS, 0.001).
  SET TO_CLIMB_FPA_CMD TO MOVE_TOWARD(
    TO_CLIMB_FPA_CMD,
    climb_fpa_tgt,
    climb_fpa_slew * IFC_ACTUAL_DT
  ).

  UNLOCK WHEELSTEERING.
  LOCAL hdg_cmd IS TO_RWY_HDG.
  AA_SET_DIRECTOR(hdg_cmd, TO_CLIMB_FPA_CMD).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO TO_CLIMB_FPA_CMD.

  SET SHIP:CONTROL:YAW TO 0.
  SET TO_YAW_CMD_PREV TO 0.
  SET SHIP:CONTROL:PITCH TO 0.
  SET TO_PITCH_CMD_PREV TO 0.
  SET TELEM_RO_YAW_TGT TO 0.
  SET TELEM_RO_YAW_SCALE TO 0.
  SET TELEM_RO_YAW_GATE TO 3.
  SET TELEM_RO_HDG_ERR TO WRAP_180(GET_COMPASS_HDG() - hdg_cmd).
  SET ROLLOUT_STEER_HDG TO hdg_cmd.
  SET TELEM_STEER_BLEND TO 1.
  SET TELEM_RO_LOC_CORR TO 0.
  SET TELEM_LOC_CORR TO 0.

  LOCAL done_agl IS AC_PARAM("takeoff_done_agl", TAKEOFF_DONE_AGL, 0.001).
  IF GET_AGL() >= done_agl AND SHIP:STATUS <> "LANDED" {
    SET_PHASE(PHASE_DONE).
  }
}

FUNCTION _CHECK_TAKEOFF_FLAPS {
  LOCAL ac IS ACTIVE_AIRCRAFT.
  IF ac = 0 { RETURN. }
  IF NOT ac:HASKEY("ag_flaps_step_up") OR NOT ac:HASKEY("ag_flaps_step_down") { RETURN. }
  LOCAL ag_up IS ac["ag_flaps_step_up"].
  LOCAL ag_dn IS ac["ag_flaps_step_down"].
  IF ag_up <= 0 OR ag_dn <= 0 { RETURN. }

  LOCAL max_det  IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).
  LOCAL det_up   IS ROUND(AC_PARAM("flaps_detent_up", 0, 0)).
  LOCAL det_clmb IS ROUND(AC_PARAM("flaps_detent_climb", 1, 0)).
  LOCAL det_to   IS det_clmb.
  IF ac:HASKEY("flaps_detent_takeoff") {
    SET det_to TO ROUND(ac["flaps_detent_takeoff"]).
  } ELSE IF ac:HASKEY("flaps_detent_approach") {
    SET det_to TO ROUND(ac["flaps_detent_approach"]).
  }
  LOCAL vfe_clmb IS AC_PARAM("vfe_climb", 160, 0.001).

  LOCAL target_det IS det_to.
  IF IFC_SUBPHASE = SUBPHASE_TO_CLIMB {
    SET target_det TO det_clmb.
    IF GET_IAS() > vfe_clmb {
      SET target_det TO det_up.
    }
  }
  SET FLAPS_TARGET_DETENT TO CLAMP(target_det, det_up, max_det).

  IF FLAPS_CURRENT_DETENT = FLAPS_TARGET_DETENT { RETURN. }
  IF TIME:SECONDS - FLAPS_LAST_STEP_UT < FLAP_STEP_INTERVAL { RETURN. }

  IF FLAPS_CURRENT_DETENT < FLAPS_TARGET_DETENT {
    PULSE_AG(ag_up).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
    SET IFC_ALERT_TEXT TO "TO FLAPS UP -> detent " + FLAPS_CURRENT_DETENT.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  } ELSE {
    PULSE_AG(ag_dn).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
    SET IFC_ALERT_TEXT TO "TO FLAPS DN -> detent " + FLAPS_CURRENT_DETENT.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  }
  SET FLAPS_LAST_STEP_UT TO TIME:SECONDS.
}
