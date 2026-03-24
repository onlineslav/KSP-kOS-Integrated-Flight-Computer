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
  IFC_GEAR_VIS_SYNC().
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
  // Force tagged spoiler modules to full touchdown deploy angle.
  AS_DEPLOY_MAX_TOUCHDOWN().
}

// ─────────────────────────────────────────────────────────
// BOUNCE RECOVERY helper
// Detects a post-touchdown bounce and transitions back to FLARE.
//
// Parameters (all resolved by the caller from AC_PARAM or constants):
//   br_agl_m     - m runway-relative main-gear height above which aircraft is considered airborne
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
  LOCAL flare_h IS GET_MAIN_GEAR_RUNWAY_HEIGHT_MIN().
  LOCAL flare_entry_vs_min IS AC_PARAM("flare_entry_vs_min", FLARE_MIN_ENTRY_SINK_VS, -50).
  LOCAL flare_ctrl_h_offset_max IS AC_PARAM("flare_ctrl_h_offset_max_m", FLARE_CTRL_H_OFFSET_MAX_M, 0.1).
  LOCAL airborne IS PHASE_ELAPSED() < br_max_s AND
                   SHIP:STATUS <> "LANDED" AND
                   flare_h > br_agl_m AND
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
      LOCAL runway_h_now IS GET_RUNWAY_REL_HEIGHT().
      LOCAL flare_ctrl_h_offset_raw IS runway_h_now - flare_h.
      SET flare_ctrl_h_offset_max TO MAX(flare_ctrl_h_offset_max, 0.1).
      SET FLARE_CTRL_H_OFFSET TO CLAMP(flare_ctrl_h_offset_raw, 0, flare_ctrl_h_offset_max).
      IF flare_entry_vs_min > -0.05 { SET flare_entry_vs_min TO -0.05. }
      SET FLARE_PITCH_CMD TO ARCTAN(SHIP:VERTICALSPEED / entry_ias).
      SET FLARE_ENTRY_VS  TO CLAMP(SHIP:VERTICALSPEED, flare_entry_vs_min, -0.05).
      SET FLARE_ENTRY_AGL TO MAX(flare_h + FLARE_CTRL_H_OFFSET, 1).
      SET FLARE_SUBMODE TO FLARE_MODE_CAPTURE.
      SET FLARE_AUTH_LIMITED TO FALSE.
      SET FLARE_BALLOON_ACTIVE TO FALSE.
      SET FLARE_AUTH_START_UT TO -1.
      SET FLARE_TECS_ET_INT TO 0.
      SET FLARE_TECS_EB_INT TO 0.
      SET FLARE_TECS_H_REF TO FLARE_ENTRY_AGL.
      SET FLARE_IAS_PREV     TO entry_ias.
      SET FLARE_IAS_DOT_FILT TO 0.
      SET THR_INTEGRAL TO 0.
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
// Submode-based flare controller:
//   FLARE_CAPTURE -> FLARE_TRACK -> ROUNDOUT -> TOUCHDOWN_CONFIRM
// Guidance uses a control-height reference (captured body-vs-gear offset) for flare
// shaping, while touchdown logic continues using runway-relative main-gear height.
// ─────────────────────────────────────────────────────────
FUNCTION _AC_PARAM_NEG1 {
  PARAMETER key, fallback.
  IF ACTIVE_AIRCRAFT = 0 { RETURN fallback. }
  IF NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL val IS ACTIVE_AIRCRAFT[key].
  IF val <> -1 { RETURN val. }
  RETURN fallback.
}

FUNCTION _GET_FLARE_TOUCHDOWN_VS {
  LOCAL td_vs IS TOUCHDOWN_VS.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flare_touchdown_vs")
      AND ACTIVE_AIRCRAFT["flare_touchdown_vs"] <> -1
      AND ACTIVE_AIRCRAFT["flare_touchdown_vs"] < 0 {
    SET td_vs TO ACTIVE_AIRCRAFT["flare_touchdown_vs"].
  }
  RETURN td_vs.
}

FUNCTION _COMPUTE_FLARE_SUBMODE {
  PARAMETER flare_h, roundout_start_h, roundout_end_h, auth_recovery_gain.

  IF SHIP:STATUS = "LANDED" OR TOUCHDOWN_CANDIDATE_UT >= 0 {
    RETURN FLARE_MODE_TOUCHDOWN_CONFIRM.
  }

  IF FLARE_SUBMODE = FLARE_MODE_CAPTURE AND PHASE_ELAPSED() < FLARE_CAPTURE_MIN_S {
    RETURN FLARE_MODE_CAPTURE.
  }

  LOCAL round_start_eff IS roundout_start_h.
  IF FLARE_AUTH_LIMITED {
    SET round_start_eff TO round_start_eff * (1 + auth_recovery_gain).
  }

  IF round_start_eff > roundout_end_h {
    IF FLARE_SUBMODE = FLARE_MODE_ROUNDOUT {
      IF flare_h <= round_start_eff + FLARE_ROUNDOUT_HYST_M { RETURN FLARE_MODE_ROUNDOUT. }
    } ELSE IF flare_h <= round_start_eff {
      RETURN FLARE_MODE_ROUNDOUT.
    }
  }

  RETURN FLARE_MODE_TRACK.
}

FUNCTION _COMPUTE_FLARE_REFS {
  PARAMETER flare_h, ias, vs_now, flare_touchdown_vs, vref, disable_speed_bleed,
            roundout_start_h, roundout_end_h, roundout_curve, roundout_ttg_start_s,
            roundout_ttg_end_s, auth_recovery_gain.

  LOCAL frac IS CLAMP(1 - flare_h / MAX(FLARE_ENTRY_AGL, 1), 0, 1).
  LOCAL vs_ref IS FLARE_ENTRY_VS + (flare_touchdown_vs - FLARE_ENTRY_VS) * frac.

  IF NOT disable_speed_bleed {
    LOCAL speed_excess IS MAX(ias - vref, 0).
    LOCAL speed_sink_blend IS CLAMP(1 - frac, 0, 1).
    SET vs_ref TO vs_ref - speed_excess * FLARE_SPEED_BLEED_GAIN * speed_sink_blend.
  }

  LOCAL round_start_eff IS roundout_start_h.
  IF FLARE_AUTH_LIMITED {
    SET round_start_eff TO round_start_eff * (1 + auth_recovery_gain).
  }

  IF roundout_curve > 0 AND round_start_eff > roundout_end_h {
    LOCAL round_norm IS CLAMP((round_start_eff - flare_h) / MAX(round_start_eff - roundout_end_h, 0.1), 0, 1).
    LOCAL round_blend_h IS CLAMP(round_norm * roundout_curve, 0, 1).
    LOCAL sink_mag IS MAX(-vs_now, 0.5).
    LOCAL ttg_now IS flare_h / sink_mag.
    LOCAL ttg_start_eff IS MAX(roundout_ttg_start_s, roundout_ttg_end_s + 0.1).
    LOCAL ttg_end_eff IS MAX(roundout_ttg_end_s, 0).
    LOCAL ttg_norm IS CLAMP((ttg_start_eff - ttg_now) / MAX(ttg_start_eff - ttg_end_eff, 0.1), 0, 1).
    LOCAL round_blend_ttg IS CLAMP(ttg_norm * roundout_curve, 0, 1).
    LOCAL round_blend IS MAX(round_blend_h, round_blend_ttg).
    SET vs_ref TO vs_ref + (flare_touchdown_vs - vs_ref) * round_blend.
  }

  SET vs_ref TO CLAMP(vs_ref, FLARE_MAX_SINK_VS, -0.05).
  LOCAL gamma_ref IS ARCTAN(vs_ref / MAX(ias, 1)).

  RETURN LEXICON(
    "frac", frac,
    "vs_ref", vs_ref,
    "gamma_ref", gamma_ref
  ).
}

FUNCTION _UPDATE_FLARE_AUTHORITY_STATE {
  PARAMETER vs_err, fpa_err, pitch_err, ctrl_limited,
            vs_err_trigger, pitch_err_trigger, fpa_err_trigger, detect_s.

  LOCAL same_dir IS (vs_err > 0 AND pitch_err > 0 AND fpa_err > 0) OR
                   (vs_err < 0 AND pitch_err < 0 AND fpa_err < 0).
  LOCAL candidate IS ABS(vs_err) > vs_err_trigger AND
                    ABS(pitch_err) > pitch_err_trigger AND
                    ABS(fpa_err) > fpa_err_trigger AND
                    same_dir AND
                    ctrl_limited.

  IF candidate {
    IF FLARE_AUTH_START_UT < 0 { SET FLARE_AUTH_START_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - FLARE_AUTH_START_UT >= detect_s {
      SET FLARE_AUTH_LIMITED TO TRUE.
    }
  } ELSE {
    SET FLARE_AUTH_START_UT TO -1.
    SET FLARE_AUTH_LIMITED TO FALSE.
  }
}

FUNCTION _COMPUTE_FLARE_VREF_SCHED {
  PARAMETER vapp, vref, flare_frac.
  LOCAL frac_ref IS CLAMP(flare_frac, 0, 1).
  RETURN vapp + (vref - vapp) * frac_ref.
}

FUNCTION _UPDATE_FLARE_H_REF {
  PARAMETER vs_ref.
  IF FLARE_TECS_H_REF <= 0 {
    SET FLARE_TECS_H_REF TO FLARE_ENTRY_AGL.
  }
  SET FLARE_TECS_H_REF TO MAX(FLARE_TECS_H_REF + vs_ref * IFC_ACTUAL_DT, 0).
}

FUNCTION _COMPUTE_GAMMA_CMD {
  PARAMETER gamma_ref, e_balance_err, ias, gamma_cmd_min, gamma_cmd_max,
            gamma_rate_min, gamma_rate_max, eb_kp, eb_ki, eb_int_lim, auth_recovery_gain,
            e_bal_dot_err, eb_kd.
  LOCAL vg IS MAX(ias * 9.81, 50).
  LOCAL gamma_unsat IS gamma_ref + (eb_kp * e_balance_err + eb_ki * FLARE_TECS_EB_INT + eb_kd * e_bal_dot_err) / vg.
  LOCAL gamma_raw IS CLAMP(gamma_unsat, gamma_cmd_min, gamma_cmd_max).
  LOCAL sat_hi IS gamma_unsat >= gamma_cmd_max.
  LOCAL sat_lo IS gamma_unsat <= gamma_cmd_min.
  LOCAL block_int IS (sat_hi AND e_balance_err > 0) OR (sat_lo AND e_balance_err < 0).
  IF NOT block_int {
    SET FLARE_TECS_EB_INT TO CLAMP(FLARE_TECS_EB_INT + e_balance_err * IFC_ACTUAL_DT, -eb_int_lim, eb_int_lim).
  } ELSE {
    SET FLARE_TECS_EB_INT TO FLARE_TECS_EB_INT * 0.95.
  }

  SET gamma_unsat TO gamma_ref + (eb_kp * e_balance_err + eb_ki * FLARE_TECS_EB_INT + eb_kd * e_bal_dot_err) / vg.
  SET TELEM_FLARE_GAMMA_REF TO gamma_ref.
  SET TELEM_FLARE_GAMMA_EB_TERM TO (eb_kp * e_balance_err + eb_ki * FLARE_TECS_EB_INT + eb_kd * e_bal_dot_err) / vg.
  SET TELEM_FLARE_GAMMA_CMD_UNSAT TO gamma_unsat.
  SET gamma_raw TO CLAMP(gamma_unsat, gamma_cmd_min, gamma_cmd_max).
  IF FLARE_AUTH_LIMITED {
    IF e_balance_err >= 0 {
      SET gamma_raw TO gamma_cmd_max.
    } ELSE {
      SET gamma_raw TO gamma_cmd_min.
    }
  }

  LOCAL rate_span IS MAX(FLARE_RATE_HIGH_IAS - FLARE_RATE_LOW_IAS, 1).
  LOCAL rate_blend IS CLAMP((ias - FLARE_RATE_LOW_IAS) / rate_span, 0, 1).
  LOCAL gamma_rate IS gamma_rate_min + (gamma_rate_max - gamma_rate_min) * rate_blend.
  IF FLARE_AUTH_LIMITED {
    SET gamma_rate TO gamma_rate * (1 + auth_recovery_gain).
  }
  SET gamma_rate TO MAX(gamma_rate, 0.05).

  SET FLARE_PITCH_CMD TO MOVE_TOWARD(
    FLARE_PITCH_CMD,
    gamma_raw,
    gamma_rate * IFC_ACTUAL_DT
  ).
  RETURN FLARE_PITCH_CMD.
}

FUNCTION _COMPUTE_THETA_CMD {
  PARAMETER gamma_cmd.
  LOCAL theta_cmd IS gamma_cmd.
  IF NOT AA_DIR_ADD_AOA_COMP {
    // Director expects pitch; add AoA so gamma guidance maps to a usable nose attitude.
    // Bound compensation to avoid pathological spikes from noisy AoA samples.
    LOCAL aoa_comp IS CLAMP(GET_AOA(), -2, 20).
    SET theta_cmd TO gamma_cmd + aoa_comp.
  }
  RETURN theta_cmd.
}

FUNCTION _RUN_FLARE_THROTTLE_POLICY {
  PARAMETER flare_h, vs_now, vs_ref, e_total_err, e_balance_err,
            thr_trim, et_kp, et_ki, et_int_lim, thr_bal_k, thr_slew,
            min_throttle, min_throttle_agl_blend, climb_vs_gate, auth_recovery_gain,
            e_total_dot_err, et_kd, balloon_active.

  LOCAL thr_floor IS CLAMP(min_throttle, 0, 1).
  IF min_throttle_agl_blend > 0 AND flare_h < min_throttle_agl_blend {
    SET thr_floor TO thr_floor * CLAMP(flare_h / min_throttle_agl_blend, 0, 1).
  }
  IF FLARE_AUTH_LIMITED {
    SET thr_floor TO CLAMP(thr_floor + auth_recovery_gain, 0, 1).
  }
  IF balloon_active {
    // Balloon guard takes priority over throttle floor retention.
    SET thr_floor TO 0.
  }

  LOCAL thr_max IS 1.
  LOCAL raw_unsat IS thr_trim
    + et_kp * e_total_err
    + et_ki * FLARE_TECS_ET_INT
    + et_kd * e_total_dot_err
    + thr_bal_k * e_balance_err.

  IF balloon_active {
    SET thr_max TO thr_floor.
    SET raw_unsat TO MIN(raw_unsat, thr_floor).
  }

  // Keep throttle from feeding balloon/climb-away conditions.
  IF vs_now > climb_vs_gate OR vs_now > vs_ref + 0.2 {
    SET thr_max TO thr_floor.
    SET raw_unsat TO MIN(raw_unsat, thr_floor).
  }
  IF e_balance_err < -300 {
    SET thr_max TO MIN(thr_max, MAX(thr_floor, 0.25)).
  }

  LOCAL raw_thr IS CLAMP(raw_unsat, thr_floor, thr_max).
  LOCAL sat_hi IS raw_unsat >= thr_max.
  LOCAL sat_lo IS raw_unsat <= thr_floor.
  LOCAL block_int IS (sat_hi AND e_total_err > 0) OR (sat_lo AND e_total_err < 0).
  IF NOT block_int {
    SET FLARE_TECS_ET_INT TO CLAMP(FLARE_TECS_ET_INT + e_total_err * IFC_ACTUAL_DT, -et_int_lim, et_int_lim).
  } ELSE {
    SET FLARE_TECS_ET_INT TO FLARE_TECS_ET_INT * 0.95.
  }

  SET raw_unsat TO thr_trim
    + et_kp * e_total_err
    + et_ki * FLARE_TECS_ET_INT
    + et_kd * e_total_dot_err
    + thr_bal_k * e_balance_err.
  SET raw_thr TO CLAMP(raw_unsat, thr_floor, thr_max).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, MAX(thr_slew, 0.1) * IFC_ACTUAL_DT).
  SET TELEM_FLARE_THR_FLOOR TO thr_floor.
}

FUNCTION _CHECK_FLARE_CLIMBAWAY {
  PARAMETER flare_h, vs_now.
  // Escape hatch: if flare has turned into sustained climb and we are back
  // above the trigger band, hand back to approach ILS tracking.
  LOCAL flare_agl IS AC_PARAM("flare_agl", FLARE_AGL_M, 0).
  IF PHASE_ELAPSED() < 2 { RETURN FALSE. }
  IF vs_now <= 0.8 { RETURN FALSE. }
  IF flare_h <= flare_agl + FLARE_TRIGGER_HYST_M + 1 { RETURN FALSE. }

  SET FLARE_TRIGGER_START_UT TO -1.
  SET TOUCHDOWN_CANDIDATE_UT TO -1.
  SET FLARE_SUBMODE          TO FLARE_MODE_CAPTURE.
  SET FLARE_AUTH_LIMITED     TO FALSE.
  SET FLARE_BALLOON_ACTIVE   TO FALSE.
  SET FLARE_AUTH_START_UT    TO -1.
  SET FLARE_TECS_ET_INT      TO 0.
  SET FLARE_TECS_EB_INT      TO 0.
  SET FLARE_TECS_H_REF       TO flare_agl.
  SET FLARE_CTRL_H_OFFSET    TO 0.
  SET THR_INTEGRAL           TO 0.
  SET IFC_ALERT_TEXT TO "FLARE climb-away -> APPROACH ILS_TRACK".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET_SUBPHASE(SUBPHASE_ILS_TRACK).
  SET_PHASE(PHASE_APPROACH).
  RETURN TRUE.
}

FUNCTION _RUN_FLARE_TOUCHDOWN_GATE {
  PARAMETER flare_h, vs_now, touchdown_confirm_s, touchdown_confirm_max_abs_vs.

  IF SHIP:STATUS = "LANDED" {
    IF TOUCHDOWN_CANDIDATE_UT < 0 { SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS. }
    SET FLARE_SUBMODE TO FLARE_MODE_TOUCHDOWN_CONFIRM.
    IF TIME:SECONDS - TOUCHDOWN_CANDIDATE_UT < touchdown_confirm_s { RETURN FALSE. }
    IF ABS(vs_now) > touchdown_confirm_max_abs_vs {
      SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS.
      RETURN FALSE.
    }
    _DEPLOY_TOUCHDOWN_SPOILERS().
    SET TOUCHDOWN_CAPTURE_PITCH_DEG TO GET_PITCH().
    SET TOUCHDOWN_INIT_DONE         TO FALSE.
    SET TOUCHDOWN_CANDIDATE_UT      TO -1.
    SET BOUNCE_RECOVERY_START_UT    TO -1.
    SET FLARE_AUTH_LIMITED          TO FALSE.
    SET FLARE_BALLOON_ACTIVE        TO FALSE.
    SET FLARE_AUTH_START_UT         TO -1.
    SET FLARE_TECS_ET_INT           TO 0.
    SET FLARE_TECS_EB_INT           TO 0.
    SET FLARE_SUBMODE               TO FLARE_MODE_TOUCHDOWN_CONFIRM.
    SET_PHASE(PHASE_TOUCHDOWN).
    RETURN TRUE.
  }

  LOCAL td_gear_count IS COUNT_MAIN_GEAR_BELOW(TOUCHDOWN_FALLBACK_AGL_M).
  LOCAL td_gear_need IS 1.
  IF FLARE_GEAR_PARTS:LENGTH >= 2 { SET td_gear_need TO 2. }
  LOCAL td_fallback IS flare_h < TOUCHDOWN_FALLBACK_AGL_M AND
                      vs_now <= TOUCHDOWN_FALLBACK_MAX_VS AND
                      td_gear_count >= td_gear_need.
  IF td_fallback {
    IF TOUCHDOWN_CANDIDATE_UT < 0 { SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS. }
    SET FLARE_SUBMODE TO FLARE_MODE_TOUCHDOWN_CONFIRM.
    IF TIME:SECONDS - TOUCHDOWN_CANDIDATE_UT >= touchdown_confirm_s {
      IF ABS(vs_now) > touchdown_confirm_max_abs_vs {
        SET TOUCHDOWN_CANDIDATE_UT TO TIME:SECONDS.
        RETURN FALSE.
      }
      _DEPLOY_TOUCHDOWN_SPOILERS().
      SET TOUCHDOWN_CAPTURE_PITCH_DEG TO GET_PITCH().
      SET TOUCHDOWN_INIT_DONE         TO FALSE.
      SET TOUCHDOWN_CANDIDATE_UT      TO -1.
      SET BOUNCE_RECOVERY_START_UT    TO -1.
      SET FLARE_AUTH_LIMITED          TO FALSE.
      SET FLARE_BALLOON_ACTIVE        TO FALSE.
      SET FLARE_AUTH_START_UT         TO -1.
      SET FLARE_TECS_ET_INT           TO 0.
      SET FLARE_TECS_EB_INT           TO 0.
      SET_PHASE(PHASE_TOUCHDOWN).
      RETURN TRUE.
    }
  } ELSE {
    SET TOUCHDOWN_CANDIDATE_UT TO -1.
  }

  RETURN FALSE.
}

FUNCTION _RUN_FLARE {
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE   TO 0.
  SET TELEM_RO_YAW_GATE    TO 0.
  SET TELEM_RO_PITCH_TGT   TO 0.
  SET TELEM_RO_PITCH_ERR   TO 0.
  SET TELEM_RO_PITCH_FF    TO 0.

  LOCAL flare_h IS GET_MAIN_GEAR_RUNWAY_HEIGHT_MIN().
  LOCAL flare_h_ctrl IS 0.
  LOCAL ias IS MAX(GET_IAS(), 10).
  LOCAL vs_now IS SHIP:VERTICALSPEED.
  LOCAL flare_touchdown_vs IS _GET_FLARE_TOUCHDOWN_VS().
  LOCAL vref IS AC_PARAM("v_ref", ACTIVE_V_APP - 10, 0).
  LOCAL touchdown_confirm_s IS AC_PARAM("touchdown_confirm_s", TOUCHDOWN_CONFIRM_S, 0.001).
  LOCAL touchdown_confirm_max_abs_vs IS AC_PARAM("touchdown_confirm_max_abs_vs", TOUCHDOWN_CONFIRM_MAX_ABS_VS, 0.001).

  IF _CHECK_FLARE_CLIMBAWAY(flare_h, vs_now) { RETURN. }

  LOCAL flare_cmd_fpa_min IS _AC_PARAM_NEG1("flare_cmd_fpa_min", FLARE_CMD_FPA_MIN).
  LOCAL flare_cmd_fpa_max IS _AC_PARAM_NEG1("flare_cmd_fpa_max", FLARE_CMD_FPA_MAX).
  LOCAL flare_cmd_rate_min_dps IS _AC_PARAM_NEG1("flare_cmd_rate_min_dps", FLARE_CMD_RATE_MIN_DPS).
  LOCAL flare_cmd_rate_max_dps IS _AC_PARAM_NEG1("flare_cmd_rate_max_dps", FLARE_CMD_RATE_MAX_DPS).
  LOCAL flare_roundout_start_h_m IS _AC_PARAM_NEG1("flare_roundout_start_h_m", FLARE_ROUNDOUT_START_H_M).
  LOCAL flare_roundout_end_h_m IS _AC_PARAM_NEG1("flare_roundout_end_h_m", FLARE_ROUNDOUT_END_H_M).
  LOCAL flare_roundout_curve IS _AC_PARAM_NEG1("flare_roundout_curve", FLARE_ROUNDOUT_CURVE).
  LOCAL flare_ctrl_h_offset_max IS _AC_PARAM_NEG1("flare_ctrl_h_offset_max_m", FLARE_CTRL_H_OFFSET_MAX_M).
  LOCAL flare_roundout_ttg_start_s IS _AC_PARAM_NEG1("flare_roundout_ttg_start_s", FLARE_ROUNDOUT_TTG_START_S).
  LOCAL flare_roundout_ttg_end_s IS _AC_PARAM_NEG1("flare_roundout_ttg_end_s", FLARE_ROUNDOUT_TTG_END_S).
  LOCAL flare_min_throttle IS _AC_PARAM_NEG1("flare_min_throttle", FLARE_MIN_THROTTLE).
  LOCAL flare_min_throttle_agl_blend IS _AC_PARAM_NEG1("flare_min_throttle_agl_blend", FLARE_MIN_THROTTLE_AGL_BLEND).
  LOCAL flare_authority_vs_err_trigger IS _AC_PARAM_NEG1("flare_authority_vs_err_trigger", FLARE_AUTH_VS_ERR_TRIGGER).
  LOCAL flare_authority_pitch_err_trigger IS _AC_PARAM_NEG1("flare_authority_pitch_err_trigger", FLARE_AUTH_PITCH_ERR_TRIGGER).
  LOCAL flare_authority_fpa_err_trigger IS _AC_PARAM_NEG1("flare_authority_fpa_err_trigger", FLARE_AUTH_FPA_ERR_TRIGGER).
  LOCAL flare_authority_detect_s IS _AC_PARAM_NEG1("flare_authority_detect_s", FLARE_AUTH_DETECT_S).
  LOCAL flare_authority_recovery_gain IS _AC_PARAM_NEG1("flare_authority_recovery_gain", FLARE_AUTH_RECOVERY_GAIN).
  LOCAL flare_tecs_et_kp IS _AC_PARAM_NEG1("flare_tecs_et_kp", FLARE_TECS_ET_KP).
  LOCAL flare_tecs_et_ki IS _AC_PARAM_NEG1("flare_tecs_et_ki", FLARE_TECS_ET_KI).
  LOCAL flare_tecs_eb_kp IS _AC_PARAM_NEG1("flare_tecs_eb_kp", FLARE_TECS_EB_KP).
  LOCAL flare_tecs_eb_ki IS _AC_PARAM_NEG1("flare_tecs_eb_ki", FLARE_TECS_EB_KI).
  LOCAL flare_tecs_et_int_lim IS _AC_PARAM_NEG1("flare_tecs_et_int_lim", FLARE_TECS_ET_INT_LIM).
  LOCAL flare_tecs_eb_int_lim IS _AC_PARAM_NEG1("flare_tecs_eb_int_lim", FLARE_TECS_EB_INT_LIM).
  LOCAL flare_tecs_thr_trim IS _AC_PARAM_NEG1("flare_tecs_thr_trim", FLARE_TECS_THR_TRIM).
  LOCAL flare_tecs_thr_bal_k IS _AC_PARAM_NEG1("flare_tecs_thr_bal_k", FLARE_TECS_THR_BAL_K).
  LOCAL flare_tecs_thr_slew_per_s IS _AC_PARAM_NEG1("flare_tecs_thr_slew_per_s", FLARE_TECS_THR_SLEW_PER_S).
  LOCAL flare_tecs_climb_vs_gate IS _AC_PARAM_NEG1("flare_tecs_climb_vs_gate", FLARE_TECS_CLIMB_VS_GATE).
  LOCAL flare_tecs_edot_alpha IS _AC_PARAM_NEG1("flare_tecs_edot_alpha", FLARE_TECS_EDOT_ALPHA).
  LOCAL flare_tecs_et_kd IS _AC_PARAM_NEG1("flare_tecs_et_kd", FLARE_TECS_ET_KD).
  LOCAL flare_tecs_eb_kd IS _AC_PARAM_NEG1("flare_tecs_eb_kd", FLARE_TECS_EB_KD).
  LOCAL flare_balloon_vs_trigger IS _AC_PARAM_NEG1("flare_balloon_vs_trigger", FLARE_BALLOON_VS_TRIGGER).
  LOCAL flare_balloon_clear_vs IS _AC_PARAM_NEG1("flare_balloon_clear_vs", FLARE_BALLOON_CLEAR_VS).
  LOCAL flare_balloon_min_h_m IS _AC_PARAM_NEG1("flare_balloon_min_h_m", FLARE_BALLOON_MIN_H_M).
  LOCAL flare_balloon_gamma_down_deg IS _AC_PARAM_NEG1("flare_balloon_gamma_down_deg", FLARE_BALLOON_GAMMA_DOWN_DEG).
  LOCAL tailstrike_pitch_max IS GET_TAILSTRIKE_PITCH_MAX().
  LOCAL flare_disable_speed_bleed_raw IS _AC_PARAM_NEG1(
    "flare_disable_speed_bleed",
    CHOOSE 1 IF FLARE_DISABLE_SPEED_BLEED_DEFAULT ELSE 0
  ).
  LOCAL flare_disable_speed_bleed IS flare_disable_speed_bleed_raw <> 0.

  IF flare_cmd_fpa_max < flare_cmd_fpa_min { SET flare_cmd_fpa_max TO flare_cmd_fpa_min. }
  IF flare_cmd_rate_max_dps < flare_cmd_rate_min_dps { SET flare_cmd_rate_max_dps TO flare_cmd_rate_min_dps. }
  IF flare_roundout_end_h_m < 0 { SET flare_roundout_end_h_m TO 0. }
  IF flare_roundout_start_h_m < flare_roundout_end_h_m {
    SET flare_roundout_start_h_m TO flare_roundout_end_h_m.
  }
  SET flare_roundout_curve TO MAX(flare_roundout_curve, 0).
  SET flare_ctrl_h_offset_max TO MAX(flare_ctrl_h_offset_max, 0.1).
  SET FLARE_CTRL_H_OFFSET TO CLAMP(FLARE_CTRL_H_OFFSET, 0, flare_ctrl_h_offset_max).
  SET flare_h_ctrl TO MAX(flare_h + FLARE_CTRL_H_OFFSET, 0).
  SET flare_roundout_ttg_end_s TO MAX(flare_roundout_ttg_end_s, 0.1).
  SET flare_roundout_ttg_start_s TO MAX(flare_roundout_ttg_start_s, flare_roundout_ttg_end_s + 0.1).
  SET flare_min_throttle TO CLAMP(flare_min_throttle, 0, 1).
  SET flare_authority_detect_s TO MAX(flare_authority_detect_s, 0.05).
  SET flare_authority_recovery_gain TO CLAMP(flare_authority_recovery_gain, 0, 1).
  SET flare_tecs_et_int_lim TO MAX(flare_tecs_et_int_lim, 1).
  SET flare_tecs_eb_int_lim TO MAX(flare_tecs_eb_int_lim, 1).
  SET flare_tecs_thr_trim TO CLAMP(flare_tecs_thr_trim, 0, 1).
  SET flare_tecs_thr_slew_per_s TO MAX(flare_tecs_thr_slew_per_s, 0.1).
  IF flare_balloon_clear_vs > flare_balloon_vs_trigger {
    SET flare_balloon_clear_vs TO flare_balloon_vs_trigger - 0.05.
  }
  SET flare_balloon_min_h_m TO MAX(flare_balloon_min_h_m, 0).
  SET flare_balloon_gamma_down_deg TO CLAMP(flare_balloon_gamma_down_deg, flare_cmd_fpa_min, flare_cmd_fpa_max).

  SET FLARE_SUBMODE TO _COMPUTE_FLARE_SUBMODE(
    flare_h_ctrl,
    flare_roundout_start_h_m,
    flare_roundout_end_h_m,
    flare_authority_recovery_gain
  ).

  LOCAL refs IS _COMPUTE_FLARE_REFS(
    flare_h_ctrl,
    ias,
    vs_now,
    flare_touchdown_vs,
    vref,
    flare_disable_speed_bleed,
    flare_roundout_start_h_m,
    flare_roundout_end_h_m,
    flare_roundout_curve,
    flare_roundout_ttg_start_s,
    flare_roundout_ttg_end_s,
    flare_authority_recovery_gain
  ).
  LOCAL flare_frac IS refs["frac"].
  LOCAL flare_tgt_vs IS refs["vs_ref"].
  LOCAL gamma_ref IS refs["gamma_ref"].
  IF FLARE_TECS_H_REF <= 0 OR PHASE_ELAPSED() < 0.3 {
    SET FLARE_TECS_H_REF TO MAX(flare_h_ctrl, 1).
  }
  _UPDATE_FLARE_H_REF(flare_tgt_vs).
  LOCAL v_ref_sched IS _COMPUTE_FLARE_VREF_SCHED(ACTIVE_V_APP, vref, flare_frac).
  LOCAL g IS 9.81.
  LOCAL e_total_ref IS 0.5 * v_ref_sched * v_ref_sched + g * FLARE_TECS_H_REF.
  LOCAL e_total_now IS 0.5 * ias * ias + g * flare_h_ctrl.
  LOCAL e_balance_ref IS g * FLARE_TECS_H_REF - 0.5 * v_ref_sched * v_ref_sched.
  LOCAL e_balance_now IS g * flare_h_ctrl - 0.5 * ias * ias.
  LOCAL e_total_err IS e_total_ref - e_total_now.
  LOCAL e_balance_err IS e_balance_ref - e_balance_now.

  // Energy rate errors for D-term damping.
  // IAS derivative: EMA-filtered to reduce sensor noise.
  LOCAL ias_dot_raw IS (ias - FLARE_IAS_PREV) / IFC_ACTUAL_DT.
  SET FLARE_IAS_DOT_FILT TO flare_tecs_edot_alpha * ias_dot_raw + (1 - flare_tecs_edot_alpha) * FLARE_IAS_DOT_FILT.
  SET FLARE_IAS_PREV TO ias.
  // Demanded rates: d/dt(E_T_ref) = V_ref * V_ref_dot + g * VS_ref
  // Measured rates: d/dt(E_T)    = V * V_dot + g * VS
  // We approximate V_ref_dot ≈ 0 (Vref schedule slews slowly).
  LOCAL e_total_dot_dem IS g * flare_tgt_vs.
  LOCAL e_total_dot_meas IS ias * FLARE_IAS_DOT_FILT + g * vs_now.
  LOCAL e_total_dot_err IS e_total_dot_dem - e_total_dot_meas.
  LOCAL e_bal_dot_dem IS g * flare_tgt_vs.
  LOCAL e_bal_dot_meas IS g * vs_now - ias * FLARE_IAS_DOT_FILT.
  LOCAL e_bal_dot_err IS e_bal_dot_dem - e_bal_dot_meas.
  SET TELEM_FLARE_ETDOT_ERR TO e_total_dot_err.
  SET TELEM_FLARE_EBDOT_ERR TO e_bal_dot_err.
  SET TELEM_FLARE_IAS_DOT   TO FLARE_IAS_DOT_FILT.

  LOCAL vs_err IS flare_tgt_vs - vs_now.
  LOCAL fpa_err IS gamma_ref - TELEM_ACTUAL_FPA_DEG.
  LOCAL pitch_err IS TELEM_AA_DIR_PITCH_DEG - TELEM_PITCH_DEG.
  LOCAL thr_now IS GET_CURRENT_THROTTLE().
  LOCAL thr_at_floor IS thr_now <= TELEM_FLARE_THR_FLOOR + 0.02.
  LOCAL thr_at_ceiling IS thr_now >= 0.98.
  LOCAL max_aoa IS AC_PARAM("aa_max_aoa", AA_MAX_AOA, 0.001).
  LOCAL aoa_near_lim IS FALSE.
  IF max_aoa > 0 AND GET_AOA() >= max_aoa - 0.5 { SET aoa_near_lim TO TRUE. }
  IF flare_h_ctrl > flare_balloon_min_h_m AND vs_now > flare_balloon_vs_trigger {
    SET FLARE_BALLOON_ACTIVE TO TRUE.
  } ELSE IF FLARE_BALLOON_ACTIVE AND (vs_now <= flare_balloon_clear_vs OR flare_h_ctrl <= flare_balloon_min_h_m) {
    SET FLARE_BALLOON_ACTIVE TO FALSE.
  }
  // Pitch at/over tailstrike limit with unresolved nose-up error is treated as
  // an authority limit so flare recovery can shift toward throttle support.
  LOCAL tailstrike_limited_prev IS TELEM_AA_DIR_PITCH_DEG >= tailstrike_pitch_max - 0.25 AND
                                   pitch_err > 0 AND vs_err > 0.
  LOCAL ctrl_limited IS thr_at_floor OR thr_at_ceiling OR aoa_near_lim OR tailstrike_limited_prev OR FLARE_BALLOON_ACTIVE.
  SET TELEM_FLARE_ET_ERR TO e_total_err.
  SET TELEM_FLARE_EB_ERR TO e_balance_err.
  SET TELEM_FLARE_H_REF TO FLARE_TECS_H_REF.
  SET TELEM_FLARE_V_REF TO v_ref_sched.

  _UPDATE_FLARE_AUTHORITY_STATE(
    vs_err,
    fpa_err,
    pitch_err,
    ctrl_limited,
    flare_authority_vs_err_trigger,
    flare_authority_pitch_err_trigger,
    flare_authority_fpa_err_trigger,
    flare_authority_detect_s
  ).

  LOCAL gamma_cmd IS _COMPUTE_GAMMA_CMD(
    gamma_ref,
    e_balance_err,
    ias,
    flare_cmd_fpa_min,
    flare_cmd_fpa_max,
    flare_cmd_rate_min_dps,
    flare_cmd_rate_max_dps,
    flare_tecs_eb_kp,
    flare_tecs_eb_ki,
    flare_tecs_eb_int_lim,
    flare_authority_recovery_gain,
    e_bal_dot_err,
    flare_tecs_eb_kd
  ).
  // Late flare priority: when sink is already worse than target in ROUNDOUT,
  // do not let the energy-balance loop command additional nose-down beyond gamma_ref.
  IF FLARE_SUBMODE = FLARE_MODE_ROUNDOUT AND vs_err > 0 AND gamma_cmd < gamma_ref {
    SET gamma_cmd TO gamma_ref.
  }
  IF FLARE_BALLOON_ACTIVE {
    SET gamma_cmd TO MIN(gamma_cmd, flare_balloon_gamma_down_deg).
    SET FLARE_AUTH_LIMITED TO TRUE.
    IF FLARE_AUTH_START_UT < 0 { SET FLARE_AUTH_START_UT TO TIME:SECONDS. }
  }

  LOCAL theta_cmd_raw IS _COMPUTE_THETA_CMD(gamma_cmd).
  LOCAL theta_cmd IS CLAMP_TAILSTRIKE_DIRECTOR_CMD(theta_cmd_raw).
  LOCAL tailstrike_limited_now IS theta_cmd < theta_cmd_raw - 0.001.
  IF tailstrike_limited_now AND vs_err > 0 {
    SET FLARE_AUTH_LIMITED TO TRUE.
    IF FLARE_AUTH_START_UT < 0 { SET FLARE_AUTH_START_UT TO TIME:SECONDS. }
  }
  AA_SET_DIRECTOR(ACTIVE_RWY_HDG, theta_cmd).
  SET TELEM_AA_HDG_CMD   TO ACTIVE_RWY_HDG.
  SET TELEM_AA_FPA_CMD   TO gamma_cmd.
  SET TELEM_FLARE_TGT_VS TO flare_tgt_vs.
  SET TELEM_FLARE_FRAC   TO flare_frac.

  _RUN_FLARE_THROTTLE_POLICY(
    flare_h_ctrl,
    vs_now,
    flare_tgt_vs,
    e_total_err,
    e_balance_err,
    flare_tecs_thr_trim,
    flare_tecs_et_kp,
    flare_tecs_et_ki,
    flare_tecs_et_int_lim,
    flare_tecs_thr_bal_k,
    flare_tecs_thr_slew_per_s,
    flare_min_throttle,
    flare_min_throttle_agl_blend,
    flare_tecs_climb_vs_gate,
    flare_authority_recovery_gain,
    e_total_dot_err,
    flare_tecs_et_kd,
    FLARE_BALLOON_ACTIVE
  ).

  IF _RUN_FLARE_TOUCHDOWN_GATE(flare_h, vs_now, touchdown_confirm_s, touchdown_confirm_max_abs_vs) {
    RETURN.
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
