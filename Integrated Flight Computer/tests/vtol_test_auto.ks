@LAZYGLOBAL OFF.

// ============================================================
// vtol_test_auto.ks - autonomous VTOL Plan-2 regression flight
//
// This script runs the VTOL module without keyboard input and
// executes a deterministic sequence of test phases:
//
// 1) takeoff/climb and altitude hold capture
// 2) bank-step disturbance
// 3) velocity-hold acceleration
// 4) KHV stop
// 5) position-hold return after push-off
// 6) physical allocator A/B window
// 7) transition scheduler acceleration window
// 8) recovery hover and shutdown
//
// It logs a CSV file in 0:/Integrated Flight Computer/logs/
// for post-flight analysis.
// ============================================================

LOCAL ifc_root IS "0:/Integrated Flight Computer/".

RUNPATH(ifc_root + "lib/ifc_constants.ks").
RUNPATH(ifc_root + "lib/ifc_state.ks").
RUNPATH(ifc_root + "lib/ifc_helpers.ks").
RUNPATH(ifc_root + "lib/ifc_autobrake.ks").
RUNPATH(ifc_root + "lib/ifc_amo.ks").
RUNPATH(ifc_root + "lib/ifc_amo_vtol.ks").
RUNPATH(ifc_root + "lib/ifc_engine_model.ks").
RUNPATH(ifc_root + "nav/nav_math.ks").

GLOBAL AUTO_RUNNING IS TRUE.
GLOBAL AUTO_LOG_FILE IS "".
GLOBAL AUTO_PHASE_NAME IS "INIT".
GLOBAL AUTO_PHASE_START_UT IS 0.
GLOBAL AUTO_MISSION_START_UT IS 0.
GLOBAL AUTO_LAST_LOG_UT IS 0.
GLOBAL AUTO_LOG_PERIOD IS 0.5.
GLOBAL AUTO_LAST_DISPLAY_UT IS 0.
GLOBAL AUTO_DISPLAY_PERIOD IS 0.5.
GLOBAL AUTO_ABORT_GUARDS_ENABLED IS FALSE.
GLOBAL AUTO_TAKEOFF_TIMEOUT_S IS 90.0.
GLOBAL AUTO_TAKEOFF_RUNAWAY_DELAY_S IS 20.0.
GLOBAL AUTO_TAKEOFF_RUNAWAY_GNDSPD_MS IS 8.0.
GLOBAL AUTO_TAKEOFF_TARGET_ALT_AGL_M IS 45.0.
GLOBAL AUTO_TAKEOFF_TARGET_VS_MS IS 1.0.

FUNCTION _AUTO_CFG_NUM {
  PARAMETER key_name, fallback_val.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY(key_name) {
    RETURN ACTIVE_AIRCRAFT[key_name].
  }
  RETURN fallback_val.
}

FUNCTION _AUTO_FMT {
  PARAMETER val_in, width_in, decimals_in.
  LOCAL out_text IS ROUND(val_in, decimals_in):TOSTRING.
  UNTIL out_text:LENGTH >= width_in {
    SET out_text TO " " + out_text.
  }
  RETURN out_text.
}

FUNCTION _AUTO_NEW_LOG_FILE {
  LOCAL log_dir IS "0:/Integrated Flight Computer/logs".
  LOCAL counter_path IS log_dir + "/counter.txt".
  LOCAL seq_num IS 1.

  IF NOT EXISTS(log_dir) {
    CREATEDIR(log_dir).
  }

  IF EXISTS(counter_path) {
    LOCAL counter_text IS OPEN(counter_path):READALL:STRING:TRIM.
    IF counter_text:LENGTH > 0 {
      LOCAL parsed_seq IS counter_text:TONUMBER(0).
      IF parsed_seq >= 1 {
        SET seq_num TO ROUND(parsed_seq).
      }
    }
  }

  IF EXISTS(counter_path) {
    DELETEPATH(counter_path).
  }
  LOG (seq_num + 1) TO counter_path.

  LOCAL seq_text IS "" + seq_num.
  UNTIL seq_text:LENGTH >= 8 {
    SET seq_text TO "0" + seq_text.
  }
  RETURN log_dir + "/ifc_log_" + seq_text + ".csv".
}

FUNCTION _AUTO_LOG_EVENT {
  PARAMETER event_text.
  IF AUTO_LOG_FILE = "" { RETURN. }
  LOG ("# EVENT t=" + ROUND(TIME:SECONDS - AUTO_MISSION_START_UT, 2) +
       " phase=" + AUTO_PHASE_NAME + " msg=" + event_text) TO AUTO_LOG_FILE.
}

FUNCTION _AUTO_SET_PHASE {
  PARAMETER next_phase_name.
  SET AUTO_PHASE_NAME TO next_phase_name.
  SET AUTO_PHASE_START_UT TO TIME:SECONDS.
  PRINT "AUTO PHASE -> " + next_phase_name AT(0, 2).
  _AUTO_LOG_EVENT("phase_enter").
}

SET ACTIVE_AIRCRAFT TO LEXICON(
  "name",                   "VTOL Auto Test Craft",
  "has_vtol",               TRUE,
  "has_nws",                FALSE,
  "vtol_eng_tag_prefix",    "vtol_eng",
  "vtol_srv_tag_prefix",    "vtol_srv",
  "vtol_hover_angle",       90,
  "vtol_cruise_angle",      0,
  "vtol_yaw_gain",          8,
  "vtol_roll_gain",         0.25,
  "vtol_pitch_gain",        0.30,
  "vtol_pitch_mix_sign",    1,
  "vtol_level_roll_kp",     0.10,
  "vtol_level_roll_kd",     0.03,
  "vtol_level_roll_ki",     0.010,
  "vtol_level_pitch_kp",    0.10,
  "vtol_level_pitch_kd",    0.08,
  "vtol_level_pitch_ki",    0.015,
  "vtol_level_roll_att2rate_kp",  0.8,
  "vtol_level_pitch_att2rate_kp", 0.40,
  "vtol_level_roll_att2rate_ki",  0.00,
  "vtol_level_pitch_att2rate_ki", 0.00,
  "vtol_level_roll_rate_kp",      0.030,
  "vtol_level_pitch_rate_kp",     0.032,
  "vtol_level_roll_rate_cmd_max_degs",  6.0,
  "vtol_level_pitch_rate_cmd_max_degs", 6.5,
  "vtol_level_i_lim",       40.0,
  "vtol_lag_filter_tau_s",  0.90,
  "vtol_level_gain_lag_ref_s", 0.8,
  "vtol_level_kp_min_scale", 0.45,
  "vtol_level_kd_min_scale", 0.45,
  "vtol_level_ki_min_scale", 0.20,
  "vtol_level_aw_lag_s",    0.60,
  "vtol_level_aw_eff_err_min", 0.05,
  "vtol_level_cmd_slew_per_s", 1.5,
  "vtol_level_cmd_slew_lag_ref_s", 0.8,
  "vtol_level_cmd_slew_min_scale", 0.35,
  "vtol_level_on_ground",   FALSE,
  "vtol_level_min_agl_m",   0.8,
  "vtol_ground_contact_agl_m", 1.5,
  "vtol_ground_contact_vs_max", 0.7,
  "vtol_static_trim_discovery", TRUE,
  "vtol_diff_collective_min",   0.12,
  "vtol_engine_limit_floor",    0.14,
  "vtol_cmd_slew_per_s",        3.0,
  "vtol_cmd_roll_max",          0.55,
  "vtol_cmd_pitch_max",         0.60,
  "vtol_diff_atten_min",        0.25,
  "vtol_upset_bank_deg",        18.0,
  "vtol_upset_pitch_deg",       12.0,
  "vtol_upset_roll_rate_degs",  18.0,
  "vtol_upset_pitch_rate_degs", 14.0,
  "vtol_upset_exit_bank_deg",   12.0,
  "vtol_upset_exit_pitch_deg",  8.0,
  "vtol_upset_exit_roll_rate_degs", 10.0,
  "vtol_upset_exit_pitch_rate_degs", 8.0,
  "vtol_upset_hold_s",          2.0,
  "vtol_upset_cmd_max",         0.30,
  "vtol_upset_cmd_roll_max",    0.60,
  "vtol_upset_cmd_pitch_max",   0.60,
  "vtol_upset_roll_rate_kp",    0.030,
  "vtol_upset_pitch_rate_kp",   0.035,
  "vtol_upset_pitch_slew_bypass", TRUE,
  "vtol_upset_diff_atten_min",  0.55,
  "vtol_upset_engine_limit_floor", 0.02,
  "vtol_upset_guard_agl_m",     20.0,
  "vtol_upset_guard_thr_min",   0.55,
  "vtol_rate_kd_roll_accel",    0.012,
  "vtol_rate_kd_pitch_accel",   0.012,
  "vtol_rate_p_alpha",          0.70,
  "vtol_rate_q_alpha",          0.70,
  "vtol_rate_pdot_alpha",       0.35,
  "vtol_rate_qdot_alpha",       0.35,
  "vtol_rate_accel_clamp_degs2", 300.0,
  "vtol_cos_att_alpha",         0.10,
  "vtol_cos_att_floor",         0.25,
  "vtol_trim_min_agl_m",        0.0,
  "vtol_trim_rate",             0.003,
  "vtol_trim_roll_rate",        0.001,
  "vtol_trim_rate_lead_s",      0.0,
  "vtol_trim_activity_min",     0.0,
  "vtol_trim_min_offset",       -0.85,
  "vtol_trim_max_offset",       0.0,
  "vtol_trim_active_pitch_max", 8.0,
  "vtol_trim_active_rate_max",  12.0,
  "vtol_trim_bank_clamp",       10.0,
  "vtol_trim_active_bank_max",  8.0,
  "vtol_trim_active_roll_rate_max", 16.0,
  "vtol_static_trim_base_min",  0.23,
  "vtol_vs_kp",             0.18,
  "vtol_vs_ki",             0.02,
  "vtol_vs_cmd_up_slew_mps2", 0.7,
  "vtol_vs_cmd_dn_slew_mps2", 1.0,
  "vtol_vs_cmd_lag_ref_s",    0.8,
  "vtol_vs_cmd_slew_min_scale", 0.35,
  "vtol_vs_gain_lag_ref_s",   0.8,
  "vtol_vs_kp_min_scale",     0.45,
  "vtol_vs_ki_min_scale",     0.25,
  "vtol_vs_aw_alpha_min",     0.95,
  "vtol_vs_aw_lag_s",         0.60,
  "vtol_vs_aw_eff_err_min",   0.05,
  "vtol_vs_i_unwind_per_s",   2.2,
  "vtol_max_vs",            1.0,
  "vtol_collective_max",    0.90,
  "vtol_collective_up_slew_per_s", 0.22,
  "vtol_collective_dn_slew_per_s", 0.85,
  "vtol_alt_kp",            0.20,
  "vtol_hover_collective",  0.75,
  "vtol_test_fixed_collective_enabled", FALSE,
  "vtol_test_fixed_collective", 0.77,
  "vtol_em_ff_enabled",     TRUE,
  "vtol_em_ff_gain",        0.75,
  "vtol_em_ff_max_lead",    0.20,
  "vtol_em_ff_lag_min_s",   0.10,
  "vtol_em_ff_alpha_min",   0.12,
  "vtol_vel_kp",            0.15,
  "vtol_vel_ki",            0.005,
  "vtol_vel_int_lim",       2.0,
  "vtol_vel_int_deadband",  0.5,
  "vtol_max_horiz_accel",   3.0,
  "vtol_max_horiz_speed",   15.0,
  "vtol_max_fwd_pitch",     20.0,
  "vtol_max_bank",          18.0,
  "vtol_pos_kp",            0.08,
  "vtol_pos_ki",            0.002,
  "vtol_pos_int_lim",       3.0,
  "vtol_pos_int_radius",    50.0,
  "vtol_pos_capture_radius",10.0,
  "vtol_khv_capture_mps",   0.5,
  "vtol_physical_alloc_enabled", FALSE,
  "vtol_trans_start_ias",   8.0,
  "vtol_trans_end_ias",     22.0,
  "vtol_nacelle_slew_dps",  5.0,
  "vtol_nacelle_alpha_min", 10.0,
  "vtol_nacelle_sin_floor", 0.10,
  "vtol_bypass_attitude_feedback", FALSE
).

IFC_INIT_STATE().
SET IFC_PHASE TO PHASE_PREARM.
SET IFC_SUBPHASE TO "".
SET IFC_CYCLE_UT TO TIME:SECONDS.
SET IFC_ACTUAL_DT TO 0.05.
SET THROTTLE_CMD TO 0.

SAS OFF.
RCS OFF.

WAIT UNTIL SHIP:UNPACKED.

EM_INIT().

IF SHIP:STATUS = "PRELAUNCH" {
  STAGE.
  WAIT 0.5.
}

VTOL_RESET().
VTOL_DISCOVER().
IF NOT VTOL_DIFF_AVAILABLE {
  CLEARSCREEN.
  PRINT "VTOL AUTO TEST: discovery failed".
  PRINT "Need tagged engines vtol_eng_1..N".
} ELSE {

LOCK THROTTLE TO 1.0.

SET AUTO_MISSION_START_UT TO TIME:SECONDS.
SET AUTO_LAST_LOG_UT TO AUTO_MISSION_START_UT.
SET AUTO_LAST_DISPLAY_UT TO AUTO_MISSION_START_UT.
SET AUTO_LOG_FILE TO _AUTO_NEW_LOG_FILE().

LOG ("# vtol_test_auto craft=" + SHIP:NAME + " UT=" + ROUND(TIME:SECONDS, 1)) TO AUTO_LOG_FILE.
LOG "t_s,phase,dt_s,alt_agl_m,vs_ms,gndspd_ms,pitch_deg,bank_deg,pitch_rate_dps,roll_rate_dps,desired_pitch_deg,desired_bank_deg,roll_cmd,pitch_cmd,thr_input_used,collective,hover_coll,alt_hold,alt_cmd,vel_hold,khv,pos_hold,trans_active,vn_actual_ms,ve_actual_ms,vn_cmd_ms,ve_cmd_ms,pos_err_dist_m,nacelle_cmd_deg,nacelle_est_deg,hover_blend,p_dot_filt,q_dot_filt,roll_d_accel,pitch_d_accel,cos_att_filt,coll_before_corr,coll_after_corr,physical_alloc,upset,alloc_alpha,alloc_shift,limit_span,em_spool_lag_s,em_starving" TO AUTO_LOG_FILE.

ON ABORT {
  SET AUTO_RUNNING TO FALSE.
  _AUTO_LOG_EVENT("ABORT").
  UNLOCK THROTTLE.
  SET SHIP:CONTROL:YAW TO 0.
  SET SHIP:CONTROL:ROLL TO 0.
  SET SHIP:CONTROL:PITCH TO 0.
  VTOL_RELEASE().
  PRINT "VTOL AUTO TEST: ABORT" AT(0, 0).
  PRESERVE.
}

SET VTOL_ALT_HOLD TO FALSE.
SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
SET VTOL_KHV_ACTIVE TO FALSE.
SET VTOL_POS_HOLD_ACTIVE TO FALSE.
SET VTOL_TRANS_ACTIVE TO FALSE.
SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.

LOCAL hold_target_lat IS SHIP:LATITUDE.
LOCAL hold_target_lng IS SHIP:LONGITUDE.
LOCAL hold_target_alt_agl IS GET_AGL().

LOCAL desired_pitch_deg IS 0.0.
LOCAL desired_bank_deg IS 0.0.
LOCAL desired_vn_ms IS 0.0.
LOCAL desired_ve_ms IS 0.0.
LOCAL pilot_roll_cmd_out IS 0.0.
LOCAL pilot_pitch_cmd_out IS 0.0.
LOCAL max_horiz_speed_cfg IS _AUTO_CFG_NUM("vtol_max_horiz_speed", 10.0).
LOCAL max_fwd_pitch_cfg IS _AUTO_CFG_NUM("vtol_max_fwd_pitch", 12.0).
LOCAL max_bank_cfg IS _AUTO_CFG_NUM("vtol_max_bank", 12.0).
IF max_horiz_speed_cfg < 0.5 {
  SET max_horiz_speed_cfg TO 0.5.
}
IF max_fwd_pitch_cfg < 0.5 {
  SET max_fwd_pitch_cfg TO 0.5.
}
IF max_bank_cfg < 0.5 {
  SET max_bank_cfg TO 0.5.
}

_AUTO_SET_PHASE("TAKEOFF_CLIMB").

UNTIL NOT AUTO_RUNNING {
  LOCAL cycle_now_ut IS TIME:SECONDS.
  SET IFC_ACTUAL_DT TO CLAMP(cycle_now_ut - IFC_CYCLE_UT, 0.01, 0.5).
  SET IFC_CYCLE_UT TO cycle_now_ut.

  EM_TICK().

  LOCAL agl_now_m IS GET_AGL().
  LOCAL phase_elapsed_s IS cycle_now_ut - AUTO_PHASE_START_UT.
  LOCAL thr_input_use IS 1.0.

  SET desired_pitch_deg TO 0.0.
  SET desired_bank_deg TO 0.0.
  SET desired_vn_ms TO 0.0.
  SET desired_ve_ms TO 0.0.

  IF AUTO_PHASE_NAME = "TAKEOFF_CLIMB" {
    SET VTOL_ALT_HOLD TO TRUE.
    LOCAL takeoff_alt_kp_cfg IS _AUTO_CFG_NUM("vtol_alt_kp", 0.20).
    IF takeoff_alt_kp_cfg < 0.05 {
      SET takeoff_alt_kp_cfg TO 0.05.
    }
    LOCAL takeoff_cmd_lead_m IS AUTO_TAKEOFF_TARGET_VS_MS / takeoff_alt_kp_cfg.
    SET takeoff_cmd_lead_m TO CLAMP(takeoff_cmd_lead_m, 1.0, 12.0).
    SET VTOL_ALT_CMD TO MIN(AUTO_TAKEOFF_TARGET_ALT_AGL_M, agl_now_m + takeoff_cmd_lead_m).
    // Keep takeoff as a pure vertical profile; horizontal hold engages later.
    SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.
    SET thr_input_use TO 1.0.
    LOCAL takeoff_abort_triggered IS FALSE.
    IF AUTO_ABORT_GUARDS_ENABLED {
      IF phase_elapsed_s > AUTO_TAKEOFF_TIMEOUT_S {
        _AUTO_LOG_EVENT("TAKEOFF_TIMEOUT").
        SET AUTO_RUNNING TO FALSE.
        SET takeoff_abort_triggered TO TRUE.
      } ELSE IF phase_elapsed_s > AUTO_TAKEOFF_RUNAWAY_DELAY_S AND
         VTOL_UPSET_ACTIVE AND
         SHIP:GROUNDSPEED > AUTO_TAKEOFF_RUNAWAY_GNDSPD_MS {
        _AUTO_LOG_EVENT("TAKEOFF_UPSET_RUNAWAY").
        SET AUTO_RUNNING TO FALSE.
        SET takeoff_abort_triggered TO TRUE.
      }
    }
    IF NOT takeoff_abort_triggered AND
       agl_now_m >= AUTO_TAKEOFF_TARGET_ALT_AGL_M - 1.0 AND
       ABS(SHIP:VERTICALSPEED) <= 0.8 AND
       NOT VTOL_UPSET_ACTIVE {
      _AUTO_SET_PHASE("HOVER_BASELINE").
    }
  } ELSE IF AUTO_PHASE_NAME = "HOVER_BASELINE" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO 45.0.
    IF phase_elapsed_s >= 8 {
      _AUTO_SET_PHASE("BANK_STEP").
    }
  } ELSE IF AUTO_PHASE_NAME = "BANK_STEP" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO 45.0.
    SET desired_bank_deg TO 10.0.
    IF phase_elapsed_s >= 8 {
      _AUTO_SET_PHASE("BANK_RETURN").
    }
  } ELSE IF AUTO_PHASE_NAME = "BANK_RETURN" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO 45.0.
    IF phase_elapsed_s >= 8 {
      _AUTO_SET_PHASE("VEL_ACCEL").
    }
  } ELSE IF AUTO_PHASE_NAME = "VEL_ACCEL" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO 45.0.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET desired_vn_ms TO 10.0.
    IF phase_elapsed_s >= 18 {
      _AUTO_SET_PHASE("KHV_STOP").
    }
  } ELSE IF AUTO_PHASE_NAME = "KHV_STOP" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO 45.0.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_KHV_ACTIVE TO TRUE.
    IF phase_elapsed_s >= 20 {
      SET hold_target_lat TO SHIP:LATITUDE.
      SET hold_target_lng TO SHIP:LONGITUDE.
      SET hold_target_alt_agl TO GET_AGL().
      _AUTO_SET_PHASE("POS_PUSH").
    }
  } ELSE IF AUTO_PHASE_NAME = "POS_PUSH" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET desired_bank_deg TO 8.0.
    IF phase_elapsed_s >= 6 {
      SET VTOL_TARGET_LAT TO hold_target_lat.
      SET VTOL_TARGET_LNG TO hold_target_lng.
      SET VTOL_TARGET_ALT TO hold_target_alt_agl.
      SET VTOL_POS_INT_N TO 0.0.
      SET VTOL_POS_INT_E TO 0.0.
      SET VTOL_VEL_INT_N TO 0.0.
      SET VTOL_VEL_INT_E TO 0.0.
      _AUTO_SET_PHASE("POS_RETURN").
    }
  } ELSE IF AUTO_PHASE_NAME = "POS_RETURN" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO TRUE.
    IF phase_elapsed_s >= 30 {
      _AUTO_SET_PHASE("ALLOC_LEGACY").
    }
  } ELSE IF AUTO_PHASE_NAME = "ALLOC_LEGACY" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO TRUE.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.
    IF phase_elapsed_s >= 10 {
      _AUTO_SET_PHASE("ALLOC_PHYSICAL").
    }
  } ELSE IF AUTO_PHASE_NAME = "ALLOC_PHYSICAL" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO TRUE.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO TRUE.
    IF phase_elapsed_s >= 12 {
      _AUTO_SET_PHASE("TRANS_ACCEL").
    }
  } ELSE IF AUTO_PHASE_NAME = "TRANS_ACCEL" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_TRANS_ACTIVE TO TRUE.
    SET desired_vn_ms TO 12.0.
    IF phase_elapsed_s >= 25 {
      _AUTO_SET_PHASE("RECOVER_HOVER").
    }
  } ELSE IF AUTO_PHASE_NAME = "RECOVER_HOVER" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_KHV_ACTIVE TO TRUE.
    IF phase_elapsed_s >= 18 {
      _AUTO_SET_PHASE("COMPLETE_HOLD").
    }
  } ELSE IF AUTO_PHASE_NAME = "COMPLETE_HOLD" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hold_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    IF phase_elapsed_s >= 8 {
      SET AUTO_RUNNING TO FALSE.
    }
  }

  SET pilot_pitch_cmd_out TO 0.0.
  SET pilot_roll_cmd_out TO 0.0.
  IF VTOL_VEL_HOLD_ACTIVE AND NOT VTOL_POS_HOLD_ACTIVE AND NOT VTOL_KHV_ACTIVE {
    SET pilot_pitch_cmd_out TO CLAMP(-desired_vn_ms / max_horiz_speed_cfg, -1, 1).
    SET pilot_roll_cmd_out TO CLAMP(desired_ve_ms / max_horiz_speed_cfg, -1, 1).
  } ELSE IF NOT VTOL_VEL_HOLD_ACTIVE {
    SET pilot_pitch_cmd_out TO CLAMP(-desired_pitch_deg / max_fwd_pitch_cfg, -1, 1).
    SET pilot_roll_cmd_out TO CLAMP(desired_bank_deg / max_bank_cfg, -1, 1).
  }

  SET SHIP:CONTROL:PITCH TO pilot_pitch_cmd_out.
  SET SHIP:CONTROL:ROLL TO pilot_roll_cmd_out.
  SET SHIP:CONTROL:YAW TO 0.
  VTOL_TICK_PREARM().
  IF VTOL_VEL_HOLD_ACTIVE {
    SET desired_bank_deg TO VTOL_PHI_CMD.
    SET desired_pitch_deg TO VTOL_THETA_CMD.
  }

  LOCAL display_due IS cycle_now_ut - AUTO_LAST_DISPLAY_UT >= AUTO_DISPLAY_PERIOD.
  LOCAL log_due IS cycle_now_ut - AUTO_LAST_LOG_UT >= AUTO_LOG_PERIOD.
  LOCAL pitch_now_deg IS 0.0.
  LOCAL bank_now_deg IS 0.0.
  LOCAL roll_rate_now_dps IS 0.0.
  LOCAL pitch_rate_now_dps IS 0.0.
  LOCAL pos_err_dist_now_m IS 0.0.
  IF display_due OR log_due {
    LOCAL facing_fore_vec IS SHIP:FACING:FOREVECTOR.
    LOCAL facing_star_vec IS SHIP:FACING:STARVECTOR.
    LOCAL up_vec_now IS SHIP:UP:VECTOR.
    LOCAL ang_vel_vec IS SHIP:ANGULARVEL.
    SET pitch_now_deg TO 90 - VANG(facing_fore_vec, up_vec_now).
    SET bank_now_deg TO ARCSIN(CLAMP(-VDOT(facing_star_vec, up_vec_now), -1, 1)).
    SET roll_rate_now_dps TO -VDOT(ang_vel_vec, facing_fore_vec) * (180 / CONSTANT:PI).
    SET pitch_rate_now_dps TO VDOT(ang_vel_vec, facing_star_vec) * (180 / CONSTANT:PI).
    LOCAL tgt_geo_now IS LATLNG(VTOL_TARGET_LAT, VTOL_TARGET_LNG).
    LOCAL ship_geo_now IS SHIP:GEOPOSITION.
    SET pos_err_dist_now_m TO GEO_DISTANCE(ship_geo_now, tgt_geo_now).
  }

  LOCAL roll_cmd_out IS VTOL_CMD_ROLL_ACTUAL.
  LOCAL pitch_cmd_out IS VTOL_CMD_PITCH_ACTUAL.

  IF display_due {
    SET AUTO_LAST_DISPLAY_UT TO cycle_now_ut.
    CLEARSCREEN.
    PRINT "VTOL AUTO TEST" AT(0, 0).
    PRINT "phase: " + AUTO_PHASE_NAME AT(0, 1).
    PRINT ("t: " + _AUTO_FMT(cycle_now_ut - AUTO_MISSION_START_UT, 7, 2) +
           "  agl: " + _AUTO_FMT(agl_now_m, 7, 2) +
           "  vs: " + _AUTO_FMT(SHIP:VERTICALSPEED, 7, 2)) AT(0, 2).
    PRINT ("pitch/bank: " + _AUTO_FMT(pitch_now_deg, 7, 2) +
           " / " + _AUTO_FMT(bank_now_deg, 7, 2)) AT(0, 3).
    PRINT ("cmd p/r: " + _AUTO_FMT(pitch_cmd_out, 7, 3) +
           " / " + _AUTO_FMT(roll_cmd_out, 7, 3)) AT(0, 4).
    PRINT ("collective: " + _AUTO_FMT(VTOL_COLLECTIVE, 7, 3) +
           "  nacelle: " + _AUTO_FMT(VTOL_NACELLE_ALPHA_CMD, 7, 2)) AT(0, 5).
    PRINT ("VH:" + VTOL_VEL_HOLD_ACTIVE +
           " KHV:" + VTOL_KHV_ACTIVE +
           " POS:" + VTOL_POS_HOLD_ACTIVE +
           " TR:" + VTOL_TRANS_ACTIVE +
           " PHY:" + ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"]) AT(0, 6).
    PRINT ("VN/VE act: " + _AUTO_FMT(VTOL_VN_ACTUAL, 7, 2) +
           " / " + _AUTO_FMT(VTOL_VE_ACTUAL, 7, 2)) AT(0, 7).
    PRINT ("VN/VE cmd: " + _AUTO_FMT(VTOL_VN_CMD, 7, 2) +
           " / " + _AUTO_FMT(VTOL_VE_CMD, 7, 2)) AT(0, 8).
    PRINT ("pos err: " + _AUTO_FMT(pos_err_dist_now_m, 7, 2) + " m") AT(0, 9).
    PRINT ("ABORT to stop") AT(0, 11).
  }

  IF log_due {
    SET AUTO_LAST_LOG_UT TO cycle_now_ut.
    LOCAL csv_row IS LIST(
      ROUND(cycle_now_ut - AUTO_MISSION_START_UT, 3),
      AUTO_PHASE_NAME,
      ROUND(IFC_ACTUAL_DT, 4),
      ROUND(agl_now_m, 3),
      ROUND(SHIP:VERTICALSPEED, 3),
      ROUND(SHIP:GROUNDSPEED, 3),
      ROUND(pitch_now_deg, 3),
      ROUND(bank_now_deg, 3),
      ROUND(pitch_rate_now_dps, 3),
      ROUND(roll_rate_now_dps, 3),
      ROUND(desired_pitch_deg, 3),
      ROUND(desired_bank_deg, 3),
      ROUND(roll_cmd_out, 4),
      ROUND(pitch_cmd_out, 4),
      ROUND(VTOL_THR_INPUT_USED, 4),
      ROUND(VTOL_COLLECTIVE, 4),
      ROUND(VTOL_HOVER_COLLECTIVE, 4),
      VTOL_ALT_HOLD,
      ROUND(VTOL_ALT_CMD, 3),
      VTOL_VEL_HOLD_ACTIVE,
      VTOL_KHV_ACTIVE,
      VTOL_POS_HOLD_ACTIVE,
      VTOL_TRANS_ACTIVE,
      ROUND(VTOL_VN_ACTUAL, 4),
      ROUND(VTOL_VE_ACTUAL, 4),
      ROUND(VTOL_VN_CMD, 4),
      ROUND(VTOL_VE_CMD, 4),
      ROUND(pos_err_dist_now_m, 4),
      ROUND(VTOL_NACELLE_ALPHA_CMD, 4),
      ROUND(VTOL_NACELLE_ALPHA_EST, 4),
      ROUND(VTOL_HOVER_BLEND, 4),
      ROUND(VTOL_DIAG_P_DOT_FILT, 4),
      ROUND(VTOL_DIAG_Q_DOT_FILT, 4),
      ROUND(VTOL_DIAG_ROLL_D_ACCEL, 4),
      ROUND(VTOL_DIAG_PITCH_D_ACCEL, 4),
      ROUND(VTOL_DIAG_COS_ATT_FILT, 5),
      ROUND(VTOL_DIAG_COLL_BEFORE_CORR, 5),
      ROUND(VTOL_DIAG_COLL_AFTER_CORR, 5),
      VTOL_DIAG_PHYSICAL_ALLOC_USED,
      VTOL_UPSET_ACTIVE,
      ROUND(VTOL_ALLOC_ALPHA, 4),
      ROUND(VTOL_ALLOC_SHIFT, 4),
      ROUND(VTOL_DIAG_LIMIT_SPAN, 5),
      ROUND(TELEM_EM_WORST_SPOOL_LAG, 4),
      TELEM_EM_STARVING
    ).
    LOG csv_row:JOIN(",") TO AUTO_LOG_FILE.
  }

  WAIT IFC_LOOP_DT.
}

_AUTO_LOG_EVENT("COMPLETE").
LOG ("# end t=" + ROUND(TIME:SECONDS - AUTO_MISSION_START_UT, 2)) TO AUTO_LOG_FILE.
UNLOCK THROTTLE.
SET SHIP:CONTROL:YAW TO 0.
SET SHIP:CONTROL:ROLL TO 0.
SET SHIP:CONTROL:PITCH TO 0.
VTOL_RELEASE().
CLEARSCREEN.
PRINT "VTOL AUTO TEST COMPLETE".
PRINT "Log: " + AUTO_LOG_FILE.
}
