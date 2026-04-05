@LAZYGLOBAL OFF.

// ============================================================
// vtol_test_auto.ks - autonomous VTOL hover/landing flight test
//
// Mission profile:
// 1) vertical takeoff and climb to target AGL
// 2) hold position over launch spot at target altitude
// 3) descend and land while maintaining position hold
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
GLOBAL AUTO_HOVER_HOLD_TIME_S IS 15.0.
GLOBAL AUTO_HOVER_POS_TOL_M IS 1.0.
GLOBAL AUTO_HOVER_TIMEOUT_S IS 60.0.
GLOBAL AUTO_LAND_FAST_DESCENT_MPS IS 1.4.
GLOBAL AUTO_LAND_GENTLE_DESCENT_MPS IS 0.35.
GLOBAL AUTO_LAND_FLARE_START_AGL_M IS 10.0.
GLOBAL AUTO_LAND_FLARE_END_AGL_M IS 3.0.
GLOBAL AUTO_LAND_FINAL_AGL_M IS 0.6.
GLOBAL AUTO_LAND_TOUCHDOWN_VS_MS IS 0.6.
GLOBAL AUTO_LAND_TOUCHDOWN_GNDSPD_MS IS 1.2.
GLOBAL AUTO_LAND_STATUS_TOUCHDOWN_AGL_M IS 2.0.
GLOBAL AUTO_LAND_TOUCHDOWN_ARM_S IS 8.0.
GLOBAL AUTO_LAND_FINAL_CONFIRM_AGL_M IS 0.25.
GLOBAL AUTO_LAND_FINAL_CONFIRM_VS_MS IS 0.25.
GLOBAL AUTO_LAND_FINAL_CONFIRM_GNDSPD_MS IS 0.6.
GLOBAL AUTO_LAND_ALT_KP_FAST IS 0.50.
GLOBAL AUTO_LAND_ALT_KP_GENTLE IS 0.24.
GLOBAL AUTO_LAND_MAX_VS_FAST_MS IS 2.0.
GLOBAL AUTO_LAND_MAX_VS_GENTLE_MS IS 0.45.
GLOBAL AUTO_LAND_VS_KP IS 0.30.
GLOBAL AUTO_LAND_VS_KI IS 0.05.
GLOBAL AUTO_LAND_VS_CMD_DN_SLEW_MPS2 IS 1.8.
GLOBAL AUTO_LAND_SETTLE_TIME_S IS 2.0.
GLOBAL AUTO_LAND_TIMEOUT_S IS 120.0.
GLOBAL AUTO_TAKEOFF_SETTLE_GNDSPD_MS IS 1.0.
GLOBAL AUTO_TAKEOFF_SETTLE_PITCH_DEG IS 6.0.
GLOBAL AUTO_TAKEOFF_SETTLE_BANK_DEG IS 6.0.
GLOBAL AUTO_TAKEOFF_SETTLE_RATE_DPS IS 6.0.
GLOBAL AUTO_HOVER_SETTLE_MIN_S IS 3.0.
GLOBAL AUTO_HOVER_SETTLE_GNDSPD_MS IS 1.2.
GLOBAL AUTO_HOVER_SETTLE_PITCH_DEG IS 8.0.
GLOBAL AUTO_HOVER_SETTLE_BANK_DEG IS 8.0.
GLOBAL AUTO_HOVER_SETTLE_RATE_DPS IS 8.0.
GLOBAL AUTO_YAW_CAS_HDG_KP IS 0.35.          // (deg/s) per deg heading error
GLOBAL AUTO_YAW_CAS_HDG_KI IS 0.002.         // (deg/s) per deg*s heading error
GLOBAL AUTO_YAW_CAS_RATE_KP IS 0.025.        // command per (deg/s) yaw-rate error
GLOBAL AUTO_YAW_CAS_RATE_KI IS 0.004.        // command per (deg/s)*s yaw-rate error
GLOBAL AUTO_YAW_CAS_RATE_CMD_MAX_DPS IS 4.0.
GLOBAL AUTO_YAW_CAS_CMD_MAX IS 0.22.
GLOBAL AUTO_YAW_CAS_HDG_DB_DEG IS 0.40.
GLOBAL AUTO_YAW_CAS_RATE_DB_DPS IS 0.20.
GLOBAL AUTO_YAW_CAS_HDG_INT_LIM IS 40.0.
GLOBAL AUTO_YAW_CAS_RATE_INT_LIM IS 8.0.

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

FUNCTION _AUTO_ANGLE_ERR {
  PARAMETER target_deg, actual_deg.
  LOCAL err_deg IS target_deg - actual_deg.
  UNTIL err_deg <= 180 {
    SET err_deg TO err_deg - 360.
  }
  UNTIL err_deg >= -180 {
    SET err_deg TO err_deg + 360.
  }
  RETURN err_deg.
}

FUNCTION _AUTO_COMPASS_HDG_NOW {
  // Compute heading from live ship vectors.
  // Do not use GET_COMPASS_HDG() here because it reads IFC cached vectors
  // that are not updated by this standalone test loop.
  LOCAL fwd_v IS SHIP:FACING:FOREVECTOR.
  LOCAL up_v IS SHIP:UP:VECTOR.
  LOCAL north_v IS SHIP:NORTH:VECTOR.
  LOCAL fwd_h IS fwd_v - up_v * VDOT(fwd_v, up_v).
  IF VDOT(fwd_h, fwd_h) < 0.0001 {
    RETURN 0.0.
  }
  SET fwd_h TO fwd_h:NORMALIZED.
  LOCAL north_u IS north_v.
  IF VDOT(north_u, north_u) < 0.0001 {
    SET north_u TO V(0, 0, 1).
  }
  SET north_u TO north_u:NORMALIZED.
  LOCAL east_v IS VCRS(up_v, north_u).
  IF VDOT(east_v, east_v) < 0.0001 {
    SET east_v TO VCRS(up_v, fwd_h).
  }
  IF VDOT(east_v, east_v) < 0.0001 {
    SET east_v TO V(1, 0, 0).
  }
  SET east_v TO east_v:NORMALIZED.
  LOCAL hdg_raw IS VANG(fwd_h, north_u).
  LOCAL hdg_out IS hdg_raw.
  IF VDOT(fwd_h, east_v) < 0 {
    SET hdg_out TO 360 - hdg_raw.
  }
  RETURN MOD(hdg_out + 360, 360).
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
  "vtol_yaw_gain",          5.0,
  "vtol_yaw_sign",          1,
  "vtol_yaw_cmd_deadband",  0.03,
  "vtol_yaw_min_deflection_deg", 0.0,
  "vtol_yaw_cas_hdg_kp",    AUTO_YAW_CAS_HDG_KP,
  "vtol_yaw_cas_hdg_ki",    AUTO_YAW_CAS_HDG_KI,
  "vtol_yaw_cas_rate_kp",   AUTO_YAW_CAS_RATE_KP,
  "vtol_yaw_cas_rate_ki",   AUTO_YAW_CAS_RATE_KI,
  "vtol_yaw_cas_rate_cmd_max_dps", AUTO_YAW_CAS_RATE_CMD_MAX_DPS,
  "vtol_yaw_cas_cmd_max",   AUTO_YAW_CAS_CMD_MAX,
  "vtol_yaw_cas_hdg_db_deg", AUTO_YAW_CAS_HDG_DB_DEG,
  "vtol_yaw_cas_rate_db_dps", AUTO_YAW_CAS_RATE_DB_DPS,
  "vtol_yaw_cas_hdg_int_lim", AUTO_YAW_CAS_HDG_INT_LIM,
  "vtol_yaw_cas_rate_int_lim", AUTO_YAW_CAS_RATE_INT_LIM,
  "vtol_srv_speed",         2.0,
  "vtol_srv_yaw_speed",     6.0,
  "vtol_roll_gain",         0.25,
  "vtol_pitch_gain",        0.30,
  // XV-3-C verified pitch polarity for this test article.
  "vtol_pitch_mix_sign",    1,
  "vtol_level_roll_kp",     0.10,
  "vtol_level_roll_kd",     0.03,
  "vtol_level_roll_ki",     0.010,
  "vtol_level_pitch_kp",    0.10,
  "vtol_level_pitch_kd",    0.08,
  "vtol_level_pitch_ki",    0.015,
  "vtol_level_roll_att2rate_kp",  0.8,
  "vtol_level_pitch_att2rate_kp", 0.48,
  "vtol_level_roll_att2rate_ki",  0.00,
  "vtol_level_pitch_att2rate_ki", 0.00,
  "vtol_level_roll_rate_kp",      0.030,
  "vtol_level_pitch_rate_kp",     0.040,
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
  "vtol_cmd_phys_slew_min",     0.45,
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
  "vtol_upset_cmd_pitch_max",   0.85,
  "vtol_upset_roll_rate_kp",    0.030,
  "vtol_upset_pitch_rate_kp",   0.055,
  "vtol_upset_pitch_phys_slew_scale", 3.0,
  "vtol_upset_pitch_slew_bypass", TRUE,
  "vtol_upset_diff_atten_min",  0.55,
  "vtol_upset_engine_limit_floor", 0.02,
  "vtol_upset_collective_cap",  0.76,
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
  "vtol_trim_rate",             0.004,
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
  "vtol_vel_kp",            0.22,
  "vtol_vel_ki",            0.010,
  "vtol_vel_int_lim",       3.0,
  "vtol_vel_int_deadband",  0.2,
  "vtol_max_horiz_accel",   2.1,
  "vtol_max_horiz_speed",   8.0,
  "vtol_max_fwd_pitch",     14.5,
  "vtol_max_bank",          14.5,
  "vtol_pos_kp",            0.16,
  "vtol_pos_ki",            0.006,
  "vtol_pos_int_lim",       5.5,
  "vtol_pos_int_radius",    50.0,
  "vtol_pos_capture_radius",4.0,
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

// Ensure consistent terminal visibility/readability even when launched
// outside the dedicated boot script.
LOCAL test_proc_module IS CORE:PART:GETMODULE("kOSProcessor").
IF test_proc_module <> 0 {
  IF test_proc_module:HASEVENT("Open Terminal") {
    test_proc_module:DOEVENT("Open Terminal").
  }
}
SET TERMINAL:VISUALBEEP TO FALSE.
SET TERMINAL:CHARHEIGHT TO 20.

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
LOG "t_s,phase,dt_s,alt_agl_m,vs_ms,gndspd_ms,pitch_deg,bank_deg,pitch_rate_dps,roll_rate_dps,yaw_hdg_deg,yaw_err_deg,yaw_rate_dps,yaw_rate_cmd_dps,yaw_rate_err_dps,yaw_cmd_out,yaw_cmd_apply,srv_pos_span_deg,srv_cmd_span_deg,srv_mix_sum,desired_pitch_deg,desired_bank_deg,roll_cmd,pitch_cmd,thr_input_used,collective,hover_coll,alt_hold,alt_cmd,vel_hold,khv,pos_hold,trans_active,vn_actual_ms,ve_actual_ms,vn_cmd_ms,ve_cmd_ms,pos_err_dist_m,nacelle_cmd_deg,nacelle_est_deg,hover_blend,p_dot_filt,q_dot_filt,roll_d_accel,pitch_d_accel,cos_att_filt,coll_before_corr,coll_after_corr,physical_alloc,upset,alloc_alpha,alloc_shift,limit_span,em_spool_lag_s,eff_spool_lag_s,em_starving" TO AUTO_LOG_FILE.
LOCAL srv_mix_sum_init IS 0.0.
LOCAL init_i IS 0.
UNTIL init_i >= VTOL_YAW_SRV_MIX:LENGTH {
  SET srv_mix_sum_init TO srv_mix_sum_init + ABS(VTOL_YAW_SRV_MIX[init_i]).
  SET init_i TO init_i + 1.
}
_AUTO_LOG_EVENT(
  "vtol_init srv_avail=" + VTOL_SRV_AVAIL +
  " srv_n=" + VTOL_SRV_LIST:LENGTH +
  " yaw_mix_sum=" + ROUND(srv_mix_sum_init, 2) +
  " yaw_gain=" + ROUND(_AUTO_CFG_NUM("vtol_yaw_gain", VTOL_YAW_SRV_GAIN), 2) +
  " yaw_sign=" + ROUND(_AUTO_CFG_NUM("vtol_yaw_sign", 1), 0)
).
LOCAL srv_tag_hits IS 0.
LOCAL srv_tag_idx IS 1.
LOCAL srv_tag_prefix_dbg IS "vtol_srv".
IF ACTIVE_AIRCRAFT:HASKEY("vtol_srv_tag_prefix") {
  SET srv_tag_prefix_dbg TO ACTIVE_AIRCRAFT["vtol_srv_tag_prefix"].
}
UNTIL srv_tag_idx > VTOL_ENG_LIST:LENGTH {
  IF SHIP:PARTSTAGGED(srv_tag_prefix_dbg + "_" + srv_tag_idx):LENGTH > 0 {
    SET srv_tag_hits TO srv_tag_hits + 1.
  }
  SET srv_tag_idx TO srv_tag_idx + 1.
}
_AUTO_LOG_EVENT(
  "vtol_tags prefix=" + srv_tag_prefix_dbg +
  " hits=" + srv_tag_hits + "/" + VTOL_ENG_LIST:LENGTH
).
_AUTO_LOG_EVENT(VTOL_IR_DIAG()).

ON ABORT {
  SET AUTO_RUNNING TO FALSE.
  _AUTO_LOG_EVENT("ABORT").
  LOCK THROTTLE TO 0.
  WAIT 0.1.
  VTOL_CLEAR_YAW_INPUT_OVERRIDE().
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
VTOL_CLEAR_YAW_INPUT_OVERRIDE().
VTOL_SET_YAW_INPUT_OVERRIDE(0).

LOCAL hold_target_lat IS SHIP:LATITUDE.
LOCAL hold_target_lng IS SHIP:LONGITUDE.
LOCAL hold_target_hdg_deg IS _AUTO_COMPASS_HDG_NOW().
LOCAL hover_target_alt_agl IS AUTO_TAKEOFF_TARGET_ALT_AGL_M.
LOCAL land_cmd_agl_track IS hover_target_alt_agl.
LOCAL land_touchdown_start_ut IS -1.
LOCAL hover_on_target_start_ut IS -1.
LOCAL land_timeout_logged IS FALSE.

LOCAL desired_pitch_deg IS 0.0.
LOCAL desired_bank_deg IS 0.0.
LOCAL desired_vn_ms IS 0.0.
LOCAL desired_ve_ms IS 0.0.
LOCAL pilot_roll_cmd_out IS 0.0.
LOCAL pilot_pitch_cmd_out IS 0.0.
LOCAL yaw_hdg_int_deg_s IS 0.0.
LOCAL yaw_rate_int_dps_s IS 0.0.
LOCAL yaw_rate_cmd_dps_diag IS 0.0.
LOCAL yaw_rate_err_dps_diag IS 0.0.
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

  SET desired_pitch_deg TO 0.0.
  SET desired_bank_deg TO 0.0.
  SET desired_vn_ms TO 0.0.
  SET desired_ve_ms TO 0.0.

  LOCAL hold_geo IS LATLNG(hold_target_lat, hold_target_lng).
  LOCAL pos_err_for_phase_m IS GEO_DISTANCE(SHIP:GEOPOSITION, hold_geo).
  LOCAL phase_fore_vec IS SHIP:FACING:FOREVECTOR.
  LOCAL phase_star_vec IS SHIP:FACING:STARVECTOR.
  LOCAL phase_top_vec IS SHIP:FACING:TOPVECTOR.
  LOCAL phase_up_vec IS SHIP:UP:VECTOR.
  LOCAL phase_ang_vel IS SHIP:ANGULARVEL.
  LOCAL phase_pitch_deg IS 90 - VANG(phase_fore_vec, phase_up_vec).
  LOCAL phase_bank_deg IS ARCSIN(CLAMP(-VDOT(phase_star_vec, phase_up_vec), -1, 1)).
  LOCAL phase_roll_rate_dps IS -VDOT(phase_ang_vel, phase_fore_vec) * (180 / CONSTANT:PI).
  LOCAL phase_pitch_rate_dps IS VDOT(phase_ang_vel, phase_star_vec) * (180 / CONSTANT:PI).
  LOCAL phase_gndspd_ms IS SHIP:GROUNDSPEED.
  LOCAL yaw_hdg_now_deg IS _AUTO_COMPASS_HDG_NOW().
  LOCAL yaw_err_deg IS _AUTO_ANGLE_ERR(hold_target_hdg_deg, yaw_hdg_now_deg).
  // Sign convention: positive yaw_rate_dps must correspond to increasing compass heading.
  LOCAL yaw_rate_dps IS VDOT(phase_ang_vel, phase_top_vec) * (180 / CONSTANT:PI).
  LOCAL yaw_rate_cmd_dps IS 0.0.
  LOCAL yaw_rate_err_dps IS 0.0.
  LOCAL yaw_cmd_out IS 0.0.
  LOCAL yaw_hold_active IS FALSE.
  IF AUTO_PHASE_NAME = "TAKEOFF_CLIMB" OR
     AUTO_PHASE_NAME = "HOVER_HOLD" OR
     AUTO_PHASE_NAME = "LAND_DESCENT" {
    SET yaw_hold_active TO TRUE.
  }
  IF yaw_hold_active {
    IF VTOL_UPSET_ACTIVE {
      // Keep upset recovery focused on pitch/roll stabilization.
      SET yaw_hdg_int_deg_s TO 0.0.
      SET yaw_rate_int_dps_s TO 0.0.
      SET yaw_cmd_out TO 0.0.
    } ELSE {
      LOCAL hdg_kp_use IS _AUTO_CFG_NUM("vtol_yaw_cas_hdg_kp", AUTO_YAW_CAS_HDG_KP).
      LOCAL hdg_ki_use IS _AUTO_CFG_NUM("vtol_yaw_cas_hdg_ki", AUTO_YAW_CAS_HDG_KI).
      LOCAL rate_kp_use IS _AUTO_CFG_NUM("vtol_yaw_cas_rate_kp", AUTO_YAW_CAS_RATE_KP).
      LOCAL rate_ki_use IS _AUTO_CFG_NUM("vtol_yaw_cas_rate_ki", AUTO_YAW_CAS_RATE_KI).
      LOCAL rate_cmd_max_use IS _AUTO_CFG_NUM(
        "vtol_yaw_cas_rate_cmd_max_dps",
        AUTO_YAW_CAS_RATE_CMD_MAX_DPS
      ).
      LOCAL cmd_max_use IS _AUTO_CFG_NUM("vtol_yaw_cas_cmd_max", AUTO_YAW_CAS_CMD_MAX).
      LOCAL hdg_db_use IS _AUTO_CFG_NUM("vtol_yaw_cas_hdg_db_deg", AUTO_YAW_CAS_HDG_DB_DEG).
      LOCAL rate_db_use IS _AUTO_CFG_NUM("vtol_yaw_cas_rate_db_dps", AUTO_YAW_CAS_RATE_DB_DPS).
      LOCAL hdg_int_lim_use IS _AUTO_CFG_NUM("vtol_yaw_cas_hdg_int_lim", AUTO_YAW_CAS_HDG_INT_LIM).
      LOCAL rate_int_lim_use IS _AUTO_CFG_NUM("vtol_yaw_cas_rate_int_lim", AUTO_YAW_CAS_RATE_INT_LIM).
      IF rate_cmd_max_use < 0.2 { SET rate_cmd_max_use TO 0.2. }
      IF cmd_max_use < 0.05 { SET cmd_max_use TO 0.05. }
      IF hdg_int_lim_use < 1.0 { SET hdg_int_lim_use TO 1.0. }
      IF rate_int_lim_use < 0.2 { SET rate_int_lim_use TO 0.2. }
      LOCAL yaw_err_use_deg IS yaw_err_deg.
      IF ABS(yaw_err_use_deg) < hdg_db_use {
        SET yaw_err_use_deg TO 0.0.
      }

      // Outer loop: heading error -> desired yaw rate.
      LOCAL yaw_rate_cmd_unsat IS yaw_err_use_deg * hdg_kp_use + yaw_hdg_int_deg_s * hdg_ki_use.
      LOCAL outer_sat_hi IS yaw_rate_cmd_unsat >= rate_cmd_max_use AND yaw_err_use_deg > 0.
      LOCAL outer_sat_lo IS yaw_rate_cmd_unsat <= -rate_cmd_max_use AND yaw_err_use_deg < 0.
      LOCAL outer_integrate IS TRUE.
      IF outer_sat_hi OR outer_sat_lo { SET outer_integrate TO FALSE. }
      IF outer_integrate {
        SET yaw_hdg_int_deg_s TO CLAMP(
          yaw_hdg_int_deg_s + yaw_err_use_deg * IFC_ACTUAL_DT,
          -hdg_int_lim_use,
          hdg_int_lim_use
        ).
      } ELSE {
        LOCAL hdg_unwind IS CLAMP(1.0 - 0.75 * IFC_ACTUAL_DT, 0, 1).
        SET yaw_hdg_int_deg_s TO yaw_hdg_int_deg_s * hdg_unwind.
      }
      SET yaw_rate_cmd_dps TO CLAMP(
        yaw_err_use_deg * hdg_kp_use + yaw_hdg_int_deg_s * hdg_ki_use,
        -rate_cmd_max_use,
        rate_cmd_max_use
      ).

      // Inner loop: yaw-rate error -> nacelle yaw command.
      SET yaw_rate_err_dps TO yaw_rate_cmd_dps - yaw_rate_dps.
      IF ABS(yaw_rate_err_dps) < rate_db_use {
        SET yaw_rate_err_dps TO 0.0.
      }
      LOCAL yaw_cmd_unsat IS yaw_rate_err_dps * rate_kp_use + yaw_rate_int_dps_s * rate_ki_use.
      LOCAL inner_sat_hi IS yaw_cmd_unsat >= cmd_max_use AND yaw_rate_err_dps > 0.
      LOCAL inner_sat_lo IS yaw_cmd_unsat <= -cmd_max_use AND yaw_rate_err_dps < 0.
      LOCAL inner_integrate IS TRUE.
      IF inner_sat_hi OR inner_sat_lo { SET inner_integrate TO FALSE. }
      IF inner_integrate {
        SET yaw_rate_int_dps_s TO CLAMP(
          yaw_rate_int_dps_s + yaw_rate_err_dps * IFC_ACTUAL_DT,
          -rate_int_lim_use,
          rate_int_lim_use
        ).
      } ELSE {
        LOCAL rate_unwind IS CLAMP(1.0 - 1.25 * IFC_ACTUAL_DT, 0, 1).
        SET yaw_rate_int_dps_s TO yaw_rate_int_dps_s * rate_unwind.
      }
      SET yaw_cmd_out TO CLAMP(
        yaw_rate_err_dps * rate_kp_use + yaw_rate_int_dps_s * rate_ki_use,
        -cmd_max_use,
        cmd_max_use
      ).
    }
  } ELSE {
    SET yaw_hdg_int_deg_s TO 0.0.
    SET yaw_rate_int_dps_s TO 0.0.
  }
  SET yaw_rate_cmd_dps_diag TO yaw_rate_cmd_dps.
  SET yaw_rate_err_dps_diag TO yaw_rate_err_dps.
  // Do not apply yaw sign here; _VTOL_APPLY_SERVOS() applies vtol_yaw_sign once.
  LOCAL yaw_cmd_apply IS CLAMP(yaw_cmd_out, -1, 1).
  LOCAL takeoff_stable IS
    phase_gndspd_ms <= AUTO_TAKEOFF_SETTLE_GNDSPD_MS AND
    ABS(phase_pitch_deg) <= AUTO_TAKEOFF_SETTLE_PITCH_DEG AND
    ABS(phase_bank_deg) <= AUTO_TAKEOFF_SETTLE_BANK_DEG AND
    ABS(phase_roll_rate_dps) <= AUTO_TAKEOFF_SETTLE_RATE_DPS AND
    ABS(phase_pitch_rate_dps) <= AUTO_TAKEOFF_SETTLE_RATE_DPS.

  IF AUTO_PHASE_NAME = "TAKEOFF_CLIMB" {
    SET VTOL_ALT_HOLD TO TRUE.
    LOCAL takeoff_alt_kp_cfg IS _AUTO_CFG_NUM("vtol_alt_kp", 0.20).
    IF takeoff_alt_kp_cfg < 0.05 {
      SET takeoff_alt_kp_cfg TO 0.05.
    }
    LOCAL takeoff_cmd_lead_m IS AUTO_TAKEOFF_TARGET_VS_MS / takeoff_alt_kp_cfg.
    SET takeoff_cmd_lead_m TO CLAMP(takeoff_cmd_lead_m, 1.0, 12.0).
    SET VTOL_ALT_CMD TO MIN(AUTO_TAKEOFF_TARGET_ALT_AGL_M, agl_now_m + takeoff_cmd_lead_m).
    // Keep horizontal surface velocity damped during climb.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO TRUE.
    SET VTOL_POS_HOLD_ACTIVE TO FALSE.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    SET VTOL_TARGET_LAT TO hold_target_lat.
    SET VTOL_TARGET_LNG TO hold_target_lng.
    SET VTOL_TARGET_ALT TO AUTO_TAKEOFF_TARGET_ALT_AGL_M.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.
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
       NOT VTOL_UPSET_ACTIVE AND
       takeoff_stable {
      SET hover_target_alt_agl TO AUTO_TAKEOFF_TARGET_ALT_AGL_M.
      SET VTOL_TARGET_LAT TO hold_target_lat.
      SET VTOL_TARGET_LNG TO hold_target_lng.
      SET VTOL_TARGET_ALT TO hover_target_alt_agl.
      SET VTOL_POS_INT_N TO 0.0.
      SET VTOL_POS_INT_E TO 0.0.
      SET VTOL_VEL_INT_N TO 0.0.
      SET VTOL_VEL_INT_E TO 0.0.
      SET hover_on_target_start_ut TO -1.
      _AUTO_SET_PHASE("HOVER_HOLD").
    }
  } ELSE IF AUTO_PHASE_NAME = "HOVER_HOLD" {
    SET VTOL_ALT_HOLD TO TRUE.
    SET VTOL_ALT_CMD TO hover_target_alt_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    SET VTOL_TARGET_LAT TO hold_target_lat.
    SET VTOL_TARGET_LNG TO hold_target_lng.
    SET VTOL_TARGET_ALT TO hover_target_alt_agl.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.

    LOCAL hover_settled IS
      phase_elapsed_s >= AUTO_HOVER_SETTLE_MIN_S AND
      phase_gndspd_ms <= AUTO_HOVER_SETTLE_GNDSPD_MS AND
      ABS(phase_pitch_deg) <= AUTO_HOVER_SETTLE_PITCH_DEG AND
      ABS(phase_bank_deg) <= AUTO_HOVER_SETTLE_BANK_DEG AND
      ABS(phase_roll_rate_dps) <= AUTO_HOVER_SETTLE_RATE_DPS AND
      ABS(phase_pitch_rate_dps) <= AUTO_HOVER_SETTLE_RATE_DPS AND
      NOT VTOL_UPSET_ACTIVE.

    IF hover_settled {
      SET VTOL_KHV_ACTIVE TO FALSE.
      SET VTOL_POS_HOLD_ACTIVE TO TRUE.
    } ELSE {
      // During early hover capture, prioritize killing horizontal velocity
      // before enabling position return commands.
      SET VTOL_KHV_ACTIVE TO TRUE.
      SET VTOL_POS_HOLD_ACTIVE TO FALSE.
      SET VTOL_POS_INT_N TO 0.0.
      SET VTOL_POS_INT_E TO 0.0.
    }

    LOCAL hover_on_target_now IS pos_err_for_phase_m <= AUTO_HOVER_POS_TOL_M.
    IF hover_on_target_now {
      IF hover_on_target_start_ut < 0 {
        SET hover_on_target_start_ut TO cycle_now_ut.
        _AUTO_LOG_EVENT("HOVER_TARGET_WINDOW_START").
      }
    } ELSE {
      IF hover_on_target_start_ut >= 0 {
        _AUTO_LOG_EVENT("HOVER_TARGET_WINDOW_RESET err_m=" + ROUND(pos_err_for_phase_m, 2)).
      }
      SET hover_on_target_start_ut TO -1.
    }

    LOCAL hover_on_target_hold_s IS 0.0.
    IF hover_on_target_start_ut >= 0 {
      SET hover_on_target_hold_s TO cycle_now_ut - hover_on_target_start_ut.
    }

    IF hover_on_target_hold_s >= AUTO_HOVER_HOLD_TIME_S {
      _AUTO_LOG_EVENT("HOVER_TARGET_HOLD_COMPLETE").
      SET land_cmd_agl_track TO agl_now_m.
      SET land_touchdown_start_ut TO -1.
      SET land_timeout_logged TO FALSE.
      _AUTO_SET_PHASE("LAND_DESCENT").
    } ELSE IF phase_elapsed_s >= AUTO_HOVER_TIMEOUT_S {
      _AUTO_LOG_EVENT("HOVER_TIMEOUT_BEGIN_LAND").
      SET hover_on_target_start_ut TO -1.
      SET land_cmd_agl_track TO agl_now_m.
      SET land_touchdown_start_ut TO -1.
      SET land_timeout_logged TO FALSE.
      _AUTO_SET_PHASE("LAND_DESCENT").
    }
  } ELSE IF AUTO_PHASE_NAME = "LAND_DESCENT" {
    // Two-stage descent: faster high up, then gentle sink in the flare band.
    LOCAL flare_denom_m IS AUTO_LAND_FLARE_START_AGL_M - AUTO_LAND_FLARE_END_AGL_M.
    IF flare_denom_m < 0.5 { SET flare_denom_m TO 0.5. }
    LOCAL flare_blend IS CLAMP((agl_now_m - AUTO_LAND_FLARE_END_AGL_M) / flare_denom_m, 0, 1).
    LOCAL land_descent_rate_mps IS AUTO_LAND_GENTLE_DESCENT_MPS +
      (AUTO_LAND_FAST_DESCENT_MPS - AUTO_LAND_GENTLE_DESCENT_MPS) * flare_blend.
    LOCAL land_alt_kp_use IS AUTO_LAND_ALT_KP_GENTLE +
      (AUTO_LAND_ALT_KP_FAST - AUTO_LAND_ALT_KP_GENTLE) * flare_blend.
    LOCAL land_max_vs_use IS AUTO_LAND_MAX_VS_GENTLE_MS +
      (AUTO_LAND_MAX_VS_FAST_MS - AUTO_LAND_MAX_VS_GENTLE_MS) * flare_blend.
    IF land_descent_rate_mps < 0.15 { SET land_descent_rate_mps TO 0.15. }
    IF land_alt_kp_use < 0.05 { SET land_alt_kp_use TO 0.05. }
    IF land_max_vs_use < 0.25 { SET land_max_vs_use TO 0.25. }

    // Strengthen VS loop in landing so it tracks descent command instead of hovering.
    SET ACTIVE_AIRCRAFT["vtol_alt_kp"] TO land_alt_kp_use.
    SET ACTIVE_AIRCRAFT["vtol_max_vs"] TO land_max_vs_use.
    SET ACTIVE_AIRCRAFT["vtol_vs_kp"] TO AUTO_LAND_VS_KP.
    SET ACTIVE_AIRCRAFT["vtol_vs_ki"] TO AUTO_LAND_VS_KI.
    SET ACTIVE_AIRCRAFT["vtol_vs_cmd_dn_slew_mps2"] TO AUTO_LAND_VS_CMD_DN_SLEW_MPS2.

    // Integrate landing altitude schedule so flare-rate changes are smooth.
    SET land_cmd_agl_track TO land_cmd_agl_track - land_descent_rate_mps * IFC_ACTUAL_DT.
    IF land_cmd_agl_track < AUTO_LAND_FINAL_AGL_M {
      SET land_cmd_agl_track TO AUTO_LAND_FINAL_AGL_M.
    }

    SET VTOL_ALT_HOLD TO TRUE.
    LOCAL land_err_margin_m IS land_descent_rate_mps / land_alt_kp_use.
    IF land_err_margin_m < 0.8 { SET land_err_margin_m TO 0.8. }
    LOCAL land_cmd_agl IS MIN(land_cmd_agl_track, agl_now_m - land_err_margin_m).
    IF land_cmd_agl < AUTO_LAND_FINAL_AGL_M { SET land_cmd_agl TO AUTO_LAND_FINAL_AGL_M. }
    SET VTOL_ALT_CMD TO land_cmd_agl.
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
    SET VTOL_KHV_ACTIVE TO FALSE.
    SET VTOL_POS_HOLD_ACTIVE TO TRUE.
    SET VTOL_TRANS_ACTIVE TO FALSE.
    SET VTOL_TARGET_LAT TO hold_target_lat.
    SET VTOL_TARGET_LNG TO hold_target_lng.
    SET VTOL_TARGET_ALT TO land_cmd_agl.
    SET ACTIVE_AIRCRAFT["vtol_physical_alloc_enabled"] TO FALSE.

    LOCAL touchdown_detected IS FALSE.
    LOCAL ship_status IS "".
    LOCAL touchdown_status_based IS FALSE.
    LOCAL touchdown_kinematic IS FALSE.
    LOCAL grounded_confirmed IS FALSE.
    IF SHIP:HASSUFFIX("STATUS") {
      SET ship_status TO SHIP:STATUS.
    }
    IF phase_elapsed_s >= AUTO_LAND_TOUCHDOWN_ARM_S {
      IF ship_status = "LANDED" OR ship_status = "SPLASHED" {
        IF agl_now_m <= AUTO_LAND_STATUS_TOUCHDOWN_AGL_M AND
           SHIP:GROUNDSPEED <= AUTO_LAND_TOUCHDOWN_GNDSPD_MS * 1.5 {
          SET touchdown_status_based TO TRUE.
        } ELSE IF agl_now_m > AUTO_LAND_STATUS_TOUCHDOWN_AGL_M + 5 {
          _AUTO_LOG_EVENT(
            "LAND_STATUS_HIGH_AGL status=" + ship_status +
            " agl_m=" + ROUND(agl_now_m, 2)
          ).
        }
      }
      IF agl_now_m <= AUTO_LAND_FINAL_AGL_M + 0.35 AND
         ABS(SHIP:VERTICALSPEED) <= AUTO_LAND_TOUCHDOWN_VS_MS AND
         SHIP:GROUNDSPEED <= AUTO_LAND_TOUCHDOWN_GNDSPD_MS {
        SET touchdown_kinematic TO TRUE.
      }
    }
    IF ship_status = "LANDED" OR ship_status = "SPLASHED" OR ship_status = "PRELAUNCH" {
      SET grounded_confirmed TO TRUE.
    } ELSE IF agl_now_m <= AUTO_LAND_FINAL_CONFIRM_AGL_M AND
       ABS(SHIP:VERTICALSPEED) <= AUTO_LAND_FINAL_CONFIRM_VS_MS AND
       SHIP:GROUNDSPEED <= AUTO_LAND_FINAL_CONFIRM_GNDSPD_MS {
      SET grounded_confirmed TO TRUE.
    }

    SET touchdown_detected TO touchdown_status_based OR touchdown_kinematic.
    IF touchdown_detected AND grounded_confirmed {
      IF land_touchdown_start_ut < 0 {
        SET land_touchdown_start_ut TO cycle_now_ut.
      } ELSE IF cycle_now_ut - land_touchdown_start_ut >= AUTO_LAND_SETTLE_TIME_S {
        _AUTO_LOG_EVENT("LANDED").
        // Explicit shutdown state: cut throttle and release control loops.
        LOCK THROTTLE TO 0.
        SET VTOL_ALT_HOLD TO FALSE.
        SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
        SET VTOL_KHV_ACTIVE TO FALSE.
        SET VTOL_POS_HOLD_ACTIVE TO FALSE.
        SET VTOL_TRANS_ACTIVE TO FALSE.
        SET AUTO_RUNNING TO FALSE.
      }
    } ELSE {
      SET land_touchdown_start_ut TO -1.
    }
    IF phase_elapsed_s >= AUTO_LAND_TIMEOUT_S {
      IF NOT land_timeout_logged {
        _AUTO_LOG_EVENT("LAND_TIMEOUT_CONTINUE_FOR_TOUCHDOWN").
        SET land_timeout_logged TO TRUE.
      }
      // Keep descending; do not terminate until grounded.
      SET land_cmd_agl_track TO AUTO_LAND_FINAL_AGL_M.
    }
  } ELSE {
    _AUTO_LOG_EVENT("UNKNOWN_PHASE_" + AUTO_PHASE_NAME).
    SET AUTO_RUNNING TO FALSE.
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
  // Keep stock yaw axis neutral so this test uses nacelle yaw only.
  SET SHIP:CONTROL:YAW TO 0.
  VTOL_SET_YAW_INPUT_OVERRIDE(yaw_cmd_apply).
  VTOL_TICK_PREARM().
  LOCAL srv_pos_span_deg IS 0.0.
  LOCAL srv_cmd_span_deg IS 0.0.
  LOCAL srv_mix_sum IS 0.0.
  LOCAL yaw_gain_diag IS _AUTO_CFG_NUM("vtol_yaw_gain", VTOL_YAW_SRV_GAIN).
  LOCAL cmd_min_deg IS 999.0.
  LOCAL cmd_max_deg IS -999.0.
  LOCAL pos_min_deg IS 999.0.
  LOCAL pos_max_deg IS -999.0.
  LOCAL srv_i IS 0.
  UNTIL srv_i >= VTOL_SRV_LIST:LENGTH {
    LOCAL yaw_mix_i IS 0.0.
    IF srv_i < VTOL_YAW_SRV_MIX:LENGTH {
      SET yaw_mix_i TO VTOL_YAW_SRV_MIX[srv_i].
    }
    SET srv_mix_sum TO srv_mix_sum + ABS(yaw_mix_i).
    LOCAL cmd_i_deg IS CLAMP(VTOL_NACELLE_ALPHA_CMD + yaw_cmd_apply * yaw_gain_diag * yaw_mix_i, 0, 180).
    IF cmd_i_deg < cmd_min_deg { SET cmd_min_deg TO cmd_i_deg. }
    IF cmd_i_deg > cmd_max_deg { SET cmd_max_deg TO cmd_i_deg. }
    LOCAL srv_mod_i IS VTOL_SRV_LIST[srv_i].
    IF srv_mod_i <> 0 {
      LOCAL pos_i_deg IS _VTOL_SERVO_CURRENT_POS(srv_mod_i, VTOL_NACELLE_ALPHA_CMD).
      IF pos_i_deg < pos_min_deg { SET pos_min_deg TO pos_i_deg. }
      IF pos_i_deg > pos_max_deg { SET pos_max_deg TO pos_i_deg. }
    }
    SET srv_i TO srv_i + 1.
  }
  IF cmd_max_deg >= cmd_min_deg {
    SET srv_cmd_span_deg TO cmd_max_deg - cmd_min_deg.
  }
  IF pos_max_deg >= pos_min_deg {
    SET srv_pos_span_deg TO pos_max_deg - pos_min_deg.
  }
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
    PRINT ("hdg/err/r/r*: " + _AUTO_FMT(yaw_hdg_now_deg, 7, 2) +
           " / " + _AUTO_FMT(yaw_err_deg, 7, 2) +
           " / " + _AUTO_FMT(yaw_rate_dps, 6, 2) +
           " / " + _AUTO_FMT(yaw_rate_cmd_dps_diag, 6, 2)) AT(0, 10).
    PRINT ("yaw u/a/re: " + _AUTO_FMT(yaw_cmd_out, 6, 3) +
           " / " + _AUTO_FMT(yaw_cmd_apply, 6, 3) +
           " / " + _AUTO_FMT(yaw_rate_err_dps_diag, 6, 2)) AT(0, 11).
    PRINT ("srv span pos/cmd: " + _AUTO_FMT(srv_pos_span_deg, 6, 2) +
           " / " + _AUTO_FMT(srv_cmd_span_deg, 6, 2) +
           " mix:" + _AUTO_FMT(srv_mix_sum, 5, 1)) AT(0, 12).
    PRINT ("ABORT to stop") AT(0, 13).
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
      ROUND(yaw_hdg_now_deg, 3),
      ROUND(yaw_err_deg, 3),
      ROUND(yaw_rate_dps, 3),
      ROUND(yaw_rate_cmd_dps_diag, 3),
      ROUND(yaw_rate_err_dps_diag, 3),
      ROUND(yaw_cmd_out, 4),
      ROUND(yaw_cmd_apply, 4),
      ROUND(srv_pos_span_deg, 4),
      ROUND(srv_cmd_span_deg, 4),
      ROUND(srv_mix_sum, 3),
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
      ROUND(VTOL_EFF_LAG_WORST_S, 4),
      TELEM_EM_STARVING
    ).
    LOG csv_row:JOIN(",") TO AUTO_LOG_FILE.
  }

  WAIT IFC_LOOP_DT.
}

_AUTO_LOG_EVENT("COMPLETE").
LOG ("# end t=" + ROUND(TIME:SECONDS - AUTO_MISSION_START_UT, 2)) TO AUTO_LOG_FILE.
LOCK THROTTLE TO 0.
VTOL_CLEAR_YAW_INPUT_OVERRIDE().
SET SHIP:CONTROL:YAW TO 0.
SET SHIP:CONTROL:ROLL TO 0.
SET SHIP:CONTROL:PITCH TO 0.
VTOL_RELEASE().
WAIT 0.1.
CLEARSCREEN.
PRINT "VTOL AUTO TEST COMPLETE".
PRINT "Log: " + AUTO_LOG_FILE.
}
