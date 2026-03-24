@LAZYGLOBAL OFF.

// ============================================================
// ifc_logger.ks  -  Integrated Flight Computer
//
// Per-cycle CSV telemetry logger.  Starts at IFC arm, runs
// until shutdown.  Writes to:
//   0:/Integrated Flight Computer/logs/ifc_log_<T>.csv
//
// Columns
// -------
// FLIGHT STATE
//   t_s            mission elapsed time (s)
//   phase          APPROACH / FLARE / TOUCHDOWN / ROLLOUT / DONE
//   subphase       FLY_TO_FIX / ILS_TRACK (blank during autoland)
//
// AIR DATA
//   ias_ms         indicated airspeed (m/s)
//   vapp_ms        current approach speed target (m/s)
//   spd_err_ms     speed target - IAS (m/s, +ve = too slow)
//   agl_m          radar altitude AGL (m)
//   gear_h_m       main-gear minimum runway-relative height (m)
//   vs_ms          vertical speed (m/s, -ve = descending)
//   pitch_deg      nose pitch above horizon (deg)
//   aoa_deg        angle of attack from FAR (deg, 0 if unavailable)
//
// ATTITUDE
//   hdg_deg        compass heading of nose (deg, 0=N 90=E)
//   bank_deg       bank angle (deg, +ve = right wing down)
//
// AUTOTHROTTLE
//   thr_cmd        throttle command (0-1)
//   thr_cur        current vessel throttle (0-1)
//   thr_intg       speed integral accumulator (m/s·s)
//
// AA DIRECTOR COMMANDS
//   aa_hdg_cmd_deg heading commanded to AA Director (deg)
//   aa_fpa_cmd_deg vertical guidance command from phase logic (deg)
//                  (cross-phase channel: FPA in most phases, may be direct pitch in
//                   pitch-commanded phases; prefer flare_fpa_cmd for flare TECS math)
//
// ILS DEVIATIONS
//   ils_loc_m      lateral deviation from centreline (m, +ve = right)
//   ils_gs_m       vertical deviation from glideslope (m, +ve = above)
//   ils_dist_km    distance from threshold (km)
//
// ILS CONTROLLER INTERNALS  (tuning: KP_LOC, KD_LOC, KP_GS, KD_GS)
//   loc_corr_deg   heading correction from localizer PD (deg)
//   gs_corr_deg    FPA correction from glideslope PD (deg)
//   gs_latched     1 when GS capture latch is active, else 0
//   active_gs_ang_deg active approach GS angle from loaded plate (deg)
//   ils_intercept_alt_m pre-capture intercept altitude hold reference (m MSL)
//   d_gs_ms        raw glideslope deviation rate d(ILS_GS_DEV)/dt (m/s)
//   fpa_preclamped_deg vertical command before FAR/AOA pitch clamp (deg)
//   gs_nom_alt_m   nominal GS altitude at current distance (m MSL)
//
// FLARE INTERNALS  (tuning: flare_* aircraft params, FLARE_* constants)
//   flare_fpa_cmd  smoothed FPA command (deg)
//   flare_tgt_vs   target sink rate from schedule (m/s)
//   flare_frac     progress: 0=entry AGL, 1=ground
//   flare_mode     FLARE_CAPTURE / FLARE_TRACK / ROUNDOUT / TOUCHDOWN_CONFIRM
//   flare_auth_limited 1 when authority-limited recovery branch is active, else 0
//   flare_auth_reason authority-limited reason (NONE/TRACK/SIGN/TAILSTRIKE/BALLOON)
//   flare_vs_err   target sink minus actual VS (m/s)
//   flare_fpa_err  commanded FPA minus actual FPA (deg)
//   flare_pitch_err commanded AA direction pitch minus actual pitch (deg)
//   flare_theta_cmd_raw director pitch command before tailstrike clamp (deg)
//   flare_theta_cmd_clamped director pitch command after tailstrike clamp (deg)
//   flare_theta_clamp_active 1 when tailstrike clamp was active this sample, else 0
//   flare_aoa_clamp_active 1 when flare is likely AoA-limited on nose-up pitch demand, else 0
//   flare_pitch_in_cmd SHIP:CONTROL:PITCH input command (unitless, -1..1)
//   elev_defl_avg_deg average absolute pitch-surface deflection seen this sample (deg)
//   elev_defl_max_deg max absolute pitch-surface deflection seen this sample (deg)
//   elev_defl_n number of pitch-surface module samples used in deflection stats
//   flare_ctrl_h_m control-height state used by flare controller (gear_h_m + FLARE_CTRL_H_OFFSET, clamped >= 0) (m)
//   flare_req_up_a estimated upward accel needed to reach target VS by touchdown using flare_ctrl_h_m (m/s^2)
//   flare_thr_floor active flare throttle floor value (0..1)
//   flare_tecs_et_err flare TECS total-energy error (m^2/s^2)
//   flare_tecs_eb_err flare TECS energy-balance error (m^2/s^2)
//   flare_tecs_h_ref flare TECS runway-relative height reference (m)
//   flare_tecs_v_ref flare TECS speed reference (m/s)
//   flare_gamma_ref flare gamma reference from sink schedule (deg)
//   flare_gamma_eb_term flare gamma contribution from TECS energy-balance loop (deg)
//   flare_gamma_unsat flare gamma command before clamp/slew (deg)
//
// ROLLOUT INTERNALS  (tuning: rollout_* aircraft params, ROLLOUT_* constants)
//   steer_hdg_deg  wheelsteering target heading (deg)
//   steer_blend    blend factor toward runway heading (0-1)
//   ro_loc_corr_d  heading correction from ILS centreline (deg)
//   ro_hdg_err_deg actual heading minus steer target (deg)
//   ro_yaw_tgt     yaw command target before slew rate limit
//   ro_yaw_scale   yaw assist blend factor (0-1)
//   ro_yaw_gate    0=active, 1=speed-gated, 2=guard-gated, 3=not landed
//   yaw_cmd        raw rudder output to SHIP:CONTROL:YAW
//   roll_cmd       raw aileron output to SHIP:CONTROL:ROLL
//   pitch_cmd      raw pitch input to SHIP:CONTROL:PITCH
//   ro_pitch_tgt   rollout pitch attitude target (deg)
//   ro_pitch_err   rollout pitch attitude error (deg)
//   ro_pitch_ff    rollout pitch feedforward bias command
//   ro_roll_asst   1 when IFC rollout roll assist is active, else 0
//
// FLAP STATE
//   flaps_cur      current flap detent index
//   flaps_tgt      target flap detent index
//
// ASCENT GUIDANCE  (non-zero only during PHASE_ASCENT)
//   asc_j_ab       J/kg·(kg/s)  air-breathing mode performance value
//   asc_j_rk       J/kg·(kg/s)  rocket mode performance value
//   asc_validity   estimator validity string (VALID / DEGRADED / INVALID)
//   asc_q_pa       dynamic pressure (Pa)
//   asc_mach       Mach number
//   asc_apo_m      smoothed apoapsis altitude (m)
//   asc_drag_n     along-track drag used in J_rk estimate (N)
//   asc_w_prop     propellant mass fraction remaining (0-1)
//   asc_edot_aero  specific orbital energy rate from aero forces (J/kg/s)
//   asc_edot_orb   specific orbital energy rate from orbital state (J/kg/s)
//   asc_pitch_bias current pitch bias above prograde (deg)
//   asc_blend      steer blend: 0=surface prograde, 1=orbital prograde
//   asc_spooling   1 while waiting for AB spool-down after rocket commit
//   asc_q_raw_pa   raw FAR dynamic pressure sample (Pa)
//   asc_ab_thr_ratio      AB actual/available thrust ratio (0-1)
//   asc_ab_t_now          AB current thrust sum
//   asc_ab_t_avail        AB available thrust sum
//   asc_ab_ign_on         AB engines with ignition true
//   asc_ab_flameouts      AB engines currently flameout=true
//   asc_rk_t_now          Rocket-class current thrust sum
//   asc_rk_t_avail        Rocket-class available thrust sum
//   asc_rk_ign_on         Rocket-class engines with ignition true
//   asc_rk_flameouts      Rocket-class engines currently flameout=true
//   ship_thrust           Vessel total thrust
//   ship_avail_thrust     Vessel total available thrust
//   ship_ign_on           Vessel engines with ignition true
//   ship_flameouts        Vessel engines currently flameout=true
//
// MISC
//   phase_el_s     time elapsed in current phase (s)
//   status         SHIP:STATUS string
//   craft_name     SHIP:NAME captured at logger init
//   cfg_file       cfg source/path resolved by ifc_main
// ============================================================

GLOBAL LOG_ACTIVE IS FALSE.
GLOBAL LOG_FILE   IS "".
GLOBAL LOG_LAST_WRITE_UT IS -1.
GLOBAL LOG_CRAFT_NAME IS "".
GLOBAL LOG_CFG_FILE   IS "".

FUNCTION _LOG_ENTRY_NAME {
  PARAMETER entry.
  IF entry = 0 { RETURN "". }
  IF entry:HASSUFFIX("NAME") { RETURN entry:NAME. }
  RETURN "" + entry.
}

FUNCTION _LOG_PART_HAS_MODULE_NAME {
  PARAMETER p, target_name.
  IF p = 0 { RETURN FALSE. }
  IF target_name = "" { RETURN FALSE. }
  IF NOT p:HASSUFFIX("ALLMODULES") { RETURN FALSE. }

  LOCAL allmods IS p:ALLMODULES.
  LOCAL i IS 0.
  UNTIL i >= allmods:LENGTH {
    LOCAL nm IS _LOG_ENTRY_NAME(allmods[i]).
    IF nm = target_name { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _LOG_RESOLVE_MODULE_OBJECT {
  PARAMETER p, module_entry, module_name.

  IF module_entry <> 0 AND module_entry:HASSUFFIX("HASFIELD") { RETURN module_entry. }

  IF p <> 0 AND p:HASSUFFIX("GETMODULE") AND module_name <> "" AND _LOG_PART_HAS_MODULE_NAME(p, module_name) {
    LOCAL probe_mod IS p:GETMODULE(module_name).
    IF probe_mod <> 0 { RETURN probe_mod. }
  }

  RETURN module_entry.
}

FUNCTION _LOG_IS_CTRL_SURFACE_MODULE {
  PARAMETER module_name.
  IF module_name = "" { RETURN FALSE. }
  LOCAL n IS module_name:TOLOWER().
  IF n:CONTAINS("controlsurface") { RETURN TRUE. }
  IF n:CONTAINS("controllablesurface") { RETURN TRUE. }
  IF n:CONTAINS("elevator") { RETURN TRUE. }
  RETURN FALSE.
}

FUNCTION _LOG_IS_TRUE_TEXT {
  PARAMETER raw_val.
  LOCAL s IS ("" + raw_val):TOLOWER().
  SET s TO s:TRIM.
  RETURN s = "true" OR s = "1" OR s = "yes" OR s = "on".
}

FUNCTION _LOG_READ_FIELD_NUMBER {
  PARAMETER module_obj, field_name, fallback.
  IF module_obj = 0 { RETURN fallback. }
  IF field_name = "" { RETURN fallback. }
  IF NOT module_obj:HASSUFFIX("GETFIELD") { RETURN fallback. }
  IF module_obj:HASSUFFIX("HASFIELD") AND NOT module_obj:HASFIELD(field_name) { RETURN fallback. }

  LOCAL raw_val IS module_obj:GETFIELD(field_name).
  RETURN ("" + raw_val):TONUMBER(fallback).
}

FUNCTION _LOG_RESOLVE_CTRL_DEFLECTION_FIELD {
  PARAMETER module_obj.
  IF module_obj = 0 { RETURN "". }
  IF NOT module_obj:HASSUFFIX("HASFIELD") { RETURN "". }

  LOCAL candidates IS LIST(
    "ctrlSurfaceAngle",
    "ctrlsurfaceangle",
    "currentDeflection",
    "currentdeflection",
    "deflectionAngle",
    "deflectionangle",
    "deflection",
    "Deflection",
    "angle",
    "Angle",
    "deployAngle",
    "deployangle"
  ).

  LOCAL i IS 0.
  UNTIL i >= candidates:LENGTH {
    LOCAL f IS candidates[i].
    IF module_obj:HASFIELD(f) { RETURN f. }
    SET i TO i + 1.
  }
  RETURN "".
}

FUNCTION _LOG_MODULE_HAS_PITCH_AUTH {
  PARAMETER module_obj.
  IF module_obj = 0 { RETURN FALSE. }

  IF module_obj:HASSUFFIX("HASFIELD") AND module_obj:HASSUFFIX("GETFIELD") {
    IF module_obj:HASFIELD("ignorePitch") {
      IF _LOG_IS_TRUE_TEXT(module_obj:GETFIELD("ignorePitch")) { RETURN FALSE. }
    }
    IF module_obj:HASFIELD("Ignore Pitch") {
      IF _LOG_IS_TRUE_TEXT(module_obj:GETFIELD("Ignore Pitch")) { RETURN FALSE. }
    }
    IF module_obj:HASFIELD("usePitch") {
      IF NOT _LOG_IS_TRUE_TEXT(module_obj:GETFIELD("usePitch")) { RETURN FALSE. }
    }
  }

  LOCAL pitch_axis IS _LOG_READ_FIELD_NUMBER(module_obj, "pitchAxis", 999999).
  IF pitch_axis = 999999 { SET pitch_axis TO _LOG_READ_FIELD_NUMBER(module_obj, "pitchaxis", 999999). }
  IF pitch_axis <> 999999 AND ABS(pitch_axis) < 0.01 { RETURN FALSE. }

  RETURN TRUE.
}

FUNCTION _LOG_SAMPLE_PITCH_SURFACE_DEFLECTION {
  LOCAL parts IS SHIP:PARTS.
  LOCAL sum_abs IS 0.
  LOCAL max_abs IS 0.
  LOCAL count IS 0.

  LOCAL i IS 0.
  UNTIL i >= parts:LENGTH {
    LOCAL p IS parts[i].
    SET i TO i + 1.
    IF p <> 0 AND p:HASSUFFIX("MODULES") {
      LOCAL skip_part IS FALSE.
      IF p:HASSUFFIX("TAG") {
        LOCAL tag_lc IS p:TAG:TOLOWER().
        IF tag_lc:CONTAINS("spoiler") { SET skip_part TO TRUE. }
      }
      IF NOT skip_part {
        LOCAL mods IS p:MODULES.
        LOCAL j IS 0.
        UNTIL j >= mods:LENGTH {
          LOCAL m IS mods[j].
          SET j TO j + 1.
          IF m <> 0 {
            LOCAL module_name IS _LOG_ENTRY_NAME(m).
            IF _LOG_IS_CTRL_SURFACE_MODULE(module_name) {
              LOCAL module_obj IS _LOG_RESOLVE_MODULE_OBJECT(p, m, module_name).
              IF module_obj <> 0 AND module_obj:HASSUFFIX("GETFIELD") {
                IF _LOG_MODULE_HAS_PITCH_AUTH(module_obj) {
                  LOCAL field_name IS _LOG_RESOLVE_CTRL_DEFLECTION_FIELD(module_obj).
                  IF field_name <> "" {
                    LOCAL defl_deg IS _LOG_READ_FIELD_NUMBER(module_obj, field_name, 999999).
                    IF defl_deg <> 999999 {
                      LOCAL abs_defl IS ABS(defl_deg).
                      SET sum_abs TO sum_abs + abs_defl.
                      IF abs_defl > max_abs { SET max_abs TO abs_defl. }
                      SET count TO count + 1.
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  LOCAL avg_abs IS 0.
  IF count > 0 { SET avg_abs TO sum_abs / count. }
  RETURN LEXICON(
    "avg_abs_deg", avg_abs,
    "max_abs_deg", max_abs,
    "count", count
  ).
}

FUNCTION LOGGER_INIT {
  LOCAL log_dir IS "0:/Integrated Flight Computer/logs".
  IF NOT EXISTS(log_dir) { CREATEDIR(log_dir). }

  // Build a unique logfile name using a persistent counter file.
  // Reading one file is O(1) regardless of how many logs exist — no scan loop.
  LOCAL counter_path IS log_dir + "/counter.txt".
  LOCAL seq IS 1.
  IF EXISTS(counter_path) {
    LOCAL counter_str IS OPEN(counter_path):READALL:STRING:TRIM.
    IF counter_str:LENGTH > 0 {
      LOCAL parsed IS counter_str:TONUMBER(0).
      IF parsed >= 1 { SET seq TO ROUND(parsed). }
    }
  }
  // Write the next sequence number back so subsequent runs get unique filenames.
  IF EXISTS(counter_path) { DELETEPATH(counter_path). }
  LOG (seq + 1) TO counter_path.

  LOCAL seq_str IS "" + seq.
  UNTIL seq_str:LENGTH >= 8 { SET seq_str TO "0" + seq_str. }
  SET LOG_FILE TO log_dir + "/ifc_log_" + seq_str + ".csv".
  SET LOG_CRAFT_NAME TO SHIP:NAME:REPLACE(",", "_").
  SET LOG_CFG_FILE   TO IFC_ACTIVE_CFG_PATH:REPLACE(",", "_").
  IF LOG_CFG_FILE = "" { SET LOG_CFG_FILE TO "UNKNOWN". }

  LOG "t_s,phase,subphase,ias_ms,vapp_ms,spd_err_ms,agl_m,gear_h_m,vs_ms,pitch_deg,aoa_deg,hdg_deg,bank_deg,thr_cmd,thr_cur,thr_intg,at_gain,at_tau_s,at_a_up,at_a_dn,at_kp_thr,at_ki_spd,at_thr_slew,as_cmd_deg,as_cap_deg,as_raw_deg,as_spd_err_ms,as_active,aa_hdg_cmd_deg,aa_fpa_cmd_deg,aa_fbw,aa_dir,actual_fpa_deg,ftf_hdg_err_deg,ftf_fix_idx,kos_steer_pit_deg,kos_steer_hdg_deg,aa_dir_vx,aa_dir_vy,aa_dir_vz,aa_dir_pitch_deg,aa_dir_hdg_deg,ils_loc_m,ils_gs_m,ils_dist_km,loc_corr_deg,gs_corr_deg,gs_latched,active_gs_ang_deg,ils_intercept_alt_m,d_gs_ms,fpa_preclamped_deg,gs_nom_alt_m,flare_fpa_cmd,flare_tgt_vs,flare_frac,flare_mode,flare_auth_limited,flare_auth_reason,flare_vs_err,flare_fpa_err,flare_pitch_err,flare_theta_cmd_raw,flare_theta_cmd_clamped,flare_theta_clamp_active,flare_aoa_clamp_active,flare_pitch_in_cmd,elev_defl_avg_deg,elev_defl_max_deg,elev_defl_n,flare_ctrl_h_m,flare_req_up_a,flare_thr_floor,flare_tecs_et_err,flare_tecs_eb_err,flare_tecs_h_ref,flare_tecs_v_ref,flare_gamma_ref,flare_gamma_eb_term,flare_gamma_unsat,steer_hdg_deg,steer_blend,ro_loc_corr_deg,ro_hdg_err_deg,ro_yaw_tgt,ro_yaw_scale,ro_yaw_gate,yaw_cmd,roll_cmd,pitch_cmd,ro_pitch_tgt_deg,ro_pitch_err_deg,ro_pitch_ff,ro_roll_assist,flaps_cur,flaps_tgt,asc_j_ab,asc_j_rk,asc_validity,asc_q_pa,asc_q_raw_pa,asc_mach,asc_apo_m,asc_drag_n,asc_w_prop,asc_edot_aero,asc_edot_orb,asc_pitch_bias,asc_blend,asc_spooling,asc_ab_thr_ratio,asc_ab_t_now,asc_ab_t_avail,asc_ab_ign_on,asc_ab_flameouts,asc_rk_t_now,asc_rk_t_avail,asc_rk_ign_on,asc_rk_flameouts,ship_thrust,ship_avail_thrust,ship_ign_on,ship_flameouts,ifc_raw_dt_s,ifc_dt_s,ifc_loop_n,ifc_hz_est,ifc_raw_dt_max_s,ifc_raw_dt_min_s,phase_el_s,status,craft_name,cfg_file" TO LOG_FILE.

  SET LOG_ACTIVE TO TRUE.
  SET LOG_LAST_WRITE_UT TO TIME:SECONDS - IFC_CSV_LOG_PERIOD.
  SET IFC_ALERT_TEXT TO "Logging -> " + LOG_FILE.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION LOGGER_WRITE {
  IF NOT LOG_ACTIVE { RETURN. }
  IF TIME:SECONDS - LOG_LAST_WRITE_UT < IFC_CSV_LOG_PERIOD { RETURN. }
  LOCAL now IS TIME:SECONDS.
  LOCAL elapsed_log IS MAX(now - LOG_LAST_WRITE_UT, 0.001).
  LOCAL loops_since_log IS MAX(IFC_LOOP_COUNT - IFC_LOOP_COUNT_SNAPSHOT, 0).
  LOCAL hz_est IS loops_since_log / elapsed_log.
  LOCAL raw_dt_max IS IFC_RAW_DT_MAX.
  LOCAL raw_dt_min IS IFC_RAW_DT_MIN.
  SET IFC_LOOP_COUNT_SNAPSHOT TO IFC_LOOP_COUNT.
  SET IFC_RAW_DT_MAX TO IFC_RAW_DT.
  SET IFC_RAW_DT_MIN TO IFC_RAW_DT.
  SET LOG_LAST_WRITE_UT TO TIME:SECONDS.

  LOCAL ias   IS GET_IAS().
  LOCAL v_tgt IS ACTIVE_V_APP.
  IF IFC_PHASE = PHASE_APPROACH { SET v_tgt TO ACTIVE_V_TGT. }
  LOCAL gear_h IS GET_MAIN_GEAR_RUNWAY_HEIGHT_MIN().
  LOCAL vs_now IS SHIP:VERTICALSPEED.
  LOCAL flare_vs_err IS 0.
  LOCAL flare_fpa_err IS 0.
  LOCAL flare_pitch_err IS 0.
  LOCAL flare_ctrl_h_m IS MAX(gear_h + FLARE_CTRL_H_OFFSET, 0).
  LOCAL flare_req_up_a IS 0.
  LOCAL flare_thr_floor IS 0.
  LOCAL flare_mode IS "".
  LOCAL flare_auth_limited IS 0.
  LOCAL flare_auth_reason IS "NONE".
  LOCAL flare_theta_cmd_raw IS 0.
  LOCAL flare_theta_cmd_clamped IS 0.
  LOCAL flare_theta_clamp_active IS 0.
  LOCAL flare_aoa_clamp_active IS 0.
  IF IFC_PHASE = PHASE_FLARE OR IFC_PHASE = PHASE_TOUCHDOWN {
    SET flare_mode TO FLARE_SUBMODE.
    IF FLARE_AUTH_LIMITED { SET flare_auth_limited TO 1. }
    SET flare_auth_reason TO TELEM_FLARE_AUTH_REASON.
    SET flare_vs_err TO TELEM_FLARE_TGT_VS - vs_now.
    // Use flare_fpa_cmd source directly for flare error math to avoid ambiguity
    // with cross-phase aa_fpa_cmd_deg semantics.
    SET flare_fpa_err TO FLARE_PITCH_CMD - TELEM_ACTUAL_FPA_DEG.
    SET flare_pitch_err TO TELEM_AA_DIR_PITCH_DEG - TELEM_PITCH_DEG.
    SET flare_theta_cmd_raw TO TELEM_FLARE_THETA_CMD_RAW.
    SET flare_theta_cmd_clamped TO TELEM_FLARE_THETA_CMD_CLAMPED.
    SET flare_theta_clamp_active TO TELEM_FLARE_THETA_CLAMP_ACTIVE.
    SET flare_aoa_clamp_active TO TELEM_FLARE_AOA_CLAMP_ACTIVE.
    IF flare_ctrl_h_m > 0.5 {
      // Required average upward acceleration to move from current VS to target VS
      // over remaining control-height. Positive = need to arrest descent.
      SET flare_req_up_a TO ((vs_now * vs_now) - (TELEM_FLARE_TGT_VS * TELEM_FLARE_TGT_VS)) / (2 * flare_ctrl_h_m).
    }
    SET flare_thr_floor TO TELEM_FLARE_THR_FLOOR.
  }
  LOCAL flare_pitch_in_cmd IS SHIP:CONTROL:PITCH.
  LOCAL elev_defl_avg_deg IS 0.
  LOCAL elev_defl_max_deg IS 0.
  LOCAL elev_defl_n IS 0.
  IF IFC_LOG_SAMPLE_CTRL_SURF AND (IFC_PHASE = PHASE_FLARE OR IFC_PHASE = PHASE_TOUCHDOWN) {
    LOCAL elev_stats IS _LOG_SAMPLE_PITCH_SURFACE_DEFLECTION().
    IF elev_stats:HASKEY("avg_abs_deg") { SET elev_defl_avg_deg TO elev_stats["avg_abs_deg"]. }
    IF elev_stats:HASKEY("max_abs_deg") { SET elev_defl_max_deg TO elev_stats["max_abs_deg"]. }
    IF elev_stats:HASKEY("count") { SET elev_defl_n TO elev_stats["count"]. }
  }
  LOCAL bank  IS ROUND(TELEM_BANK_DEG, 3).
  LOCAL ship_t_now   IS SHIP:THRUST.
  LOCAL ship_t_avail IS SHIP:AVAILABLETHRUST.
  // P5: avoid SHIP:ENGINES traversal (expensive); use ascent telemetry during
  // ascent phase and fall back to a single traversal only for other phases.
  LOCAL ship_ign_on IS 0.
  LOCAL ship_flameouts IS 0.
  IF IFC_PHASE = PHASE_ASCENT {
    SET ship_ign_on    TO TELEM_ASC_AB_IGN_ON    + TELEM_ASC_RK_IGN_ON.
    SET ship_flameouts TO TELEM_ASC_AB_FLAMEOUTS + TELEM_ASC_RK_FLAMEOUTS.
  } ELSE {
    LOCAL all_eng IS SHIP:ENGINES.
    LOCAL ei IS 0.
    UNTIL ei >= all_eng:LENGTH {
      LOCAL eng IS all_eng[ei].
      IF eng:IGNITION {
        SET ship_ign_on TO ship_ign_on + 1.
        IF eng:FLAMEOUT { SET ship_flameouts TO ship_flameouts + 1. }
      }
      SET ei TO ei + 1.
    }
  }

  // P6b: build row as a LIST then JOIN to avoid repeated string allocations.
  LOCAL row_parts IS LIST(
    ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 2),
    IFC_PHASE,
    IFC_SUBPHASE,
    ROUND(ias,                        2),
    ROUND(v_tgt,                      2),
    ROUND(v_tgt - ias,                2),
    ROUND(GET_AGL(),                  2),
    ROUND(gear_h,                     2),
    ROUND(vs_now,                     2),
    ROUND(TELEM_PITCH_DEG,            2),
    ROUND(GET_AOA(),                  2),
    ROUND(TELEM_COMPASS_HDG,          2),
    bank,
    ROUND(THROTTLE_CMD,               4),
    ROUND(GET_CURRENT_THROTTLE(),     4),
    ROUND(THR_INTEGRAL,               3),
    ROUND(TELEM_AT_GAIN,              4),
    ROUND(TELEM_AT_TAU,               3),
    ROUND(TELEM_AT_A_UP_LIM,          3),
    ROUND(TELEM_AT_A_DN_LIM,          3),
    ROUND(TELEM_AT_KP_THR,            4),
    ROUND(TELEM_AT_KI_SPD,            4),
    ROUND(TELEM_AT_THR_SLEW,          4),
    ROUND(TELEM_AS_CMD_DEG,           3),
    ROUND(TELEM_AS_CAP_DEG,           3),
    ROUND(TELEM_AS_RAW_DEG,           3),
    ROUND(TELEM_AS_ERR_MPS,           3),
    ROUND(TELEM_AS_ACTIVE,            0),
    ROUND(TELEM_AA_HDG_CMD,           2),
    ROUND(TELEM_AA_FPA_CMD,           3),
    ROUND(TELEM_AA_FBW_ON,            0),
    ROUND(TELEM_AA_DIR_ON,            0),
    ROUND(TELEM_ACTUAL_FPA_DEG,       2),
    ROUND(TELEM_FTF_HDG_ERR,          2),
    ROUND(TELEM_FTF_FIX_IDX,          0),
    ROUND(TELEM_KOS_STEER_PIT,        2),
    ROUND(TELEM_KOS_STEER_HDG,        2),
    ROUND(TELEM_AA_DIR_VX,            4),
    ROUND(TELEM_AA_DIR_VY,            4),
    ROUND(TELEM_AA_DIR_VZ,            4),
    ROUND(TELEM_AA_DIR_PITCH_DEG,     2),
    ROUND(TELEM_AA_DIR_HDG_DEG,       2),
    ROUND(ILS_LOC_DEV,                2),
    ROUND(ILS_GS_DEV,                 2),
    ROUND(ILS_DIST_M / 1000,          3),
    ROUND(TELEM_LOC_CORR,             3),
    ROUND(TELEM_GS_CORR,              3),
    CHOOSE 1 IF APP_GS_LATCHED ELSE 0,
    ROUND(ACTIVE_GS_ANGLE,            3),
    ROUND(ILS_INTERCEPT_ALT,          2),
    ROUND(TELEM_D_GS,                 3),
    ROUND(TELEM_FPA_PRECLAMPED,       3),
    ROUND(ACTIVE_THR_ALT + ILS_DIST_M * ILS_GS_TAN_CACHED, 2),
    ROUND(FLARE_PITCH_CMD,            3),
    ROUND(TELEM_FLARE_TGT_VS,         3),
    ROUND(TELEM_FLARE_FRAC,           3),
    flare_mode,
    flare_auth_limited,
    flare_auth_reason,
    ROUND(flare_vs_err,               3),
    ROUND(flare_fpa_err,              3),
    ROUND(flare_pitch_err,            3),
    ROUND(flare_theta_cmd_raw,        3),
    ROUND(flare_theta_cmd_clamped,    3),
    ROUND(flare_theta_clamp_active,   0),
    ROUND(flare_aoa_clamp_active,     0),
    ROUND(flare_pitch_in_cmd,         4),
    ROUND(elev_defl_avg_deg,          3),
    ROUND(elev_defl_max_deg,          3),
    ROUND(elev_defl_n,                0),
    ROUND(flare_ctrl_h_m,             2),
    ROUND(flare_req_up_a,             3),
    flare_thr_floor,
    ROUND(TELEM_FLARE_ET_ERR,         3),
    ROUND(TELEM_FLARE_EB_ERR,         3),
    ROUND(TELEM_FLARE_H_REF,          2),
    ROUND(TELEM_FLARE_V_REF,          2),
    ROUND(TELEM_FLARE_GAMMA_REF,      3),
    ROUND(TELEM_FLARE_GAMMA_EB_TERM,  3),
    ROUND(TELEM_FLARE_GAMMA_CMD_UNSAT,3),
    ROUND(ROLLOUT_STEER_HDG,          2),
    ROUND(TELEM_STEER_BLEND,          3),
    ROUND(TELEM_RO_LOC_CORR,          3),
    ROUND(TELEM_RO_HDG_ERR,           3),
    ROUND(TELEM_RO_YAW_TGT,           4),
    ROUND(TELEM_RO_YAW_SCALE,         3),
    ROUND(TELEM_RO_YAW_GATE,          0),
    ROUND(SHIP:CONTROL:YAW,           4),
    ROUND(SHIP:CONTROL:ROLL,          4),
    ROUND(SHIP:CONTROL:PITCH,         4),
    ROUND(TELEM_RO_PITCH_TGT,         3),
    ROUND(TELEM_RO_PITCH_ERR,         3),
    ROUND(TELEM_RO_PITCH_FF,          4),
    ROUND(TELEM_RO_ROLL_ASSIST,       0),
    FLAPS_CURRENT_DETENT,
    FLAPS_TARGET_DETENT,
    ROUND(TELEM_ASC_J_AB,             3),
    ROUND(TELEM_ASC_J_RK,             3),
    TELEM_ASC_VALIDITY,
    ROUND(TELEM_ASC_Q,                1),
    ROUND(TELEM_ASC_Q_RAW,            1),
    ROUND(TELEM_ASC_MACH,             3),
    ROUND(TELEM_ASC_APO,              0),
    ROUND(TELEM_ASC_DRAG_RK,          1),
    ROUND(TELEM_ASC_W_PROP,           4),
    ROUND(TELEM_ASC_EDOT_VAL,         2),
    ROUND(TELEM_ASC_EDOT_ORB,         2),
    ROUND(TELEM_ASC_PITCH_BIAS,       3),
    ROUND(TELEM_ASC_BLEND,            3),
    CHOOSE 1 IF TELEM_ASC_SPOOLING ELSE 0,
    ROUND(TELEM_ASC_AB_THR_RATIO,     4),
    ROUND(TELEM_ASC_AB_T_NOW,         2),
    ROUND(TELEM_ASC_AB_T_AVAIL,       2),
    ROUND(TELEM_ASC_AB_IGN_ON,        0),
    ROUND(TELEM_ASC_AB_FLAMEOUTS,     0),
    ROUND(TELEM_ASC_RK_T_NOW,         2),
    ROUND(TELEM_ASC_RK_T_AVAIL,       2),
    ROUND(TELEM_ASC_RK_IGN_ON,        0),
    ROUND(TELEM_ASC_RK_FLAMEOUTS,     0),
    ROUND(ship_t_now,                 2),
    ROUND(ship_t_avail,               2),
    ROUND(ship_ign_on,                0),
    ROUND(ship_flameouts,             0),
    ROUND(IFC_RAW_DT,                 3),
    ROUND(IFC_ACTUAL_DT,              3),
    ROUND(IFC_LOOP_COUNT,             0),
    ROUND(hz_est,                     2),
    ROUND(raw_dt_max,                 3),
    ROUND(raw_dt_min,                 3),
    ROUND(PHASE_ELAPSED(),            2),
    SHIP:STATUS,
    LOG_CRAFT_NAME,
    LOG_CFG_FILE
  ).

  LOG row_parts:JOIN(",") TO LOG_FILE.
}

FUNCTION LOGGER_CLOSE {
  IF NOT LOG_ACTIVE { RETURN. }
  LOG "# end T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1) +
      "  IAS " + ROUND(GET_IAS(), 1) + " m/s" TO LOG_FILE.
  SET LOG_ACTIVE TO FALSE.
  SET LOG_LAST_WRITE_UT TO -1.
  SET IFC_ALERT_TEXT TO "Log saved -> " + LOG_FILE.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}
