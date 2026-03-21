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
//   aa_fpa_cmd_deg FPA commanded to AA Director (deg)
//
// ILS DEVIATIONS
//   ils_loc_m      lateral deviation from centreline (m, +ve = right)
//   ils_gs_m       vertical deviation from glideslope (m, +ve = above)
//   ils_dist_km    distance from threshold (km)
//
// ILS CONTROLLER INTERNALS  (tuning: KP_LOC, KD_LOC, KP_GS, KD_GS)
//   loc_corr_deg   heading correction from localizer PD (deg)
//   gs_corr_deg    FPA correction from glideslope PD (deg)
//
// FLARE INTERNALS  (tuning: flare_* aircraft params, FLARE_* constants)
//   flare_fpa_cmd  smoothed FPA command (deg)
//   flare_tgt_vs   target sink rate from schedule (m/s)
//   flare_frac     progress: 0=entry AGL, 1=ground
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
// ============================================================

GLOBAL LOG_ACTIVE IS FALSE.
GLOBAL LOG_FILE   IS "".
GLOBAL LOG_LAST_WRITE_UT IS -1.

FUNCTION LOGGER_INIT {
  LOCAL log_dir IS "0:/Integrated Flight Computer/logs".
  IF NOT EXISTS(log_dir) { CREATEDIR(log_dir). }

  // Build a unique logfile name every run.
  // Use centiseconds to reduce collisions, then add a numeric suffix
  // if a file with the same name already exists (e.g. after quickload).
  LOCAL ts IS ROUND(MOD(TIME:SECONDS * 100, 10000000), 0).
  LOCAL base_name IS "ifc_log_" + ts.
  LOCAL candidate IS log_dir + "/" + base_name + ".csv".
  LOCAL suffix IS 0.
  UNTIL NOT EXISTS(candidate) {
    SET suffix TO suffix + 1.
    SET candidate TO log_dir + "/" + base_name + "_" + suffix + ".csv".
  }
  SET LOG_FILE TO candidate.

  LOG "t_s,phase,subphase,ias_ms,vapp_ms,spd_err_ms,agl_m,vs_ms,pitch_deg,aoa_deg,hdg_deg,bank_deg,thr_cmd,thr_cur,thr_intg,at_gain,at_tau_s,at_a_up,at_a_dn,at_kp_thr,at_ki_spd,at_thr_slew,aa_hdg_cmd_deg,aa_fpa_cmd_deg,ils_loc_m,ils_gs_m,ils_dist_km,loc_corr_deg,gs_corr_deg,flare_fpa_cmd,flare_tgt_vs,flare_frac,steer_hdg_deg,steer_blend,ro_loc_corr_deg,ro_hdg_err_deg,ro_yaw_tgt,ro_yaw_scale,ro_yaw_gate,yaw_cmd,roll_cmd,pitch_cmd,ro_pitch_tgt_deg,ro_pitch_err_deg,ro_pitch_ff,ro_roll_assist,flaps_cur,flaps_tgt,asc_j_ab,asc_j_rk,asc_validity,asc_q_pa,asc_q_raw_pa,asc_mach,asc_apo_m,asc_drag_n,asc_w_prop,asc_edot_aero,asc_edot_orb,asc_pitch_bias,asc_blend,asc_spooling,asc_ab_thr_ratio,asc_ab_t_now,asc_ab_t_avail,asc_ab_ign_on,asc_ab_flameouts,asc_rk_t_now,asc_rk_t_avail,asc_rk_ign_on,asc_rk_flameouts,ship_thrust,ship_avail_thrust,ship_ign_on,ship_flameouts,ifc_raw_dt_s,ifc_dt_s,ifc_loop_n,ifc_hz_est,ifc_raw_dt_max_s,ifc_raw_dt_min_s,phase_el_s,status" TO LOG_FILE.

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
    ROUND(SHIP:VERTICALSPEED,         2),
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
    ROUND(TELEM_AA_HDG_CMD,           2),
    ROUND(TELEM_AA_FPA_CMD,           3),
    ROUND(ILS_LOC_DEV,                2),
    ROUND(ILS_GS_DEV,                 2),
    ROUND(ILS_DIST_M / 1000,          3),
    ROUND(TELEM_LOC_CORR,             3),
    ROUND(TELEM_GS_CORR,              3),
    ROUND(FLARE_PITCH_CMD,            3),
    ROUND(TELEM_FLARE_TGT_VS,         3),
    ROUND(TELEM_FLARE_FRAC,           3),
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
    SHIP:STATUS
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
