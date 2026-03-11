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
// MISC
//   phase_el_s     time elapsed in current phase (s)
//   status         SHIP:STATUS string
// ============================================================

GLOBAL LOG_ACTIVE IS FALSE.
GLOBAL LOG_FILE   IS "".

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

  LOG "t_s,phase,subphase,ias_ms,vapp_ms,spd_err_ms,agl_m,vs_ms,pitch_deg,aoa_deg,hdg_deg,bank_deg,thr_cmd,thr_intg,aa_hdg_cmd_deg,aa_fpa_cmd_deg,ils_loc_m,ils_gs_m,ils_dist_km,loc_corr_deg,gs_corr_deg,flare_fpa_cmd,flare_tgt_vs,flare_frac,steer_hdg_deg,steer_blend,ro_loc_corr_deg,ro_hdg_err_deg,ro_yaw_tgt,ro_yaw_scale,ro_yaw_gate,yaw_cmd,roll_cmd,pitch_cmd,ro_pitch_tgt_deg,ro_pitch_err_deg,ro_pitch_ff,ro_roll_assist,flaps_cur,flaps_tgt,phase_el_s,status" TO LOG_FILE.

  SET LOG_ACTIVE TO TRUE.
  SET IFC_ALERT_TEXT TO "Logging -> " + LOG_FILE.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION LOGGER_WRITE {
  IF NOT LOG_ACTIVE { RETURN. }

  LOCAL ias   IS GET_IAS().
  LOCAL v_tgt IS ACTIVE_V_APP.
  IF IFC_PHASE = PHASE_APPROACH { SET v_tgt TO ACTIVE_V_TGT. }
  LOCAL bank  IS ROUND(90 - VECTORANGLE(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), 3).

  LOCAL row IS
    ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 2) + "," +
    IFC_PHASE                              + "," +
    IFC_SUBPHASE                           + "," +
    ROUND(ias,                        2)   + "," +
    ROUND(v_tgt,                      2)   + "," +
    ROUND(v_tgt - ias,                2)   + "," +
    ROUND(GET_AGL(),                  2)   + "," +
    ROUND(SHIP:VERTICALSPEED,         2)   + "," +
    ROUND(GET_PITCH(),                2)   + "," +
    ROUND(GET_AOA(),                  2)   + "," +
    ROUND(GET_COMPASS_HDG(),          2)   + "," +
    bank                                   + "," +
    ROUND(THROTTLE_CMD,               4)   + "," +
    ROUND(THR_INTEGRAL,               3)   + "," +
    ROUND(TELEM_AA_HDG_CMD,           2)   + "," +
    ROUND(TELEM_AA_FPA_CMD,           3)   + "," +
    ROUND(ILS_LOC_DEV,                2)   + "," +
    ROUND(ILS_GS_DEV,                 2)   + "," +
    ROUND(ILS_DIST_M / 1000,          3)   + "," +
    ROUND(TELEM_LOC_CORR,             3)   + "," +
    ROUND(TELEM_GS_CORR,              3)   + "," +
    ROUND(FLARE_PITCH_CMD,            3)   + "," +
    ROUND(TELEM_FLARE_TGT_VS,         3)   + "," +
    ROUND(TELEM_FLARE_FRAC,           3)   + "," +
    ROUND(ROLLOUT_STEER_HDG,          2)   + "," +
    ROUND(TELEM_STEER_BLEND,          3)   + "," +
    ROUND(TELEM_RO_LOC_CORR,          3)   + "," +
    ROUND(TELEM_RO_HDG_ERR,           3)   + "," +
    ROUND(TELEM_RO_YAW_TGT,           4)   + "," +
    ROUND(TELEM_RO_YAW_SCALE,         3)   + "," +
    ROUND(TELEM_RO_YAW_GATE,          0)   + "," +
    ROUND(SHIP:CONTROL:YAW,           4)   + "," +
    ROUND(SHIP:CONTROL:ROLL,          4)   + "," +
    ROUND(SHIP:CONTROL:PITCH,         4)   + "," +
    ROUND(TELEM_RO_PITCH_TGT,         3)   + "," +
    ROUND(TELEM_RO_PITCH_ERR,         3)   + "," +
    ROUND(TELEM_RO_PITCH_FF,          4)   + "," +
    ROUND(TELEM_RO_ROLL_ASSIST,       0)   + "," +
    FLAPS_CURRENT_DETENT                   + "," +
    FLAPS_TARGET_DETENT                    + "," +
    ROUND(PHASE_ELAPSED(),            2)   + "," +
    SHIP:STATUS.

  LOG row TO LOG_FILE.
}

FUNCTION LOGGER_CLOSE {
  IF NOT LOG_ACTIVE { RETURN. }
  LOG "# end T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1) +
      "  IAS " + ROUND(GET_IAS(), 1) + " m/s" TO LOG_FILE.
  SET LOG_ACTIVE TO FALSE.
  SET IFC_ALERT_TEXT TO "Log saved -> " + LOG_FILE.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}
