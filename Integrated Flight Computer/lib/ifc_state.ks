@LAZYGLOBAL OFF.

// ============================================================
// ifc_state.ks  -  Integrated Flight Computer
// All mutable runtime globals.  Reset via IFC_INIT_STATE().
// ============================================================

// ----------------------------
// Phase state
// ----------------------------
GLOBAL IFC_PHASE          IS PHASE_APPROACH.
GLOBAL IFC_SUBPHASE       IS SUBPHASE_FLY_TO_FIX.
GLOBAL IFC_PHASE_START_UT IS 0.
GLOBAL IFC_MISSION_START_UT IS 0.

// ----------------------------
// Active approach
// ----------------------------
// Set these before starting the approach loop.
// ACTIVE_PLATE is the approach plate lexicon from nav_beacons.ks.
// ACTIVE_AIRCRAFT is the aircraft config lexicon from an aircraft/*.ks file.
GLOBAL ACTIVE_PLATE    IS 0.  // set before run
GLOBAL ACTIVE_AIRCRAFT IS 0.  // set before run

// Derived from ACTIVE_PLATE at approach start (cached for speed):
GLOBAL ACTIVE_ILS_ID   IS "".
GLOBAL ACTIVE_RWY_HDG  IS 90.
GLOBAL ACTIVE_GS_ANGLE IS 3.0.
GLOBAL ACTIVE_THR_ALT  IS 69.
GLOBAL ACTIVE_V_APP    IS 80.
GLOBAL ACTIVE_V_TGT    IS 80. // current dynamic approach speed target (m/s)
GLOBAL ACTIVE_FIXES    IS LIST().  // ordered list of beacon IDs
GLOBAL ACTIVE_ALT_AT   IS LEXICON(). // beacon_id -> target altitude (m)

// Index into ACTIVE_FIXES for FLY_TO_FIX sub-phase
GLOBAL FIX_INDEX IS 0.

// ----------------------------
// Throttle command
// ----------------------------
GLOBAL THROTTLE_CMD     IS 0.
GLOBAL THR_INTEGRAL     IS 0.   // accumulated speed error for integral trim
GLOBAL PREV_IAS         IS 0.   // IAS from previous cycle for d(IAS)/dt measurement
GLOBAL A_ACTUAL_FILT    IS 0.   // low-pass filtered measured acceleration (m/s²)
GLOBAL APP_ON_FINAL IS FALSE. // TRUE once LOC/GS capture is stable for final-speed mode
GLOBAL APP_FINAL_ARM_UT IS -1. // timer used to debounce entry to final-speed mode
GLOBAL APP_SPD_MODE IS "INIT". // FIXES / INTERCEPT / FINAL / SHORT_FINAL
GLOBAL APP_VREF_TGT IS 0. // current computed Vref used by scheduler (m/s)
GLOBAL APP_VINT_TGT IS 0. // current computed intercept speed target (m/s)
GLOBAL APP_BASE_V_TGT IS 0. // current unslewed phase target speed (m/s)
GLOBAL APP_SHORT_FINAL_FRAC IS 0. // 0..1 short-final blend fraction (Vapp->Vref)
GLOBAL APP_LOC_CAP_OK IS 0. // 1 when |LOC| is inside capture band, else 0
GLOBAL APP_GS_CAP_OK IS 0.  // 1 when |GS| is inside capture band, else 0

// ----------------------------
// ILS deviation state  (updated each cycle in ILS_TRACK)
// ----------------------------
GLOBAL ILS_LOC_DEV  IS 0.   // m, + = right of centerline
GLOBAL ILS_GS_DEV   IS 0.   // m, + = above glideslope
GLOBAL ILS_DIST_M   IS 0.   // m  horizontal from threshold (+ = on approach)

// Previous-cycle values for derivative term
GLOBAL PREV_LOC_DEV IS 0.
GLOBAL PREV_GS_DEV  IS 0.

// ----------------------------
// AA state flags
// ----------------------------
GLOBAL AA_AVAILABLE    IS FALSE.
GLOBAL AA_DIRECTOR_ON  IS FALSE.
GLOBAL AA_FBW_ON       IS FALSE.

// ----------------------------
// FAR state flag
// ----------------------------
GLOBAL FAR_AVAILABLE IS FALSE.

// ----------------------------
// Loop timing
// ----------------------------
GLOBAL IFC_CYCLE_UT  IS 0.    // TIME:SECONDS at start of last cycle (for real dt)
GLOBAL IFC_ACTUAL_DT IS 0.05. // measured elapsed time of the most recent loop cycle (s)

// ----------------------------
// Telemetry
// ----------------------------
GLOBAL LAST_TELEM_UT IS 0.

// ----------------------------
// Telemetry export
// Written each cycle by phase functions so ifc_logger can read them
// without needing access to local variables.
// ----------------------------
GLOBAL TELEM_AA_HDG_CMD    IS 0.  // heading sent to AA Director (deg)
GLOBAL TELEM_AA_FPA_CMD    IS 0.  // FPA sent to AA Director (deg)
GLOBAL TELEM_LOC_CORR      IS 0.  // ILS localizer heading correction (deg)
GLOBAL TELEM_GS_CORR       IS 0.  // ILS glideslope FPA correction (deg)
GLOBAL TELEM_FLARE_TGT_VS  IS 0.  // target sink rate from flare schedule (m/s)
GLOBAL TELEM_FLARE_FRAC    IS 0.  // flare progress: 0 = entry AGL, 1 = ground
GLOBAL TELEM_STEER_BLEND   IS 0.  // rollout wheelsteering blend factor (0-1)
GLOBAL TELEM_RO_HDG_ERR    IS 0.  // rollout: actual hdg minus steer target (deg)
GLOBAL TELEM_RO_YAW_TGT    IS 0.  // rollout: yaw command target before slew rate
GLOBAL TELEM_RO_LOC_CORR   IS 0.  // rollout: heading correction from localizer (deg)
GLOBAL TELEM_RO_ROLL_ASSIST IS 0. // rollout: 1 when IFC roll assist is active, else 0
GLOBAL TELEM_RO_YAW_SCALE  IS 0.  // rollout: yaw assist blend factor (0-1)
GLOBAL TELEM_RO_YAW_GATE   IS 0.  // rollout yaw gate: 0=active,1=speed,2=guard,3=not landed
GLOBAL TELEM_RO_PITCH_TGT  IS 0.  // rollout: target pitch attitude (deg)
GLOBAL TELEM_RO_PITCH_ERR  IS 0.  // rollout: pitch attitude error (deg)
GLOBAL TELEM_RO_PITCH_FF   IS 0.  // rollout: feedforward pitch command bias

// ----------------------------
// Flap detent state
// ----------------------------
GLOBAL FLAPS_CURRENT_DETENT    IS 0.
GLOBAL FLAPS_TARGET_DETENT     IS 0.
GLOBAL FLAPS_LAST_STEP_UT      IS 0.
GLOBAL FLAPS_LAST_TARGET_LOGGED IS -1.

// ----------------------------
// Autoland phase state
// ----------------------------
GLOBAL FLARE_PITCH_CMD  IS 0.   // current commanded FPA during flare (deg, smoothed)
GLOBAL FLARE_ENTRY_VS   IS 0.   // vertical speed at flare trigger (m/s, negative)
GLOBAL FLARE_ENTRY_AGL  IS 15.  // AGL at flare trigger (m)
GLOBAL FLARE_TRIGGER_START_UT IS -1. // debounce timer start for entering flare
GLOBAL TOUCHDOWN_CANDIDATE_UT IS -1. // debounce timer start for entering touchdown
GLOBAL TOUCHDOWN_INIT_DONE IS FALSE. // one-time touchdown handoff latch
GLOBAL TOUCHDOWN_CAPTURE_PITCH_DEG IS 0. // pitch snapshot captured at flare->touchdown transition
GLOBAL BOUNCE_RECOVERY_START_UT IS -1. // debounce timer for confirming a real airborne bounce
GLOBAL ROLLOUT_ENTRY_HDG IS 0.  // heading captured at touchdown for blended wheelsteering
GLOBAL ROLLOUT_YAW_CMD_PREV IS 0. // previous-cycle rudder command for slew limiting
GLOBAL ROLLOUT_PITCH_CMD_PREV IS 0. // previous-cycle pitch command for slew limiting
GLOBAL ROLLOUT_PITCH_REF_DEG IS 0. // touchdown-captured pitch attitude reference (deg)
GLOBAL ROLLOUT_PITCH_TGT_DEG IS 0. // time-evolving pitch target for rollout nose lowering (deg)
GLOBAL ROLLOUT_STEER_HDG IS 0. // computed wheelsteering target (logged for telemetry)

// ----------------------------
// Init / reset
// ----------------------------
FUNCTION IFC_INIT_STATE {
  SET IFC_PHASE          TO PHASE_APPROACH.
  SET IFC_SUBPHASE       TO SUBPHASE_FLY_TO_FIX.
  SET IFC_PHASE_START_UT TO TIME:SECONDS.
  SET IFC_MISSION_START_UT TO TIME:SECONDS.

  SET ACTIVE_ILS_ID   TO "".
  SET ACTIVE_RWY_HDG  TO 90.
  SET ACTIVE_GS_ANGLE TO 3.0.
  SET ACTIVE_THR_ALT  TO 69.
  SET ACTIVE_V_APP    TO 80.
  SET ACTIVE_V_TGT    TO 80.
  SET ACTIVE_FIXES    TO LIST().
  SET ACTIVE_ALT_AT   TO LEXICON().
  SET FIX_INDEX TO 0.

  SET THROTTLE_CMD  TO 0.
  SET THR_INTEGRAL  TO 0.
  SET PREV_IAS      TO GET_IAS().
  SET A_ACTUAL_FILT TO 0.
  SET APP_ON_FINAL TO FALSE.
  SET APP_FINAL_ARM_UT TO -1.
  SET APP_SPD_MODE TO "INIT".
  SET APP_VREF_TGT TO ACTIVE_V_APP - 10.
  SET APP_VINT_TGT TO ACTIVE_V_APP.
  SET APP_BASE_V_TGT TO ACTIVE_V_APP.
  SET APP_SHORT_FINAL_FRAC TO 0.
  SET APP_LOC_CAP_OK TO 0.
  SET APP_GS_CAP_OK TO 0.

  SET ILS_LOC_DEV  TO 0.
  SET ILS_GS_DEV   TO 0.
  SET ILS_DIST_M   TO 0.
  SET PREV_LOC_DEV TO 0.
  SET PREV_GS_DEV  TO 0.

  SET AA_AVAILABLE   TO FALSE.
  SET AA_DIRECTOR_ON TO FALSE.
  SET AA_FBW_ON      TO FALSE.
  SET FAR_AVAILABLE  TO FALSE.

  SET LAST_TELEM_UT TO TIME:SECONDS.
  SET IFC_CYCLE_UT  TO TIME:SECONDS.
  SET IFC_ACTUAL_DT TO IFC_LOOP_DT.
  SET FLARE_PITCH_CMD  TO 0.
  SET FLARE_ENTRY_VS   TO 0.
  SET FLARE_ENTRY_AGL  TO FLARE_AGL_M.
  SET FLARE_TRIGGER_START_UT TO -1.
  SET TOUCHDOWN_CANDIDATE_UT TO -1.
  SET TOUCHDOWN_INIT_DONE TO FALSE.
  SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
  SET BOUNCE_RECOVERY_START_UT TO -1.
  SET ROLLOUT_ENTRY_HDG   TO GET_COMPASS_HDG().
  SET ROLLOUT_YAW_CMD_PREV TO 0.
  SET ROLLOUT_PITCH_CMD_PREV TO 0.
  SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
  SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
  SET ROLLOUT_STEER_HDG   TO GET_COMPASS_HDG().

  SET TELEM_AA_HDG_CMD   TO 0.
  SET TELEM_AA_FPA_CMD   TO 0.
  SET TELEM_LOC_CORR     TO 0.
  SET TELEM_GS_CORR      TO 0.
  SET TELEM_FLARE_TGT_VS TO 0.
  SET TELEM_FLARE_FRAC   TO 0.
  SET TELEM_STEER_BLEND  TO 0.
  SET TELEM_RO_HDG_ERR   TO 0.
  SET TELEM_RO_YAW_TGT   TO 0.
  SET TELEM_RO_LOC_CORR  TO 0.
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE TO 0.
  SET TELEM_RO_YAW_GATE TO 0.
  SET TELEM_RO_PITCH_TGT TO 0.
  SET TELEM_RO_PITCH_ERR TO 0.
  SET TELEM_RO_PITCH_FF TO 0.

  LOCAL initial_det IS 0.
  LOCAL max_det     IS 3.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_max_detent") {
    SET max_det TO MAX(0, ROUND(ACTIVE_AIRCRAFT["flaps_max_detent"], 0)).
  }
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_initial_detent") {
    SET initial_det TO CLAMP(ROUND(ACTIVE_AIRCRAFT["flaps_initial_detent"], 0), 0, max_det).
  }

  SET FLAPS_CURRENT_DETENT     TO initial_det.
  SET FLAPS_TARGET_DETENT      TO initial_det.
  SET FLAPS_LAST_STEP_UT       TO TIME:SECONDS.
  SET FLAPS_LAST_TARGET_LOGGED TO -1.
}

// Called once after ACTIVE_PLATE and ACTIVE_AIRCRAFT are set.
FUNCTION IFC_LOAD_PLATE {
  SET ACTIVE_ILS_ID   TO ACTIVE_PLATE["ils_id"].
  SET ACTIVE_RWY_HDG  TO GET_BEACON(ACTIVE_ILS_ID)["hdg"].
  SET ACTIVE_GS_ANGLE TO GET_BEACON(ACTIVE_ILS_ID)["gs_angle"].
  SET ACTIVE_THR_ALT  TO GET_BEACON(ACTIVE_ILS_ID)["alt_asl"].
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("v_app") {
    SET ACTIVE_V_APP  TO ACTIVE_AIRCRAFT["v_app"].
  } ELSE {
    SET ACTIVE_V_APP  TO ACTIVE_PLATE["vapp"].
  }
  SET ACTIVE_V_TGT TO ACTIVE_V_APP.
  SET APP_ON_FINAL TO FALSE.
  SET APP_FINAL_ARM_UT TO -1.
  SET APP_SPD_MODE TO "FIXES".
  SET APP_VREF_TGT TO ACTIVE_V_APP - 10.
  SET APP_VINT_TGT TO ACTIVE_V_APP.
  SET APP_BASE_V_TGT TO ACTIVE_V_APP.
  SET APP_SHORT_FINAL_FRAC TO 0.
  SET APP_LOC_CAP_OK TO 0.
  SET APP_GS_CAP_OK TO 0.
  SET ACTIVE_FIXES    TO ACTIVE_PLATE["fixes"].
  SET ACTIVE_ALT_AT   TO ACTIVE_PLATE["alt_at"].
  SET FIX_INDEX TO 0.
}
