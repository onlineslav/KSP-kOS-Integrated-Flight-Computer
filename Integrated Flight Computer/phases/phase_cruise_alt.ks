@LAZYGLOBAL OFF.

// ============================================================
// phase_cruise_alt.ks  -  Integrated Flight Computer
// Enroute cruise navigation phase (alternate implementation).
//
// Drop-in replacement contract for phase_cruise.ks:
// - Exports RUN_CRUISE
// - Reads/writes the same CRUISE_* and TELEM_* globals
// - Advances waypoints and sets PHASE_DONE identically
//
// Cruise policy:
// - IFC autothrottle always owns throttle.
// - AA native SPEEDCONTROL is never used.
// - Lateral guidance uses AA CRUISE heading.
// - Vertical guidance uses AA CRUISE native ALTITUDE hold.
// ============================================================

// Legacy throttle path kept for drop-in compatibility and fallback.
FUNCTION _RUN_CRUISE_THROTTLE {
  AT_RUN_SPEED_HOLD(CRUISE_SPD_MPS, 0, 1).
}

FUNCTION _CRUISE_AA_GET_HANDLE {
  IF NOT ADDONS:AVAILABLE("AA") { RETURN 0. }
  IF NOT ADDONS:HASSUFFIX("AA") { RETURN 0. }
  LOCAL aa_handle IS ADDONS:AA.
  IF aa_handle = 0 { RETURN 0. }
  RETURN aa_handle.
}

// Ensure AA speed control cannot leak into other phases when cruise exits
// or when we fall back to IFC autothrottle.
// Also restores FBW in case native CRUISE mode disabled it — the approach
// phase's Director mode relies on FBW for inner-loop stability.
FUNCTION _CRUISE_AA_RELEASE {
  LOCAL aa IS _CRUISE_AA_GET_HANDLE().
  IF aa = 0 { RETURN. }
  IF aa:HASSUFFIX("SPEEDCONTROL") AND aa:SPEEDCONTROL { SET aa:SPEEDCONTROL TO FALSE. }
  IF aa:HASSUFFIX("CRUISE") AND aa:CRUISE { SET aa:CRUISE TO FALSE. }
  IF aa:HASSUFFIX("FBW") AND NOT aa:FBW { SET aa:FBW TO TRUE. }
}

// Legacy helper retained for compatibility; unused when AA native speed mode is disabled.
FUNCTION _CRUISE_HANDOFF_TO_AA_THROTTLE {
  UNLOCK THROTTLE.
}

FUNCTION _CRUISE_HANDOFF_TO_IFC_THROTTLE {
  // Preserve current thrust when taking lock ownership back.
  SET THROTTLE_CMD TO GET_CURRENT_THROTTLE().
  LOCK THROTTLE TO THROTTLE_CMD.
}

// Run AA CRUISE heading + ALTITUDE hold while IFC autothrottle owns speed.
// Returns TRUE when AA cruise guidance is active.
FUNCTION _RUN_CRUISE_AA_CRUISE {
  PARAMETER hdg_cmd_deg, tgt_alt_m.
  LOCAL aa IS _CRUISE_AA_GET_HANDLE().
  IF aa = 0 { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("CRUISE") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("HEADING") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("ALTITUDE") { RETURN FALSE. }

  // IFC owns throttle in all phases.
  IF aa:HASSUFFIX("SPEEDCONTROL") AND aa:SPEEDCONTROL { SET aa:SPEEDCONTROL TO FALSE. }

  // CRUISE mode is mutually exclusive with DIRECTOR/FBW.
  IF aa:HASSUFFIX("DIRECTOR") AND aa:DIRECTOR { SET aa:DIRECTOR TO FALSE. }
  IF aa:HASSUFFIX("FBW") AND aa:FBW { SET aa:FBW TO FALSE. }
  IF NOT aa:CRUISE { SET aa:CRUISE TO TRUE. }
  SET aa:HEADING TO WRAP_360(hdg_cmd_deg).
  SET aa:ALTITUDE TO MAX(tgt_alt_m, 0).
  RETURN TRUE.
}

// Retract flaps to the "up" detent for cruise.
// Mirrors _CHECK_TAKEOFF_FLAPS: steps one detent per FLAP_STEP_INTERVAL
// using the aircraft's configured flap action groups.
FUNCTION _CRUISE_CHECK_FLAPS {
  LOCAL ac IS ACTIVE_AIRCRAFT.
  IF ac = 0 { RETURN. }
  IF NOT ac:HASKEY("ag_flaps_step_up") OR NOT ac:HASKEY("ag_flaps_step_down") { RETURN. }
  LOCAL ag_up IS ac["ag_flaps_step_up"].
  LOCAL ag_dn IS ac["ag_flaps_step_down"].
  IF ag_up <= 0 OR ag_dn <= 0 { RETURN. }

  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent",  3, 0)).
  LOCAL det_up  IS ROUND(AC_PARAM("flaps_detent_up",   0, 0)).
  SET FLAPS_TARGET_DETENT TO CLAMP(det_up, 0, max_det).

  IF FLAPS_CURRENT_DETENT = FLAPS_TARGET_DETENT { RETURN. }
  IF TIME:SECONDS - FLAPS_LAST_STEP_UT < FLAP_STEP_INTERVAL { RETURN. }

  IF FLAPS_CURRENT_DETENT < FLAPS_TARGET_DETENT {
    PULSE_AG(ag_up).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
  } ELSE {
    PULSE_AG(ag_dn).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
  }
  SET IFC_ALERT_TEXT TO "CRUISE FLAPS -> detent " + FLAPS_CURRENT_DETENT.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET FLAPS_LAST_STEP_UT TO TIME:SECONDS.
}

FUNCTION _CRUISE_COMPLETE {
  _CRUISE_AA_RELEASE().
  _CRUISE_HANDOFF_TO_IFC_THROTTLE().
  SET IFC_ALERT_TEXT TO "CRUISE complete".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET_PHASE(PHASE_DONE).
}

FUNCTION RUN_CRUISE {
  _CRUISE_CHECK_FLAPS().

  // Course + time mode: fly fixed heading until timer expires.
  IF CRUISE_NAV_TYPE = "course_time" {
    IF TIME:SECONDS >= CRUISE_END_UT {
      _CRUISE_COMPLETE().
      RETURN.
    }

    LOCAL tgt_alt IS CRUISE_ALT_M.
    LOCAL alt_err IS SHIP:ALTITUDE - tgt_alt.
    LOCAL fpa_cmd IS CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).

    LOCAL using_native IS _RUN_CRUISE_AA_CRUISE(CRUISE_COURSE_DEG, tgt_alt).
    IF NOT using_native { AA_SET_DIRECTOR(CRUISE_COURSE_DEG, fpa_cmd). }
    _RUN_CRUISE_THROTTLE().

    SET TELEM_AA_HDG_CMD TO CRUISE_COURSE_DEG.
    SET TELEM_AA_FPA_CMD TO CHOOSE 0 IF using_native ELSE fpa_cmd.
    SET TELEM_LOC_CORR   TO 0.
    SET TELEM_GS_CORR    TO 0.
    RETURN.
  }

  // Waypoint mode (also handles course_dist via synthetic "_CRUISE_TGT" beacon).
  IF CRUISE_WP_INDEX >= CRUISE_WAYPOINTS:LENGTH {
    _CRUISE_COMPLETE().
    RETURN.
  }

  LOCAL wp_id IS CRUISE_WAYPOINTS[CRUISE_WP_INDEX].
  LOCAL wp    IS GET_BEACON(wp_id).
  LOCAL wp_ll IS wp["ll"].
  LOCAL brg   IS GEO_BEARING(SHIP:GEOPOSITION, wp_ll).
  LOCAL dist  IS GEO_DISTANCE(SHIP:GEOPOSITION, wp_ll).

  // Altitude target: hold cruise altitude while far out; blend down
  // to waypoint's published altitude within CRUISE_DESCENT_START_M.
  LOCAL tgt_alt IS CRUISE_ALT_M.
  IF dist < CRUISE_DESCENT_START_M {
    LOCAL wp_alt IS wp["alt_asl"].
    LOCAL blend  IS CLAMP(1 - dist / CRUISE_DESCENT_START_M, 0, 1).
    SET tgt_alt  TO CRUISE_ALT_M + (wp_alt - CRUISE_ALT_M) * blend.
  }

  LOCAL alt_err IS SHIP:ALTITUDE - tgt_alt.
  LOCAL fpa_cmd IS CLAMP(
    -(alt_err * KP_ALT_FPA + SHIP:VERTICALSPEED * KD_ALT_FPA),
    MAX_DESC_FPA, MAX_CLIMB_FPA
  ).

  // Keep telemetry/fallback behavior aligned with phase_cruise.ks:
  // suppress descent during large heading changes.
  LOCAL hdg_err IS WRAP_360(brg - GET_COMPASS_HDG()).
  IF hdg_err > 180 { SET hdg_err TO 360 - hdg_err. }
  IF hdg_err > 45 AND fpa_cmd < 0 { SET fpa_cmd TO 0. }

  LOCAL using_native IS _RUN_CRUISE_AA_CRUISE(brg, tgt_alt).
  IF NOT using_native { AA_SET_DIRECTOR(brg, fpa_cmd). }
  _RUN_CRUISE_THROTTLE().

  SET TELEM_AA_HDG_CMD TO brg.
  SET TELEM_AA_FPA_CMD TO CHOOSE 0 IF using_native ELSE fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  // Waypoint capture.
  IF dist < FIX_CAPTURE_RADIUS {
    SET IFC_ALERT_TEXT TO "CRUISE WPT: " + wp_id + "  (" + ROUND(dist) + " m)".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET CRUISE_WP_INDEX TO CRUISE_WP_INDEX + 1.
  }
}
