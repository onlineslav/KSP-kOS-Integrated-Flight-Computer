@LAZYGLOBAL OFF.

// ============================================================
// phase_cruise.ks  -  Integrated Flight Computer
// Enroute cruise navigation phase.
//
// Navigates CRUISE_WAYPOINTS in order using AA Director.
// Holds CRUISE_ALT_M until within CRUISE_DESCENT_START_M of
// each waypoint, then blends down toward the waypoint's alt_asl.
// After all waypoints are captured, sets PHASE_DONE so the
// flight plan leg queue can advance to the next leg.
// ============================================================

// ── Autothrottle ─────────────────────────────────────────
// Same cascade structure as the approach throttle,
// but targets CRUISE_SPD_MPS directly.
FUNCTION _RUN_CRUISE_THROTTLE {
  LOCAL ias IS GET_IAS().
  LOCAL spd_err IS CRUISE_SPD_MPS - ias.
  LOCAL a_cmd IS CLAMP(KP_SPD_ACL * spd_err, -ACL_MAX, ACL_MAX).
  LOCAL a_raw IS CLAMP((ias - PREV_IAS) / IFC_ACTUAL_DT, -10, 10).
  SET PREV_IAS      TO ias.
  SET A_ACTUAL_FILT TO A_ACTUAL_FILT * ACL_FILTER_ALPHA + a_raw * (1 - ACL_FILTER_ALPHA).
  LOCAL a_err IS a_cmd - A_ACTUAL_FILT.
  SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * IFC_ACTUAL_DT, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  LOCAL raw_thr IS CLAMP(KP_ACL_THR * a_err + KI_SPD * THR_INTEGRAL, 0, 1).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).
}

// ── Main cruise tick ─────────────────────────────────────
FUNCTION RUN_CRUISE {
  IF CRUISE_WP_INDEX >= CRUISE_WAYPOINTS:LENGTH {
    SET IFC_ALERT_TEXT TO "CRUISE complete".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET_PHASE(PHASE_DONE).
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
  LOCAL fpa_cmd IS CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).

  AA_SET_DIRECTOR(brg, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO brg.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _RUN_CRUISE_THROTTLE().

  // Waypoint capture.
  IF dist < FIX_CAPTURE_RADIUS {
    SET IFC_ALERT_TEXT TO "CRUISE WPT: " + wp_id + "  (" + ROUND(dist) + " m)".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET CRUISE_WP_INDEX TO CRUISE_WP_INDEX + 1.
  }
}
