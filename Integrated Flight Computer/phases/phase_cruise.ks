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
  AT_RUN_SPEED_HOLD(CRUISE_SPD_MPS, 0, 1).
}

// ── Main cruise tick ─────────────────────────────────────
FUNCTION RUN_CRUISE {
  // Course + time mode: fly a fixed heading until the timer expires.
  IF CRUISE_NAV_TYPE = "course_time" {
    IF TIME:SECONDS >= CRUISE_END_UT {
      SET IFC_ALERT_TEXT TO "CRUISE complete".
      SET IFC_ALERT_UT   TO TIME:SECONDS.
      SET_PHASE(PHASE_DONE).
      RETURN.
    }
    LOCAL alt_err IS SHIP:ALTITUDE - CRUISE_ALT_M.
    LOCAL fpa_cmd IS CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).
    AA_SET_DIRECTOR_FPA(CRUISE_COURSE_DEG, fpa_cmd).
    SET TELEM_AA_HDG_CMD TO CRUISE_COURSE_DEG.
    SET TELEM_AA_FPA_CMD TO fpa_cmd.
    SET TELEM_LOC_CORR   TO 0.
    SET TELEM_GS_CORR    TO 0.
    _RUN_CRUISE_THROTTLE().
    RETURN.
  }

  // Waypoint mode (also handles course_dist via synthetic "_CRUISE_TGT" beacon).
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
  LOCAL fpa_cmd IS CLAMP(
    -(alt_err * KP_ALT_FPA + SHIP:VERTICALSPEED * KD_ALT_FPA),
    MAX_DESC_FPA, MAX_CLIMB_FPA
  ).
  // Suppress descent during large heading changes to prevent spiral dives.
  // NOTE: SHIP:HEADING returns 0 in kOS; use the vector-based GET_COMPASS_HDG().
  LOCAL hdg_err IS WRAP_360(brg - GET_COMPASS_HDG()).
  IF hdg_err > 180 { SET hdg_err TO 360 - hdg_err. }
  IF hdg_err > 45 AND fpa_cmd < 0 { SET fpa_cmd TO 0. }

  AA_SET_DIRECTOR_FPA(brg, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO brg.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _RUN_CRUISE_THROTTLE().

  // Seed start-distance the first time this waypoint is targeted.
  IF CRUISE_WP_START_DIST <= 0 { SET CRUISE_WP_START_DIST TO dist. }

  // Waypoint capture.
  // Guard: require the aircraft to have closed to 70 % of its starting distance
  // to this waypoint (or be within 300 m as a failsafe).  For a normal far
  // waypoint (start >> FIX_CAPTURE_RADIUS) this is invisible — the radius gate
  // fires first.  For a close waypoint (e.g. a tight triangle where the next WP
  // is already within FIX_CAPTURE_RADIUS when we switch to it), capture cannot
  // fire until the aircraft has demonstrably approached it, preventing cascaded
  // skipping regardless of the triangle's internal angles.
  IF dist < FIX_CAPTURE_RADIUS AND (dist < 300 OR dist <= CRUISE_WP_START_DIST * 0.7) {
    SET IFC_ALERT_TEXT TO "CRUISE WPT: " + wp_id + "  (" + ROUND(dist) + " m)".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET CRUISE_WP_INDEX      TO CRUISE_WP_INDEX + 1.
    SET CRUISE_WP_START_DIST TO 0.
  }
}
