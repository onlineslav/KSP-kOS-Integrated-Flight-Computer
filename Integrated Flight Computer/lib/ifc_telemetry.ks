@LAZYGLOBAL OFF.

// ============================================================
// ifc_telemetry.ks  -  Integrated Flight Computer
// Rate-limited HUD display.
// ============================================================

// Call each loop cycle; internally rate-limits to IFC_TELEMETRY_PERIOD.
FUNCTION PRINT_TELEMETRY {
  LOCAL now IS TIME:SECONDS.
  IF (now - LAST_TELEM_UT) < IFC_TELEMETRY_PERIOD { RETURN. }
  SET LAST_TELEM_UT TO now.

  LOCAL t IS ROUND(now - IFC_MISSION_START_UT, 1).
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL agl IS ROUND(GET_AGL(), 0).
  LOCAL vs  IS ROUND(SHIP:VERTICALSPEED, 1).
  LOCAL hdg IS ROUND(SHIP:HEADING, 1).
  LOCAL pitch IS ROUND(GET_PITCH(), 1).
  LOCAL thr IS ROUND(THROTTLE_CMD, 2).

  // ── Line 1: flight state ─────────────────────────────────
  PRINT "IFC T+" + t + "  [" + IFC_PHASE + " / " + IFC_SUBPHASE + "]" AT (0,0).

  // ── Line 2: air data ─────────────────────────────────────
  PRINT "IAS " + ias + " m/s   AGL " + agl + " m   VS " + vs + " m/s   THR " + thr
      + "   FLP " + FLAPS_CURRENT_DETENT + "->" + FLAPS_TARGET_DETENT AT (0,1).

  // ── Line 3: attitude ─────────────────────────────────────
  PRINT "HDG " + hdg + " deg   PITCH " + pitch + " deg                    " AT (0,2).

  // ── Line 4: ILS deviations (only meaningful during ILS_TRACK) ──
  IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK OR IFC_PHASE = PHASE_FLARE {
    LOCAL loc  IS ROUND(ILS_LOC_DEV, 1).
    LOCAL gs   IS ROUND(ILS_GS_DEV, 1).
    LOCAL dist IS ROUND(ILS_DIST_M / 1000, 2).
    LOCAL loc_arrow IS ">".
    IF loc > 0 { SET loc_arrow TO "R". }
    ELSE IF loc < 0 { SET loc_arrow TO "L". }
    LOCAL gs_arrow IS "-".
    IF gs > 0 { SET gs_arrow TO "^". }
    ELSE IF gs < 0 { SET gs_arrow TO "v". }

    PRINT "LOC " + loc + " m [" + loc_arrow + "]   GS " + gs + " m [" + gs_arrow + "]   DIST " + dist + " km" AT (0,3).
  } ELSE IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX AND ACTIVE_FIXES:LENGTH > 0 AND FIX_INDEX < ACTIVE_FIXES:LENGTH {
    LOCAL fix_id IS ACTIVE_FIXES[FIX_INDEX].
    LOCAL fix    IS GET_BEACON(fix_id).
    LOCAL dist_to_fix IS ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, fix["ll"]) / 1000, 1).
    LOCAL brg_to_fix  IS ROUND(GEO_BEARING(SHIP:GEOPOSITION, fix["ll"]), 1).
    PRINT "FIX [" + fix_id + "]  dist " + dist_to_fix + " km   brg " + brg_to_fix + " deg          " AT (0,3).
  } ELSE {
    PRINT "                                                  " AT (0,3).
  }

  // ── Line 5: runway / approach info ───────────────────────
  IF ACTIVE_ILS_ID <> "" {
    PRINT "RWY " + ACTIVE_ILS_ID + "  hdg " + ACTIVE_RWY_HDG + "  GS " + ACTIVE_GS_ANGLE + " deg  Vapp " + ACTIVE_V_APP + " m/s" AT (0,4).
  }
}
