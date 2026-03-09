@LAZYGLOBAL OFF.

// ============================================================
// phase_approach.ks  -  Integrated Flight Computer
//
// Handles the APPROACH top-level phase, which contains two
// sub-phases driven by IFC_SUBPHASE:
//
//   SUBPHASE_FLY_TO_FIX  →  navigate the IAF/FAF fix sequence
//   SUBPHASE_ILS_TRACK   →  coupled localizer + glideslope tracking
//
// The main loop calls RUN_APPROACH() once per cycle.
// ============================================================

// ── Public entry point ────────────────────────────────────
FUNCTION RUN_APPROACH {
  IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX {
    _RUN_FLY_TO_FIX().
  } ELSE IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK {
    _RUN_ILS_TRACK().
  }
}

// ─────────────────────────────────────────────────────────
// SUB-PHASE: FLY_TO_FIX
// Navigate toward each fix in the approach sequence.
// Descend to the target altitude associated with each fix.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLY_TO_FIX {

  // If all fixes have been passed, begin ILS tracking.
  IF FIX_INDEX >= ACTIVE_FIXES:LENGTH {
    SET_SUBPHASE(SUBPHASE_ILS_TRACK).
    SET PREV_LOC_DEV TO 0.
    SET PREV_GS_DEV  TO 0.
    RETURN.
  }

  LOCAL fix_id  IS ACTIVE_FIXES[FIX_INDEX].
  LOCAL fix     IS GET_BEACON(fix_id).
  LOCAL fix_ll  IS fix["ll"].

  // Target altitude for this leg.
  LOCAL tgt_alt IS SHIP:ALTITUDE.  // default: hold current
  IF ACTIVE_ALT_AT:HASKEY(fix_id) {
    SET tgt_alt TO ACTIVE_ALT_AT[fix_id].
  }

  // Bearing and distance to fix.
  LOCAL brg   IS GEO_BEARING(SHIP:GEOPOSITION, fix_ll).
  LOCAL dist  IS GEO_DISTANCE(SHIP:GEOPOSITION, fix_ll).

  // Proportional FPA based on altitude error.
  LOCAL alt_err IS SHIP:ALTITUDE - tgt_alt.
  LOCAL fpa_cmd IS CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).

  AA_SET_DIRECTOR(brg, fpa_cmd).

  // Autothrottle: hold Vapp during transit.
  SET THROTTLE_CMD TO CLAMP(KP_SPD * (ACTIVE_V_APP - GET_IAS()), MIN_APPROACH_THR, 1).
  LOCK THROTTLE TO THROTTLE_CMD.

  // Extend gear if aircraft config specifies a gear-down AGL.
  LOCAL gear_agl IS ACTIVE_AIRCRAFT["gear_down_agl"].
  IF gear_agl > 0 AND GET_AGL() < gear_agl {
    GEAR ON.
  }

  // Capture: move to the next fix when within capture radius.
  IF dist < FIX_CAPTURE_RADIUS {
    PRINT "  FIX captured: " + fix_id + "  (dist=" + ROUND(dist) + " m)".
    SET FIX_INDEX TO FIX_INDEX + 1.
    // If this was the last fix, immediately transition to ILS tracking.
    IF FIX_INDEX >= ACTIVE_FIXES:LENGTH {
      SET_SUBPHASE(SUBPHASE_ILS_TRACK).
      SET PREV_LOC_DEV TO 0.
      SET PREV_GS_DEV  TO 0.
    }
  }
}

// ─────────────────────────────────────────────────────────
// SUB-PHASE: ILS_TRACK
// Coupled localizer and glideslope tracking via AA Director.
// Transitions to PHASE_FLARE when AGL drops below FLARE_AGL_M.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_ILS_TRACK {

  // Gear must be down before final approach.
  GEAR ON.

  // Compute ILS deviations.
  LOCAL dev IS _COMPUTE_ILS_DEVIATIONS().
  LOCAL loc_m IS dev["loc"].
  LOCAL gs_m  IS dev["gs"].
  SET ILS_LOC_DEV TO loc_m.
  SET ILS_GS_DEV  TO gs_m.
  SET ILS_DIST_M  TO dev["dist"].

  // Derivative (rate of change over one loop cycle).
  LOCAL d_loc IS (loc_m - PREV_LOC_DEV) / IFC_LOOP_DT.
  LOCAL d_gs  IS (gs_m  - PREV_GS_DEV)  / IFC_LOOP_DT.
  SET PREV_LOC_DEV TO loc_m.
  SET PREV_GS_DEV  TO gs_m.

  // ── Lateral (localizer) ──
  // Positive loc_m = right of centerline → turn left (reduce heading).
  LOCAL loc_corr IS -(KP_LOC * loc_m + KD_LOC * d_loc).
  LOCAL hdg_cmd  IS WRAP_360(ACTIVE_RWY_HDG + CLAMP(loc_corr, -MAX_LOC_CORR, MAX_LOC_CORR)).

  // ── Vertical (glideslope) ──
  // Positive gs_m = above GS → pitch more negative (increase descent rate).
  LOCAL gs_corr IS -(KP_GS * gs_m + KD_GS * d_gs).
  LOCAL fpa_cmd IS -ACTIVE_GS_ANGLE + CLAMP(gs_corr, -MAX_GS_CORR_DN, MAX_GS_CORR_UP).

  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).

  // ── Autothrottle ──
  SET THROTTLE_CMD TO CLAMP(KP_SPD * (ACTIVE_V_APP - GET_IAS()), MIN_APPROACH_THR, 1).
  LOCK THROTTLE TO THROTTLE_CMD.

  // ── Check for flare trigger ──
  LOCAL flare_agl IS FLARE_AGL_M.
  IF ACTIVE_AIRCRAFT["flare_agl"] >= 0 { SET flare_agl TO ACTIVE_AIRCRAFT["flare_agl"]. }

  IF GET_AGL() < flare_agl {
    SET_PHASE(PHASE_FLARE).
    SET FLARE_PITCH_CMD TO GET_PITCH().  // start flare from current pitch
  }
}

// ─────────────────────────────────────────────────────────
// ILS deviation computation
// Returns a LEXICON: "loc" (m), "gs" (m), "dist" (m from thr)
// ─────────────────────────────────────────────────────────
FUNCTION _COMPUTE_ILS_DEVIATIONS {
  LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).

  // Threshold world-space position.
  LOCAL thr_pos IS ils["ll"]:ALTITUDEPOSITION(ils["alt"]).

  // Unit vectors along and perpendicular to the runway (horizontal).
  // HEADING(hdg, 0):FOREVECTOR gives the horizontal surface direction.
  LOCAL rwy_fwd   IS HEADING(ils["hdg"],      0):FOREVECTOR.
  LOCAL rwy_right IS HEADING(ils["hdg"] + 90, 0):FOREVECTOR.

  // Displacement from threshold to aircraft.
  LOCAL disp IS SHIP:POSITION - thr_pos.

  // Along-track component: negative = aircraft is still on approach (before threshold).
  LOCAL along_m IS VDOT(disp, rwy_fwd).
  LOCAL dist_m  IS -along_m.   // positive when on approach

  // Lateral component: positive = right of centerline.
  LOCAL loc_m IS VDOT(disp, rwy_right).

  // Nominal glideslope altitude at this distance from threshold.
  LOCAL gs_nom_alt IS ils["alt"] + dist_m * TAN(ils["gs_angle"]).
  LOCAL gs_m       IS SHIP:ALTITUDE - gs_nom_alt.  // positive = above GS

  RETURN LEXICON("loc", loc_m, "gs", gs_m, "dist", dist_m).
}
