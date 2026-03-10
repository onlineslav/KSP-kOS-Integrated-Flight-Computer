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
  _CHECK_FLAP_DEPLOYMENT().
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

  // Autothrottle PI: hold Vapp.
  LOCAL spd_err IS ACTIVE_V_APP - GET_IAS().
  SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * IFC_LOOP_DT, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  SET THROTTLE_CMD TO CLAMP(KP_SPD * spd_err + KI_SPD * THR_INTEGRAL, MIN_APPROACH_THR, 1).

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

  // ── Autothrottle PI: hold Vapp ──
  LOCAL spd_err IS ACTIVE_V_APP - GET_IAS().
  SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * IFC_LOOP_DT, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  SET THROTTLE_CMD TO CLAMP(KP_SPD * spd_err + KI_SPD * THR_INTEGRAL, MIN_APPROACH_THR, 1).

  // ── Check for flare trigger ──
  LOCAL flare_agl IS FLARE_AGL_M.
  IF ACTIVE_AIRCRAFT["flare_agl"] >= 0 { SET flare_agl TO ACTIVE_AIRCRAFT["flare_agl"]. }

  IF GET_AGL() < flare_agl {
    SET FLARE_PITCH_CMD TO GET_PITCH().          // seed FPA from current attitude
    SET FLARE_ENTRY_VS  TO SHIP:VERTICALSPEED.   // capture sink rate at entry
    SET FLARE_ENTRY_AGL TO MAX(GET_AGL(), 1).    // capture AGL (floor 1 to avoid /0)
    SET_PHASE(PHASE_FLARE).
  }
}

// ─────────────────────────────────────────────────────────
// FLAP DETENT MANAGER
// FAR-style stepped flaps:
// - computes distance-based desired detent
// - limits by IAS/Vfe safety detent
// - steps one notch at a time using action-group pulses
// Runs in both FLY_TO_FIX and ILS_TRACK sub-phases.
// ─────────────────────────────────────────────────────────
FUNCTION _CHECK_FLAP_DEPLOYMENT {
  LOCAL ac IS ACTIVE_AIRCRAFT.
  IF ac = 0 { RETURN. }
  IF NOT ac:HASKEY("ag_flaps_step_up") OR NOT ac:HASKEY("ag_flaps_step_down") { RETURN. }

  LOCAL ag_step_up IS ac["ag_flaps_step_up"].
  LOCAL ag_step_dn IS ac["ag_flaps_step_down"].
  IF ag_step_up <= 0 OR ag_step_dn <= 0 { RETURN. }

  LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).
  IF NOT ils:HASKEY("ll") { RETURN. }

  LOCAL dist_km IS GEO_DISTANCE(SHIP:GEOPOSITION, ils["ll"]) / 1000.
  LOCAL ias     IS GET_IAS().

  LOCAL max_det IS 3.
  IF ac:HASKEY("flaps_max_detent") {
    SET max_det TO MAX(0, ROUND(ac["flaps_max_detent"], 0)).
  }

  LOCAL det_up   IS 0.
  LOCAL det_clmb IS 1.
  LOCAL det_app  IS 2.
  LOCAL det_land IS 3.
  IF ac:HASKEY("flaps_detent_up")       { SET det_up   TO ROUND(ac["flaps_detent_up"], 0). }
  IF ac:HASKEY("flaps_detent_climb")    { SET det_clmb TO ROUND(ac["flaps_detent_climb"], 0). }
  IF ac:HASKEY("flaps_detent_approach") { SET det_app  TO ROUND(ac["flaps_detent_approach"], 0). }
  IF ac:HASKEY("flaps_detent_landing")  { SET det_land TO ROUND(ac["flaps_detent_landing"], 0). }

  SET det_up   TO CLAMP(det_up,   0, max_det).
  SET det_clmb TO CLAMP(det_clmb, 0, max_det).
  SET det_app  TO CLAMP(det_app,  0, max_det).
  SET det_land TO CLAMP(det_land, 0, max_det).

  LOCAL climb_km IS 45.
  LOCAL app_km   IS 30.
  LOCAL land_km  IS 8.
  IF ac:HASKEY("flaps_climb_km")    { SET climb_km TO ac["flaps_climb_km"]. }
  IF ac:HASKEY("flaps_approach_km") { SET app_km   TO ac["flaps_approach_km"]. }
  IF ac:HASKEY("flaps_landing_km")  { SET land_km  TO ac["flaps_landing_km"]. }

  LOCAL vfe_clmb IS 160.
  LOCAL vfe_app  IS 120.
  LOCAL vfe_land IS 95.
  IF ac:HASKEY("vfe_climb")    { SET vfe_clmb TO ac["vfe_climb"]. }
  IF ac:HASKEY("vfe_approach") { SET vfe_app  TO ac["vfe_approach"]. }
  IF ac:HASKEY("vfe_landing")  { SET vfe_land TO ac["vfe_landing"]. }

  // Desired detent from range to threshold (far -> near).
  LOCAL desired_det IS det_up.
  IF dist_km < climb_km { SET desired_det TO det_clmb. }
  IF dist_km < app_km   { SET desired_det TO det_app. }
  IF dist_km < land_km  { SET desired_det TO det_land. }

  // Max detent allowed for extension at current IAS.
  LOCAL speed_extend_det IS det_up.
  IF ias <= vfe_clmb { SET speed_extend_det TO det_clmb. }
  IF ias <= vfe_app  { SET speed_extend_det TO det_app. }
  IF ias <= vfe_land { SET speed_extend_det TO det_land. }

  // Slightly larger limit for "hold" to reduce step chatter near Vfe.
  LOCAL speed_hold_det IS det_up.
  IF ias <= (vfe_clmb + FLAP_VFE_HYST) { SET speed_hold_det TO det_clmb. }
  IF ias <= (vfe_app  + FLAP_VFE_HYST) { SET speed_hold_det TO det_app. }
  IF ias <= (vfe_land + FLAP_VFE_HYST) { SET speed_hold_det TO det_land. }

  LOCAL target_det IS MIN(desired_det, speed_extend_det).
  IF FLAPS_CURRENT_DETENT > speed_hold_det {
    SET target_det TO speed_hold_det.
  }
  SET target_det TO CLAMP(target_det, det_up, max_det).
  SET FLAPS_TARGET_DETENT TO target_det.

  IF FLAPS_TARGET_DETENT <> FLAPS_LAST_TARGET_LOGGED {
    PRINT "  FLAPS target detent " + FLAPS_TARGET_DETENT
        + " (IAS " + ROUND(ias, 1) + " m/s, D " + ROUND(dist_km, 1) + " km)".
    SET FLAPS_LAST_TARGET_LOGGED TO FLAPS_TARGET_DETENT.
  }

  IF FLAPS_CURRENT_DETENT = FLAPS_TARGET_DETENT { RETURN. }
  IF TIME:SECONDS - FLAPS_LAST_STEP_UT < FLAP_STEP_INTERVAL { RETURN. }

  IF FLAPS_CURRENT_DETENT < FLAPS_TARGET_DETENT {
    PULSE_AG(ag_step_up).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
    PRINT "  FLAPS step UP -> detent " + FLAPS_CURRENT_DETENT.
  } ELSE {
    PULSE_AG(ag_step_dn).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
    PRINT "  FLAPS step DN -> detent " + FLAPS_CURRENT_DETENT.
  }

  SET FLAPS_LAST_STEP_UT TO TIME:SECONDS.
}

// ─────────────────────────────────────────────────────────
// ILS deviation computation
// Returns a LEXICON: "loc" (m), "gs" (m), "dist" (m from thr)
// ─────────────────────────────────────────────────────────
FUNCTION _COMPUTE_ILS_DEVIATIONS {
  LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).

  // Threshold world-space position.
  LOCAL thr_pos IS ils["ll"]:ALTITUDEPOSITION(ils["alt_asl"]).

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
  LOCAL gs_nom_alt IS ils["alt_asl"] + dist_m * TAN(ils["gs_angle"]).
  LOCAL gs_m       IS SHIP:ALTITUDE - gs_nom_alt.  // positive = above GS

  RETURN LEXICON("loc", loc_m, "gs", gs_m, "dist", dist_m).
}
