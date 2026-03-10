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
// APPROACH SPEED SCHEDULER
// Keeps the number of aircraft-specific knobs low by deriving
// intercept speed from existing Vapp/Vref values.
//
// Speed modes:
// - INTERCEPT: before stable LOC/GS capture, target derived Vint.
// - FINAL: once LOC/GS capture is stable, target Vapp.
// - SHORT FINAL: blend Vapp -> Vref below APP_SHORT_FINAL_AGL_M.
// ─────────────────────────────────────────────────────────
FUNCTION _GET_VREF_TARGET {
  LOCAL vref IS ACTIVE_V_APP - 10.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("v_ref") AND ACTIVE_AIRCRAFT["v_ref"] > 0 {
    SET vref TO ACTIVE_AIRCRAFT["v_ref"].
  }
  RETURN CLAMP(vref, 1, ACTIVE_V_APP).
}

FUNCTION _GET_VINTERCEPT_TARGET {
  PARAMETER vapp, vref, gain, add_min, add_max.
  LOCAL intercept_add IS (vapp - vref) * gain.
  SET intercept_add TO CLAMP(intercept_add, add_min, add_max).
  RETURN vapp + intercept_add.
}

FUNCTION _UPDATE_APPROACH_SPEED_TARGET {
  LOCAL vapp IS ACTIVE_V_APP.
  LOCAL vref IS _GET_VREF_TARGET().
  LOCAL intercept_gain IS APP_SPD_INTERCEPT_GAIN.
  LOCAL intercept_min_add IS APP_SPD_INTERCEPT_MIN_ADD.
  LOCAL intercept_max_add IS APP_SPD_INTERCEPT_MAX_ADD.
  LOCAL short_final_agl IS APP_SHORT_FINAL_AGL_M.
  LOCAL speed_tgt_slew_per_s IS APP_SPEED_TGT_SLEW_PER_S.
  LOCAL short_final_cap_when_not_final IS APP_SHORT_FINAL_CAP_WHEN_NOT_FINAL.

  // Optional per-aircraft overrides (keep minimal; defaults are robust).
  IF ACTIVE_AIRCRAFT <> 0 {
    IF ACTIVE_AIRCRAFT:HASKEY("app_spd_intercept_gain") AND ACTIVE_AIRCRAFT["app_spd_intercept_gain"] >= 0 {
      SET intercept_gain TO ACTIVE_AIRCRAFT["app_spd_intercept_gain"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("app_spd_intercept_min_add") AND ACTIVE_AIRCRAFT["app_spd_intercept_min_add"] > 0 {
      SET intercept_min_add TO ACTIVE_AIRCRAFT["app_spd_intercept_min_add"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("app_spd_intercept_max_add") AND ACTIVE_AIRCRAFT["app_spd_intercept_max_add"] > 0 {
      SET intercept_max_add TO ACTIVE_AIRCRAFT["app_spd_intercept_max_add"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("app_short_final_agl") AND ACTIVE_AIRCRAFT["app_short_final_agl"] > 0 {
      SET short_final_agl TO ACTIVE_AIRCRAFT["app_short_final_agl"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("app_speed_tgt_slew_per_s") AND ACTIVE_AIRCRAFT["app_speed_tgt_slew_per_s"] > 0 {
      SET speed_tgt_slew_per_s TO ACTIVE_AIRCRAFT["app_speed_tgt_slew_per_s"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("app_short_final_cap") AND ACTIVE_AIRCRAFT["app_short_final_cap"] >= 0 {
      SET short_final_cap_when_not_final TO ACTIVE_AIRCRAFT["app_short_final_cap"] <> 0.
    }
  }

  IF intercept_max_add < intercept_min_add { SET intercept_max_add TO intercept_min_add. }
  LOCAL vint IS _GET_VINTERCEPT_TARGET(vapp, vref, intercept_gain, intercept_min_add, intercept_max_add).
  LOCAL base_tgt IS vint.
  LOCAL short_final_frac IS 0.
  LOCAL short_final_target IS vapp.
  LOCAL short_cap_applied IS FALSE.
  LOCAL agl IS GET_AGL().
  IF short_final_agl > 0 AND agl < short_final_agl {
    SET short_final_frac TO CLAMP((short_final_agl - agl) / short_final_agl, 0, 1).
    SET short_final_target TO vapp + (vref - vapp) * short_final_frac.
  }

  // Final-speed mode is only available once ILS tracking is active.
  IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK {
    LOCAL loc_cap IS LOC_CAPTURE_M.
    LOCAL gs_cap IS GS_CAPTURE_M.
    LOCAL in_capture IS ABS(ILS_LOC_DEV) <= loc_cap AND ABS(ILS_GS_DEV) <= gs_cap.
    SET APP_LOC_CAP_OK TO 0.
    SET APP_GS_CAP_OK TO 0.
    IF ABS(ILS_LOC_DEV) <= loc_cap { SET APP_LOC_CAP_OK TO 1. }
    IF ABS(ILS_GS_DEV) <= gs_cap { SET APP_GS_CAP_OK TO 1. }

    IF in_capture {
      IF APP_FINAL_ARM_UT < 0 { SET APP_FINAL_ARM_UT TO TIME:SECONDS. }
      IF NOT APP_ON_FINAL AND TIME:SECONDS - APP_FINAL_ARM_UT >= APP_FINAL_CAPTURE_CONFIRM_S {
        SET APP_ON_FINAL TO TRUE.
        PRINT "  APP SPD mode -> FINAL  tgt " + ROUND(vapp, 1) + " m/s".
      }
    } ELSE {
      SET APP_FINAL_ARM_UT TO -1.
      LOCAL loc_rel IS loc_cap * APP_FINAL_RELEASE_FACTOR.
      LOCAL gs_rel  IS gs_cap * APP_FINAL_RELEASE_FACTOR.
      IF APP_ON_FINAL AND (ABS(ILS_LOC_DEV) > loc_rel OR ABS(ILS_GS_DEV) > gs_rel) {
        SET APP_ON_FINAL TO FALSE.
        PRINT "  APP SPD mode -> INTERCEPT  tgt " + ROUND(vint, 1) + " m/s".
      }
    }
  } ELSE {
    SET APP_FINAL_ARM_UT TO -1.
    SET APP_ON_FINAL TO FALSE.
    SET APP_LOC_CAP_OK TO 0.
    SET APP_GS_CAP_OK TO 0.
  }

  IF APP_ON_FINAL {
    SET base_tgt TO vapp.

    // On short final, bleed from Vapp toward Vref automatically.
    IF short_final_frac > 0 { SET base_tgt TO short_final_target. }
  } ELSE IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK AND short_final_cap_when_not_final AND short_final_frac > 0 {
    // Safety cap: near the runway, never carry intercept speeds.
    SET base_tgt TO MIN(base_tgt, short_final_target).
    SET short_cap_applied TO TRUE.
  }

  // Export scheduler internals for terminal/log diagnostics.
  SET APP_VREF_TGT TO vref.
  SET APP_VINT_TGT TO vint.
  SET APP_BASE_V_TGT TO base_tgt.
  SET APP_SHORT_FINAL_FRAC TO short_final_frac.
  IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX {
    SET APP_SPD_MODE TO "FIXES".
  } ELSE IF APP_ON_FINAL {
    IF short_final_frac > 0 {
      SET APP_SPD_MODE TO "SHORT_FINAL".
    } ELSE {
      SET APP_SPD_MODE TO "FINAL".
    }
  } ELSE IF short_cap_applied {
    SET APP_SPD_MODE TO "SHORT_CAP".
  } ELSE {
    SET APP_SPD_MODE TO "INTERCEPT".
  }

  SET ACTIVE_V_TGT TO MOVE_TOWARD(
    ACTIVE_V_TGT,
    base_tgt,
    speed_tgt_slew_per_s * IFC_ACTUAL_DT
  ).
  RETURN ACTIVE_V_TGT.
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
  SET TELEM_AA_HDG_CMD TO brg.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  // Cascade autothrottle: speed outer loop + acceleration inner loop.
  // Outer: speed error → a_cmd.  Inner: (a_cmd - a_actual) → throttle.
  // Integral on speed error provides steady-state trim (drag / slope).
  LOCAL v_tgt IS _UPDATE_APPROACH_SPEED_TARGET().
  LOCAL ias IS GET_IAS().
  LOCAL spd_err IS v_tgt - ias.
  LOCAL a_cmd IS CLAMP(KP_SPD_ACL * spd_err, -ACL_MAX, ACL_MAX).
  LOCAL a_raw IS CLAMP((ias - PREV_IAS) / IFC_ACTUAL_DT, -10, 10).
  SET PREV_IAS      TO ias.
  SET A_ACTUAL_FILT TO A_ACTUAL_FILT * ACL_FILTER_ALPHA + a_raw * (1 - ACL_FILTER_ALPHA).
  LOCAL a_err IS a_cmd - A_ACTUAL_FILT.
  SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * IFC_ACTUAL_DT, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  LOCAL raw_thr IS CLAMP(KP_ACL_THR * a_err + KI_SPD * THR_INTEGRAL, MIN_APPROACH_THR, 1).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).

  // Extend gear if aircraft config specifies a gear-down AGL.
  LOCAL gear_agl IS 0.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("gear_down_agl") {
    SET gear_agl TO ACTIVE_AIRCRAFT["gear_down_agl"].
  }
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
  LOCAL d_loc IS (loc_m - PREV_LOC_DEV) / IFC_ACTUAL_DT.
  LOCAL d_gs  IS (gs_m  - PREV_GS_DEV)  / IFC_ACTUAL_DT.
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
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO loc_corr.
  SET TELEM_GS_CORR    TO gs_corr.

  // ── Cascade autothrottle: speed outer loop + acceleration inner loop ──
  LOCAL v_tgt IS _UPDATE_APPROACH_SPEED_TARGET().
  LOCAL ias IS GET_IAS().
  LOCAL spd_err IS v_tgt - ias.
  LOCAL a_cmd IS CLAMP(KP_SPD_ACL * spd_err, -ACL_MAX, ACL_MAX).
  LOCAL a_raw IS CLAMP((ias - PREV_IAS) / IFC_ACTUAL_DT, -10, 10).
  SET PREV_IAS      TO ias.
  SET A_ACTUAL_FILT TO A_ACTUAL_FILT * ACL_FILTER_ALPHA + a_raw * (1 - ACL_FILTER_ALPHA).
  LOCAL a_err IS a_cmd - A_ACTUAL_FILT.
  SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * IFC_ACTUAL_DT, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  LOCAL raw_thr IS CLAMP(KP_ACL_THR * a_err + KI_SPD * THR_INTEGRAL, MIN_APPROACH_THR, 1).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, THR_SLEW_PER_S * IFC_ACTUAL_DT).

  // ── Check for flare trigger (with hysteresis + debounce) ──
  LOCAL flare_agl IS FLARE_AGL_M.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flare_agl") AND ACTIVE_AIRCRAFT["flare_agl"] >= 0 {
    SET flare_agl TO ACTIVE_AIRCRAFT["flare_agl"].
  }
  LOCAL flare_arm_agl IS MAX(flare_agl - FLARE_TRIGGER_HYST_M, 0.5).
  LOCAL agl_now IS GET_AGL().

  IF agl_now < flare_arm_agl {
    IF FLARE_TRIGGER_START_UT < 0 { SET FLARE_TRIGGER_START_UT TO TIME:SECONDS. }
    IF TIME:SECONDS - FLARE_TRIGGER_START_UT >= FLARE_TRIGGER_CONFIRM_S {
      LOCAL entry_ias IS MAX(GET_IAS(), 10).
      SET FLARE_PITCH_CMD TO ARCTAN(SHIP:VERTICALSPEED / entry_ias). // seed from current flight path angle
      SET FLARE_ENTRY_VS  TO CLAMP(SHIP:VERTICALSPEED, FLARE_MIN_ENTRY_SINK_VS, -0.05). // seed to a shallow descending band
      SET FLARE_ENTRY_AGL TO MAX(agl_now, 1).    // capture AGL (floor 1 to avoid /0)
      SET FLARE_TRIGGER_START_UT TO -1.
      SET TOUCHDOWN_CANDIDATE_UT TO -1.
      SET_PHASE(PHASE_FLARE).
    }
  } ELSE IF agl_now > flare_agl {
    // Fully reset trigger when we climb back above the flare threshold.
    SET FLARE_TRIGGER_START_UT TO -1.
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
