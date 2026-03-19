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
  _CHECK_APPROACH_SPOILERS().
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
  LOCAL intercept_gain IS AC_PARAM("app_spd_intercept_gain",    APP_SPD_INTERCEPT_GAIN,    0).
  LOCAL intercept_min_add IS AC_PARAM("app_spd_intercept_min_add", APP_SPD_INTERCEPT_MIN_ADD, 0.001).
  LOCAL intercept_max_add IS AC_PARAM("app_spd_intercept_max_add", APP_SPD_INTERCEPT_MAX_ADD, 0.001).
  LOCAL short_final_agl IS AC_PARAM("app_short_final_agl",      APP_SHORT_FINAL_AGL_M,     0.001).
  LOCAL speed_tgt_slew_per_s IS AC_PARAM("app_speed_tgt_slew_per_s", APP_SPEED_TGT_SLEW_PER_S, 0.001).

  // app_short_final_cap is a boolean flag stored as 0/1 in aircraft config.
  LOCAL short_final_cap_when_not_final IS APP_SHORT_FINAL_CAP_WHEN_NOT_FINAL.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("app_short_final_cap") AND ACTIVE_AIRCRAFT["app_short_final_cap"] >= 0 {
    SET short_final_cap_when_not_final TO ACTIVE_AIRCRAFT["app_short_final_cap"] <> 0.
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
        SET IFC_ALERT_TEXT TO "APP SPD -> FINAL  tgt " + ROUND(vapp, 1) + " m/s".
        SET IFC_ALERT_UT   TO TIME:SECONDS.
      }
    } ELSE {
      SET APP_FINAL_ARM_UT TO -1.
      LOCAL loc_rel IS loc_cap * APP_FINAL_RELEASE_FACTOR.
      LOCAL gs_rel  IS gs_cap * APP_FINAL_RELEASE_FACTOR.
      IF APP_ON_FINAL AND (ABS(ILS_LOC_DEV) > loc_rel OR ABS(ILS_GS_DEV) > gs_rel) {
        SET APP_ON_FINAL TO FALSE.
        SET IFC_ALERT_TEXT TO "APP SPD -> INTERCEPT  tgt " + ROUND(vint, 1) + " m/s".
        SET IFC_ALERT_UT   TO TIME:SECONDS.
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

  // AoA protection: if FAR AoA is approaching a_crit, raise speed floor.
  LOCAL a_crit IS AC_PARAM("a_crit", 0, 0.001).
  IF FAR_AVAILABLE AND a_crit > 0 {
    LOCAL aoa_now  IS GET_AOA().
    LOCAL aoa_warn IS a_crit * APP_AOA_PROTECT_FRAC.
    IF aoa_now > aoa_warn {
      LOCAL aoa_excess IS aoa_now - aoa_warn.
      LOCAL aoa_floor  IS GET_IAS() + aoa_excess * APP_AOA_SPD_GAIN.
      IF aoa_floor > base_tgt { SET base_tgt TO aoa_floor. }
    }
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

  // As soon as localizer is captured in ILS_TRACK, drive speed to Vapp.
  // This avoids carrying intercept-speed additives down final.
  IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK AND ABS(ILS_LOC_DEV) <= LOC_CAPTURE_M {
    SET base_tgt TO MIN(base_tgt, vapp).
    IF APP_SPD_MODE = "INTERCEPT" { SET APP_SPD_MODE TO "LOC_CAPTURE". }
  }

  SET ACTIVE_V_TGT TO MOVE_TOWARD(
    ACTIVE_V_TGT,
    base_tgt,
    speed_tgt_slew_per_s * IFC_ACTUAL_DT
  ).
  RETURN ACTIVE_V_TGT.
}

// ─────────────────────────────────────────────────────────
// CASCADE AUTOTHROTTLE  (shared by FLY_TO_FIX and ILS_TRACK)
//
// Outer loop: speed error (m/s) → a_cmd (m/s²)
// Inner loop: (a_cmd - a_actual) → throttle delta
// Integral on speed error provides steady-state trim for drag / slope.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_APPROACH_THROTTLE {
  LOCAL v_tgt IS _UPDATE_APPROACH_SPEED_TARGET().
  AT_RUN_SPEED_HOLD(v_tgt, MIN_APPROACH_THR, 1).
}

// ─────────────────────────────────────────────────────────
// SUB-PHASE: FLY_TO_FIX
// Navigate toward each fix in the approach sequence.
// Descend to the target altitude associated with each fix.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLY_TO_FIX {
  // If we're already on the localizer inbound, hand off directly to ILS track
  // without waiting for the remaining fix sequence.
  IF ACTIVE_ILS_ID <> "" {
    LOCAL ils_dev IS _COMPUTE_ILS_DEVIATIONS().
    IF ils_dev:HASKEY("loc") AND ils_dev:HASKEY("dist")
        AND ils_dev["dist"] > 0
        AND ABS(ils_dev["loc"]) <= LOC_CAPTURE_M {
      SET ILS_INTERCEPT_ALT TO SHIP:ALTITUDE.
      SET_SUBPHASE(SUBPHASE_ILS_TRACK).
      SET PREV_LOC_DEV TO 0.
      SET PREV_GS_DEV  TO 0.
      RETURN.
    }
  }

  // If all fixes have been passed, begin ILS tracking.
  IF FIX_INDEX >= ACTIVE_FIXES:LENGTH {
    SET ILS_INTERCEPT_ALT TO SHIP:ALTITUDE.
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

  // Heading error to the fix (0-180).
  LOCAL hdg_err IS WRAP_360(brg - SHIP:HEADING).
  IF hdg_err > 180 { SET hdg_err TO 360 - hdg_err. }

  // Proportional FPA based on altitude error.
  // Suppress descent during large heading changes to prevent spiral dives.
  // Climbing is still allowed (fix may be above current altitude).
  LOCAL alt_err IS SHIP:ALTITUDE - tgt_alt.
  LOCAL fpa_cmd IS CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).
  IF hdg_err > 45 AND fpa_cmd < 0 { SET fpa_cmd TO 0. }

  AA_SET_DIRECTOR(brg, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO brg.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _RUN_APPROACH_THROTTLE().

  // Extend gear if aircraft config specifies a gear-down AGL.
  LOCAL gear_agl IS AC_PARAM("gear_down_agl", 0, 0.001).
  IF gear_agl > 0 AND GET_AGL() < gear_agl {
    GEAR ON.
  }

  // Capture: move to the next fix when within capture radius.
  IF dist < FIX_CAPTURE_RADIUS {
    SET IFC_ALERT_TEXT TO "FIX captured: " + fix_id + "  (" + ROUND(dist) + " m)".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
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
// Transitions to PHASE_FLARE when runway-relative height drops below FLARE_AGL_M.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_ILS_TRACK {
  // Compute ILS deviations.
  LOCAL dev IS _COMPUTE_ILS_DEVIATIONS().
  LOCAL loc_m IS dev["loc"].
  LOCAL gs_m  IS dev["gs"].
  SET ILS_LOC_DEV TO loc_m.
  SET ILS_GS_DEV  TO gs_m.
  SET ILS_DIST_M  TO dev["dist"].
  LOCAL gs_captured IS ABS(gs_m) <= GS_CAPTURE_M.

  // Enter full approach configuration on GS capture.
  // Before GS capture, keep gear-up unless explicit AGL rule asks for gear.
  LOCAL gear_agl IS AC_PARAM("gear_down_agl", 0, 0.001).
  IF gs_captured OR (gear_agl > 0 AND GET_RUNWAY_REL_HEIGHT() < gear_agl) {
    GEAR ON.
  }

  // Derivative (rate of change over one loop cycle).
  LOCAL d_loc IS (loc_m - PREV_LOC_DEV) / IFC_ACTUAL_DT.
  LOCAL d_gs  IS (gs_m  - PREV_GS_DEV)  / IFC_ACTUAL_DT.
  SET PREV_LOC_DEV TO loc_m.
  SET PREV_GS_DEV  TO gs_m.

  // ── Lateral (localizer) ──
  // Positive loc_m = right of centerline → turn left (reduce heading).
  LOCAL loc_corr IS -(KP_LOC * loc_m + KD_LOC * d_loc).

  // Bank angle limiter: AA has no native bank limit, so we implement it here.
  // Fade loc_corr to zero as bank approaches limit; add counter-correction if over.
  LOCAL max_bank IS AC_PARAM("aa_max_bank", AA_MAX_BANK, 0.001).
  LOCAL bank IS 90 - VECTORANGLE(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR).
  LOCAL bank_abs IS ABS(bank).
  LOCAL bank_fade_start IS max_bank * 0.70.
  LOCAL bank_fade_span  IS MAX(max_bank - bank_fade_start, 0.1).
  SET loc_corr TO loc_corr * CLAMP((max_bank - bank_abs) / bank_fade_span, 0, 1).
  IF bank_abs > max_bank {
    LOCAL bank_push IS (bank_abs - max_bank) * KP_BANK_LIMIT.
    IF bank > 0 { SET loc_corr TO loc_corr - bank_push. }
    ELSE        { SET loc_corr TO loc_corr + bank_push. }
  }

  LOCAL hdg_cmd  IS WRAP_360(ACTIVE_RWY_HDG + CLAMP(loc_corr, -MAX_LOC_CORR, MAX_LOC_CORR)).

  // ── Vertical (glideslope) ──
  // Before GS capture: hold current altitude and let the glideslope descend
  // to intercept the aircraft from above (always intercept from below).
  // This prevents an aggressive pitch-down if the aircraft arrives slightly
  // above the GS on LOC capture.
  // After GS capture: track the glideslope with PD feedback.
  LOCAL fpa_cmd IS 0.
  LOCAL gs_corr IS 0.
  IF gs_captured {
    SET gs_corr TO -(KP_GS * gs_m + KD_GS * d_gs).
    SET fpa_cmd TO -ACTIVE_GS_ANGLE + CLAMP(gs_corr, -MAX_GS_CORR_DN, MAX_GS_CORR_UP).
  } ELSE {
    // Pre-capture: hold the altitude at which LOC was captured.
    // Let the glideslope descend to the aircraft (always intercept from below).
    // Prevents aggressive pitch-down if aircraft is slightly above the GS on LOC capture.
    LOCAL alt_err IS SHIP:ALTITUDE - ILS_INTERCEPT_ALT.
    SET fpa_cmd TO CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).
  }

  AA_SET_DIRECTOR(hdg_cmd, fpa_cmd).
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO loc_corr.
  SET TELEM_GS_CORR    TO gs_corr.

  _RUN_APPROACH_THROTTLE().

  // ── Check for flare trigger (with hysteresis + descent gate + debounce) ──
  LOCAL flare_agl IS AC_PARAM("flare_agl", FLARE_AGL_M, 0).
  LOCAL flare_arm_agl IS MAX(flare_agl - FLARE_TRIGGER_HYST_M, 0.5).
  // Require descent (or near-level, if tuned) before arming flare.
  // Default 0 => flare only while VS is not climbing.
  LOCAL flare_trigger_max_vs IS AC_PARAM("flare_trigger_max_vs", 0, 0).
  LOCAL flare_h_now IS GET_RUNWAY_REL_HEIGHT().
  LOCAL vs_now IS SHIP:VERTICALSPEED.

  IF flare_h_now < flare_arm_agl {
    IF vs_now <= flare_trigger_max_vs {
      IF FLARE_TRIGGER_START_UT < 0 { SET FLARE_TRIGGER_START_UT TO TIME:SECONDS. }
      IF TIME:SECONDS - FLARE_TRIGGER_START_UT >= FLARE_TRIGGER_CONFIRM_S {
        LOCAL entry_ias IS MAX(GET_IAS(), 10).
        SET FLARE_PITCH_CMD TO ARCTAN(vs_now / entry_ias). // seed from current flight path angle
        SET FLARE_ENTRY_VS  TO CLAMP(vs_now, FLARE_MIN_ENTRY_SINK_VS, -0.05). // seed to a shallow descending band
        // Capture runway-relative flare-entry height (floor 1 to avoid /0).
        SET FLARE_ENTRY_AGL TO MAX(flare_h_now, 1).
        SET FLARE_TRIGGER_START_UT TO -1.
        SET TOUCHDOWN_CANDIDATE_UT TO -1.
        SET_PHASE(PHASE_FLARE).
      }
    } ELSE {
      // Below flare AGL but still climbing: do not allow flare trigger timer to accumulate.
      SET FLARE_TRIGGER_START_UT TO -1.
    }
  } ELSE IF flare_h_now > flare_agl {
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

  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).

  LOCAL det_up   IS ROUND(AC_PARAM("flaps_detent_up",       0, 0)).
  LOCAL det_clmb IS ROUND(AC_PARAM("flaps_detent_climb",    1, 0)).
  LOCAL det_app  IS ROUND(AC_PARAM("flaps_detent_approach", 2, 0)).
  LOCAL det_land IS ROUND(AC_PARAM("flaps_detent_landing",  3, 0)).

  SET det_up   TO CLAMP(det_up,   0, max_det).
  SET det_clmb TO CLAMP(det_clmb, 0, max_det).
  SET det_app  TO CLAMP(det_app,  0, max_det).
  SET det_land TO CLAMP(det_land, 0, max_det).

  LOCAL climb_km IS AC_PARAM("flaps_climb_km",    45, 0.001).
  LOCAL app_km   IS AC_PARAM("flaps_approach_km", 30, 0.001).
  LOCAL land_km  IS AC_PARAM("flaps_landing_km",   8, 0.001).

  LOCAL vfe_clmb IS AC_PARAM("vfe_climb",    160, 0.001).
  LOCAL vfe_app  IS AC_PARAM("vfe_approach", 120, 0.001).
  LOCAL vfe_land IS AC_PARAM("vfe_landing",   95, 0.001).

  // Desired detent from range to threshold (far -> near).
  LOCAL desired_det IS det_up.
  IF dist_km < climb_km { SET desired_det TO det_clmb. }
  IF dist_km < app_km   { SET desired_det TO det_app. }
  IF dist_km < land_km  { SET desired_det TO det_land. }

  // On ILS glideslope capture, force at least approach detent immediately.
  IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK AND ABS(ILS_GS_DEV) <= GS_CAPTURE_M {
    SET desired_det TO MAX(desired_det, det_app).
  }

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
    SET IFC_ALERT_TEXT TO "FLAPS tgt detent " + FLAPS_TARGET_DETENT
        + "  IAS " + ROUND(ias, 1) + " D " + ROUND(dist_km, 1) + "km".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET FLAPS_LAST_TARGET_LOGGED TO FLAPS_TARGET_DETENT.
  }

  IF FLAPS_CURRENT_DETENT = FLAPS_TARGET_DETENT { RETURN. }
  IF TIME:SECONDS - FLAPS_LAST_STEP_UT < FLAP_STEP_INTERVAL { RETURN. }

  IF FLAPS_CURRENT_DETENT < FLAPS_TARGET_DETENT {
    PULSE_AG(ag_step_up).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
    SET IFC_ALERT_TEXT TO "FLAPS UP -> detent " + FLAPS_CURRENT_DETENT.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  } ELSE {
    PULSE_AG(ag_step_dn).
    SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
    SET IFC_ALERT_TEXT TO "FLAPS DN -> detent " + FLAPS_CURRENT_DETENT.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  }

  SET FLAPS_LAST_STEP_UT TO TIME:SECONDS.
}

// ─────────────────────────────────────────────────────────
// APPROACH SPOILER ARM
// Triggers ag_spoilers_arm once when within app_spoiler_arm_km.
// Ground deployment on touchdown is handled separately by phase_autoland.
// ─────────────────────────────────────────────────────────
FUNCTION _CHECK_APPROACH_SPOILERS {
  IF APP_SPOILERS_ARMED { RETURN. }
  LOCAL arm_ag IS AC_PARAM("ag_spoilers_arm", 0, 0.001).
  LOCAL arm_km IS AC_PARAM("app_spoiler_arm_km", 0, 0.001).
  IF arm_ag <= 0 OR arm_km <= 0 { RETURN. }

  LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).
  IF NOT ils:HASKEY("ll") { RETURN. }
  LOCAL dist_km IS GEO_DISTANCE(SHIP:GEOPOSITION, ils["ll"]) / 1000.
  IF dist_km <= arm_km {
    TRIGGER_AG(arm_ag, TRUE).
    SET APP_SPOILERS_ARMED TO TRUE.
    SET IFC_ALERT_TEXT TO "SPOILERS armed at " + ROUND(dist_km, 1) + " km".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  }
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
