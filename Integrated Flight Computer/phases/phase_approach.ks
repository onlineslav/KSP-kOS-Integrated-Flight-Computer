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
FUNCTION _APP_GS_VIS_CLEAR {
  IF IFC_DEBUG_GS_DRAW_MAIN <> 0 {
    SET IFC_DEBUG_GS_DRAW_MAIN:SHOW TO FALSE.
    SET IFC_DEBUG_GS_DRAW_MAIN TO 0.
  }
  IF IFC_DEBUG_GS_DRAW_REF <> 0 {
    SET IFC_DEBUG_GS_DRAW_REF:SHOW TO FALSE.
    SET IFC_DEBUG_GS_DRAW_REF TO 0.
  }
  IF IFC_DEBUG_GS_DRAW_TUBE:LENGTH > 0 {
    LOCAL i IS 0.
    UNTIL i >= IFC_DEBUG_GS_DRAW_TUBE:LENGTH {
      LOCAL draw_h IS IFC_DEBUG_GS_DRAW_TUBE[i].
      IF draw_h <> 0 { SET draw_h:SHOW TO FALSE. }
      SET i TO i + 1.
    }
    SET IFC_DEBUG_GS_DRAW_TUBE TO LIST().
  }
  SET IFC_DEBUG_GS_DRAW_ILS_ID TO "".
}

FUNCTION _APP_GS_VIS_SYNC {
  IF NOT IFC_DEBUG_DRAW_GS {
    IF IFC_DEBUG_GS_DRAW_MAIN <> 0 OR IFC_DEBUG_GS_DRAW_REF <> 0 OR IFC_DEBUG_GS_DRAW_TUBE:LENGTH > 0 {
      _APP_GS_VIS_CLEAR().
    }
    RETURN.
  }

  IF ACTIVE_ILS_ID = "" { RETURN. }
  LOCAL ils_bcn IS GET_BEACON(ACTIVE_ILS_ID).
  IF NOT ils_bcn:HASKEY("ll") { RETURN. }

  IF IFC_DEBUG_GS_DRAW_ILS_ID <> ACTIVE_ILS_ID OR IFC_DEBUG_GS_DRAW_MAIN = 0 {
    _APP_GS_VIS_CLEAR().
    SET IFC_DEBUG_GS_DRAW_MAIN TO VECDRAW(
      V(0, 0, 0),
      V(0, 0, 0),
      RGBA(1, 1, 0, 0.60),
      "",
      1,
      TRUE,
      0.2
    ).
    SET IFC_DEBUG_GS_DRAW_ILS_ID TO ACTIVE_ILS_ID.
  }

  LOCAL thr_ll IS ils_bcn["ll"].
  LOCAL thr_alt IS ils_bcn["alt_asl"].
  LOCAL gs_hdg IS MOD(ils_bcn["hdg"] + 180, 360).
  LOCAL gs_ang IS ils_bcn["gs_angle"].
  LOCAL draw_len_m IS MAX(IFC_DEBUG_GS_DRAW_LEN_M, 200).
  LOCAL draw_ahead_m IS MAX(IFC_DEBUG_GS_DRAW_LOCAL_AHEAD_M, 1).
  LOCAL draw_behind_m IS MAX(IFC_DEBUG_GS_DRAW_LOCAL_BEHIND_M, 1).
  // Keep debug beam width sane regardless of accidental config extremes.
  LOCAL draw_width_m IS CLAMP(IFC_DEBUG_GS_DRAW_WIDTH_M, 0.01, 0.5).
  LOCAL ship_ll IS SHIP:GEOPOSITION.
  LOCAL ship_dist_from_thr IS GEO_DISTANCE(thr_ll, ship_ll).
  LOCAL ship_brg_from_thr IS GEO_BEARING(thr_ll, ship_ll).
  LOCAL brg_err IS WRAP_180(ship_brg_from_thr - gs_hdg).
  LOCAL ship_ax IS ship_dist_from_thr * COS(brg_err).
  SET ship_ax TO CLAMP(ship_ax, 0, draw_len_m).
  LOCAL seg_start_ax IS CLAMP(ship_ax - draw_behind_m, 0, draw_len_m).
  LOCAL seg_end_ax IS CLAMP(ship_ax + draw_ahead_m, 0, draw_len_m).
  IF seg_end_ax <= seg_start_ax {
    SET seg_end_ax TO MIN(seg_start_ax + 1, draw_len_m).
  }
  LOCAL seg_start_ll IS GEO_DESTINATION(thr_ll, gs_hdg, seg_start_ax).
  LOCAL seg_end_ll IS GEO_DESTINATION(thr_ll, gs_hdg, seg_end_ax).
  LOCAL seg_start_alt IS thr_alt + seg_start_ax * TAN(gs_ang).
  LOCAL seg_end_alt IS thr_alt + seg_end_ax * TAN(gs_ang).
  LOCAL seg_start_shipraw IS seg_start_ll:ALTITUDEPOSITION(seg_start_alt).
  LOCAL seg_end_shipraw IS seg_end_ll:ALTITUDEPOSITION(seg_end_alt).

  IF IFC_DEBUG_GS_DRAW_MAIN <> 0 {
    // VecDraw expects SHIP-RAW coordinates.
    // Draw a local segment around the aircraft to avoid very-long-line artifacts.
    SET IFC_DEBUG_GS_DRAW_MAIN:START TO seg_start_shipraw.
    SET IFC_DEBUG_GS_DRAW_MAIN:VEC TO seg_end_shipraw - seg_start_shipraw.
    SET IFC_DEBUG_GS_DRAW_MAIN:WIDTH TO draw_width_m.
    SET IFC_DEBUG_GS_DRAW_MAIN:SHOW TO TRUE.
  }
}

FUNCTION RUN_APPROACH {
  // Ensure main-gear flare sensor cache is ready for approach-only save loads.
  PRIME_MAIN_GEAR_CACHE().
  IFC_GEAR_VIS_SYNC().
  _APP_GS_VIS_SYNC().
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
// - ENROUTE: FLY_TO_FIX before speed gate, keep cruise/enroute speed.
// - FIX_INT: FLY_TO_FIX after speed gate, decelerate to derived Vint.
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

FUNCTION _APP_GET_DIST_TO_THR_FOR_SPEED_GATE {
  // Prefer along-track ILS distance when valid; fall back to direct geo range.
  IF ILS_DIST_M > 0 { RETURN ILS_DIST_M. }
  IF ILS_THR_GEO_LL <> 0 { RETURN GEO_DISTANCE(SHIP:GEOPOSITION, ILS_THR_GEO_LL). }
  RETURN 1000000000.
}

FUNCTION _UPDATE_APPROACH_SPEED_TARGET {
  LOCAL vapp IS ACTIVE_V_APP.
  LOCAL vref IS _GET_VREF_TARGET().
  LOCAL intercept_gain IS AC_PARAM("app_spd_intercept_gain",    APP_SPD_INTERCEPT_GAIN,    0).
  LOCAL intercept_min_add IS AC_PARAM("app_spd_intercept_min_add", APP_SPD_INTERCEPT_MIN_ADD, 0.001).
  LOCAL intercept_max_add IS AC_PARAM("app_spd_intercept_max_add", APP_SPD_INTERCEPT_MAX_ADD, 0.001).
  LOCAL intercept_arm_dist_m IS AC_PARAM("app_spd_intercept_arm_dist_m", APP_SPD_INTERCEPT_ARM_DIST_M, 0.001).
  LOCAL intercept_arm_alt_m IS AC_PARAM("app_spd_intercept_arm_alt_m", APP_SPD_INTERCEPT_ARM_ALT_M, 0.001).
  LOCAL intercept_release_factor IS AC_PARAM("app_spd_intercept_release_factor", APP_SPD_INTERCEPT_RELEASE_FACTOR, 0.001).
  LOCAL short_final_agl IS AC_PARAM("app_short_final_agl",      APP_SHORT_FINAL_AGL_M,     0.001).
  LOCAL speed_tgt_slew_per_s IS AC_PARAM("app_speed_tgt_slew_per_s", APP_SPEED_TGT_SLEW_PER_S, 0.001).

  // app_short_final_cap is a boolean flag stored as 0/1 in aircraft config.
  LOCAL short_final_cap_when_not_final IS APP_SHORT_FINAL_CAP_WHEN_NOT_FINAL.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("app_short_final_cap") AND ACTIVE_AIRCRAFT["app_short_final_cap"] >= 0 {
    SET short_final_cap_when_not_final TO ACTIVE_AIRCRAFT["app_short_final_cap"] <> 0.
  }

  IF intercept_max_add < intercept_min_add { SET intercept_max_add TO intercept_min_add. }
  IF intercept_release_factor < 1 { SET intercept_release_factor TO 1. }
  LOCAL vint IS _GET_VINTERCEPT_TARGET(vapp, vref, intercept_gain, intercept_min_add, intercept_max_add).
  LOCAL enroute_tgt IS APP_SPD_ENROUTE_TARGET.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("app_spd_enroute_target") {
    SET enroute_tgt TO ACTIVE_AIRCRAFT["app_spd_enroute_target"].
  }
  IF enroute_tgt <= 0 {
    IF CRUISE_SPD_MPS > 0 {
      SET enroute_tgt TO CRUISE_SPD_MPS.
    } ELSE {
      SET enroute_tgt TO GET_IAS().
    }
  }
  IF enroute_tgt < vint { SET enroute_tgt TO vint. }

  LOCAL base_tgt IS vint.
  LOCAL short_final_frac IS 0.
  LOCAL short_final_target IS vapp.
  LOCAL short_cap_applied IS FALSE.
  LOCAL agl IS GET_AGL().
  LOCAL ias_now IS GET_IAS().
  IF short_final_agl > 0 AND agl < short_final_agl {
    SET short_final_frac TO CLAMP((short_final_agl - agl) / short_final_agl, 0, 1).
    SET short_final_target TO vapp + (vref - vapp) * short_final_frac.
  }

  // In FLY_TO_FIX, keep enroute speed until a distance/altitude gate
  // says it's time to slow to intercept speed.
  IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX {
    LOCAL dist_to_thr_m IS _APP_GET_DIST_TO_THR_FOR_SPEED_GATE().
    LOCAL gate_m IS intercept_arm_dist_m.
    LOCAL alt_now_m IS SHIP:ALTITUDE.
    LOCAL arm_now IS FALSE.
    IF APP_INTERCEPT_ARMED {
      LOCAL release_gate_m IS gate_m * intercept_release_factor.
      LOCAL release_alt_m IS intercept_arm_alt_m * intercept_release_factor.
      SET arm_now TO dist_to_thr_m <= release_gate_m OR alt_now_m <= release_alt_m.
    } ELSE {
      SET arm_now TO dist_to_thr_m <= gate_m OR alt_now_m <= intercept_arm_alt_m.
    }
    SET APP_INTERCEPT_ARMED TO arm_now.
    SET APP_SPD_DIST_THR_M TO dist_to_thr_m.
    IF NOT APP_INTERCEPT_ARMED {
      SET base_tgt TO enroute_tgt.
    }
  } ELSE {
    SET APP_INTERCEPT_ARMED TO FALSE.
    SET APP_SPD_DIST_THR_M TO _APP_GET_DIST_TO_THR_FOR_SPEED_GATE().
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
      LOCAL aoa_floor  IS ias_now + aoa_excess * APP_AOA_SPD_GAIN.
      IF aoa_floor > base_tgt { SET base_tgt TO aoa_floor. }
    }
  }

  // Export scheduler internals for terminal/log diagnostics.
  SET APP_VREF_TGT TO vref.
  SET APP_VINT_TGT TO vint.
  SET APP_BASE_V_TGT TO base_tgt.
  SET APP_SHORT_FINAL_FRAC TO short_final_frac.
  IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX {
    IF APP_INTERCEPT_ARMED {
      SET APP_SPD_MODE TO "FIX_INT".
    } ELSE {
      SET APP_SPD_MODE TO "ENROUTE".
    }
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
// APPROACH THROTTLE  (shared by FLY_TO_FIX and ILS_TRACK)
//
// IFC autothrottle (AT_RUN_SPEED_HOLD) always owns the throttle.
// AA Cruise / Director manage heading + vertical guidance; IFC AT owns throttle.
// AA SPEEDCONTROL is explicitly disabled so it cannot fight AT.
//
// Speed target comes from _UPDATE_APPROACH_SPEED_TARGET().
// ─────────────────────────────────────────────────────────
FUNCTION _APP_AA_GET_HANDLE {
  IF NOT ADDONS:AVAILABLE("AA") { RETURN 0. }
  IF NOT ADDONS:HASSUFFIX("AA") { RETURN 0. }
  LOCAL aa IS ADDONS:AA.
  IF aa = 0 { RETURN 0. }
  RETURN aa.
}

// Disable AA speed control + cruise; hand throttle authority back to IFC.
// Must be called before transitioning to flare/autoland.
FUNCTION _APP_AA_RELEASE {
  LOCAL aa IS _APP_AA_GET_HANDLE().
  IF aa <> 0 {
    IF aa:HASSUFFIX("SPEEDCONTROL") AND aa:SPEEDCONTROL { SET aa:SPEEDCONTROL TO FALSE. }
    IF aa:HASSUFFIX("CRUISE")       AND aa:CRUISE       { SET aa:CRUISE       TO FALSE. }
  }
  AS_RELEASE().
  SET THROTTLE_CMD TO GET_CURRENT_THROTTLE().
  LOCK THROTTLE TO THROTTLE_CMD.
}

FUNCTION _RUN_APPROACH_THROTTLE {
  LOCAL v_tgt IS _UPDATE_APPROACH_SPEED_TARGET().
  // AA SPEEDCONTROL is not used during approach: its aa:SPEED target is not
  // reliably writable and may retain the previous cruise speed.  IFC AT
  // owns the throttle; disable SPEEDCONTROL so AA Cruise does not fight it.
  LOCAL aa IS _APP_AA_GET_HANDLE().
  IF aa <> 0 AND aa:HASSUFFIX("SPEEDCONTROL") AND aa:SPEEDCONTROL {
    SET aa:SPEEDCONTROL TO FALSE.
    SET THROTTLE_CMD TO GET_CURRENT_THROTTLE().
    LOCK THROTTLE TO THROTTLE_CMD.
  }
  AT_RUN_SPEED_HOLD(v_tgt, MIN_APPROACH_THR, 1).
  AS_RUN(v_tgt, "APPROACH").
}

FUNCTION _APP_GEAR_MAX_EXTEND_IAS {
  RETURN AC_PARAM("gear_max_extend_ias", GEAR_MAX_EXTEND_IAS, 0.001).
}

FUNCTION _APP_GEAR_CAN_EXTEND {
  LOCAL vmax IS _APP_GEAR_MAX_EXTEND_IAS().
  IF vmax <= 0 { RETURN TRUE. }
  RETURN GET_IAS() <= vmax.
}

FUNCTION _APP_TRY_EXTEND_GEAR {
  IF _APP_GEAR_CAN_EXTEND() {
    GEAR ON.
  }
}

// ─────────────────────────────────────────────────────────
// SUB-PHASE: FLY_TO_FIX
// Navigate toward each fix in the approach sequence.
// Descend to the target altitude associated with each fix.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLY_TO_FIX {
  // If we're already on the localizer inbound and heading toward the runway,
  // hand off directly to ILS track without waiting for the remaining fix sequence.
  // Heading alignment check prevents early capture when heading away from the runway.
  IF ACTIVE_ILS_ID <> "" {
    LOCAL ils_dev IS _COMPUTE_ILS_DEVIATIONS().
    LOCAL early_hdg_ok IS ABS(WRAP_180(GET_COMPASS_HDG() - ACTIVE_RWY_HDG)) < 90.
    IF ils_dev:HASKEY("loc") AND ils_dev:HASKEY("dist")
        AND ils_dev["dist"] > 0
        AND ABS(ils_dev["loc"]) <= LOC_CAPTURE_M
        AND early_hdg_ok {
      SET ILS_LOC_DEV TO ils_dev["loc"].
      IF ils_dev:HASKEY("gs") { SET ILS_GS_DEV TO ils_dev["gs"]. }
      SET ILS_DIST_M TO ils_dev["dist"].
      SET ILS_INTERCEPT_ALT TO SHIP:ALTITUDE.
      SET_SUBPHASE(SUBPHASE_ILS_TRACK).
      SET PREV_LOC_DEV TO 0.
      SET PREV_GS_DEV  TO 0.
      RETURN.
    }
  }

  // If all fixes have been passed, begin ILS tracking.
  IF FIX_INDEX >= ACTIVE_FIXES:LENGTH {
    LOCAL dev_now IS _COMPUTE_ILS_DEVIATIONS().
    IF dev_now:HASKEY("loc")  { SET ILS_LOC_DEV TO dev_now["loc"]. }
    IF dev_now:HASKEY("gs")   { SET ILS_GS_DEV  TO dev_now["gs"]. }
    IF dev_now:HASKEY("dist") { SET ILS_DIST_M  TO dev_now["dist"]. }
    SET ILS_INTERCEPT_ALT TO SHIP:ALTITUDE.
    SET_SUBPHASE(SUBPHASE_ILS_TRACK).
    SET PREV_LOC_DEV TO 0.
    SET PREV_GS_DEV  TO 0.
    RETURN.
  }

  SET TELEM_FTF_FIX_IDX TO FIX_INDEX.

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

  // Skip fixes that require a reversal relative to the runway approach heading.
  // If the bearing to this fix differs by more than 90° from the approach heading,
  // the fix is effectively "behind" the aircraft (already overflown or wrong-direction).
  // Advance FIX_INDEX and return immediately — no reversal command is issued.
  IF ABS(WRAP_180(brg - ACTIVE_RWY_HDG)) > 90 {
    SET FIX_INDEX TO FIX_INDEX + 1.
    RETURN.
  }

  // Heading error to the fix (0-180).
  // NOTE: SHIP:HEADING returns 0 in kOS; use the vector-based GET_COMPASS_HDG().
  LOCAL hdg_err IS WRAP_360(brg - GET_COMPASS_HDG()).
  IF hdg_err > 180 { SET hdg_err TO 360 - hdg_err. }
  SET TELEM_FTF_HDG_ERR TO hdg_err.

  // FPA command = proportional altitude error + VS damping.
  // KD_FTF_ALT_FPA (0.3) is much larger than the global KD_ALT_FPA (0.05) to
  // suppress the phugoid that develops when decelerating from cruise to
  // approach speed (rising AOA drives pitch_cmd upward without damping).
  //
  // Suppression logic during heading changes:
  //   hdg_err > 90°: keep pure altitude-hold (no VS damping) instead of
  //                  forcing FPA=0, so vertical guidance remains active.
  //   45–90°:  suppress altitude-error descent (avoid spiral dives) but keep
  //            VS damping active so phugoid doesn't build up during the turn.
  //   < 45°:   full KP + KD command.
  LOCAL alt_err IS SHIP:ALTITUDE - tgt_alt.
  LOCAL vs_now  IS SHIP:VERTICALSPEED.
  LOCAL fpa_cmd IS CLAMP(-(alt_err * KP_ALT_FPA + vs_now * KD_FTF_ALT_FPA), MAX_DESC_FPA, MAX_CLIMB_FPA).
  IF hdg_err > 90 {
    SET fpa_cmd TO CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, MAX_CLIMB_FPA).
  }
  ELSE IF hdg_err > 45 AND -alt_err * KP_ALT_FPA < 0 {
    // Altitude correction would descend — suppress it during moderate turns.
    // Retain anti-phugoid VS damping so climbs are still corrected.
    SET fpa_cmd TO CLAMP(-(vs_now * KD_FTF_ALT_FPA), MAX_DESC_FPA, 0).
  }

  // AA Cruise with WAYPOINT mode: feed fix_ll directly to aa:WAYPOINT so AA
  // navigates to the fix natively (no per-cycle bearing computation needed).
  // WAYPOINT and HEADING are mutually exclusive in AA — setting WAYPOINT
  // deactivates HEADING automatically.
  //
  // Vertical path in Approach/Cruise uses AA altitude hold at the current leg
  // target altitude (tgt_alt). This avoids driving pre-capture vertical path
  // through FPANGLE commands in Cruise mode.
  //
  // Compute hdg_cmd for the fallback Director path and for TELEM only.
  LOCAL hdg_now IS GET_COMPASS_HDG().
  LOCAL signed_err IS WRAP_180(brg - hdg_now).
  LOCAL hdg_cmd IS WRAP_360(hdg_now + CLAMP(signed_err, -90, 90)).
  LOCAL aa IS _APP_AA_GET_HANDLE().
  IF aa <> 0 {
    IF aa:HASSUFFIX("DIRECTOR") AND aa:DIRECTOR { SET aa:DIRECTOR TO FALSE. }
    IF aa:HASSUFFIX("FBW")      AND aa:FBW      { SET aa:FBW      TO FALSE. }
    IF aa:HASSUFFIX("CRUISE")   AND NOT aa:CRUISE { SET aa:CRUISE TO TRUE. }
    IF aa:HASSUFFIX("WAYPOINT") { SET aa:WAYPOINT TO fix_ll. }
    ELSE IF aa:HASSUFFIX("HEADING") { SET aa:HEADING TO hdg_cmd. }
    IF aa:HASSUFFIX("ALTITUDE") {
      SET aa:ALTITUDE TO tgt_alt.
    } ELSE IF aa:HASSUFFIX("FPANGLE") {
      // Compatibility fallback for AA versions without ALTITUDE in Cruise.
      SET aa:FPANGLE TO fpa_cmd.
    }
    SET IFC_DESIRED_STEERING TO HEADING(hdg_cmd, fpa_cmd).
  } ELSE {
    AA_SET_DIRECTOR_FPA(hdg_cmd, fpa_cmd).
  }
  SET TELEM_AA_HDG_CMD TO hdg_cmd.
  SET TELEM_AA_FPA_CMD TO fpa_cmd.
  SET TELEM_LOC_CORR   TO 0.
  SET TELEM_GS_CORR    TO 0.

  _RUN_APPROACH_THROTTLE().

  // Extend gear if aircraft config specifies a gear-down AGL.
  LOCAL gear_agl IS AC_PARAM("gear_down_agl", 0, 0.001).
  IF gear_agl > 0 AND GET_AGL() < gear_agl {
    _APP_TRY_EXTEND_GEAR().
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
  // GS capture latch with hysteresis: hold capture once acquired, but allow
  // release if we diverge well outside the capture band.
  IF ABS(gs_m) <= GS_CAPTURE_M { SET APP_GS_LATCHED TO TRUE. }
  ELSE IF APP_GS_LATCHED AND ABS(gs_m) > GS_CAPTURE_M * GS_LATCH_RELEASE_FACTOR {
    SET APP_GS_LATCHED TO FALSE.
  }
  LOCAL gs_captured IS APP_GS_LATCHED.

  // Enter full approach configuration on GS capture.
  // Before GS capture, keep gear-up unless explicit AGL rule asks for gear.
  LOCAL gear_agl IS AC_PARAM("gear_down_agl", 0, 0.001).
  IF gs_captured OR (gear_agl > 0 AND GET_RUNWAY_REL_HEIGHT() < gear_agl) {
    _APP_TRY_EXTEND_GEAR().
  }

  // Derivative (rate of change over one loop cycle).
  LOCAL d_loc IS (loc_m - PREV_LOC_DEV) / IFC_ACTUAL_DT.
  LOCAL d_gs  IS (gs_m  - PREV_GS_DEV)  / IFC_ACTUAL_DT.
  SET PREV_LOC_DEV TO loc_m.
  SET PREV_GS_DEV  TO gs_m.
  SET TELEM_D_GS   TO d_gs.

  // ── Lateral (localizer) ──
  // Positive loc_m = right of centerline → turn left (reduce heading).
  LOCAL loc_corr IS -(KP_LOC * loc_m + KD_LOC * d_loc).

  // Bank angle limiter: AA has no native bank limit, so we implement it here.
  // Fade loc_corr to zero as bank approaches limit; add counter-correction if over.
  LOCAL max_bank IS AC_PARAM("aa_max_bank", AA_MAX_BANK, 0.001).
  LOCAL bank IS TELEM_BANK_DEG.
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
  // Before GS capture: steer toward the glideslope (see ELSE branch below).
  // After GS capture: track the glideslope with PD feedback.
  LOCAL fpa_cmd IS 0.
  LOCAL gs_corr IS 0.
  IF gs_captured {
    SET gs_corr TO -(KP_GS * gs_m + KD_GS * d_gs).
    SET fpa_cmd TO -ACTIVE_GS_ANGLE + CLAMP(gs_corr, -MAX_GS_CORR_DN, MAX_GS_CORR_UP).
  } ELSE {
    // Pre-capture vertical guidance — two cases depending on GS position:
    //
    // Above GS (gs_m > 0):
    //   The GS beam descends away from the aircraft as it approaches, so
    //   altitude-hold cannot achieve intercept.  Command descent at least as
    //   steep as the GS slope, plus a proportional term for how far above the
    //   beam the aircraft is.  Upper clamp at -ACTIVE_GS_ANGLE ensures the
    //   aircraft always descends at least as fast as the beam.
    //
    // Below GS (gs_m < 0):
    //   Hold the ILS intercept altitude captured at LOC capture.  The GS beam
    //   descends to the aircraft as it approaches (standard intercept from
    //   below).  Never command a climb pre-capture.
    IF gs_m > 0 {
      SET fpa_cmd TO CLAMP(-ACTIVE_GS_ANGLE - KP_ALT_FPA * gs_m, MAX_DESC_FPA, -ACTIVE_GS_ANGLE).
    } ELSE {
      LOCAL alt_err IS SHIP:ALTITUDE - ILS_INTERCEPT_ALT.
      SET fpa_cmd TO CLAMP(-alt_err * KP_ALT_FPA, MAX_DESC_FPA, 0).
    }
  }

  // Use Director for all ILS tracking (including pre-GS intercept) so
  // localizer correction is immediate once ILS is active.
  // Keep pre-clamp telemetry for diagnosis.
  SET TELEM_FPA_PRECLAMPED TO fpa_cmd.
  IF FAR_AVAILABLE {
    LOCAL aoa_now IS GET_AOA().
    LOCAL max_fpa_for_pitch IS MAX_APP_PITCH_CMD.
    IF AA_DIR_ADD_AOA_COMP {
      SET max_fpa_for_pitch TO MAX_APP_PITCH_CMD - aoa_now.
    }
    SET fpa_cmd TO MAX(MIN(fpa_cmd, max_fpa_for_pitch), APP_FPA_PITCH_FLOOR).
  }
  AA_SET_DIRECTOR_FPA(hdg_cmd, fpa_cmd).
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
  LOCAL flare_entry_vs_min IS AC_PARAM("flare_entry_vs_min", FLARE_MIN_ENTRY_SINK_VS, -50).
  LOCAL flare_ctrl_h_offset_max IS AC_PARAM("flare_ctrl_h_offset_max_m", FLARE_CTRL_H_OFFSET_MAX_M, 0.1).
  LOCAL flare_h_now IS GET_MAIN_GEAR_RUNWAY_HEIGHT_MIN().
  LOCAL vs_now IS SHIP:VERTICALSPEED.

  IF flare_h_now < flare_arm_agl {
    IF vs_now <= flare_trigger_max_vs {
      IF FLARE_TRIGGER_START_UT < 0 { SET FLARE_TRIGGER_START_UT TO TIME:SECONDS. }
      IF TIME:SECONDS - FLARE_TRIGGER_START_UT >= FLARE_TRIGGER_CONFIRM_S {
        LOCAL entry_ias IS MAX(GET_IAS(), 10).
        LOCAL runway_h_now IS GET_RUNWAY_REL_HEIGHT().
        LOCAL flare_ctrl_h_offset_raw IS runway_h_now - flare_h_now.
        SET flare_ctrl_h_offset_max TO MAX(flare_ctrl_h_offset_max, 0.1).
        SET FLARE_CTRL_H_OFFSET TO CLAMP(flare_ctrl_h_offset_raw, 0, flare_ctrl_h_offset_max).
        IF flare_entry_vs_min > -0.05 { SET flare_entry_vs_min TO -0.05. }
        SET FLARE_PITCH_CMD TO ARCTAN(vs_now / entry_ias). // seed from current flight path angle
        SET FLARE_ENTRY_VS  TO CLAMP(vs_now, flare_entry_vs_min, -0.05). // seed to a bounded descending band
        // Capture control reference height at flare entry (gear height plus entry offset).
        SET FLARE_ENTRY_AGL TO MAX(flare_h_now + FLARE_CTRL_H_OFFSET, 1).
        SET FLARE_SUBMODE TO FLARE_MODE_CAPTURE.
        SET FLARE_AUTH_LIMITED TO FALSE.
        SET FLARE_BALLOON_ACTIVE TO FALSE.
        SET FLARE_AUTH_START_UT TO -1.
        SET FLARE_TECS_ET_INT TO 0.
        SET FLARE_TECS_EB_INT TO 0.
        SET FLARE_TECS_H_REF TO FLARE_ENTRY_AGL.
        SET THR_INTEGRAL TO 0.
        SET FLARE_TRIGGER_START_UT TO -1.
        SET TOUCHDOWN_CANDIDATE_UT TO -1.
        _APP_AA_RELEASE().  // hand throttle authority back to IFC before flare
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

  // Use ILS_DIST_M cached by _COMPUTE_ILS_DEVIATIONS this cycle (P3c).
  IF ILS_DIST_M <= 0 { RETURN. }
  LOCAL dist_km IS ILS_DIST_M / 1000.
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

  // Use ILS_DIST_M cached by _COMPUTE_ILS_DEVIATIONS this cycle (P3c).
  IF ILS_DIST_M <= 0 { RETURN. }
  LOCAL dist_km IS ILS_DIST_M / 1000.
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
  // Threshold world-space position — use cached GeoCoordinates (P2b).
  LOCAL thr_pos IS ILS_THR_GEO_LL:ALTITUDEPOSITION(ACTIVE_THR_ALT).

  // Unit vectors along and perpendicular to the runway (horizontal).
  // Build a single HEADING Direction and extract both vectors (P2c).
  LOCAL rwy_dir   IS HEADING(ACTIVE_RWY_HDG, 0).
  LOCAL rwy_fwd   IS rwy_dir:FOREVECTOR.
  LOCAL rwy_right IS rwy_dir:STARVECTOR.

  // Displacement from threshold to aircraft.
  LOCAL disp IS SHIP:POSITION - thr_pos.

  // Along-track component: negative = aircraft is still on approach (before threshold).
  LOCAL along_m IS VDOT(disp, rwy_fwd).
  LOCAL dist_m  IS -along_m.   // positive when on approach

  // Lateral component: positive = right of centerline.
  LOCAL loc_m IS VDOT(disp, rwy_right).

  // Nominal glideslope altitude at this distance from threshold.
  // Use cached TAN(gs_angle) — set once in IFC_LOAD_PLATE (P2b).
  LOCAL gs_nom_alt IS ACTIVE_THR_ALT + dist_m * ILS_GS_TAN_CACHED.
  LOCAL gs_m       IS SHIP:ALTITUDE - gs_nom_alt.  // positive = above GS

  // Cache distance globally so flap/spoiler checks can reuse it (P3b).
  SET ILS_DIST_M TO dist_m.

  RETURN LEXICON("loc", loc_m, "gs", gs_m, "dist", dist_m).
}
