@LAZYGLOBAL OFF.

// ============================================================
// ifc_display.ks  -  Integrated Flight Computer
// Context-sensitive page renderers for the FMS terminal UI.
//
// Zone layout (row numbers):
//   0        thick separator
//   1        HEADER  (aircraft, timer, arm state)
//   2        thick separator
//   3        BREADCRUMB  (phase > subphase, runway)
//   4        thin separator
//   5-10     PRIMARY  (phase-specific main data)
//   11       thin separator
//   12-14    SECONDARY  (debug panel, toggle with D)
//   15       thin separator
//   16       ALERT  (event messages, auto-expire 5 s)
//   17       thin separator
//   18       LOGGER STATUS
//   19       thin separator
//   20       KEY HINTS
//   21       thick separator
// ============================================================

// ── Alert helper ──────────────────────────────────────────
// Set by phase events; also called directly by display code.
FUNCTION DISPLAY_ALERT {
  PARAMETER text.
  SET IFC_ALERT_TEXT TO text.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

// ── Header + breadcrumb ───────────────────────────────────

FUNCTION DISPLAY_HEADER {
  LOCAL ac_name IS "---".
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("name") {
    SET ac_name TO ACTIVE_AIRCRAFT["name"].
  }
  LOCAL t_str IS "T+" + UI_FORMAT_TIME(TIME:SECONDS - IFC_MISSION_START_UT).
  LOCAL arm_str IS "[ARM]".
  IF IFC_PHASE = PHASE_DONE { SET arm_str TO "[DONE]". }
  // Build header: "IFC 2.0  <name>         T+ 00:00:00  [ARM]"
  LOCAL left  IS "IFC 2.0  " + ac_name.
  LOCAL right IS t_str + "  " + arm_str.
  LOCAL gap IS MAX(UI_W - left:LENGTH - right:LENGTH, 1).
  UI_P(left + STR_REPEAT(" ", gap) + right, UI_HDR_ROW).
}

FUNCTION DISPLAY_BREADCRUMB {
  LOCAL phase_str IS IFC_PHASE.
  IF IFC_SUBPHASE <> "" { SET phase_str TO IFC_PHASE + " › " + IFC_SUBPHASE. }
  LOCAL rwy_str IS "".
  IF ACTIVE_ILS_ID <> "" {
    SET rwy_str TO ACTIVE_ILS_ID + "  " + ROUND(ACTIVE_GS_ANGLE, 1) + "° GS".
  }
  LOCAL gap IS MAX(UI_W - phase_str:LENGTH - rwy_str:LENGTH, 1).
  UI_P(phase_str + STR_REPEAT(" ", gap) + rwy_str, UI_CRUMB_ROW).
}

// ── Primary data rows (phase-specific) ────────────────────

// Shared top-two-row air data block used by most pages.
// Returns nothing; writes rows UI_PRI_TOP and UI_PRI_TOP+1.
FUNCTION _DISPLAY_AIR_DATA {
  LOCAL ias   IS ROUND(GET_IAS(), 1).
  LOCAL agl   IS ROUND(GET_AGL(), 0).
  LOCAL vs    IS ROUND(SHIP:VERTICALSPEED, 1).
  LOCAL hdg   IS ROUND(GET_COMPASS_HDG(), 1).
  LOCAL thr   IS ROUND(THROTTLE_CMD, 2).
  LOCAL flp   IS UI_FORMAT_FLP(FLAPS_CURRENT_DETENT).
  LOCAL pitch IS ROUND(GET_PITCH(), 1).
  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s   AGL " +
       STR_RJUST("" + agl, 5) + " m    VS " +
       STR_RJUST("" + vs, 6) + " m/s", UI_PRI_TOP).
  UI_P("  HDG " + STR_RJUST("" + hdg, 5) + " deg   THR " +
       STR_RJUST("" + thr, 4) + "    FLP " + flp +
       "   PCH " + pitch + " deg", UI_PRI_TOP + 1).
}

// PRE-FLIGHT / ARM SCREEN ─────────────────────────────────
// Shown before ARM is confirmed.

FUNCTION DISPLAY_PREARM {
  PARAMETER proc_idx, rwy_idx, dist_idx.
  LOCAL ac_name IS "Unknown".
  LOCAL mass_t  IS ROUND(SHIP:MASS, 1).
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("name") {
    SET ac_name TO ACTIVE_AIRCRAFT["name"].
  }

  LOCAL proc_names IS LIST("ILS APPROACH", "TAKEOFF").
  LOCAL rwy_names  IS LIST("KSC RWY 09", "KSC RWY 27").
  LOCAL dist_names IS LIST("LONG  60 km", "SHORT 30 km").

  // Speeds
  LOCAL vapp IS ROUND(AC_PARAM("v_app",  75.0, 0.001), 0).
  LOCAL vref IS ROUND(AC_PARAM("v_ref",  65.0, 0.001), 0).
  LOCAL vr   IS ROUND(AC_PARAM("v_r",   70.0, 0.001), 0).
  LOCAL v2   IS ROUND(AC_PARAM("v2",    80.0, 0.001), 0).

  // Flap info for takeoff
  LOCAL det_clmb IS ROUND(AC_PARAM("flaps_detent_climb", 1, 0)).
  LOCAL det_app  IS ROUND(AC_PARAM("flaps_detent_approach", 2, 0)).
  LOCAL det_to   IS det_clmb.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_detent_takeoff") {
    SET det_to TO ROUND(ACTIVE_AIRCRAFT["flaps_detent_takeoff"]).
  } ELSE IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_detent_approach") {
    SET det_to TO det_app.
  }
  LOCAL vfe_str IS "".
  IF det_to = ROUND(AC_PARAM("flaps_detent_climb",    1, 0)) {
    SET vfe_str TO "Vfe " + ROUND(AC_PARAM("vfe_climb",    250, 0.001), 0) + " m/s".
  } ELSE IF det_to = ROUND(AC_PARAM("flaps_detent_approach", 2, 0)) {
    SET vfe_str TO "Vfe " + ROUND(AC_PARAM("vfe_approach", 220, 0.001), 0) + " m/s".
  } ELSE IF det_to = ROUND(AC_PARAM("flaps_detent_landing",  3, 0)) {
    SET vfe_str TO "Vfe " + ROUND(AC_PARAM("vfe_landing",  210, 0.001), 0) + " m/s".
  }

  UI_P("  AIRCRAFT  " + STR_PAD(ac_name, 24) + "MASS  " + mass_t + " t", UI_PRI_TOP).
  LOCAL dist_str IS "".
  IF proc_idx = 0 { SET dist_str TO "   " + dist_names[dist_idx]. }
  UI_P("  PROCEDURE " + proc_names[proc_idx] +
       "   " + rwy_names[rwy_idx] +
       dist_str, UI_PRI_TOP + 1).
  UI_CLR(UI_PRI_TOP + 2).
  UI_P("  SPEEDS  Vapp " + vapp + "  Vref " + vref +
       "  VR " + vr + "  V2 " + v2 + "  m/s", UI_PRI_TOP + 3).
  UI_P("  FLAPS   TAKEOFF (detent " + det_to + ")   " + vfe_str, UI_PRI_TOP + 4).
  UI_CLR(UI_PRI_TOP + 5).
}

// FLY TO FIX ─────────────────────────────────────────────
FUNCTION DISPLAY_FLY_TO_FIX {
  _DISPLAY_AIR_DATA().
  UI_CLR(UI_PRI_TOP + 2).

  IF ACTIVE_FIXES:LENGTH > 0 AND FIX_INDEX < ACTIVE_FIXES:LENGTH {
    LOCAL fix_id  IS ACTIVE_FIXES[FIX_INDEX].
    LOCAL fix     IS GET_BEACON(fix_id).
    LOCAL dist_km IS ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, fix["ll"]) / 1000, 1).
    LOCAL brg     IS ROUND(GEO_BEARING(SHIP:GEOPOSITION, fix["ll"]), 1).
    LOCAL tgt_alt IS 0.
    IF ACTIVE_ALT_AT:HASKEY(fix_id) { SET tgt_alt TO ACTIVE_ALT_AT[fix_id]. }
    UI_P("  FIX  " + STR_PAD(fix_id, 18) + "BRG " +
         STR_RJUST("" + brg, 5) + "°  DIST " + dist_km + " km", UI_PRI_TOP + 3).
    UI_P("  ALT TARGET " + tgt_alt + " m", UI_PRI_TOP + 4).
  } ELSE {
    UI_P("  FIX  (none)", UI_PRI_TOP + 3).
    UI_CLR(UI_PRI_TOP + 4).
  }

  LOCAL next_str IS "".
  IF FIX_INDEX + 1 < ACTIVE_FIXES:LENGTH {
    SET next_str TO "  NEXT " + ACTIVE_FIXES[FIX_INDEX + 1].
  }
  UI_P("  SPD " + APP_SPD_MODE + "  Vtgt " + ROUND(ACTIVE_V_TGT, 1) +
       "  Vapp " + ROUND(ACTIVE_V_APP, 1) + next_str, UI_PRI_TOP + 5).
}

// ILS TRACK ───────────────────────────────────────────────
FUNCTION DISPLAY_ILS_TRACK {
  _DISPLAY_AIR_DATA().

  // Deviation bars (inner width 18 → total 20 with end caps)
  LOCAL loc_bar IS UI_BAR_DEV(ILS_LOC_DEV, 200, 18).
  LOCAL gs_bar  IS UI_BAR_DEV(ILS_GS_DEV,   80, 18).

  LOCAL loc_dir IS "CTR".
  IF ILS_LOC_DEV >  1 { SET loc_dir TO "R  ". }
  IF ILS_LOC_DEV < -1 { SET loc_dir TO "L  ". }
  LOCAL gs_dir IS "ON ".
  IF ILS_GS_DEV >  1 { SET gs_dir TO "HI ". }
  IF ILS_GS_DEV < -1 { SET gs_dir TO "LO ". }

  UI_P("  LOC " + loc_bar + "  " + STR_RJUST(UI_FMT(ILS_LOC_DEV, 1), 6) + " m " + loc_dir, UI_PRI_TOP + 2).
  UI_P("  G/S " + gs_bar  + "  " + STR_RJUST(UI_FMT(ILS_GS_DEV, 1), 6) + " m " + gs_dir +
       "  DIST " + ROUND(ILS_DIST_M / 1000, 2) + " km", UI_PRI_TOP + 3).

  LOCAL gear_str IS "UP  ".
  IF GEAR { SET gear_str TO "DOWN". }
  LOCAL sp_str IS "DISARMED".
  IF APP_SPOILERS_ARMED { SET sp_str TO "ARMED   ". }

  UI_P("  SPD " + STR_PAD(APP_SPD_MODE, 11) +
       " Vtgt " + ROUND(ACTIVE_V_TGT, 1) +
       "  Vapp " + ROUND(ACTIVE_V_APP, 1), UI_PRI_TOP + 4).
  UI_P("  GEAR " + gear_str + "  SPOILERS " + sp_str, UI_PRI_TOP + 5).
}

// FLARE ───────────────────────────────────────────────────
FUNCTION DISPLAY_FLARE {
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL agl IS GET_AGL().
  LOCAL vs  IS ROUND(SHIP:VERTICALSPEED, 2).
  LOCAL aoa IS ROUND(GET_AOA(), 1).
  LOCAL flp IS UI_FORMAT_FLP(FLAPS_CURRENT_DETENT).

  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s   THR " +
       STR_RJUST("" + ROUND(THROTTLE_CMD, 2), 4) + "   FLP " + flp, UI_PRI_TOP).
  UI_P("  AoA " + STR_RJUST("" + aoa, 5) + " deg   PITCH " +
       ROUND(GET_PITCH(), 1) + " deg", UI_PRI_TOP + 1).

  // AGL fill bar (fills right-to-left as altitude drops)
  LOCAL flare_entry IS MAX(FLARE_ENTRY_AGL, 1).
  LOCAL agl_frac IS CLAMP(agl / flare_entry, 0, 1).
  LOCAL agl_bar IS UI_BAR_FILL(agl_frac, 24, "█", "░").
  UI_P("  AGL  " + agl_bar + " " + STR_RJUST("" + ROUND(agl, 0), 4) + " m", UI_PRI_TOP + 2).

  // VS bar: range -6..0, marker at tgt vs current
  LOCAL vs_bar IS UI_BAR_DEV(vs, 6.0, 18).
  UI_P("  VS   " + vs_bar + "  " + STR_RJUST("" + vs, 6) +
       " m/s  tgt " + ROUND(TELEM_FLARE_TGT_VS, 2), UI_PRI_TOP + 3).

  // Flare progress bar
  LOCAL frac_bar IS UI_BAR_FILL(TELEM_FLARE_FRAC, 24, "█", "░").
  UI_P("  PROG " + frac_bar + " " + ROUND(TELEM_FLARE_FRAC * 100, 0) + "%", UI_PRI_TOP + 4).
  UI_P("  FPA CMD " + ROUND(FLARE_PITCH_CMD, 2) + " deg", UI_PRI_TOP + 5).
}

// TOUCHDOWN ───────────────────────────────────────────────
FUNCTION DISPLAY_TOUCHDOWN {
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL vs  IS ROUND(SHIP:VERTICALSPEED, 2).
  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s    VS  " + vs + " m/s", UI_PRI_TOP).
  UI_CLR(UI_PRI_TOP + 1).
  UI_P("  ████  TOUCHDOWN CONFIRMED  ████", UI_PRI_TOP + 2).
  UI_CLR(UI_PRI_TOP + 3).

  LOCAL sp_ag IS 0.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("ag_spoilers") {
    SET sp_ag TO ACTIVE_AIRCRAFT["ag_spoilers"].
  }
  LOCAL sp_str IS "n/a".
  IF sp_ag > 0 { SET sp_str TO "DEPLOYING (AG" + sp_ag + ")". }
  UI_P("  SPOILERS  " + sp_str, UI_PRI_TOP + 4).
  UI_CLR(UI_PRI_TOP + 5).
}

// ROLLOUT ─────────────────────────────────────────────────
FUNCTION DISPLAY_ROLLOUT {
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL brk_str IS "OFF".
  IF BRAKES { SET brk_str TO "ON ". }
  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s   AGL " +
       ROUND(GET_AGL(), 0) + " m    BRAKES " + brk_str, UI_PRI_TOP).
  UI_CLR(UI_PRI_TOP + 1).

  // Heading error bar (±15 deg scale, inner width 18)
  LOCAL hdg_bar IS UI_BAR_DEV(TELEM_RO_HDG_ERR, 15, 18).
  UI_P("  HDG ERR " + hdg_bar + " " +
       STR_RJUST(UI_FMT(TELEM_RO_HDG_ERR, 1), 5) + " deg", UI_PRI_TOP + 2).

  // Pitch bar (±8 deg scale)
  LOCAL p_err IS TELEM_RO_PITCH_TGT - GET_PITCH().
  LOCAL p_bar IS UI_BAR_DEV(p_err, 8, 18).
  UI_P("  PITCH    " + p_bar + " tgt " +
       ROUND(TELEM_RO_PITCH_TGT, 1) + " deg", UI_PRI_TOP + 3).

  // Yaw assist and steer blend
  LOCAL gate_str IS "ACTIVE ".
  IF TELEM_RO_YAW_GATE = 1 { SET gate_str TO "SPD-GT ". }
  IF TELEM_RO_YAW_GATE = 2 { SET gate_str TO "ERR-GT ". }
  IF TELEM_RO_YAW_GATE = 3 { SET gate_str TO "AIRBRN ". }
  LOCAL yaw_bar IS UI_BAR_FILL(ABS(TELEM_RO_YAW_TGT) / 0.5, 10, "▐", "░").
  UI_P("  YAW  " + yaw_bar + " " + UI_FMT(TELEM_RO_YAW_TGT, 3) +
       "  GATE " + gate_str +
       "  BLEND " + ROUND(TELEM_STEER_BLEND * 100, 0) + "%", UI_PRI_TOP + 4).

  // Rough stopping estimate (IAS² / 2g, very approximate)
  LOCAL stop_str IS "".
  IF ias > 5 {
    LOCAL est_m IS ROUND(ias * ias / (2.0 * 6.0), 0).
    SET stop_str TO "~" + est_m + " m est".
  }
  UI_P("  STOPPING  " + stop_str, UI_PRI_TOP + 5).
}

// TAKEOFF ─────────────────────────────────────────────────
FUNCTION DISPLAY_TAKEOFF {
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL agl IS ROUND(GET_AGL(), 0).
  LOCAL thr IS ROUND(THROTTLE_CMD, 2).
  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s   AGL " +
       STR_RJUST("" + agl, 5) + " m    THR " + thr, UI_PRI_TOP).

  // V-speed tape (IAS mapped from 0..V2+20)
  LOCAL v_r IS ROUND(AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001), 0).
  LOCAL v2  IS ROUND(AC_PARAM("v2",  TAKEOFF_V2_DEFAULT,  0.001), 0).
  LOCAL tape_max IS v2 + 20.
  LOCAL tape_bar IS UI_BAR_FILL(ias / tape_max, 28, "▐", "░").
  UI_P("  IAS  " + tape_bar + " " + STR_RJUST("" + ias, 4) + " m/s", UI_PRI_TOP + 1).
  // V-speed markers below the tape
  LOCAL vr_pos IS ROUND(CLAMP(v_r / tape_max, 0, 1) * 28).
  LOCAL v2_pos IS ROUND(CLAMP(v2  / tape_max, 0, 1) * 28).
  LOCAL vr_label IS STR_REPEAT(" ", 6 + vr_pos) + "▲VR".
  // Merge: place both on the same row (V2 overwrites if overlap)
  LOCAL marker_line IS STR_PAD(vr_label, UI_W).
  LOCAL v2_start IS 6 + v2_pos.
  IF v2_start + 3 <= UI_W {
    SET marker_line TO marker_line:SUBSTRING(0, v2_start) +
                       "▲V2" +
                       marker_line:SUBSTRING(v2_start + 3, UI_W - v2_start - 3).
  }
  UI_P(marker_line, UI_PRI_TOP + 2).

  // Sub-phase label
  LOCAL sub_str IS IFC_SUBPHASE.
  IF IFC_SUBPHASE = SUBPHASE_TO_PREFLIGHT   { SET sub_str TO "SPOOL UP    ". }
  IF IFC_SUBPHASE = SUBPHASE_TO_GROUND_ROLL { SET sub_str TO "GROUND ROLL ". }
  IF IFC_SUBPHASE = SUBPHASE_TO_ROTATE      { SET sub_str TO "ROTATE      ". }
  IF IFC_SUBPHASE = SUBPHASE_TO_CLIMB       { SET sub_str TO "CLIMB OUT   ". }
  UI_P("  SUB-PHASE   " + sub_str, UI_PRI_TOP + 3).

  // Yaw assist + centerline
  LOCAL yaw_bar IS UI_BAR_FILL(ABS(TELEM_RO_YAW_TGT) / 0.65, 10, "▐", "░").
  LOCAL loc_m IS ROUND(_TO_COMPUTE_LOC_DEV_M(), 1).
  UI_P("  YAW  " + yaw_bar + " " + UI_FMT(TELEM_RO_YAW_TGT, 3) +
       "   CLINE " + STR_RJUST("" + loc_m, 5) + " m", UI_PRI_TOP + 4).

  // Thrust + pitch cmd
  LOCAL avail IS ROUND(SHIP:AVAILABLETHRUST, 0).
  LOCAL maxth IS ROUND(SHIP:MAXTHRUST, 0).
  LOCAL thr_pct IS "---".
  IF maxth > 0 { SET thr_pct TO "" + ROUND(avail / maxth * 100, 1) + "%". }
  UI_P("  THRUST  " + avail + " / " + maxth + " kN  " + thr_pct +
       "   PITCH " + ROUND(TELEM_RO_PITCH_TGT, 1) + " deg", UI_PRI_TOP + 5).
}

// COMPLETE ────────────────────────────────────────────────
FUNCTION DISPLAY_COMPLETE {
  LOCAL elapsed IS UI_FORMAT_TIME(TIME:SECONDS - IFC_MISSION_START_UT).
  UI_CLR(UI_PRI_TOP).
  UI_P("  ╔══════════════════════════════════════╗", UI_PRI_TOP + 1).
  UI_P("  ║  FLIGHT COMPLETE                     ║", UI_PRI_TOP + 2).
  UI_P("  ╠══════════════════════════════════════╣", UI_PRI_TOP + 3).
  UI_P("  ║  TOTAL TIME  T+ " + STR_PAD(elapsed, 22) + "║", UI_PRI_TOP + 4).
  UI_P("  ╚══════════════════════════════════════╝", UI_PRI_TOP + 5).
}

// ── Secondary / debug zone (rows 12-14) ───────────────────

FUNCTION DISPLAY_SECONDARY {
  IF IFC_PHASE = PHASE_APPROACH {
    LOCAL arm_left IS 0.
    IF APP_FINAL_ARM_UT >= 0 {
      SET arm_left TO MAX(APP_FINAL_CAPTURE_CONFIRM_S - (TIME:SECONDS - APP_FINAL_ARM_UT), 0).
    }
    UI_P("  SPD " + APP_SPD_MODE +
         "  Vtgt " + ROUND(ACTIVE_V_TGT, 1) +
         "  Vbase " + ROUND(APP_BASE_V_TGT, 1) +
         "  Vint " + ROUND(APP_VINT_TGT, 1) +
         "  ThrI " + ROUND(THR_INTEGRAL, 3), UI_SEC_TOP).
    UI_P("  CAP LOC " + APP_LOC_CAP_OK + "  GS " + APP_GS_CAP_OK +
         "  arm " + ROUND(arm_left, 2) + "s" +
         "  SF " + ROUND(APP_SHORT_FINAL_FRAC, 2), UI_SEC_TOP + 1).
    UI_P("  AA  hdg " + ROUND(TELEM_AA_HDG_CMD, 1) +
         "  fpa " + ROUND(TELEM_AA_FPA_CMD, 2) +
         "  locC " + ROUND(TELEM_LOC_CORR, 3) +
         "  gsC " + ROUND(TELEM_GS_CORR, 3), UI_SEC_TOP + 2).

  } ELSE IF IFC_PHASE = PHASE_FLARE {
    UI_P("  tgtVS " + ROUND(TELEM_FLARE_TGT_VS, 3) +
         "  VS " + ROUND(SHIP:VERTICALSPEED, 2) +
         "  frac " + ROUND(TELEM_FLARE_FRAC, 3) +
         "  FPAcmd " + ROUND(FLARE_PITCH_CMD, 3), UI_SEC_TOP).
    UI_P("  AoA " + ROUND(GET_AOA(), 2) +
         "  pitch " + ROUND(GET_PITCH(), 2) +
         "  IAS " + ROUND(GET_IAS(), 1) +
         "  AGL " + ROUND(GET_AGL(), 1), UI_SEC_TOP + 1).
    UI_P("  AA  hdg " + ROUND(TELEM_AA_HDG_CMD, 1) +
         "  fpa " + ROUND(TELEM_AA_FPA_CMD, 3), UI_SEC_TOP + 2).

  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN OR IFC_PHASE = PHASE_ROLLOUT {
    UI_P("  yawTgt " + ROUND(TELEM_RO_YAW_TGT, 4) +
         "  cmd " + ROUND(SHIP:CONTROL:YAW, 4) +
         "  sc " + ROUND(TELEM_RO_YAW_SCALE, 3) +
         "  gate " + ROUND(TELEM_RO_YAW_GATE, 0) +
         "  hErr " + ROUND(TELEM_RO_HDG_ERR, 2), UI_SEC_TOP).
    UI_P("  pTgt " + ROUND(TELEM_RO_PITCH_TGT, 2) +
         "  pErr " + ROUND(TELEM_RO_PITCH_ERR, 2) +
         "  ff " + ROUND(TELEM_RO_PITCH_FF, 3) +
         "  cmd " + ROUND(SHIP:CONTROL:PITCH, 3) +
         "  blend " + ROUND(TELEM_STEER_BLEND, 2), UI_SEC_TOP + 1).
    UI_P("  steerHdg " + ROUND(ROLLOUT_STEER_HDG, 1) +
         "  locC " + ROUND(TELEM_RO_LOC_CORR, 3) +
         "  rollAsst " + ROUND(TELEM_RO_ROLL_ASSIST, 0), UI_SEC_TOP + 2).

  } ELSE IF IFC_PHASE = PHASE_TAKEOFF {
    LOCAL v_r IS AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001).
    LOCAL v2  IS AC_PARAM("v2",  TAKEOFF_V2_DEFAULT,  0.001).
    UI_P("  sub " + IFC_SUBPHASE +
         "  Vtgt " + ROUND(ACTIVE_V_TGT, 1) +
         "  VR " + ROUND(v_r, 1) +
         "  V2 " + ROUND(v2, 1), UI_SEC_TOP).
    UI_P("  hdg " + ROUND(TO_RWY_HDG, 1) +
         "  steer " + ROUND(ROLLOUT_STEER_HDG, 1) +
         "  locC " + ROUND(TELEM_RO_LOC_CORR, 3) +
         "  pCmd " + ROUND(SHIP:CONTROL:PITCH, 3) +
         "  AGL " + ROUND(GET_AGL(), 1), UI_SEC_TOP + 1).
    UI_P("  yawTgt " + ROUND(TELEM_RO_YAW_TGT, 4) +
         "  sc " + ROUND(TELEM_RO_YAW_SCALE, 3) +
         "  gate " + ROUND(TELEM_RO_YAW_GATE, 0) +
         "  THR " + ROUND(THROTTLE_CMD, 3), UI_SEC_TOP + 2).
  } ELSE {
    UI_CLR(UI_SEC_TOP).
    UI_CLR(UI_SEC_TOP + 1).
    UI_CLR(UI_SEC_TOP + 2).
  }
}

// ── Alert bar (row 16) ────────────────────────────────────

FUNCTION DISPLAY_ALERT_BAR {
  LOCAL now IS TIME:SECONDS.
  IF IFC_ALERT_UT < 0 OR (now - IFC_ALERT_UT) > IFC_ALERT_EXPIRE_S {
    // Alert expired — clear bar
    IF IFC_ALERT_TEXT <> "" {
      SET IFC_ALERT_TEXT TO "".
      UI_CLR(UI_ALERT_ROW).
    }
  } ELSE {
    UI_P("  " + IFC_ALERT_TEXT, UI_ALERT_ROW).
  }
}

// ── Logger status bar (row 18) ────────────────────────────

FUNCTION DISPLAY_LOGGER_BAR {
  LOCAL state_str IS "inactive".
  IF LOG_ACTIVE { SET state_str TO "ACTIVE". }
  LOCAL file_str IS "---".
  IF LOG_FILE <> "" {
    // Show just the filename, not the full path
    LOCAL slash IS LOG_FILE:FIND("/").
    LOCAL f IS LOG_FILE.
    UNTIL slash < 0 {
      SET f TO f:SUBSTRING(slash + 1, f:LENGTH - slash - 1).
      SET slash TO f:FIND("/").
    }
    SET file_str TO f.
  }
  UI_P("  LOG  " + STR_PAD(file_str, 30) + "  " + state_str, UI_LOG_ROW).
}

// ── Key hints bar (row 20) ────────────────────────────────

FUNCTION DISPLAY_KEY_HINTS {
  LOCAL hints IS "  [M]Menu  [D]Debug  [G]Gear  [F/f]Flap  [S]Spoil  [L]Log  [Q]Quit".
  IF IFC_PHASE = PHASE_ROLLOUT OR IFC_PHASE = PHASE_TOUCHDOWN {
    SET hints TO "  [M]Menu  [D]Debug  [B]Brakes  [L]Log  [Q]Quit".
  } ELSE IF IFC_PHASE = PHASE_DONE {
    SET hints TO "  [Q]Exit  [L]Toggle log".
  }
  UI_P(hints, UI_HINT_ROW).
}

// ── Master tick (call from main loop) ─────────────────────

FUNCTION DISPLAY_TICK {
  LOCAL now IS TIME:SECONDS.

  // Header + breadcrumb (1 Hz)
  IF (now - LAST_HEADER_UT) >= IFC_HEADER_PERIOD {
    SET LAST_HEADER_UT TO now.
    DISPLAY_HEADER().
    DISPLAY_BREADCRUMB().
    DISPLAY_KEY_HINTS().
  }

  // Alert bar (every tick — checks expiry internally)
  DISPLAY_ALERT_BAR().

  // Logger bar (0.5 Hz)
  IF (now - LAST_LOGGER_UT) >= IFC_LOGGER_PERIOD {
    SET LAST_LOGGER_UT TO now.
    DISPLAY_LOGGER_BAR().
  }

  // Primary zone: skip if menu is overlaying it (2 Hz)
  IF IFC_MENU_OPEN { RETURN. }
  IF (now - LAST_DISPLAY_UT) < IFC_DISPLAY_PERIOD { RETURN. }
  SET LAST_DISPLAY_UT TO now.

  IF IFC_PHASE = PHASE_APPROACH {
    IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX  { DISPLAY_FLY_TO_FIX().  }
    IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK   { DISPLAY_ILS_TRACK().   }
  } ELSE IF IFC_PHASE = PHASE_FLARE {
    DISPLAY_FLARE().
  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN {
    DISPLAY_TOUCHDOWN().
  } ELSE IF IFC_PHASE = PHASE_ROLLOUT {
    DISPLAY_ROLLOUT().
  } ELSE IF IFC_PHASE = PHASE_TAKEOFF {
    DISPLAY_TAKEOFF().
  } ELSE IF IFC_PHASE = PHASE_DONE {
    DISPLAY_COMPLETE().
  }

  // Secondary debug panel (1 Hz)
  IF IFC_DEBUG_PANEL_ON {
    IF (now - LAST_SECONDARY_UT) >= IFC_SECONDARY_PERIOD {
      SET LAST_SECONDARY_UT TO now.
      DISPLAY_SECONDARY().
    }
  } ELSE {
    // Ensure secondary zone is cleared when debug is off
    IF LAST_SECONDARY_UT <> 0 {
      SET LAST_SECONDARY_UT TO 0.
      UI_CLR(UI_SEC_TOP).
      UI_CLR(UI_SEC_TOP + 1).
      UI_CLR(UI_SEC_TOP + 2).
    }
  }
}
