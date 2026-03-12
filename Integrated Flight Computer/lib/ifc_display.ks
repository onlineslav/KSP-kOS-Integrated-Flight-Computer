@LAZYGLOBAL OFF.

// ============================================================
// ifc_display.ks  -  Integrated Flight Computer
// Mode-based terminal rendering.
// ============================================================

FUNCTION DISPLAY_ALERT {
  PARAMETER text.
  IFC_SET_ALERT(text).
}

FUNCTION DISPLAY_HEADER {
  LOCAL ac_name IS "---".
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("name") {
    SET ac_name TO ACTIVE_AIRCRAFT["name"].
  }

  LOCAL t_str IS "T+" + UI_FORMAT_TIME(TIME:SECONDS - IFC_MISSION_START_UT).
  LOCAL mode_str IS IFC_UI_MODE.
  IF IFC_MANUAL_MODE { SET mode_str TO "MANUAL". }
  IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY { SET mode_str TO "MENU". }
  IF IFC_PHASE = PHASE_DONE { SET mode_str TO "DONE". }

  LOCAL left IS "IFC 2.1  " + ac_name.
  LOCAL right IS t_str + "  [" + mode_str + "]".
  LOCAL gap IS MAX(UI_W - left:LENGTH - right:LENGTH, 1).
  UI_P(left + STR_REPEAT(" ", gap) + right, UI_HDR_ROW).
}

FUNCTION DISPLAY_BREADCRUMB {
  LOCAL phase_str IS IFC_PHASE.
  IF IFC_SUBPHASE <> "" { SET phase_str TO IFC_PHASE + " > " + IFC_SUBPHASE. }

  LOCAL rwy_str IS "".
  IF ACTIVE_ILS_ID <> "" {
    SET rwy_str TO ACTIVE_ILS_ID + "  GS " + ROUND(ACTIVE_GS_ANGLE, 1) + " deg".
  }

  LOCAL gap IS MAX(UI_W - phase_str:LENGTH - rwy_str:LENGTH, 1).
  UI_P(phase_str + STR_REPEAT(" ", gap) + rwy_str, UI_CRUMB_ROW).
}

FUNCTION _DISPLAY_AIR_DATA {
  LOCAL ias   IS ROUND(GET_IAS(), 1).
  LOCAL agl   IS ROUND(GET_AGL(), 0).
  LOCAL vs    IS ROUND(SHIP:VERTICALSPEED, 1).
  LOCAL hdg   IS ROUND(GET_COMPASS_HDG(), 1).
  LOCAL thr   IS ROUND(THROTTLE_CMD, 2).
  LOCAL flp   IS UI_FORMAT_FLP(FLAPS_CURRENT_DETENT).
  LOCAL pitch_deg IS ROUND(GET_PITCH(), 1).

  UI_P("  IAS " + STR_RJUST("" + ias, 6) + " m/s   AGL " +
       STR_RJUST("" + agl, 5) + " m    VS " +
       STR_RJUST("" + vs, 6) + " m/s", UI_PRI_TOP).

  UI_P("  HDG " + STR_RJUST("" + hdg, 5) + " deg   THR " +
       STR_RJUST("" + thr, 4) + "   FLP " + STR_PAD(flp, 10) +
       " PCH " + pitch_deg + " deg", UI_PRI_TOP + 1).
}

FUNCTION _DISPLAY_PREARM_CHECKLIST {
  LOCAL cfg_ok IS ACTIVE_AIRCRAFT <> 0.
  LOCAL aa_ok IS ADDONS:AVAILABLE("AA").
  LOCAL far_ok IS ADDONS:AVAILABLE("FAR").
  LOCAL val_msg IS MENU_VALIDATE_STAGE().
  LOCAL plan_ok IS val_msg = "".

  LOCAL c1 IS "[ ] Config loaded".
  LOCAL c2 IS "[ ] AA detected".
  LOCAL c3 IS "[ ] FAR detected".
  LOCAL c4 IS "[ ] Plan validation".

  IF cfg_ok { SET c1 TO "[x] Config loaded". }
  IF aa_ok { SET c2 TO "[x] AA detected". }
  IF far_ok { SET c3 TO "[x] FAR detected". }
  IF plan_ok { SET c4 TO "[x] Plan validation". }

  UI_P("  CHK " + c1 + "   " + c2, UI_PRI_TOP + 3).
  UI_P("      " + c3 + "   " + c4, UI_PRI_TOP + 4).

  IF NOT plan_ok {
    UI_P("  VALIDATION: " + val_msg, UI_PRI_TOP + 5).
  } ELSE {
    UI_P("  READY: configure in menu, then ARM.", UI_PRI_TOP + 5).
  }
}

FUNCTION DISPLAY_PREARM {
  PARAMETER proc_idx, rwy_idx, dist_idx, dest_idx.

  LOCAL proc_sel IS proc_idx.
  LOCAL rwy_sel IS rwy_idx.
  LOCAL dist_sel IS dist_idx.
  LOCAL dest_sel IS dest_idx.

  IF IFC_PHASE = PHASE_PREARM {
    SET proc_sel TO IFC_MENU_STG_PROC.
    SET rwy_sel TO IFC_MENU_STG_RWY.
    SET dist_sel TO IFC_MENU_STG_DIST.
    SET dest_sel TO IFC_MENU_STG_DEST.
  }

  LOCAL ac_name IS "Unknown".
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("name") {
    SET ac_name TO ACTIVE_AIRCRAFT["name"].
  }

  LOCAL proc_names IS LIST("ILS APPROACH", "TAKEOFF").
  LOCAL rwy_names  IS LIST("KSC RWY 09", "KSC RWY 27").
  LOCAL dist_names IS LIST("LONG 60 km", "SHORT 30 km").
  LOCAL dest_names IS LIST("KSC RETURN", "ISLAND").

  LOCAL vapp IS IFC_MENU_STG_VAPP.
  LOCAL vr IS IFC_MENU_STG_VR.
  LOCAL v2 IS IFC_MENU_STG_V2.
  IF vapp <= 0 { SET vapp TO ROUND(AC_PARAM("v_app", 75, 0.001), 0). }
  IF vr <= 0   { SET vr   TO ROUND(AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001), 0). }
  IF v2 <= 0   { SET v2   TO ROUND(AC_PARAM("v2", TAKEOFF_V2_DEFAULT, 0.001), 0). }

  UI_P("  AIRCRAFT  " + STR_PAD(ac_name, 20) + "MASS " + ROUND(SHIP:MASS, 1) + " t", UI_PRI_TOP).
  UI_P("  PROC " + proc_names[proc_sel] + "   " + rwy_names[rwy_sel], UI_PRI_TOP + 1).

  IF proc_sel = 0 {
    UI_P("  START " + dist_names[dist_sel] + "   VAPP " + vapp + " m/s", UI_PRI_TOP + 2).
  } ELSE {
    UI_P("  DEST " + dest_names[dest_sel] + "   VR " + vr + "   V2 " + v2 + " m/s", UI_PRI_TOP + 2).
  }

  _DISPLAY_PREARM_CHECKLIST().
}

FUNCTION DISPLAY_CRUISE {
  _DISPLAY_AIR_DATA().
  UI_CLR(UI_PRI_TOP + 2).

  IF CRUISE_WP_INDEX < CRUISE_WAYPOINTS:LENGTH {
    LOCAL wp_id IS CRUISE_WAYPOINTS[CRUISE_WP_INDEX].
    LOCAL wp IS GET_BEACON(wp_id).
    LOCAL dist_km IS ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, wp["ll"]) / 1000, 1).
    LOCAL brg IS ROUND(GEO_BEARING(SHIP:GEOPOSITION, wp["ll"]), 1).
    UI_P("  WPT " + (CRUISE_WP_INDEX + 1) + "/" + CRUISE_WAYPOINTS:LENGTH +
         "  " + STR_PAD(wp_id, 12) + " BRG " + brg + " deg  DIST " + dist_km + " km", UI_PRI_TOP + 3).
    UI_P("  ALT TGT " + ROUND(CRUISE_ALT_M, 0) + " m   SPD TGT " + ROUND(CRUISE_SPD_MPS, 0) + " m/s", UI_PRI_TOP + 4).
  } ELSE {
    UI_P("  Cruise route complete, awaiting next leg", UI_PRI_TOP + 3).
    UI_CLR(UI_PRI_TOP + 4).
  }

  UI_P("  CMD HDG " + ROUND(TELEM_AA_HDG_CMD, 1) + " deg   CMD FPA " + ROUND(TELEM_AA_FPA_CMD, 2) + " deg", UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_FLY_TO_FIX {
  _DISPLAY_AIR_DATA().
  UI_CLR(UI_PRI_TOP + 2).

  IF ACTIVE_FIXES:LENGTH > 0 AND FIX_INDEX < ACTIVE_FIXES:LENGTH {
    LOCAL fix_id IS ACTIVE_FIXES[FIX_INDEX].
    LOCAL fix IS GET_BEACON(fix_id).
    LOCAL dist_km IS ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, fix["ll"]) / 1000, 1).
    LOCAL brg IS ROUND(GEO_BEARING(SHIP:GEOPOSITION, fix["ll"]), 1).
    LOCAL tgt_alt IS 0.
    IF ACTIVE_ALT_AT:HASKEY(fix_id) { SET tgt_alt TO ACTIVE_ALT_AT[fix_id]. }

    UI_P("  FIX " + STR_PAD(fix_id, 16) + " BRG " + brg + " deg   DIST " + dist_km + " km", UI_PRI_TOP + 3).
    UI_P("  ALT TARGET " + tgt_alt + " m   SPD MODE " + APP_SPD_MODE, UI_PRI_TOP + 4).
  } ELSE {
    UI_P("  No active fix", UI_PRI_TOP + 3).
    UI_CLR(UI_PRI_TOP + 4).
  }

  UI_P("  Vtgt " + ROUND(ACTIVE_V_TGT, 1) + "  Vapp " + ROUND(ACTIVE_V_APP, 1) + "  ThrI " + ROUND(THR_INTEGRAL, 3), UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_ILS_TRACK {
  _DISPLAY_AIR_DATA().

  LOCAL loc_bar IS UI_BAR_DEV(ILS_LOC_DEV, 200, 18).
  LOCAL gs_bar  IS UI_BAR_DEV(ILS_GS_DEV, 80, 18).

  UI_P("  LOC " + loc_bar + " " + STR_RJUST(UI_FMT(ILS_LOC_DEV, 1), 7) + " m", UI_PRI_TOP + 2).
  UI_P("  GS  " + gs_bar + " " + STR_RJUST(UI_FMT(ILS_GS_DEV, 1), 7) + " m  DIST " + ROUND(ILS_DIST_M / 1000, 2) + " km", UI_PRI_TOP + 3).
  UI_P("  SPD " + STR_PAD(APP_SPD_MODE, 10) + " Vtgt " + ROUND(ACTIVE_V_TGT, 1) + "  Vapp " + ROUND(ACTIVE_V_APP, 1), UI_PRI_TOP + 4).

  LOCAL gear_str IS "UP".
  IF GEAR { SET gear_str TO "DOWN". }
  LOCAL sp_str IS "DISARMED".
  IF APP_SPOILERS_ARMED { SET sp_str TO "ARMED". }
  UI_P("  GEAR " + gear_str + "  SPOILERS " + sp_str, UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_FLARE {
  LOCAL agl IS ROUND(GET_AGL(), 1).
  LOCAL vs  IS ROUND(SHIP:VERTICALSPEED, 2).
  LOCAL bar IS UI_BAR_FILL(CLAMP(TELEM_FLARE_FRAC, 0, 1), 24, "#", ".").

  _DISPLAY_AIR_DATA().
  UI_P("  FLARE VS " + vs + " m/s   TGT " + ROUND(TELEM_FLARE_TGT_VS, 2) + " m/s", UI_PRI_TOP + 2).
  UI_P("  AGL " + agl + " m   AOA " + ROUND(GET_AOA(), 1) + " deg", UI_PRI_TOP + 3).
  UI_P("  PROGRESS [" + bar + "]", UI_PRI_TOP + 4).
  UI_P("  FPA CMD " + ROUND(FLARE_PITCH_CMD, 2) + " deg", UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_TOUCHDOWN {
  _DISPLAY_AIR_DATA().
  UI_P("  TOUCHDOWN CONFIRMED", UI_PRI_TOP + 2).
  UI_P("  Settling gear loads...", UI_PRI_TOP + 3).
  UI_CLR(UI_PRI_TOP + 4).
  UI_CLR(UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_ROLLOUT {
  _DISPLAY_AIR_DATA().
  LOCAL hdg_bar IS UI_BAR_DEV(TELEM_RO_HDG_ERR, 15, 18).
  LOCAL yaw_bar IS UI_BAR_FILL(ABS(TELEM_RO_YAW_TGT) / MAX(ROLLOUT_YAW_MAX_CMD, 0.001), 12, "#", ".").

  UI_P("  HDG ERR " + hdg_bar + " " + ROUND(TELEM_RO_HDG_ERR, 1) + " deg", UI_PRI_TOP + 2).
  UI_P("  YAW CMD [" + yaw_bar + "] " + ROUND(TELEM_RO_YAW_TGT, 3), UI_PRI_TOP + 3).
  UI_P("  PITCH TGT " + ROUND(TELEM_RO_PITCH_TGT, 2) + " deg   BRAKES " + BRAKES, UI_PRI_TOP + 4).
  UI_P("  STEER HDG " + ROUND(ROLLOUT_STEER_HDG, 1) + " deg", UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_TAKEOFF {
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL v_r IS AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001).
  LOCAL v2  IS AC_PARAM("v2", TAKEOFF_V2_DEFAULT, 0.001).
  LOCAL tape_max IS MAX(v2 + 20, 40).

  _DISPLAY_AIR_DATA().

  LOCAL tape IS UI_BAR_FILL(ias / tape_max, 24, "#", ".").
  UI_P("  IAS TAPE [" + tape + "] " + ias + " m/s", UI_PRI_TOP + 2).
  UI_P("  VR " + ROUND(v_r, 0) + "   V2 " + ROUND(v2, 0) + "   SUB " + IFC_SUBPHASE, UI_PRI_TOP + 3).
  UI_P("  CLINE " + ROUND(_TO_COMPUTE_LOC_DEV_M(), 1) + " m   YAW " + ROUND(SHIP:CONTROL:YAW, 3), UI_PRI_TOP + 4).
  UI_P("  PITCH CMD " + ROUND(SHIP:CONTROL:PITCH, 3) + "   THRUST " + ROUND(SHIP:AVAILABLETHRUST, 0) + " kN", UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_COMPLETE {
  LOCAL elapsed IS UI_FORMAT_TIME(TIME:SECONDS - IFC_MISSION_START_UT).
  UI_CLR(UI_PRI_TOP).
  UI_P("  +----------------------------------------------+", UI_PRI_TOP + 1).
  UI_P("  |                FLIGHT COMPLETE                |", UI_PRI_TOP + 2).
  UI_P("  |   TOTAL TIME  T+ " + STR_PAD(elapsed, 24) + "|", UI_PRI_TOP + 3).
  UI_P("  |   Events logged: " + STR_PAD("" + IFC_EVENT_QUEUE:LENGTH, 24) + "|", UI_PRI_TOP + 4).
  UI_P("  +----------------------------------------------+", UI_PRI_TOP + 5).
}

FUNCTION DISPLAY_SECONDARY_DEBUG {
  IF IFC_PHASE = PHASE_APPROACH {
    LOCAL arm_left IS 0.
    IF APP_FINAL_ARM_UT >= 0 {
      SET arm_left TO MAX(APP_FINAL_CAPTURE_CONFIRM_S - (TIME:SECONDS - APP_FINAL_ARM_UT), 0).
    }

    UI_P("  SPD " + APP_SPD_MODE + "  Vtgt " + ROUND(ACTIVE_V_TGT, 1) + "  Vbase " + ROUND(APP_BASE_V_TGT, 1) + "  Vint " + ROUND(APP_VINT_TGT, 1), UI_SEC_TOP).
    UI_P("  CAP LOC " + APP_LOC_CAP_OK + "  GS " + APP_GS_CAP_OK + "  ARM " + ROUND(arm_left, 2) + "s  SF " + ROUND(APP_SHORT_FINAL_FRAC, 2), UI_SEC_TOP + 1).
    UI_P("  CMD hdg " + ROUND(TELEM_AA_HDG_CMD, 1) + "  fpa " + ROUND(TELEM_AA_FPA_CMD, 2) + "  locC " + ROUND(TELEM_LOC_CORR, 2) + "  gsC " + ROUND(TELEM_GS_CORR, 2), UI_SEC_TOP + 2).
    RETURN.
  }

  IF IFC_PHASE = PHASE_TAKEOFF {
    UI_P("  RWY HDG " + ROUND(TO_RWY_HDG, 1) + "  steer " + ROUND(ROLLOUT_STEER_HDG, 1) + "  locC " + ROUND(TELEM_RO_LOC_CORR, 3), UI_SEC_TOP).
    UI_P("  yawTgt " + ROUND(TELEM_RO_YAW_TGT, 3) + "  scale " + ROUND(TELEM_RO_YAW_SCALE, 2) + "  gate " + ROUND(TELEM_RO_YAW_GATE, 0), UI_SEC_TOP + 1).
    UI_P("  climbFPAcmd " + ROUND(TO_CLIMB_FPA_CMD, 2) + "  ThrI " + ROUND(THR_INTEGRAL, 3), UI_SEC_TOP + 2).
    RETURN.
  }

  UI_P("  DEBUG: phase " + IFC_PHASE + "  sub " + IFC_SUBPHASE, UI_SEC_TOP).
  UI_P("  IAS " + ROUND(GET_IAS(), 1) + "  VS " + ROUND(SHIP:VERTICALSPEED, 1) + "  AGL " + ROUND(GET_AGL(), 1), UI_SEC_TOP + 1).
  UI_P("  CMD hdg " + ROUND(TELEM_AA_HDG_CMD, 1) + "  fpa " + ROUND(TELEM_AA_FPA_CMD, 2), UI_SEC_TOP + 2).
}

FUNCTION DISPLAY_QUICK_ACTIONS {
  LOCAL row0 IS "  QUICK: M menu  D debug  L log  Q quit".
  LOCAL row1 IS "  ACT:   G gear  F/f flaps  S spoilers  B brakes".
  LOCAL row2 IS "  MODE:  C manual/auto  T reverse  P drogue".

  IF IFC_PHASE = PHASE_ROLLOUT OR IFC_PHASE = PHASE_TOUCHDOWN {
    SET row1 TO "  ACT:   B brakes  S spoilers  G gear".
  }

  IF IFC_PHASE = PHASE_PREARM {
    SET row1 TO "  PREARM: W/S move  A/D change  Y execute  M close".
    SET row2 TO "  ARM requires valid plan settings".
  }

  UI_P(row0, UI_SEC_TOP).
  UI_P(row1, UI_SEC_TOP + 1).
  UI_P(row2, UI_SEC_TOP + 2).
}

FUNCTION DISPLAY_ALERT_BAR {
  IFC_SYNC_ALERT_QUEUE().

  LOCAL now IS TIME:SECONDS.
  LOCAL bar_txt IS "".
  IF IFC_ALERT_UT >= 0 AND (now - IFC_ALERT_UT) <= IFC_ALERT_EXPIRE_S {
    SET bar_txt TO "  " + IFC_ALERT_TEXT.
  } ELSE IF IFC_EVENT_QUEUE:LENGTH > 0 {
    LOCAL e IS IFC_EVENT_QUEUE[IFC_EVENT_QUEUE:LENGTH - 1].
    LOCAL t IS UI_FORMAT_TIME(e["ut"] - IFC_MISSION_START_UT).
    SET bar_txt TO "  LAST T+" + t + " [" + e["sev"] + "] " + e["msg"].
  }

  IF bar_txt = IFC_ALERT_LAST_LINE { RETURN. }

  IF bar_txt = "" {
    UI_CLR(UI_ALERT_ROW).
  } ELSE {
    UI_P(bar_txt, UI_ALERT_ROW).
  }
  SET IFC_ALERT_LAST_LINE TO bar_txt.
}

FUNCTION DISPLAY_LOGGER_BAR {
  LOCAL state_str IS "INACTIVE".
  IF LOG_ACTIVE { SET state_str TO "ACTIVE". }

  LOCAL file_str IS "---".
  IF LOG_FILE <> "" {
    LOCAL slash IS LOG_FILE:FIND("/").
    LOCAL f IS LOG_FILE.
    UNTIL slash < 0 {
      SET f TO f:SUBSTRING(slash + 1, f:LENGTH - slash - 1).
      SET slash TO f:FIND("/").
    }
    SET file_str TO f.
  }

  UI_P("  LOG " + STR_PAD(file_str, 32) + " " + state_str, UI_LOG_ROW).
}

FUNCTION DISPLAY_KEY_HINTS {
  LOCAL hints IS "  [M]Menu  [C]Manual  [D]Debug  [L]Log  [Q]Quit".

  IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY {
    SET hints TO "  [W/S]Move  [A/D]Change  [Y]Exec  [M]Close  [Q]Quit".
  } ELSE IF IFC_PHASE = PHASE_DONE {
    SET hints TO "  [M]Menu  [L]Log toggle  [Q]Exit".
  }

  UI_P(hints, UI_HINT_ROW).
}

FUNCTION _DISPLAY_PRIMARY_BY_PHASE {
  IF IFC_PHASE = PHASE_PREARM {
    DISPLAY_PREARM(IFC_MENU_OPT_PROC, IFC_MENU_OPT_RWY, IFC_MENU_OPT_DIST, IFC_MENU_OPT_DEST).
    RETURN.
  }

  IF IFC_PHASE = PHASE_TAKEOFF { DISPLAY_TAKEOFF(). RETURN. }
  IF IFC_PHASE = PHASE_CRUISE { DISPLAY_CRUISE(). RETURN. }

  IF IFC_PHASE = PHASE_APPROACH {
    IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX { DISPLAY_FLY_TO_FIX(). RETURN. }
    DISPLAY_ILS_TRACK().
    RETURN.
  }

  IF IFC_PHASE = PHASE_FLARE { DISPLAY_FLARE(). RETURN. }
  IF IFC_PHASE = PHASE_TOUCHDOWN { DISPLAY_TOUCHDOWN(). RETURN. }
  IF IFC_PHASE = PHASE_ROLLOUT { DISPLAY_ROLLOUT(). RETURN. }

  IF IFC_PHASE = PHASE_ASCENT {
    _DISPLAY_AIR_DATA().
    UI_P("  ASCENT phase scaffold active", UI_PRI_TOP + 3).
    UI_CLR(UI_PRI_TOP + 4).
    UI_CLR(UI_PRI_TOP + 5).
    RETURN.
  }

  IF IFC_PHASE = PHASE_REENTRY {
    _DISPLAY_AIR_DATA().
    UI_P("  REENTRY phase scaffold active", UI_PRI_TOP + 3).
    UI_CLR(UI_PRI_TOP + 4).
    UI_CLR(UI_PRI_TOP + 5).
    RETURN.
  }

  DISPLAY_COMPLETE().
}

FUNCTION DISPLAY_TICK {
  IFC_SYNC_ALERT_QUEUE().

  // While menu overlay is open, prioritize menu responsiveness and avoid
  // background repaint churn from header/logger updates.
  IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY {
    IF IFC_MENU_DIRTY { MENU_RENDER(). }
    RETURN.
  }

  LOCAL now IS TIME:SECONDS.

  IF (now - LAST_HEADER_UT) >= IFC_HEADER_PERIOD {
    SET LAST_HEADER_UT TO now.
    DISPLAY_HEADER().
    DISPLAY_BREADCRUMB().
    DISPLAY_KEY_HINTS().
  }

  DISPLAY_ALERT_BAR().

  IF (now - LAST_LOGGER_UT) >= IFC_LOGGER_PERIOD {
    SET LAST_LOGGER_UT TO now.
    DISPLAY_LOGGER_BAR().
  }

  IF (now - LAST_DISPLAY_UT) >= IFC_DISPLAY_PERIOD {
    SET LAST_DISPLAY_UT TO now.
    _DISPLAY_PRIMARY_BY_PHASE().
  }

  IF IFC_DEBUG_PANEL_ON {
    IF (now - LAST_SECONDARY_UT) >= IFC_SECONDARY_PERIOD {
      SET LAST_SECONDARY_UT TO now.
      DISPLAY_SECONDARY_DEBUG().
    }
  } ELSE {
    DISPLAY_QUICK_ACTIONS().
    SET LAST_SECONDARY_UT TO 0.
  }
}
