@LAZYGLOBAL OFF.

// ============================================================
// ifc_menu.ks  -  Integrated Flight Computer
// FMS menu navigation state and keyboard input handler.
//
// MENU_TICK() is called once per main loop cycle (non-blocking).
// In-flight live keys are handled here too.
//
// Menu navigation (when menu is open):
//   W / S     = cursor up / down
//   A / D     = cycle value left / right for [value] items
//   Y / Enter = execute selected item
//   M / X     = close menu without action
//
// Live keys (when menu is closed, in-flight):
//   M         = open menu
//   C         = toggle manual override (CWS)
//   D         = toggle debug panel
//   G         = toggle gear
//   F / f     = flap step up / down
//   S         = toggle spoilers
//   T         = toggle thrust reversers
//   P         = toggle drogue chute
//   B         = toggle wheel brakes
//   L         = start / stop logger
//   Q         = abort IFC (returns "QUIT")
// ============================================================

// ----------------------------
// Persistent selection state
// (used by startup screen and retained in-flight for re-arm)
// ----------------------------
GLOBAL IFC_MENU_OPT_PROC IS 0.  // 0 = ILS Approach, 1 = Takeoff
GLOBAL IFC_MENU_OPT_RWY  IS 0.  // 0 = RWY 09,       1 = RWY 27
GLOBAL IFC_MENU_OPT_DIST IS 0.  // 0 = Long 60 km,   1 = Short 30 km  (approach only)
GLOBAL IFC_MENU_OPT_DEST IS 0.  // 0 = KSC return,   1 = Island        (takeoff only)
GLOBAL IFC_MENU_OPT_VR   IS 0.  // 0 = use config default, else override m/s
GLOBAL IFC_MENU_OPT_V2   IS 0.  // 0 = use config default, else override m/s
GLOBAL IFC_MENU_OPT_VAPP IS 0.  // 0 = use config default, else override m/s

// Menu cursor position (0-based index into item list).
GLOBAL IFC_MENU_CURSOR IS 0.

LOCAL _MENU_NITEMS IS 12.

// V-speed step size used by A/D keys
LOCAL _VSpd_STEP IS 5.  // m/s

// ----------------------------
// Action-group helpers
// (called by both live keys and menu execute)
// ----------------------------

FUNCTION MENU_DO_FLAP_UP {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_up") { SET ag TO ACTIVE_AIRCRAFT["ag_flaps_step_up"]. }
  IF ag <= 0 { RETURN. }
  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).
  IF FLAPS_CURRENT_DETENT >= max_det { RETURN. }
  PULSE_AG(ag).
  SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
  SET FLAPS_TARGET_DETENT  TO FLAPS_CURRENT_DETENT.
  SET FLAPS_LAST_STEP_UT   TO TIME:SECONDS.
  SET IFC_ALERT_TEXT TO "FLAP UP -> detent " + FLAPS_CURRENT_DETENT.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_FLAP_DN {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_down") { SET ag TO ACTIVE_AIRCRAFT["ag_flaps_step_down"]. }
  IF ag <= 0 { RETURN. }
  LOCAL det_up IS ROUND(AC_PARAM("flaps_detent_up", 0, 0)).
  IF FLAPS_CURRENT_DETENT <= det_up { RETURN. }
  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).
  PULSE_AG(ag).
  SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
  SET FLAPS_TARGET_DETENT  TO FLAPS_CURRENT_DETENT.
  SET FLAPS_LAST_STEP_UT   TO TIME:SECONDS.
  SET IFC_ALERT_TEXT TO "FLAP DN -> detent " + FLAPS_CURRENT_DETENT.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_SPOILERS {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_spoilers") { SET ag TO ACTIVE_AIRCRAFT["ag_spoilers"]. }
  IF ag <= 0 { RETURN. }
  PULSE_AG(ag).
  SET IFC_ALERT_TEXT TO "SPOILERS toggled (AG" + ag + ")".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_GEAR {
  IF GEAR { GEAR OFF. } ELSE { GEAR ON. }
  SET IFC_ALERT_TEXT TO "GEAR toggled".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_THRUST_REV {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_thrust_rev") { SET ag TO ACTIVE_AIRCRAFT["ag_thrust_rev"]. }
  IF ag <= 0 { RETURN. }
  PULSE_AG(ag).
  SET IFC_ALERT_TEXT TO "THRUST REV (AG" + ag + ")".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_DROGUE {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_drogue") { SET ag TO ACTIVE_AIRCRAFT["ag_drogue"]. }
  IF ag <= 0 { RETURN. }
  PULSE_AG(ag).
  SET IFC_ALERT_TEXT TO "DROGUE toggled (AG" + ag + ")".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_BRAKES {
  IF BRAKES { BRAKES OFF. } ELSE { BRAKES ON. }
  SET IFC_ALERT_TEXT TO "BRAKES toggled".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

FUNCTION MENU_DO_LOGGER {
  IF LOG_ACTIVE { LOGGER_CLOSE(). } ELSE { LOGGER_INIT(). }
}

FUNCTION MENU_DO_MANUAL {
  SET IFC_MANUAL_MODE TO NOT IFC_MANUAL_MODE.
  IF IFC_MANUAL_MODE {
    AA_DISABLE_ALL().
    LOCK THROTTLE TO SHIP:THROTTLE.
    SET IFC_ALERT_TEXT TO "MANUAL OVERRIDE  —  autopilot suspended".
  } ELSE {
    LOCK THROTTLE TO THROTTLE_CMD.
    AA_INIT().
    SET IFC_ALERT_TEXT TO "AUTO  —  autopilot resumed".
  }
  SET IFC_ALERT_UT TO TIME:SECONDS.
}

// ----------------------------
// Menu render (overlay into primary + secondary zone)
// ----------------------------

FUNCTION MENU_RENDER {
  LOCAL proc_str IS "ILS APPROACH".
  IF IFC_MENU_OPT_PROC = 1 { SET proc_str TO "TAKEOFF     ". }
  LOCAL rwy_str IS "RWY 09".
  IF IFC_MENU_OPT_RWY = 1  { SET rwy_str  TO "RWY 27". }
  LOCAL dist_str IS "LONG  60 km ".
  IF IFC_MENU_OPT_DIST = 1 { SET dist_str TO "SHORT 30 km". }
  LOCAL dbg_str IS "OFF".
  IF IFC_DEBUG_PANEL_ON { SET dbg_str TO "ON ". }
  LOCAL log_str IS "STOPPED".
  IF LOG_ACTIVE { SET log_str TO "ACTIVE ". }

  LOCAL dist_item IS "DISTANCE   [" + dist_str + "]".
  IF IFC_MENU_OPT_PROC = 1 {
    LOCAL dest_label IS "KSC RTRN".
    IF IFC_MENU_OPT_DEST = 1 { SET dest_label TO "ISLAND  ". }
    SET dist_item TO "DESTINATION[" + dest_label + "]".
  }

  // V-speed display: show current override or "---" (= use config)
  LOCAL vr_str   IS "---".
  LOCAL v2_str   IS "---".
  LOCAL vapp_str IS "---".
  IF IFC_MENU_OPT_VR   > 0 { SET vr_str   TO "" + IFC_MENU_OPT_VR   + " m/s". }
  IF IFC_MENU_OPT_V2   > 0 { SET v2_str   TO "" + IFC_MENU_OPT_V2   + " m/s". }
  IF IFC_MENU_OPT_VAPP > 0 { SET vapp_str TO "" + IFC_MENU_OPT_VAPP + " m/s". }

  LOCAL items IS LIST(
    "PROCEDURE  [" + proc_str + "]",
    "RUNWAY     [" + rwy_str  + "]",
    dist_item,
    "VR         [" + STR_PAD(vr_str, 8)   + "]",
    "V2         [" + STR_PAD(v2_str, 8)   + "]",
    "VAPP       [" + STR_PAD(vapp_str, 8) + "]",
    "─── ARM ────────────────── [Y]",
    "DEBUG PANEL  [" + dbg_str + "]",
    "LOGGER       [" + log_str + "]",
    "SAVE PLAN ──────────────── [Y]",
    "LOAD PLAN ──────────────── [Y]",
    "─── ABORT IFC ───────────── [Q]"
  ).

  // Menu box: starts at UI_PRI_TOP
  LOCAL box_w IS MIN(UI_W - 4, 44).
  LOCAL border IS "  ╔" + STR_REPEAT("═", box_w) + "╗".
  UI_P(border, UI_PRI_TOP).

  LOCAL row IS UI_PRI_TOP + 1.
  LOCAL i IS 0.
  UNTIL i >= items:LENGTH {
    LOCAL cursor IS "  ║  ".
    IF i = IFC_MENU_CURSOR { SET cursor TO "  ║▶ ". }
    UI_P(cursor + STR_PAD(items[i], box_w - 1) + "║", row).
    SET row TO row + 1.
    SET i   TO i + 1.
  }

  UI_P("  ╚" + STR_REPEAT("═", box_w) + "╝", row).
  UI_P(STR_PAD("  W/S=navigate  A/D=change  Y=execute  M=close", UI_W), row + 1).

  // Clear any leftover lines in the primary zone below the box
  SET row TO row + 2.
  UNTIL row > UI_PRI_BOT {
    UI_CLR(row).
    SET row TO row + 1.
  }
}

// ----------------------------
// Main tick  (call once per loop cycle)
// ----------------------------
// Returns "" normally, "ARM" when user confirmed ARM in menu,
// or "QUIT" when user pressed Q / selected Abort.

FUNCTION MENU_TICK {
  IF NOT TERMINAL:INPUT:HASCHAR { RETURN "". }
  LOCAL ch IS TERMINAL:INPUT:GETCHAR().

  IF IFC_MENU_OPEN {
    // ── Menu navigation ──────────────────────────────────
    IF ch = "w" OR ch = "W" {
      SET IFC_MENU_CURSOR TO MOD(IFC_MENU_CURSOR - 1 + _MENU_NITEMS, _MENU_NITEMS).
      MENU_RENDER().

    } ELSE IF ch = "s" OR ch = "S" {
      SET IFC_MENU_CURSOR TO MOD(IFC_MENU_CURSOR + 1, _MENU_NITEMS).
      MENU_RENDER().

    } ELSE IF ch = "a" OR ch = "A" {
      IF IFC_MENU_CURSOR = 0 { SET IFC_MENU_OPT_PROC TO MOD(IFC_MENU_OPT_PROC - 1 + 2, 2). }
      IF IFC_MENU_CURSOR = 1 { SET IFC_MENU_OPT_RWY  TO MOD(IFC_MENU_OPT_RWY  - 1 + 2, 2). }
      IF IFC_MENU_CURSOR = 2 AND IFC_MENU_OPT_PROC = 0 { SET IFC_MENU_OPT_DIST TO MOD(IFC_MENU_OPT_DIST - 1 + 2, 2). }
      IF IFC_MENU_CURSOR = 2 AND IFC_MENU_OPT_PROC = 1 { SET IFC_MENU_OPT_DEST TO MOD(IFC_MENU_OPT_DEST - 1 + 2, 2). }
      IF IFC_MENU_CURSOR = 3 { SET IFC_MENU_OPT_VR   TO MAX(0, IFC_MENU_OPT_VR   - _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 4 { SET IFC_MENU_OPT_V2   TO MAX(0, IFC_MENU_OPT_V2   - _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 5 { SET IFC_MENU_OPT_VAPP TO MAX(0, IFC_MENU_OPT_VAPP - _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 7 { SET IFC_DEBUG_PANEL_ON TO NOT IFC_DEBUG_PANEL_ON. }
      IF IFC_MENU_CURSOR = 8 { MENU_DO_LOGGER(). }
      MENU_RENDER().

    } ELSE IF ch = "d" OR ch = "D" {
      IF IFC_MENU_CURSOR = 0 { SET IFC_MENU_OPT_PROC TO MOD(IFC_MENU_OPT_PROC + 1, 2). }
      IF IFC_MENU_CURSOR = 1 { SET IFC_MENU_OPT_RWY  TO MOD(IFC_MENU_OPT_RWY  + 1, 2). }
      IF IFC_MENU_CURSOR = 2 AND IFC_MENU_OPT_PROC = 0 { SET IFC_MENU_OPT_DIST TO MOD(IFC_MENU_OPT_DIST + 1, 2). }
      IF IFC_MENU_CURSOR = 2 AND IFC_MENU_OPT_PROC = 1 { SET IFC_MENU_OPT_DEST TO MOD(IFC_MENU_OPT_DEST + 1, 2). }
      IF IFC_MENU_CURSOR = 3 { SET IFC_MENU_OPT_VR   TO MAX(_VSpd_STEP, IFC_MENU_OPT_VR   + _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 4 { SET IFC_MENU_OPT_V2   TO MAX(_VSpd_STEP, IFC_MENU_OPT_V2   + _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 5 { SET IFC_MENU_OPT_VAPP TO MAX(_VSpd_STEP, IFC_MENU_OPT_VAPP + _VSpd_STEP). }
      IF IFC_MENU_CURSOR = 7 { SET IFC_DEBUG_PANEL_ON TO NOT IFC_DEBUG_PANEL_ON. }
      IF IFC_MENU_CURSOR = 8 { MENU_DO_LOGGER(). }
      MENU_RENDER().

    } ELSE IF ch = "y" OR ch = "Y" OR ch = CHAR(13) {
      SET IFC_MENU_OPEN     TO FALSE.
      SET LAST_SECONDARY_UT TO 1.
      SET LAST_DISPLAY_UT   TO 0.
      IF IFC_MENU_CURSOR = 6  { RETURN "ARM". }
      IF IFC_MENU_CURSOR = 9  { FMS_SAVE_PLAN(). }
      IF IFC_MENU_CURSOR = 10 { FMS_LOAD_PLAN(). MENU_RENDER(). SET IFC_MENU_OPEN TO TRUE. }
      IF IFC_MENU_CURSOR = 11 { RETURN "QUIT". }

    } ELSE IF ch = "m" OR ch = "M" OR ch = "x" OR ch = "X" {
      SET IFC_MENU_OPEN     TO FALSE.
      SET LAST_SECONDARY_UT TO 1.
      SET LAST_DISPLAY_UT   TO 0.
    }
    RETURN "".
  }

  // ── Live flight keys (menu closed) ──────────────────────
  IF      ch = "m" OR ch = "M" {
    SET IFC_MENU_OPEN TO TRUE.
    MENU_RENDER().
  } ELSE IF ch = "c" OR ch = "C" {
    MENU_DO_MANUAL().
  } ELSE IF ch = "d" OR ch = "D" {
    SET IFC_DEBUG_PANEL_ON TO NOT IFC_DEBUG_PANEL_ON.
    LOCAL s IS "OFF". IF IFC_DEBUG_PANEL_ON { SET s TO "ON". }
    SET IFC_ALERT_TEXT TO "Debug panel " + s.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  } ELSE IF ch = "g" OR ch = "G" {
    MENU_DO_GEAR().
  } ELSE IF ch = "F" {
    MENU_DO_FLAP_UP().
  } ELSE IF ch = "f" {
    MENU_DO_FLAP_DN().
  } ELSE IF ch = "s" OR ch = "S" {
    MENU_DO_SPOILERS().
  } ELSE IF ch = "t" OR ch = "T" {
    MENU_DO_THRUST_REV().
  } ELSE IF ch = "p" OR ch = "P" {
    MENU_DO_DROGUE().
  } ELSE IF ch = "b" OR ch = "B" {
    MENU_DO_BRAKES().
  } ELSE IF ch = "l" OR ch = "L" {
    MENU_DO_LOGGER().
  } ELSE IF ch = "q" OR ch = "Q" {
    RETURN "QUIT".
  }

  RETURN "".
}
