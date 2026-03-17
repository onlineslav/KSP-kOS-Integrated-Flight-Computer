@LAZYGLOBAL OFF.

// ============================================================
// ifc_menu.ks  -  Integrated Flight Computer
// Data-driven menu + command dispatcher.
// ============================================================

GLOBAL IFC_MENU_OPT_PROC IS 0.  // 0 = approach, 1 = takeoff
GLOBAL IFC_MENU_OPT_RWY  IS 0.  // 0 = 09, 1 = 27
GLOBAL IFC_MENU_OPT_DIST IS 0.  // 0 = long, 1 = short
GLOBAL IFC_MENU_OPT_DEST IS 0.  // 0 = KSC, 1 = island
GLOBAL IFC_MENU_OPT_VR   IS 0.  // 0 = config default
GLOBAL IFC_MENU_OPT_V2   IS 0.  // 0 = config default
GLOBAL IFC_MENU_OPT_VAPP IS 0.  // 0 = config default

GLOBAL IFC_MENU_STG_PROC IS 0.
GLOBAL IFC_MENU_STG_RWY  IS 0.
GLOBAL IFC_MENU_STG_DIST IS 0.
GLOBAL IFC_MENU_STG_DEST IS 0.
GLOBAL IFC_MENU_STG_VR   IS 0.
GLOBAL IFC_MENU_STG_V2   IS 0.
GLOBAL IFC_MENU_STG_VAPP IS 0.

GLOBAL IFC_MENU_SLOT   IS 1.
GLOBAL IFC_MENU_CURSOR IS 0.
GLOBAL IFC_MENU_SCROLL IS 0.
GLOBAL IFC_MENU_PAGE   IS "MAIN". // MAIN or EVENTS
GLOBAL IFC_MENU_ITEMS  IS LIST().

LOCAL _VSPD_STEP IS 5.

FUNCTION MENU_STAGE_FROM_LIVE {
  SET IFC_MENU_STG_PROC TO IFC_MENU_OPT_PROC.
  SET IFC_MENU_STG_RWY  TO IFC_MENU_OPT_RWY.
  SET IFC_MENU_STG_DIST TO IFC_MENU_OPT_DIST.
  SET IFC_MENU_STG_DEST TO IFC_MENU_OPT_DEST.
  SET IFC_MENU_STG_VR   TO IFC_MENU_OPT_VR.
  SET IFC_MENU_STG_V2   TO IFC_MENU_OPT_V2.
  SET IFC_MENU_STG_VAPP TO IFC_MENU_OPT_VAPP.
}

FUNCTION MENU_STAGE_COMMIT {
  SET IFC_MENU_OPT_PROC TO IFC_MENU_STG_PROC.
  SET IFC_MENU_OPT_RWY  TO IFC_MENU_STG_RWY.
  SET IFC_MENU_OPT_DIST TO IFC_MENU_STG_DIST.
  SET IFC_MENU_OPT_DEST TO IFC_MENU_STG_DEST.
  SET IFC_MENU_OPT_VR   TO IFC_MENU_STG_VR.
  SET IFC_MENU_OPT_V2   TO IFC_MENU_STG_V2.
  SET IFC_MENU_OPT_VAPP TO IFC_MENU_STG_VAPP.
}

FUNCTION MENU_VALIDATE_STAGE {
  IF DRAFT_PLAN:LENGTH = 0 { RETURN "Flight plan is empty". }
  IF IFC_MENU_STG_VAPP > 0 AND IFC_MENU_STG_VAPP < 35 { RETURN "VAPP too low". }
  IF IFC_MENU_STG_VR > 0 AND IFC_MENU_STG_V2 > 0 AND IFC_MENU_STG_VR >= IFC_MENU_STG_V2 {
    RETURN "VR must be below V2".
  }
  RETURN "".
}

FUNCTION _MENU_GET_VALUE {
  PARAMETER key.

  IF key = "proc" { RETURN IFC_MENU_STG_PROC. }
  IF key = "rwy"  { RETURN IFC_MENU_STG_RWY. }
  IF key = "dist" { RETURN IFC_MENU_STG_DIST. }
  IF key = "dest" { RETURN IFC_MENU_STG_DEST. }
  IF key = "vr"   { RETURN IFC_MENU_STG_VR. }
  IF key = "v2"   { RETURN IFC_MENU_STG_V2. }
  IF key = "vapp" { RETURN IFC_MENU_STG_VAPP. }
  IF key = "slot" { RETURN IFC_MENU_SLOT. }

  IF key = "debug" {
    IF IFC_DEBUG_PANEL_ON { RETURN 1. }
    RETURN 0.
  }
  IF key = "logger" {
    IF LOG_ACTIVE { RETURN 1. }
    RETURN 0.
  }
  IF key = "manual" {
    IF IFC_MANUAL_MODE { RETURN 1. }
    RETURN 0.
  }
  RETURN 0.
}

FUNCTION _MENU_SET_VALUE {
  PARAMETER key, val.

  IF key = "proc" {
    SET IFC_MENU_STG_PROC TO MOD(MAX(ROUND(val, 0), 0), 2).
    RETURN.
  }
  IF key = "rwy" {
    SET IFC_MENU_STG_RWY TO MOD(MAX(ROUND(val, 0), 0), 2).
    RETURN.
  }
  IF key = "dist" {
    SET IFC_MENU_STG_DIST TO MOD(MAX(ROUND(val, 0), 0), 2).
    RETURN.
  }
  IF key = "dest" {
    SET IFC_MENU_STG_DEST TO MOD(MAX(ROUND(val, 0), 0), 2).
    RETURN.
  }
  IF key = "vr" {
    SET IFC_MENU_STG_VR TO MAX(0, ROUND(val, 0)).
    RETURN.
  }
  IF key = "v2" {
    SET IFC_MENU_STG_V2 TO MAX(0, ROUND(val, 0)).
    RETURN.
  }
  IF key = "vapp" {
    SET IFC_MENU_STG_VAPP TO MAX(0, ROUND(val, 0)).
    RETURN.
  }
  IF key = "slot" {
    SET IFC_MENU_SLOT TO CLAMP(ROUND(val, 0), FMS_SLOT_MIN, FMS_SLOT_MAX).
    RETURN.
  }

  IF key = "debug" {
    SET IFC_DEBUG_PANEL_ON TO val > 0.
    LOCAL s IS "OFF".
    IF IFC_DEBUG_PANEL_ON { SET s TO "ON". }
    IFC_SET_ALERT("Debug panel " + s).
    RETURN.
  }

  IF key = "logger" {
    IF val > 0 {
      IF NOT LOG_ACTIVE { LOGGER_INIT(). }
    } ELSE {
      IF LOG_ACTIVE { LOGGER_CLOSE(). }
    }
    RETURN.
  }

  IF key = "manual" {
    IF val > 0 AND NOT IFC_MANUAL_MODE { MENU_DO_MANUAL(). }
    IF val <= 0 AND IFC_MANUAL_MODE { MENU_DO_MANUAL(). }
    RETURN.
  }
}

FUNCTION _MENU_ADD_ITEM {
  PARAMETER item.
  IFC_MENU_ITEMS:ADD(item).
}

FUNCTION MENU_BUILD_ITEMS {
  SET IFC_MENU_ITEMS TO LIST().

  IF IFC_PHASE = PHASE_PREARM {
    LOCAL ac_name IS "Unknown".
    IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("name") {
      SET ac_name TO ACTIVE_AIRCRAFT["name"].
    }
    LOCAL aa_str IS "NO".  IF AA_AVAILABLE  { SET aa_str  TO "YES". }
    LOCAL far_str IS "NO". IF FAR_AVAILABLE { SET far_str TO "YES". }

    _MENU_ADD_ITEM(LEXICON("type","numeric","key","vr","label","VR (m/s)","min",0,"max",220,"step",_VSPD_STEP,"auto",1)).
    _MENU_ADD_ITEM(LEXICON("type","numeric","key","v2","label","V2 (m/s)","min",0,"max",260,"step",_VSPD_STEP,"auto",1)).
    _MENU_ADD_ITEM(LEXICON("type","numeric","key","vapp","label","VAPP (m/s)","min",0,"max",220,"step",_VSPD_STEP,"auto",1)).
    _MENU_ADD_ITEM(LEXICON("type","numeric","key","slot","label","Plan Slot","min",FMS_SLOT_MIN,"max",FMS_SLOT_MAX,"step",1,"auto",0)).
    _MENU_ADD_ITEM(LEXICON("type","action","key","save","label","Save Slot")).
    _MENU_ADD_ITEM(LEXICON("type","action","key","load","label","Load Slot")).
    _MENU_ADD_ITEM(LEXICON("type","toggle","key","debug","label","Debug Panel")).
    _MENU_ADD_ITEM(LEXICON("type","toggle","key","logger","label","Logger")).
    _MENU_ADD_ITEM(LEXICON("type","action","key","events","label","Event History")).
    _MENU_ADD_ITEM(LEXICON("type","action","key","arm","label","ARM IFC")).
    _MENU_ADD_ITEM(LEXICON("type","action","key","quit","label","Quit")).
    _MENU_ADD_ITEM(LEXICON("type","info","label","--- Summary ---")).
    _MENU_ADD_ITEM(LEXICON("type","info","label","Aircraft: " + ac_name)).
    _MENU_ADD_ITEM(LEXICON("type","info","label","Plan: " + DRAFT_PLAN:LENGTH + " legs")).
    _MENU_ADD_ITEM(LEXICON("type","info","label","AA: " + aa_str + "  FAR: " + far_str)).
    _MENU_ADD_ITEM(LEXICON("type","info","label","Edit plan on main screen (close menu)")).
    RETURN.
  }

  _MENU_ADD_ITEM(LEXICON("type","action","key","gear","label","Toggle Gear")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","flap_up","label","Flaps Step Up")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","flap_dn","label","Flaps Step Down")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","spoilers","label","Toggle Spoilers")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","brakes","label","Toggle Brakes")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","thrust_rev","label","Toggle Thrust Rev")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","drogue","label","Toggle Drogue")).
  _MENU_ADD_ITEM(LEXICON("type","toggle","key","manual","label","Manual Override")).
  _MENU_ADD_ITEM(LEXICON("type","toggle","key","debug","label","Debug Panel")).
  _MENU_ADD_ITEM(LEXICON("type","toggle","key","logger","label","Logger")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","events","label","Event History")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","close","label","Close Menu")).
  _MENU_ADD_ITEM(LEXICON("type","action","key","quit","label","Quit IFC")).
}

FUNCTION _MENU_ITEM_VALUE_TEXT {
  PARAMETER item.
  LOCAL t IS item["type"].
  LOCAL key IS item["key"].

  IF t = "choice" {
    LOCAL idx IS ROUND(_MENU_GET_VALUE(key), 0).
    LOCAL opts IS item["choices"].
    IF idx < 0 { SET idx TO 0. }
    IF idx >= opts:LENGTH { SET idx TO opts:LENGTH - 1. }
    RETURN opts[idx].
  }

  IF t = "numeric" {
    LOCAL num_val IS ROUND(_MENU_GET_VALUE(key), 0).
    IF item["auto"] = 1 AND num_val <= 0 { RETURN "AUTO". }
    RETURN "" + num_val.
  }

  IF t = "toggle" {
    IF _MENU_GET_VALUE(key) > 0 { RETURN "ON". }
    RETURN "OFF".
  }

  RETURN "".
}

FUNCTION _MENU_ITEM_LINE {
  PARAMETER item.
  LOCAL t IS item["type"].
  LOCAL label IS item["label"].
  IF t = "action" { RETURN label. }
  IF t = "info" { RETURN label. }
  RETURN label + ": " + _MENU_ITEM_VALUE_TEXT(item).
}

FUNCTION _MENU_VISIBLE_ROWS {
  // Dynamic: fill the full menu box height between primary/secondary bounds.
  // rows used by frame/header/tail:
  //   top border, title row, tail row, bottom border.
  LOCAL rows_avail IS UI_SEC_BOT - UI_PRI_TOP - 3.
  RETURN MAX(rows_avail, 4).
}

FUNCTION _MENU_CURSOR_CLAMP {
  IF IFC_MENU_ITEMS:LENGTH <= 0 {
    SET IFC_MENU_CURSOR TO 0.
    SET IFC_MENU_SCROLL TO 0.
    RETURN.
  }

  SET IFC_MENU_CURSOR TO CLAMP(IFC_MENU_CURSOR, 0, IFC_MENU_ITEMS:LENGTH - 1).
  LOCAL vis IS _MENU_VISIBLE_ROWS().
  IF IFC_MENU_CURSOR < IFC_MENU_SCROLL { SET IFC_MENU_SCROLL TO IFC_MENU_CURSOR. }
  IF IFC_MENU_CURSOR >= IFC_MENU_SCROLL + vis {
    SET IFC_MENU_SCROLL TO IFC_MENU_CURSOR - vis + 1.
  }
  LOCAL max_scroll IS MAX(IFC_MENU_ITEMS:LENGTH - vis, 0).
  SET IFC_MENU_SCROLL TO CLAMP(IFC_MENU_SCROLL, 0, max_scroll).
}

FUNCTION _MENU_RENDER_MAIN_ROW {
  PARAMETER idx.

  LOCAL vis_rows IS _MENU_VISIBLE_ROWS().
  IF idx < IFC_MENU_SCROLL OR idx >= IFC_MENU_SCROLL + vis_rows { RETURN. }
  IF idx < 0 OR idx >= IFC_MENU_ITEMS:LENGTH { RETURN. }

  LOCAL inner_w IS MAX(UI_W - 4, 30).
  LOCAL row_idx IS UI_PRI_TOP + 2 + (idx - IFC_MENU_SCROLL).
  LOCAL prefix IS "  ".
  IF idx = IFC_MENU_CURSOR { SET prefix TO "> ". }
  LOCAL txt IS prefix + _MENU_ITEM_LINE(IFC_MENU_ITEMS[idx]).
  UI_P("  | " + STR_PAD(txt, inner_w - 1) + "|", row_idx).
}

FUNCTION _MENU_RENDER_MAIN_TAIL {
  LOCAL inner_w IS MAX(UI_W - 4, 30).
  LOCAL vis_rows IS _MENU_VISIBLE_ROWS().

  LOCAL tail IS "items " + (IFC_MENU_CURSOR + 1) + "/" + IFC_MENU_ITEMS:LENGTH.
  IF IFC_PHASE = PHASE_PREARM {
    LOCAL msg IS MENU_VALIDATE_STAGE().
    IF msg <> "" { SET tail TO "validation: " + msg. }
  }
  UI_P("  | " + STR_PAD(tail, inner_w - 1) + "|", UI_PRI_TOP + 2 + vis_rows).
}

FUNCTION _MENU_RENDER_CURSOR_FAST {
  PARAMETER prev_cursor, prev_scroll.

  IF IFC_MENU_PAGE <> "MAIN" { MENU_RENDER(). RETURN. }
  IF prev_scroll <> IFC_MENU_SCROLL { MENU_RENDER(). RETURN. }

  _MENU_RENDER_MAIN_ROW(prev_cursor).
  _MENU_RENDER_MAIN_ROW(IFC_MENU_CURSOR).
  _MENU_RENDER_MAIN_TAIL().
  SET IFC_MENU_DIRTY TO FALSE.
}

FUNCTION MENU_RENDER {
  LOCAL top IS UI_PRI_TOP.
  LOCAL bot IS UI_SEC_BOT.
  LOCAL inner_w IS MAX(UI_W - 4, 30).

  IF IFC_MENU_PAGE = "EVENTS" {
    UI_P("  +" + STR_REPEAT("-", inner_w) + "+", top).
    UI_P("  | " + STR_PAD("Event History (W/S scroll, Y/M back)", inner_w - 1) + "|", top + 1).

    LOCAL rows IS IFC_EVENT_HISTORY_ROWS.
    LOCAL row_idx IS 0.
    UNTIL row_idx >= rows {
      LOCAL idx IS IFC_EVENT_VIEW_IDX - row_idx.
      LOCAL line IS "".
      IF idx >= 0 AND idx < IFC_EVENT_QUEUE:LENGTH {
        LOCAL e IS IFC_EVENT_QUEUE[idx].
        LOCAL t IS UI_FORMAT_TIME(e["ut"] - IFC_MISSION_START_UT).
        SET line TO "T+" + t + " " + e["sev"] + " " + e["msg"].
      }
      UI_P("  | " + STR_PAD(line, inner_w - 1) + "|", top + 2 + row_idx).
      SET row_idx TO row_idx + 1.
    }

    UI_P("  | " + STR_PAD("Use W/S then Y to return", inner_w - 1) + "|", top + 2 + rows).
    UI_P("  +" + STR_REPEAT("-", inner_w) + "+", bot).
    SET IFC_MENU_DIRTY TO FALSE.
    RETURN.
  }

  MENU_BUILD_ITEMS().
  _MENU_CURSOR_CLAMP().

  UI_P("  +" + STR_REPEAT("-", inner_w) + "+", top).
  LOCAL title IS "FMS MENU".
  IF IFC_PHASE = PHASE_PREARM { SET title TO "FMS PREARM". }
  UI_P("  | " + STR_PAD(title + "  [W/S move  A/D change  Y exec  M close]", inner_w - 1) + "|", top + 1).

  LOCAL vis IS _MENU_VISIBLE_ROWS().
  LOCAL i IS 0.
  UNTIL i >= vis {
    LOCAL idx IS IFC_MENU_SCROLL + i.
    LOCAL line IS "".
    IF idx < IFC_MENU_ITEMS:LENGTH {
      LOCAL prefix IS "  ".
      IF idx = IFC_MENU_CURSOR { SET prefix TO "> ". }
      SET line TO prefix + _MENU_ITEM_LINE(IFC_MENU_ITEMS[idx]).
    }
    UI_P("  | " + STR_PAD(line, inner_w - 1) + "|", top + 2 + i).
    SET i TO i + 1.
  }

  LOCAL tail IS "items " + (IFC_MENU_CURSOR + 1) + "/" + IFC_MENU_ITEMS:LENGTH.
  IF IFC_PHASE = PHASE_PREARM {
    LOCAL msg IS MENU_VALIDATE_STAGE().
    IF msg <> "" { SET tail TO "validation: " + msg. }
  }
  UI_P("  | " + STR_PAD(tail, inner_w - 1) + "|", top + 2 + vis).
  UI_P("  +" + STR_REPEAT("-", inner_w) + "+", bot).
  SET IFC_MENU_DIRTY TO FALSE.
}

FUNCTION MENU_DO_FLAP_UP {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_up") { SET ag TO ACTIVE_AIRCRAFT["ag_flaps_step_up"]. }
  IF ag <= 0 { IFC_SET_ALERT("No flap-up AG configured", "WARN"). RETURN. }
  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).
  IF FLAPS_CURRENT_DETENT >= max_det { RETURN. }
  PULSE_AG(ag).
  SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT + 1, 0, max_det).
  SET FLAPS_TARGET_DETENT  TO FLAPS_CURRENT_DETENT.
  SET FLAPS_LAST_STEP_UT   TO TIME:SECONDS.
  IFC_SET_ALERT("Flaps up -> detent " + FLAPS_CURRENT_DETENT).
}

FUNCTION MENU_DO_FLAP_DN {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_flaps_step_down") { SET ag TO ACTIVE_AIRCRAFT["ag_flaps_step_down"]. }
  IF ag <= 0 { IFC_SET_ALERT("No flap-down AG configured", "WARN"). RETURN. }
  LOCAL det_up IS ROUND(AC_PARAM("flaps_detent_up", 0, 0)).
  IF FLAPS_CURRENT_DETENT <= det_up { RETURN. }
  LOCAL max_det IS ROUND(AC_PARAM("flaps_max_detent", 3, 0)).
  PULSE_AG(ag).
  SET FLAPS_CURRENT_DETENT TO CLAMP(FLAPS_CURRENT_DETENT - 1, 0, max_det).
  SET FLAPS_TARGET_DETENT  TO FLAPS_CURRENT_DETENT.
  SET FLAPS_LAST_STEP_UT   TO TIME:SECONDS.
  IFC_SET_ALERT("Flaps down -> detent " + FLAPS_CURRENT_DETENT).
}

FUNCTION MENU_DO_SPOILERS {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_spoilers") { SET ag TO ACTIVE_AIRCRAFT["ag_spoilers"]. }
  IF ag <= 0 { IFC_SET_ALERT("No spoiler AG configured", "WARN"). RETURN. }
  PULSE_AG(ag).
  IFC_SET_ALERT("Spoilers toggled (AG" + ag + ")").
}

FUNCTION MENU_DO_GEAR {
  IF GEAR { GEAR OFF. } ELSE { GEAR ON. }
  IFC_SET_ALERT("Gear toggled").
}

FUNCTION MENU_DO_THRUST_REV {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_thrust_rev") { SET ag TO ACTIVE_AIRCRAFT["ag_thrust_rev"]. }
  IF ag <= 0 { IFC_SET_ALERT("No thrust-reverse AG configured", "WARN"). RETURN. }
  PULSE_AG(ag).
  IFC_SET_ALERT("Thrust reverse toggled (AG" + ag + ")").
}

FUNCTION MENU_DO_DROGUE {
  IF ACTIVE_AIRCRAFT = 0 { RETURN. }
  LOCAL ag IS 0.
  IF ACTIVE_AIRCRAFT:HASKEY("ag_drogue") { SET ag TO ACTIVE_AIRCRAFT["ag_drogue"]. }
  IF ag <= 0 { IFC_SET_ALERT("No drogue AG configured", "WARN"). RETURN. }
  PULSE_AG(ag).
  IFC_SET_ALERT("Drogue toggled (AG" + ag + ")").
}

FUNCTION MENU_DO_BRAKES {
  IF BRAKES { BRAKES OFF. } ELSE { BRAKES ON. }
  IFC_SET_ALERT("Brakes toggled").
}

FUNCTION MENU_DO_LOGGER {
  IF LOG_ACTIVE { LOGGER_CLOSE(). } ELSE { LOGGER_INIT(). }
}

FUNCTION MENU_DO_MANUAL {
  SET IFC_MANUAL_MODE TO NOT IFC_MANUAL_MODE.
  IF IFC_MANUAL_MODE {
    AA_DISABLE_ALL().
    LOCK THROTTLE TO SHIP:THROTTLE.
    IFC_SET_ALERT("Manual override active", "WARN").
    IF IFC_UI_MODE <> UI_MODE_MENU_OVERLAY {
      IFC_SET_UI_MODE(UI_MODE_MANUAL_OVERRIDE).
    }
  } ELSE {
    LOCK THROTTLE TO THROTTLE_CMD.
    AA_INIT().
    IFC_SET_ALERT("Autoflow resumed").
    IF IFC_UI_MODE <> UI_MODE_MENU_OVERLAY {
      IFC_SET_UI_MODE(UI_MODE_AUTOFLOW).
    }
  }
}

FUNCTION MENU_DISPATCH {
  PARAMETER cmd.

  IF cmd = "gear" { MENU_DO_GEAR(). RETURN. }
  IF cmd = "flap_up" { MENU_DO_FLAP_UP(). RETURN. }
  IF cmd = "flap_dn" { MENU_DO_FLAP_DN(). RETURN. }
  IF cmd = "spoilers" { MENU_DO_SPOILERS(). RETURN. }
  IF cmd = "brakes" { MENU_DO_BRAKES(). RETURN. }
  IF cmd = "thrust_rev" { MENU_DO_THRUST_REV(). RETURN. }
  IF cmd = "drogue" { MENU_DO_DROGUE(). RETURN. }
  IF cmd = "manual" { MENU_DO_MANUAL(). RETURN. }
  IF cmd = "logger" { MENU_DO_LOGGER(). RETURN. }

  IF cmd = "debug" {
    SET IFC_DEBUG_PANEL_ON TO NOT IFC_DEBUG_PANEL_ON.
    LOCAL s IS "OFF".
    IF IFC_DEBUG_PANEL_ON { SET s TO "ON". }
    IFC_SET_ALERT("Debug panel " + s).
    RETURN.
  }
}

// ============================================================
// FMS Plan Editor  (pre-arm screen, non-menu-overlay mode)
// ============================================================

LOCAL _FMS_LEG_TYPES       IS LIST(LEG_TAKEOFF, LEG_CRUISE, LEG_APPROACH).
LOCAL _FMS_LEG_TYPE_LABELS IS LIST("TAKEOFF", "CRUISE", "APPROACH").

// Returns a fresh leg LEXICON with default params for the given type string.
FUNCTION _FMS_DEFAULT_LEG {
  PARAMETER ltype.
  IF ltype = LEG_TAKEOFF {
    RETURN LEXICON("type", LEG_TAKEOFF,
      "params", LEXICON("rwy_idx", 0)).
  }
  IF ltype = LEG_CRUISE {
    RETURN LEXICON("type", LEG_CRUISE,
      "params", LEXICON("alt_m", CRUISE_DEFAULT_ALT_M, "spd", CRUISE_DEFAULT_SPD,
                        "wpt0", -1, "wpt1", -1, "wpt2", -1)).
  }
  IF ltype = LEG_APPROACH {
    RETURN LEXICON("type", LEG_APPROACH,
      "params", LEXICON("plate_idx", 0)).
  }
  RETURN LEXICON("type", LEG_TAKEOFF,
    "params", LEXICON("rwy_idx", 0)).
}

// Returns the index (0/1/2) of this leg's type in _FMS_LEG_TYPES.
FUNCTION _FMS_LEG_TYPE_IDX {
  PARAMETER leg.
  LOCAL t IS leg["type"].
  IF t = LEG_TAKEOFF  { RETURN 0. }
  IF t = LEG_CRUISE   { RETURN 1. }
  IF t = LEG_APPROACH { RETURN 2. }
  RETURN 0.
}

// Number of editable fields for this leg (field 0 is always Type).
FUNCTION _FMS_LEG_FIELD_COUNT {
  PARAMETER leg.
  LOCAL t IS leg["type"].
  IF t = LEG_TAKEOFF  { RETURN 2. }              // type, rwy
  IF t = LEG_CRUISE   { RETURN 3 + FMS_WPT_SLOTS. } // type, alt, spd, wpt0..wptN
  IF t = LEG_APPROACH { RETURN 2. }              // type, plate
  RETURN 1.
}

// Label for field fi in leg.
FUNCTION _FMS_LEG_FIELD_LABEL {
  PARAMETER leg, fi.
  IF fi = 0 { RETURN "Type". }
  LOCAL t IS leg["type"].
  IF t = LEG_TAKEOFF {
    IF fi = 1 { RETURN "Runway". }
  }
  IF t = LEG_CRUISE {
    IF fi = 1 { RETURN "Alt (m)". }
    IF fi = 2 { RETURN "Spd (m/s)". }
    IF fi = 3 { RETURN "Waypoint 1". }
    IF fi = 4 { RETURN "Waypoint 2". }
    IF fi = 5 { RETURN "Waypoint 3". }
  }
  IF t = LEG_APPROACH {
    IF fi = 1 { RETURN "Plate". }
  }
  RETURN "---".
}

// Display value string for field fi in leg.
FUNCTION _FMS_LEG_FIELD_VALUE_TEXT {
  PARAMETER leg, fi.
  LOCAL t   IS leg["type"].
  LOCAL prm IS leg["params"].
  IF fi = 0 { RETURN _FMS_LEG_TYPE_LABELS[_FMS_LEG_TYPE_IDX(leg)]. }
  IF t = LEG_TAKEOFF {
    IF fi = 1 {
      IF ROUND(prm["rwy_idx"], 0) = 0 { RETURN "09". }
      RETURN "27".
    }
  }
  IF t = LEG_CRUISE {
    IF fi = 1 { RETURN "" + ROUND(prm["alt_m"], 0). }
    IF fi = 2 { RETURN "" + ROUND(prm["spd"], 0). }
    IF fi = 3 OR fi = 4 OR fi = 5 {
      LOCAL wkey IS "wpt" + (fi - 3).
      LOCAL idx  IS ROUND(prm[wkey], 0).
      IF idx < 0 OR idx >= CUSTOM_WPT_IDS:LENGTH { RETURN "- none -". }
      RETURN CUSTOM_WPT_IDS[idx].
    }
  }
  IF t = LEG_APPROACH {
    IF fi = 1 {
      LOCAL pidx IS ROUND(prm["plate_idx"], 0).
      IF pidx >= 0 AND pidx < PLATE_IDS:LENGTH { RETURN PLATE_IDS[pidx]. }
      RETURN "---".
    }
  }
  RETURN "---".
}

// Mutate field fi of DRAFT_PLAN[leg_idx] by direction dir (+1 or -1).
// Changing field 0 (type) replaces the entire leg with a fresh default.
FUNCTION _FMS_LEG_CHANGE_FIELD {
  PARAMETER leg_idx, fi, dir.
  IF leg_idx < 0 OR leg_idx >= DRAFT_PLAN:LENGTH { RETURN. }
  LOCAL leg IS DRAFT_PLAN[leg_idx].
  LOCAL t   IS leg["type"].
  LOCAL prm IS leg["params"].
  IF fi = 0 {
    LOCAL cur_ti IS _FMS_LEG_TYPE_IDX(leg).
    LOCAL new_ti IS MOD(cur_ti + dir + _FMS_LEG_TYPES:LENGTH, _FMS_LEG_TYPES:LENGTH).
    SET DRAFT_PLAN[leg_idx] TO _FMS_DEFAULT_LEG(_FMS_LEG_TYPES[new_ti]).
    RETURN.
  }
  IF t = LEG_TAKEOFF {
    IF fi = 1 { SET prm["rwy_idx"] TO ROUND(MOD(ROUND(prm["rwy_idx"], 0) + dir + 2, 2), 0). }
  }
  IF t = LEG_CRUISE {
    IF fi = 1 { SET prm["alt_m"] TO CLAMP(ROUND(prm["alt_m"] + dir * 500, 0), 300, 20000). }
    IF fi = 2 { SET prm["spd"]   TO CLAMP(ROUND(prm["spd"]   + dir * 10,  0), 50, 500). }
    IF fi = 3 OR fi = 4 OR fi = 5 {
      LOCAL wkey IS "wpt" + (fi - 3).
      LOCAL n    IS CUSTOM_WPT_IDS:LENGTH.
      LOCAL cur  IS ROUND(prm[wkey], 0) + dir.
      IF cur < -1 { SET cur TO n - 1. }
      IF cur >= n { SET cur TO -1. }
      SET prm[wkey] TO ROUND(cur, 0).
    }
  }
  IF t = LEG_APPROACH {
    IF fi = 1 {
      LOCAL n    IS MAX(PLATE_IDS:LENGTH, 1).
      LOCAL cur  IS ROUND(prm["plate_idx"], 0).
      SET prm["plate_idx"] TO ROUND(MOD(cur + dir + n, n), 0).
    }
  }
}

// One-line summary text for a leg in the list view.
FUNCTION _FMS_LEG_LINE_TEXT {
  PARAMETER leg.
  IF NOT leg:HASKEY("type")   { RETURN "??? (no type)". }
  IF NOT leg:HASKEY("params") { RETURN "??? (no params)". }
  LOCAL t IS leg["type"].
  LOCAL p IS leg["params"].
  IF t = LEG_TAKEOFF {
    LOCAL rwy IS "09".
    IF p:HASKEY("rwy_idx") AND ROUND(p["rwy_idx"], 0) = 1 { SET rwy TO "27". }
    RETURN "TAKEOFF   RWY " + rwy.
  }
  IF t = LEG_CRUISE {
    LOCAL wpts IS "".
    LOCAL slot IS 0.
    UNTIL slot >= FMS_WPT_SLOTS {
      LOCAL wkey IS "wpt" + slot.
      IF p:HASKEY(wkey) AND ROUND(p[wkey], 0) >= 0 {
        LOCAL widx IS ROUND(p[wkey], 0).
        IF widx < CUSTOM_WPT_IDS:LENGTH {
          LOCAL wname IS CUSTOM_WPT_IDS[widx].
          IF wpts = "" { SET wpts TO wname. }
          ELSE { SET wpts TO wpts + "+". }
        }
      }
      SET slot TO slot + 1.
    }
    LOCAL alt_m_val IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd_val   IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("alt_m") { SET alt_m_val TO p["alt_m"]. }
    IF p:HASKEY("spd")   { SET spd_val   TO p["spd"]. }
    LOCAL alt_ft IS ROUND(alt_m_val * 3.281 / 100, 0) * 100.
    IF wpts = "" { RETURN "CRUISE    " + alt_ft + "ft  " + ROUND(spd_val, 0) + "m/s". }
    RETURN "CRUISE    " + alt_ft + "ft  " + wpts.
  }
  IF t = LEG_APPROACH {
    LOCAL pidx IS 0.
    IF p:HASKEY("plate_idx") { SET pidx TO ROUND(p["plate_idx"], 0). }
    IF pidx >= 0 AND pidx < PLATE_IDS:LENGTH { RETURN "APPROACH  " + PLATE_IDS[pidx]. }
    RETURN "APPROACH  ---".
  }
  RETURN "??? leg".
}

// Handle a keypress in FMS prearm mode (not menu overlay).
// Returns "" or "QUIT".
FUNCTION _FMS_EDITOR_KEY {
  PARAMETER ch.

  // GUI is active — suppress keyboard editor to avoid double-processing.
  IF GUI_WIN <> 0 { RETURN "". }

  IF ch = "q" OR ch = "Q" { RETURN "QUIT". }
  IF ch = "m" OR ch = "M" { MENU_OPEN(). RETURN "". }

  IF FMS_EDITING_LEG {
    // --- Edit-field mode ---
    IF ch = "w" OR ch = "W" {
      IF FMS_EDIT_FIELD > 0 { SET FMS_EDIT_FIELD TO FMS_EDIT_FIELD - 1. }
    } ELSE IF ch = "s" OR ch = "S" {
      LOCAL fcount IS 1.
      IF FMS_LEG_CURSOR < DRAFT_PLAN:LENGTH {
        SET fcount TO _FMS_LEG_FIELD_COUNT(DRAFT_PLAN[FMS_LEG_CURSOR]).
      }
      IF FMS_EDIT_FIELD < fcount - 1 { SET FMS_EDIT_FIELD TO FMS_EDIT_FIELD + 1. }
    } ELSE IF ch = "a" OR ch = "A" {
      _FMS_LEG_CHANGE_FIELD(FMS_LEG_CURSOR, FMS_EDIT_FIELD, -1).
      // Clamp field if type changed and new type has fewer fields
      LOCAL fcount IS _FMS_LEG_FIELD_COUNT(DRAFT_PLAN[FMS_LEG_CURSOR]).
      IF FMS_EDIT_FIELD >= fcount { SET FMS_EDIT_FIELD TO fcount - 1. }
    } ELSE IF ch = "d" OR ch = "D" {
      _FMS_LEG_CHANGE_FIELD(FMS_LEG_CURSOR, FMS_EDIT_FIELD, 1).
      LOCAL fcount IS _FMS_LEG_FIELD_COUNT(DRAFT_PLAN[FMS_LEG_CURSOR]).
      IF FMS_EDIT_FIELD >= fcount { SET FMS_EDIT_FIELD TO fcount - 1. }
    } ELSE IF ch = "e" OR ch = "E" OR ch = "y" OR ch = "Y" OR ch = CHAR(13) OR ch = "x" OR ch = "X" {
      SET FMS_EDITING_LEG TO FALSE.
    }
    SET LAST_DISPLAY_UT TO 0.
    RETURN "".
  }

  // --- List mode ---
  IF ch = "w" OR ch = "W" {
    IF FMS_LEG_CURSOR > 0 { SET FMS_LEG_CURSOR TO FMS_LEG_CURSOR - 1. }
  } ELSE IF ch = "s" OR ch = "S" {
    IF FMS_LEG_CURSOR < DRAFT_PLAN:LENGTH - 1 { SET FMS_LEG_CURSOR TO FMS_LEG_CURSOR + 1. }
  } ELSE IF ch = "a" OR ch = "A" {
    // Add a new default leg after cursor (CRUISE by default, or TAKEOFF if list is empty)
    IF DRAFT_PLAN:LENGTH = 0 {
      DRAFT_PLAN:ADD(_FMS_DEFAULT_LEG(LEG_TAKEOFF)).
      SET FMS_LEG_CURSOR TO 0.
    } ELSE {
      LOCAL insert_at IS FMS_LEG_CURSOR + 1.
      DRAFT_PLAN:INSERT(insert_at, _FMS_DEFAULT_LEG(LEG_CRUISE)).
      SET FMS_LEG_CURSOR TO insert_at.
    }
  } ELSE IF ch = "x" OR ch = "X" {
    // Delete leg at cursor
    IF DRAFT_PLAN:LENGTH > 0 {
      DRAFT_PLAN:REMOVE(FMS_LEG_CURSOR).
      IF FMS_LEG_CURSOR >= DRAFT_PLAN:LENGTH AND FMS_LEG_CURSOR > 0 {
        SET FMS_LEG_CURSOR TO DRAFT_PLAN:LENGTH - 1.
      }
    }
  } ELSE IF ch = "e" OR ch = "E" OR ch = "y" OR ch = "Y" OR ch = CHAR(13) {
    // Enter edit mode for current leg
    IF DRAFT_PLAN:LENGTH > 0 {
      SET FMS_EDIT_FIELD  TO 0.
      SET FMS_EDITING_LEG TO TRUE.
    }
  }
  SET LAST_DISPLAY_UT TO 0.
  RETURN "".
}

FUNCTION MENU_OPEN {
  IF IFC_PHASE = PHASE_PREARM {
    MENU_STAGE_FROM_LIVE().
  }
  SET IFC_MENU_PAGE TO "MAIN".
  SET IFC_MENU_CURSOR TO 0.
  SET IFC_MENU_SCROLL TO 0.
  SET IFC_MENU_DIRTY TO TRUE.
  IFC_SET_UI_MODE(UI_MODE_MENU_OVERLAY).
  MENU_RENDER().
}

FUNCTION MENU_CLOSE {
  IF IFC_PHASE = PHASE_PREARM {
    IFC_SET_UI_MODE(UI_MODE_PREARM).
  } ELSE IF IFC_PHASE = PHASE_DONE {
    IFC_SET_UI_MODE(UI_MODE_COMPLETE).
  } ELSE IF IFC_MANUAL_MODE {
    IFC_SET_UI_MODE(UI_MODE_MANUAL_OVERRIDE).
  } ELSE {
    IFC_SET_UI_MODE(UI_MODE_AUTOFLOW).
  }
  SET IFC_MENU_PAGE TO "MAIN".
  SET IFC_MENU_DIRTY TO TRUE.
  SET LAST_DISPLAY_UT TO 0.
  SET LAST_SECONDARY_UT TO 1.
}

FUNCTION _MENU_ADJUST_CURRENT {
  PARAMETER dir.
  IF IFC_MENU_CURSOR < 0 OR IFC_MENU_CURSOR >= IFC_MENU_ITEMS:LENGTH { RETURN. }

  LOCAL item IS IFC_MENU_ITEMS[IFC_MENU_CURSOR].
  LOCAL t IS item["type"].
  LOCAL key IS item["key"].

  IF t = "choice" {
    LOCAL opts IS item["choices"].
    LOCAL n IS MAX(opts:LENGTH, 1).
    LOCAL cur IS ROUND(_MENU_GET_VALUE(key), 0).
    SET cur TO MOD(cur + dir + n, n).
    _MENU_SET_VALUE(key, cur).
    RETURN.
  }

  IF t = "numeric" {
    LOCAL cur IS ROUND(_MENU_GET_VALUE(key), 0).
    LOCAL step IS ROUND(item["step"], 0).
    LOCAL mn IS ROUND(item["min"], 0).
    LOCAL mx IS ROUND(item["max"], 0).
    SET cur TO CLAMP(cur + dir * step, mn, mx).
    IF item["auto"] = 1 AND cur < step { SET cur TO 0. }
    _MENU_SET_VALUE(key, cur).
    RETURN.
  }

  IF t = "toggle" {
    IF _MENU_GET_VALUE(key) > 0 { _MENU_SET_VALUE(key, 0). }
    ELSE { _MENU_SET_VALUE(key, 1). }
    RETURN.
  }
}

FUNCTION _MENU_EXEC_CURRENT {
  IF IFC_MENU_CURSOR < 0 OR IFC_MENU_CURSOR >= IFC_MENU_ITEMS:LENGTH { RETURN "". }

  LOCAL item IS IFC_MENU_ITEMS[IFC_MENU_CURSOR].
  LOCAL t IS item["type"].
  LOCAL key IS item["key"].

  IF t = "toggle" {
    _MENU_ADJUST_CURRENT(1).
    RETURN "".
  }
  IF t <> "action" { RETURN "". }

  IF key = "save" { FMS_SAVE_PLAN(IFC_MENU_SLOT). RETURN "". }

  IF key = "load" {
    IF FMS_LOAD_PLAN(IFC_MENU_SLOT) AND IFC_PHASE = PHASE_PREARM {
      MENU_STAGE_FROM_LIVE().
    }
    RETURN "".
  }

  IF key = "events" {
    SET IFC_MENU_PAGE TO "EVENTS".
    SET IFC_EVENT_VIEW_IDX TO MAX(IFC_EVENT_QUEUE:LENGTH - 1, 0).
    RETURN "".
  }

  IF key = "close" {
    MENU_CLOSE().
    RETURN "".
  }

  IF key = "quit" { RETURN "QUIT". }

  IF key = "arm" {
    LOCAL vmsg IS MENU_VALIDATE_STAGE().
    IF vmsg <> "" {
      IFC_SET_ALERT("Cannot ARM: " + vmsg, "ERROR").
      RETURN "".
    }
    MENU_STAGE_COMMIT().
    MENU_CLOSE().
    IFC_SET_ALERT("IFC armed from prearm menu").
    RETURN "ARM".
  }

  MENU_DISPATCH(key).
  RETURN "".
}

FUNCTION MENU_TICK {
  IFC_SYNC_ALERT_QUEUE().

  IF NOT TERMINAL:INPUT:HASCHAR { RETURN "". }

  LOCAL result IS "".
  LOCAL processed IS 0.
  LOCAL max_chars IS IFC_MENU_MAX_CHARS_TICK.
  LOCAL needs_full_render IS FALSE.
  LOCAL nav_changed IS FALSE.
  LOCAL nav_prev_cursor IS IFC_MENU_CURSOR.
  LOCAL nav_prev_scroll IS IFC_MENU_SCROLL.

  UNTIL NOT TERMINAL:INPUT:HASCHAR OR processed >= max_chars OR result <> "" {
    LOCAL ch IS TERMINAL:INPUT:GETCHAR().
    SET processed TO processed + 1.

    IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY {
      IF IFC_MENU_PAGE = "EVENTS" {
        IF ch = "q" OR ch = "Q" {
          SET result TO "QUIT".
        } ELSE IF ch = "w" OR ch = "W" {
          SET IFC_EVENT_VIEW_IDX TO CLAMP(IFC_EVENT_VIEW_IDX + 1, 0, MAX(IFC_EVENT_QUEUE:LENGTH - 1, 0)).
          SET needs_full_render TO TRUE.
        } ELSE IF ch = "s" OR ch = "S" {
          SET IFC_EVENT_VIEW_IDX TO CLAMP(IFC_EVENT_VIEW_IDX - 1, 0, MAX(IFC_EVENT_QUEUE:LENGTH - 1, 0)).
          SET needs_full_render TO TRUE.
        } ELSE IF ch = "y" OR ch = "Y" OR ch = CHAR(13) OR ch = "m" OR ch = "M" OR ch = "x" OR ch = "X" {
          SET IFC_MENU_PAGE TO "MAIN".
          SET needs_full_render TO TRUE.
        }
      } ELSE {
        IF ch = "q" OR ch = "Q" {
          SET result TO "QUIT".
        } ELSE IF ch = "w" OR ch = "W" {
          IF NOT nav_changed {
            SET nav_prev_cursor TO IFC_MENU_CURSOR.
            SET nav_prev_scroll TO IFC_MENU_SCROLL.
            SET nav_changed TO TRUE.
          }
          SET IFC_MENU_CURSOR TO IFC_MENU_CURSOR - 1.
          _MENU_CURSOR_CLAMP().
        } ELSE IF ch = "s" OR ch = "S" {
          IF NOT nav_changed {
            SET nav_prev_cursor TO IFC_MENU_CURSOR.
            SET nav_prev_scroll TO IFC_MENU_SCROLL.
            SET nav_changed TO TRUE.
          }
          SET IFC_MENU_CURSOR TO IFC_MENU_CURSOR + 1.
          _MENU_CURSOR_CLAMP().
        } ELSE IF ch = "a" OR ch = "A" {
          _MENU_ADJUST_CURRENT(-1).
          SET needs_full_render TO TRUE.
        } ELSE IF ch = "d" OR ch = "D" {
          _MENU_ADJUST_CURRENT(1).
          SET needs_full_render TO TRUE.
        } ELSE IF ch = "y" OR ch = "Y" OR ch = CHAR(13) {
          SET result TO _MENU_EXEC_CURRENT().
          IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY { SET needs_full_render TO TRUE. }
        } ELSE IF ch = "m" OR ch = "M" OR ch = "x" OR ch = "X" {
          MENU_CLOSE().
        }
      }
    } ELSE IF IFC_PHASE = PHASE_PREARM {
      SET result TO _FMS_EDITOR_KEY(ch).
    } ELSE {
      IF ch = "m" OR ch = "M" {
        MENU_OPEN().
      } ELSE IF ch = "q" OR ch = "Q" {
        SET result TO "QUIT".
      } ELSE IF ch = "d" OR ch = "D" {
        MENU_DISPATCH("debug").
      } ELSE IF ch = "l" OR ch = "L" {
        MENU_DISPATCH("logger").
      } ELSE IF ch = "c" OR ch = "C" {
        MENU_DISPATCH("manual").
      } ELSE IF ch = "g" OR ch = "G" {
        MENU_DISPATCH("gear").
      } ELSE IF ch = "F" {
        MENU_DISPATCH("flap_up").
      } ELSE IF ch = "f" {
        MENU_DISPATCH("flap_dn").
      } ELSE IF ch = "s" OR ch = "S" {
        MENU_DISPATCH("spoilers").
      } ELSE IF ch = "t" OR ch = "T" {
        MENU_DISPATCH("thrust_rev").
      } ELSE IF ch = "p" OR ch = "P" {
        MENU_DISPATCH("drogue").
      } ELSE IF ch = "b" OR ch = "B" {
        MENU_DISPATCH("brakes").
      }
    }
  }

  IF IFC_UI_MODE = UI_MODE_MENU_OVERLAY {
    IF needs_full_render OR IFC_MENU_DIRTY {
      MENU_RENDER().
    } ELSE IF nav_changed {
      _MENU_RENDER_CURSOR_FAST(nav_prev_cursor, nav_prev_scroll).
    }
  }
  RETURN result.
}
