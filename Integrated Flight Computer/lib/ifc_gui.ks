@LAZYGLOBAL OFF.

// ============================================================
// ifc_gui.ks  -  Integrated Flight Computer
//
// Two-window flight plan editor for PHASE_PREARM.
//
// Window 1  GUI_WIN      â€” leg list + ARM/QUIT/SAVE/LOAD.
//           Pre-allocated (8 rows), never rebuilt.
//
// Window 2  GUI_EDIT_WIN â€” leg detail editor.
//           Uses cycle buttons for type/nav switching and text/popup controls
//           for numeric entry and waypoint/airport/plate selection.
//           Rebuilt when the leg type or cruise nav subtype changes.
//
// Handle layout in GUI_EDIT_HANDLES (LIST):
//   [0] type_prev   [1] type_lbl   [2] type_next
//   --- type-specific (index 3+) ---
//   TAKEOFF:
//     [3] rwy_prev   [4] rwy_lbl   [5] rwy_next
//   CRUISE:
//     [3] alt_tf                          (TEXTFIELD, m)
//     [4] spd_tf                          (TEXTFIELD, numeric value only)
//     [5] spd_mode_pm                     (POPUPMENU, "IAS m/s"=0 / "Mach"=1)
//     [6] nav_prev  [7] nav_lbl  [8] nav_next  (Waypoint+Alt/Dist+Course/Time+Course)
//     -- nav_type-specific (index 9+) --
//     waypoint:    [9]  wpt0_pm
//                  [10] wpt1_pm
//                  [11] wpt2_pm
//     course_dist: [9]  hdg_tf   (TEXTFIELD, deg)
//                  [10] dst_tf   (TEXTFIELD, nm)
//     course_time: [9]  hdg_tf   (TEXTFIELD, deg)
//                  [10] tme_tf   (TEXTFIELD, min)
//   APPROACH:
//     [3] airport_popup  (POPUPMENU)
//     [4] plate_popup    (POPUPMENU)
//
// Public entry points (called from ifc_main.ks):
//   _GUI_BUILD()  â€” open both windows
//   _GUI_CLOSE()  â€” close both windows
//   _GUI_TICK()   â€” poll all widgets; returns "" / "ARM" / "QUIT"
// ============================================================

LOCAL GUI_MAX_LEGS IS 8.

// â”€â”€ Safe string-to-number parser â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
FUNCTION _PARSE_NUM {
  PARAMETER txt, fallback.
  IF txt:LENGTH = 0 { RETURN fallback. }
  LOCAL VALID IS LIST("0","1","2","3","4","5","6","7","8","9",".").
  LOCAL i IS 0.
  UNTIL i >= txt:LENGTH {
    LOCAL ch IS txt:SUBSTRING(i, 1).
    IF NOT VALID:CONTAINS(ch) {
      IF NOT (ch = "-" AND i = 0) { RETURN fallback. }
    }
    SET i TO i + 1.
  }
  RETURN ("" + txt):TONUMBER(fallback).
}

FUNCTION _GUI_CRUISE_SPD_TEXT {
  PARAMETER mode_raw, spd_val.
  IF CRUISE_IS_MACH_MODE(CRUISE_NORM_SPD_MODE(mode_raw)) {
    RETURN "" + ROUND(spd_val, 2).
  }
  RETURN "" + ROUND(spd_val, 0).
}

FUNCTION _GUI_CRUISE_PARSE_SPD_INPUT {
  PARAMETER txt, fallback_spd, mode_raw.
  LOCAL parsed IS _PARSE_NUM(txt:TRIM, fallback_spd).
  IF CRUISE_IS_MACH_MODE(CRUISE_NORM_SPD_MODE(mode_raw)) {
    RETURN CLAMP(parsed, 0.20, 5.00).
  }
  RETURN CLAMP(parsed, 10, 500).
}

// â”€â”€ Leg-type helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
FUNCTION _TYPE_NAME {
  PARAMETER ti.
  IF ti = 0 { RETURN "TAKEOFF". }
  IF ti = 1 { RETURN "CRUISE". }
  RETURN "APPROACH".
}
FUNCTION _TYPE_TO_IDX {
  PARAMETER t.
  IF t = LEG_TAKEOFF { RETURN 0. }
  IF t = LEG_CRUISE  { RETURN 1. }
  RETURN 2.
}
FUNCTION _IDX_TO_TYPE {
  PARAMETER i.
  IF i = 0 { RETURN LEG_TAKEOFF. }
  IF i = 1 { RETURN LEG_CRUISE. }
  RETURN LEG_APPROACH.
}

// â”€â”€ Swap two legs â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
FUNCTION _GUI_SWAP_LEGS {
  PARAMETER a, b.
  LOCAL tmp IS DRAFT_PLAN[a].
  SET DRAFT_PLAN[a] TO DRAFT_PLAN[b].
  SET DRAFT_PLAN[b] TO tmp.
}

// â”€â”€ Cycle-button row builder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Adds a row: LABEL(row_label) [<] LABEL(val_text) [>]
// Returns LIST(prev_btn, val_lbl, next_btn).
FUNCTION _GUI_CYCLE_ROW {
  PARAMETER container, row_label, val_text.
  LOCAL row IS container:ADDHBOX().
  LOCAL rl IS row:ADDLABEL(row_label).
  SET rl:STYLE:WIDTH TO 80.
  LOCAL pb IS row:ADDBUTTON("<").
  SET pb:STYLE:WIDTH TO 24.
  LOCAL vl IS row:ADDLABEL(val_text).
  SET vl:STYLE:WIDTH TO 110.
  SET vl:STYLE:ALIGN TO "CENTER".
  LOCAL nb IS row:ADDBUTTON(">").
  SET nb:STYLE:WIDTH TO 24.
  RETURN LIST(pb, vl, nb).
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// EDIT WINDOW  (GUI_EDIT_WIN)
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

FUNCTION _GUI_CLOSE_EDIT {
  IF GUI_EDIT_WIN <> 0 {
    SET GUI_EDIT_LAST_X TO GUI_EDIT_WIN:X.
    SET GUI_EDIT_LAST_Y TO GUI_EDIT_WIN:Y.
    SET GUI_EDIT_POS_VALID TO TRUE.
    GUI_EDIT_WIN:DISPOSE().
    SET GUI_EDIT_WIN TO 0.
  }
  GUI_EDIT_HANDLES:CLEAR().
  SET GUI_EDIT_LEG_TYPE TO -1.
  SET GUI_EDIT_OK_BTN TO 0.
}

// Build (or rebuild) the edit window for the current leg.
FUNCTION _GUI_BUILD_EDIT {
  _GUI_CLOSE_EDIT().
  IF DRAFT_PLAN:LENGTH = 0 { RETURN. }

  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  IF NOT leg:HASKEY("type") OR NOT leg:HASKEY("params") { RETURN. }

  LOCAL t IS leg["type"].
  LOCAL p IS leg["params"].
  SET GUI_EDIT_LEG_TYPE TO t.
  SET GUI_EDIT_CLOSED_BY_USER TO FALSE.

  SET GUI_EDIT_WIN TO GUI(380).
  IF GUI_EDIT_POS_VALID {
    SET GUI_EDIT_WIN:X TO GUI_EDIT_LAST_X.
    SET GUI_EDIT_WIN:Y TO GUI_EDIT_LAST_Y.
  }
  LOCAL hdr IS GUI_EDIT_WIN:ADDLABEL("  EDIT LEG " + (cur + 1)).
  SET hdr:STYLE:FONTSIZE TO 11.

  GUI_EDIT_HANDLES:CLEAR().

  // â”€â”€ [0..2] Leg type cycle row â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL ti IS _TYPE_TO_IDX(t).
  LOCAL type_row IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "Leg Type:", _TYPE_NAME(ti)).
  GUI_EDIT_HANDLES:ADD(type_row[0]).  // [0] type_prev
  GUI_EDIT_HANDLES:ADD(type_row[1]).  // [1] type_lbl
  GUI_EDIT_HANDLES:ADD(type_row[2]).  // [2] type_next

  // â”€â”€ Type-specific widgets (index 3+) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  IF t = LEG_TAKEOFF {
    // [3..5] Runway cycle row
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO CLAMP(ROUND(p["rwy_idx"], 0), 0, 1). }
    LOCAL rwy_name IS "RWY 09".
    IF rwy_idx = 1 { SET rwy_name TO "RWY 27". }
    LOCAL rwy_row IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "Runway:", rwy_name).
    GUI_EDIT_HANDLES:ADD(rwy_row[0]).  // [3]
    GUI_EDIT_HANDLES:ADD(rwy_row[1]).  // [4]
    GUI_EDIT_HANDLES:ADD(rwy_row[2]).  // [5]

  } ELSE IF t = LEG_CRUISE {
    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd   IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    IF p:HASKEY("spd")   { SET spd   TO p["spd"]. }
    LOCAL spd_mode IS CRUISE_SPD_MODE_IAS.
    IF p:HASKEY("spd_mode") { SET spd_mode TO CRUISE_NORM_SPD_MODE(p["spd_mode"]). }
    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }

    // [3] Altitude text field (meters)
    LOCAL alt_row IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL alt_rl  IS alt_row:ADDLABEL("Alt (m):").
    SET alt_rl:STYLE:WIDTH TO 80.
    LOCAL alt_tf IS alt_row:ADDTEXTFIELD().
    SET alt_tf:TEXT TO "" + ROUND(alt_m, 0).
    SET alt_tf:TOOLTIP TO "meters ASL".
    SET alt_tf:STYLE:WIDTH TO 110.
    GUI_EDIT_HANDLES:ADD(alt_tf).  // [3]

    // [4] Speed text field  [5] speed mode popup
    LOCAL spd_row IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL spd_rl  IS spd_row:ADDLABEL("Spd:").
    SET spd_rl:STYLE:WIDTH TO 80.
    LOCAL spd_tf IS spd_row:ADDTEXTFIELD().
    SET spd_tf:TEXT TO _GUI_CRUISE_SPD_TEXT(spd_mode, spd).
    SET spd_tf:TOOLTIP TO "IAS m/s or Mach number".
    SET spd_tf:STYLE:WIDTH TO 70.
    GUI_EDIT_HANDLES:ADD(spd_tf).  // [4]
    LOCAL spd_mode_pm IS spd_row:ADDPOPUPMENU().
    spd_mode_pm:ADDOPTION("IAS m/s").
    spd_mode_pm:ADDOPTION("Mach").
    SET spd_mode_pm:INDEX TO CHOOSE 1 IF CRUISE_IS_MACH_MODE(spd_mode) ELSE 0.
    SET spd_mode_pm:CHANGED TO FALSE.
    GUI_EDIT_HANDLES:ADD(spd_mode_pm).  // [5]

    // [6..8] Nav type cycle row
    LOCAL nav_names IS LIST("Waypoint+Alt", "Dist+Course", "Time+Course").
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    LOCAL nav_row IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "Nav:", nav_names[nav_ni]).
    GUI_EDIT_HANDLES:ADD(nav_row[0]).  // [6]
    GUI_EDIT_HANDLES:ADD(nav_row[1]).  // [7]
    GUI_EDIT_HANDLES:ADD(nav_row[2]).  // [8]
    SET GUI_EDIT_NAV_TYPE TO nt.

    // [9+] Nav-type-specific rows
    IF nt = "course_dist" {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }
      LOCAL hdg_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL hdg_rl  IS hdg_row:ADDLABEL("Course (deg):").
      SET hdg_rl:STYLE:WIDTH TO 80.
      LOCAL hdg_tf IS hdg_row:ADDTEXTFIELD().
      SET hdg_tf:TEXT TO "" + cdeg.
      SET hdg_tf:TOOLTIP TO "0-359".
      SET hdg_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(hdg_tf).  // [9]
      LOCAL dst_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL dst_rl  IS dst_row:ADDLABEL("Dist (nm):").
      SET dst_rl:STYLE:WIDTH TO 80.
      LOCAL dst_tf IS dst_row:ADDTEXTFIELD().
      SET dst_tf:TEXT TO "" + dnm.
      SET dst_tf:TOOLTIP TO "nm".
      SET dst_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(dst_tf).  // [10]
    } ELSE IF nt = "course_time" {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }
      LOCAL hdg_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL hdg_rl  IS hdg_row:ADDLABEL("Course (deg):").
      SET hdg_rl:STYLE:WIDTH TO 80.
      LOCAL hdg_tf IS hdg_row:ADDTEXTFIELD().
      SET hdg_tf:TEXT TO "" + cdeg.
      SET hdg_tf:TOOLTIP TO "0-359".
      SET hdg_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(hdg_tf).  // [9]
      LOCAL tme_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL tme_rl  IS tme_row:ADDLABEL("Time (min):").
      SET tme_rl:STYLE:WIDTH TO 80.
      LOCAL tme_tf IS tme_row:ADDTEXTFIELD().
      SET tme_tf:TEXT TO "" + tmin.
      SET tme_tf:TOOLTIP TO "min".
      SET tme_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(tme_tf).  // [10]
    } ELSE {
      // waypoint: [9..11] popup menus
      LOCAL univ IS _FMS_GET_WPT_UNIVERSE().
      LOCAL wids IS univ["ids"].
      LOCAL wnames IS univ["names"].
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL wkey IS "wpt" + slot.
        LOCAL wid IS "".
        IF p:HASKEY(wkey) { SET wid TO p[wkey]. }

        LOCAL wrow IS GUI_EDIT_WIN:ADDHBOX().
        LOCAL wl IS wrow:ADDLABEL("WPT " + (slot + 1) + ":").
        SET wl:STYLE:WIDTH TO 80.
        LOCAL wpm IS wrow:ADDPOPUPMENU().
        SET wpm:STYLE:WIDTH TO 280.
        wpm:ADDOPTION("(none)").
        LOCAL wi IS 0.
        UNTIL wi >= wids:LENGTH {
          wpm:ADDOPTION(wnames[wi]).
          SET wi TO wi + 1.
        }
        LOCAL sel_idx IS 0.
        SET wi TO 0.
        UNTIL wi >= wids:LENGTH OR sel_idx > 0 {
          IF wids[wi] = wid { SET sel_idx TO wi + 1. }
          SET wi TO wi + 1.
        }
        SET wpm:INDEX TO sel_idx.
        SET wpm:CHANGED TO FALSE.
        GUI_EDIT_HANDLES:ADD(wpm).  // [9..11]
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH {
    // [3] airport popup, [4] plate popup (filtered by airport)
    _FMS_AP_NORMALISE_PARAMS(p).
    LOCAL ap_idx IS ROUND(p["airport_idx"], 0).
    LOCAL pl_idx IS ROUND(p["plate_sel_idx"], 0).

    LOCAL ap_row IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL ap_lbl IS ap_row:ADDLABEL("Airport:").
    SET ap_lbl:STYLE:WIDTH TO 80.
    LOCAL ap_pm IS ap_row:ADDPOPUPMENU().
    LOCAL ai IS 0.
    LOCAL ap_count IS _FMS_AP_AIRPORT_COUNT().
    UNTIL ai >= ap_count {
      ap_pm:ADDOPTION(_FMS_AP_AIRPORT_LABEL_BY_INDEX(ai)).
      SET ai TO ai + 1.
    }
    IF ap_count > 0 {
      SET ap_idx TO CLAMP(ap_idx, 0, ap_count - 1).
      SET ap_pm:INDEX TO ap_idx.
    }
    SET ap_pm:CHANGED TO FALSE.
    GUI_EDIT_HANDLES:ADD(ap_pm). // [3]

    LOCAL pl_row IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL pl_lbl IS pl_row:ADDLABEL("Approach:").
    SET pl_lbl:STYLE:WIDTH TO 80.
    LOCAL pl_pm IS pl_row:ADDPOPUPMENU().
    LOCAL ap_plates IS _FMS_AP_PLATES_FOR_AIRPORT_INDEX(ap_idx).
    LOCAL pi IS 0.
    UNTIL pi >= ap_plates:LENGTH {
      pl_pm:ADDOPTION(_FMS_AP_PLATE_LABEL(ap_plates[pi])).
      SET pi TO pi + 1.
    }
    IF ap_plates:LENGTH > 0 {
      SET pl_idx TO CLAMP(pl_idx, 0, ap_plates:LENGTH - 1).
      SET pl_pm:INDEX TO pl_idx.
    }
    SET pl_pm:CHANGED TO FALSE.
    GUI_EDIT_HANDLES:ADD(pl_pm). // [4]
  }

  LOCAL ok_row IS GUI_EDIT_WIN:ADDHBOX().
  SET GUI_EDIT_OK_BTN TO ok_row:ADDBUTTON("      OK      ").
  SET GUI_EDIT_OK_BTN:STYLE:WIDTH TO 140.

  GUI_EDIT_WIN:SHOW().
}

// Refresh edit widget values in-place (no rebuild).
// Forces a rebuild if the current leg's type changed.
FUNCTION _GUI_REFRESH_EDIT {
  IF DRAFT_PLAN:LENGTH = 0 { _GUI_CLOSE_EDIT(). RETURN. }
  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  IF NOT leg:HASKEY("type") { RETURN. }
  LOCAL t IS leg["type"].

  IF t <> GUI_EDIT_LEG_TYPE { _GUI_BUILD_EDIT(). RETURN. }
  IF GUI_EDIT_WIN = 0 OR GUI_EDIT_HANDLES:LENGTH = 0 { RETURN. }

  LOCAL p IS leg["params"].

  // [1] Type label.
  SET GUI_EDIT_HANDLES[1]:TEXT TO _TYPE_NAME(_TYPE_TO_IDX(t)).

  IF t = LEG_TAKEOFF AND GUI_EDIT_HANDLES:LENGTH >= 5 {
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO CLAMP(ROUND(p["rwy_idx"], 0), 0, 1). }
    IF rwy_idx = 0 { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 09". }
    ELSE           { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 27". }

  } ELSE IF t = LEG_CRUISE AND GUI_EDIT_HANDLES:LENGTH >= 9 {
    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }
    // Rebuild if nav_type changed (different sub-rows)
    IF nt <> GUI_EDIT_NAV_TYPE { _GUI_BUILD_EDIT(). RETURN. }

    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd   IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    IF p:HASKEY("spd")   { SET spd   TO p["spd"]. }
    LOCAL spd_mode IS CRUISE_SPD_MODE_IAS.
    IF p:HASKEY("spd_mode") { SET spd_mode TO CRUISE_NORM_SPD_MODE(p["spd_mode"]). }
    SET GUI_EDIT_HANDLES[3]:TEXT TO "" + ROUND(alt_m, 0).
    SET GUI_EDIT_HANDLES[4]:TEXT TO _GUI_CRUISE_SPD_TEXT(spd_mode, spd).
    SET GUI_EDIT_HANDLES[5]:INDEX TO CHOOSE 1 IF CRUISE_IS_MACH_MODE(spd_mode) ELSE 0.
    SET GUI_EDIT_HANDLES[5]:CHANGED TO FALSE.
    LOCAL nav_names IS LIST("Waypoint+Alt", "Dist+Course", "Time+Course").
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    SET GUI_EDIT_HANDLES[7]:TEXT TO nav_names[nav_ni].

    IF nt = "course_dist" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + cdeg.
      SET GUI_EDIT_HANDLES[10]:TEXT TO "" + dnm.
    } ELSE IF nt = "course_time" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + cdeg.
      SET GUI_EDIT_HANDLES[10]:TEXT TO "" + tmin.
    } ELSE {
      LOCAL univ IS _FMS_GET_WPT_UNIVERSE().
      LOCAL wids IS univ["ids"].
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL hi IS 9 + slot.
        IF hi < GUI_EDIT_HANDLES:LENGTH {
          LOCAL wkey IS "wpt" + slot.
          LOCAL wid IS "".
          IF p:HASKEY(wkey) { SET wid TO p[wkey]. }
          LOCAL sel_idx IS 0.
          LOCAL wi IS 0.
          UNTIL wi >= wids:LENGTH OR sel_idx > 0 {
            IF wids[wi] = wid { SET sel_idx TO wi + 1. }
            SET wi TO wi + 1.
          }
          SET GUI_EDIT_HANDLES[hi]:INDEX TO sel_idx.
          SET GUI_EDIT_HANDLES[hi]:CHANGED TO FALSE.
        }
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH AND GUI_EDIT_HANDLES:LENGTH >= 5 {
    _FMS_AP_NORMALISE_PARAMS(p).
    LOCAL ap_count IS _FMS_AP_AIRPORT_COUNT().
    IF ap_count <= 0 { RETURN. }
    LOCAL ap_idx IS CLAMP(ROUND(p["airport_idx"], 0), 0, ap_count - 1).
    IF GUI_EDIT_HANDLES[3]:INDEX <> ap_idx {
      SET GUI_EDIT_HANDLES[3]:INDEX TO ap_idx.
    }
    SET GUI_EDIT_HANDLES[3]:CHANGED TO FALSE.
    LOCAL ap_plates IS _FMS_AP_PLATES_FOR_AIRPORT_INDEX(ap_idx).
    IF ap_plates:LENGTH > 0 {
      LOCAL pl_idx IS CLAMP(ROUND(p["plate_sel_idx"], 0), 0, ap_plates:LENGTH - 1).
      IF GUI_EDIT_HANDLES[4]:INDEX <> pl_idx {
        SET GUI_EDIT_HANDLES[4]:INDEX TO pl_idx.
      }
    }
    SET GUI_EDIT_HANDLES[4]:CHANGED TO FALSE.
  }
}

// Commit edit-window values even if text fields were not explicitly "confirmed".
FUNCTION _GUI_COMMIT_EDIT_FIELDS {
  IF GUI_EDIT_WIN = 0 OR GUI_EDIT_HANDLES:LENGTH = 0 { RETURN FALSE. }
  IF DRAFT_PLAN:LENGTH = 0 { RETURN FALSE. }

  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  LOCAL t   IS GUI_EDIT_LEG_TYPE.
  LOCAL p   IS leg["params"].
  LOCAL changed IS FALSE.

  IF t = LEG_CRUISE AND GUI_EDIT_HANDLES:LENGTH >= 9 {
    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    LOCAL alt_new IS _PARSE_NUM(GUI_EDIT_HANDLES[3]:TEXT, alt_m).
    SET alt_new TO CLAMP(alt_new, 100, 25000).
    IF NOT p:HASKEY("alt_m") OR p["alt_m"] <> alt_new { SET changed TO TRUE. }
    SET p["alt_m"] TO alt_new.
    SET GUI_EDIT_HANDLES[3]:TEXT TO "" + ROUND(alt_new, 0).

    LOCAL spd IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("spd") { SET spd TO p["spd"]. }
    LOCAL mode_idx IS CLAMP(ROUND(GUI_EDIT_HANDLES[5]:INDEX, 0), 0, 1).
    LOCAL spd_mode_new IS CHOOSE CRUISE_SPD_MODE_MACH IF mode_idx = 1 ELSE CRUISE_SPD_MODE_IAS.
    LOCAL spd_new IS _GUI_CRUISE_PARSE_SPD_INPUT(GUI_EDIT_HANDLES[4]:TEXT, spd, spd_mode_new).
    IF NOT p:HASKEY("spd_mode") OR CRUISE_NORM_SPD_MODE(p["spd_mode"]) <> spd_mode_new { SET changed TO TRUE. }
    IF NOT p:HASKEY("spd") OR p["spd"] <> spd_new { SET changed TO TRUE. }
    SET p["spd_mode"] TO spd_mode_new.
    SET p["spd"] TO spd_new.
    SET GUI_EDIT_HANDLES[4]:TEXT TO _GUI_CRUISE_SPD_TEXT(spd_mode_new, spd_new).

    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }

    IF nt = "course_dist" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }

      LOCAL cdeg_new IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, cdeg).
      SET cdeg_new TO MOD(ROUND(cdeg_new, 0) + 360, 360).
      IF NOT p:HASKEY("course_deg") OR p["course_deg"] <> cdeg_new { SET changed TO TRUE. }
      SET p["course_deg"] TO cdeg_new.
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + cdeg_new.

      LOCAL dnm_new IS _PARSE_NUM(GUI_EDIT_HANDLES[10]:TEXT, dnm).
      SET dnm_new TO CLAMP(ROUND(dnm_new, 0), 1, 9999).
      IF NOT p:HASKEY("dist_nm") OR p["dist_nm"] <> dnm_new { SET changed TO TRUE. }
      SET p["dist_nm"] TO dnm_new.
      SET GUI_EDIT_HANDLES[10]:TEXT TO "" + dnm_new.

    } ELSE IF nt = "course_time" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }

      LOCAL cdeg_new IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, cdeg).
      SET cdeg_new TO MOD(ROUND(cdeg_new, 0) + 360, 360).
      IF NOT p:HASKEY("course_deg") OR p["course_deg"] <> cdeg_new { SET changed TO TRUE. }
      SET p["course_deg"] TO cdeg_new.
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + cdeg_new.

      LOCAL tmin_new IS _PARSE_NUM(GUI_EDIT_HANDLES[10]:TEXT, tmin).
      SET tmin_new TO CLAMP(ROUND(tmin_new, 0), 1, 9999).
      IF NOT p:HASKEY("time_min") OR p["time_min"] <> tmin_new { SET changed TO TRUE. }
      SET p["time_min"] TO tmin_new.
      SET GUI_EDIT_HANDLES[10]:TEXT TO "" + tmin_new.
    }
  } ELSE IF t = LEG_APPROACH AND GUI_EDIT_HANDLES:LENGTH >= 5 {
    _FMS_AP_NORMALISE_PARAMS(p).
    LOCAL old_pid IS _FMS_AP_SELECTED_PLATE_ID(p).

    LOCAL ai IS CLAMP(ROUND(GUI_EDIT_HANDLES[3]:INDEX, 0), 0, MAX(_FMS_AP_AIRPORT_COUNT() - 1, 0)).
    LOCAL ap_plates IS _FMS_AP_PLATES_FOR_AIRPORT_INDEX(ai).
    LOCAL pi IS 0.
    IF ap_plates:LENGTH > 0 {
      SET pi TO CLAMP(ROUND(GUI_EDIT_HANDLES[4]:INDEX, 0), 0, ap_plates:LENGTH - 1).
    }

    LOCAL pid IS "".
    IF ap_plates:LENGTH > 0 { SET pid TO ap_plates[pi]. }
    _FMS_AP_SYNC_PARAMS_FROM_PID(p, pid).

    IF _FMS_AP_SELECTED_PLATE_ID(p) <> old_pid { SET changed TO TRUE. }
    SET GUI_EDIT_HANDLES[3]:CHANGED TO FALSE.
    SET GUI_EDIT_HANDLES[4]:CHANGED TO FALSE.
  }

  RETURN changed.
}

// Poll edit widgets each tick.  Returns TRUE if leg data changed.
FUNCTION _GUI_TICK_EDIT {
  IF GUI_EDIT_WIN = 0 OR GUI_EDIT_HANDLES:LENGTH = 0 { RETURN FALSE. }
  IF DRAFT_PLAN:LENGTH = 0 { RETURN FALSE. }

  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  LOCAL t   IS GUI_EDIT_LEG_TYPE.
  LOCAL p   IS leg["params"].

  // â”€â”€ [0] type_prev / [2] type_next â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL ti IS _TYPE_TO_IDX(t).
  IF GUI_EDIT_HANDLES[0]:TAKEPRESS {
    LOCAL new_t IS _IDX_TO_TYPE(MOD(ti + 2, 3)).
    SET DRAFT_PLAN[cur] TO _FMS_DEFAULT_LEG(new_t).
    _GUI_BUILD_EDIT().
    RETURN TRUE.
  }
  IF GUI_EDIT_HANDLES[2]:TAKEPRESS {
    LOCAL new_t IS _IDX_TO_TYPE(MOD(ti + 1, 3)).
    SET DRAFT_PLAN[cur] TO _FMS_DEFAULT_LEG(new_t).
    _GUI_BUILD_EDIT().
    RETURN TRUE.
  }

  // â”€â”€ Type-specific polling â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  IF t = LEG_TAKEOFF AND GUI_EDIT_HANDLES:LENGTH >= 6 {
    // [3]/[5] â€” 2-option toggle
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO CLAMP(ROUND(p["rwy_idx"], 0), 0, 1). }
    IF GUI_EDIT_HANDLES[3]:TAKEPRESS OR GUI_EDIT_HANDLES[5]:TAKEPRESS {
      SET rwy_idx TO 1 - rwy_idx.
      SET p["rwy_idx"] TO rwy_idx.
      IF rwy_idx = 0 { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 09". }
      ELSE           { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 27". }
      RETURN TRUE.
    }

  } ELSE IF t = LEG_CRUISE AND GUI_EDIT_HANDLES:LENGTH >= 9 {
    // [3] Altitude text field (meters)
    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    IF GUI_EDIT_HANDLES[3]:CONFIRMED {
      LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[3]:TEXT, alt_m).
      SET val TO CLAMP(val, 100, 25000).
      SET p["alt_m"] TO val.
      SET GUI_EDIT_HANDLES[3]:TEXT TO "" + ROUND(val, 0).
      RETURN TRUE.
    }

    // [4] Speed text field  [5] speed mode popup
    LOCAL spd IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("spd") { SET spd TO p["spd"]. }
    LOCAL spd_mode IS CRUISE_SPD_MODE_IAS.
    IF p:HASKEY("spd_mode") { SET spd_mode TO CRUISE_NORM_SPD_MODE(p["spd_mode"]). }
    IF GUI_EDIT_HANDLES[4]:CONFIRMED {
      LOCAL mode_idx IS CLAMP(ROUND(GUI_EDIT_HANDLES[5]:INDEX, 0), 0, 1).
      LOCAL mode_str IS CHOOSE CRUISE_SPD_MODE_MACH IF mode_idx = 1 ELSE CRUISE_SPD_MODE_IAS.
      LOCAL val IS _GUI_CRUISE_PARSE_SPD_INPUT(GUI_EDIT_HANDLES[4]:TEXT, spd, mode_str).
      SET p["spd_mode"] TO mode_str.
      SET p["spd"] TO val.
      SET GUI_EDIT_HANDLES[4]:TEXT TO _GUI_CRUISE_SPD_TEXT(mode_str, val).
      RETURN TRUE.
    }
    IF GUI_EDIT_HANDLES[5]:CHANGED {
      SET GUI_EDIT_HANDLES[5]:CHANGED TO FALSE.
      LOCAL mode_idx IS CLAMP(ROUND(GUI_EDIT_HANDLES[5]:INDEX, 0), 0, 1).
      LOCAL new_mode IS CHOOSE CRUISE_SPD_MODE_MACH IF mode_idx = 1 ELSE CRUISE_SPD_MODE_IAS.
      SET p["spd_mode"] TO new_mode.
      SET GUI_EDIT_HANDLES[4]:TEXT TO _GUI_CRUISE_SPD_TEXT(new_mode, spd).
      RETURN TRUE.
    }

    // [6]/[8] â€” Nav type cycle (rebuild on change)
    LOCAL nav_types IS LIST("waypoint", "course_dist", "course_time").
    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    IF GUI_EDIT_HANDLES[6]:TAKEPRESS {
      SET nav_ni TO MOD(nav_ni + 2, 3).
      SET p["nav_type"] TO nav_types[nav_ni].
      _GUI_BUILD_EDIT().
      RETURN TRUE.
    }
    IF GUI_EDIT_HANDLES[8]:TAKEPRESS {
      SET nav_ni TO MOD(nav_ni + 1, 3).
      SET p["nav_type"] TO nav_types[nav_ni].
      _GUI_BUILD_EDIT().
      RETURN TRUE.
    }

    // [9+] â€” Nav-type-specific fields
    IF nt = "course_dist" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }
      // [9] heading text field
      IF GUI_EDIT_HANDLES[9]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, cdeg).
        SET val TO MOD(ROUND(val, 0) + 360, 360).
        SET p["course_deg"] TO val.
        SET GUI_EDIT_HANDLES[9]:TEXT TO "" + val.
        RETURN TRUE.
      }
      // [10] distance text field
      IF GUI_EDIT_HANDLES[10]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[10]:TEXT, dnm).
        SET val TO CLAMP(ROUND(val, 0), 1, 9999).
        SET p["dist_nm"] TO val.
        SET GUI_EDIT_HANDLES[10]:TEXT TO "" + val.
        RETURN TRUE.
      }
    } ELSE IF nt = "course_time" AND GUI_EDIT_HANDLES:LENGTH >= 11 {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }
      // [9] heading text field
      IF GUI_EDIT_HANDLES[9]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, cdeg).
        SET val TO MOD(ROUND(val, 0) + 360, 360).
        SET p["course_deg"] TO val.
        SET GUI_EDIT_HANDLES[9]:TEXT TO "" + val.
        RETURN TRUE.
      }
      // [10] time text field
      IF GUI_EDIT_HANDLES[10]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[10]:TEXT, tmin).
        SET val TO CLAMP(ROUND(val, 0), 1, 9999).
        SET p["time_min"] TO val.
        SET GUI_EDIT_HANDLES[10]:TEXT TO "" + val.
        RETURN TRUE.
      }
    } ELSE {
      // waypoint: [9..11] popup menus
      LOCAL univ IS _FMS_GET_WPT_UNIVERSE().
      LOCAL wids IS univ["ids"].
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL hi IS 9 + slot.
        IF hi < GUI_EDIT_HANDLES:LENGTH AND GUI_EDIT_HANDLES[hi]:CHANGED {
          SET GUI_EDIT_HANDLES[hi]:CHANGED TO FALSE.
          LOCAL idx IS ROUND(GUI_EDIT_HANDLES[hi]:INDEX, 0).
          LOCAL wkey IS "wpt" + slot.
          IF idx <= 0 {
            SET p[wkey] TO "".
          } ELSE IF idx - 1 < wids:LENGTH {
            SET p[wkey] TO wids[idx - 1].
          } ELSE {
            SET p[wkey] TO "".
          }
          RETURN TRUE.
        }
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH AND GUI_EDIT_HANDLES:LENGTH >= 5 {
    _FMS_AP_NORMALISE_PARAMS(p).
    LOCAL cur_ai IS CLAMP(ROUND(p["airport_idx"], 0), 0, MAX(_FMS_AP_AIRPORT_COUNT() - 1, 0)).
    LOCAL cur_pid IS _FMS_AP_SELECTED_PLATE_ID(p).

    IF GUI_EDIT_HANDLES[3]:CHANGED {
      SET GUI_EDIT_HANDLES[3]:CHANGED TO FALSE.
      LOCAL ai IS CLAMP(ROUND(GUI_EDIT_HANDLES[3]:INDEX, 0), 0, MAX(_FMS_AP_AIRPORT_COUNT() - 1, 0)).
      IF ai <> cur_ai {
        LOCAL ap_plates IS _FMS_AP_PLATES_FOR_AIRPORT_INDEX(ai).
        LOCAL pid IS "".
        IF ap_plates:LENGTH > 0 { SET pid TO ap_plates[0]. }
        _FMS_AP_SYNC_PARAMS_FROM_PID(p, pid).
        _GUI_BUILD_EDIT().
        RETURN TRUE.
      }
    }

    IF GUI_EDIT_HANDLES[4]:CHANGED {
      SET GUI_EDIT_HANDLES[4]:CHANGED TO FALSE.
      LOCAL ai IS CLAMP(ROUND(GUI_EDIT_HANDLES[3]:INDEX, 0), 0, MAX(_FMS_AP_AIRPORT_COUNT() - 1, 0)).
      LOCAL ap_plates IS _FMS_AP_PLATES_FOR_AIRPORT_INDEX(ai).
      LOCAL pi IS 0.
      IF ap_plates:LENGTH > 0 {
        SET pi TO CLAMP(ROUND(GUI_EDIT_HANDLES[4]:INDEX, 0), 0, ap_plates:LENGTH - 1).
      }
      LOCAL pid IS "".
      IF ap_plates:LENGTH > 0 { SET pid TO ap_plates[pi]. }
      IF pid <> cur_pid {
        _FMS_AP_SYNC_PARAMS_FROM_PID(p, pid).
        RETURN TRUE.
      }
    }
  }

  RETURN FALSE.
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// MAIN WINDOW  (GUI_WIN)
// Pre-allocated leg list with 8 rows.  Never rebuilt.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

FUNCTION _GUI_CLOSE {
  _GUI_CLOSE_EDIT().
  SET GUI_EDIT_CLOSED_BY_USER TO FALSE.
  IF GUI_WIN <> 0 {
    GUI_WIN:DISPOSE().
    SET GUI_WIN TO 0.
  }
  SET GUI_ADD_BTN  TO 0.
  SET GUI_ARM_BTN  TO 0.
  SET GUI_QUIT_BTN TO 0.
  SET GUI_PLAN_LBL TO 0.
  GUI_LEG_SEL_BTNS:CLEAR().
  GUI_LEG_UP_BTNS:CLEAR().
  GUI_LEG_DN_BTNS:CLEAR().
  GUI_LEG_DEL_BTNS:CLEAR().
  GUI_LEG_SUM_LBLS:CLEAR().
  SET GUI_SAVE_NAME_TF TO 0.
  SET GUI_SAVE_BTN TO 0.
  SET GUI_LOAD_PM TO 0.
  SET GUI_LOAD_BTN TO 0.
}

// Refresh all in-place labels and enabled states.
FUNCTION _GUI_REFRESH {
  IF GUI_WIN = 0 { RETURN. }
  LOCAL n IS DRAFT_PLAN:LENGTH.
  LOCAL cur IS 0.
  IF n > 0 { SET cur TO CLAMP(FMS_LEG_CURSOR, 0, n - 1). }

  SET GUI_PLAN_LBL:TEXT TO "FLIGHT PLAN  (" + n + " legs)".
  SET GUI_ADD_BTN:ENABLED TO n < GUI_MAX_LEGS.

  LOCAL li IS 0.
  UNTIL li >= GUI_MAX_LEGS {
    IF li < n {
      LOCAL pfx IS "   ".
      IF li = cur { SET pfx TO ">  ". }
      SET GUI_LEG_SEL_BTNS[li]:TEXT    TO "" + (li + 1).
      SET GUI_LEG_SEL_BTNS[li]:ENABLED TO TRUE.
      SET GUI_LEG_UP_BTNS[li]:ENABLED  TO li > 0.
      SET GUI_LEG_DN_BTNS[li]:ENABLED  TO li < n - 1.
      SET GUI_LEG_DEL_BTNS[li]:ENABLED TO n > 1.
      SET GUI_LEG_SUM_LBLS[li]:TEXT    TO pfx + _FMS_LEG_LINE_TEXT(DRAFT_PLAN[li]).
    } ELSE {
      SET GUI_LEG_SEL_BTNS[li]:TEXT    TO " ".
      SET GUI_LEG_SEL_BTNS[li]:ENABLED TO FALSE.
      SET GUI_LEG_UP_BTNS[li]:ENABLED  TO FALSE.
      SET GUI_LEG_DN_BTNS[li]:ENABLED  TO FALSE.
      SET GUI_LEG_DEL_BTNS[li]:ENABLED TO FALSE.
      SET GUI_LEG_SUM_LBLS[li]:TEXT    TO "".
    }
    SET li TO li + 1.
  }

  // Save/Load â€” prefill name and repopulate plan list.
  IF GUI_SAVE_NAME_TF <> 0 AND GUI_SAVE_NAME_TF:TEXT = "" {
    SET GUI_SAVE_NAME_TF:TEXT TO FMS_LAST_SAVE_NAME.
  }
  IF GUI_LOAD_PM <> 0 {
    LOCAL selected_idx IS CLAMP(ROUND(GUI_LOAD_PM:INDEX, 0), 0, 9999).
    GUI_LOAD_PM:CLEAR().
    LOCAL plans IS FMS_LIST_PLANS().
    LOCAL pi IS 0.
    LOCAL select_idx IS selected_idx.
    IF select_idx >= plans:LENGTH { SET select_idx TO MAX(plans:LENGTH - 1, 0). }
    UNTIL pi >= plans:LENGTH {
      GUI_LOAD_PM:ADDOPTION(plans[pi]).
      IF FMS_LAST_SAVE_NAME <> "" AND plans[pi] = FMS_LAST_SAVE_NAME {
        SET select_idx TO pi.
      }
      SET pi TO pi + 1.
    }
    IF plans:LENGTH > 0 {
      SET GUI_LOAD_PM:INDEX TO select_idx.
      SET GUI_LOAD_PM:CHANGED TO FALSE.
    }
  }
}

// Build the main window (pre-allocated, never rebuilt).
FUNCTION _GUI_BUILD {
  _GUI_CLOSE().

  SET GUI_WIN TO GUI(370).

  // Header.
  LOCAL hdr IS GUI_WIN:ADDLABEL("  IFC FLIGHT PLAN EDITOR").
  SET hdr:STYLE:FONTSIZE TO 13.
  SET hdr:STYLE:ALIGN    TO "CENTER".

  // Plan header row.
  LOCAL pl_row IS GUI_WIN:ADDHBOX().
  SET GUI_PLAN_LBL TO pl_row:ADDLABEL("").
  SET GUI_PLAN_LBL:STYLE:WIDTH TO 220.
  SET GUI_ADD_BTN TO pl_row:ADDBUTTON("+ ADD LEG").

  // 8 pre-allocated leg rows.
  GUI_LEG_SEL_BTNS:CLEAR().
  GUI_LEG_UP_BTNS:CLEAR().
  GUI_LEG_DN_BTNS:CLEAR().
  GUI_LEG_DEL_BTNS:CLEAR().
  GUI_LEG_SUM_LBLS:CLEAR().

  LOCAL li IS 0.
  UNTIL li >= GUI_MAX_LEGS {
    LOCAL lrow IS GUI_WIN:ADDHBOX().
    LOCAL sel_btn IS lrow:ADDBUTTON(" ").
    SET sel_btn:STYLE:WIDTH TO 26.
    LOCAL up_btn IS lrow:ADDBUTTON("^").
    SET up_btn:STYLE:WIDTH TO 26.
    LOCAL dn_btn IS lrow:ADDBUTTON("v").
    SET dn_btn:STYLE:WIDTH TO 26.
    LOCAL del_btn IS lrow:ADDBUTTON("X").
    SET del_btn:STYLE:WIDTH TO 26.
    LOCAL sum_lbl IS lrow:ADDLABEL("").
    GUI_LEG_SEL_BTNS:ADD(sel_btn).
    GUI_LEG_UP_BTNS:ADD(up_btn).
    GUI_LEG_DN_BTNS:ADD(dn_btn).
    GUI_LEG_DEL_BTNS:ADD(del_btn).
    GUI_LEG_SUM_LBLS:ADD(sum_lbl).
    SET li TO li + 1.
  }

  // Save / Load.
  GUI_WIN:ADDLABEL(" ").
  GUI_WIN:ADDLABEL("  SAVE / LOAD").
  LOCAL save_row IS GUI_WIN:ADDHBOX().
  LOCAL save_lbl IS save_row:ADDLABEL("Name:").
  SET save_lbl:STYLE:WIDTH TO 50.
  SET GUI_SAVE_NAME_TF TO save_row:ADDTEXTFIELD().
  SET GUI_SAVE_NAME_TF:TEXT TO FMS_LAST_SAVE_NAME.
  SET GUI_SAVE_NAME_TF:STYLE:WIDTH TO 200.
  SET GUI_SAVE_BTN TO save_row:ADDBUTTON("SAVE").
  SET GUI_SAVE_BTN:STYLE:WIDTH TO 70.

  LOCAL load_row IS GUI_WIN:ADDHBOX().
  LOCAL load_lbl IS load_row:ADDLABEL("Plan:").
  SET load_lbl:STYLE:WIDTH TO 50.
  SET GUI_LOAD_PM TO load_row:ADDPOPUPMENU().
  SET GUI_LOAD_PM:STYLE:WIDTH TO 200.
  SET GUI_LOAD_BTN TO load_row:ADDBUTTON("LOAD").
  SET GUI_LOAD_BTN:STYLE:WIDTH TO 70.

  // ARM / QUIT.
  GUI_WIN:ADDLABEL(" ").
  LOCAL aq_row IS GUI_WIN:ADDHBOX().
  SET GUI_ARM_BTN  TO aq_row:ADDBUTTON("     ARM     ").
  SET GUI_ARM_BTN:STYLE:WIDTH  TO 160.
  SET GUI_QUIT_BTN TO aq_row:ADDBUTTON("     QUIT     ").
  SET GUI_QUIT_BTN:STYLE:WIDTH TO 160.

  _GUI_REFRESH().
  GUI_WIN:SHOW().

  // Open edit window for the initial leg.
  _GUI_BUILD_EDIT().
}

// Poll all widgets each loop tick.
// Returns "" (normal), "ARM", or "QUIT".
FUNCTION _GUI_TICK {
  // Guard against focus/z-order hiding: if a GUI window gets hidden
  // by user interaction, force it visible again on the next tick.
  IF GUI_WIN = 0 {
    _GUI_BUILD().
    RETURN "".
  }
  IF NOT GUI_WIN:VISIBLE { GUI_WIN:SHOW(). }
  IF GUI_EDIT_WIN = 0 AND DRAFT_PLAN:LENGTH > 0 AND NOT GUI_EDIT_CLOSED_BY_USER {
    _GUI_BUILD_EDIT().
  } ELSE IF GUI_EDIT_WIN <> 0 AND NOT GUI_EDIT_WIN:VISIBLE {
    GUI_EDIT_WIN:SHOW().
  }

  // ARM / QUIT.
  IF GUI_ARM_BTN  <> 0 AND GUI_ARM_BTN:TAKEPRESS  { RETURN "ARM".  }
  IF GUI_QUIT_BTN <> 0 AND GUI_QUIT_BTN:TAKEPRESS { RETURN "QUIT". }
  IF GUI_EDIT_OK_BTN <> 0 AND GUI_EDIT_OK_BTN:TAKEPRESS {
    _GUI_COMMIT_EDIT_FIELDS().
    _GUI_CLOSE_EDIT().
    SET GUI_EDIT_CLOSED_BY_USER TO TRUE.
    _GUI_REFRESH().
    RETURN "".
  }

  // Poll edit window.
  IF _GUI_TICK_EDIT() { _GUI_REFRESH(). }

  // Add leg.
  IF GUI_ADD_BTN <> 0 AND GUI_ADD_BTN:TAKEPRESS {
    IF DRAFT_PLAN:LENGTH < GUI_MAX_LEGS {
      DRAFT_PLAN:ADD(_FMS_DEFAULT_LEG(LEG_CRUISE)).
      SET FMS_LEG_CURSOR TO DRAFT_PLAN:LENGTH - 1.
      _GUI_BUILD_EDIT().
    }
    _GUI_REFRESH().
    RETURN "".
  }

  // Leg list row buttons.
  LOCAL li IS 0.
  UNTIL li >= GUI_LEG_SEL_BTNS:LENGTH {
    IF GUI_LEG_SEL_BTNS[li]:TAKEPRESS {
      SET FMS_LEG_CURSOR TO li.
      IF GUI_EDIT_CLOSED_BY_USER { _GUI_BUILD_EDIT(). }
      ELSE { _GUI_REFRESH_EDIT(). }
      _GUI_REFRESH().
      RETURN "".
    }
    IF GUI_LEG_UP_BTNS[li]:TAKEPRESS {
      IF li > 0 {
        _GUI_SWAP_LEGS(li, li - 1).
        SET FMS_LEG_CURSOR TO li - 1.
        IF GUI_EDIT_CLOSED_BY_USER { _GUI_BUILD_EDIT(). }
        ELSE { _GUI_REFRESH_EDIT(). }
        _GUI_REFRESH().
      }
      RETURN "".
    }
    IF GUI_LEG_DN_BTNS[li]:TAKEPRESS {
      IF li < DRAFT_PLAN:LENGTH - 1 {
        _GUI_SWAP_LEGS(li, li + 1).
        SET FMS_LEG_CURSOR TO li + 1.
        IF GUI_EDIT_CLOSED_BY_USER { _GUI_BUILD_EDIT(). }
        ELSE { _GUI_REFRESH_EDIT(). }
        _GUI_REFRESH().
      }
      RETURN "".
    }
    IF GUI_LEG_DEL_BTNS[li]:TAKEPRESS {
      IF DRAFT_PLAN:LENGTH > 1 {
        DRAFT_PLAN:REMOVE(li).
        IF FMS_LEG_CURSOR >= DRAFT_PLAN:LENGTH {
          SET FMS_LEG_CURSOR TO DRAFT_PLAN:LENGTH - 1.
        }
        _GUI_BUILD_EDIT().
        _GUI_REFRESH().
      }
      RETURN "".
    }
    SET li TO li + 1.
  }

  // Save / Load.
  IF GUI_SAVE_BTN <> 0 AND GUI_SAVE_BTN:TAKEPRESS {
    LOCAL sname IS "plan".
    IF GUI_SAVE_NAME_TF <> 0 AND GUI_SAVE_NAME_TF:TEXT:LENGTH > 0 {
      SET sname TO GUI_SAVE_NAME_TF:TEXT.
    }
    FMS_SAVE_PLAN(sname).
    _GUI_REFRESH().
    RETURN "".
  }
  IF GUI_LOAD_BTN <> 0 AND GUI_LOAD_BTN:TAKEPRESS {
    IF GUI_LOAD_PM <> 0 {
      LOCAL plans IS FMS_LIST_PLANS().
      IF plans:LENGTH > 0 {
        LOCAL pi IS CLAMP(ROUND(GUI_LOAD_PM:INDEX, 0), 0, plans:LENGTH - 1).
        FMS_LOAD_PLAN(plans[pi]).
      } ELSE IF GUI_SAVE_NAME_TF <> 0 {
        LOCAL typed_name IS GUI_SAVE_NAME_TF:TEXT:TRIM.
        IF typed_name <> "" AND FMS_PLAN_EXISTS(typed_name) {
          FMS_LOAD_PLAN(typed_name).
        } ELSE {
          IFC_SET_ALERT("No saved plans in index; enter plan name to load").
        }
      }
      _GUI_BUILD_EDIT().
      _GUI_REFRESH().
    }
    RETURN "".
  }

  RETURN "".
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// END-OF-FLIGHT CONFIRMATION DIALOG
// Blocking mini-loop GUI.  Returns TRUE = new flight, FALSE = exit.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
FUNCTION _GUI_SHOW_END_CONFIRM {
  LOCAL win IS GUI(280).
  LOCAL hdr IS win:ADDLABEL("  FLIGHT COMPLETE").
  SET hdr:STYLE:FONTSIZE TO 13.
  SET hdr:STYLE:ALIGN TO "CENTER".
  win:ADDLABEL(" ").
  win:ADDLABEL("  Return to plan editor?").
  win:ADDLABEL(" ").
  LOCAL btn_row IS win:ADDHBOX().
  LOCAL new_btn  IS btn_row:ADDBUTTON("  NEW FLIGHT  ").
  LOCAL exit_btn IS btn_row:ADDBUTTON("     EXIT     ").
  SET new_btn:STYLE:WIDTH  TO 125.
  SET exit_btn:STYLE:WIDTH TO 125.
  win:SHOW().

  LOCAL chosen  IS FALSE.
  LOCAL confirm IS FALSE.
  UNTIL chosen {
    IF new_btn:TAKEPRESS  { SET chosen TO TRUE. SET confirm TO TRUE. }
    IF exit_btn:TAKEPRESS { SET chosen TO TRUE. }
    WAIT IFC_LOOP_DT.
  }

  win:DISPOSE().
  RETURN confirm.
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// IN-FLIGHT PLAN EDITING
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

// Open the plan editor mid-flight.  Populates DRAFT_PLAN from the
// remaining legs in FLIGHT_PLAN_DRAFT_COPY (editor format), then
// builds the GUI window with COMMIT / CANCEL button labels.
FUNCTION _GUI_OPEN_INFLIGHT {
  DRAFT_PLAN:CLEAR().
  LOCAL start IS FLIGHT_PLAN_INDEX + 1.
  IF start < FLIGHT_PLAN_DRAFT_COPY:LENGTH {
    LOCAL i IS start.
    UNTIL i >= FLIGHT_PLAN_DRAFT_COPY:LENGTH {
      DRAFT_PLAN:ADD(FLIGHT_PLAN_DRAFT_COPY[i]).
      SET i TO i + 1.
    }
  }
  IF DRAFT_PLAN:LENGTH = 0 {
    DRAFT_PLAN:ADD(_FMS_DEFAULT_LEG(LEG_APPROACH)).
  }
  SET FMS_LEG_CURSOR TO 0.

  _GUI_BUILD().

  // Relabel ARM â†’ COMMIT, QUIT â†’ DISCARD for in-flight context.
  IF GUI_ARM_BTN  <> 0 { SET GUI_ARM_BTN:TEXT  TO "    COMMIT    ". }
  IF GUI_QUIT_BTN <> 0 { SET GUI_QUIT_BTN:TEXT TO "    CANCEL    ". }

  SET GUI_INFLIGHT_MODE TO TRUE.
}

// Poll the in-flight plan editor each loop tick.
// Delegates to _GUI_TICK(); translates ARM â†’ COMMIT, QUIT â†’ DISCARD.
// Returns "" / "COMMIT" / "CANCEL".
FUNCTION _GUI_TICK_INFLIGHT {
  LOCAL tick_res IS _GUI_TICK().
  IF tick_res = "ARM" {
    _GUI_COMMIT_EDIT_FIELDS().
    SET GUI_INFLIGHT_COMMIT_PENDING TO TRUE.
    SET GUI_INFLIGHT_MODE TO FALSE.
    _GUI_CLOSE().
    RETURN "COMMIT".
  }
  IF tick_res = "QUIT" {
    SET GUI_INFLIGHT_MODE TO FALSE.
    _GUI_CLOSE().
    RETURN "CANCEL".
  }
  RETURN "".
}

