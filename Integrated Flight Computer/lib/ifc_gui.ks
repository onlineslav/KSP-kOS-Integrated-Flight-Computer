@LAZYGLOBAL OFF.

// ============================================================
// ifc_gui.ks  -  Integrated Flight Computer
//
// Two-window flight plan editor for PHASE_PREARM.
//
// Window 1  GUI_WIN      — leg list + ARM/QUIT/SAVE/LOAD.
//           Pre-allocated (8 rows), never rebuilt.
//
// Window 2  GUI_EDIT_WIN — leg detail editor.
//           All selections use [<] LABEL [>] cycle buttons.
//           No TEXTFIELD / POPUPMENU (both have kOS compat issues).
//           Rebuilt only when the leg type changes.
//
// Handle layout in GUI_EDIT_HANDLES (LIST):
//   [0] type_prev   [1] type_lbl   [2] type_next
//   --- type-specific (index 3+) ---
//   TAKEOFF:
//     [3] rwy_prev   [4] rwy_lbl   [5] rwy_next
//   CRUISE:
//     [3] alt_tf                          (TEXTFIELD, ft)
//     [4] spd_tf                          (TEXTFIELD, m/s)
//     [5] nav_prev  [6] nav_lbl  [7] nav_next  (Waypoint/Dist+Course/Time+Course)
//     -- nav_type-specific (index 8+) --
//     waypoint:    [8]  wpt0_prev [9]  wpt0_lbl [10] wpt0_next
//                  [11] wpt1_prev [12] wpt1_lbl [13] wpt1_next
//                  [14] wpt2_prev [15] wpt2_lbl [16] wpt2_next
//     course_dist: [8]  hdg_tf   (TEXTFIELD, deg)
//                  [9]  dst_tf   (TEXTFIELD, nm)
//     course_time: [8]  hdg_tf   (TEXTFIELD, deg)
//                  [9]  tme_tf   (TEXTFIELD, min)
//   APPROACH:
//     [3] plate_popup  (POPUPMENU)
//
// Public entry points (called from ifc_main.ks):
//   _GUI_BUILD()  — open both windows
//   _GUI_CLOSE()  — close both windows
//   _GUI_TICK()   — poll all widgets; returns "" / "ARM" / "QUIT"
// ============================================================

LOCAL GUI_MAX_LEGS IS 8.

// ── Safe string-to-number parser ─────────────────────────
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
  RETURN TONUMBER(txt).
}

// ── Leg-type helpers ──────────────────────────────────────
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

// ── Swap two legs ─────────────────────────────────────────
FUNCTION _GUI_SWAP_LEGS {
  PARAMETER a, b.
  LOCAL tmp IS DRAFT_PLAN[a].
  SET DRAFT_PLAN[a] TO DRAFT_PLAN[b].
  SET DRAFT_PLAN[b] TO tmp.
}

// ── Cycle-button row builder ──────────────────────────────
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

// ══════════════════════════════════════════════════════════
// EDIT WINDOW  (GUI_EDIT_WIN)
// ══════════════════════════════════════════════════════════

FUNCTION _GUI_CLOSE_EDIT {
  IF GUI_EDIT_WIN <> 0 {
    GUI_EDIT_WIN:DISPOSE().
    SET GUI_EDIT_WIN TO 0.
  }
  GUI_EDIT_HANDLES:CLEAR().
  SET GUI_EDIT_LEG_TYPE TO -1.
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

  SET GUI_EDIT_WIN TO GUI(270).
  LOCAL hdr IS GUI_EDIT_WIN:ADDLABEL("  EDIT LEG " + (cur + 1)).
  SET hdr:STYLE:FONTSIZE TO 11.

  GUI_EDIT_HANDLES:CLEAR().

  // ── [0..2] Leg type cycle row ─────────────────────────
  LOCAL ti IS _TYPE_TO_IDX(t).
  LOCAL type_row IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "Leg Type:", _TYPE_NAME(ti)).
  GUI_EDIT_HANDLES:ADD(type_row[0]).  // [0] type_prev
  GUI_EDIT_HANDLES:ADD(type_row[1]).  // [1] type_lbl
  GUI_EDIT_HANDLES:ADD(type_row[2]).  // [2] type_next

  // ── Type-specific widgets (index 3+) ─────────────────
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

    // [4] Speed text field (m/s)
    LOCAL spd_row IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL spd_rl  IS spd_row:ADDLABEL("Spd (m/s):").
    SET spd_rl:STYLE:WIDTH TO 80.
    LOCAL spd_tf IS spd_row:ADDTEXTFIELD().
    SET spd_tf:TEXT TO "" + ROUND(spd, 0).
    SET spd_tf:TOOLTIP TO "m/s".
    SET spd_tf:STYLE:WIDTH TO 110.
    GUI_EDIT_HANDLES:ADD(spd_tf).  // [4]

    // [5..7] Nav type cycle row
    LOCAL nav_names IS LIST("Waypoint", "Dist+Course", "Time+Course").
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    LOCAL nav_row IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "Nav:", nav_names[nav_ni]).
    GUI_EDIT_HANDLES:ADD(nav_row[0]).  // [5]
    GUI_EDIT_HANDLES:ADD(nav_row[1]).  // [6]
    GUI_EDIT_HANDLES:ADD(nav_row[2]).  // [7]
    SET GUI_EDIT_NAV_TYPE TO nt.

    // [8+] Nav-type-specific rows
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
      GUI_EDIT_HANDLES:ADD(hdg_tf).  // [8]
      LOCAL dst_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL dst_rl  IS dst_row:ADDLABEL("Dist (nm):").
      SET dst_rl:STYLE:WIDTH TO 80.
      LOCAL dst_tf IS dst_row:ADDTEXTFIELD().
      SET dst_tf:TEXT TO "" + dnm.
      SET dst_tf:TOOLTIP TO "nm".
      SET dst_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(dst_tf).  // [9]
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
      GUI_EDIT_HANDLES:ADD(hdg_tf).  // [8]
      LOCAL tme_row IS GUI_EDIT_WIN:ADDHBOX().
      LOCAL tme_rl  IS tme_row:ADDLABEL("Time (min):").
      SET tme_rl:STYLE:WIDTH TO 80.
      LOCAL tme_tf IS tme_row:ADDTEXTFIELD().
      SET tme_tf:TEXT TO "" + tmin.
      SET tme_tf:TOOLTIP TO "min".
      SET tme_tf:STYLE:WIDTH TO 110.
      GUI_EDIT_HANDLES:ADD(tme_tf).  // [9]
    } ELSE {
      // waypoint: [8..16] — 3 slots × 3 handles each
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL wkey IS "wpt" + slot.
        LOCAL widx IS -1.
        IF p:HASKEY(wkey) { SET widx TO ROUND(p[wkey], 0). }
        LOCAL wpt_name IS "- none -".
        IF widx >= 0 AND widx < CUSTOM_WPT_IDS:LENGTH {
          SET wpt_name TO CUSTOM_WPT_IDS[widx].
        }
        LOCAL wrow IS _GUI_CYCLE_ROW(GUI_EDIT_WIN, "WPT " + (slot + 1) + ":", wpt_name).
        GUI_EDIT_HANDLES:ADD(wrow[0]).
        GUI_EDIT_HANDLES:ADD(wrow[1]).
        GUI_EDIT_HANDLES:ADD(wrow[2]).
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH {
    // [3] Approach plate popup menu
    LOCAL pidx IS 0.
    IF p:HASKEY("plate_idx") { SET pidx TO ROUND(p["plate_idx"], 0). }
    IF PLATE_IDS:LENGTH > 0 {
      SET pidx TO CLAMP(pidx, 0, PLATE_IDS:LENGTH - 1).
    }
    LOCAL aprow IS GUI_EDIT_WIN:ADDHBOX().
    LOCAL apl IS aprow:ADDLABEL("Approach:").
    SET apl:STYLE:WIDTH TO 80.
    LOCAL pm IS aprow:ADDPOPUPMENU().
    LOCAL oi IS 0.
    UNTIL oi >= PLATE_IDS:LENGTH {
      pm:ADDOPTION(PLATE_IDS[oi]).
      SET oi TO oi + 1.
    }
    IF PLATE_IDS:LENGTH > 0 { SET pm:INDEX TO pidx. }
    GUI_EDIT_HANDLES:ADD(pm).  // [3]
  }

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

  } ELSE IF t = LEG_CRUISE AND GUI_EDIT_HANDLES:LENGTH >= 8 {
    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }
    // Rebuild if nav_type changed (different sub-rows)
    IF nt <> GUI_EDIT_NAV_TYPE { _GUI_BUILD_EDIT(). RETURN. }

    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd   IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    IF p:HASKEY("spd")   { SET spd   TO p["spd"]. }
    SET GUI_EDIT_HANDLES[3]:TEXT TO "" + ROUND(alt_m, 0).
    SET GUI_EDIT_HANDLES[4]:TEXT TO "" + ROUND(spd, 0).
    LOCAL nav_names IS LIST("Waypoint", "Dist+Course", "Time+Course").
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    SET GUI_EDIT_HANDLES[6]:TEXT TO nav_names[nav_ni].

    IF nt = "course_dist" AND GUI_EDIT_HANDLES:LENGTH >= 10 {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }
      SET GUI_EDIT_HANDLES[8]:TEXT TO "" + cdeg.
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + dnm.
    } ELSE IF nt = "course_time" AND GUI_EDIT_HANDLES:LENGTH >= 10 {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }
      SET GUI_EDIT_HANDLES[8]:TEXT TO "" + cdeg.
      SET GUI_EDIT_HANDLES[9]:TEXT TO "" + tmin.
    } ELSE {
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL hi IS 8 + slot * 3 + 1.
        IF hi < GUI_EDIT_HANDLES:LENGTH {
          LOCAL wkey IS "wpt" + slot.
          LOCAL widx IS -1.
          IF p:HASKEY(wkey) { SET widx TO ROUND(p[wkey], 0). }
          IF widx >= 0 AND widx < CUSTOM_WPT_IDS:LENGTH {
            SET GUI_EDIT_HANDLES[hi]:TEXT TO CUSTOM_WPT_IDS[widx].
          } ELSE {
            SET GUI_EDIT_HANDLES[hi]:TEXT TO "- none -".
          }
        }
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH AND GUI_EDIT_HANDLES:LENGTH >= 4 {
    LOCAL pidx IS 0.
    IF p:HASKEY("plate_idx") { SET pidx TO ROUND(p["plate_idx"], 0). }
    IF PLATE_IDS:LENGTH > 0 {
      SET pidx TO CLAMP(pidx, 0, PLATE_IDS:LENGTH - 1).
      SET GUI_EDIT_HANDLES[3]:INDEX TO pidx.
    }
  }
}

// Poll edit widgets each tick.  Returns TRUE if leg data changed.
FUNCTION _GUI_TICK_EDIT {
  IF GUI_EDIT_WIN = 0 OR GUI_EDIT_HANDLES:LENGTH = 0 { RETURN FALSE. }
  IF DRAFT_PLAN:LENGTH = 0 { RETURN FALSE. }

  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  LOCAL t   IS GUI_EDIT_LEG_TYPE.
  LOCAL p   IS leg["params"].

  // ── [0] type_prev / [2] type_next ────────────────────
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

  // ── Type-specific polling ─────────────────────────────
  IF t = LEG_TAKEOFF AND GUI_EDIT_HANDLES:LENGTH >= 6 {
    // [3]/[5] — 2-option toggle
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO CLAMP(ROUND(p["rwy_idx"], 0), 0, 1). }
    IF GUI_EDIT_HANDLES[3]:TAKEPRESS OR GUI_EDIT_HANDLES[5]:TAKEPRESS {
      SET rwy_idx TO 1 - rwy_idx.
      SET p["rwy_idx"] TO rwy_idx.
      IF rwy_idx = 0 { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 09". }
      ELSE           { SET GUI_EDIT_HANDLES[4]:TEXT TO "RWY 27". }
      RETURN TRUE.
    }

  } ELSE IF t = LEG_CRUISE AND GUI_EDIT_HANDLES:LENGTH >= 8 {
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

    // [4] Speed text field (m/s)
    LOCAL spd IS CRUISE_DEFAULT_SPD.
    IF p:HASKEY("spd") { SET spd TO p["spd"]. }
    IF GUI_EDIT_HANDLES[4]:CONFIRMED {
      LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[4]:TEXT, spd).
      SET val TO CLAMP(val, 10, 500).
      SET p["spd"] TO val.
      SET GUI_EDIT_HANDLES[4]:TEXT TO "" + ROUND(val, 0).
      RETURN TRUE.
    }

    // [5]/[7] — Nav type cycle (rebuild on change)
    LOCAL nav_types IS LIST("waypoint", "course_dist", "course_time").
    LOCAL nt IS "waypoint".
    IF p:HASKEY("nav_type") { SET nt TO p["nav_type"]. }
    LOCAL nav_ni IS 0.
    IF nt = "course_dist" { SET nav_ni TO 1. }
    IF nt = "course_time" { SET nav_ni TO 2. }
    IF GUI_EDIT_HANDLES[5]:TAKEPRESS {
      SET nav_ni TO MOD(nav_ni + 2, 3).
      SET p["nav_type"] TO nav_types[nav_ni].
      _GUI_BUILD_EDIT().
      RETURN TRUE.
    }
    IF GUI_EDIT_HANDLES[7]:TAKEPRESS {
      SET nav_ni TO MOD(nav_ni + 1, 3).
      SET p["nav_type"] TO nav_types[nav_ni].
      _GUI_BUILD_EDIT().
      RETURN TRUE.
    }

    // [8+] — Nav-type-specific fields
    IF nt = "course_dist" AND GUI_EDIT_HANDLES:LENGTH >= 10 {
      LOCAL cdeg IS 90.
      LOCAL dnm  IS 100.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("dist_nm")    { SET dnm  TO ROUND(p["dist_nm"],    0). }
      // [8] heading text field
      IF GUI_EDIT_HANDLES[8]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[8]:TEXT, cdeg).
        SET val TO MOD(ROUND(val, 0) + 360, 360).
        SET p["course_deg"] TO val.
        SET GUI_EDIT_HANDLES[8]:TEXT TO "" + val.
        RETURN TRUE.
      }
      // [9] distance text field
      IF GUI_EDIT_HANDLES[9]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, dnm).
        SET val TO CLAMP(ROUND(val, 0), 1, 9999).
        SET p["dist_nm"] TO val.
        SET GUI_EDIT_HANDLES[9]:TEXT TO "" + val.
        RETURN TRUE.
      }
    } ELSE IF nt = "course_time" AND GUI_EDIT_HANDLES:LENGTH >= 10 {
      LOCAL cdeg IS 90.
      LOCAL tmin IS 60.
      IF p:HASKEY("course_deg") { SET cdeg TO ROUND(p["course_deg"], 0). }
      IF p:HASKEY("time_min")   { SET tmin TO ROUND(p["time_min"],   0). }
      // [8] heading text field
      IF GUI_EDIT_HANDLES[8]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[8]:TEXT, cdeg).
        SET val TO MOD(ROUND(val, 0) + 360, 360).
        SET p["course_deg"] TO val.
        SET GUI_EDIT_HANDLES[8]:TEXT TO "" + val.
        RETURN TRUE.
      }
      // [9] time text field
      IF GUI_EDIT_HANDLES[9]:CONFIRMED {
        LOCAL val IS _PARSE_NUM(GUI_EDIT_HANDLES[9]:TEXT, tmin).
        SET val TO CLAMP(ROUND(val, 0), 1, 9999).
        SET p["time_min"] TO val.
        SET GUI_EDIT_HANDLES[9]:TEXT TO "" + val.
        RETURN TRUE.
      }
    } ELSE {
      // waypoint: [8..16]
      LOCAL slot IS 0.
      UNTIL slot >= FMS_WPT_SLOTS {
        LOCAL base IS 8 + slot * 3.
        IF base + 2 < GUI_EDIT_HANDLES:LENGTH {
          LOCAL wkey IS "wpt" + slot.
          LOCAL widx IS -1.
          IF p:HASKEY(wkey) { SET widx TO ROUND(p[wkey], 0). }
          LOCAL n IS CUSTOM_WPT_IDS:LENGTH.
          IF GUI_EDIT_HANDLES[base]:TAKEPRESS {
            SET widx TO widx - 1.
            IF widx < -1 { SET widx TO n - 1. }
            SET p[wkey] TO widx.
            IF widx < 0 { SET GUI_EDIT_HANDLES[base + 1]:TEXT TO "- none -". }
            ELSE        { SET GUI_EDIT_HANDLES[base + 1]:TEXT TO CUSTOM_WPT_IDS[widx]. }
            RETURN TRUE.
          }
          IF GUI_EDIT_HANDLES[base + 2]:TAKEPRESS {
            SET widx TO widx + 1.
            IF widx >= n { SET widx TO -1. }
            SET p[wkey] TO widx.
            IF widx < 0 { SET GUI_EDIT_HANDLES[base + 1]:TEXT TO "- none -". }
            ELSE        { SET GUI_EDIT_HANDLES[base + 1]:TEXT TO CUSTOM_WPT_IDS[widx]. }
            RETURN TRUE.
          }
        }
        SET slot TO slot + 1.
      }
    }

  } ELSE IF t = LEG_APPROACH AND GUI_EDIT_HANDLES:LENGTH >= 4 {
    IF PLATE_IDS:LENGTH = 0 { RETURN FALSE. }
    IF GUI_EDIT_HANDLES[3]:CHANGED {
      SET GUI_EDIT_HANDLES[3]:CHANGED TO FALSE.
      LOCAL ni IS GUI_EDIT_HANDLES[3]:INDEX.
      IF ni >= 0 AND ni < PLATE_IDS:LENGTH {
        SET p["plate_idx"] TO ni.
        RETURN TRUE.
      }
    }
  }

  RETURN FALSE.
}

// ══════════════════════════════════════════════════════════
// MAIN WINDOW  (GUI_WIN)
// Pre-allocated leg list with 8 rows.  Never rebuilt.
// ══════════════════════════════════════════════════════════

FUNCTION _GUI_CLOSE {
  _GUI_CLOSE_EDIT().
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
  GUI_SLOT_SAVE_BTNS:CLEAR().
  GUI_SLOT_LOAD_BTNS:CLEAR().
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

  // Save/Load slot labels.
  LOCAL si IS 0.
  UNTIL si >= GUI_SLOT_LOAD_BTNS:LENGTH {
    LOCAL sn IS si + 1.
    LOCAL lmark IS "  ".
    IF FMS_SLOT_EXISTS(sn) { SET lmark TO "* ". }
    SET GUI_SLOT_LOAD_BTNS[si]:TEXT TO "LOAD " + sn + lmark.
    SET si TO si + 1.
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
  GUI_SLOT_SAVE_BTNS:CLEAR().
  GUI_SLOT_LOAD_BTNS:CLEAR().

  LOCAL slot_rows IS LIST(LIST(1, 2), LIST(3, 4), LIST(5)).
  LOCAL ri IS 0.
  UNTIL ri >= slot_rows:LENGTH {
    LOCAL srow IS GUI_WIN:ADDHBOX().
    LOCAL cols IS slot_rows[ri].
    LOCAL ci IS 0.
    UNTIL ci < cols:LENGTH {
      LOCAL sn IS cols[ci].
      LOCAL save_btn IS srow:ADDBUTTON("SAVE " + sn).
      SET save_btn:STYLE:WIDTH TO 76.
      LOCAL load_btn IS srow:ADDBUTTON("").
      SET load_btn:STYLE:WIDTH TO 76.
      GUI_SLOT_SAVE_BTNS:ADD(save_btn).
      GUI_SLOT_LOAD_BTNS:ADD(load_btn).
      SET ci TO ci + 1.
    }
    SET ri TO ri + 1.
  }

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
  IF GUI_WIN = 0 { RETURN "". }

  // ARM / QUIT.
  IF GUI_ARM_BTN  <> 0 AND GUI_ARM_BTN:TAKEPRESS  { RETURN "ARM".  }
  IF GUI_QUIT_BTN <> 0 AND GUI_QUIT_BTN:TAKEPRESS { RETURN "QUIT". }

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
      _GUI_REFRESH_EDIT().
      _GUI_REFRESH().
      RETURN "".
    }
    IF GUI_LEG_UP_BTNS[li]:TAKEPRESS {
      IF li > 0 {
        _GUI_SWAP_LEGS(li, li - 1).
        SET FMS_LEG_CURSOR TO li - 1.
        _GUI_REFRESH_EDIT().
        _GUI_REFRESH().
      }
      RETURN "".
    }
    IF GUI_LEG_DN_BTNS[li]:TAKEPRESS {
      IF li < DRAFT_PLAN:LENGTH - 1 {
        _GUI_SWAP_LEGS(li, li + 1).
        SET FMS_LEG_CURSOR TO li + 1.
        _GUI_REFRESH_EDIT().
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
  LOCAL si IS 0.
  UNTIL si >= GUI_SLOT_SAVE_BTNS:LENGTH {
    IF GUI_SLOT_SAVE_BTNS[si]:TAKEPRESS {
      FMS_SAVE_PLAN(si + 1).
      _GUI_REFRESH().
      RETURN "".
    }
    IF GUI_SLOT_LOAD_BTNS[si]:TAKEPRESS {
      FMS_LOAD_PLAN(si + 1).
      _GUI_BUILD_EDIT().
      _GUI_REFRESH().
      RETURN "".
    }
    SET si TO si + 1.
  }

  RETURN "".
}
