@LAZYGLOBAL OFF.

// ============================================================
// ifc_gui.ks  -  Integrated Flight Computer
//
// kOS GUI-based flight plan editor for PHASE_PREARM.
//
// Public entry points:
//   _GUI_BUILD()  - create (or rebuild) the editor window
//   _GUI_CLOSE()  - dispose the window and clear all handles
//   _GUI_TICK()   - call each loop tick; returns "" / "ARM" / "QUIT"
//
// The GUI floats alongside the terminal display.  DISPLAY_TICK()
// continues to run for the header and status bar.
// ============================================================

// ── Internal: swap two adjacent legs ─────────────────────
FUNCTION _GUI_SWAP_LEGS {
  PARAMETER a, b.
  LOCAL tmp IS DRAFT_PLAN[a].
  SET DRAFT_PLAN[a] TO DRAFT_PLAN[b].
  SET DRAFT_PLAN[b] TO tmp.
}

// ── Close / dispose ───────────────────────────────────────
FUNCTION _GUI_CLOSE {
  IF GUI_WIN <> 0 {
    GUI_WIN:DISPOSE().
    SET GUI_WIN TO 0.
  }
  SET GUI_ADD_BTN  TO 0.
  SET GUI_ARM_BTN  TO 0.
  SET GUI_QUIT_BTN TO 0.
  GUI_LEG_SEL_BTNS:CLEAR().
  GUI_LEG_UP_BTNS:CLEAR().
  GUI_LEG_DN_BTNS:CLEAR().
  GUI_LEG_DEL_BTNS:CLEAR().
  GUI_LEG_SUM_LBLS:CLEAR().
  GUI_FIELD_LBLS:CLEAR().
  GUI_FIELD_DEC_BTNS:CLEAR().
  GUI_FIELD_INC_BTNS:CLEAR().
  GUI_SLOT_SAVE_BTNS:CLEAR().
  GUI_SLOT_LOAD_BTNS:CLEAR().
}

// ── Refresh edit-panel value labels in place (no rebuild) ─
// Called after a non-type field is changed.
FUNCTION _GUI_REFRESH_FIELDS {
  IF DRAFT_PLAN:LENGTH = 0 OR GUI_FIELD_LBLS:LENGTH = 0 { RETURN. }
  LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, DRAFT_PLAN:LENGTH - 1).
  LOCAL leg IS DRAFT_PLAN[cur].
  LOCAL fi IS 0.
  UNTIL fi >= GUI_FIELD_LBLS:LENGTH {
    SET GUI_FIELD_LBLS[fi]:TEXT TO _FMS_LEG_FIELD_VALUE_TEXT(leg, fi).
    SET fi TO fi + 1.
  }
  // Also refresh this leg's summary label in the leg list.
  IF cur < GUI_LEG_SUM_LBLS:LENGTH {
    SET GUI_LEG_SUM_LBLS[cur]:TEXT TO ">  " + _FMS_LEG_LINE_TEXT(leg).
  }
}

// ── Build / rebuild the full editor window ────────────────
FUNCTION _GUI_BUILD {
  _GUI_CLOSE().

  SET GUI_WIN TO GUI(370).

  // ── Header ──────────────────────────────────────────────
  LOCAL hdr IS GUI_WIN:ADDLABEL("  IFC FLIGHT PLAN EDITOR").
  SET hdr:STYLE:FONTSIZE TO 13.
  SET hdr:STYLE:ALIGN    TO "CENTER".

  // ── Leg list ────────────────────────────────────────────
  LOCAL n IS DRAFT_PLAN:LENGTH.

  LOCAL pl_row IS GUI_WIN:ADDHBOX().
  LOCAL pl_lbl IS pl_row:ADDLABEL("FLIGHT PLAN  (" + n + " legs)").
  SET pl_lbl:STYLE:WIDTH TO 220.
  SET GUI_ADD_BTN TO pl_row:ADDBUTTON("+ ADD LEG").
  SET GUI_ADD_BTN:ENABLED TO n < 8.

  GUI_LEG_SEL_BTNS:CLEAR().
  GUI_LEG_UP_BTNS:CLEAR().
  GUI_LEG_DN_BTNS:CLEAR().
  GUI_LEG_DEL_BTNS:CLEAR().
  GUI_LEG_SUM_LBLS:CLEAR().

  LOCAL li IS 0.
  UNTIL li >= n {
    LOCAL lrow IS GUI_WIN:ADDHBOX().

    LOCAL sel_btn IS lrow:ADDBUTTON("" + (li + 1)).
    SET sel_btn:STYLE:WIDTH TO 26.

    LOCAL up_btn IS lrow:ADDBUTTON("^").
    SET up_btn:STYLE:WIDTH TO 26.
    SET up_btn:ENABLED TO li > 0.

    LOCAL dn_btn IS lrow:ADDBUTTON("v").
    SET dn_btn:STYLE:WIDTH TO 26.
    SET dn_btn:ENABLED TO li < n - 1.

    LOCAL del_btn IS lrow:ADDBUTTON("X").
    SET del_btn:STYLE:WIDTH TO 26.
    SET del_btn:ENABLED TO n > 1.

    LOCAL pfx IS "   ".
    IF li = FMS_LEG_CURSOR { SET pfx TO ">  ". }
    LOCAL sum_lbl IS lrow:ADDLABEL(pfx + _FMS_LEG_LINE_TEXT(DRAFT_PLAN[li])).

    GUI_LEG_SEL_BTNS:ADD(sel_btn).
    GUI_LEG_UP_BTNS:ADD(up_btn).
    GUI_LEG_DN_BTNS:ADD(dn_btn).
    GUI_LEG_DEL_BTNS:ADD(del_btn).
    GUI_LEG_SUM_LBLS:ADD(sum_lbl).

    SET li TO li + 1.
  }

  // ── Edit panel ───────────────────────────────────────────
  GUI_WIN:ADDLABEL(" ").

  GUI_FIELD_LBLS:CLEAR().
  GUI_FIELD_DEC_BTNS:CLEAR().
  GUI_FIELD_INC_BTNS:CLEAR().

  IF n > 0 {
    LOCAL cur IS CLAMP(FMS_LEG_CURSOR, 0, n - 1).
    LOCAL ehdr IS GUI_WIN:ADDLABEL("  EDIT LEG " + (cur + 1)).
    SET ehdr:STYLE:FONTSIZE TO 11.

    LOCAL leg IS DRAFT_PLAN[cur].
    LOCAL fc IS _FMS_LEG_FIELD_COUNT(leg).
    LOCAL fi IS 0.
    UNTIL fi >= fc {
      LOCAL frow IS GUI_WIN:ADDHBOX().

      LOCAL flbl IS frow:ADDLABEL(_FMS_LEG_FIELD_LABEL(leg, fi)).
      SET flbl:STYLE:WIDTH TO 95.

      LOCAL dec_btn IS frow:ADDBUTTON("<").
      SET dec_btn:STYLE:WIDTH TO 30.

      LOCAL val_lbl IS frow:ADDLABEL(_FMS_LEG_FIELD_VALUE_TEXT(leg, fi)).
      SET val_lbl:STYLE:WIDTH TO 130.
      SET val_lbl:STYLE:ALIGN TO "CENTER".

      LOCAL inc_btn IS frow:ADDBUTTON(">").
      SET inc_btn:STYLE:WIDTH TO 30.

      GUI_FIELD_LBLS:ADD(val_lbl).
      GUI_FIELD_DEC_BTNS:ADD(dec_btn).
      GUI_FIELD_INC_BTNS:ADD(inc_btn).

      SET fi TO fi + 1.
    }
  }

  // ── Save / Load ──────────────────────────────────────────
  GUI_WIN:ADDLABEL(" ").
  GUI_WIN:ADDLABEL("  SAVE / LOAD").

  GUI_SLOT_SAVE_BTNS:CLEAR().
  GUI_SLOT_LOAD_BTNS:CLEAR().

  // Two slots per row: (1,2), (3,4), (5)
  LOCAL slot_rows IS LIST(LIST(1, 2), LIST(3, 4), LIST(5)).
  LOCAL ri IS 0.
  UNTIL ri >= slot_rows:LENGTH {
    LOCAL srow IS GUI_WIN:ADDHBOX().
    LOCAL cols IS slot_rows[ri].
    LOCAL ci IS 0.
    UNTIL ci >= cols:LENGTH {
      LOCAL sn IS cols[ci].
      LOCAL save_btn IS srow:ADDBUTTON("SAVE " + sn).
      SET save_btn:STYLE:WIDTH TO 76.
      LOCAL lmark IS "  ".
      IF FMS_SLOT_EXISTS(sn) { SET lmark TO "* ". }
      LOCAL load_btn IS srow:ADDBUTTON("LOAD " + sn + lmark).
      SET load_btn:STYLE:WIDTH TO 76.
      GUI_SLOT_SAVE_BTNS:ADD(save_btn).
      GUI_SLOT_LOAD_BTNS:ADD(load_btn).
      SET ci TO ci + 1.
    }
    SET ri TO ri + 1.
  }

  // ── ARM / QUIT ───────────────────────────────────────────
  GUI_WIN:ADDLABEL(" ").
  LOCAL aq_row IS GUI_WIN:ADDHBOX().
  SET GUI_ARM_BTN  TO aq_row:ADDBUTTON("     ARM     ").
  SET GUI_ARM_BTN:STYLE:WIDTH  TO 160.
  SET GUI_QUIT_BTN TO aq_row:ADDBUTTON("     QUIT     ").
  SET GUI_QUIT_BTN:STYLE:WIDTH TO 160.

  GUI_WIN:SHOW().
}

// ── Poll widgets each loop tick ───────────────────────────
// Returns "" (normal), "ARM", or "QUIT".
FUNCTION _GUI_TICK {
  IF GUI_WIN = 0 { RETURN "". }

  // ARM / QUIT — checked first so they always respond.
  IF GUI_ARM_BTN  <> 0 AND GUI_ARM_BTN:TAKEPRESS  { RETURN "ARM".  }
  IF GUI_QUIT_BTN <> 0 AND GUI_QUIT_BTN:TAKEPRESS { RETURN "QUIT". }

  // Add leg.
  IF GUI_ADD_BTN <> 0 AND GUI_ADD_BTN:TAKEPRESS {
    IF DRAFT_PLAN:LENGTH < 8 {
      DRAFT_PLAN:ADD(_FMS_DEFAULT_LEG(LEG_CRUISE)).
      SET FMS_LEG_CURSOR TO DRAFT_PLAN:LENGTH - 1.
    }
    _GUI_BUILD().
    RETURN "".
  }

  // Leg list row buttons.
  LOCAL li IS 0.
  UNTIL li >= GUI_LEG_SEL_BTNS:LENGTH {
    // Select.
    IF GUI_LEG_SEL_BTNS[li]:TAKEPRESS {
      SET FMS_LEG_CURSOR TO li.
      _GUI_BUILD().
      RETURN "".
    }
    // Move up.
    IF GUI_LEG_UP_BTNS[li]:TAKEPRESS {
      IF li > 0 {
        _GUI_SWAP_LEGS(li, li - 1).
        SET FMS_LEG_CURSOR TO li - 1.
        _GUI_BUILD().
      }
      RETURN "".
    }
    // Move down.
    IF GUI_LEG_DN_BTNS[li]:TAKEPRESS {
      IF li < DRAFT_PLAN:LENGTH - 1 {
        _GUI_SWAP_LEGS(li, li + 1).
        SET FMS_LEG_CURSOR TO li + 1.
        _GUI_BUILD().
      }
      RETURN "".
    }
    // Delete.
    IF GUI_LEG_DEL_BTNS[li]:TAKEPRESS {
      IF DRAFT_PLAN:LENGTH > 1 {
        DRAFT_PLAN:REMOVE(li).
        IF FMS_LEG_CURSOR >= DRAFT_PLAN:LENGTH {
          SET FMS_LEG_CURSOR TO DRAFT_PLAN:LENGTH - 1.
        }
        _GUI_BUILD().
      }
      RETURN "".
    }
    SET li TO li + 1.
  }

  // Edit panel [<] / [>] buttons.
  LOCAL fi IS 0.
  UNTIL fi >= GUI_FIELD_LBLS:LENGTH {
    LOCAL dec_pressed IS GUI_FIELD_DEC_BTNS[fi]:TAKEPRESS.
    LOCAL inc_pressed IS GUI_FIELD_INC_BTNS[fi]:TAKEPRESS.
    IF dec_pressed OR inc_pressed {
      LOCAL dir IS 1.
      IF dec_pressed { SET dir TO -1. }
      _FMS_LEG_CHANGE_FIELD(FMS_LEG_CURSOR, fi, dir).
      // Type field (fi=0) changes leg structure — full rebuild needed.
      // Other fields — update labels in place.
      IF fi = 0 { _GUI_BUILD(). }
      ELSE      { _GUI_REFRESH_FIELDS(). }
      RETURN "".
    }
    SET fi TO fi + 1.
  }

  // Save / Load slot buttons.
  LOCAL si IS 0.
  UNTIL si >= GUI_SLOT_SAVE_BTNS:LENGTH {
    IF GUI_SLOT_SAVE_BTNS[si]:TAKEPRESS {
      FMS_SAVE_PLAN(si + 1).
      SET GUI_SLOT_LOAD_BTNS[si]:TEXT TO "LOAD " + (si + 1) + "* ".
      RETURN "".
    }
    IF GUI_SLOT_LOAD_BTNS[si]:TAKEPRESS {
      FMS_LOAD_PLAN(si + 1).
      _GUI_BUILD().
      RETURN "".
    }
    SET si TO si + 1.
  }

  RETURN "".
}
