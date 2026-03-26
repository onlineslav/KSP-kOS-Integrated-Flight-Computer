@LAZYGLOBAL OFF.

// ============================================================
// ifc_fms.ks  -  Integrated Flight Computer
// Save/load DRAFT_PLAN by slot.
//
// File format v2: {"version":2, "slot":N, "legs":[...]}
// Each leg: {"type":"TAKEOFF"|"CRUISE"|"APPROACH", "params":{...}}
// Backward compat: v1 files (menu options) are silently ignored
// with a warning — they cannot be converted without user input.
// ============================================================

GLOBAL FMS_SLOT_MIN IS 1.
GLOBAL FMS_SLOT_MAX IS 5.
GLOBAL FMS_SLOT_PREFIX IS "0:/ifc_plan_".
GLOBAL FMS_SLOT_SUFFIX IS ".json".

FUNCTION _FMS_CLAMP_SLOT {
  PARAMETER slot.
  RETURN CLAMP(ROUND(slot, 0), FMS_SLOT_MIN, FMS_SLOT_MAX).
}

FUNCTION _FMS_PATH {
  PARAMETER slot.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  RETURN FMS_SLOT_PREFIX + s + FMS_SLOT_SUFFIX.
}

// Serialise one DRAFT_PLAN leg to a plain LEXICON safe for WRITEJSON.
FUNCTION _FMS_SERIALISE_LEG {
  PARAMETER leg.
  LOCAL t IS leg["type"].
  LOCAL p IS leg["params"].

  // Deep-copy params into a fresh LEXICON (kOS WRITEJSON handles LEXICON/scalar).
  LOCAL sp IS LEXICON().
  FOR k IN p:KEYS { sp:ADD(k, p[k]). }

  RETURN LEXICON("type", t, "params", sp).
}

// Reconstruct one DRAFT_PLAN leg from a deserialised LEXICON.
// Returns 0 if the entry is invalid.
FUNCTION _FMS_DESERIALISE_LEG {
  PARAMETER entry.
  IF NOT entry:HASKEY("type")   { RETURN 0. }
  IF NOT entry:HASKEY("params") { RETURN 0. }
  LOCAL t IS entry["type"].
  LOCAL p IS entry["params"].

  // Validate type and rebuild params with correct types/defaults.
  IF t = LEG_TAKEOFF {
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO ROUND(p["rwy_idx"], 0). }
    RETURN LEXICON("type", LEG_TAKEOFF,
      "params", LEXICON("rwy_idx", CLAMP(rwy_idx, 0, 1))).

  } ELSE IF t = LEG_CRUISE {
    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd   IS CRUISE_DEFAULT_SPD.
    LOCAL spd_mode IS CRUISE_SPD_MODE_IAS.
    LOCAL nav_type IS "waypoint".
    IF p:HASKEY("alt_m") { SET alt_m TO p["alt_m"]. }
    IF p:HASKEY("spd_mode") { SET spd_mode TO CRUISE_NORM_SPD_MODE(p["spd_mode"]). }
    IF p:HASKEY("spd")   { SET spd   TO p["spd"]. }
    IF p:HASKEY("nav_type") { SET nav_type TO p["nav_type"]. }
    LOCAL cp IS LEXICON("alt_m", alt_m, "spd_mode", spd_mode, "spd", spd, "nav_type", nav_type).
    LOCAL wi IS 0.
    UNTIL wi >= FMS_WPT_SLOTS {
      LOCAL wkey IS "wpt" + wi.
      LOCAL widx IS -1.
      IF p:HASKEY(wkey) { SET widx TO ROUND(p[wkey], 0). }
      cp:ADD(wkey, widx).
      SET wi TO wi + 1.
    }
    IF p:HASKEY("course_deg") { SET cp["course_deg"] TO ROUND(p["course_deg"], 0). }
    IF p:HASKEY("dist_nm")    { SET cp["dist_nm"] TO ROUND(p["dist_nm"], 0). }
    IF p:HASKEY("time_min")   { SET cp["time_min"] TO ROUND(p["time_min"], 0). }
    RETURN LEXICON("type", LEG_CRUISE, "params", cp).

  } ELSE IF t = LEG_APPROACH {
    LOCAL ap IS LEXICON().
    IF p:HASKEY("plate_id") { SET ap["plate_id"] TO p["plate_id"]. }
    IF p:HASKEY("plate_idx") { SET ap["plate_idx"] TO ROUND(p["plate_idx"], 0). }
    IF p:HASKEY("airport_idx") { SET ap["airport_idx"] TO ROUND(p["airport_idx"], 0). }
    IF p:HASKEY("plate_sel_idx") { SET ap["plate_sel_idx"] TO ROUND(p["plate_sel_idx"], 0). }
    IF ap:LENGTH = 0 { SET ap["plate_idx"] TO 0. }
    _FMS_AP_NORMALISE_PARAMS(ap).
    RETURN LEXICON("type", LEG_APPROACH,
      "params", ap).
  }

  RETURN 0.  // Unknown leg type.
}

FUNCTION FMS_SAVE_PLAN {
  PARAMETER slot IS 1.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  LOCAL slot_path IS _FMS_PATH(s).

  LOCAL legs IS LIST().
  LOCAL i IS 0.
  UNTIL i >= DRAFT_PLAN:LENGTH {
    legs:ADD(_FMS_SERIALISE_LEG(DRAFT_PLAN[i])).
    SET i TO i + 1.
  }

  LOCAL snapshot IS LEXICON(
    "version", 2,
    "slot",    s,
    "legs",    legs
  ).
  WRITEJSON(snapshot, slot_path).
  IFC_SET_ALERT("Plan saved: slot " + s).
  RETURN TRUE.
}

FUNCTION FMS_LOAD_PLAN {
  PARAMETER slot IS 1.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  LOCAL slot_path IS _FMS_PATH(s).

  IF NOT EXISTS(slot_path) {
    IFC_SET_ALERT("No saved plan in slot " + s, "WARN").
    RETURN FALSE.
  }

  LOCAL p IS READJSON(slot_path).

  // v1 files lack "version" or have version < 2 — cannot restore.
  IF NOT p:HASKEY("version") OR p["version"] < 2 {
    IFC_SET_ALERT("Slot " + s + ": old format, re-save needed", "WARN").
    RETURN FALSE.
  }

  IF NOT p:HASKEY("legs") {
    IFC_SET_ALERT("Slot " + s + ": corrupt file", "WARN").
    RETURN FALSE.
  }

  LOCAL raw_legs IS p["legs"].
  LOCAL new_plan IS LIST().
  LOCAL i IS 0.
  UNTIL i >= raw_legs:LENGTH {
    LOCAL leg IS _FMS_DESERIALISE_LEG(raw_legs[i]).
    IF leg <> 0 { new_plan:ADD(leg). }
    SET i TO i + 1.
  }

  IF new_plan:LENGTH = 0 {
    IFC_SET_ALERT("Slot " + s + ": no valid legs", "WARN").
    RETURN FALSE.
  }

  // Replace DRAFT_PLAN contents.
  DRAFT_PLAN:CLEAR().
  SET i TO 0.
  UNTIL i >= new_plan:LENGTH {
    DRAFT_PLAN:ADD(new_plan[i]).
    SET i TO i + 1.
  }

  SET FMS_LEG_CURSOR TO 0.
  SET FMS_EDIT_FIELD  TO 0.
  SET FMS_EDITING_LEG TO FALSE.
  SET LAST_DISPLAY_UT TO 0.

  IFC_SET_ALERT("Plan loaded: slot " + s).
  RETURN TRUE.
}

FUNCTION FMS_SLOT_EXISTS {
  PARAMETER slot.
  RETURN EXISTS(_FMS_PATH(slot)).
}
