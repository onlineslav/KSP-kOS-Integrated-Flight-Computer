@LAZYGLOBAL OFF.

// ============================================================
// ifc_fms.ks  -  Integrated Flight Computer
// Save/load DRAFT_PLAN by name.
//
// File format v3: {"version":3, "name":"<display_name>", "legs":[...]}
// Waypoints stored as beacon ID strings (e.g. "WPT_APT_KSC").
// Backward compat: v2 files loaded; integer wpt indices converted to IDs.
// Files stored as:  0:/ifc_plan_<safe_name>.json
// ============================================================

GLOBAL FMS_PLAN_PREFIX IS "0:/ifc_plan_".
GLOBAL FMS_PLAN_SUFFIX IS ".json".
GLOBAL FMS_PLAN_INDEX_PATH IS "0:/ifc_plan_index.json".

// Sanitize a display name into a safe filename stem.
// Spaces become underscores; non-alnum/underscore/hyphen chars stripped.
// Result capped at 32 chars; falls back to "plan" if empty.
FUNCTION _FMS_SANITIZE_NAME {
  PARAMETER raw.
  LOCAL s IS ("" + raw):TRIM:REPLACE(" ", "_").
  LOCAL valid IS "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-".
  LOCAL safe IS "".
  LOCAL i IS 0.
  UNTIL i >= s:LENGTH {
    LOCAL ch IS s:SUBSTRING(i, 1).
    IF valid:INDEXOF(ch) >= 0 { SET safe TO safe + ch. }
    SET i TO i + 1.
  }
  IF safe:LENGTH = 0 { RETURN "plan". }
  IF safe:LENGTH > 32 { RETURN safe:SUBSTRING(0, 32). }
  RETURN safe.
}

FUNCTION _FMS_NAME_PATH {
  PARAMETER name.
  RETURN FMS_PLAN_PREFIX + _FMS_SANITIZE_NAME(name) + FMS_PLAN_SUFFIX.
}

FUNCTION _FMS_READ_PLAN_INDEX {
  LOCAL plans IS LIST().
  IF NOT EXISTS(FMS_PLAN_INDEX_PATH) { RETURN plans. }
  LOCAL data IS READJSON(FMS_PLAN_INDEX_PATH).
  LOCAL src IS LIST().
  IF data:TYPENAME = "List" OR data:TYPENAME = "ListValue" {
    SET src TO data.
  } ELSE {
    LOCAL is_lex IS data:TYPENAME = "Lexicon" OR data:TYPENAME = "LexiconValue".
    IF NOT is_lex OR NOT data:HASKEY("plans") { RETURN plans. }
    LOCAL psrc IS data["plans"].
    IF psrc:TYPENAME = "List" OR psrc:TYPENAME = "ListValue" {
      SET src TO psrc.
    } ELSE {
      RETURN plans.
    }
  }

  LOCAL i IS 0.
  UNTIL i >= src:LENGTH {
    LOCAL n IS src[i].
    IF n:TYPENAME = "String" {
      LOCAL clean IS n:TRIM.
      IF clean <> "" AND NOT plans:CONTAINS(clean) {
        plans:ADD(clean).
      }
    }
    SET i TO i + 1.
  }
  RETURN plans.
}

FUNCTION _FMS_WRITE_PLAN_INDEX {
  PARAMETER plans.
  WRITEJSON(plans, FMS_PLAN_INDEX_PATH).
}

// Serialise one DRAFT_PLAN leg to a plain LEXICON safe for WRITEJSON.
FUNCTION _FMS_SERIALISE_LEG {
  PARAMETER leg.
  LOCAL t IS leg["type"].
  LOCAL p IS leg["params"].

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

  IF t = LEG_TAKEOFF {
    LOCAL rwy_idx IS 0.
    IF p:HASKEY("rwy_idx") { SET rwy_idx TO ROUND(p["rwy_idx"], 0). }
    RETURN LEXICON("type", LEG_TAKEOFF,
      "params", LEXICON("rwy_idx", CLAMP(rwy_idx, 0, 1))).

  } ELSE IF t = LEG_CRUISE {
    LOCAL alt_m    IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd      IS CRUISE_DEFAULT_SPD.
    LOCAL spd_mode IS CRUISE_SPD_MODE_IAS.
    LOCAL nav_type IS "waypoint".
    IF p:HASKEY("alt_m")    { SET alt_m    TO p["alt_m"]. }
    IF p:HASKEY("spd_mode") { SET spd_mode TO CRUISE_NORM_SPD_MODE(p["spd_mode"]). }
    IF p:HASKEY("spd")      { SET spd      TO p["spd"]. }
    IF p:HASKEY("nav_type") { SET nav_type TO p["nav_type"]. }
    LOCAL cp IS LEXICON("alt_m", alt_m, "spd_mode", spd_mode, "spd", spd, "nav_type", nav_type).
    LOCAL wi IS 0.
    UNTIL wi >= FMS_WPT_SLOTS {
      LOCAL wkey IS "wpt" + wi.
      LOCAL wid  IS "".
      IF p:HASKEY(wkey) {
        LOCAL raw IS p[wkey].
        // v2 compatibility: integer index -> beacon ID string
        IF raw:TYPENAME = "Scalar" {
          LOCAL idx IS ROUND(raw, 0).
          IF idx >= 0 AND idx < CUSTOM_WPT_IDS:LENGTH {
            SET wid TO CUSTOM_WPT_IDS[idx].
          }
        } ELSE IF raw:TYPENAME = "String" {
          SET wid TO raw.
        }
      }
      cp:ADD(wkey, wid).
      SET wi TO wi + 1.
    }
    IF p:HASKEY("course_deg") { cp:ADD("course_deg", ROUND(p["course_deg"], 0)). }
    IF p:HASKEY("dist_nm")    { cp:ADD("dist_nm",    ROUND(p["dist_nm"],    0)). }
    IF p:HASKEY("time_min")   { cp:ADD("time_min",   ROUND(p["time_min"],   0)). }
    RETURN LEXICON("type", LEG_CRUISE, "params", cp).

  } ELSE IF t = LEG_APPROACH {
    LOCAL ap IS LEXICON().
    IF p:HASKEY("plate_id")      { SET ap["plate_id"]      TO p["plate_id"]. }
    IF p:HASKEY("plate_idx")     { SET ap["plate_idx"]     TO ROUND(p["plate_idx"],     0). }
    IF p:HASKEY("airport_idx")   { SET ap["airport_idx"]   TO ROUND(p["airport_idx"],   0). }
    IF p:HASKEY("plate_sel_idx") { SET ap["plate_sel_idx"] TO ROUND(p["plate_sel_idx"], 0). }
    IF ap:LENGTH = 0 { SET ap["plate_idx"] TO 0. }
    _FMS_AP_NORMALISE_PARAMS(ap).
    RETURN LEXICON("type", LEG_APPROACH, "params", ap).
  }

  RETURN 0.  // Unknown leg type.
}

FUNCTION FMS_SAVE_PLAN {
  PARAMETER name IS "plan".
  LOCAL display_name IS ("" + name):TRIM.
  IF display_name = "" { SET display_name TO "plan". }
  LOCAL file_path IS _FMS_NAME_PATH(display_name).

  LOCAL legs IS LIST().
  LOCAL i IS 0.
  UNTIL i >= DRAFT_PLAN:LENGTH {
    legs:ADD(_FMS_SERIALISE_LEG(DRAFT_PLAN[i])).
    SET i TO i + 1.
  }

  LOCAL snapshot IS LEXICON(
    "version", 3,
    "name",    display_name,
    "legs",    legs
  ).
  WRITEJSON(snapshot, file_path).

  LOCAL plans IS _FMS_READ_PLAN_INDEX().
  IF NOT plans:CONTAINS(display_name) {
    plans:ADD(display_name).
    _FMS_WRITE_PLAN_INDEX(plans).
  }

  SET FMS_LAST_SAVE_NAME TO display_name.
  IFC_SET_ALERT("Plan saved: " + display_name).
  RETURN TRUE.
}

FUNCTION FMS_LOAD_PLAN {
  PARAMETER name IS "plan".
  LOCAL file_path IS _FMS_NAME_PATH(name).
  LOCAL display_name IS name.

  IF NOT EXISTS(file_path) {
    IFC_SET_ALERT("No saved plan: " + name).
    RETURN FALSE.
  }

  LOCAL p IS READJSON(file_path).
  IF p:HASKEY("name") AND p["name"]:TYPENAME = "String" AND p["name"] <> "" {
    SET display_name TO p["name"].
  }

  IF NOT p:HASKEY("version") OR p["version"] < 2 {
    IFC_SET_ALERT("Plan " + name + ": old format, re-save needed").
    RETURN FALSE.
  }

  IF NOT p:HASKEY("legs") {
    IFC_SET_ALERT("Plan " + name + ": corrupt file").
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
    IFC_SET_ALERT("Plan " + name + ": no valid legs").
    RETURN FALSE.
  }

  DRAFT_PLAN:CLEAR().
  SET i TO 0.
  UNTIL i >= new_plan:LENGTH {
    DRAFT_PLAN:ADD(new_plan[i]).
    SET i TO i + 1.
  }

  SET FMS_LEG_CURSOR    TO 0.
  SET FMS_EDIT_FIELD    TO 0.
  SET FMS_EDITING_LEG   TO FALSE.
  SET LAST_DISPLAY_UT   TO 0.
  SET FMS_LAST_SAVE_NAME TO display_name.

  LOCAL plans IS _FMS_READ_PLAN_INDEX().
  IF NOT plans:CONTAINS(display_name) {
    plans:ADD(display_name).
    _FMS_WRITE_PLAN_INDEX(plans).
  }

  IFC_SET_ALERT("Plan loaded: " + display_name).
  RETURN TRUE.
}

FUNCTION FMS_PLAN_EXISTS {
  PARAMETER name.
  RETURN EXISTS(_FMS_NAME_PATH(name)).
}

// Returns a LIST of saved plan names from the plan index file.
FUNCTION FMS_LIST_PLANS {
  RETURN _FMS_READ_PLAN_INDEX().
}
