@LAZYGLOBAL OFF.

// ============================================================
// ifc_autobrake.ks  -  Integrated Flight Computer
//
// Auto-brake module:
// - Discovers tagged main-gear wheel-brake modules once
// - Maintains left/right brake binding banks
// - Supports legacy single-tag main-gear setups by splitting by side
// - Exposes per-side brake command API for AMO differential steering
// ============================================================

FUNCTION ABRK_RESET {
  SET ABRK_DISCOVERED TO FALSE.
  SET ABRK_AVAILABLE TO FALSE.
  SET ABRK_WARNED_NO_PARTS TO FALSE.
  ABRK_LEFT_BINDINGS:CLEAR().
  ABRK_RIGHT_BINDINGS:CLEAR().
  ABRK_NOSE_BINDINGS:CLEAR().
  SET ABRK_LAST_LEFT_CMD TO 0.
  SET ABRK_LAST_RIGHT_CMD TO 0.
}

FUNCTION _ABRK_CFG_STR {
  PARAMETER key, fallback.
  IF ACTIVE_AIRCRAFT = 0 OR NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL cfg_val IS ACTIVE_AIRCRAFT[key].
  IF cfg_val = 0 { RETURN fallback. }
  LOCAL txt IS ("" + cfg_val):TRIM.
  IF txt = "" { RETURN fallback. }
  RETURN txt.
}

FUNCTION _ABRK_CFG_NUM {
  PARAMETER key, fallback.
  IF ACTIVE_AIRCRAFT = 0 OR NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL cfg_val IS ACTIVE_AIRCRAFT[key].
  IF cfg_val:TYPENAME <> "Scalar" { RETURN fallback. }
  RETURN cfg_val.
}

FUNCTION _ABRK_GET_GEAR_TAG_BASE {
  LOCAL tag IS FLARE_MAIN_GEAR_TAG_DEFAULT.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flare_gear_tag") {
    LOCAL ac_tag IS ACTIVE_AIRCRAFT["flare_gear_tag"].
    IF ac_tag <> "" { SET tag TO ac_tag. }
  }
  IF tag:LENGTH >= 2 {
    LOCAL sfx IS tag:SUBSTRING(tag:LENGTH - 2, 2):TOUPPER.
    IF sfx = "_L" OR sfx = "_R" {
      RETURN tag:SUBSTRING(0, tag:LENGTH - 2).
    }
  }
  RETURN tag.
}

FUNCTION _ABRK_GET_NOSE_TAG_BASE {
  // Optional explicit override.
  LOCAL tag IS _ABRK_CFG_STR("abrk_nose_tag", "").
  IF tag <> "" { RETURN tag. }

  // Default: derive from main-gear base tag.
  LOCAL main_tag IS _ABRK_GET_GEAR_TAG_BASE().
  IF main_tag <> "" {
    LOCAL low IS main_tag:TOLOWER.
    IF low:FIND("maingear") >= 0 {
      RETURN main_tag:REPLACE("maingear", "nosegear"):REPLACE("MAINGEAR", "NOSEGEAR"):REPLACE("MainGear", "NoseGear").
    }
  }
  RETURN "ifc_nosegear".
}

FUNCTION _ABRK_TAG_VARIANTS {
  PARAMETER base_tag.
  LOCAL tags IS LIST().
  IF base_tag = "" { RETURN tags. }

  LOCAL stems IS LIST(base_tag).
  IF base_tag:LENGTH > 4 {
    LOCAL pref IS base_tag:SUBSTRING(0, 4):TOLOWER.
    IF pref = "ifc_" {
      stems:ADD("ifs_" + base_tag:SUBSTRING(4, base_tag:LENGTH - 4)).
    } ELSE IF pref = "ifs_" {
      stems:ADD("ifc_" + base_tag:SUBSTRING(4, base_tag:LENGTH - 4)).
    }
  }

  LOCAL si IS 0.
  UNTIL si >= stems:LENGTH {
    LOCAL s IS stems[si].
    LOCAL cands IS LIST(s, s + "_L", s + "_R").
    LOCAL ci IS 0.
    UNTIL ci >= cands:LENGTH {
      LOCAL cand IS cands[ci].
      LOCAL seen IS FALSE.
      LOCAL ti IS 0.
      UNTIL ti >= tags:LENGTH OR seen {
        IF tags[ti] = cand { SET seen TO TRUE. }
        SET ti TO ti + 1.
      }
      IF NOT seen { tags:ADD(cand). }
      SET ci TO ci + 1.
    }
    SET si TO si + 1.
  }

  RETURN tags.
}

FUNCTION _ABRK_LIST_HAS_PART {
  PARAMETER lst, p.
  LOCAL i IS 0.
  UNTIL i >= lst:LENGTH {
    IF lst[i] = p { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _ABRK_BINDINGS_HAS_PART {
  PARAMETER lst, p.
  LOCAL i IS 0.
  UNTIL i >= lst:LENGTH {
    LOCAL b IS lst[i].
    IF b:HASKEY("part") AND b["part"] = p { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _ABRK_COLLECT_TAGGED_PARTS {
  PARAMETER tag_name.
  PARAMETER out_list.
  IF tag_name = "" { RETURN. }
  IF NOT SHIP:HASSUFFIX("PARTSTAGGED") { RETURN. }
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  LOCAL i IS 0.
  UNTIL i >= tagged:LENGTH {
    LOCAL p IS tagged[i].
    IF p <> 0 AND NOT _ABRK_LIST_HAS_PART(out_list, p) {
      out_list:ADD(p).
    }
    SET i TO i + 1.
  }
}

FUNCTION _ABRK_COLLECT_UNTAGGED_WHEEL_PARTS {
  PARAMETER out_list.
  IF NOT SHIP:HASSUFFIX("PARTS") { RETURN. }
  LOCAL parts IS SHIP:PARTS.
  LOCAL pi IS 0.
  UNTIL pi >= parts:LENGTH {
    LOCAL p IS parts[pi].
    SET pi TO pi + 1.
    IF p <> 0 {
      LOCAL mods IS _ABRK_MODULE_ENTRIES(p).
      LOCAL has_wheel IS FALSE.
      LOCAL mi IS 0.
      UNTIL mi >= mods:LENGTH OR has_wheel {
        LOCAL m IS mods[mi].
        SET mi TO mi + 1.
        IF m <> 0 {
          LOCAL mname IS "" + m.
          IF m:HASSUFFIX("NAME") AND m:NAME <> "" { SET mname TO m:NAME. }
          IF _ABRK_IS_WHEEL_MODULE_NAME(mname) { SET has_wheel TO TRUE. }
        }
      }
      IF has_wheel AND NOT _ABRK_LIST_HAS_PART(out_list, p) {
        out_list:ADD(p).
      }
    }
  }
}

FUNCTION _ABRK_IS_WHEEL_MODULE_NAME {
  PARAMETER mod_name.
  IF mod_name = "" { RETURN FALSE. }
  LOCAL n IS mod_name:TOLOWER.
  IF n = "modulewheelbrakes" { RETURN TRUE. }
  IF n = "modulewheelbase" { RETURN TRUE. }
  IF n:FIND("wheel") >= 0 { RETURN TRUE. }
  RETURN FALSE.
}

FUNCTION _ABRK_MODULE_ENTRIES {
  PARAMETER p.
  IF p = 0 { RETURN LIST(). }
  IF p:HASSUFFIX("MODULES") { RETURN p:MODULES. }
  IF p:HASSUFFIX("ALLMODULES") { RETURN p:ALLMODULES. }
  RETURN LIST().
}

FUNCTION _ABRK_ENTRY_NAME {
  PARAMETER module_entry.
  IF module_entry = 0 { RETURN "". }
  IF module_entry:HASSUFFIX("NAME") AND module_entry:NAME <> "" {
    RETURN module_entry:NAME.
  }
  RETURN "" + module_entry.
}

FUNCTION _ABRK_RESOLVE_MODULE_OBJECT {
  PARAMETER p, module_entry, module_name.

  IF module_entry <> 0 AND module_entry:HASSUFFIX("HASFIELD") {
    RETURN module_entry.
  }

  IF p <> 0 AND p:HASSUFFIX("GETMODULE") AND module_name <> "" {
    LOCAL probe_mod IS p:GETMODULE(module_name).
    IF probe_mod <> 0 { RETURN probe_mod. }
  }

  RETURN module_entry.
}

FUNCTION _ABRK_PART_HAS_MODULE_NAME {
  PARAMETER p, wanted_name.
  IF p = 0 OR wanted_name = "" { RETURN FALSE. }
  LOCAL wanted_lc IS wanted_name:TOLOWER.
  LOCAL mods IS _ABRK_MODULE_ENTRIES(p).
  LOCAL i IS 0.
  UNTIL i >= mods:LENGTH {
    LOCAL m IS mods[i].
    IF m <> 0 {
      LOCAL mname IS _ABRK_ENTRY_NAME(m).
      IF mname:TOLOWER = wanted_lc { RETURN TRUE. }
    }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _ABRK_CANDIDATE_MODULES {
  RETURN LIST(
    "ModuleWheelBrakes",
    "ModuleWheelBase",
    "KSPWheelBase",
    "FSwheel"
  ).
}

FUNCTION _ABRK_RESOLVE_BRAKE_FIELD {
  PARAMETER module_ref.
  IF module_ref = 0 OR NOT module_ref:HASSUFFIX("HASFIELD") { RETURN "". }
  LOCAL forced_field IS _ABRK_CFG_STR("abrk_field_name", "").
  IF forced_field <> "" AND module_ref:HASFIELD(forced_field) { RETURN forced_field. }
  LOCAL fields IS LIST(
    "brakeInput",
    "BrakeInput",
    "brake_input",
    "brake",
    "Brake",
    "brakes",
    "Brakes",
    "brakeTweakable",
    "Brake Tweakable",
    "brakeTorque",
    "Brake Torque",
    "BrakeForce",
    "brakeForce",
    "brakePower",
    "brake_force",
    "brakePercent",
    "brakeBias"
  ).
  LOCAL i IS 0.
  UNTIL i >= fields:LENGTH {
    LOCAL f IS fields[i].
    IF module_ref:HASFIELD(f) { RETURN f. }
    SET i TO i + 1.
  }

  // Final heuristic: if module exposes field enumeration, pick first field
  // whose name contains "brake".
  IF module_ref:HASSUFFIX("ALLFIELDS") {
    LOCAL af IS module_ref:ALLFIELDS.
    LOCAL fi IS 0.
    UNTIL fi >= af:LENGTH {
      LOCAL fname IS "" + af[fi].
      IF fname:TOLOWER:FIND("brake") >= 0 AND module_ref:HASFIELD(fname) { RETURN fname. }
      SET fi TO fi + 1.
    }
  } ELSE IF module_ref:HASSUFFIX("FIELDS") {
    LOCAL f2 IS module_ref:FIELDS.
    LOCAL fi2 IS 0.
    UNTIL fi2 >= f2:LENGTH {
      LOCAL fname2 IS "" + f2[fi2].
      IF fname2:TOLOWER:FIND("brake") >= 0 AND module_ref:HASFIELD(fname2) { RETURN fname2. }
      SET fi2 TO fi2 + 1.
    }
  }

  RETURN "".
}

FUNCTION _ABRK_BASE_FOR_FIELD {
  PARAMETER field_name.
  LOCAL forced_base IS _ABRK_CFG_NUM("abrk_field_base", -1).
  IF forced_base > 0 { RETURN forced_base. }
  LOCAL f IS field_name:TOLOWER.
  IF f:FIND("input") >= 0 { RETURN 1.0. }
  IF f = "brake" OR f = "brakes" { RETURN 1.0. }
  IF f:FIND("tweak") >= 0 { RETURN 100.0. }
  RETURN 100.0.
}

FUNCTION _ABRK_DEFAULT_STRENGTH {
  RETURN CLAMP(_ABRK_CFG_NUM("abrk_default_strength", ABRK_DEFAULT_STRENGTH), 0, 1).
}

FUNCTION _ABRK_DEFAULT_CMD_FOR_BINDING {
  PARAMETER b.
  IF b = 0 OR NOT b:HASKEY("field") { RETURN _ABRK_DEFAULT_STRENGTH(). }
  LOCAL f IS ("" + b["field"]):TOLOWER.
  IF f:FIND("input") >= 0 { RETURN 0.0. }
  RETURN _ABRK_DEFAULT_STRENGTH().
}

FUNCTION _ABRK_BINDING_FOR_PART {
  PARAMETER p.
  IF p = 0 { RETURN 0. }

  // Aircraft-specific override: module + field names from config.
  LOCAL forced_module IS _ABRK_CFG_STR("abrk_module_name", "").
  IF forced_module <> "" AND p:HASSUFFIX("GETMODULE") {
    LOCAL forced_ref IS p:GETMODULE(forced_module).
    IF forced_ref <> 0 {
      LOCAL forced_field IS _ABRK_RESOLVE_BRAKE_FIELD(forced_ref).
      IF forced_field <> "" {
        LOCAL forced_base IS _ABRK_BASE_FOR_FIELD(forced_field).
        RETURN LEXICON("part", p, "module", forced_ref, "field", forced_field, "base", forced_base).
      }
    }
  }

  LOCAL modules IS _ABRK_CANDIDATE_MODULES().
  LOCAL i IS 0.
  UNTIL i >= modules:LENGTH {
    LOCAL name IS modules[i].
    SET i TO i + 1.
    IF p:HASSUFFIX("GETMODULE") AND _ABRK_PART_HAS_MODULE_NAME(p, name) {
      LOCAL module_ref IS p:GETMODULE(name).
      IF module_ref <> 0 {
        LOCAL mod_name IS "" + module_ref.
        IF module_ref:HASSUFFIX("NAME") AND module_ref:NAME <> "" { SET mod_name TO module_ref:NAME. }
        IF _ABRK_IS_WHEEL_MODULE_NAME(mod_name) {
          LOCAL field_name IS _ABRK_RESOLVE_BRAKE_FIELD(module_ref).
          IF field_name <> "" {
            LOCAL base IS _ABRK_BASE_FOR_FIELD(field_name).
            RETURN LEXICON("part", p, "module", module_ref, "field", field_name, "base", base).
          }
        }
      }
    }
  }

  // Fallback for modded wheels: probe all exposed module names.
  LOCAL mods IS _ABRK_MODULE_ENTRIES(p).
  LOCAL mi IS 0.
  UNTIL mi >= mods:LENGTH {
    LOCAL m IS mods[mi].
    SET mi TO mi + 1.
    IF m <> 0 {
      LOCAL mname IS _ABRK_ENTRY_NAME(m).
      LOCAL module_obj IS _ABRK_RESOLVE_MODULE_OBJECT(p, m, mname).
      LOCAL field_name IS _ABRK_RESOLVE_BRAKE_FIELD(module_obj).
      IF field_name <> "" {
        LOCAL base IS _ABRK_BASE_FOR_FIELD(field_name).
        RETURN LEXICON("part", p, "module", module_obj, "field", field_name, "base", base).
      }
    }
  }

  RETURN 0.
}

FUNCTION _ABRK_ADD_PART_BINDING {
  PARAMETER p, bank.
  IF p = 0 OR _ABRK_BINDINGS_HAS_PART(bank, p) { RETURN. }
  LOCAL b IS _ABRK_BINDING_FOR_PART(p).
  IF b <> 0 { bank:ADD(b). }
}

FUNCTION _ABRK_PART_MODULE_SUMMARY {
  PARAMETER p.
  LOCAL mods IS _ABRK_MODULE_ENTRIES(p).
  LOCAL txt IS "".
  LOCAL i IS 0.
  UNTIL i >= mods:LENGTH OR i >= 4 {
    LOCAL nm IS _ABRK_ENTRY_NAME(mods[i]).
    IF nm = "" { SET nm TO "?". }
    IF txt = "" { SET txt TO nm. }
    ELSE { SET txt TO txt + "," + nm. }
    SET i TO i + 1.
  }
  IF mods:LENGTH > 4 { SET txt TO txt + ",...". }
  RETURN txt.
}

FUNCTION ABRK_DISCOVER_PARTS {
  IF ABRK_DISCOVERED { RETURN. }

  ABRK_LEFT_BINDINGS:CLEAR().
  ABRK_RIGHT_BINDINGS:CLEAR().
  ABRK_NOSE_BINDINGS:CLEAR().
  SET ABRK_AVAILABLE TO FALSE.
  SET ABRK_DISCOVERED TO TRUE.

  LOCAL base_tag IS _ABRK_GET_GEAR_TAG_BASE().
  IF base_tag = "" { RETURN. }
  LOCAL nose_tag IS _ABRK_GET_NOSE_TAG_BASE().

  LOCAL legacy_parts IS LIST().
  LOCAL left_parts IS LIST().
  LOCAL right_parts IS LIST().
  LOCAL nose_parts IS LIST().

  // Explicit side tags + legacy unsided tags across ifc_/ifs_ families.
  IF SHIP:HASSUFFIX("PARTSTAGGED") {
    LOCAL tags IS _ABRK_TAG_VARIANTS(base_tag).
    LOCAL ti IS 0.
    UNTIL ti >= tags:LENGTH {
      LOCAL tg IS tags[ti].
      SET ti TO ti + 1.
      IF tg:LENGTH >= 2 {
        LOCAL sfx IS tg:SUBSTRING(tg:LENGTH - 2, 2):TOUPPER.
        IF sfx = "_L" {
          _ABRK_COLLECT_TAGGED_PARTS(tg, left_parts).
        } ELSE IF sfx = "_R" {
          _ABRK_COLLECT_TAGGED_PARTS(tg, right_parts).
        } ELSE {
          _ABRK_COLLECT_TAGGED_PARTS(tg, legacy_parts).
        }
      } ELSE {
        _ABRK_COLLECT_TAGGED_PARTS(tg, legacy_parts).
      }
    }
  }

  // Optional nose-gear tag family for nose-brake suppression during differential steer.
  IF SHIP:HASSUFFIX("PARTSTAGGED") AND nose_tag <> "" {
    LOCAL nose_tags IS _ABRK_TAG_VARIANTS(nose_tag).
    LOCAL nti IS 0.
    UNTIL nti >= nose_tags:LENGTH {
      _ABRK_COLLECT_TAGGED_PARTS(nose_tags[nti], nose_parts).
      SET nti TO nti + 1.
    }
  }

  // Fallback for untagged craft: infer wheel parts by module names.
  IF left_parts:LENGTH = 0 AND right_parts:LENGTH = 0 AND legacy_parts:LENGTH = 0 {
    _ABRK_COLLECT_UNTAGGED_WHEEL_PARTS(legacy_parts).
    IF legacy_parts:LENGTH > 0 {
      IFC_SET_ALERT("ABRK: using untagged wheel fallback", "WARN").
    }
  }

  LOCAL right_vec IS SHIP:FACING:STARVECTOR.
  LOCAL ship_pos IS SHIP:POSITION.
  LOCAL eps_m IS AMO_ENGINE_SIDE_EPS_M.

  // Add explicit side-tagged parts first.
  LOCAL i IS 0.
  UNTIL i >= left_parts:LENGTH {
    _ABRK_ADD_PART_BINDING(left_parts[i], ABRK_LEFT_BINDINGS).
    SET i TO i + 1.
  }
  SET i TO 0.
  UNTIL i >= right_parts:LENGTH {
    _ABRK_ADD_PART_BINDING(right_parts[i], ABRK_RIGHT_BINDINGS).
    SET i TO i + 1.
  }
  SET i TO 0.
  UNTIL i >= nose_parts:LENGTH {
    _ABRK_ADD_PART_BINDING(nose_parts[i], ABRK_NOSE_BINDINGS).
    SET i TO i + 1.
  }

  // Add legacy parts by geometric side split.
  SET i TO 0.
  UNTIL i >= legacy_parts:LENGTH {
    LOCAL p IS legacy_parts[i].
    SET i TO i + 1.
    IF p <> 0 AND p:HASSUFFIX("POSITION") {
      LOCAL side_m IS VDOT(p:POSITION - ship_pos, right_vec).
      IF side_m <= -eps_m {
        _ABRK_ADD_PART_BINDING(p, ABRK_LEFT_BINDINGS).
      } ELSE IF side_m >= eps_m {
        _ABRK_ADD_PART_BINDING(p, ABRK_RIGHT_BINDINGS).
      }
    }
  }

  IF ABRK_LEFT_BINDINGS:LENGTH > 0 AND ABRK_RIGHT_BINDINGS:LENGTH > 0 {
    SET ABRK_AVAILABLE TO TRUE.
    ABRK_RELEASE().
  } ELSE IF NOT ABRK_WARNED_NO_PARTS {
    SET ABRK_WARNED_NO_PARTS TO TRUE.
    LOCAL cand_total IS left_parts:LENGTH + right_parts:LENGTH + legacy_parts:LENGTH.
    IFC_SET_ALERT("ABRK: no valid L/R brake bindings for tag '" + base_tag + "' (candidates=" + cand_total + ")", "WARN").
    IF cand_total > 0 {
      LOCAL sample_part IS 0.
      IF left_parts:LENGTH > 0 { SET sample_part TO left_parts[0]. }
      ELSE IF right_parts:LENGTH > 0 { SET sample_part TO right_parts[0]. }
      ELSE { SET sample_part TO legacy_parts[0]. }
      IFC_SET_ALERT("ABRK hint: set abrk_module_name/abrk_field_name. sample modules=" + _ABRK_PART_MODULE_SUMMARY(sample_part), "WARN").
    }
  }
}

FUNCTION _ABRK_SET_BINDING {
  PARAMETER b, cmd_frac.
  IF b = 0 OR NOT b:HASKEY("module") OR NOT b:HASKEY("field") OR NOT b:HASKEY("base") { RETURN. }
  LOCAL module_ref IS b["module"].
  LOCAL field_name IS b["field"].
  LOCAL base IS MAX(b["base"], 0).
  IF module_ref = 0 OR field_name = "" { RETURN. }
  IF NOT module_ref:HASSUFFIX("SETFIELD") { RETURN. }
  IF module_ref:HASSUFFIX("HASFIELD") AND NOT module_ref:HASFIELD(field_name) { RETURN. }
  module_ref:SETFIELD(field_name, ROUND(CLAMP(cmd_frac, 0, 1) * base, 3)).
}

FUNCTION _ABRK_APPLY_BANK {
  PARAMETER bank, cmd_frac.
  LOCAL i IS 0.
  UNTIL i >= bank:LENGTH {
    _ABRK_SET_BINDING(bank[i], cmd_frac).
    SET i TO i + 1.
  }
}

FUNCTION _ABRK_APPLY_DEFAULT_BANK {
  PARAMETER bank.
  LOCAL i IS 0.
  UNTIL i >= bank:LENGTH {
    LOCAL b IS bank[i].
    _ABRK_SET_BINDING(b, _ABRK_DEFAULT_CMD_FOR_BINDING(b)).
    SET i TO i + 1.
  }
}

FUNCTION ABRK_APPLY {
  PARAMETER left_cmd, right_cmd.
  ABRK_DISCOVER_PARTS().
  IF NOT ABRK_AVAILABLE { RETURN FALSE. }

  SET left_cmd TO CLAMP(left_cmd, 0, 1).
  SET right_cmd TO CLAMP(right_cmd, 0, 1).

  _ABRK_APPLY_BANK(ABRK_LEFT_BINDINGS, left_cmd).
  _ABRK_APPLY_BANK(ABRK_RIGHT_BINDINGS, right_cmd).
  // Keep nose gear free during differential steering to avoid fighting turn-in.
  IF ABRK_NOSE_BINDINGS:LENGTH > 0 {
    IF ABS(left_cmd - right_cmd) > 0.001 AND MAX(left_cmd, right_cmd) > 0.001 {
      _ABRK_APPLY_BANK(ABRK_NOSE_BINDINGS, 0).
    } ELSE {
      _ABRK_APPLY_DEFAULT_BANK(ABRK_NOSE_BINDINGS).
    }
  }
  SET ABRK_LAST_LEFT_CMD TO left_cmd.
  SET ABRK_LAST_RIGHT_CMD TO right_cmd.
  RETURN TRUE.
}

FUNCTION ABRK_RELEASE {
  IF ABRK_AVAILABLE {
    _ABRK_APPLY_DEFAULT_BANK(ABRK_LEFT_BINDINGS).
    _ABRK_APPLY_DEFAULT_BANK(ABRK_RIGHT_BINDINGS).
    _ABRK_APPLY_DEFAULT_BANK(ABRK_NOSE_BINDINGS).
  }
  SET ABRK_LAST_LEFT_CMD TO _ABRK_DEFAULT_STRENGTH().
  SET ABRK_LAST_RIGHT_CMD TO _ABRK_DEFAULT_STRENGTH().
}
