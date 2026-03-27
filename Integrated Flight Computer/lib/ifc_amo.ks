@LAZYGLOBAL OFF.

// ============================================================
// ifc_amo.ks  -  Integrated Flight Computer
//
// AMO (Augmented Manual Operation) ground steering assist:
// - Active in pre-arm when no flight plan is armed
// - Only engages on-ground and when aircraft has_nws = FALSE
// - Uses pilot A/D steering input to apply differential thrust
//   (outside engines command throttle, inside engines idle)
// - Optionally forces brakes at low IAS while steering input is present
// ============================================================

FUNCTION _AMO_CFG_BOOL {
  PARAMETER key, fallback.
  IF ACTIVE_AIRCRAFT = 0 OR NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL cfg_val IS ACTIVE_AIRCRAFT[key].
  IF cfg_val:TYPENAME = "Boolean" { RETURN cfg_val. }
  IF cfg_val:TYPENAME = "Scalar" { RETURN cfg_val <> 0. }
  LOCAL txt IS ("" + cfg_val):TOUPPER.
  IF txt = "TRUE" OR txt = "YES" OR txt = "ON" { RETURN TRUE. }
  IF txt = "FALSE" OR txt = "NO" OR txt = "OFF" { RETURN FALSE. }
  RETURN fallback.
}

FUNCTION _AMO_CFG_NUM {
  PARAMETER key, fallback, min_guard.
  IF ACTIVE_AIRCRAFT = 0 OR NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL cfg_val IS ACTIVE_AIRCRAFT[key].
  IF cfg_val:TYPENAME <> "Scalar" { RETURN fallback. }
  IF cfg_val < min_guard { RETURN fallback. }
  RETURN cfg_val.
}

FUNCTION _AMO_CFG_STR {
  PARAMETER key, fallback.
  IF ACTIVE_AIRCRAFT = 0 OR NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL cfg_val IS ACTIVE_AIRCRAFT[key].
  IF cfg_val = 0 { RETURN fallback. }
  LOCAL txt IS ("" + cfg_val):TRIM.
  IF txt = "" { RETURN fallback. }
  RETURN txt.
}

FUNCTION _AMO_HAS_NWS {
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("has_nws") {
    RETURN _AMO_CFG_BOOL("has_nws", TRUE).
  }
  RETURN TRUE.
}

FUNCTION _AMO_ENABLED {
  RETURN _AMO_CFG_BOOL("amo_enabled", AMO_ENABLED_DEFAULT).
}

FUNCTION _AMO_STEER_SIGN {
  LOCAL s IS 1.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("amo_steer_sign") {
    LOCAL cfg_s IS ACTIVE_AIRCRAFT["amo_steer_sign"].
    IF cfg_s:TYPENAME = "Scalar" AND cfg_s < 0 { SET s TO -1. }
  }
  RETURN s.
}

FUNCTION _AMO_ON_GROUND {
  RETURN SHIP:STATUS = "LANDED" OR SHIP:STATUS = "PRELAUNCH".
}

FUNCTION _AMO_STEER_INPUT {
  IF NOT SHIP:HASSUFFIX("CONTROL") { RETURN 0. }
  LOCAL ctrl IS SHIP:CONTROL.
  LOCAL steer IS 0.
  // Prefer pilot-command channels so AMO follows real control input
  // even when script-side control channels stay at zero.
  IF ctrl:HASSUFFIX("PILOTWHEELSTEERING") { SET steer TO ctrl:PILOTWHEELSTEERING. }
  IF ABS(steer) < 0.001 AND ctrl:HASSUFFIX("PILOTYAW") { SET steer TO ctrl:PILOTYAW. }
  IF ABS(steer) < 0.001 AND ctrl:HASSUFFIX("WHEELSTEERING") { SET steer TO ctrl:WHEELSTEERING. }
  IF ABS(steer) < 0.001 AND ctrl:HASSUFFIX("YAW") { SET steer TO ctrl:YAW. }
  RETURN CLAMP(steer, -1, 1).
}

FUNCTION _AMO_MODULE_ENTRIES {
  PARAMETER p.
  IF p = 0 { RETURN LIST(). }
  IF p:HASSUFFIX("MODULES") { RETURN p:MODULES. }
  IF p:HASSUFFIX("ALLMODULES") { RETURN p:ALLMODULES. }
  RETURN LIST().
}

FUNCTION _AMO_ENTRY_NAME {
  PARAMETER module_entry.
  IF module_entry = 0 { RETURN "". }
  IF module_entry:HASSUFFIX("NAME") AND module_entry:NAME <> "" { RETURN module_entry:NAME. }
  RETURN "" + module_entry.
}

FUNCTION _AMO_RESOLVE_MODULE_OBJECT {
  PARAMETER p, module_entry, module_name.
  IF module_entry <> 0 AND module_entry:HASSUFFIX("HASFIELD") { RETURN module_entry. }
  IF p <> 0 AND p:HASSUFFIX("GETMODULE") AND module_name <> "" {
    LOCAL probe_mod IS p:GETMODULE(module_name).
    IF probe_mod <> 0 { RETURN probe_mod. }
  }
  RETURN module_entry.
}

FUNCTION _AMO_THR_BASE_FOR_FIELD {
  PARAMETER field_name.
  LOCAL forced_base IS _AMO_CFG_NUM("amo_engine_field_base", -1, -9999).
  IF forced_base > 0 { RETURN forced_base. }
  LOCAL f IS field_name:TOLOWER.
  IF f:FIND("input") >= 0 { RETURN 1.0. }
  IF f:FIND("percent") >= 0 { RETURN 100.0. }
  IF f:FIND("limit") >= 0 { RETURN 100.0. }
  RETURN 100.0.
}

FUNCTION _AMO_RESOLVE_THR_FIELD {
  PARAMETER module_ref.
  IF module_ref = 0 OR NOT module_ref:HASSUFFIX("HASFIELD") { RETURN "". }

  LOCAL forced_field IS _AMO_CFG_STR("amo_engine_field_name", "").
  IF forced_field <> "" AND module_ref:HASFIELD(forced_field) { RETURN forced_field. }

  LOCAL fields IS LIST(
    "thrustPercentage",
    "thrustpercentage",
    "thrustLimiter",
    "thrust_limit",
    "Throttle Limiter",
    "Thrust Limiter"
  ).

  LOCAL i IS 0.
  UNTIL i >= fields:LENGTH {
    LOCAL f IS fields[i].
    IF module_ref:HASFIELD(f) { RETURN f. }
    SET i TO i + 1.
  }

  IF module_ref:HASSUFFIX("ALLFIELDS") {
    LOCAL af IS module_ref:ALLFIELDS.
    LOCAL fi IS 0.
    UNTIL fi >= af:LENGTH {
      LOCAL fname IS "" + af[fi].
      LOCAL flc IS fname:TOLOWER.
      IF flc:FIND("thrust") >= 0 AND (flc:FIND("limit") >= 0 OR flc:FIND("percent") >= 0) AND module_ref:HASFIELD(fname) {
        RETURN fname.
      }
      SET fi TO fi + 1.
    }
  }
  IF module_ref:HASSUFFIX("FIELDS") {
    LOCAL f2 IS module_ref:FIELDS.
    LOCAL fi2 IS 0.
    UNTIL fi2 >= f2:LENGTH {
      LOCAL fname2 IS "" + f2[fi2].
      LOCAL flc2 IS fname2:TOLOWER.
      IF flc2:FIND("thrust") >= 0 AND (flc2:FIND("limit") >= 0 OR flc2:FIND("percent") >= 0) AND module_ref:HASFIELD(fname2) {
        RETURN fname2.
      }
      SET fi2 TO fi2 + 1.
    }
  }
  RETURN "".
}

FUNCTION _AMO_GET_MODULE_FIELD {
  PARAMETER module_ref, field_name, fallback.
  IF module_ref = 0 OR field_name = "" { RETURN fallback. }
  IF NOT module_ref:HASSUFFIX("GETFIELD") { RETURN fallback. }
  IF module_ref:HASSUFFIX("HASFIELD") AND NOT module_ref:HASFIELD(field_name) { RETURN fallback. }
  LOCAL fv IS module_ref:GETFIELD(field_name).
  IF fv:TYPENAME = "Scalar" { RETURN fv. }
  RETURN fallback.
}

FUNCTION _AMO_SET_MODULE_FIELD {
  PARAMETER module_ref, field_name, value_num.
  IF module_ref = 0 OR field_name = "" { RETURN FALSE. }
  IF NOT module_ref:HASSUFFIX("SETFIELD") { RETURN FALSE. }
  IF module_ref:HASSUFFIX("HASFIELD") AND NOT module_ref:HASFIELD(field_name) { RETURN FALSE. }
  module_ref:SETFIELD(field_name, ROUND(value_num, 3)).
  RETURN TRUE.
}

FUNCTION _AMO_MAKE_ENGINE_ENTRY {
  PARAMETER eng.
  IF eng = 0 { RETURN 0. }

  // If the aircraft config names an explicit module+field, prefer that path
  // over the THRUSTLIMIT suffix.  SET eng:THRUSTLIMIT is readable on all kOS
  // Engine objects but may not actually change in-game thrust for engines that
  // use ModuleEnginesFX or other modded modules — the module field write is the
  // authoritative path when the config says so.
  IF eng:HASSUFFIX("PART") {
    LOCAL p IS eng:PART.
    IF p <> 0 {
      LOCAL forced_mod IS _AMO_CFG_STR("amo_engine_module_name", "").
      IF forced_mod <> "" AND p:HASSUFFIX("GETMODULE") {
        LOCAL mref IS p:GETMODULE(forced_mod).
        IF mref <> 0 {
          LOCAL fld IS _AMO_RESOLVE_THR_FIELD(mref).
          IF fld <> "" {
            LOCAL scale IS _AMO_THR_BASE_FOR_FIELD(fld).
            LOCAL base_limit IS CLAMP(_AMO_GET_MODULE_FIELD(mref, fld, scale), 0, scale).
            RETURN LEXICON("eng", eng, "mode", "field", "module", mref, "field", fld, "base_limit", base_limit, "scale", scale).
          }
        }
      }
    }
  }

  IF eng:HASSUFFIX("THRUSTLIMIT") {
    LOCAL raw_limit IS eng:THRUSTLIMIT.
    LOCAL scale IS 100.
    IF raw_limit <= 1.5 { SET scale TO 1.0. }
    LOCAL base_limit IS CLAMP(raw_limit, 0, scale).
    RETURN LEXICON("eng", eng, "mode", "suffix", "base_limit", base_limit, "scale", scale).
  }

  IF NOT eng:HASSUFFIX("PART") { RETURN 0. }
  LOCAL p IS eng:PART.
  IF p = 0 { RETURN 0. }

  // Fallback: scan all part modules for a thrust-limiter field.
  LOCAL forced_mod IS _AMO_CFG_STR("amo_engine_module_name", "").
  IF forced_mod <> "" AND p:HASSUFFIX("GETMODULE") {
    LOCAL mref IS p:GETMODULE(forced_mod).
    IF mref <> 0 {
      LOCAL fld IS _AMO_RESOLVE_THR_FIELD(mref).
      IF fld <> "" {
        LOCAL scale IS _AMO_THR_BASE_FOR_FIELD(fld).
        LOCAL base_limit IS CLAMP(_AMO_GET_MODULE_FIELD(mref, fld, scale), 0, scale).
        RETURN LEXICON("eng", eng, "mode", "field", "module", mref, "field", fld, "base_limit", base_limit, "scale", scale).
      }
    }
  }

  LOCAL mods IS _AMO_MODULE_ENTRIES(p).
  LOCAL i IS 0.
  UNTIL i >= mods:LENGTH {
    LOCAL m IS mods[i].
    SET i TO i + 1.
    IF m <> 0 {
      LOCAL mname IS _AMO_ENTRY_NAME(m).
      LOCAL module_ref IS _AMO_RESOLVE_MODULE_OBJECT(p, m, mname).
      LOCAL field_name IS _AMO_RESOLVE_THR_FIELD(module_ref).
      IF field_name <> "" {
        LOCAL scale IS _AMO_THR_BASE_FOR_FIELD(field_name).
        LOCAL base_limit IS CLAMP(_AMO_GET_MODULE_FIELD(module_ref, field_name, scale), 0, scale).
        RETURN LEXICON("eng", eng, "mode", "field", "module", module_ref, "field", field_name, "base_limit", base_limit, "scale", scale).
      }
    }
  }
  RETURN 0.
}

FUNCTION _AMO_SET_ENTRY_LIMIT_FRAC {
  PARAMETER entry, frac01.
  IF entry = 0 OR NOT entry:HASKEY("mode") { RETURN. }
  LOCAL mode IS entry["mode"].
  IF mode = "suffix" {
    IF NOT entry:HASKEY("eng") { RETURN. }
    LOCAL eng IS entry["eng"].
    IF eng = 0 OR NOT eng:HASSUFFIX("THRUSTLIMIT") { RETURN. }
    LOCAL scale IS 100.
    IF entry:HASKEY("scale") { SET scale TO entry["scale"]. }
    SET eng:THRUSTLIMIT TO CLAMP(frac01, 0, 1) * scale.
    RETURN.
  }
  IF mode = "field" {
    IF NOT entry:HASKEY("module") OR NOT entry:HASKEY("field") { RETURN. }
    LOCAL module_ref IS entry["module"].
    LOCAL field_name IS entry["field"].
    LOCAL scale IS 100.
    IF entry:HASKEY("scale") { SET scale TO entry["scale"]. }
    _AMO_SET_MODULE_FIELD(module_ref, field_name, CLAMP(frac01, 0, 1) * scale).
  }
}

FUNCTION _AMO_RESTORE_ENTRY_LIMIT {
  PARAMETER entry.
  IF entry = 0 OR NOT entry:HASKEY("mode") OR NOT entry:HASKEY("base_limit") { RETURN. }
  LOCAL mode IS entry["mode"].
  LOCAL base_limit IS entry["base_limit"].
  IF mode = "suffix" {
    IF NOT entry:HASKEY("eng") { RETURN. }
    LOCAL eng IS entry["eng"].
    IF eng = 0 OR NOT eng:HASSUFFIX("THRUSTLIMIT") { RETURN. }
    SET eng:THRUSTLIMIT TO base_limit.
    RETURN.
  }
  IF mode = "field" {
    IF NOT entry:HASKEY("module") OR NOT entry:HASKEY("field") { RETURN. }
    _AMO_SET_MODULE_FIELD(entry["module"], entry["field"], base_limit).
  }
}

FUNCTION _AMO_SET_BANK_LIMIT_FRAC {
  PARAMETER bank, frac01.
  LOCAL i IS 0.
  UNTIL i >= bank:LENGTH {
    _AMO_SET_ENTRY_LIMIT_FRAC(bank[i], frac01).
    SET i TO i + 1.
  }
}

FUNCTION _AMO_RESTORE_BANK_LIMITS {
  PARAMETER bank.
  LOCAL i IS 0.
  UNTIL i >= bank:LENGTH {
    _AMO_RESTORE_ENTRY_LIMIT(bank[i]).
    SET i TO i + 1.
  }
}

FUNCTION _AMO_RESTORE_ALL_LIMITS {
  _AMO_RESTORE_BANK_LIMITS(AMO_LEFT_ENGS).
  _AMO_RESTORE_BANK_LIMITS(AMO_RIGHT_ENGS).
}

FUNCTION _AMO_CAND_HAS_ENGINE {
  PARAMETER cand_list, eng.
  LOCAL i IS 0.
  UNTIL i >= cand_list:LENGTH {
    LOCAL c IS cand_list[i].
    IF c:HASKEY("entry") {
      LOCAL e IS c["entry"].
      IF e <> 0 AND e:HASKEY("eng") AND e["eng"] = eng { RETURN TRUE. }
    }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _AMO_COLLECT_TAGGED_CANDIDATES {
  PARAMETER tag_name, all_eng, ship_pos, right_vec, out_cands.
  IF tag_name = "" { RETURN. }
  IF NOT SHIP:HASSUFFIX("PARTSTAGGED") { RETURN. }
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  LOCAL ti IS 0.
  UNTIL ti >= tagged:LENGTH {
    LOCAL tp IS tagged[ti].
    SET ti TO ti + 1.
    IF tp <> 0 {
      LOCAL ei IS 0.
      UNTIL ei >= all_eng:LENGTH {
        LOCAL eng IS all_eng[ei].
        SET ei TO ei + 1.
        IF eng <> 0 AND eng:HASSUFFIX("PART") AND eng:PART = tp {
          LOCAL entry IS _AMO_MAKE_ENGINE_ENTRY(eng).
          IF entry <> 0 AND NOT _AMO_CAND_HAS_ENGINE(out_cands, eng) {
            LOCAL side_m IS 0.
            IF tp:HASSUFFIX("POSITION") { SET side_m TO VDOT(tp:POSITION - ship_pos, right_vec). }
            out_cands:ADD(LEXICON("entry", entry, "side", side_m)).
          }
        }
      }
    }
  }
}

// Discover engine entries by ordered tags: tag_prefix + "_1", "_2", ... left to right.
// Splits at floor(N/2): lower-indexed half → out_left, upper half → out_right.
FUNCTION _AMO_DISCOVER_NUMBERED_ENGS {
  PARAMETER tag_prefix, all_eng, out_left, out_right.
  LOCAL found IS LIST().
  LOCAL n IS 1.
  UNTIL n > 20 {
    LOCAL tagged IS SHIP:PARTSTAGGED(tag_prefix + "_" + n).
    IF tagged:LENGTH = 0 { BREAK. }
    LOCAL ti IS 0.
    UNTIL ti >= tagged:LENGTH {
      LOCAL tp IS tagged[ti].
      SET ti TO ti + 1.
      LOCAL ei IS 0.
      UNTIL ei >= all_eng:LENGTH {
        LOCAL eng IS all_eng[ei].
        SET ei TO ei + 1.
        IF eng <> 0 AND eng:HASSUFFIX("PART") AND eng:PART = tp {
          LOCAL entry IS _AMO_MAKE_ENGINE_ENTRY(eng).
          IF entry <> 0 { found:ADD(entry). }
        }
      }
    }
    SET n TO n + 1.
  }
  LOCAL total IS found:LENGTH.
  IF total < 2 { RETURN. }
  LOCAL left_count IS FLOOR(total / 2).
  LOCAL i IS 0.
  UNTIL i >= total {
    IF i < left_count { out_left:ADD(found[i]). }
    ELSE              { out_right:ADD(found[i]). }
    SET i TO i + 1.
  }
}

FUNCTION _AMO_DISCOVER_ENGINES {
  IF AMO_ENG_DISCOVERED AND AMO_DIFF_AVAILABLE { RETURN. }

  AMO_LEFT_ENGS:CLEAR().
  AMO_RIGHT_ENGS:CLEAR().
  SET AMO_DIFF_AVAILABLE TO FALSE.
  SET AMO_ENG_DISCOVERED TO FALSE.

  IF NOT SHIP:HASSUFFIX("ENGINES") { RETURN. }
  LOCAL all_eng IS SHIP:ENGINES.
  IF all_eng:LENGTH <= 1 {
    SET AMO_ENG_DISCOVERED TO TRUE.
    RETURN.
  }

  // Numbered-tag override: amo_engine_tag_prefix + "_1", "_2", ... ordered left to right.
  // Lower half → left bank, upper half → right bank.  No geometry needed.
  LOCAL tag_prefix IS _AMO_CFG_STR("amo_engine_tag_prefix", "").
  IF tag_prefix <> "" AND SHIP:HASSUFFIX("PARTSTAGGED") {
    _AMO_DISCOVER_NUMBERED_ENGS(tag_prefix, all_eng, AMO_LEFT_ENGS, AMO_RIGHT_ENGS).
    SET AMO_ENG_DISCOVERED TO TRUE.
    IF AMO_LEFT_ENGS:LENGTH > 0 AND AMO_RIGHT_ENGS:LENGTH > 0 {
      SET AMO_DIFF_AVAILABLE TO TRUE.
      LOCAL _dbg_mode IS "".
      IF AMO_LEFT_ENGS[0]:HASKEY("mode") { SET _dbg_mode TO AMO_LEFT_ENGS[0]["mode"]. }
      IFC_SET_ALERT("AMO eng (numbered): L=" + AMO_LEFT_ENGS:LENGTH + " R=" + AMO_RIGHT_ENGS:LENGTH + " mode=" + _dbg_mode, "INFO").
    } ELSE {
      IFC_SET_ALERT("AMO eng: numbered tags '" + tag_prefix + "_N' found L=" + AMO_LEFT_ENGS:LENGTH + " R=" + AMO_RIGHT_ENGS:LENGTH, "WARN").
    }
    RETURN.
  }

  LOCAL right_vec IS SHIP:FACING:STARVECTOR.
  LOCAL ship_pos IS SHIP:POSITION.
  LOCAL side_eps IS _AMO_CFG_NUM("amo_engine_side_eps_m", AMO_ENGINE_SIDE_EPS_M, 0).
  LOCAL cands IS LIST().

  // Explicit side tags (optional, deterministic).
  LOCAL left_tag IS _AMO_CFG_STR("amo_engine_left_tag", "").
  LOCAL right_tag IS _AMO_CFG_STR("amo_engine_right_tag", "").
  IF left_tag <> "" { _AMO_COLLECT_TAGGED_CANDIDATES(left_tag, all_eng, ship_pos, right_vec, cands). }
  IF right_tag <> "" { _AMO_COLLECT_TAGGED_CANDIDATES(right_tag, all_eng, ship_pos, right_vec, cands). }

  LOCAL i IS 0.
  UNTIL i >= all_eng:LENGTH {
    LOCAL eng IS all_eng[i].
    SET i TO i + 1.
    IF eng <> 0 AND eng:HASSUFFIX("PART") {
      LOCAL p IS eng:PART.
      IF p <> 0 AND p:HASSUFFIX("POSITION") {
        LOCAL entry IS _AMO_MAKE_ENGINE_ENTRY(eng).
        IF entry <> 0 AND NOT _AMO_CAND_HAS_ENGINE(cands, eng) {
          LOCAL side_m IS VDOT(p:POSITION - ship_pos, right_vec).
          cands:ADD(LEXICON("entry", entry, "side", side_m)).
        }
      }
    }
  }

  // First pass: strict epsilon split.
  SET i TO 0.
  UNTIL i >= cands:LENGTH {
    LOCAL c IS cands[i].
    SET i TO i + 1.
    LOCAL side_m IS c["side"].
    LOCAL entry IS c["entry"].
    IF side_m <= -side_eps {
      AMO_LEFT_ENGS:ADD(entry).
    } ELSE IF side_m >= side_eps {
      AMO_RIGHT_ENGS:ADD(entry).
    }
  }

  // Second pass: sign-only split when epsilon was too strict.
  IF (AMO_LEFT_ENGS:LENGTH = 0 OR AMO_RIGHT_ENGS:LENGTH = 0) AND cands:LENGTH >= 2 {
    AMO_LEFT_ENGS:CLEAR().
    AMO_RIGHT_ENGS:CLEAR().
    SET i TO 0.
    UNTIL i >= cands:LENGTH {
      LOCAL c IS cands[i].
      SET i TO i + 1.
      LOCAL side_m IS c["side"].
      IF side_m < 0 { AMO_LEFT_ENGS:ADD(c["entry"]). }
      ELSE IF side_m > 0 { AMO_RIGHT_ENGS:ADD(c["entry"]). }
    }
  }

  // Final pass: if still unsplit, use extremal pair by lateral offset.
  IF (AMO_LEFT_ENGS:LENGTH = 0 OR AMO_RIGHT_ENGS:LENGTH = 0) AND cands:LENGTH >= 2 {
    LOCAL li IS 0.
    LOCAL ri IS 0.
    LOCAL lval IS cands[0]["side"].
    LOCAL rval IS cands[0]["side"].
    SET i TO 1.
    UNTIL i >= cands:LENGTH {
      LOCAL s IS cands[i]["side"].
      IF s < lval { SET lval TO s. SET li TO i. }
      IF s > rval { SET rval TO s. SET ri TO i. }
      SET i TO i + 1.
    }
    IF li <> ri {
      AMO_LEFT_ENGS:CLEAR().
      AMO_RIGHT_ENGS:CLEAR().
      AMO_LEFT_ENGS:ADD(cands[li]["entry"]).
      AMO_RIGHT_ENGS:ADD(cands[ri]["entry"]).
    }
  }

  IF AMO_LEFT_ENGS:LENGTH > 0 AND AMO_RIGHT_ENGS:LENGTH > 0 {
    SET AMO_DIFF_AVAILABLE TO TRUE.
  }
  SET AMO_ENG_DISCOVERED TO TRUE.

  // One-shot discovery report — visible in terminal and alert bar.
  LOCAL _amo_dbg_mode IS "".
  IF AMO_LEFT_ENGS:LENGTH > 0 AND AMO_LEFT_ENGS[0]:HASKEY("mode") {
    SET _amo_dbg_mode TO AMO_LEFT_ENGS[0]["mode"].
  }
  IF AMO_DIFF_AVAILABLE {
    IFC_SET_ALERT("AMO eng: L=" + AMO_LEFT_ENGS:LENGTH + " R=" + AMO_RIGHT_ENGS:LENGTH + " mode=" + _amo_dbg_mode, "INFO").
  } ELSE {
    IFC_SET_ALERT("AMO eng: diff unavail L=" + AMO_LEFT_ENGS:LENGTH + " R=" + AMO_RIGHT_ENGS:LENGTH + " total=" + cands:LENGTH, "WARN").
  }
}

FUNCTION AMO_RELEASE {
  IF AMO_DIFF_AVAILABLE {
    _AMO_RESTORE_ALL_LIMITS().
  }
  ABRK_RELEASE().
  IF AMO_BRAKES_FORCED {
    BRAKES OFF.
    SET AMO_BRAKES_FORCED TO FALSE.
  }
  SET AMO_ACTIVE TO FALSE.
}

FUNCTION AMO_RESET {
  SET AMO_ACTIVE TO FALSE.
  SET AMO_ENG_DISCOVERED TO FALSE.
  SET AMO_DIFF_AVAILABLE TO FALSE.
  AMO_LEFT_ENGS:CLEAR().
  AMO_RIGHT_ENGS:CLEAR().
  SET AMO_BRAKES_FORCED TO FALSE.
}

FUNCTION AMO_TICK_PREARM {
  IF NOT _AMO_ENABLED() {
    AMO_RELEASE().
    RETURN.
  }
  IF _AMO_HAS_NWS() {
    AMO_RELEASE().
    RETURN.
  }
  IF NOT _AMO_ON_GROUND() {
    AMO_RELEASE().
    RETURN.
  }

  _AMO_DISCOVER_ENGINES().
  ABRK_DISCOVER_PARTS().

  LOCAL steer_raw IS _AMO_STEER_INPUT() * _AMO_STEER_SIGN().
  LOCAL deadband IS CLAMP(_AMO_CFG_NUM("amo_steer_deadband", AMO_STEER_DEADBAND, 0), 0, 1).
  IF ABS(steer_raw) <= deadband {
    _AMO_RESTORE_ALL_LIMITS().
    ABRK_RELEASE().
    IF AMO_BRAKES_FORCED {
      BRAKES OFF.
      SET AMO_BRAKES_FORCED TO FALSE.
    }
    SET AMO_ACTIVE TO FALSE.
    RETURN.
  }

  IF AMO_DIFF_AVAILABLE {
    LOCAL thr_frac IS CLAMP(GET_CURRENT_THROTTLE(), 0, 1).
    IF steer_raw > 0 {
      // Steering right: left bank drives, right bank idles.
      _AMO_SET_BANK_LIMIT_FRAC(AMO_LEFT_ENGS, thr_frac).
      _AMO_SET_BANK_LIMIT_FRAC(AMO_RIGHT_ENGS, 0).
    } ELSE {
      // Steering left: right bank drives, left bank idles.
      _AMO_SET_BANK_LIMIT_FRAC(AMO_RIGHT_ENGS, thr_frac).
      _AMO_SET_BANK_LIMIT_FRAC(AMO_LEFT_ENGS, 0).
    }
  }

  LOCAL steer_eff IS CLAMP((ABS(steer_raw) - deadband) / MAX(1 - deadband, 0.001), 0, 1).
  LOCAL diff_brake_strength IS CLAMP(_AMO_CFG_NUM("diff_brake_strength", AMO_DIFF_BRAKE_STRENGTH, 0), 0, 1).
  LOCAL brake_cmd IS diff_brake_strength * steer_eff.
  LOCAL brake_max_ias IS _AMO_CFG_NUM("amo_brake_max_ias", AMO_BRAKE_MAX_IAS, 0).

  IF brake_cmd > 0 {
    IF steer_raw > 0 {
      ABRK_APPLY(0, brake_cmd).
    } ELSE {
      ABRK_APPLY(brake_cmd, 0).
    }
  } ELSE {
    ABRK_RELEASE().
  }

  // Engage wheel brakes while differential braking is commanded.
  // For some wheel modules (for example with "brakeTweakable"), per-side
  // values are only meaningful when BRAKES is actively ON.
  IF brake_cmd > 0 AND GET_IAS() <= brake_max_ias {
    BRAKES ON.
    SET AMO_BRAKES_FORCED TO TRUE.
  } ELSE IF AMO_BRAKES_FORCED {
    BRAKES OFF.
    SET AMO_BRAKES_FORCED TO FALSE.
  }

  SET AMO_ACTIVE TO TRUE.
}
