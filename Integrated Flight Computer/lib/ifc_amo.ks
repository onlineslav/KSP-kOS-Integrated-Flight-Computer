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

FUNCTION _AMO_MAKE_ENGINE_ENTRY {
  PARAMETER eng.
  IF eng = 0 { RETURN 0. }
  IF NOT eng:HASSUFFIX("THRUSTLIMIT") { RETURN 0. }
  LOCAL raw_limit IS eng:THRUSTLIMIT.
  LOCAL scale IS 100.
  IF raw_limit <= 1.5 { SET scale TO 1.0. }
  LOCAL base_limit IS CLAMP(raw_limit, 0, scale).
  RETURN LEXICON("eng", eng, "base_limit", base_limit, "scale", scale).
}

FUNCTION _AMO_SET_ENTRY_LIMIT_FRAC {
  PARAMETER entry, frac01.
  IF entry = 0 OR NOT entry:HASKEY("eng") { RETURN. }
  LOCAL eng IS entry["eng"].
  IF eng = 0 OR NOT eng:HASSUFFIX("THRUSTLIMIT") { RETURN. }
  LOCAL scale IS 100.
  IF entry:HASKEY("scale") { SET scale TO entry["scale"]. }
  SET eng:THRUSTLIMIT TO CLAMP(frac01, 0, 1) * scale.
}

FUNCTION _AMO_RESTORE_ENTRY_LIMIT {
  PARAMETER entry.
  IF entry = 0 OR NOT entry:HASKEY("eng") OR NOT entry:HASKEY("base_limit") { RETURN. }
  LOCAL eng IS entry["eng"].
  IF eng = 0 OR NOT eng:HASSUFFIX("THRUSTLIMIT") { RETURN. }
  SET eng:THRUSTLIMIT TO entry["base_limit"].
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

FUNCTION _AMO_DISCOVER_ENGINES {
  IF AMO_ENG_DISCOVERED { RETURN. }

  AMO_LEFT_ENGS:CLEAR().
  AMO_RIGHT_ENGS:CLEAR().
  SET AMO_DIFF_AVAILABLE TO FALSE.
  SET AMO_ENG_DISCOVERED TO TRUE.

  IF NOT SHIP:HASSUFFIX("ENGINES") { RETURN. }
  LOCAL all_eng IS SHIP:ENGINES.
  IF all_eng:LENGTH <= 1 { RETURN. }

  LOCAL right_vec IS SHIP:FACING:STARVECTOR.
  LOCAL ship_pos IS SHIP:POSITION.
  LOCAL side_eps IS _AMO_CFG_NUM("amo_engine_side_eps_m", AMO_ENGINE_SIDE_EPS_M, 0).

  LOCAL i IS 0.
  UNTIL i >= all_eng:LENGTH {
    LOCAL eng IS all_eng[i].
    SET i TO i + 1.
    IF eng <> 0 AND eng:HASSUFFIX("PART") {
      LOCAL p IS eng:PART.
      IF p <> 0 AND p:HASSUFFIX("POSITION") {
        LOCAL entry IS _AMO_MAKE_ENGINE_ENTRY(eng).
        IF entry <> 0 {
          LOCAL side_m IS VDOT(p:POSITION - ship_pos, right_vec).
          IF side_m <= -side_eps {
            AMO_LEFT_ENGS:ADD(entry).
          } ELSE IF side_m >= side_eps {
            AMO_RIGHT_ENGS:ADD(entry).
          }
        }
      }
    }
  }

  IF AMO_LEFT_ENGS:LENGTH > 0 AND AMO_RIGHT_ENGS:LENGTH > 0 {
    SET AMO_DIFF_AVAILABLE TO TRUE.
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
