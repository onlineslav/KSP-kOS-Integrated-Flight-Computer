@LAZYGLOBAL OFF.

// ============================================================
// ifc_helpers.ks  -  Integrated Flight Computer
// General-purpose utility functions.
// ============================================================

FUNCTION CLAMP {
  PARAMETER X, LO, HI.
  IF X < LO { RETURN LO. }
  IF X > HI { RETURN HI. }
  RETURN X.
}

FUNCTION MOVE_TOWARD {
  PARAMETER CURRENT, TARGET, MAX_STEP.
  IF ABS(TARGET - CURRENT) <= MAX_STEP { RETURN TARGET. }
  IF CURRENT < TARGET { RETURN CURRENT + MAX_STEP. }
  RETURN CURRENT - MAX_STEP.
}

// Wrap an angle into [-180, 180).
FUNCTION WRAP_180 {
  PARAMETER A.
  SET A TO MOD(A, 360).
  IF A >= 180  { RETURN A - 360. }
  IF A < -180  { RETURN A + 360. }
  RETURN A.
}

// Wrap an angle into [0, 360).
FUNCTION WRAP_360 {
  PARAMETER A.
  SET A TO MOD(A, 360).
  IF A < 0 { RETURN A + 360. }
  RETURN A.
}

// ----------------------------
// UI mode + event/alert helpers
// ----------------------------
FUNCTION IFC_SET_UI_MODE {
  PARAMETER mode.
  SET IFC_UI_MODE TO mode.
  SET IFC_MENU_OPEN TO mode = UI_MODE_MENU_OVERLAY.
}

FUNCTION _IFC_GUESS_SEVERITY {
  PARAMETER msg.
  LOCAL msg_upper IS msg:TOUPPER.
  IF msg_upper:FIND("ABORT") >= 0 OR msg_upper:FIND("ERROR") >= 0 { RETURN "ERROR". }
  IF msg_upper:FIND("WARN") >= 0 OR msg_upper:FIND("UNAVAIL") >= 0 { RETURN "WARN". }
  RETURN "INFO".
}

FUNCTION IFC_PUSH_EVENT {
  PARAMETER msg.
  PARAMETER sev IS "INFO".
  LOCAL e IS LEXICON(
    "ut", TIME:SECONDS,
    "sev", sev,
    "msg", msg
  ).
  IFC_EVENT_QUEUE:ADD(e).
  UNTIL IFC_EVENT_QUEUE:LENGTH <= IFC_EVENT_MAX {
    IFC_EVENT_QUEUE:REMOVE(0).
  }
  SET IFC_EVENT_VIEW_IDX TO MAX(IFC_EVENT_QUEUE:LENGTH - 1, 0).
  SET IFC_ALERT_TEXT TO "[" + sev + "] " + msg.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET IFC_EVENT_LAST_UT TO IFC_ALERT_UT.
  SET IFC_EVENT_LAST_TEXT TO IFC_ALERT_TEXT.
}

FUNCTION IFC_SET_ALERT {
  PARAMETER msg.
  PARAMETER sev IS "INFO".
  IFC_PUSH_EVENT(msg, sev).
}

// Mirror legacy direct IFC_ALERT_TEXT/IFC_ALERT_UT writes into the event queue.
FUNCTION IFC_SYNC_ALERT_QUEUE {
  IF IFC_ALERT_UT < IFC_EVENT_LAST_UT { RETURN. }
  IF IFC_ALERT_UT = IFC_EVENT_LAST_UT AND IFC_ALERT_TEXT = IFC_EVENT_LAST_TEXT { RETURN. }
  IF IFC_ALERT_TEXT = "" {
    SET IFC_EVENT_LAST_UT TO IFC_ALERT_UT.
    SET IFC_EVENT_LAST_TEXT TO "".
    RETURN.
  }
  LOCAL sev IS _IFC_GUESS_SEVERITY(IFC_ALERT_TEXT).
  LOCAL e IS LEXICON(
    "ut", IFC_ALERT_UT,
    "sev", sev,
    "msg", IFC_ALERT_TEXT
  ).
  IFC_EVENT_QUEUE:ADD(e).
  UNTIL IFC_EVENT_QUEUE:LENGTH <= IFC_EVENT_MAX {
    IFC_EVENT_QUEUE:REMOVE(0).
  }
  SET IFC_EVENT_VIEW_IDX TO MAX(IFC_EVENT_QUEUE:LENGTH - 1, 0).
  SET IFC_EVENT_LAST_UT TO IFC_ALERT_UT.
  SET IFC_EVENT_LAST_TEXT TO IFC_ALERT_TEXT.
}

// ----------------------------
// Phase management
// ----------------------------
FUNCTION SET_PHASE {
  PARAMETER NEW_PHASE.
  IF IFC_PHASE <> NEW_PHASE {
    SET IFC_ALERT_TEXT TO IFC_PHASE + " -> " + NEW_PHASE
      + "  T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1).
    SET IFC_ALERT_UT       TO TIME:SECONDS.
    SET IFC_PHASE          TO NEW_PHASE.
    SET IFC_PHASE_START_UT TO TIME:SECONDS.
    // Clear the approach sub-phase label when leaving APPROACH so telemetry
    // and logs don't show a stale "ILS_TRACK" during FLARE/TOUCHDOWN/ROLLOUT.
    IF NEW_PHASE <> PHASE_APPROACH { SET IFC_SUBPHASE TO "". }
  }
}

FUNCTION SET_SUBPHASE {
  PARAMETER NEW_SUB.
  IF IFC_SUBPHASE <> NEW_SUB {
    SET IFC_ALERT_TEXT TO "SUBPHASE: " + IFC_SUBPHASE + " -> " + NEW_SUB.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    SET IFC_SUBPHASE   TO NEW_SUB.
  }
}

FUNCTION PHASE_ELAPSED {
  RETURN TIME:SECONDS - IFC_PHASE_START_UT.
}

// ----------------------------
// Aircraft config helper
// ----------------------------
// Resolve a numeric parameter from ACTIVE_AIRCRAFT config.
// Returns the aircraft's value if it exists AND the value >= guard,
// otherwise returns fallback.
//
// Common guard values:
//   0.001  →  "accept if > 0"        (positive-only params, e.g. IAS limits)
//   0      →  "accept if >= 0"       (params that can legitimately be zero)
//  -0.5    →  "accept if > -1"       (where -1 is the no-override sentinel)
//
// For negative-valued params (e.g. flare_touchdown_vs) where -1 is still the
// sentinel, keep the check inline since AC_PARAM cannot express "< 0 AND <> -1".
FUNCTION AC_PARAM {
  PARAMETER key, fallback, guard.
  IF ACTIVE_AIRCRAFT = 0 { RETURN fallback. }
  IF NOT ACTIVE_AIRCRAFT:HASKEY(key) { RETURN fallback. }
  LOCAL val IS ACTIVE_AIRCRAFT[key].
  IF val >= guard { RETURN val. }
  RETURN fallback.
}

// ----------------------------
// Sensor helpers (FAR or fallback)
// ----------------------------
FUNCTION GET_IAS {
  IF FAR_AVAILABLE { RETURN ADDONS:FAR:IAS. }
  RETURN SHIP:AIRSPEED.
}

FUNCTION GET_AGL {
  // kOS ALT:RADAR is radar altitude above terrain.
  RETURN ALT:RADAR.
}

// Height above active runway reference altitude (ASL).
// Falls back to radar AGL if no active ILS runway reference is available.
FUNCTION GET_RUNWAY_REL_HEIGHT {
  IF ACTIVE_ILS_ID <> "" {
    LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).
    IF ils:HASKEY("alt_asl") {
      RETURN SHIP:ALTITUDE - ils["alt_asl"].
    }
  }
  RETURN GET_AGL().
}

FUNCTION _GET_MAIN_GEAR_TAG {
  LOCAL tag IS FLARE_MAIN_GEAR_TAG_DEFAULT.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flare_gear_tag") {
    LOCAL ac_tag IS ACTIVE_AIRCRAFT["flare_gear_tag"].
    IF ac_tag <> "" { SET tag TO ac_tag. }
  }
  RETURN tag.
}

FUNCTION _GET_RUNWAY_REF_ALT_ASL {
  IF ACTIVE_ILS_ID <> "" {
    LOCAL ils IS GET_BEACON(ACTIVE_ILS_ID).
    IF ils:HASKEY("alt_asl") { RETURN ils["alt_asl"]. }
  }
  RETURN SHIP:ALTITUDE - GET_AGL().
}

FUNCTION _DISCOVER_MAIN_GEAR_PARTS {
  LOCAL tag IS _GET_MAIN_GEAR_TAG().
  IF FLARE_GEAR_DISCOVERED AND FLARE_GEAR_TAG_ACTIVE = tag { RETURN. }

  FLARE_GEAR_PARTS:CLEAR().
  SET FLARE_GEAR_TAG_ACTIVE TO tag.
  SET FLARE_GEAR_DISCOVERED TO TRUE.

  IF tag = "" { RETURN. }
  IF NOT SHIP:HASSUFFIX("PARTSTAGGED") { RETURN. }

  LOCAL tagged_parts IS SHIP:PARTSTAGGED(tag).
  LOCAL i IS 0.
  UNTIL i >= tagged_parts:LENGTH {
    LOCAL p IS tagged_parts[i].
    IF p <> 0 AND p:HASSUFFIX("BOUNDS") {
      LOCAL b IS p:BOUNDS.
      IF b <> 0 AND (b:HASSUFFIX("BOTTOMALT") OR b:HASSUFFIX("BOTTOMALTRADAR")) {
        FLARE_GEAR_PARTS:ADD(p).
      }
    }
    SET i TO i + 1.
  }
}

FUNCTION PRIME_MAIN_GEAR_CACHE {
  _DISCOVER_MAIN_GEAR_PARTS().
}

FUNCTION _GET_GEAR_BOTTOM_RUNWAY_HEIGHT {
  PARAMETER gear_part, runway_ref_alt_asl.
  IF gear_part = 0 OR NOT gear_part:HASSUFFIX("BOUNDS") { RETURN 999999. }
  LOCAL b IS gear_part:BOUNDS.
  IF b = 0 { RETURN 999999. }
  IF b:HASSUFFIX("BOTTOMALT") { RETURN b:BOTTOMALT - runway_ref_alt_asl. }
  IF b:HASSUFFIX("BOTTOMALTRADAR") { RETURN b:BOTTOMALTRADAR. }
  RETURN 999999.
}

// Returns minimum runway-relative height (m) among tagged main-gear parts.
// Falls back to vessel runway-relative height if no tagged gear parts exist.
FUNCTION GET_MAIN_GEAR_RUNWAY_HEIGHT_MIN {
  _DISCOVER_MAIN_GEAR_PARTS().
  IF FLARE_GEAR_PARTS:LENGTH <= 0 { RETURN GET_RUNWAY_REL_HEIGHT(). }

  LOCAL runway_ref_alt_asl IS _GET_RUNWAY_REF_ALT_ASL().
  LOCAL min_h IS 999999.
  LOCAL i IS 0.
  UNTIL i >= FLARE_GEAR_PARTS:LENGTH {
    LOCAL h IS _GET_GEAR_BOTTOM_RUNWAY_HEIGHT(FLARE_GEAR_PARTS[i], runway_ref_alt_asl).
    IF h < min_h { SET min_h TO h. }
    SET i TO i + 1.
  }
  IF min_h < 999998 { RETURN min_h. }
  RETURN GET_RUNWAY_REL_HEIGHT().
}

// Counts tagged main-gear parts currently at or below the given
// runway-relative height threshold.
FUNCTION COUNT_MAIN_GEAR_BELOW {
  PARAMETER height_m.
  _DISCOVER_MAIN_GEAR_PARTS().

  IF FLARE_GEAR_PARTS:LENGTH <= 0 {
    IF GET_RUNWAY_REL_HEIGHT() <= height_m { RETURN 1. }
    RETURN 0.
  }

  LOCAL runway_ref_alt_asl IS _GET_RUNWAY_REF_ALT_ASL().
  LOCAL count IS 0.
  LOCAL i IS 0.
  UNTIL i >= FLARE_GEAR_PARTS:LENGTH {
    LOCAL h IS _GET_GEAR_BOTTOM_RUNWAY_HEIGHT(FLARE_GEAR_PARTS[i], runway_ref_alt_asl).
    IF h <= height_m { SET count TO count + 1. }
    SET i TO i + 1.
  }
  RETURN count.
}

FUNCTION GET_AOA {
  IF FAR_AVAILABLE { RETURN ADDONS:FAR:AOA. }
  RETURN 0.
}

// Safe current throttle read for telemetry/estimators.
// Prefer CONTROL suffixes when available; fall back to IFC command state.
FUNCTION GET_CURRENT_THROTTLE {
  IF SHIP:HASSUFFIX("CONTROL") {
    LOCAL ctrl IS SHIP:CONTROL.
    IF ctrl:HASSUFFIX("MAINTHROTTLE") {
      RETURN CLAMP(ctrl:MAINTHROTTLE, 0, 1).
    }
    IF ctrl:HASSUFFIX("PILOTMAINTHROTTLE") {
      RETURN CLAMP(ctrl:PILOTMAINTHROTTLE, 0, 1).
    }
  }
  RETURN CLAMP(THROTTLE_CMD, 0, 1).
}

// Current aircraft pitch above the horizon.
// Reads the per-loop cached facing/up vectors (IFC_FACING_FWD, IFC_UP_VEC).
FUNCTION GET_PITCH {
  RETURN 90 - VECTORANGLE(IFC_FACING_FWD, IFC_UP_VEC).
}

// Compass heading of SHIP:FACING (0-360, 0=North, 90=East).
// SHIP:HEADING returns 0 on the ground in kOS; use vectors instead.
// Reads the per-loop cached vectors to avoid redundant VM suffix calls.
FUNCTION GET_COMPASS_HDG {
  LOCAL horiz  IS (IFC_FACING_FWD - IFC_UP_VEC * VDOT(IFC_FACING_FWD, IFC_UP_VEC)):NORMALIZED.
  LOCAL east_v IS VCRS(IFC_UP_VEC, IFC_NORTH_VEC):NORMALIZED.  // up × north = east in KSP's left-handed surface frame
  RETURN MOD(ARCTAN2(VDOT(horiz, east_v), VDOT(horiz, IFC_NORTH_VEC)) + 360, 360).
}

// ----------------------------
// Action group trigger
// ----------------------------
// kOS action groups AG1-AG10 are named variables, not indexable by number.
// Pass enable = TRUE to activate, FALSE to deactivate.
FUNCTION TRIGGER_AG {
  PARAMETER num, enable.
  IF      num = 1  { SET AG1  TO enable. }
  ELSE IF num = 2  { SET AG2  TO enable. }
  ELSE IF num = 3  { SET AG3  TO enable. }
  ELSE IF num = 4  { SET AG4  TO enable. }
  ELSE IF num = 5  { SET AG5  TO enable. }
  ELSE IF num = 6  { SET AG6  TO enable. }
  ELSE IF num = 7  { SET AG7  TO enable. }
  ELSE IF num = 8  { SET AG8  TO enable. }
  ELSE IF num = 9  { SET AG9  TO enable. }
  ELSE IF num = 10 { SET AG10 TO enable. }
}

// Toggle an action group state once to generate a "pulse" event.
// Useful for FAR flap increment/decrement bindings.
FUNCTION PULSE_AG {
  PARAMETER num.
  IF      num = 1  { SET AG1  TO NOT AG1. }
  ELSE IF num = 2  { SET AG2  TO NOT AG2. }
  ELSE IF num = 3  { SET AG3  TO NOT AG3. }
  ELSE IF num = 4  { SET AG4  TO NOT AG4. }
  ELSE IF num = 5  { SET AG5  TO NOT AG5. }
  ELSE IF num = 6  { SET AG6  TO NOT AG6. }
  ELSE IF num = 7  { SET AG7  TO NOT AG7. }
  ELSE IF num = 8  { SET AG8  TO NOT AG8. }
  ELSE IF num = 9  { SET AG9  TO NOT AG9. }
  ELSE IF num = 10 { SET AG10 TO NOT AG10. }
}
