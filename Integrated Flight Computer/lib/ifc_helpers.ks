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

FUNCTION GET_AOA {
  IF FAR_AVAILABLE { RETURN ADDONS:FAR:AOA. }
  RETURN 0.
}

// Current aircraft pitch above the horizon.
FUNCTION GET_PITCH {
  RETURN 90 - VECTORANGLE(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
}

// Compass heading of SHIP:FACING (0-360, 0=North, 90=East).
// SHIP:HEADING returns 0 on the ground in kOS; use vectors instead.
FUNCTION GET_COMPASS_HDG {
  LOCAL fwd_v   IS SHIP:FACING:FOREVECTOR.
  LOCAL up_v    IS SHIP:UP:VECTOR.
  LOCAL north_v IS SHIP:NORTH:VECTOR.
  LOCAL horiz   IS (fwd_v - up_v * VDOT(fwd_v, up_v)):NORMALIZED.
  LOCAL east_v  IS VCRS(up_v, north_v):NORMALIZED.  // up × north = east in KSP surface frame
  RETURN MOD(ARCTAN2(VDOT(horiz, east_v), VDOT(horiz, north_v)) + 360, 360).
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
