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
    PRINT "PHASE: " + IFC_PHASE + " -> " + NEW_PHASE
      + "  T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1).
    SET IFC_PHASE          TO NEW_PHASE.
    SET IFC_PHASE_START_UT TO TIME:SECONDS.
  }
}

FUNCTION SET_SUBPHASE {
  PARAMETER NEW_SUB.
  IF IFC_SUBPHASE <> NEW_SUB {
    PRINT "  SUBPHASE: " + IFC_SUBPHASE + " -> " + NEW_SUB.
    SET IFC_SUBPHASE TO NEW_SUB.
  }
}

FUNCTION PHASE_ELAPSED {
  RETURN TIME:SECONDS - IFC_PHASE_START_UT.
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
