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
