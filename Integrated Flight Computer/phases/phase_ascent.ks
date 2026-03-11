@LAZYGLOBAL OFF.

// ============================================================
// phase_ascent.ks  -  Integrated Flight Computer
// Spaceplane suborbital ascent program.
//
// STUB — not yet implemented.
//
// Leg params (future):
//   "apoapsis_m"  : target apoapsis altitude in metres
// ============================================================

FUNCTION RUN_ASCENT {
  SET IFC_ALERT_TEXT TO "ASCENT: not yet implemented".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET_PHASE(PHASE_DONE).
}
