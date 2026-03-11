@LAZYGLOBAL OFF.

// ============================================================
// phase_reentry.ks  -  Integrated Flight Computer
// Spaceplane atmospheric re-entry program.
//
// STUB — not yet implemented.
//
// Leg params (future):
//   "target_lat"  : target landing latitude (deg)
//   "target_lng"  : target landing longitude (deg)
// ============================================================

FUNCTION RUN_REENTRY {
  SET IFC_ALERT_TEXT TO "REENTRY: not yet implemented".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  SET_PHASE(PHASE_DONE).
}
