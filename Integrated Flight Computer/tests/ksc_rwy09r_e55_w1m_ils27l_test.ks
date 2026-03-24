@LAZYGLOBAL OFF.

// ============================================================
// ksc_rwy09r_e55_w1m_ils27l_test.ks  -  IFC route test
//
// Sequence:
//   1. Takeoff from KSC RWY 09R
//   2. Cruise east (090) for 55 km at 250 m/s
//   3. Cruise west (270) for 1:00 at 250 m/s
//   4. ILS approach KSC RWY 27L (30 km short approach)
//
// Usage (kOS terminal):
//   RUNPATH("0:/Integrated Flight Computer/tests/ksc_rwy09r_e55_w1m_ils27l_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM_RWY09R_E55_W1M {
  PRINT "Type A to ARM, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION RUN_KSC_RWY09R_E55_W1M_ILS27L_TEST {
  // Tunables for this profile.
  LOCAL cruise_alt_m IS 1500.
  LOCAL cruise_spd_mps IS 250.
  LOCAL east_dist_km IS 55.
  LOCAL east_dist_nm IS east_dist_km / 1.852.
  LOCAL west_time_min IS 1.0.

  CLEARSCREEN.
  PRINT "==============================================".
  PRINT " IFC TEST: RWY 09R -> E 55 KM -> W 1:00 -> RWY 27L".
  PRINT "==============================================".
  PRINT " ".
  PRINT "Vessel : " + SHIP:NAME.
  PRINT "Leg 1  : TAKEOFF  RWY 09R".
  PRINT "Leg 2  : CRUISE   090 deg / " + ROUND(east_dist_km, 0) + " km / " + ROUND(cruise_spd_mps, 0) + " m/s".
  PRINT "Leg 3  : CRUISE   270 deg / 1:00 / " + ROUND(cruise_spd_mps, 0) + " m/s".
  PRINT "Leg 4  : ILS APP  RWY 27L  (30 km short approach)".
  PRINT "Cruise : " + ROUND(cruise_alt_m, 0) + " m".
  PRINT " ".

  LOCAL arm IS _PROMPT_ARM_RWY09R_E55_W1M().
  IF arm = "q" OR arm = "Q" {
    PRINT "Test aborted.".
    WAIT 0.
    RETURN.
  }

  PRINT " ".
  PRINT "Loading IFC...".
  SET IFC_SKIP_INTERACTIVE TO TRUE.

  LOCAL ifc_main IS "0:/Integrated Flight Computer/ifc_main.ks".
  IF NOT EXISTS(ifc_main) {
    PRINT "ERROR: missing " + ifc_main.
    SET IFC_SKIP_INTERACTIVE TO FALSE.
    WAIT 0.
    RETURN.
  }
  RUNONCEPATH(ifc_main).

  IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
    PRINT "No config for [" + SHIP:NAME + "] - using defaults.".
    SET ACTIVE_AIRCRAFT TO LEXICON("name", SHIP:NAME).
  }
  PRINT "Aircraft: " + ACTIVE_AIRCRAFT["name"].

  LOCAL plan IS LIST().

  // Leg 1: Takeoff from KSC RWY 09R.
  plan:ADD(LEXICON(
    "type",   LEG_TAKEOFF,
    "params", LEXICON("rwy_id", "09R")
  )).

  // Leg 2: Fly east for a fixed distance.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_dist",
      "course_deg", 90,
      "dist_nm",    east_dist_nm,
      "alt_m",      cruise_alt_m,
      "spd",        cruise_spd_mps
    )
  )).

  // Leg 3: Fly west for 1:00.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_time",
      "course_deg", 270,
      "time_min",   west_time_min,
      "alt_m",      cruise_alt_m,
      "spd",        cruise_spd_mps
    )
  )).

  // Leg 4: ILS approach to KSC RWY 27L from the 30 km fix.
  plan:ADD(LEXICON(
    "type",   LEG_APPROACH,
    "params", LEXICON("rwy_id", "27L", "short_approach", TRUE)
  )).

  PRINT "Plan: " + plan:LENGTH + " legs.  Engaging IFC...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "RWY 09R departure -> 27L short ILS test complete.".
}

RUN_KSC_RWY09R_E55_W1M_ILS27L_TEST().
