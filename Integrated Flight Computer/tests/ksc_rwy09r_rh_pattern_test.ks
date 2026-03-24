@LAZYGLOBAL OFF.

// ============================================================
// ksc_rwy09r_rh_pattern_test.ks  -  IFC route test
//
// Sequence:
//   1. Takeoff from KSC RWY 09
//   2. Fly south for 1:30 (course_time) (crosswind leg)
//   3. Turn west and fly 33 km (course_dist) (downwind leg)
//   4. Fly north for 1:00 (course_time) (base leg)
//   5. ILS approach KSC RWY 09R (30 km short approach)
//
// Usage (kOS terminal):
//   RUNPATH("0:/Integrated Flight Computer/tests/ksc_rwy09r_rh_pattern_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM_RWY09R_RH {
  PRINT "Type A to ARM, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION RUN_KSC_RWY09R_RH_PATTERN_TEST {
  // Tunables for this profile.
  LOCAL cruise_alt_m IS 1500.
  LOCAL crosswind_spd_mps IS 150.
  LOCAL downwind_spd_mps IS 200.
  LOCAL base_spd_mps IS 150.
  LOCAL south_time_s IS 90.
  LOCAL south_time_min IS south_time_s / 60.
  LOCAL west_dist_km IS 33.
  LOCAL west_dist_nm IS west_dist_km / 1.852.
  LOCAL north_before_app_time_s IS 60.
  LOCAL north_before_app_time_min IS north_before_app_time_s / 60.

  CLEARSCREEN.
  PRINT "==============================================".
  PRINT " IFC TEST: RWY 09 -> S 1:30 -> W 33 KM -> N 1:00 -> RWY 09R".
  PRINT "==============================================".
  PRINT " ".
  PRINT "Vessel : " + SHIP:NAME.
  PRINT "Leg 1  : TAKEOFF  RWY 09".
  PRINT "Leg 2  : CRUISE   180 deg / 1:30 / " + ROUND(crosswind_spd_mps, 0) + " m/s".
  PRINT "Leg 3  : CRUISE   270 deg / " + ROUND(west_dist_km, 0) + " km / " + ROUND(downwind_spd_mps, 0) + " m/s".
  PRINT "Leg 4  : CRUISE   000 deg / 1:00 / " + ROUND(base_spd_mps, 0) + " m/s".
  PRINT "Leg 5  : ILS APP  RWY 09R  (30 km short approach)".
  PRINT "Cruise : " + ROUND(cruise_alt_m, 0) + " m".
  PRINT " ".

  LOCAL arm IS _PROMPT_ARM_RWY09R_RH().
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

  // Leg 1: Takeoff from KSC RWY 09.
  plan:ADD(LEXICON(
    "type",   LEG_TAKEOFF,
    "params", LEXICON("rwy_id", "09")
  )).

  // Leg 2: Fly south for a fixed time.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_time",
      "course_deg", 180,
      "time_min",   south_time_min,
      "alt_m",      cruise_alt_m,
      "spd",        crosswind_spd_mps
    )
  )).

  // Leg 3: Fly west for a fixed distance.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_dist",
      "course_deg", 270,
      "dist_nm",    west_dist_nm,
      "alt_m",      cruise_alt_m,
      "spd",        downwind_spd_mps
    )
  )).

  // Leg 4: Fly north for 1:00 before approach setup.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_time",
      "course_deg", 0,
      "time_min",   north_before_app_time_min,
      "alt_m",      cruise_alt_m,
      "spd",        base_spd_mps
    )
  )).

  // Leg 5: ILS approach to KSC RWY 09R from the 30 km fix.
  plan:ADD(LEXICON(
    "type",   LEG_APPROACH,
    "params", LEXICON("rwy_id", "09R", "short_approach", TRUE)
  )).

  PRINT "Plan: " + plan:LENGTH + " legs.  Engaging IFC...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "RWY 09R right-hand pattern into RWY 09R approach test complete.".
}

RUN_KSC_RWY09R_RH_PATTERN_TEST().
