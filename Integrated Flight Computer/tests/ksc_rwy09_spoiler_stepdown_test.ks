@LAZYGLOBAL OFF.

// ============================================================
// ksc_rwy09_spoiler_stepdown_test.ks  -  IFC autospoiler test
//
// Sequence:
//   1) TAKEOFF from KSC RWY 09
//   2) CRUISE (course_time) heading 090 for 1.5 min at 1500 m / 250 m/s
//   3) CRUISE (course_time) heading 090 for 1.5 min at 1500 m / 150 m/s
//
// Purpose:
//   Exercise autospoiler behavior during a commanded speed step-down.
//
// Usage:
//   RUNPATH("0:/Integrated Flight Computer/tests/ksc_rwy09_spoiler_stepdown_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM_RWY09_STEPDOWN {
  PRINT "Type A to ARM, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL selection IS "".
  UNTIL selection = "a" OR selection = "A" OR selection = "q" OR selection = "Q" {
    SET selection TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN selection.
}

FUNCTION RUN_KSC_RWY09_SPOILER_STEPDOWN_TEST {
  LOCAL cruise_alt_m IS 1500.
  LOCAL leg_time_min IS 1.5.
  LOCAL heading_deg IS 90.
  LOCAL speed_high_mps IS 250.
  LOCAL speed_low_mps IS 150.

  CLEARSCREEN.
  PRINT "======================================================".
  PRINT " IFC TEST: RWY 09 -> 090/1.5min@250 -> 090/1.5min@150".
  PRINT "======================================================".
  PRINT "Vessel : " + SHIP:NAME.
  PRINT "Leg 1  : TAKEOFF  RWY 09".
  PRINT "Leg 2  : CRUISE   090 deg / 1.5 min / 1500 m / 250 m/s".
  PRINT "Leg 3  : CRUISE   090 deg / 1.5 min / 1500 m / 150 m/s".
  PRINT " ".

  LOCAL arm_selection IS _PROMPT_ARM_RWY09_STEPDOWN().
  IF arm_selection = "q" OR arm_selection = "Q" {
    PRINT "Test aborted.".
    WAIT 0.
    RETURN.
  }

  PRINT " ".
  PRINT "Loading IFC...".
  SET IFC_SKIP_INTERACTIVE TO TRUE.

  LOCAL ifc_main_path IS "0:/Integrated Flight Computer/ifc_main.ks".
  IF NOT EXISTS(ifc_main_path) {
    PRINT "ERROR: missing " + ifc_main_path.
    SET IFC_SKIP_INTERACTIVE TO FALSE.
    WAIT 0.
    RETURN.
  }
  RUNONCEPATH(ifc_main_path).

  IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
    PRINT "No config for [" + SHIP:NAME + "] - using defaults.".
    SET ACTIVE_AIRCRAFT TO LEXICON("name", SHIP:NAME).
  }
  PRINT "Aircraft: " + ACTIVE_AIRCRAFT["name"].

  LOCAL flight_plan IS LIST().

  flight_plan:ADD(LEXICON(
    "type",   LEG_TAKEOFF,
    "params", LEXICON("rwy_id", "09")
  )).

  flight_plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_time",
      "course_deg", heading_deg,
      "time_min",   leg_time_min,
      "alt_m",      cruise_alt_m,
      "spd",        speed_high_mps
    )
  )).

  flight_plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_time",
      "course_deg", heading_deg,
      "time_min",   leg_time_min,
      "alt_m",      cruise_alt_m,
      "spd",        speed_low_mps
    )
  )).

  PRINT "Plan: " + flight_plan:LENGTH + " legs. Engaging IFC...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(flight_plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "RWY09 autospoiler step-down test complete.".
}

RUN_KSC_RWY09_SPOILER_STEPDOWN_TEST().
