@LAZYGLOBAL OFF.

// ============================================================
// cruise_alt_speed_test.ks - IFC cruise-alt native AA speed-mode test
//
// Purpose:
//   Validate phase_cruise_alt.ks as a drop-in RUN_CRUISE replacement.
//
// What this script does:
//   1) Loads ifc_main without interactive menu.
//   2) RUNPATH-loads phase_cruise_alt.ks to override RUN_CRUISE().
//   3) Builds a test flight plan:
//      - If on runway: TAKEOFF + CRUISE (course_dist)
//      - If airborne:  CRUISE (course_dist)
//   4) Executes _RUN_FLIGHT_PLAN(plan).
//
// Usage:
//   RUNPATH("0:/Integrated Flight Computer/tests/cruise_alt_speed_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM {
  PRINT "Type A to ARM this test, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION _PROMPT_RWY {
  PRINT "Select takeoff runway: 1=09, 2=27".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "1" OR sel = "2" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  IF sel = "1" { RETURN "09". }
  RETURN "27".
}

FUNCTION _PROMPT_PROFILE {
  PRINT "Select cruise profile:".
  PRINT "  1 = 2000 m / 180 m/s / 20 nm".
  PRINT "  2 = 3000 m / 220 m/s / 30 nm".
  PRINT "  3 = 5000 m / 260 m/s / 40 nm".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "1" OR sel = "2" OR sel = "3" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  IF sel = "1" { RETURN LEXICON("alt_m", 2000, "spd", 180, "dist_nm", 20). }
  IF sel = "2" { RETURN LEXICON("alt_m", 3000, "spd", 220, "dist_nm", 30). }
  RETURN LEXICON("alt_m", 5000, "spd", 260, "dist_nm", 40).
}

FUNCTION _GET_COMPASS_HDG_NOW {
  LOCAL fwd IS SHIP:FACING:FOREVECTOR.
  LOCAL up_v IS SHIP:UP:VECTOR.
  LOCAL north_v IS SHIP:NORTH:VECTOR.
  LOCAL east_v IS VCRS(up_v, north_v):NORMALIZED.
  LOCAL horiz IS (fwd - up_v * VDOT(fwd, up_v)):NORMALIZED.
  RETURN MOD(ARCTAN2(VDOT(horiz, east_v), VDOT(horiz, north_v)) + 360, 360).
}

FUNCTION _AA_NATIVE_SPEEDMODE_SUPPORTED {
  IF NOT ADDONS:AVAILABLE("AA") { RETURN FALSE. }
  IF NOT ADDONS:HASSUFFIX("AA") { RETURN FALSE. }
  LOCAL aa IS ADDONS:AA.
  IF aa = 0 { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("CRUISE") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("HEADING") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("ALTITUDE") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("SPEEDCONTROL") { RETURN FALSE. }
  IF NOT aa:HASSUFFIX("SPEED") { RETURN FALSE. }
  RETURN TRUE.
}

FUNCTION RUN_CRUISE_ALT_SPEED_TEST {
  CLEARSCREEN.
  PRINT "================================================".
  PRINT " IFC TEST: CRUISE ALT (AA CRUISE + SPEEDCONTROL)".
  PRINT "================================================".
  PRINT "Vessel: " + SHIP:NAME.
  PRINT " ".

  LOCAL arm IS _PROMPT_ARM().
  IF arm = "q" OR arm = "Q" {
    PRINT "Test aborted.".
    WAIT 0.
    RETURN.
  }

  LOCAL profile IS _PROMPT_PROFILE().

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

  LOCAL phase_alt IS "0:/Integrated Flight Computer/phases/phase_cruise_alt.ks".
  IF NOT EXISTS(phase_alt) {
    PRINT "ERROR: missing " + phase_alt.
    SET IFC_SKIP_INTERACTIVE TO FALSE.
    WAIT 0.
    RETURN.
  }

  // Drop-in override: replaces RUN_CRUISE implementation for this CPU session.
  RUNPATH(phase_alt).

  IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
    PRINT "No config for [" + SHIP:NAME + "] - using defaults.".
    SET ACTIVE_AIRCRAFT TO LEXICON("name", SHIP:NAME).
  }

  LOCAL aa_native_ok IS _AA_NATIVE_SPEEDMODE_SUPPORTED().
  LOCAL aa_native_label IS "NO (fallback path expected)".
  IF aa_native_ok { SET aa_native_label TO "YES". }
  PRINT "AA native cruise speed-mode suffix set: " + aa_native_label.

  LOCAL plan IS LIST().
  LOCAL ground_start IS ALT:RADAR < 20 AND SHIP:AIRSPEED < 40.
  LOCAL course_deg IS ROUND(_GET_COMPASS_HDG_NOW(), 1).

  IF ground_start {
    LOCAL rwy_id IS _PROMPT_RWY().
    LOCAL rwy_hdg IS 90.
    IF rwy_id = "27" { SET rwy_hdg TO 270. }
    SET course_deg TO rwy_hdg.
    plan:ADD(LEXICON(
      "type",   LEG_TAKEOFF,
      "params", LEXICON("rwy_id", rwy_id)
    )).
  }

  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_dist",
      "course_deg", course_deg,
      "dist_nm",    profile["dist_nm"],
      "alt_m",      profile["alt_m"],
      "spd",        profile["spd"]
    )
  )).

  PRINT " ".
  IF ground_start {
    PRINT "Plan: TAKEOFF + CRUISE(course_dist)".
  } ELSE {
    PRINT "Plan: CRUISE(course_dist) from current flight state".
  }
  PRINT "Cruise target: " + ROUND(profile["alt_m"], 0) + " m, "
    + ROUND(profile["spd"], 0) + " m/s, "
    + ROUND(profile["dist_nm"], 1) + " nm, hdg "
    + ROUND(course_deg, 1) + " deg.".
  PRINT "Engaging IFC flight plan...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "Cruise ALT speed-mode test complete.".
}

RUN_CRUISE_ALT_SPEED_TEST().
