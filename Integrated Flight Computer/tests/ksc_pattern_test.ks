@LAZYGLOBAL OFF.

// ============================================================
// ksc_pattern_test.ks  -  IFC circuit test
//
// Sequence:
//   1. Takeoff from KSC RWY 09
//   2. Cruise east at 1500 m / 250 m/s for 60 km
//   3. ILS approach KSC RWY 27  (30 km short approach)
//
// Usage (kOS terminal):
//   RUNPATH("0:/Integrated Flight Computer/tests/ksc_pattern_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM {
  PRINT "Type A to ARM, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION RUN_KSC_PATTERN_TEST {
  CLEARSCREEN.
  PRINT "==============================================".
  PRINT " IFC TEST: KSC RWY 09 -> CRUISE -> RWY 27".
  PRINT "==============================================".
  PRINT " ".
  PRINT "Vessel : " + SHIP:NAME.
  PRINT "Leg 1  : TAKEOFF  RWY 09".
  PRINT "Leg 2  : CRUISE   090 deg / 60 km / 1500 m / 250 m/s".
  PRINT "Leg 3  : ILS APP  RWY 27  (30 km short approach)".
  PRINT " ".

  LOCAL arm IS _PROMPT_ARM().
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

  // ── Flight plan ────────────────────────────────────────

  LOCAL plan IS LIST().

  // Leg 1: Takeoff from KSC RWY 09 (heading 090)
  plan:ADD(LEXICON(
    "type",   LEG_TAKEOFF,
    "params", LEXICON("rwy_id", "09")
  )).

  // Leg 2: Cruise east at 1500 m / 250 m/s for 60 km.
  // nav_type "course_dist" creates a synthetic waypoint at
  // GEO_DESTINATION(current_pos, 090, 60 km) when the leg initialises.
  plan:ADD(LEXICON(
    "type",   LEG_CRUISE,
    "params", LEXICON(
      "nav_type",   "course_dist",
      "course_deg", 90,
      "dist_nm",    32.4,       // 60 km / 1.852 km per nm
      "alt_m",      1500,
      "spd",        250
    )
  )).

  // Leg 3: ILS approach to KSC RWY 27, entering via the 30 km fix.
  // short_approach TRUE selects PLATE_KSC_ILS27_SHORT
  // (fixes: KSC_IAF_27_30 -> KSC_FAF_27 -> ILS track -> autoland).
  plan:ADD(LEXICON(
    "type",   LEG_APPROACH,
    "params", LEXICON("rwy_id", "27", "short_approach", TRUE)
  )).

  PRINT "Plan: " + plan:LENGTH + " legs.  Engaging IFC...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "KSC pattern test complete.".
}

RUN_KSC_PATTERN_TEST().
