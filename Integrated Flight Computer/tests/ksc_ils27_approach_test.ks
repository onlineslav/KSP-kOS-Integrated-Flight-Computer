@LAZYGLOBAL OFF.

// ============================================================
// ksc_ils27_approach_test.ks  -  IFC approach-only test
//
// Sequence:
//   1. ILS approach KSC RWY 27L  (30 km short approach)
//
// Position the aircraft airborne on approach before running.
//
// Usage (kOS terminal):
//   RUNPATH("0:/Integrated Flight Computer/tests/ksc_ils27_approach_test.ks").
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.

FUNCTION _PROMPT_ARM_APP {
  PRINT "Type A to ARM, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION RUN_KSC_ILS27_APPROACH_TEST {
  CLEARSCREEN.
  PRINT "==============================================".
  PRINT " IFC TEST: KSC ILS RWY 27L  (30 km approach)".
  PRINT "==============================================".
  PRINT " ".
  PRINT "Vessel : " + SHIP:NAME.
  PRINT "Leg 1  : ILS APP  RWY 27L  (30 km short approach)".
  PRINT " ".
  PRINT "Position the aircraft airborne on the 30 km approach.".
  PRINT " ".

  LOCAL arm IS _PROMPT_ARM_APP().
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

  // Leg 1: ILS approach to KSC RWY 27L, entering via the 30 km fix.
  // short_approach TRUE selects PLATE_KSC_ILS27L_SHORT
  // (fixes: KSC_IAF_27L_30 -> KSC_FAF_27L -> ILS track -> autoland).
  plan:ADD(LEXICON(
    "type",   LEG_APPROACH,
    "params", LEXICON("rwy_id", "27L", "short_approach", TRUE)
  )).

  PRINT "Plan: " + plan:LENGTH + " leg.  Engaging IFC...".
  PRINT " ".

  _RUN_FLIGHT_PLAN(plan).

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "KSC ILS 27L approach test complete.".
}

RUN_KSC_ILS27_APPROACH_TEST().
