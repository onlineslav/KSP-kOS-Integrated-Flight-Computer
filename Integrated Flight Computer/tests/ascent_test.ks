@LAZYGLOBAL OFF.

// ============================================================
// ascent_test.ks - IFC ascent test program
//
// Pipeline:
//   ifc_testflight_bootloader.ks -> this script -> IFC ascent phase
//
// Requires explicit operator arm in terminal, then executes
// an IFC ascent leg to a 100 km apoapsis target.
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.
GLOBAL ASCENT_TARGET_APO_M IS 100000.

FUNCTION _PROMPT_RWY {
  PRINT "Select takeoff runway:".
  PRINT "  1 = RWY 09".
  PRINT "  2 = RWY 27".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "1" OR sel = "2" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  IF sel = "1" { RETURN "09". }
  RETURN "27".
}

FUNCTION _PROMPT_ARM_ASCENT {
  PRINT "Type A to ARM ascent, or Q to abort.".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "a" OR sel = "A" OR sel = "q" OR sel = "Q" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  RETURN sel.
}

FUNCTION RUN_ASCENT_TEST {
  CLEARSCREEN.
  PRINT "==============================================".
  PRINT "IFC TESTFLIGHT: ASCENT TEST".
  PRINT "==============================================".
  PRINT " ".
  PRINT "Vessel: " + SHIP:NAME.
  PRINT "Target apoapsis: 100.0 km (" + ROUND(ASCENT_TARGET_APO_M, 0) + " m)".
  PRINT " ".

  LOCAL arm_sel IS _PROMPT_ARM_ASCENT().
  IF arm_sel = "q" OR arm_sel = "Q" {
    PRINT "Ascent test aborted.".
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

  // Load aircraft config from ship name; fall back to minimal lexicon.
  IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
    PRINT "No aircraft config found for [" + SHIP:NAME + "] - using defaults.".
    SET ACTIVE_AIRCRAFT TO LEXICON("name", SHIP:NAME).
  }

  // Override (or add) ascent apoapsis target for this test run.
  SET ACTIVE_AIRCRAFT["ascent_apoapsis_target_m"] TO ASCENT_TARGET_APO_M.

  PRINT "Aircraft: " + ACTIVE_AIRCRAFT["name"].
  PRINT "Ascent target: " + ROUND(ASCENT_TARGET_APO_M / 1000, 1) + " km".
  PRINT " ".
  LOCAL plan IS LIST().

  // If started from runway/standstill, run takeoff first, then ascent.
  IF ALT:RADAR < 20 AND SHIP:AIRSPEED < 40 {
    PRINT "Ground start detected - prepending TAKEOFF leg.".
    LOCAL rwy_id IS _PROMPT_RWY().
    plan:ADD(LEXICON(
      "type",   LEG_TAKEOFF,
      "params", LEXICON("rwy_id", rwy_id)
    )).
  }

  plan:ADD(LEXICON(
    "type",   LEG_ASCENT,
    "params", LEXICON()
  )).

  // Performance profile for ascent test:
  // reduce terminal/UI churn and file I/O so control loop cadence stays high.
  LOCAL prev_disp_period IS IFC_DISPLAY_PERIOD.
  LOCAL prev_hdr_period IS IFC_HEADER_PERIOD.
  LOCAL prev_sec_period IS IFC_SECONDARY_PERIOD.
  LOCAL prev_log_bar_period IS IFC_LOGGER_PERIOD.
  LOCAL prev_csv_period IS IFC_CSV_LOG_PERIOD.
  LOCAL prev_fast_mode IS IFC_FAST_MODE.

  SET IFC_FAST_MODE TO TRUE.
  SET IFC_DISPLAY_PERIOD TO 0.50.
  SET IFC_HEADER_PERIOD TO 0.50.
  SET IFC_SECONDARY_PERIOD TO 0.50.
  SET IFC_LOGGER_PERIOD TO 1.00.
  SET IFC_CSV_LOG_PERIOD TO 5.00.

  PRINT "IFC perf profile: FAST MODE ON, UI 2 Hz, CSV log every 5 s.".
  PRINT "Engaging IFC ascent test plan...".

  _RUN_FLIGHT_PLAN(plan).

  // Restore global rates for non-test runs.
  SET IFC_DISPLAY_PERIOD TO prev_disp_period.
  SET IFC_HEADER_PERIOD TO prev_hdr_period.
  SET IFC_SECONDARY_PERIOD TO prev_sec_period.
  SET IFC_LOGGER_PERIOD TO prev_log_bar_period.
  SET IFC_CSV_LOG_PERIOD TO prev_csv_period.
  SET IFC_FAST_MODE TO prev_fast_mode.

  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT " ".
  PRINT "Ascent test complete.".
}

RUN_ASCENT_TEST().
