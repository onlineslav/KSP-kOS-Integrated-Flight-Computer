@LAZYGLOBAL OFF.

// ============================================================
// vtol_test_auto_boot.ks - boot handoff for autonomous VTOL tests
//
// Set this as boot file in the kOS processor:
//   boot/vtol_test_auto_boot.ks
// ============================================================

WAIT UNTIL SHIP:UNPACKED.
WAIT UNTIL HOMECONNECTION:ISCONNECTED.

// Open terminal automatically for this test boot.
LOCAL proc_module IS CORE:PART:GETMODULE("kOSProcessor").
IF proc_module <> 0 {
  IF proc_module:HASEVENT("Open Terminal") {
    proc_module:DOEVENT("Open Terminal").
  }
}

// Use larger terminal text for test visibility.
SET TERMINAL:VISUALBEEP TO FALSE.
SET TERMINAL:CHARHEIGHT TO 20.

CLEARSCREEN.
PRINT "=================================".
PRINT " VTOL AUTO TEST - BOOT          ".
PRINT "=================================".
PRINT "Vessel: " + SHIP:NAME.
PRINT "UT:     " + ROUND(TIME:SECONDS, 1).
PRINT "".
PRINT "Select test flight script:".
PRINT "  [1] Previous: vtol_test_auto.ks".
PRINT "  [2] New:      vtol_test_auto_v2.ks (hover+spool tuned)".
PRINT "".
PRINT "Press 1 or 2.".

LOCAL prev_test_path IS "0:/Integrated Flight Computer/tests/vtol_test_auto.ks".
LOCAL new_test_path IS "0:/Integrated Flight Computer/tests/vtol_test_auto_v2.ks".
LOCAL selected_test_path IS "".

TERMINAL:INPUT:CLEAR().
UNTIL selected_test_path <> "" {
  LOCAL key_char IS TERMINAL:INPUT:GETCHAR().
  IF key_char = "1" {
    SET selected_test_path TO prev_test_path.
  } ELSE IF key_char = "2" {
    SET selected_test_path TO new_test_path.
  }
}

IF selected_test_path = new_test_path AND NOT EXISTS(new_test_path) {
  PRINT "New test not found, falling back to previous.".
  SET selected_test_path TO prev_test_path.
}

IF NOT EXISTS(selected_test_path) {
  PRINT "Selected test script missing:".
  PRINT selected_test_path.
} ELSE {
  PRINT "Loading: " + selected_test_path.
  WAIT 0.3.
  RUNPATH(selected_test_path).
}
