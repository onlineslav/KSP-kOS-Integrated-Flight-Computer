@LAZYGLOBAL OFF.

// ============================================================
// vtol_test_auto_boot.ks - boot handoff for autonomous VTOL test
//
// Set this as boot file in the kOS processor:
//   boot/vtol_test_auto_boot.ks
// ============================================================

WAIT UNTIL SHIP:UNPACKED.
WAIT UNTIL HOMECONNECTION:ISCONNECTED.

CLEARSCREEN.
PRINT "=================================".
PRINT " VTOL AUTO TEST - BOOT          ".
PRINT "=================================".
PRINT "Vessel: " + SHIP:NAME.
PRINT "UT:     " + ROUND(TIME:SECONDS, 1).
PRINT "Loading autonomous VTOL test...".

WAIT 0.5.
RUNPATH("0:/Integrated Flight Computer/tests/vtol_test_auto.ks").
