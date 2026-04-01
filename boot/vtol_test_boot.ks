@LAZYGLOBAL OFF.

// ============================================================
// vtol_test_boot.ks  -  kOS boot file for VTOL test aircraft
//
// Set this as the boot file in the kOS module right-click menu:
//   Boot File: boot/vtol_test_boot.ks
//
// On vessel load kOS will run this automatically.
// It waits for physics to settle, then launches vtol_test.ks.
// ============================================================

// Wait for the physics engine to fully load the vessel.
WAIT UNTIL SHIP:UNPACKED.

// Ensure archive is reachable before trying to run anything.
WAIT UNTIL HOMECONNECTION:ISCONNECTED.

CLEARSCREEN.
PRINT "=================================".
PRINT " VTOL TEST AIRCRAFT — BOOT      ".
PRINT "=================================".
PRINT "".
PRINT "Vessel: " + SHIP:NAME.
PRINT "UT:     " + ROUND(TIME:SECONDS, 1).
PRINT "".
PRINT "Loading VTOL test script ...".

WAIT 0.5.

RUNPATH("0:/Integrated Flight Computer/tests/vtol_test.ks").
