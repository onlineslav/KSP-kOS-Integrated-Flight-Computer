@LAZYGLOBAL OFF.

// ============================================================
// engine_test_boot.ks - kOS bootloader for engine test stands
//
// Set this in the kOS part right-click menu:
//   Boot File: boot/engine_test_boot.ks
//
// Behavior:
//   - wait for vessel unpack + archive connectivity
//   - open kOS terminal automatically
//   - run IFC engine characterization test script
// ============================================================

WAIT UNTIL SHIP:UNPACKED.
WAIT UNTIL HOMECONNECTION:ISCONNECTED.

LOCAL open_terminal_on_boot IS TRUE.
IF open_terminal_on_boot {
  LOCAL proc_module IS CORE:PART:GETMODULE("kOSProcessor").
  IF proc_module <> 0 {
    proc_module:DOEVENT("Open Terminal").
  }
}

SET TERMINAL:VISUALBEEP TO FALSE.
SET TERMINAL:CHARHEIGHT TO 18.

CLEARSCREEN.
PRINT "========================================".
PRINT " IFC ENGINE TEST - BOOTLOADER           ".
PRINT "========================================".
PRINT "Vessel: " + SHIP:NAME.
PRINT "Core:   " + CORE:PART.
PRINT "UT:     " + ROUND(TIME:SECONDS, 1).
PRINT "".

LOCAL main_script IS "0:/Integrated Flight Computer/tests/engine_test.ks".
IF EXISTS(main_script) {
  PRINT "Loading " + main_script + " ...".
  RUNPATH(main_script).
} ELSE {
  PRINT "ERROR: missing script".
  PRINT main_script.
}

PRINT "engine_test_boot: script returned.".
