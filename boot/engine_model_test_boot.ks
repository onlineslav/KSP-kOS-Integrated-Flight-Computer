@LAZYGLOBAL OFF.

// ============================================================
// engine_model_test_boot.ks  -  kOS boot file
//
// Set this as the boot file in the kOS module right-click menu:
//   Boot File: boot/engine_model_test_boot.ks
//
// On vessel load kOS will:
//   1) Wait for physics + archive connection
//   2) Open the kOS terminal automatically
//   3) Launch the engine model test script
// ============================================================

WAIT UNTIL SHIP:UNPACKED.
WAIT UNTIL HOMECONNECTION:ISCONNECTED.

// Open terminal automatically
LOCAL proc_module IS CORE:PART:GETMODULE("kOSProcessor").
IF proc_module:HASEVENT("Open Terminal") {
    proc_module:DOEVENT("Open Terminal").
}

SET TERMINAL:VISUALBEEP TO FALSE.
SET TERMINAL:CHARHEIGHT TO 18.

CLEARSCREEN.
PRINT "========================================".
PRINT " ENGINE MODEL TEST — BOOTLOADER         ".
PRINT "========================================".
PRINT "Vessel : " + SHIP:NAME.
PRINT "Core   : " + CORE:PART:NAME.
PRINT "UT     : " + ROUND(TIME:SECONDS, 1).
PRINT "".

LOCAL main_script IS "0:/Integrated Flight Computer/tests/engine_model_test.ks".
IF EXISTS(main_script) {
    PRINT "Loading " + main_script + " ...".
    WAIT 0.5.
    RUNPATH(main_script).
} ELSE {
    PRINT "ERROR: script not found:".
    PRINT main_script.
    PRINT "".
    PRINT "Check that the IFC scripts are synced to the archive.".
}

PRINT "engine_model_test_boot: script returned.".
