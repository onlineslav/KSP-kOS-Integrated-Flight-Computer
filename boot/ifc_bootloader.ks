@LAZYGLOBAL OFF.



core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

set terminal:charheight to 18.
PRINT "+-----------------------------------------------+".
PRINT "|       INTEGRATED FLIGHT COMPUTER  v1.0        |".
PRINT "+-----------------------------------------------+".
PRINT "".
PRINT "IFC boot on core: " + CORE:PART.

SAS OFF.
RCS OFF.

// Keep local volume (1:) in sync with archive (0:) to avoid stale boot scripts.
LOCAL sync_local_scripts IS TRUE.

FUNCTION _SYNC_ONE {
  PARAMETER src_path, dst_path.
  IF EXISTS(src_path) {
    COPY src_path TO dst_path.
    PRINT "IFC boot sync: " + src_path + " -> " + dst_path.
  } ELSE {
    PRINT "IFC boot sync: missing source " + src_path.
  }
}

IF sync_local_scripts {
  IF NOT EXISTS("1:/boot") { CREATEDIR("1:/boot"). }

  _SYNC_ONE("0:/boot/ifc_bootloader.ks", "1:/boot/ifc_bootloader.ks").
  _SYNC_ONE("0:/boot/ifc_testflight_bootloader.ks", "1:/boot/ifc_testflight_bootloader.ks").
}

LOCAL test_boot IS "0:/boot/ifc_testflight_bootloader.ks".
LOCAL main IS "0:/Integrated Flight Computer/ifc_main.ks".

IF EXISTS(test_boot) {
  PRINT "IFC boot: delegating to testflight bootloader " + test_boot.
  RUNPATH(test_boot).
} ELSE IF EXISTS(main) {
  PRINT "IFC boot: testflight bootloader not found, loading " + main.
  RUNPATH(main).
} ELSE {
  PRINT "IFC boot: ERROR - no runnable script found.".
  PRINT "  missing: " + test_boot.
  PRINT "  " + main.
}

PRINT "IFC boot: main program returned".
