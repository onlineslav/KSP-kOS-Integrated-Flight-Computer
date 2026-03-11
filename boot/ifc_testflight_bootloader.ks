@LAZYGLOBAL OFF.



core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

set terminal:charheight to 18.
PRINT "+-----------------------------------------------+".
PRINT "|                  IFC v1.0                     |".
PRINT "|             TEST FLIGHT PROGRAM               |".
PRINT "+-----------------------------------------------+".
PRINT "".
PRINT "IFC boot on core: " + CORE:PART.

SAS OFF.
RCS OFF.

// Keep local volume (1:) in sync with archive (0:) for boot scripts only.
// Do not mirror large IFC scripts to 1: because local CPU storage is small.
LOCAL sync_local_scripts IS TRUE.

FUNCTION _SYNC_ONE {
  PARAMETER src_path, dst_path.
  IF EXISTS(src_path) {
    COPYPATH(src_path, dst_path).
    PRINT "IFC test sync: " + src_path + " -> " + dst_path.
  } ELSE {
    PRINT "IFC test sync: missing source " + src_path.
  }
}

IF sync_local_scripts {
  IF NOT EXISTS("1:/boot") { CREATEDIR("1:/boot"). }

  _SYNC_ONE("0:/boot/ifc_testflight_bootloader.ks", "1:/boot/ifc_testflight_bootloader.ks").
}

LOCAL test_script IS "0:/Integrated Flight Computer/tests/takeoff_vr_probe.ks".

IF EXISTS(test_script) {
  PRINT "IFC test boot: loading " + test_script.
  RUNPATH(test_script).
} ELSE {
  PRINT "IFC test boot: ERROR - no runnable script found.".
  PRINT "  missing: " + test_script.
}
