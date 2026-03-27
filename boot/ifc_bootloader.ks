@LAZYGLOBAL OFF.


// Multi-CPU guard:
// Only one processor should own terminal/UI startup to avoid state thrash.
// Tune PRIMARY_CORE_HINTS per craft as needed.
// Any matching token will designate this CPU as primary.
LOCAL PRIMARY_CORE_HINTS IS LIST("OPT0", "COCKPIT").
LOCAL core_name_raw IS ("" + CORE:PART):TOUPPER.
LOCAL core_name IS core_name_raw:REPLACE(" ", ""):REPLACE("-", ""):REPLACE("_", "").
LOCAL is_primary_core IS FALSE.
LOCAL hint_i IS 0.
UNTIL hint_i >= PRIMARY_CORE_HINTS:LENGTH OR is_primary_core {
  LOCAL hint_name IS ("" + PRIMARY_CORE_HINTS[hint_i]):TOUPPER:REPLACE(" ", ""):REPLACE("-", ""):REPLACE("_", "").
  IF hint_name = "" OR core_name:FIND(hint_name) >= 0 {
    SET is_primary_core TO TRUE.
  }
  SET hint_i TO hint_i + 1.
}

IF is_primary_core {
  core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

  SET TERMINAL:CHARHEIGHT TO 18.
  SET TERMINAL:VISUALBEEP TO FALSE.
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
      COPYPATH(src_path, dst_path).
      PRINT "IFC boot sync: " + src_path + " -> " + dst_path.
    } ELSE {
      PRINT "IFC boot sync: missing source " + src_path.
    }
  }

  IF sync_local_scripts {
    IF NOT EXISTS("1:/boot") { CREATEDIR("1:/boot"). }

    _SYNC_ONE("0:/boot/ifc_bootloader.ks", "1:/boot/ifc_bootloader.ks").
  }

  LOCAL main IS "0:/Integrated Flight Computer/ifc_main.ks".

  IF EXISTS(main) {
    PRINT "IFC boot: loading " + main.
    RUNPATH(main).
  } ELSE {
    PRINT "IFC boot: ERROR - ifc_main.ks not found.".
    PRINT "  missing: " + main.
  }

  PRINT "IFC boot: main program returned".
} ELSE {
  PRINT "IFC boot: skipped on non-primary core.".
  PRINT "  core: " + core_name_raw.
  PRINT "  hints: " + PRIMARY_CORE_HINTS.
}
