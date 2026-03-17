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

FUNCTION _SET_STARTUP_CAMERA {
  IF NOT ADDONS:HASSUFFIX("CAMERA") {
    RETURN.
  }

  LOCAL cam IS ADDONS:CAMERA.
  IF cam:HASSUFFIX("FLIGHTCAMERA") {
    SET cam TO cam:FLIGHTCAMERA.
  }

  IF cam:HASSUFFIX("MODE") {
    SET cam:MODE TO "LOCKED".
  }

  // Rear chase angle. If your craft faces the wrong way, change to 0.
  IF cam:HASSUFFIX("HEADING") {
    SET cam:HEADING TO 180.
  } ELSE IF cam:HASSUFFIX("HDG") {
    SET cam:HDG TO 180.
  }
}

_SET_STARTUP_CAMERA().

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
