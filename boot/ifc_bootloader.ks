@LAZYGLOBAL OFF.


// Keep terminal popup optional so accidental secondary boots do not steal focus.
LOCAL auto_open_terminal IS FALSE.
IF auto_open_terminal {
  core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
}
set terminal:charheight to 18.
PRINT "+-----------------------------------------------+".
PRINT "|       INTEGRATED FLIGHT COMPUTER  v1.0        |".
PRINT "+-----------------------------------------------+".
PRINT "".
PRINT "IFC boot on core: " + CORE:PART.

SAS OFF.
RCS OFF.

LOCAL main IS "0:/Integrated Flight Computer/ifc_main.ks".

IF EXISTS(main) {
  PRINT "IFC boot: loading " + main.
  RUNPATH(main).
} ELSE {
  PRINT "IFC boot: ERROR - ifc_main.ks not found at:".
  PRINT "  " + main.
}

PRINT "IFC boot: main program returned".
