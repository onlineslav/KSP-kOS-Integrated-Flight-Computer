@LAZYGLOBAL OFF.


core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
set terminal:charheight to 20.
PRINT "+-----------------------------------------------+".
PRINT "|       INTEGRATED FLIGHT COMPUTER  v1.0        |".
PRINT "+-----------------------------------------------+".
PRINT "".

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
