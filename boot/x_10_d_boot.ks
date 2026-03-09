@LAZYGLOBAL OFF.

// Keep boot logic minimal and robust.
core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
PRINT "+-----------------------------------------------+".
PRINT "|             X-10 D FLIGHT COMPUTER            |".
PRINT "|                 Version 1.0                   |".
PRINT "+-----------------------------------------------+".
PRINT "".
PRINT "X-10 D boot: init".

// Conservative defaults before main handoff.
SAS OFF.
RCS OFF.

// Path-aware handoff to main program.
IF EXISTS("0:/Vehicles/X_10_D.ks") {
  PRINT "X-10 D boot: running Vehicles/X_10_D.ks".
  RUNPATH("0:/Vehicles/X_10_D.ks").
} ELSE IF EXISTS("X_10_D.ks") {
  PRINT "X-10 D boot: running fallback X_10_D.ks".
  RUNPATH("X_10_D.ks").
} ELSE {
  PRINT "X-10 D boot: ERROR main script not found".
}

PRINT "X-10 D boot: main program returned".
