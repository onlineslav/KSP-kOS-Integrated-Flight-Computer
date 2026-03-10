@LAZYGLOBAL OFF.

// Keep boot logic minimal and robust.
LOCAL auto_open_terminal IS FALSE.
IF auto_open_terminal {
  core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
}
PRINT "+-----------------------------------------------+".
PRINT "|         kOS-AtmosphereAutopilot Debug         |".
PRINT "+-----------------------------------------------+".
PRINT "".
PRINT "AA debug boot on core: " + CORE:PART.

RUNPATH("0:/kOS-AA_Debug_new.ks").
