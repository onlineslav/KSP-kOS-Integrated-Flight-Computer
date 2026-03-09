@LAZYGLOBAL OFF.

// Keep boot logic minimal and robust.
core:part:getmodule("kOSProcessor"):doevent("Open Terminal").
PRINT "+-----------------------------------------------+".
PRINT "|         kOS-AtmosphereAutopilot Debug         |".
PRINT "+-----------------------------------------------+".
PRINT "".

RUNPATH("0:/kOS-AA_Debug_new.ks").