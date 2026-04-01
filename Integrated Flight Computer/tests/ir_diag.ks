@LAZYGLOBAL OFF.

// ir_diag.ks — prints all fields, events, and actions on every
// ModuleIRServo_v3 found on this vessel.
// Run this once, note the field names, then we can control servos
// without ADDONS:IR.

CLEARSCREEN.
LOCAL mods IS SHIP:MODULESNAMED("ModuleIRServo_v3").
PRINT "ModuleIRServo_v3 count: " + mods:LENGTH.
PRINT "".

IF mods:LENGTH = 0 {
  PRINT "None found. Check part module name.".
  PRINT "Try: ModuleIRServo, MuMechToggle, or similar.".
  WAIT 10.
} ELSE {
  LOCAL srv_mod IS mods[0]. // inspect first servo only

  PRINT "=== FIELDS ===".
  LOCAL fnames IS srv_mod:ALLFIELDS.
  LOCAL fi IS 0.
  UNTIL fi >= fnames:LENGTH {
    LOCAL fname IS fnames[fi].
    LOCAL fval  IS "?".
    SET fval TO srv_mod:GETFIELD(fname).
    PRINT "  " + fname + " = " + fval.
    SET fi TO fi + 1.
  }

  PRINT "".
  PRINT "=== EVENTS ===".
  LOCAL evts IS mod:ALLEVENTS.
  LOCAL ei IS 0.
  UNTIL ei >= evts:LENGTH {
    PRINT "  " + evts[ei].
    SET ei TO ei + 1.
  }

  PRINT "".
  PRINT "=== ACTIONS ===".
  LOCAL acts IS mod:ALLACTIONS.
  LOCAL ai IS 0.
  UNTIL ai >= acts:LENGTH {
    PRINT "  " + acts[ai].
    SET ai TO ai + 1.
  }

  PRINT "".
  PRINT "Done. Note field names above.".
}

WAIT UNTIL TERMINAL:INPUT:HASCHAR.
