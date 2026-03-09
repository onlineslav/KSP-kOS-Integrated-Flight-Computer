@LAZYGLOBAL OFF.

// ============================================================
// ifc_constants.ks  -  Integrated Flight Computer
// All tunables.  Edit here; do not hardcode values elsewhere.
// ============================================================

// ----------------------------
// Loop timing
// ----------------------------
GLOBAL IFC_LOOP_DT         IS 0.05.   // s  (20 Hz)
GLOBAL IFC_TELEMETRY_PERIOD IS 1.0.   // s  between HUD refreshes

// ----------------------------
// Phase names  (top-level)
// ----------------------------
GLOBAL PHASE_APPROACH   IS "APPROACH".
GLOBAL PHASE_FLARE      IS "FLARE".
GLOBAL PHASE_TOUCHDOWN  IS "TOUCHDOWN".
GLOBAL PHASE_ROLLOUT    IS "ROLLOUT".
GLOBAL PHASE_DONE       IS "DONE".

// ----------------------------
// Approach sub-phase names
// ----------------------------
GLOBAL SUBPHASE_FLY_TO_FIX  IS "FLY_TO_FIX".
GLOBAL SUBPHASE_ILS_TRACK   IS "ILS_TRACK".

// ----------------------------
// Navigation / waypoint capture
// ----------------------------
GLOBAL FIX_CAPTURE_RADIUS  IS 1500.   // m  distance to consider a fix "passed"
GLOBAL LOC_CAPTURE_M       IS 200.    // m  lateral deviation to arm ILS_TRACK
GLOBAL GS_CAPTURE_M        IS 80.     // m  vertical deviation to arm ILS_TRACK

// ----------------------------
// ILS control gains  (PD outer loop → AA Director)
// ----------------------------
// Localizer:  heading correction (deg) per meter and per m/s of lateral deviation
GLOBAL KP_LOC  IS 0.04.    // deg / m
GLOBAL KD_LOC  IS 0.25.    // deg / (m/s)
GLOBAL MAX_LOC_CORR IS 25. // deg  max heading correction from centerline

// Glideslope:  FPA correction (deg) per meter and per m/s of vertical deviation
GLOBAL KP_GS   IS 0.30.    // deg / m
GLOBAL KD_GS   IS 0.80.    // deg / (m/s)
GLOBAL MAX_GS_CORR_DN IS 4.0. // deg  max extra nose-down beyond nominal GS
GLOBAL MAX_GS_CORR_UP IS 3.0. // deg  max nose-up relief above nominal GS

// ----------------------------
// Speed control (autothrottle)
// ----------------------------
GLOBAL KP_SPD       IS 0.04.  // throttle / (m/s)  proportional
GLOBAL MIN_APPROACH_THR IS 0. // hard throttle floor on approach

// ----------------------------
// Enroute descent to fix
// ----------------------------
// Proportional FPA from altitude error during FLY_TO_FIX legs
GLOBAL KP_ALT_FPA  IS 0.005.  // deg FPA / m altitude error
GLOBAL MAX_DESC_FPA IS -6.0.  // deg  steepest descent allowed enroute
GLOBAL MAX_CLIMB_FPA IS 5.0.  // deg  steepest climb allowed enroute

// ----------------------------
// Flare
// ----------------------------
GLOBAL FLARE_AGL_M    IS 15.   // m radar AGL to trigger flare
GLOBAL FLARE_PITCH    IS 3.0.  // deg nose-up pitch target in flare
GLOBAL FLARE_PITCH_RATE IS 0.8.// deg/s pitch-up rate during flare

// ----------------------------
// Touchdown / rollout
// ----------------------------
GLOBAL TOUCHDOWN_AGL_M  IS 2.  // m AGL considered touchdown (radar alt)
GLOBAL ROLLOUT_DONE_IAS IS 3.  // m/s IAS to declare rollout complete

// ----------------------------
// AA limits (set on FBW init)
// ----------------------------
GLOBAL AA_MAX_AOA       IS 12.  // deg
GLOBAL AA_MAX_G         IS 3.5.
GLOBAL AA_MAX_SIDESLIP  IS 5.   // deg
GLOBAL AA_MAX_SIDE_G    IS 1.5.

// ----------------------------
// Beacon type tags
// ----------------------------
GLOBAL BTYPE_ILS  IS "ILS".
GLOBAL BTYPE_VOR  IS "VOR".
GLOBAL BTYPE_IAF  IS "IAF".  // Initial Approach Fix
GLOBAL BTYPE_FAF  IS "FAF".  // Final Approach Fix
