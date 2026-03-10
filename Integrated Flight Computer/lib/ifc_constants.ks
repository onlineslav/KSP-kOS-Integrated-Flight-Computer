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
// Speed control (autothrottle PI)
// ----------------------------
GLOBAL KP_SPD           IS 0.055. // throttle / (m/s)   proportional
GLOBAL KI_SPD           IS 0.012. // throttle / (m/s*s)  integral
GLOBAL THR_INTEGRAL_LIM IS 20.    // m/s*s  anti-windup clamp on integral
GLOBAL MIN_APPROACH_THR IS 0.     // hard throttle floor on approach

// ----------------------------
// Enroute descent to fix
// ----------------------------
// Proportional FPA from altitude error during FLY_TO_FIX legs
GLOBAL KP_ALT_FPA  IS 0.005.  // deg FPA / m altitude error
GLOBAL MAX_DESC_FPA IS -6.0.  // deg  steepest descent allowed enroute
GLOBAL MAX_CLIMB_FPA IS 5.0.  // deg  steepest climb allowed enroute

// ----------------------------
// Flap detent stepping
// ----------------------------
GLOBAL FLAP_STEP_INTERVAL IS 0.35. // s minimum time between flap detent steps
GLOBAL FLAP_VFE_HYST      IS 2.0.  // m/s hold margin before forced retract

// ----------------------------
// Flare
// ----------------------------
GLOBAL FLARE_AGL_M      IS 25.   // m radar AGL to trigger flare
GLOBAL FLARE_TRIGGER_HYST_M IS 1.0. // m hysteresis for flare trigger re-arm
GLOBAL FLARE_TRIGGER_CONFIRM_S IS 0.12. // s AGL must remain below trigger to enter flare
GLOBAL FLARE_PITCH_RATE IS 0.8.  // deg/s max rate of FPA change during flare
GLOBAL FLARE_PITCH_RATE_MIN IS 0.6. // deg/s flare response at low IAS
GLOBAL FLARE_PITCH_RATE_MAX IS 1.8. // deg/s flare response at high IAS
GLOBAL FLARE_RATE_LOW_IAS   IS 40.  // m/s low-speed end of flare-rate schedule
GLOBAL FLARE_RATE_HIGH_IAS  IS 95.  // m/s high-speed end of flare-rate schedule
GLOBAL TOUCHDOWN_VS     IS -0.3. // m/s target sink rate at wheel contact
GLOBAL FLARE_MIN_ENTRY_SINK_VS IS -6.0. // m/s deepest sink accepted to seed flare profile (entries worse than this are clamped)
GLOBAL FLARE_MAX_SINK_VS IS -2.5. // m/s strongest sink command allowed in flare
GLOBAL FLARE_IAS_TO_VS_GAIN IS 0.02. // extra sink per m/s above Vref during flare
GLOBAL FLARE_BALLOON_VS_TRIGGER IS 0.15. // m/s trigger balloon recovery when climbing
GLOBAL FLARE_BALLOON_FPA_PUSH IS 1.0. // deg additional nose-down when ballooning
GLOBAL MAX_FLARE_AOA    IS 10.   // deg AoA ceiling (FAR) — freeze pitch-up if exceeded

// ----------------------------
// Touchdown / rollout
// ----------------------------
GLOBAL TOUCHDOWN_AGL_M  IS 2.    // m AGL considered touchdown (radar alt)
GLOBAL TOUCHDOWN_CONFIRM_S IS 0.12. // s touchdown conditions must persist before switching phases
GLOBAL TOUCHDOWN_FALLBACK_AGL_M IS 0.8. // m AGL fallback detector if LANDED status lags
GLOBAL TOUCHDOWN_FALLBACK_MAX_VS IS 0.2. // m/s max VS for fallback detector (descending/near-zero)
GLOBAL TOUCHDOWN_SETTLE_S IS 0.20. // s hold TOUCHDOWN phase to let gear loads settle
GLOBAL BOUNCE_RECOVERY_AGL_M IS 1.5. // m if airborne above this in rollout, return to flare
GLOBAL BOUNCE_RECOVERY_MAX_S IS 6.0. // s after touchdown where bounce recovery is allowed
GLOBAL ROLLOUT_DONE_IAS IS 3.    // m/s IAS to declare rollout complete
GLOBAL KP_ROLLOUT_ROLL  IS 0.16. // aileron authority per deg of bank (wings level)
GLOBAL KP_ROLLOUT_YAW   IS 0.05. // rudder authority per deg of heading error
GLOBAL ROLLOUT_YAW_SLEW_PER_S IS 2.5. // control units/s max rudder command rate change
GLOBAL KP_ROLLOUT_LOC   IS 0.02. // heading correction per meter of localizer error on rollout
GLOBAL MAX_ROLLOUT_LOC_CORR IS 12. // deg max centerline steering correction
GLOBAL ROLLOUT_LOC_MAX_ERR_M IS 250. // m disable localizer correction if error is implausibly large
GLOBAL ROLLOUT_LOC_MAX_AHEAD_M IS 800. // m only trust localizer when not far before threshold
GLOBAL ROLLOUT_LOC_MAX_PAST_M IS 5000. // m only trust localizer while still near runway rollout area
GLOBAL ROLLOUT_STEER_START_IAS IS 95.  // m/s begin steering toward runway heading below this speed
GLOBAL ROLLOUT_STEER_FULL_IAS  IS 45.  // m/s full wheelsteering authority by this speed
GLOBAL ROLLOUT_STEER_MIN_BLEND IS 0.10. // minimum blend toward runway/centerline even at high IAS
GLOBAL ROLLOUT_YAW_ASSIST_IAS  IS 95.  // m/s start applying rudder assist below this speed
GLOBAL ROLLOUT_ROLL_ASSIST_IAS IS 95.  // m/s start applying roll assist below this speed
GLOBAL ROLLOUT_YAW_ERR_GUARD_DEG IS 45. // deg disable rudder assist when heading error is unrealistically large
GLOBAL ROLLOUT_BRAKE_DELAY_S   IS 0.7. // s delay after touchdown before enabling wheel brakes
GLOBAL ROLLOUT_BRAKE_MAX_IAS   IS 70.  // m/s wheel brakes enabled only below this speed

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
