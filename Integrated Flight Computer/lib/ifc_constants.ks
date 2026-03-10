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
GLOBAL PHASE_TAKEOFF    IS "TAKEOFF".
GLOBAL PHASE_DONE       IS "DONE".

// ----------------------------
// Approach sub-phase names
// ----------------------------
GLOBAL SUBPHASE_FLY_TO_FIX  IS "FLY_TO_FIX".
GLOBAL SUBPHASE_ILS_TRACK   IS "ILS_TRACK".

// ----------------------------
// Takeoff sub-phase names
// ----------------------------
GLOBAL SUBPHASE_TO_PREFLIGHT   IS "TO_PREFLIGHT".
GLOBAL SUBPHASE_TO_GROUND_ROLL IS "TO_GROUND_ROLL".
GLOBAL SUBPHASE_TO_ROTATE      IS "TO_ROTATE".
GLOBAL SUBPHASE_TO_CLIMB       IS "TO_CLIMB".

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
// Speed control (cascade autothrottle: speed outer loop + acceleration inner loop)
// ----------------------------
// Outer loop: speed error (m/s) → commanded acceleration a_cmd (m/s²).
// Inner loop: acceleration error (a_cmd - a_actual) → throttle.
//   a_actual = d(IAS)/dt, low-pass filtered to suppress sensor noise.
// Integral on speed error provides steady-state trim throttle (drag/slope).
// No physics model required; both loops close on measured quantities.
GLOBAL KP_SPD_ACL       IS 0.20.  // m/s²/(m/s)  outer loop: accel commanded per m/s speed error
GLOBAL ACL_MAX          IS 1.5.   // m/s²  outer loop clamp on commanded acceleration
GLOBAL KP_ACL_THR       IS 0.25.  // throttle/(m/s²)  inner loop: throttle per m/s² of accel error
GLOBAL ACL_FILTER_ALPHA IS 0.80.  // 0..1  low-pass weight on measured acceleration (higher = smoother)
GLOBAL KI_SPD           IS 0.012. // throttle/(m/s*s)  integral trim
GLOBAL THR_INTEGRAL_LIM IS 20.    // m/s*s  anti-windup clamp on integral
GLOBAL THR_SLEW_PER_S   IS 0.40.  // /s    max throttle change per second
GLOBAL MIN_APPROACH_THR IS 0.     // hard throttle floor on approach

// Derived approach speed schedule (minimal tuning):
// - Pre-capture/intercept: target Vint = Vapp + derived additive.
// - Final: target Vapp.
// - Short final: blend Vapp toward Vref.
GLOBAL APP_SPD_INTERCEPT_GAIN      IS 0.60. // additive = gain * (Vapp - Vref)
GLOBAL APP_SPD_INTERCEPT_MIN_ADD   IS 4.0.  // m/s minimum additive for intercept phase
GLOBAL APP_SPD_INTERCEPT_MAX_ADD   IS 9.0.  // m/s maximum additive for intercept phase
GLOBAL APP_FINAL_CAPTURE_CONFIRM_S IS 1.0.  // s LOC/GS capture must persist before final-speed mode
GLOBAL APP_FINAL_RELEASE_FACTOR    IS 1.35. // hysteresis factor on LOC/GS capture limits to exit final-speed mode
GLOBAL APP_SHORT_FINAL_AGL_M       IS 60.0. // m AGL where Vapp -> Vref blend starts
GLOBAL APP_SPEED_TGT_SLEW_PER_S    IS 0.8.  // m/s/s max speed-target change rate (prevents throttle step jumps)
GLOBAL APP_SHORT_FINAL_CAP_WHEN_NOT_FINAL IS TRUE. // if TRUE, short-final Vapp->Vref blend can cap speed even before final capture

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
GLOBAL FLARE_ROUNDOUT_AGL_M IS 0.0. // m AGL final roundout blend toward touchdown target (0 = disabled)
GLOBAL FLARE_ROUNDOUT_STRENGTH IS 1.0. // 0..1 blend strength toward flare_touchdown_vs in roundout zone
GLOBAL FLARE_BALLOON_VS_TRIGGER IS 0.15. // m/s trigger balloon recovery when climbing
GLOBAL FLARE_BALLOON_FPA_PUSH IS 1.0. // deg additional nose-down when ballooning


// ----------------------------
// Touchdown / rollout
// ----------------------------
GLOBAL TOUCHDOWN_AGL_M  IS 2.    // m AGL considered touchdown (radar alt)
GLOBAL TOUCHDOWN_CONFIRM_S IS 0.12. // s touchdown conditions must persist before switching phases
GLOBAL TOUCHDOWN_CONFIRM_MAX_ABS_VS IS 2.5. // m/s max |VS| allowed when committing FLARE->TOUCHDOWN
GLOBAL TOUCHDOWN_FALLBACK_AGL_M IS 0.8. // m AGL fallback detector if LANDED status lags
GLOBAL TOUCHDOWN_FALLBACK_MAX_VS IS 0.2. // m/s max VS for fallback detector (descending/near-zero)
GLOBAL TOUCHDOWN_SETTLE_S IS 0.20. // s hold TOUCHDOWN phase to let gear loads settle
GLOBAL BOUNCE_RECOVERY_AGL_M IS 2.5. // m if airborne above this in touchdown/rollout, consider bounce recovery
GLOBAL BOUNCE_RECOVERY_MIN_VS IS 0.6. // m/s minimum upward VS to count as a real bounce
GLOBAL BOUNCE_RECOVERY_CONFIRM_S IS 0.30. // s airborne criteria must persist before bounce recovery
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
GLOBAL ROLLOUT_YAW_FADE_IAS    IS 20.  // m/s fade yaw assist toward zero below this speed
GLOBAL ROLLOUT_YAW_MAX_CMD     IS 0.50. // max magnitude of yaw command target
GLOBAL ROLLOUT_BRAKE_DELAY_S   IS 0.7. // s delay after touchdown before enabling wheel brakes
GLOBAL ROLLOUT_BRAKE_MAX_IAS   IS 70.  // m/s wheel brakes enabled only below this speed
GLOBAL ROLLOUT_NOSE_HOLD_CMD    IS 0.0. // pitch command to hold nose up on rollout (0 = disabled)
GLOBAL ROLLOUT_NOSE_RELEASE_IAS IS 35.  // m/s fade nose-hold command to zero by this IAS
GLOBAL ROLLOUT_NOSE_HOLD_MIN_S  IS 1.0. // s keep full nose-hold after rollout entry before releasing by IAS
GLOBAL ROLLOUT_NOSE_MIN_REF_DEG IS 2.0. // deg minimum touchdown pitch reference to hold after mains contact
GLOBAL ROLLOUT_NOSE_TARGET_PITCH_DEG IS 0.0. // deg final rollout pitch target once nose is lowered
GLOBAL ROLLOUT_NOSE_TARGET_SLEW_DPS IS 1.2. // deg/s max target attitude change while lowering nose
GLOBAL ROLLOUT_PITCH_HOLD_KP    IS 0.08. // pitch command per deg of (target - actual) pitch attitude error
GLOBAL ROLLOUT_PITCH_MAX_CMD    IS 0.35. // max magnitude of closed-loop rollout pitch command
GLOBAL ROLLOUT_PITCH_MAX_DOWN_CMD IS 0.18. // max nose-down rollout pitch command magnitude to avoid nosewheel slams
GLOBAL ROLLOUT_PITCH_SLEW_PER_S IS 1.2. // control units/s max pitch command rate change on rollout

// ----------------------------
// AA limits (set on FBW init)
// ----------------------------
GLOBAL AA_MAX_AOA       IS 12.  // deg
GLOBAL AA_MAX_G         IS 3.5.
GLOBAL AA_MAX_SIDESLIP  IS 5.   // deg
GLOBAL AA_MAX_SIDE_G    IS 1.5.

// ----------------------------
// Bank angle limiter  (IFC-level, applied during ILS tracking)
// AA does not expose a native bank limit; IFC fades the heading
// correction and adds a counter-correction when over limit.
// ----------------------------
GLOBAL AA_MAX_BANK   IS 35.   // deg  max roll bank during approach
GLOBAL KP_BANK_LIMIT IS 0.5.  // deg heading corr per deg of bank overshoot

// ----------------------------
// AoA speed protection  (FAR-only; requires a_crit in aircraft config)
// When AoA approaches a_crit, the speed target floor is raised.
// ----------------------------
GLOBAL APP_AOA_PROTECT_FRAC IS 0.90. // trigger when AoA > a_crit * this
GLOBAL APP_AOA_SPD_GAIN     IS 3.0.  // m/s added to speed floor per deg above warning AoA

// ----------------------------
// Takeoff defaults
// ----------------------------
GLOBAL TAKEOFF_V_R_DEFAULT        IS 70.   // m/s  rotate speed
GLOBAL TAKEOFF_V2_DEFAULT         IS 80.   // m/s  initial climb speed target
GLOBAL TAKEOFF_ROTATE_PITCH_TGT   IS 12.   // deg  pitch target at rotation
GLOBAL TAKEOFF_PITCH_SLEW_DPS     IS 3.0.  // deg/s  max rate to slew pitch cmd during rotation
GLOBAL TAKEOFF_ROTATE_PITCH_KP    IS 0.08. // control cmd per deg pitch error during on-ground rotate
GLOBAL TAKEOFF_ROTATE_PITCH_FF    IS 0.08. // control cmd baseline back-pressure during on-ground rotate
GLOBAL TAKEOFF_ROTATE_PITCH_MIN_CMD IS 0.12. // minimum nose-up pitch cmd while rotate target is above current pitch
GLOBAL TAKEOFF_ROTATE_PITCH_MAX_CMD IS 0.45. // max pitch control cmd during on-ground rotate
GLOBAL TAKEOFF_ROTATE_PITCH_SLEW_PER_S IS 1.6. // control units/s pitch cmd slew during on-ground rotate
GLOBAL TAKEOFF_CLIMB_FPA          IS 8.0.  // deg  FPA commanded during climb-out
GLOBAL TAKEOFF_DONE_AGL           IS 300.  // m AGL  altitude where takeoff phase ends
GLOBAL TAKEOFF_AIRBORNE_AGL_M     IS 3.    // m AGL  detect liftoff above this
GLOBAL TAKEOFF_AIRBORNE_CONFIRM_S IS 0.5.  // s  airborne must persist before gear retract
GLOBAL TAKEOFF_AIRBORNE_MIN_VS    IS 0.5.  // m/s  min vertical speed to confirm airborne
GLOBAL TAKEOFF_AUTOSTAGE          IS TRUE. // attempt STAGE during takeoff if no available thrust
GLOBAL TAKEOFF_STAGE_MAX_ATTEMPTS IS 1.    // max auto-stage attempts before giving up
GLOBAL TAKEOFF_STAGE_RETRY_S      IS 1.0.  // s between auto-stage attempts
GLOBAL TAKEOFF_ENGINE_SPOOL_TIMEOUT_S IS 8.0. // s wait in preflight for thrust before forced rollout
GLOBAL TAKEOFF_SPOOL_THRUST_FRAC  IS 0.95. // 0..1 min fraction of max available thrust required before brake release
GLOBAL TAKEOFF_SPOOL_STEADY_DKNPS IS 3.0. // kN/s max |d(available thrust)/dt| considered "steady"
GLOBAL TAKEOFF_SPOOL_STEADY_HOLD_S IS 0.6. // s thrust must remain steady before brake release
GLOBAL TAKEOFF_FLAP_SETTLE_S      IS 2.0. // s wait after last takeoff-flap step before brake release
GLOBAL TAKEOFF_MIN_AVAIL_THRUST   IS 5.0.  // kN threshold to consider engines lit
GLOBAL KP_TAKEOFF_LOC             IS 0.020. // deg heading correction per meter localizer error
GLOBAL TAKEOFF_LOC_GUARD_M        IS 120.0. // m cap localizer error used by takeoff steering
GLOBAL TAKEOFF_STEER_MAX_CORR     IS 10.0. // deg max steering heading correction on ground roll
GLOBAL TAKEOFF_STEER_HDG_RATE_KD  IS 0.20. // deg/deg/s heading-rate damping applied to steering heading
GLOBAL TAKEOFF_DIR_MAX_CORR       IS 6.0.  // deg max heading correction in rotate/climb
GLOBAL TAKEOFF_YAW_START_IAS      IS 20.0. // m/s IAS where rudder assist starts ramping in
GLOBAL TAKEOFF_YAW_FULL_IAS       IS 90.0. // m/s IAS where rudder assist reaches full gain
GLOBAL TAKEOFF_YAW_MIN_SCALE      IS 0.25. // 0..1 minimum rudder-assist scale applied from rollout start
GLOBAL KP_TAKEOFF_YAW             IS 0.025. // rudder command per deg heading error
GLOBAL KD_TAKEOFF_YAW             IS 0.00. // rudder command per deg/s heading-rate error (damping)
GLOBAL TAKEOFF_YAW_BOOST_ERR_DEG  IS 0.50. // deg heading error where yaw gain boost reaches +1x
GLOBAL TAKEOFF_YAW_BOOST_MAX      IS 0.80. // max extra yaw gain multiplier from heading-error boost
GLOBAL TAKEOFF_YAW_MAX_CMD        IS 0.30. // max rudder command magnitude
GLOBAL TAKEOFF_YAW_SLEW_PER_S     IS 2.0.  // control units/s max rudder command slew
GLOBAL TAKEOFF_CLIMB_MIN_THR      IS 0.78. // throttle floor in climb to avoid sink-back
GLOBAL TAKEOFF_CLIMB_SPD_THR_GAIN IS 0.010. // throttle trim per m/s (V2-IAS) in climb
GLOBAL TAKEOFF_CLIMB_FPA_SPD_GAIN IS 0.08. // deg FPA reduction per m/s below V2
GLOBAL TAKEOFF_CLIMB_FPA_MIN      IS 3.0.  // deg minimum climb FPA when speed-protecting
GLOBAL TAKEOFF_CLIMB_FPA_SLEW_DPS IS 1.6.  // deg/s max FPA change rate for rotate->climb handoff

// ----------------------------
// Beacon type tags
// ----------------------------
GLOBAL BTYPE_ILS  IS "ILS".
GLOBAL BTYPE_VOR  IS "VOR".
GLOBAL BTYPE_IAF  IS "IAF".  // Initial Approach Fix
GLOBAL BTYPE_FAF  IS "FAF".  // Final Approach Fix
