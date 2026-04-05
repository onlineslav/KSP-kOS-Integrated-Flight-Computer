@LAZYGLOBAL OFF.

// ============================================================
// ifc_constants.ks  -  Integrated Flight Computer
// All tunables.  Edit here; do not hardcode values elsewhere.
// ============================================================

// ----------------------------
// Loop timing
// ----------------------------
GLOBAL IFC_LOOP_DT         IS 0.02.   // s  (50 Hz)
GLOBAL IFC_TELEMETRY_PERIOD IS 1.0.   // s  between HUD refreshes
GLOBAL IFC_DEBUG_GS_DRAW_LEN_M IS 60000. // m log-projection length from threshold along GS track
GLOBAL IFC_DEBUG_GS_DRAW_WIDTH_M IS 3.0. // m draw width for glideslope visual line
GLOBAL IFC_DEBUG_GS_DRAW_LOCAL_AHEAD_M IS 250.  // m segment shown ahead of aircraft along GS axis
GLOBAL IFC_DEBUG_GS_DRAW_LOCAL_BEHIND_M IS 250. // m segment shown behind aircraft along GS axis
GLOBAL IFC_DEBUG_GS_LOG_PERIOD_S IS 1.0. // s between glideslope debug log lines

// ----------------------------
// Phase names  (top-level)
// ----------------------------
GLOBAL PHASE_PREARM     IS "PRE-ARM".
GLOBAL PHASE_APPROACH   IS "APPROACH".
GLOBAL PHASE_FLARE      IS "FLARE".
GLOBAL PHASE_TOUCHDOWN  IS "TOUCHDOWN".
GLOBAL PHASE_ROLLOUT    IS "ROLLOUT".
GLOBAL PHASE_TAKEOFF    IS "TAKEOFF".
GLOBAL PHASE_CRUISE     IS "CRUISE".
GLOBAL PHASE_ASCENT     IS "ASCENT".
GLOBAL PHASE_REENTRY    IS "REENTRY".
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
GLOBAL GS_LATCH_RELEASE_FACTOR IS 3.0. // GS capture releases only when |dev| > factor * GS_CAPTURE_M

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

// Adaptive autothrottle estimator + scheduler
GLOBAL AT_GAIN_INIT       IS 1.0.   // m/s² per throttle initial throttle effectiveness estimate
GLOBAL AT_GAIN_MIN        IS 0.08.  // m/s² per throttle floor for gain estimate
GLOBAL AT_GAIN_MAX        IS 6.0.   // m/s² per throttle cap for gain estimate
GLOBAL AT_GAIN_ALPHA      IS 0.92.  // EWMA hold factor for gain estimate
GLOBAL AT_GAIN_DTHR_MIN   IS 0.010. // min |dthr| per cycle to accept a gain sample
GLOBAL AT_TAU_INIT        IS 0.60.  // s initial effective throttle lag estimate
GLOBAL AT_TAU_MIN         IS 0.15.  // s min effective lag
GLOBAL AT_TAU_MAX         IS 3.00.  // s max effective lag
GLOBAL AT_TAU_ALPHA       IS 0.95.  // EWMA hold factor for lag estimate
GLOBAL AT_TAU_DTHR_MIN    IS 0.003. // min |dthr| per cycle to accept lag sample
GLOBAL AT_TAU_ERR_MIN     IS 0.040. // min |cmd-cur| throttle error for lag sample
GLOBAL AT_AUTH_ALPHA      IS 0.92.  // EWMA hold factor for thrust authority estimate
GLOBAL AT_IDLE_ALPHA      IS 0.96.  // EWMA hold factor for idle decel estimate
GLOBAL AT_IDLE_THR_THRESH IS 0.03.  // throttle threshold considered idle
GLOBAL AT_IDLE_DECEL_INIT IS 0.70.  // m/s² initial idle decel estimate
GLOBAL AT_IDLE_DECEL_MAX  IS 6.00.  // m/s² max idle decel sample retained
GLOBAL AT_A_UP_FRAC       IS 0.85.  // fraction of thrust authority allowed as +a_cmd clamp
GLOBAL AT_A_UP_MIN_FRAC   IS 0.60.  // min fraction of ACL_MAX kept as +a_cmd clamp floor
GLOBAL AT_A_UP_MIN        IS 0.80.  // m/s² min +a_cmd clamp
GLOBAL AT_A_UP_MAX        IS 4.50.  // m/s² max +a_cmd clamp
GLOBAL AT_A_DN_GAIN       IS 1.10.  // multiplier on observed idle decel for -a_cmd clamp
GLOBAL AT_A_DN_MIN_FRAC   IS 0.50.  // min fraction of ACL_MAX kept as -a_cmd clamp floor
GLOBAL AT_A_DN_MIN        IS 0.60.  // m/s² min -a_cmd clamp magnitude
GLOBAL AT_A_DN_MAX        IS 5.00.  // m/s² max -a_cmd clamp magnitude
GLOBAL AT_INNER_BW        IS 0.35.  // 1/s target inner-loop bandwidth for kp scheduling
GLOBAL AT_KP_THR_MIN      IS 0.08.  // min scheduled KP_ACL_THR
GLOBAL AT_KP_THR_MAX      IS 0.90.  // max scheduled KP_ACL_THR
GLOBAL AT_KI_SPD_MIN      IS 0.004. // min scheduled KI_SPD
GLOBAL AT_KI_SPD_MAX      IS 0.040. // max scheduled KI_SPD
GLOBAL AT_TAU_REF_S       IS 0.60.  // s reference lag for base throttle slew scaling
GLOBAL AT_THR_SLEW_MIN    IS 0.12.  // /s min scheduled throttle slew
GLOBAL AT_THR_SLEW_MAX    IS 0.80.  // /s max scheduled throttle slew
GLOBAL AT_INT_BLEED       IS 0.995. // integral bleed factor while blocked by anti-windup

// ----------------------------
// Autospoiler (tagged control-surface deploy-angle writer)
// ----------------------------
GLOBAL AS_ENABLED_DEFAULT    IS TRUE.          // master enable when aircraft cfg does not override
GLOBAL AS_SPOILER_TAG_DEFAULT IS "ifc_spoiler". // required part tag for spoiler discovery
GLOBAL AS_THR_IDLE_GATE      IS 0.08.          // only deploy spoilers when THROTTLE_CMD <= gate
GLOBAL AS_ERR_DEADBAND_MPS   IS 1.5.           // m/s overspeed deadband before spoiler response
GLOBAL AS_ERR_FULL_MPS       IS 20.0.          // m/s overspeed that maps to full capped deflection
GLOBAL AS_ANGLE_SLEW_DPS     IS 25.0.          // deg/s max spoiler deploy-angle command slew
GLOBAL AS_MAX_DEFLECTION_DEG IS 70.0.          // deg deploy-angle value written while spoilers are not deployed

// Cap schedule: max spoiler deploy angle as a function of IAS (used in all phases).
GLOBAL AS_SPEED_LO    IS 70.0.  // m/s
GLOBAL AS_SPEED_HI    IS 320.0. // m/s
GLOBAL AS_CAP_DEG_LO  IS 45.0.  // deg cap at/below AS_SPEED_LO
GLOBAL AS_CAP_DEG_HI  IS 6.0.   // deg cap at/above AS_SPEED_HI

// Derived approach speed schedule (minimal tuning):
// - Pre-capture/intercept: target Vint = Vapp + derived additive.
// - Final: target Vapp.
// - Short final: blend Vapp toward Vref.
GLOBAL APP_SPD_INTERCEPT_GAIN      IS 0.60. // additive = gain * (Vapp - Vref)
GLOBAL APP_SPD_INTERCEPT_MIN_ADD   IS 4.0.  // m/s minimum additive for intercept phase
GLOBAL APP_SPD_INTERCEPT_MAX_ADD   IS 9.0.  // m/s maximum additive for intercept phase
// Far from the terminal area, hold enroute speed and delay slowing to Vint.
// Intercept speed is armed when either:
// - distance to threshold is inside APP_SPD_INTERCEPT_ARM_DIST_M
// - altitude is below APP_SPD_INTERCEPT_ARM_ALT_M
GLOBAL APP_SPD_ENROUTE_TARGET      IS -1.0. // m/s <=0 => auto (use CRUISE_SPD_MPS, else current IAS)
GLOBAL APP_SPD_INTERCEPT_ARM_DIST_M IS 30000.0. // m baseline distance gate for slowing from enroute to intercept speed
GLOBAL APP_SPD_INTERCEPT_ARM_ALT_M  IS 2600.0.  // m MSL altitude gate for arming intercept speed
GLOBAL APP_SPD_INTERCEPT_RELEASE_FACTOR IS 1.25. // hysteresis multiplier to keep gate from chattering
GLOBAL APP_FINAL_CAPTURE_CONFIRM_S IS 1.0.  // s LOC/GS capture must persist before final-speed mode
GLOBAL APP_FINAL_RELEASE_FACTOR    IS 1.35. // hysteresis factor on LOC/GS capture limits to exit final-speed mode
GLOBAL APP_SHORT_FINAL_AGL_M       IS 60.0. // m AGL where Vapp -> Vref blend starts
GLOBAL APP_SPEED_TGT_SLEW_PER_S    IS 0.8.  // m/s/s max speed-target change rate (prevents throttle step jumps)
GLOBAL APP_SHORT_FINAL_CAP_WHEN_NOT_FINAL IS TRUE. // if TRUE, short-final Vapp->Vref blend can cap speed even before final capture

// ----------------------------
// Enroute descent to fix
// ----------------------------
// Altitude hold for cruise and FLY_TO_FIX legs.
// fpa_cmd = -(alt_err * KP + vs * KD), clamped to [MAX_DESC, MAX_CLIMB].
// KD damps the phugoid: 10 m/s VS contributes 0.5 deg of opposing FPA.
GLOBAL KP_ALT_FPA     IS 0.005.  // deg FPA / m altitude error
GLOBAL KD_ALT_FPA     IS 0.05.   // deg FPA / (m/s) vertical speed  (damping)
GLOBAL KD_FTF_ALT_FPA IS 0.3.    // FLY_TO_FIX VS damping — larger than KD_ALT_FPA to suppress phugoid during speed transitions
GLOBAL MAX_DESC_FPA IS -6.0.  // deg  steepest descent allowed enroute
GLOBAL MAX_CLIMB_FPA IS 5.0.  // deg  steepest climb allowed enroute
// IFC FPA->Director pitch mapping mode:
// FALSE (default): FPA guidance is sent as-is as Director pitch command.
// TRUE:            IFC converts FPA guidance to pitch (pitch_cmd = fpa_cmd + AoA).
// Keep FALSE unless flight-test evidence shows a specific aircraft needs AoA add.
GLOBAL AA_DIR_ADD_AOA_COMP IS FALSE.
// Absolute pitch cap for low-height tailstrike protection.
// Intended to be overridden per-aircraft by tailstrike_pitch_max_deg.
GLOBAL TAILSTRIKE_PITCH_MAX_DEG IS 20.0. // deg
// Approach phase pitch cap.
// With AA_DIR_ADD_AOA_COMP = FALSE, cap fpa_cmd <= MAX_APP_PITCH_CMD.
// With AA_DIR_ADD_AOA_COMP = TRUE, cap fpa_cmd <= MAX_APP_PITCH_CMD - AOA.
GLOBAL MAX_APP_PITCH_CMD   IS  6.0.  // deg  max pitch AA Director may be commanded during approach
GLOBAL APP_FPA_PITCH_FLOOR IS -15.0. // deg  absolute fpa_cmd floor inside pitch cap (allows fpa_cmd < MAX_DESC_FPA when AOA is high)

// ----------------------------
// Flap detent stepping
// ----------------------------
GLOBAL FLAP_STEP_INTERVAL IS 0.35. // s minimum time between flap detent steps
GLOBAL FLAP_VFE_HYST      IS 2.0.  // m/s hold margin before forced retract

// ----------------------------
// Gear deployment safety
// ----------------------------
GLOBAL GEAR_MAX_EXTEND_IAS IS 120. // m/s max IAS allowed for automatic gear extension

// ----------------------------
// Flare
// ----------------------------
GLOBAL FLARE_MAIN_GEAR_TAG_DEFAULT IS "ifc_maingear". // base part tag for main-gear flare/touchdown sensors; discovery checks base + "_L" + "_R"
GLOBAL FLARE_CTRL_H_OFFSET_MAX_M IS 30.0. // m max allowed body-vs-gear height offset captured at flare entry for control shaping
GLOBAL FLARE_AGL_M      IS 25.   // m runway-relative height to trigger flare
GLOBAL FLARE_TRIGGER_HYST_M IS 1.0. // m hysteresis for flare trigger re-arm
GLOBAL FLARE_TRIGGER_CONFIRM_S IS 0.12. // s runway-relative height must remain below trigger to enter flare
GLOBAL FLARE_RATE_LOW_IAS   IS 40.  // m/s low-speed end of flare-rate schedule
GLOBAL FLARE_RATE_HIGH_IAS  IS 95.  // m/s high-speed end of flare-rate schedule
GLOBAL TOUCHDOWN_VS     IS -0.3. // m/s target sink rate at wheel contact
GLOBAL FLARE_MIN_ENTRY_SINK_VS IS -6.0. // m/s deepest sink accepted to seed flare profile (entries worse than this are clamped)
GLOBAL FLARE_MAX_SINK_VS IS -2.5. // m/s strongest sink command allowed in flare
GLOBAL FLARE_MODE_CAPTURE IS "FLARE_CAPTURE". // initial flare-capture shaping period
GLOBAL FLARE_MODE_TRACK IS "FLARE_TRACK". // nominal tracking of sink profile
GLOBAL FLARE_MODE_ROUNDOUT IS "ROUNDOUT". // late flare sink arrest near touchdown
GLOBAL FLARE_MODE_TOUCHDOWN_CONFIRM IS "TOUCHDOWN_CONFIRM". // fallback touchdown debounce window
GLOBAL FLARE_CAPTURE_MIN_S IS 0.50. // s minimum time to remain in flare-capture before normal track mode
GLOBAL FLARE_ROUNDOUT_HYST_M IS 1.0. // m hysteresis applied at roundout-start threshold
GLOBAL FLARE_VS_KP IS 0.30. // deg FPA correction per m/s sink-rate error
GLOBAL FLARE_FPA_KP IS 0.35. // deg FPA correction per deg FPA error
GLOBAL FLARE_CMD_FPA_MIN IS -6.0. // deg lower clamp for flare gamma command
GLOBAL FLARE_CMD_FPA_MAX IS 4.0. // deg upper clamp for flare gamma command
GLOBAL FLARE_CMD_RATE_MIN_DPS IS 0.8. // deg/s low-speed flare gamma slew
GLOBAL FLARE_CMD_RATE_MAX_DPS IS 2.2. // deg/s high-speed flare gamma slew
GLOBAL FLARE_ROUNDOUT_START_H_M IS 8.0. // m runway-relative main-gear height where roundout shaping begins
GLOBAL FLARE_ROUNDOUT_END_H_M IS 0.8. // m runway-relative main-gear height where roundout shaping reaches full effect
GLOBAL FLARE_ROUNDOUT_CURVE IS 1.0. // 0 disables roundout; >0 scales roundout blend strength
GLOBAL FLARE_ROUNDOUT_TTG_START_S IS 3.0. // s time-to-ground where roundout blending starts (speed/sink adaptive path)
GLOBAL FLARE_ROUNDOUT_TTG_END_S IS 0.8. // s time-to-ground where roundout blending reaches full effect
GLOBAL FLARE_DISABLE_SPEED_BLEED_DEFAULT IS TRUE. // TRUE disables speed-driven extra sink during flare by default
GLOBAL FLARE_SPEED_BLEED_GAIN IS 0.02. // extra sink per m/s above Vref when speed bleed is enabled
GLOBAL FLARE_MIN_THROTTLE IS 0.0. // minimum throttle floor while in flare (before blenddown/recovery)
GLOBAL FLARE_MIN_THROTTLE_AGL_BLEND IS 8.0. // m runway-relative height below which flare throttle floor blends down
GLOBAL FLARE_AUTH_VS_ERR_TRIGGER IS 0.8. // m/s minimum positive vs_err for authority-limited detection
GLOBAL FLARE_AUTH_PITCH_ERR_TRIGGER IS 2.0. // deg minimum pitch tracking error for authority-limited detection
GLOBAL FLARE_AUTH_FPA_ERR_TRIGGER IS 1.0. // deg minimum FPA tracking error for authority-limited detection
GLOBAL FLARE_AUTH_DETECT_S IS 0.35. // s detection persistence before authority-limited state latches
GLOBAL FLARE_AUTH_RECOVERY_GAIN IS 0.20. // additive throttle-floor and roundout-advance gain in authority recovery
// TECS-style flare control (coupled pitch/throttle handling):
// - throttle controls total energy error (Et)
// - pitch/gamma controls energy-balance error (Eb)
GLOBAL FLARE_TECS_ET_KP IS 0.00008. // throttle per (m^2/s^2) total-energy error
GLOBAL FLARE_TECS_ET_KI IS 0.00002. // throttle/(m^2/s^2*s) total-energy integral gain
// EB gains are V*g-normalized: output = (eb_kp * Eb_err + ...) / (ias * g)
// At 80 m/s these are equivalent to the old 0.00075 / 0.00008 deg/(m^2/s^2) values.
// Per-aircraft overrides must also use the normalized scale.
GLOBAL FLARE_TECS_EB_KP IS 0.59.    // V*g-normalized pitch gain on energy-balance error
GLOBAL FLARE_TECS_EB_KI IS 0.063.   // V*g-normalized pitch integral gain (per s)
GLOBAL FLARE_TECS_ET_INT_LIM IS 8000. // integral clamp for Et loop
GLOBAL FLARE_TECS_EB_INT_LIM IS 5000. // integral clamp for Eb loop
GLOBAL FLARE_TECS_THR_TRIM IS 0.18. // nominal flare throttle trim
GLOBAL FLARE_TECS_THR_BAL_K IS 0.00008. // throttle bias from energy-balance error (reduces thrust when high on path)
GLOBAL FLARE_TECS_THR_SLEW_PER_S IS 1.2. // /s max flare throttle command slew
GLOBAL FLARE_TECS_CLIMB_VS_GATE IS 0.2. // m/s if VS exceeds this in flare, throttle forced to floor
GLOBAL FLARE_TECS_EDOT_ALPHA IS 0.30. // EMA smoothing factor for IAS derivative filter (0=max smooth, 1=raw)
GLOBAL FLARE_TECS_ET_KD IS 0.0.       // throttle damping on total-energy rate error (0 = disabled until tuned)
GLOBAL FLARE_TECS_EB_KD IS 0.0.       // pitch damping on energy-balance rate error (0 = disabled until tuned)
GLOBAL FLARE_BALLOON_VS_TRIGGER IS 0.2. // m/s upward VS that latches anti-balloon supervision above touchdown region
GLOBAL FLARE_BALLOON_CLEAR_VS IS -0.2. // m/s VS threshold that clears anti-balloon latch once descending again
GLOBAL FLARE_BALLOON_MIN_H_M IS 3.0. // m control-height floor below which anti-balloon latch is no longer enforced
GLOBAL FLARE_BALLOON_GAMMA_DOWN_DEG IS -3.0. // deg minimum nose-down gamma command applied while anti-balloon latch is active


// ----------------------------
// Touchdown / rollout
// ----------------------------
GLOBAL TOUCHDOWN_AGL_M  IS 2.    // m AGL considered touchdown (radar alt)
GLOBAL TOUCHDOWN_CONFIRM_S IS 0.12. // s touchdown conditions must persist before switching phases
GLOBAL TOUCHDOWN_CONFIRM_MAX_ABS_VS IS 2.5. // m/s max |VS| allowed when committing FLARE->TOUCHDOWN
GLOBAL TOUCHDOWN_FALLBACK_AGL_M IS 0.8. // m runway-relative fallback detector if LANDED status lags
GLOBAL TOUCHDOWN_FALLBACK_MAX_VS IS 0.2. // m/s max |VS| for fallback detector
GLOBAL TOUCHDOWN_FALLBACK_MAX_AGL_M IS 8.0. // m absolute AGL guard for fallback detector
GLOBAL TOUCHDOWN_SETTLE_S IS 0.20. // s hold TOUCHDOWN phase to let gear loads settle
GLOBAL TOUCHDOWN_NOSE_HOLD_S IS 0.15. // s hold captured touchdown pitch before starting nose-lowering profile
GLOBAL TOUCHDOWN_NOSE_LOWER_RATE_DPS IS 2.0. // deg/s max commanded nose-lowering pitch-target rate in TOUCHDOWN
GLOBAL BOUNCE_RECOVERY_AGL_M IS 2.5. // m if airborne above this in touchdown/rollout, consider bounce recovery
GLOBAL BOUNCE_RECOVERY_MIN_VS IS 0.6. // m/s minimum upward VS to count as a real bounce
GLOBAL BOUNCE_RECOVERY_CONFIRM_S IS 0.30. // s airborne criteria must persist before bounce recovery
GLOBAL BOUNCE_RECOVERY_MAX_S IS 6.0. // s after touchdown where bounce recovery is allowed
GLOBAL ROLLOUT_DONE_IAS IS 3.    // m/s IAS to declare rollout complete
GLOBAL THR_REV_DEACT_IAS IS 5.  // m/s IAS below which thrust reversers are cut and throttle zeroed
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
GLOBAL TAKEOFF_ROTATE_PITCH_MIN_DOWN_CMD IS -0.10. // max nose-down pitch cmd allowed during on-ground rotate
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
// Cruise phase defaults
// ----------------------------
GLOBAL CRUISE_SPD_MODE_IAS   IS "IAS". // speed target interpreted as indicated airspeed
GLOBAL CRUISE_SPD_MODE_MACH  IS "MACH". // speed target interpreted as Mach number
GLOBAL CRUISE_DEFAULT_SPD     IS 150.    // m/s  default cruise IAS
GLOBAL CRUISE_DEFAULT_MACH    IS 0.70.   // Mach default cruise Mach target when Mach mode is selected
GLOBAL CRUISE_DEFAULT_ALT_M   IS 3000.   // m    default cruise altitude MSL
GLOBAL CRUISE_DESCENT_START_M IS 15000.  // m    begin blending toward waypoint alt within this range
GLOBAL CRUISE_VNAV_DESCENT_FPA IS 3.0.  // deg  constant-FPA angle for VNAV top-of-descent profile
GLOBAL FMS_WPT_SLOTS          IS 3.      // cruise waypoint picker slots in plan editor

// ----------------------------
// Flight plan leg type tags
// ----------------------------
GLOBAL LEG_TAKEOFF  IS "TAKEOFF".
GLOBAL LEG_CRUISE   IS "CRUISE".
GLOBAL LEG_APPROACH IS "APPROACH".
GLOBAL LEG_ASCENT   IS "ASCENT".
GLOBAL LEG_REENTRY  IS "REENTRY".

// ----------------------------
// Beacon type tags
// ----------------------------
GLOBAL BTYPE_ILS  IS "ILS".
GLOBAL BTYPE_VOR  IS "VOR".
GLOBAL BTYPE_IAF  IS "IAF".  // Initial Approach Fix
GLOBAL BTYPE_FAF  IS "FAF".  // Final Approach Fix
GLOBAL BTYPE_WPT  IS "WPT".  // User-defined waypoint

// ----------------------------
// UI layout  (row numbers for AT(col,row) positioning)
// Separators drawn at rows: 0, 2, 4, 11, 15, 17, 19, 21
// ----------------------------
GLOBAL UI_HDR_ROW    IS 1.   // aircraft + timer header
GLOBAL UI_CRUMB_ROW  IS 3.   // phase breadcrumb
GLOBAL UI_PRI_TOP    IS 5.   // primary data zone, first row
GLOBAL UI_PRI_BOT    IS 10.  // primary data zone, last row
GLOBAL UI_SEC_TOP    IS 12.  // secondary/debug zone, first row
GLOBAL UI_SEC_BOT    IS 14.  // secondary/debug zone, last row
GLOBAL UI_ALERT_ROW  IS 16.  // alert / event bar
GLOBAL UI_LOG_ROW    IS 18.  // logger status bar
GLOBAL UI_HINT_ROW   IS 20.  // key hint bar

// ----------------------------
// Display update rates
// ----------------------------
GLOBAL IFC_DISPLAY_PERIOD   IS 0.10. // s  primary zone (10 Hz)
GLOBAL IFC_HEADER_PERIOD    IS 0.20. // s  header + breadcrumb (5 Hz)
GLOBAL IFC_SECONDARY_PERIOD IS 0.20. // s  debug panel (5 Hz)
GLOBAL IFC_LOGGER_PERIOD    IS 0.50. // s  logger bar (2 Hz)
GLOBAL IFC_CSV_LOG_PERIOD   IS 0.25. // s  CSV telemetry write period (4 Hz)
GLOBAL IFC_LOG_SAMPLE_CTRL_SURF IS FALSE. // FALSE = skip expensive reflection-based control-surface deflection scan
                                      // WARNING: kOS LOG..TO blocks the main loop for the
                                      // duration of the file I/O. On Windows with AV this
                                      // can be 100-500 ms per call — reduce rate or disable
                                      // logging if autothrottle lag is observed.
GLOBAL IFC_ALERT_EXPIRE_S   IS 5.0.  // s  auto-clear alert after this long

// ----------------------------
// UI interaction modes
// ----------------------------
GLOBAL UI_MODE_PREARM         IS "MODE_PREARM".
GLOBAL UI_MODE_PREARM_AMO     IS "MODE_PREARM_AMO".
GLOBAL UI_MODE_AUTOFLOW       IS "MODE_AUTOFLOW".
GLOBAL UI_MODE_MENU_OVERLAY   IS "MODE_MENU_OVERLAY".
GLOBAL UI_MODE_MANUAL_OVERRIDE IS "MODE_MANUAL_OVERRIDE".
GLOBAL UI_MODE_COMPLETE       IS "MODE_COMPLETE".

// ----------------------------
// Augmented Manual Operation (AMO) ground steering assist
// ----------------------------
GLOBAL AMO_ENABLED_DEFAULT        IS TRUE. // default enable flag when aircraft config omits amo_enabled
GLOBAL AMO_STEER_DEADBAND         IS 0.08. // 0..1 minimum steering input before diff assist engages
GLOBAL AMO_BRAKE_MAX_IAS          IS 35.0. // m/s apply brake assist only at/below this IAS
GLOBAL AMO_ENGINE_SIDE_EPS_M      IS 0.25. // m engine lateral offset deadzone when auto-splitting left/right banks
GLOBAL AMO_DIFF_BRAKE_STRENGTH    IS 1.00. // 0..1 nominal per-side brake strength used by AMO differential braking
GLOBAL ABRK_DEFAULT_STRENGTH      IS 0.70. // 0..1 default per-wheel brake strength setting when differential braking is idle

// ----------------------------
// AMO VTOL assist
// ----------------------------
GLOBAL VTOL_VS_KP            IS 0.30.  // collective per (m/s) VS error
GLOBAL VTOL_VS_KI            IS 0.04.  // collective per (m/s·s) VS error integral
GLOBAL VTOL_VS_INTEGRAL_LIM  IS 0.40.  // anti-windup clamp on VS integral
GLOBAL VTOL_VS_CMD_UP_SLEW_MPS2 IS 1.2. // m/s^2 max upward VS-command slew
GLOBAL VTOL_VS_CMD_DN_SLEW_MPS2 IS 1.8. // m/s^2 max downward VS-command slew
GLOBAL VTOL_VS_CMD_LAG_REF_S IS 0.8. // s spool-lag reference for VS-command slew scheduling
GLOBAL VTOL_VS_CMD_SLEW_MIN_SCALE IS 0.35. // minimum scale applied to VS-command slew under high lag
GLOBAL VTOL_VS_GAIN_LAG_REF_S IS 0.8. // s spool-lag reference for VS PI gain scheduling
GLOBAL VTOL_VS_KP_MIN_SCALE IS 0.45. // lower bound on VS proportional gain scale under high lag
GLOBAL VTOL_VS_KI_MIN_SCALE IS 0.25. // lower bound on VS integral gain scale under high lag
GLOBAL VTOL_VS_AW_ALPHA_MIN IS 0.95. // freeze/unwind VS integrator when prior allocator alpha is below this
GLOBAL VTOL_VS_AW_LAG_S IS 0.6. // freeze/unwind VS integrator when model spool lag exceeds this
GLOBAL VTOL_VS_AW_EFF_ERR_MIN IS 0.05. // minimum |cmd-achieved collective| error to treat lag as actuator-limited
GLOBAL VTOL_VS_I_UNWIND_PER_S IS 1.8. // /s VS-integrator bleed rate while actuator-limited/saturated
GLOBAL VTOL_ALT_KP           IS 0.40.  // (m/s) VS command per metre altitude error
GLOBAL VTOL_MAX_VS           IS 8.0.   // m/s  max commanded vertical speed
GLOBAL VTOL_COLLECTIVE_MAX   IS 0.90.  // reserve headroom for differential attitude authority
GLOBAL VTOL_COLLECTIVE_UP_SLEW_PER_S IS 0.35. // /s max collective rise (spool-friendly)
GLOBAL VTOL_COLLECTIVE_DN_SLEW_PER_S IS 2.50. // /s max collective fall (fast authority recovery)
GLOBAL VTOL_YAW_SRV_GAIN     IS 8.0.   // deg  differential servo tilt per unit yaw input
GLOBAL VTOL_ROLL_GAIN        IS 0.25.  // fraction of collective used as roll authority
GLOBAL VTOL_PITCH_GAIN       IS 0.25.  // fraction of collective used as pitch authority
GLOBAL VTOL_PITCH_MIX_SIGN   IS 1.0.   // pitch mix polarity; +1 means forward engines increase on nose-up correction (correct for fwd engines ahead of CoM)
GLOBAL VTOL_HOVER_LEARN_RATE IS 0.001. // rate at which hover_collective tracks actual collective
GLOBAL VTOL_EM_FF_ENABLED_DEFAULT IS TRUE. // TRUE = apply engine-model feed-forward in VTOL VS-hold
GLOBAL VTOL_EM_FF_GAIN IS 0.75. // 0..1 blend from PI target toward model-inverse feed-forward target
GLOBAL VTOL_EM_FF_MAX_LEAD IS 0.20. // max collective lead (+/-) applied by feed-forward before slew/clamp
GLOBAL VTOL_EM_FF_LAG_MIN_S IS 0.10. // ignore feed-forward when modelled lag is below this (near-instant response)
GLOBAL VTOL_EM_FF_ALPHA_MIN IS 0.12. // lower bound on model response fraction used by inverse step
GLOBAL VTOL_BYPASS_ATTITUDE_FEEDBACK_DEFAULT IS FALSE. // TRUE = bypass VTOL level/upset attitude feedback (FF-only test mode)
GLOBAL VTOL_MAX_PODS         IS 8.     // max pod tags scanned during discovery
GLOBAL VTOL_SRV_SPEED        IS 1.0.   // servo speed multiplier for baseline hover commands
GLOBAL VTOL_SRV_YAW_SPEED    IS 2.0.   // servo speed multiplier for yaw differential commands
GLOBAL VTOL_LAT_SIGN_THRESH  IS 0.15.  // m  lateral offset below which a pod is considered centerline
GLOBAL VTOL_TRIM_RATE        IS 0.005. // limit change per degree of pitch error per second
GLOBAL VTOL_TRIM_DEADBAND    IS 0.05.  // pilot input magnitude below which auto systems engage
GLOBAL VTOL_TRIM_PITCH_CLAMP IS 15.0.  // max pitch error (deg) considered by trim integrator
GLOBAL VTOL_TRIM_RATE_LEAD_S IS 0.25.  // s of angular-rate lead added into trim error
GLOBAL VTOL_TRIM_ACTIVITY_MIN IS 0.35. // minimum adaptive-trim activity scale during large excursions
GLOBAL VTOL_LEVEL_ROLL_KP   IS 0.10.  // roll authority fraction per degree of bank (full at 10°)
GLOBAL VTOL_LEVEL_ROLL_KD   IS 0.03.  // roll authority per deg/s roll rate (damp oscillation)
GLOBAL VTOL_LEVEL_ROLL_KI   IS 0.010. // roll integral authority per deg*s bank error (steady bias removal)
GLOBAL VTOL_LEVEL_PITCH_KP  IS 0.40.  // pitch authority fraction per degree of pitch (full at 2.5°)
GLOBAL VTOL_LEVEL_PITCH_KD  IS 0.03.  // pitch authority per deg/s pitch rate (damp divergence)
GLOBAL VTOL_LEVEL_PITCH_KI  IS 0.015. // pitch integral authority per deg*s pitch error (steady bias removal)
GLOBAL VTOL_LEVEL_ROLL_ATT2RATE_KP IS 2.5. // (deg/s) target roll-rate per degree bank error
GLOBAL VTOL_LEVEL_PITCH_ATT2RATE_KP IS 3.0. // (deg/s) target pitch-rate per degree pitch error magnitude
GLOBAL VTOL_LEVEL_ROLL_ATT2RATE_KI IS 0.10. // (deg/s) per (deg*s) roll-angle integral contribution
GLOBAL VTOL_LEVEL_PITCH_ATT2RATE_KI IS 0.10. // (deg/s) per (deg*s) pitch-angle integral contribution
GLOBAL VTOL_LEVEL_ROLL_RATE_KP IS 0.05. // command per (deg/s) roll-rate error
GLOBAL VTOL_LEVEL_PITCH_RATE_KP IS 0.05. // command per (deg/s) pitch-rate error
GLOBAL VTOL_LEVEL_ROLL_RATE_CMD_MAX_DEGS IS 20.0. // max target roll-rate magnitude from outer attitude loop
GLOBAL VTOL_LEVEL_PITCH_RATE_CMD_MAX_DEGS IS 24.0. // max target pitch-rate magnitude from outer attitude loop
GLOBAL VTOL_LEVEL_I_LIM     IS 40.0.  // deg*s clamp for level-hold integrators
GLOBAL VTOL_LEVEL_GAIN_LAG_REF_S IS 0.8. // s spool-lag reference for attitude gain scheduling
GLOBAL VTOL_LAG_FILTER_TAU_S IS 0.45. // s low-pass filter time constant for spool-lag scheduling signals
GLOBAL VTOL_LEVEL_KP_MIN_SCALE IS 0.45. // lower bound on attitude proportional gain scale under high lag
GLOBAL VTOL_LEVEL_KD_MIN_SCALE IS 0.45. // lower bound on attitude damping gain scale under high lag
GLOBAL VTOL_LEVEL_KI_MIN_SCALE IS 0.20. // lower bound on attitude integral gain scale under high lag
GLOBAL VTOL_LEVEL_AW_ALPHA_MIN IS 0.98. // freeze/unwind attitude integrators when last allocator alpha drops below this
GLOBAL VTOL_LEVEL_AW_LAG_S IS 0.6. // freeze/unwind attitude integrators when model spool lag exceeds this
GLOBAL VTOL_LEVEL_AW_EFF_ERR_MIN IS 0.05. // minimum |cmd-achieved collective| error to treat lag as actuator-limited
GLOBAL VTOL_LEVEL_I_UNWIND_PER_S IS 2.0. // /s integrator bleed rate while allocator is authority-limited
GLOBAL VTOL_LEVEL_CMD_SLEW_PER_S IS 2.0. // /s max attitude feedback command slew before engine allocation
GLOBAL VTOL_LEVEL_CMD_SLEW_LAG_REF_S IS 0.8. // s spool-lag reference for feedback-command slew scheduling
GLOBAL VTOL_LEVEL_CMD_SLEW_MIN_SCALE IS 0.35. // minimum scale for feedback-command slew under high lag
GLOBAL VTOL_LEVEL_ON_GROUND_DEFAULT IS FALSE. // TRUE = allow auto-level while LANDED/PRELAUNCH
GLOBAL VTOL_LEVEL_MIN_AGL_M IS 0.8. // min AGL for auto-level when ground-leveling is disabled
GLOBAL VTOL_GROUND_CONTACT_AGL_M IS 1.5. // treat as ground-contact only below this AGL
GLOBAL VTOL_GROUND_CONTACT_VS_MAX IS 0.7. // m/s; must also be below this VS to count as ground-contact
GLOBAL VTOL_STATIC_TRIM_DISCOVERY_DEFAULT IS TRUE. // TRUE = enable geometry-based static trim at discovery
GLOBAL VTOL_STATIC_TRIM_BASE_MIN IS 0.23. // discovery trim base lower clamp; computed ~0.23 for forward engines ahead of CoM
GLOBAL VTOL_DIFF_COLLECTIVE_MIN IS 0.12. // no roll/pitch differential below this collective
GLOBAL VTOL_TRIM_MIN_AGL_M IS 1.5. // adaptive trim runs only above this AGL
GLOBAL VTOL_TRIM_MIN_OFFSET IS -0.85. // lower clamp for adaptive trim offset
GLOBAL VTOL_TRIM_MAX_OFFSET IS 0.0. // upper clamp for adaptive trim offset
GLOBAL VTOL_TRIM_ACTIVE_PITCH_MAX IS 6.0. // adaptive trim only while |pitch| is below this
GLOBAL VTOL_TRIM_ACTIVE_RATE_MAX IS 8.0. // adaptive trim only while |pitch_rate| is below this
GLOBAL VTOL_TRIM_ROLL_RATE IS 0.0015. // limit change per degree of bank error per second
GLOBAL VTOL_TRIM_BANK_CLAMP IS 12.0. // max bank error (deg) considered by trim integrator
GLOBAL VTOL_TRIM_ACTIVE_BANK_MAX IS 8.0. // adaptive trim only while |bank| is below this
GLOBAL VTOL_TRIM_ACTIVE_ROLL_RATE_MAX IS 12.0. // adaptive trim only while |roll_rate| is below this
GLOBAL VTOL_ENGINE_LIMIT_FLOOR IS 0.10. // minimum per-engine limiter fraction while differential is active
GLOBAL VTOL_CMD_SLEW_PER_S IS 5.0. // max change per second for roll/pitch command before engine allocation
GLOBAL VTOL_DIFF_STATE_ATTEN_ENABLED_DEFAULT IS FALSE. // FALSE = do not reduce differential authority from attitude/rate excursions
GLOBAL VTOL_DIFF_ATTEN_MIN IS 0.10. // lower bound on differential authority attenuation factor
GLOBAL VTOL_DIFF_SOFT_BANK_DEG IS 8.0. // begin reducing differential authority above this bank angle
GLOBAL VTOL_DIFF_HARD_BANK_DEG IS 35.0. // fully reduced differential authority at/above this bank angle
GLOBAL VTOL_DIFF_SOFT_PITCH_DEG IS 8.0. // begin reducing differential authority above this pitch angle
GLOBAL VTOL_DIFF_HARD_PITCH_DEG IS 25.0. // fully reduced differential authority at/above this pitch angle
GLOBAL VTOL_DIFF_SOFT_ROLL_RATE_DEGS IS 10.0. // begin reducing differential authority above this roll rate
GLOBAL VTOL_DIFF_HARD_ROLL_RATE_DEGS IS 30.0. // fully reduced differential authority at/above this roll rate
GLOBAL VTOL_DIFF_SOFT_PITCH_RATE_DEGS IS 10.0. // begin reducing differential authority above this pitch rate
GLOBAL VTOL_DIFF_HARD_PITCH_RATE_DEGS IS 30.0. // fully reduced differential authority at/above this pitch rate
GLOBAL VTOL_CMD_ROLL_MAX IS 0.65. // hard cap on roll command into differential mixer
GLOBAL VTOL_CMD_PITCH_MAX IS 0.65. // hard cap on pitch command into differential mixer
GLOBAL VTOL_UPSET_BANK_DEG IS 18.0. // upset mode activates above this absolute bank
GLOBAL VTOL_UPSET_PITCH_DEG IS 14.0. // upset mode activates above this absolute pitch
GLOBAL VTOL_UPSET_ROLL_RATE_DEGS IS 18.0. // upset mode activates above this absolute roll rate
GLOBAL VTOL_UPSET_PITCH_RATE_DEGS IS 18.0. // upset mode activates above this absolute pitch rate
GLOBAL VTOL_UPSET_EXIT_BANK_DEG IS 12.0. // upset exits only when |bank| is below this (hysteresis)
GLOBAL VTOL_UPSET_EXIT_PITCH_DEG IS 9.0. // upset exits only when |pitch| is below this (hysteresis)
GLOBAL VTOL_UPSET_EXIT_ROLL_RATE_DEGS IS 10.0. // upset exits only when |roll-rate| is below this (hysteresis)
GLOBAL VTOL_UPSET_EXIT_PITCH_RATE_DEGS IS 10.0. // upset exits only when |pitch-rate| is below this (hysteresis)
GLOBAL VTOL_UPSET_HOLD_S IS 1.5. // minimum time upset stays latched once entered
GLOBAL VTOL_UPSET_CMD_MAX IS 0.35. // max |roll/pitch command| allowed while upset mode is active
GLOBAL VTOL_UPSET_CMD_MAX_ROLL IS 0.70. // upset-mode roll command cap (higher to preserve recovery authority)
GLOBAL VTOL_UPSET_CMD_MAX_PITCH IS 0.35. // upset-mode pitch command cap
GLOBAL VTOL_UPSET_ROLL_RATE_KP IS 0.04. // upset-mode roll-rate damping gain (command per deg/s)
GLOBAL VTOL_UPSET_PITCH_RATE_KP IS 0.05. // upset-mode pitch-rate damping gain (command per deg/s)
GLOBAL VTOL_UPSET_PITCH_SLEW_BYPASS IS TRUE. // TRUE = bypass pitch command slew while upset recovery is active
GLOBAL VTOL_UPSET_DIFF_ATTEN_MIN IS 0.55. // minimum differential authority scale while upset is active
GLOBAL VTOL_UPSET_ENGINE_LIMIT_FLOOR IS 0.02. // per-engine limiter floor while upset is active
GLOBAL VTOL_UPSET_GUARD_AGL_M IS 20.0. // below this AGL, upset + low pilot throttle is clamped to preserve lift
GLOBAL VTOL_UPSET_GUARD_THR_MIN IS 0.55. // minimum pilot throttle surrogate when low-alt upset guard is active
GLOBAL VTOL_RATE_KD_ROLL_ACCEL IS 0.02. // roll D-term gain on filtered roll acceleration
GLOBAL VTOL_RATE_KD_PITCH_ACCEL IS 0.02. // pitch D-term gain on filtered pitch acceleration
GLOBAL VTOL_RATE_P_ALPHA IS 0.70. // EMA alpha for roll-rate filtering
GLOBAL VTOL_RATE_Q_ALPHA IS 0.70. // EMA alpha for pitch-rate filtering
GLOBAL VTOL_RATE_PDOT_ALPHA IS 0.35. // EMA alpha for roll-acceleration filtering
GLOBAL VTOL_RATE_QDOT_ALPHA IS 0.35. // EMA alpha for pitch-acceleration filtering
GLOBAL VTOL_RATE_ACCEL_CLAMP_DEGS2 IS 300.0. // max absolute filtered acceleration for D-term stability
GLOBAL VTOL_COS_ATT_ALPHA IS 0.10. // EMA alpha for attitude cosine correction in collective loop
GLOBAL VTOL_COS_ATT_FLOOR IS 0.25. // lower bound on cos(attitude) correction factor
GLOBAL VTOL_VEL_KP IS 0.15. // horiz accel cmd per horizontal velocity error (m/s^2 per m/s)
GLOBAL VTOL_VEL_KI IS 0.005. // horiz accel cmd per integral velocity error
GLOBAL VTOL_VEL_INT_LIM IS 2.0. // clamp for velocity integrators
GLOBAL VTOL_VEL_INT_DEADBAND IS 0.5. // do not integrate horizontal velocity error inside this deadband
GLOBAL VTOL_MAX_HORIZ_ACCEL IS 1.5. // max commanded horizontal acceleration
GLOBAL VTOL_MAX_HORIZ_SPEED IS 8.0. // max commanded horizontal speed
GLOBAL VTOL_MAX_FWD_PITCH IS 15.0. // max forward pitch command magnitude from velocity loop
GLOBAL VTOL_MAX_BANK IS 15.0. // max bank command magnitude from velocity loop
GLOBAL VTOL_POS_KP IS 0.08. // position error to velocity command proportional gain
GLOBAL VTOL_POS_KI IS 0.002. // position integrator gain
GLOBAL VTOL_POS_INT_LIM IS 3.0. // clamp for position integrators
GLOBAL VTOL_POS_INT_RADIUS IS 50.0. // only integrate position error inside this radius (m)
GLOBAL VTOL_POS_CAPTURE_RADIUS IS 10.0. // decel schedule radius for position hold (m)
GLOBAL VTOL_KHV_CAPTURE_MPS IS 0.5. // kill-horizontal-velocity completion threshold
GLOBAL VTOL_PHYSICAL_ALLOC_ENABLED_DEFAULT IS FALSE. // TRUE enables arm-based physical allocation path
GLOBAL VTOL_TRANS_START_IAS IS 30.0. // IAS where nacelle transition begins (m/s)
GLOBAL VTOL_TRANS_END_IAS IS 80.0. // IAS where nacelle transition reaches cruise angle (m/s)
GLOBAL VTOL_NACELLE_SLEW_DPS IS 5.0. // max collective nacelle-angle slew rate (deg/s)
GLOBAL VTOL_NACELLE_ALPHA_MIN IS 10.0. // lower bound for nacelle angle during transition
GLOBAL VTOL_NACELLE_SIN_FLOOR IS 0.10. // min sin(alpha) used for vertical-thrust compensation

// ----------------------------
// UI event queue
// ----------------------------
GLOBAL IFC_EVENT_MAX            IS 16. // max retained alert/event entries
GLOBAL IFC_EVENT_HISTORY_ROWS   IS 6.  // history rows shown in menu overlay
GLOBAL IFC_MENU_MAX_CHARS_TICK  IS 64. // max queued keypresses consumed per MENU_TICK

// ============================================================
// ASCENT GUIDANCE
// ============================================================

// ----------------------------
// Ascent sub-phase names
// ----------------------------
GLOBAL SUBPHASE_ASC_AB_CORRIDOR    IS "AB_CORRIDOR".
GLOBAL SUBPHASE_ASC_AB_ZOOM        IS "AB_ZOOM".
GLOBAL SUBPHASE_ASC_ROCKET_SUSTAIN IS "RKT_SUSTAIN".
GLOBAL SUBPHASE_ASC_CLOSEOUT       IS "RKT_CLOSEOUT".
GLOBAL SUBPHASE_ASC_CIRCULARISE    IS "CIRCULARISE".

// ----------------------------
// Estimator validity states
// ----------------------------
GLOBAL ASC_VALID   IS "VALID".
GLOBAL ASC_DEGRADED IS "DEGRADED".
GLOBAL ASC_INVALID  IS "INVALID".

// ----------------------------
// Kerbin physical constants
// ----------------------------
GLOBAL KERBIN_MU IS 3.5316e12.  // m^3/s^2  gravitational parameter
GLOBAL KERBIN_R  IS 600000.0.   // m        equatorial radius
GLOBAL KERBIN_ATMO_TOP IS 70000.0. // m     top of atmosphere

// ----------------------------
// Signal smoothing alphas
// (higher = faster response, less smoothing)
// Valuation signals: slow/stable  Control signals: fast/responsive
// ----------------------------
GLOBAL ASC_EMA_CTRL_AERO  IS 0.40. // along-track aero force (control copy)
GLOBAL ASC_EMA_CTRL_Q     IS 0.35. // dynamic pressure (control copy)
GLOBAL ASC_EMA_CTRL_AOA   IS 0.40. // angle of attack (control copy)

GLOBAL ASC_EMA_VAL_AERO   IS 0.15. // along-track aero force (valuation copy)
GLOBAL ASC_EMA_VAL_EDOT   IS 0.15. // orbital energy rate (valuation copy)
GLOBAL ASC_EMA_VAL_APO    IS 0.20. // apoapsis altitude (valuation copy)
GLOBAL ASC_EMA_VAL_MDOT   IS 0.20. // mass flow, both modes (valuation copy)
GLOBAL ASC_EMA_VAL_Q      IS 0.12. // dynamic pressure (valuation copy — slower than control)

GLOBAL ASC_EMA_DRAG_RK    IS 0.10. // drag for J_rk — dedicated heavy filter
GLOBAL ASC_EMA_EDOT_ORB   IS 0.15. // orbital-state Ė cross-check (same band as valuation)

// ----------------------------
// Dynamic pressure corridor defaults
// (override per-aircraft via ascent_q_target / ascent_q_max / ascent_q_min)
// ----------------------------
// FAR DYNPRES on this stack reports kPa; convert to Pa for corridor logic.
// Set to 1.0 only if your FAR build already reports Pa.
GLOBAL ASC_FAR_Q_SCALE      IS 1000.  // multiply FAR DYNPRES by this to get Pa
GLOBAL ASC_Q_TARGET_DEFAULT IS 30000. // Pa  corridor centre
GLOBAL ASC_Q_MAX_DEFAULT    IS 60000. // Pa  structural limit
GLOBAL ASC_Q_MIN_DEFAULT    IS 8000.  // Pa  lower corridor bound
GLOBAL ASC_Q_REF            IS 30000. // Pa  gain-scheduling reference point

// ----------------------------
// Pitch corridor control
// ----------------------------
GLOBAL ASC_Q_KP             IS 2.5e-4.  // deg/Pa  q-error to pitch-bias gain
GLOBAL ASC_APO_RATE_KP      IS 5.0e-4.  // deg/(m/s)  apoapsis growth rate bias
GLOBAL ASC_APO_RATE_TGT     IS 60.0.    // m/s  desired apoapsis growth rate
GLOBAL ASC_PITCH_BIAS_MAX   IS 20.0.    // deg  max pitch bias above prograde
GLOBAL ASC_PITCH_BIAS_MIN   IS -2.0.    // deg  max pitch bias below prograde
GLOBAL ASC_PITCH_SLEW_DPS   IS 1.0.     // deg/s  max bias slew rate
GLOBAL ASC_VS_KP            IS 0.008.   // deg/(m/s)  vertical-speed feedforward gain
                                         // adds -VS*KP nose-down bias when climbing fast;
                                         // preempts altitude overshoot before q falls below target.
                                         // Tapered to zero by ASC_VS_FF_ALT_CUTOFF — inactive at altitude.
GLOBAL ASC_VS_FF_ALT_CUTOFF IS 10000.  // m  altitude at which VS feedforward fades to zero

// ----------------------------
// Zoom phase
// ----------------------------
GLOBAL ASC_ZOOM_PITCH_RATE     IS 0.75.  // deg/s  controlled pitch rise rate
GLOBAL ASC_ZOOM_PITCH_MAX      IS 40.0.  // deg    max pitch in zoom
GLOBAL ASC_ZOOM_FPA_MIN        IS -10.0. // deg    FPA command floor in zoom phase
GLOBAL ASC_ZOOM_FPA_MAX        IS 55.0.  // deg    FPA command ceiling in zoom phase
GLOBAL ASC_ZOOM_APO_RATE_MAX   IS 500.0. // m/s    apoapsis growth rate above which pitch is held in zoom
GLOBAL ASC_Q_ZOOM_ENTRY        IS 0.70.  // fraction of q_target below which zoom eligible
GLOBAL ASC_CEIL_EDOT_THR       IS 1000.0. // m²/s³  orbital Ė below this = speed-ceiling
GLOBAL ASC_CEIL_PERSIST_S      IS 20.0.  // s  orbital Ė must stay below threshold before committing
GLOBAL ASC_ZOOM_APO_WINDOW_S   IS 8.0.  // s  window for apoapsis growth rate estimation

// ----------------------------
// Mode switch persistence
// ----------------------------
GLOBAL ASC_PERSIST_MIN_S    IS 2.0.  // s  minimum persistence window
GLOBAL ASC_PERSIST_MAX_S    IS 6.0.  // s  maximum persistence window
GLOBAL ASC_PERSIST_CONF_BAND IS 0.3. // J confidence band fraction for adaptive window

// ----------------------------
// J_rk drag estimation
// ----------------------------
GLOBAL ASC_DRAG_LOOKAHEAD_S IS 4.0.  // s  drag projection horizon during zoom
GLOBAL ASC_DRAG_FLOOR_FRAC  IS 0.05. // fraction of current drag as absolute floor

// ----------------------------
// Fuel exhaustion floor
// ----------------------------
GLOBAL ASC_FUEL_FLOOR_DV_MARGIN IS 150. // m/s  safety margin above estimated circ ΔV
GLOBAL ASC_PROP_DENSITY         IS 5.0. // kg/unit  LiquidFuel and Oxidizer (KSP stock)
GLOBAL ASC_CORRIDOR_FPA_MIN     IS 0.0.  // deg  FPA command floor in AB corridor (never command descent)
GLOBAL ASC_CORRIDOR_GUARD_AGL_M IS 1500. // m AGL  below this, safety floor overrides FPA minimum
GLOBAL ASC_CORRIDOR_GUARD_APO_FRAC IS 0.90. // fraction of zoom target: if apo below this, guard FPA floor applies
GLOBAL ASC_APO_GUARD_MAX_DEG    IS 10.0. // deg  max nose-up guard bias applied in ROCKET_CLOSEOUT to recover apoapsis
GLOBAL ASC_FUEL_FLOOR_MIN_APO_FRAC IS 0.50. // fraction of zoom_apo required before fuel floor can commit

// ----------------------------
// Propellant equivalency
// ----------------------------
GLOBAL ASC_K_PROP_DEFAULT   IS 0.5.  // coefficient (higher = longer AB preference when OX scarce)

// ----------------------------
// Dual-mode (AB + rocket) surrogate for pre-switch J_rk valuation
// ----------------------------
// While a dual-mode engine is still in AB mode, infer its rocket-mode potential
// conservatively to avoid over-valuing J_rk from AB-mode ISP/thrust.
GLOBAL ASC_DUALMODE_RK_ISP_DEFAULT      IS 320.0. // s   conservative closed-cycle ISP
GLOBAL ASC_DUALMODE_RK_THR_FRAC_DEFAULT IS 0.25.  // 0-1 fraction of current available thrust

// ----------------------------
// Guard thresholds (prevent division by zero)
// ----------------------------
GLOBAL ASC_MIN_MDOT         IS 0.01. // kg/s  minimum mass flow to compute J
GLOBAL ASC_MIN_VORB         IS 50.0. // m/s   minimum orbital speed for projections
GLOBAL ASC_MIN_Q_GAIN       IS 3000. // Pa    minimum q for gain scheduling denominator

// ----------------------------
// Flameout / regime detection
// ----------------------------
GLOBAL ASC_FLAMEOUT_THR_RATIO IS 0.5. // actual/available thrust below this = flameout
GLOBAL ASC_REGIME_MACH_DEFAULT IS 4.5. // Mach  default RAPIER-type regime boundary
GLOBAL ASC_SWITCH_MACH_FLOOR_FRAC IS 0.75. // fraction of regime Mach below which planned AB→rocket/ZOOM transitions are blocked
GLOBAL ASC_FLAMEOUT_MACH_GUARD_FRAC IS 0.85. // fraction of regime Mach below which flameout detection is active (tighter than switch floor — near regime boundary, thrust decline is scheduled not a failure)

// ----------------------------
// Estimator validity thresholds
// ----------------------------
GLOBAL ASC_AOA_OSC_THRESH   IS 3.0.  // deg  AoA oscillation amplitude for DEGRADED
GLOBAL ASC_AOA_OSC_WINDOW_S IS 2.0.  // s    AoA oscillation check window
GLOBAL ASC_EDOT_TOL         IS 0.15. // fractional disagreement force-Ė vs orbital-Ė

// ----------------------------
// Apoapsis targets
// ----------------------------
// Default rocket-takeover target (apoapsis at which we're willing to switch modes).
// Final orbit apoapsis target is specified in aircraft config or here as fallback.
GLOBAL ASC_APO_ZOOM_TARGET_DEFAULT IS 45000.  // m  target apo before committing mode switch
GLOBAL ASC_APO_FINAL_DEFAULT       IS 80000.  // m  target orbit apoapsis
GLOBAL ASC_APO_BAND_M              IS 5000.   // m  apoapsis within this of target = closeout

// ----------------------------
// Rocket sustain control
// ----------------------------
GLOBAL ASC_APO_KP           IS 0.0015. // deg/m  apoapsis error to pitch bias
GLOBAL ASC_ETA_FLATTEN_S    IS 90.0.   // s   ETA:APOAPSIS below this starts flattening
GLOBAL ASC_PITCH_MAX_SUSTAIN IS 25.0.  // deg  max pitch command in rocket phases
GLOBAL ASC_STEER_BLEND_S    IS 4.0.    // s   surface→orbital prograde blend duration
GLOBAL ASC_SPOOL_THR_THRESH IS 0.05.  // fraction of rated AB thrust = spool-down complete

// ----------------------------
// Circularisation
// ----------------------------
GLOBAL ASC_CIRC_ETA_S       IS 45.0.  // s   coast to apoapsis trigger
GLOBAL ASC_CIRC_PERI_TGT    IS 75000. // m   target periapsis for circularisation check

// ----------------------------
// Engine model (ifc_engine_model.ks)
// ----------------------------
GLOBAL EM_G0                     IS 9.80665. // m/s²    standard gravity
GLOBAL EM_KSL_RHO                IS 1.225.   // kg/m³   Kerbin sea-level air density
GLOBAL EM_IA_DENSITY             IS 5.0.     // kg/unit IntakeAir resource density (KSP stock)
GLOBAL EM_LOOKAHEAD_S            IS 30.0.    // s       altitude lookahead for starvation prediction
GLOBAL EM_SCALE_MIN              IS 0.10.    // minimum allowed engine/intake scale factor
GLOBAL EM_SCALE_MAX              IS 10.0.    // maximum allowed engine/intake scale factor
GLOBAL EM_STARVATION_WARN_MARGIN IS -0.05.  // units/s margin threshold for starvation alert
GLOBAL EM_STARVATION_WARN_RATE_S IS 5.0.    // s       minimum interval between starvation alerts
