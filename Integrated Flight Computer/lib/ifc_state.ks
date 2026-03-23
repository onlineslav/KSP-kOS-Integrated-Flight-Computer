@LAZYGLOBAL OFF.

// ============================================================
// ifc_state.ks  -  Integrated Flight Computer
// All mutable runtime globals.  Reset via IFC_INIT_STATE().
// ============================================================

// ----------------------------
// Phase state
// ----------------------------
GLOBAL IFC_PHASE          IS PHASE_APPROACH.
GLOBAL IFC_SUBPHASE       IS SUBPHASE_FLY_TO_FIX.
GLOBAL IFC_PHASE_START_UT IS 0.
GLOBAL IFC_MISSION_START_UT IS 0.

// ----------------------------
// Active approach
// ----------------------------
// Set these before starting the approach loop.
// ACTIVE_PLATE is the approach plate lexicon from nav_beacons.ks.
// ACTIVE_AIRCRAFT is the aircraft config lexicon from an aircraft/*.ks file.
GLOBAL ACTIVE_PLATE    IS 0.  // set before run
GLOBAL ACTIVE_AIRCRAFT IS 0.  // set before run
// Source string for the active aircraft config (path or descriptor).
// Written by ifc_main config-load flow; consumed by ifc_logger metadata fields.
GLOBAL IFC_ACTIVE_CFG_PATH IS "".

// Derived from ACTIVE_PLATE at approach start (cached for speed):
GLOBAL ACTIVE_ILS_ID   IS "".
GLOBAL ACTIVE_RWY_HDG  IS 90.
GLOBAL ACTIVE_GS_ANGLE IS 3.0.
GLOBAL ACTIVE_THR_ALT  IS 69.
GLOBAL ACTIVE_V_APP    IS 80.
GLOBAL ACTIVE_V_TGT    IS 80. // current dynamic approach speed target (m/s)
GLOBAL ACTIVE_FIXES    IS LIST().  // ordered list of beacon IDs
GLOBAL ACTIVE_ALT_AT   IS LEXICON(). // beacon_id -> target altitude (m)

// Index into ACTIVE_FIXES for FLY_TO_FIX sub-phase
GLOBAL FIX_INDEX IS 0.

// ----------------------------
// Steering and throttle commands
// ----------------------------
GLOBAL IFC_DESIRED_STEERING IS HEADING(90, 0). // locked once at flight start; phase code writes to this
GLOBAL THROTTLE_CMD     IS 0.
GLOBAL THR_INTEGRAL     IS 0.   // accumulated speed error for integral trim
GLOBAL PREV_IAS         IS 0.   // IAS from previous cycle for d(IAS)/dt measurement
GLOBAL A_ACTUAL_FILT    IS 0.   // low-pass filtered measured acceleration (m/s²)
GLOBAL AT_EST_INIT         IS FALSE. // estimator initialization latch
GLOBAL AT_PREV_THR_CUR     IS 0.     // previous measured main throttle
GLOBAL AT_PREV_A_FILT      IS 0.     // previous filtered acceleration
GLOBAL AT_GAIN_EST         IS 1.0.   // estimated d(accel)/d(throttle)
GLOBAL AT_TAU_EST          IS 0.6.   // estimated effective throttle lag (s)
GLOBAL AT_A_THRUST_MAX_EST IS 0.0.   // estimated max positive accel authority (m/s²)
GLOBAL AT_IDLE_DECEL_EST   IS 0.7.   // estimated idle decel authority (m/s², positive magnitude)
GLOBAL APP_ON_FINAL IS FALSE. // TRUE once LOC/GS capture is stable for final-speed mode
GLOBAL APP_FINAL_ARM_UT IS -1. // timer used to debounce entry to final-speed mode
GLOBAL APP_SPD_MODE IS "INIT". // FIXES / INTERCEPT / FINAL / SHORT_FINAL
GLOBAL APP_VREF_TGT IS 0. // current computed Vref used by scheduler (m/s)
GLOBAL APP_VINT_TGT IS 0. // current computed intercept speed target (m/s)
GLOBAL APP_BASE_V_TGT IS 0. // current unslewed phase target speed (m/s)
GLOBAL APP_SHORT_FINAL_FRAC IS 0. // 0..1 short-final blend fraction (Vapp->Vref)
GLOBAL APP_LOC_CAP_OK IS 0. // 1 when |LOC| is inside capture band, else 0
GLOBAL APP_GS_CAP_OK IS 0.  // 1 when |GS| is inside capture band, else 0
GLOBAL APP_GS_LATCHED IS FALSE. // TRUE once GS captured; releases only at GS_LATCH_RELEASE_FACTOR * GS_CAPTURE_M

// ----------------------------
// ILS deviation state  (updated each cycle in ILS_TRACK)
// ----------------------------
GLOBAL ILS_LOC_DEV      IS 0.      // m, + = right of centerline
GLOBAL ILS_GS_DEV       IS 0.      // m, + = above glideslope
GLOBAL ILS_DIST_M       IS 0.      // m  horizontal from threshold (+ = on approach)
GLOBAL ILS_INTERCEPT_ALT IS 0.     // m MSL altitude held until GS capture

// Cached per plate-load (eliminates per-cycle GET_BEACON + TAN calls in _COMPUTE_ILS_DEVIATIONS)
GLOBAL ILS_GS_TAN_CACHED IS 0.     // TAN(ACTIVE_GS_ANGLE), set in IFC_LOAD_PLATE
GLOBAL ILS_THR_GEO_LL    IS 0.     // GeoCoordinates of ILS threshold, set in IFC_LOAD_PLATE

// Previous-cycle values for derivative term
GLOBAL PREV_LOC_DEV IS 0.
GLOBAL PREV_GS_DEV  IS 0.

// ----------------------------
// AA state flags
// ----------------------------
GLOBAL AA_AVAILABLE    IS FALSE.
GLOBAL AA_DIRECTOR_ON  IS FALSE.
GLOBAL AA_FBW_ON       IS FALSE.

// ----------------------------
// FAR state flag
// ----------------------------
GLOBAL FAR_AVAILABLE IS FALSE.

// ----------------------------
// Per-cycle cached vessel vectors
// Computed once at the top of the main loop; all subsystems read these instead of
// querying SHIP:FACING / SHIP:UP / SHIP:NORTH directly (saves ~6 cross-VM calls/consumer).
// ----------------------------
GLOBAL IFC_FACING_FWD  IS V(0, 0, 1).   // SHIP:FACING:FOREVECTOR
GLOBAL IFC_FACING_STAR IS V(1, 0, 0).   // SHIP:FACING:STARVECTOR
GLOBAL IFC_FACING_TOP  IS V(0, 1, 0).   // SHIP:FACING:TOPVECTOR
GLOBAL IFC_UP_VEC      IS V(0, 1, 0).   // SHIP:UP:VECTOR
GLOBAL IFC_NORTH_VEC   IS V(0, 0, 1).   // SHIP:NORTH:VECTOR

// ----------------------------
// Loop timing
// ----------------------------
GLOBAL IFC_CYCLE_UT  IS 0.    // TIME:SECONDS at start of last cycle (for real dt)
GLOBAL IFC_RAW_DT    IS 0.05. // unclamped elapsed time of the most recent loop cycle (s)
GLOBAL IFC_ACTUAL_DT IS 0.05. // measured elapsed time of the most recent loop cycle (s)
GLOBAL IFC_LOOP_COUNT IS 0.   // total control-loop iterations since IFC_INIT_STATE
GLOBAL IFC_LOOP_COUNT_SNAPSHOT IS 0. // loop counter snapshot at last CSV write
GLOBAL IFC_RAW_DT_MAX IS 0.05. // max raw dt observed since last CSV write
GLOBAL IFC_RAW_DT_MIN IS 0.05. // min raw dt observed since last CSV write

// ----------------------------
// Telemetry (legacy rate-limiter; kept for ifc_telemetry.ks compat)
// ----------------------------
GLOBAL LAST_TELEM_UT IS 0.

// ----------------------------
// Display / UI state
// ----------------------------
GLOBAL UI_W               IS 50.     // terminal width, set by UI_INIT
GLOBAL IFC_DEBUG_PANEL_ON IS FALSE.  // show secondary debug zone
GLOBAL LAST_DISPLAY_UT    IS 0.      // primary zone rate-limiter
GLOBAL LAST_HEADER_UT     IS 0.      // header rate-limiter
GLOBAL LAST_SECONDARY_UT  IS 0.      // secondary zone rate-limiter
GLOBAL LAST_LOGGER_UT     IS 0.      // logger bar rate-limiter
GLOBAL IFC_ALERT_TEXT     IS "".     // current alert message
GLOBAL IFC_ALERT_UT       IS -99.    // UT when alert was set (for expiry)
GLOBAL IFC_ALERT_LAST_LINE IS "__INIT__". // last rendered alert row text (prevents redundant UI writes)
GLOBAL IFC_MENU_OPEN      IS FALSE.  // TRUE when menu overlay is visible
GLOBAL IFC_MENU_DIRTY     IS FALSE.  // TRUE when overlay needs redraw
GLOBAL IFC_UI_MODE        IS "".     // active interaction mode (UI_MODE_*)
GLOBAL IFC_FAST_MODE      IS FALSE.  // TRUE = skip UI/menu rendering work in main loop
GLOBAL IFC_EVENT_QUEUE    IS LIST(). // recent events, newest at end
GLOBAL IFC_EVENT_LAST_UT  IS -99.    // last alert UT mirrored into queue
GLOBAL IFC_EVENT_LAST_TEXT IS "".    // last mirrored alert text
GLOBAL IFC_EVENT_VIEW_IDX IS 0.      // newest event index shown in history view

// ----------------------------
// Telemetry export
// Written each cycle by phase functions so ifc_logger can read them
// without needing access to local variables.
// ----------------------------
// Per-cycle attitude telemetry (computed once in main loop; display + logger read these)
GLOBAL TELEM_COMPASS_HDG   IS 0.  // compass heading of nose (deg, 0=N 90=E)
GLOBAL TELEM_PITCH_DEG     IS 0.  // nose pitch above horizon (deg)
GLOBAL TELEM_BANK_DEG      IS 0.  // bank angle (deg, + = right wing down)

GLOBAL TELEM_AA_HDG_CMD    IS 0.  // heading sent to AA Director (deg)
GLOBAL TELEM_AA_FPA_CMD    IS 0.  // FPA sent to AA Director (deg)
GLOBAL TELEM_LOC_CORR      IS 0.  // ILS localizer heading correction (deg)
GLOBAL TELEM_GS_CORR       IS 0.  // ILS glideslope FPA correction (deg)
GLOBAL TELEM_D_GS          IS 0.  // raw d(GS_dev)/dt before combining into gs_corr (m/s)
GLOBAL TELEM_FPA_PRECLAMPED IS 0. // fpa_cmd before AOA/pitch-cap clamp (deg)
GLOBAL TELEM_FLARE_TGT_VS  IS 0.  // target sink rate from flare schedule (m/s)
GLOBAL TELEM_FLARE_FRAC    IS 0.  // flare progress: 0 = entry AGL, 1 = ground
GLOBAL TELEM_STEER_BLEND   IS 0.  // rollout wheelsteering blend factor (0-1)
GLOBAL TELEM_RO_HDG_ERR    IS 0.  // rollout: actual hdg minus steer target (deg)
GLOBAL TELEM_RO_YAW_TGT    IS 0.  // rollout: yaw command target before slew rate
GLOBAL TELEM_RO_LOC_CORR   IS 0.  // rollout: heading correction from localizer (deg)
GLOBAL TELEM_RO_ROLL_ASSIST IS 0. // rollout: 1 when IFC roll assist is active, else 0
GLOBAL TELEM_RO_YAW_SCALE  IS 0.  // rollout: yaw assist blend factor (0-1)
GLOBAL TELEM_RO_YAW_GATE   IS 0.  // rollout yaw gate: 0=active,1=speed,2=guard,3=not landed
GLOBAL TELEM_RO_PITCH_TGT  IS 0.  // rollout: target pitch attitude (deg)
GLOBAL TELEM_RO_PITCH_ERR  IS 0.  // rollout: pitch attitude error (deg)
GLOBAL TELEM_RO_PITCH_FF   IS 0.  // rollout: feedforward pitch command bias
GLOBAL TELEM_AT_V_TGT      IS 0.  // adaptive throttle: target IAS (m/s)
GLOBAL TELEM_AT_A_CMD      IS 0.  // adaptive throttle: commanded accel (m/s²)
GLOBAL TELEM_AT_A_ACT      IS 0.  // adaptive throttle: filtered measured accel (m/s²)
GLOBAL TELEM_AT_GAIN       IS 0.  // adaptive throttle: estimated throttle effectiveness
GLOBAL TELEM_AT_TAU        IS 0.  // adaptive throttle: estimated throttle lag (s)
GLOBAL TELEM_AT_A_UP_LIM   IS 0.  // adaptive throttle: +accel clamp (m/s²)
GLOBAL TELEM_AT_A_DN_LIM   IS 0.  // adaptive throttle: -accel clamp magnitude (m/s²)
GLOBAL TELEM_AT_KP_THR     IS 0.  // adaptive throttle: scheduled KP_ACL_THR
GLOBAL TELEM_AT_KI_SPD     IS 0.  // adaptive throttle: scheduled KI_SPD
GLOBAL TELEM_AT_THR_SLEW   IS 0.  // adaptive throttle: scheduled throttle slew (/s)

// Diagnostic telemetry for AA mode and approach guidance
GLOBAL TELEM_AA_FBW_ON      IS 0.  // actual aa:FBW state this cycle (0=off, 1=on)
GLOBAL TELEM_AA_DIR_ON      IS 0.  // actual aa:DIRECTOR state this cycle (0=off, 1=on)
GLOBAL TELEM_ACTUAL_FPA_DEG IS 0.  // actual flight path angle = ARCTAN(VS/IAS) (deg)
GLOBAL TELEM_FTF_HDG_ERR    IS 0.  // FLY_TO_FIX: heading error to current fix (0-180 deg)
GLOBAL TELEM_FTF_FIX_IDX    IS 0.  // FLY_TO_FIX: current fix index
GLOBAL TELEM_KOS_STEER_PIT  IS 0.  // pitch angle of IFC_DESIRED_STEERING (kOS lock target, deg)
GLOBAL TELEM_KOS_STEER_HDG  IS 0.  // compass heading of IFC_DESIRED_STEERING (kOS lock target, deg)
GLOBAL TELEM_AA_DIR_VX        IS 0.  // X component of direction vector passed to aa:DIRECTION (world frame)
GLOBAL TELEM_AA_DIR_VY        IS 0.  // Y component
GLOBAL TELEM_AA_DIR_VZ        IS 0.  // Z component
GLOBAL TELEM_AA_DIR_PITCH_DEG IS 0.  // actual pitch of the direction vector (should match aa_fpa_cmd_deg)
GLOBAL TELEM_AA_DIR_HDG_DEG   IS 0.  // actual heading of the direction vector (should match aa_hdg_cmd_deg)

// ----------------------------
// Flap detent state
// ----------------------------
GLOBAL FLAPS_CURRENT_DETENT    IS 0.
GLOBAL FLAPS_TARGET_DETENT     IS 0.
GLOBAL FLAPS_LAST_STEP_UT      IS 0.
GLOBAL FLAPS_LAST_TARGET_LOGGED IS -1.

// ----------------------------
// Approach spoiler arming
// ----------------------------
GLOBAL APP_SPOILERS_ARMED IS FALSE. // TRUE once ag_spoilers_arm has been triggered in-flight

// ----------------------------
// Takeoff phase state
// ----------------------------
GLOBAL TO_RWY_HDG          IS 90.  // runway heading for takeoff (set by RUN_TAKEOFF_IFC)
GLOBAL TO_ROTATE_PITCH_CMD IS 0.   // current ramped pitch target during rotation (deg)
GLOBAL TO_AIRBORNE_UT      IS -1.  // UT when airborne was first detected (debounce)
GLOBAL TO_STAGE_ATTEMPTS   IS 0.   // auto-stage attempts used during takeoff
GLOBAL TO_LAST_STAGE_UT    IS -9999. // UT of last auto-stage attempt
GLOBAL TO_YAW_CMD_PREV     IS 0.   // previous-cycle takeoff yaw cmd for slew limiting
GLOBAL TO_PITCH_CMD_PREV   IS 0.   // previous-cycle takeoff pitch cmd for slew limiting
GLOBAL TO_HDG_PREV         IS 0.   // previous-cycle heading for yaw-rate damping in takeoff rollout
GLOBAL TO_CLIMB_FPA_CMD    IS 0.   // slewed climb FPA command (deg) for rotate->climb transition
GLOBAL TO_SPOOL_PREV_AVAIL IS 0.   // previous-cycle available thrust during preflight spool gate (kN)
GLOBAL TO_SPOOL_STABLE_UT  IS -1.  // UT when thrust first entered steady-state window
GLOBAL TO_START_POS        IS V(0, 0, 0). // vessel position at first preflight tick (centerline reference)

// ----------------------------
// Flight plan / leg queue
// ----------------------------
GLOBAL FLIGHT_PLAN       IS LIST(). // ordered LIST of leg LEXICONs
GLOBAL FLIGHT_PLAN_INDEX IS 0.      // index of currently executing leg

// ----------------------------
// FMS plan editor state  (pre-arm only)
// ----------------------------
GLOBAL DRAFT_PLAN        IS LIST(). // leg list being built in pre-arm
GLOBAL FMS_LEG_CURSOR    IS 0.      // selected leg index in list view
GLOBAL FMS_EDIT_FIELD    IS 0.      // selected field index when editing a leg
GLOBAL FMS_EDITING_LEG   IS FALSE.  // TRUE = per-leg edit overlay is open

// ----------------------------
// Manual override (CWS)
// ----------------------------
GLOBAL IFC_MANUAL_MODE IS FALSE. // TRUE = autopilot suspended, pilot has control

// ----------------------------
// Cruise phase state
// ----------------------------
GLOBAL CRUISE_WAYPOINTS   IS LIST().      // ordered list of beacon IDs to navigate
GLOBAL CRUISE_WP_INDEX    IS 0.           // current index into CRUISE_WAYPOINTS
GLOBAL CRUISE_ALT_M       IS 3000.        // target cruise altitude MSL (m)
GLOBAL CRUISE_SPD_MPS     IS 150.         // target cruise IAS (m/s)
GLOBAL CRUISE_DEST_PLATE  IS 0.           // (legacy) approach plate for destination
GLOBAL CRUISE_NAV_TYPE    IS "waypoint".  // "waypoint" | "course_dist" | "course_time"
GLOBAL CRUISE_COURSE_DEG  IS 0.           // compass heading for course-based cruise
GLOBAL CRUISE_END_UT      IS -1.          // UT when course_time cruise ends (-1 = unused)

// ----------------------------
// Autoland phase state
// ----------------------------
GLOBAL FLARE_PITCH_CMD  IS 0.   // current commanded FPA during flare (deg, smoothed)
GLOBAL FLARE_ENTRY_VS   IS 0.   // vertical speed at flare trigger (m/s, negative)
GLOBAL FLARE_ENTRY_AGL  IS 15.  // AGL at flare trigger (m)
GLOBAL FLARE_TRIGGER_START_UT IS -1. // debounce timer start for entering flare
GLOBAL TOUCHDOWN_CANDIDATE_UT IS -1. // debounce timer start for entering touchdown
GLOBAL TOUCHDOWN_INIT_DONE IS FALSE. // one-time touchdown handoff latch
GLOBAL TOUCHDOWN_CAPTURE_PITCH_DEG IS 0. // pitch snapshot captured at flare->touchdown transition
GLOBAL BOUNCE_RECOVERY_START_UT IS -1. // debounce timer for confirming a real airborne bounce
GLOBAL ROLLOUT_ENTRY_HDG IS 0.  // heading captured at touchdown for blended wheelsteering
GLOBAL ROLLOUT_YAW_CMD_PREV IS 0. // previous-cycle rudder command for slew limiting
GLOBAL ROLLOUT_PITCH_CMD_PREV IS 0. // previous-cycle pitch command for slew limiting
GLOBAL ROLLOUT_PITCH_REF_DEG IS 0. // touchdown-captured pitch attitude reference (deg)
GLOBAL ROLLOUT_PITCH_TGT_DEG IS 0. // time-evolving pitch target for rollout nose lowering (deg)
GLOBAL ROLLOUT_STEER_HDG IS 0.         // computed wheelsteering target (logged for telemetry)
GLOBAL ROLLOUT_REV_DEACTIVATED IS FALSE. // TRUE once reversers have been cut during rollout

// ----------------------------
// GUI editor state
// ----------------------------
GLOBAL GUI_WIN IS 0.                   // kOS GUI window handle; 0 = not open
GLOBAL GUI_ADD_BTN IS 0.              // [+ ADD LEG] button
GLOBAL GUI_ARM_BTN IS 0.              // [ARM] button
GLOBAL GUI_QUIT_BTN IS 0.             // [QUIT] button
GLOBAL GUI_LEG_SEL_BTNS IS LIST().    // numbered select buttons, one per leg row
GLOBAL GUI_LEG_UP_BTNS IS LIST().     // [^] move-up buttons
GLOBAL GUI_LEG_DN_BTNS IS LIST().     // [v] move-down buttons
GLOBAL GUI_LEG_DEL_BTNS IS LIST().    // [X] delete buttons
GLOBAL GUI_LEG_SUM_LBLS IS LIST().    // leg summary labels
GLOBAL GUI_FIELD_LBLS IS LIST().      // value labels in the edit panel
GLOBAL GUI_FIELD_DEC_BTNS IS LIST().  // [<] buttons in the edit panel
GLOBAL GUI_FIELD_INC_BTNS IS LIST().  // [>] buttons in the edit panel
GLOBAL GUI_SLOT_SAVE_BTNS IS LIST().  // [SAVE n] buttons for slots 1-5
GLOBAL GUI_SLOT_LOAD_BTNS IS LIST().  // [LOAD n] buttons for slots 1-5
GLOBAL GUI_PLAN_LBL IS 0.            // "FLIGHT PLAN (n legs)" label
GLOBAL GUI_EDIT_HDR_LBL IS 0.        // (unused — kept for compatibility)
GLOBAL GUI_FIELD_NAME_LBLS IS LIST(). // (unused — kept for compatibility)
GLOBAL GUI_EDIT_WIN IS 0.            // edit sub-window handle
GLOBAL GUI_EDIT_HANDLES IS LIST().   // widget handles in edit sub-window
GLOBAL GUI_EDIT_LEG_TYPE IS -1.      // leg type currently shown in edit window
GLOBAL GUI_EDIT_NAV_TYPE IS "".      // cruise nav_type currently shown in edit window

// ----------------------------
// Ascent phase state
// ----------------------------
GLOBAL ASC_INITIALIZED    IS FALSE.  // TRUE after ASC_STATE_INIT has run

// Heading / FPA filter state (captured at ascent init)
GLOBAL ASC_HDG_REF        IS 90.0.  // deg  locked heading at phase entry
GLOBAL ASC_SURF_FPA_FILT  IS 0.0.   // deg  EMA-smoothed surface FPA
GLOBAL ASC_HDG_CMD_FILT   IS 90.0.  // deg  EMA-smoothed heading command

// Engine classification lists (cached at ascent init, never iterated in loop)
GLOBAL ASC_AB_ENGINES     IS LIST().
GLOBAL ASC_RK_ENGINES     IS LIST().

// Propellant capacity for equivalency weight (stored at init)
GLOBAL ASC_LF_MAX         IS 1.0.   // kg  total liquid fuel capacity
GLOBAL ASC_OX_MAX         IS 1.0.   // kg  total oxidizer capacity

// Estimator validity state
GLOBAL ASC_ESTIMATOR_VALIDITY IS "VALID".

// Frame-consistent facing vectors (refreshed each cycle, used by all downstream)
GLOBAL ASC_STAR_V         IS V(1,0,0).
GLOBAL ASC_TOP_V          IS V(0,1,0).
GLOBAL ASC_NFWD_V         IS V(0,0,-1). // -FOREVECTOR (backward body axis = +Z in AEROFORCE frame)

// Control-smoothed signals  (fast α, drives pitch/throttle commands)
GLOBAL ASC_AERO_ALONG_CTRL IS 0. // N  along-track aero force, fast copy
GLOBAL ASC_Q_CTRL          IS 0. // Pa dynamic pressure, fast copy
GLOBAL ASC_AOA_CTRL        IS 0. // deg angle of attack, fast copy

// Valuation-smoothed signals  (slow α, drives mode-switch decisions only)
GLOBAL ASC_AERO_ALONG_VAL  IS 0. // N  along-track aero force, slow copy
GLOBAL ASC_Q_VAL           IS 0. // Pa dynamic pressure, slow copy (for penalty functions)
GLOBAL ASC_EDOT_VAL        IS 0. // J/kg/s  specific orbital energy rate, slow copy
GLOBAL ASC_APO_VAL         IS 0. // m  smoothed apoapsis altitude
GLOBAL ASC_MDOT_AB_VAL     IS 0. // kg/s  AB mass flow, slow copy
GLOBAL ASC_MDOT_RK_VAL     IS 0. // kg/s  rocket mass flow estimate, slow copy

// Dedicated drag filter for J_rk  (heaviest filter in the system)
GLOBAL ASC_DRAG_RK_VAL     IS 0. // N  along-track drag magnitude for J_rk estimate
GLOBAL ASC_DRAG_DDOT       IS 0. // N/s  smoothed drag rate of change (for zoom projection)
GLOBAL ASC_DRAG_PREV       IS 0. // N  previous cycle drag (raw) for dD/dt

// Specific orbital energy tracking
GLOBAL ASC_ENERGY_PREV     IS 0. // J/kg  energy at end of previous cycle
GLOBAL ASC_EDOT_ORBITAL    IS 0. // J/kg/s  dE/dt from orbital state (raw, for logging)
GLOBAL ASC_EDOT_ORB_FILT   IS 0. // J/kg/s  EMA-smoothed orbital Ė (validity cross-check)

// Mode valuations
GLOBAL ASC_J_AB            IS 0. // J/kg per kg/s  air-breathing mode value
GLOBAL ASC_J_RK            IS 0. // J/kg per kg/s  rocket mode value

// AoA oscillation tracking
GLOBAL ASC_AOA_MAX_WIN     IS 0. // deg  max AoA seen in current validity window
GLOBAL ASC_AOA_MIN_WIN     IS 0. // deg  min AoA seen in current validity window
GLOBAL ASC_AOA_WIN_UT      IS 0. // UT  start of current AoA window

// Apoapsis growth rate tracking
GLOBAL ASC_APO_PREV        IS 0. // m  apoapsis at start of growth window
GLOBAL ASC_APO_PREV_UT     IS 0. // UT  start of growth window

// Mode switch persistence
GLOBAL ASC_PERSIST_UT      IS -1. // UT when J_rk > J_ab was first satisfied (-1 = not active)

// Steering reference blend (0 = surface prograde, 1 = orbital prograde)
GLOBAL ASC_STEER_BLEND     IS 0.
GLOBAL ASC_BLEND_START_UT  IS -1.

// Pitch bias (slew-rate-limited, fed to AA Director as FPA offset)
GLOBAL ASC_PITCH_BIAS      IS 0. // deg  current active pitch bias above prograde

// Spooling state
GLOBAL ASC_IS_SPOOLING     IS FALSE.

// Flameout flag (set by WHEN...THEN trigger; cleared on phase exit)
GLOBAL ASC_FLAMEOUT        IS FALSE.

// AB thrust ratio cache — updated each cycle for the WHEN...THEN trigger.
// Using a cached scalar keeps the WHEN condition expression cheap.
GLOBAL ASC_AB_THR_RATIO       IS 1.   // actual / available AB thrust (1 = healthy)
// Regime Mach boundary cached at ascent init — avoids AC_PARAM call in WHEN expression.
GLOBAL ASC_REGIME_MACH_CACHED IS 4.5. // Mach
// Per-aircraft ascent parameters cached at ascent init — avoids AC_PARAM
// lookups inside per-cycle guidance/valuation hot paths.
GLOBAL ASC_Q_TARGET_CACHED         IS ASC_Q_TARGET_DEFAULT.
GLOBAL ASC_Q_MAX_CACHED            IS ASC_Q_MAX_DEFAULT.
GLOBAL ASC_Q_MIN_CACHED            IS ASC_Q_MIN_DEFAULT.
GLOBAL ASC_HEAT_LIMIT_CACHED       IS 3.5e8.
GLOBAL ASC_ZOOM_APO_TARGET_CACHED  IS ASC_APO_ZOOM_TARGET_DEFAULT.
GLOBAL ASC_FINAL_APO_TARGET_CACHED IS ASC_APO_FINAL_DEFAULT.
GLOBAL ASC_AOA_LIMIT_CACHED        IS AA_MAX_AOA.
GLOBAL ASC_CORRIDOR_FPA_MAX_CACHED IS 25.0.
GLOBAL ASC_DUALMODE_RK_ISP_CACHED  IS ASC_DUALMODE_RK_ISP_DEFAULT.
GLOBAL ASC_DUALMODE_RK_THR_CACHED  IS ASC_DUALMODE_RK_THR_FRAC_DEFAULT.
GLOBAL ASC_K_PROP_CACHED           IS ASC_K_PROP_DEFAULT.
GLOBAL ASC_PRESSURE_CACHED         IS 0.      // atm  atmospheric pressure cached once per cycle
GLOBAL ASC_RK_DUALMODE             IS LIST(). // bool per ASC_RK_ENGINES entry: TRUE = multimode engine also in AB list
GLOBAL ASC_CEIL_PERSIST_UT         IS -1.     // UT when speed-ceiling persistence began (-1 = not active)

// Telemetry exports  (TELEM_ prefix = readable by logger without phase-local access)
GLOBAL TELEM_ASC_J_AB       IS 0.
GLOBAL TELEM_ASC_J_RK       IS 0.
GLOBAL TELEM_ASC_VALIDITY   IS "VALID".
GLOBAL TELEM_ASC_Q          IS 0.
GLOBAL TELEM_ASC_Q_RAW      IS 0.
GLOBAL TELEM_ASC_MACH       IS 0.
GLOBAL TELEM_ASC_APO        IS 0.
GLOBAL TELEM_ASC_DRAG_RK    IS 0.
GLOBAL TELEM_ASC_W_PROP     IS 1.
GLOBAL TELEM_ASC_EDOT_VAL   IS 0.
GLOBAL TELEM_ASC_EDOT_ORB   IS 0.
GLOBAL TELEM_ASC_PITCH_BIAS IS 0.
GLOBAL TELEM_ASC_BLEND      IS 0.
GLOBAL TELEM_ASC_SPOOLING   IS FALSE.
GLOBAL TELEM_ASC_AB_THR_RATIO    IS 0.
GLOBAL TELEM_ASC_AB_T_NOW        IS 0.
GLOBAL TELEM_ASC_AB_T_AVAIL      IS 0.
GLOBAL TELEM_ASC_AB_IGN_ON       IS 0.
GLOBAL TELEM_ASC_AB_FLAMEOUTS    IS 0.
GLOBAL TELEM_ASC_RK_T_NOW        IS 0.
GLOBAL TELEM_ASC_RK_T_AVAIL      IS 0.
GLOBAL TELEM_ASC_RK_IGN_ON       IS 0.
GLOBAL TELEM_ASC_RK_FLAMEOUTS    IS 0.

// ----------------------------
// Init / reset
// ----------------------------
FUNCTION IFC_INIT_STATE {
  SET IFC_PHASE          TO PHASE_APPROACH.
  SET IFC_SUBPHASE       TO SUBPHASE_FLY_TO_FIX.
  SET IFC_PHASE_START_UT TO TIME:SECONDS.
  SET IFC_MISSION_START_UT TO TIME:SECONDS.
  SET IFC_ACTIVE_CFG_PATH TO "".

  SET ACTIVE_ILS_ID   TO "".
  SET ACTIVE_RWY_HDG  TO 90.
  SET ACTIVE_GS_ANGLE TO 3.0.
  SET ACTIVE_THR_ALT  TO 69.
  SET ACTIVE_V_APP    TO 80.
  SET ACTIVE_V_TGT    TO 80.
  SET ACTIVE_FIXES    TO LIST().
  SET ACTIVE_ALT_AT   TO LEXICON().
  SET FIX_INDEX TO 0.

  SET IFC_DESIRED_STEERING TO HEADING(90, 0).
  SET THROTTLE_CMD  TO 0.
  SET THR_INTEGRAL  TO 0.
  SET PREV_IAS      TO GET_IAS().
  SET A_ACTUAL_FILT TO 0.
  SET AT_EST_INIT         TO FALSE.
  SET AT_PREV_THR_CUR     TO 0.
  SET AT_PREV_A_FILT      TO 0.
  SET AT_GAIN_EST         TO AT_GAIN_INIT.
  SET AT_TAU_EST          TO AT_TAU_INIT.
  SET AT_A_THRUST_MAX_EST TO 0.
  SET AT_IDLE_DECEL_EST   TO AT_IDLE_DECEL_INIT.
  SET APP_ON_FINAL TO FALSE.
  SET APP_FINAL_ARM_UT TO -1.
  SET APP_SPD_MODE TO "INIT".
  // Speed targets are 0 here; IFC_LOAD_PLATE (called immediately after
  // INIT_STATE) sets them to their correct ACTIVE_V_APP-derived values.
  SET APP_VREF_TGT TO 0.
  SET APP_VINT_TGT TO 0.
  SET APP_BASE_V_TGT TO 0.
  SET APP_SHORT_FINAL_FRAC TO 0.
  SET APP_LOC_CAP_OK TO 0.
  SET APP_GS_CAP_OK TO 0.
  SET APP_GS_LATCHED TO FALSE.

  SET ILS_LOC_DEV       TO 0.
  SET ILS_GS_DEV        TO 0.
  SET ILS_DIST_M        TO 0.
  SET ILS_INTERCEPT_ALT TO 0.
  SET PREV_LOC_DEV      TO 0.
  SET PREV_GS_DEV       TO 0.
  SET ILS_GS_TAN_CACHED TO 0.
  SET ILS_THR_GEO_LL    TO 0.

  SET AA_AVAILABLE   TO FALSE.
  SET AA_DIRECTOR_ON TO FALSE.
  SET AA_FBW_ON      TO FALSE.
  SET FAR_AVAILABLE  TO FALSE.

  SET LAST_TELEM_UT    TO TIME:SECONDS.
  SET LAST_DISPLAY_UT  TO 0.
  SET LAST_HEADER_UT   TO 0.
  SET LAST_SECONDARY_UT TO 0.
  SET LAST_LOGGER_UT   TO 0.
  SET IFC_ALERT_TEXT   TO "".
  SET IFC_ALERT_UT     TO -99.
  SET IFC_ALERT_LAST_LINE TO "__INIT__".
  SET IFC_MENU_OPEN    TO FALSE.
  SET IFC_MENU_DIRTY   TO FALSE.
  SET IFC_UI_MODE      TO UI_MODE_PREARM.
  SET IFC_EVENT_QUEUE  TO LIST().
  SET IFC_EVENT_LAST_UT TO -99.
  SET IFC_EVENT_LAST_TEXT TO "".
  SET IFC_EVENT_VIEW_IDX TO 0.
  SET IFC_CYCLE_UT     TO TIME:SECONDS.
  SET IFC_RAW_DT       TO IFC_LOOP_DT.
  SET IFC_ACTUAL_DT    TO IFC_LOOP_DT.
  SET IFC_FACING_FWD   TO SHIP:FACING:FOREVECTOR.
  SET IFC_FACING_STAR  TO SHIP:FACING:STARVECTOR.
  SET IFC_FACING_TOP   TO SHIP:FACING:TOPVECTOR.
  SET IFC_UP_VEC       TO SHIP:UP:VECTOR.
  SET IFC_NORTH_VEC    TO SHIP:NORTH:VECTOR.
  SET IFC_LOOP_COUNT   TO 0.
  SET IFC_LOOP_COUNT_SNAPSHOT TO 0.
  SET IFC_RAW_DT_MAX   TO IFC_LOOP_DT.
  SET IFC_RAW_DT_MIN   TO IFC_LOOP_DT.
  SET FLARE_PITCH_CMD  TO 0.
  SET FLARE_ENTRY_VS   TO 0.
  SET FLARE_ENTRY_AGL  TO FLARE_AGL_M.
  SET FLARE_TRIGGER_START_UT TO -1.
  SET TOUCHDOWN_CANDIDATE_UT TO -1.
  SET TOUCHDOWN_INIT_DONE TO FALSE.
  SET TOUCHDOWN_CAPTURE_PITCH_DEG TO 0.
  SET BOUNCE_RECOVERY_START_UT TO -1.
  SET ROLLOUT_ENTRY_HDG   TO GET_COMPASS_HDG().
  SET ROLLOUT_YAW_CMD_PREV TO 0.
  SET ROLLOUT_PITCH_CMD_PREV TO 0.
  SET ROLLOUT_PITCH_REF_DEG TO GET_PITCH().
  SET ROLLOUT_PITCH_TGT_DEG TO GET_PITCH().
  SET ROLLOUT_STEER_HDG       TO GET_COMPASS_HDG().
  SET ROLLOUT_REV_DEACTIVATED TO FALSE.

  SET TELEM_COMPASS_HDG  TO 0.
  SET TELEM_PITCH_DEG    TO 0.
  SET TELEM_BANK_DEG     TO 0.
  SET TELEM_AA_HDG_CMD    TO 0.
  SET TELEM_AA_FPA_CMD    TO 0.
  SET TELEM_AA_FBW_ON     TO 0.
  SET TELEM_AA_DIR_ON     TO 0.
  SET TELEM_ACTUAL_FPA_DEG TO 0.
  SET TELEM_FTF_HDG_ERR   TO 0.
  SET TELEM_FTF_FIX_IDX   TO 0.
  SET TELEM_KOS_STEER_PIT TO 0.
  SET TELEM_KOS_STEER_HDG TO 0.
  SET TELEM_AA_DIR_VX     TO 0.
  SET TELEM_AA_DIR_VY     TO 0.
  SET TELEM_AA_DIR_VZ         TO 0.
  SET TELEM_AA_DIR_PITCH_DEG  TO 0.
  SET TELEM_AA_DIR_HDG_DEG    TO 0.
  SET TELEM_LOC_CORR      TO 0.
  SET TELEM_GS_CORR       TO 0.
  SET TELEM_D_GS          TO 0.
  SET TELEM_FPA_PRECLAMPED TO 0.
  SET TELEM_FLARE_TGT_VS TO 0.
  SET TELEM_FLARE_FRAC   TO 0.
  SET TELEM_STEER_BLEND  TO 0.
  SET TELEM_RO_HDG_ERR   TO 0.
  SET TELEM_RO_YAW_TGT   TO 0.
  SET TELEM_RO_LOC_CORR  TO 0.
  SET TELEM_RO_ROLL_ASSIST TO 0.
  SET TELEM_RO_YAW_SCALE TO 0.
  SET TELEM_RO_YAW_GATE TO 0.
  SET TELEM_RO_PITCH_TGT TO 0.
  SET TELEM_RO_PITCH_ERR TO 0.
  SET TELEM_RO_PITCH_FF TO 0.
  SET TELEM_AT_V_TGT    TO 0.
  SET TELEM_AT_A_CMD    TO 0.
  SET TELEM_AT_A_ACT    TO 0.
  SET TELEM_AT_GAIN     TO 0.
  SET TELEM_AT_TAU      TO 0.
  SET TELEM_AT_A_UP_LIM TO 0.
  SET TELEM_AT_A_DN_LIM TO 0.
  SET TELEM_AT_KP_THR   TO 0.
  SET TELEM_AT_KI_SPD   TO 0.
  SET TELEM_AT_THR_SLEW TO 0.

  LOCAL initial_det IS 0.
  LOCAL max_det     IS 3.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_max_detent") {
    SET max_det TO MAX(0, ROUND(ACTIVE_AIRCRAFT["flaps_max_detent"], 0)).
  }
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("flaps_initial_detent") {
    SET initial_det TO CLAMP(ROUND(ACTIVE_AIRCRAFT["flaps_initial_detent"], 0), 0, max_det).
  }

  SET FLAPS_CURRENT_DETENT     TO initial_det.
  SET FLAPS_TARGET_DETENT      TO initial_det.
  SET FLAPS_LAST_STEP_UT       TO TIME:SECONDS.
  SET FLAPS_LAST_TARGET_LOGGED TO -1.

  SET FLIGHT_PLAN       TO LIST().
  SET FLIGHT_PLAN_INDEX TO 0.
  SET IFC_MANUAL_MODE   TO FALSE.

  SET DRAFT_PLAN      TO LIST().
  SET FMS_LEG_CURSOR  TO 0.
  SET FMS_EDIT_FIELD  TO 0.
  SET FMS_EDITING_LEG TO FALSE.

  // GUI editor handles — cleared here; window disposal is handled by
  // _GUI_CLOSE() in ifc_gui.ks before _IFC_INTERACTIVE_START calls INIT_STATE.
  SET GUI_WIN      TO 0.
  SET GUI_ADD_BTN  TO 0.
  SET GUI_ARM_BTN  TO 0.
  SET GUI_QUIT_BTN TO 0.
  GUI_LEG_SEL_BTNS:CLEAR().
  GUI_LEG_UP_BTNS:CLEAR().
  GUI_LEG_DN_BTNS:CLEAR().
  GUI_LEG_DEL_BTNS:CLEAR().
  GUI_LEG_SUM_LBLS:CLEAR().
  GUI_FIELD_LBLS:CLEAR().
  GUI_FIELD_DEC_BTNS:CLEAR().
  GUI_FIELD_INC_BTNS:CLEAR().
  GUI_SLOT_SAVE_BTNS:CLEAR().
  GUI_SLOT_LOAD_BTNS:CLEAR().
  SET GUI_PLAN_LBL     TO 0.
  SET GUI_EDIT_HDR_LBL TO 0.
  GUI_FIELD_NAME_LBLS:CLEAR().
  SET GUI_EDIT_WIN      TO 0.
  GUI_EDIT_HANDLES:CLEAR().
  SET GUI_EDIT_LEG_TYPE TO -1.
  SET GUI_EDIT_NAV_TYPE TO "".

  SET CRUISE_WAYPOINTS  TO LIST().
  SET CRUISE_WP_INDEX   TO 0.
  SET CRUISE_ALT_M      TO 3000.
  SET CRUISE_SPD_MPS    TO CRUISE_DEFAULT_SPD.
  SET CRUISE_DEST_PLATE TO 0.
  SET CRUISE_NAV_TYPE   TO "waypoint".
  SET CRUISE_COURSE_DEG TO 0.
  SET CRUISE_END_UT     TO -1.

  SET APP_SPOILERS_ARMED  TO FALSE.
  SET TO_RWY_HDG          TO 90.
  SET TO_ROTATE_PITCH_CMD TO 0.
  SET TO_AIRBORNE_UT      TO -1.
  SET TO_STAGE_ATTEMPTS   TO 0.
  SET TO_LAST_STAGE_UT    TO -9999.
  SET TO_YAW_CMD_PREV     TO 0.
  SET TO_PITCH_CMD_PREV   TO 0.
  SET TO_HDG_PREV         TO GET_COMPASS_HDG().
  SET TO_CLIMB_FPA_CMD    TO 0.
  SET TO_SPOOL_PREV_AVAIL TO SHIP:AVAILABLETHRUST.
  SET TO_SPOOL_STABLE_UT  TO -1.
  SET TO_START_POS        TO -SHIP:BODY:POSITION.

  // Ascent guidance state
  SET ASC_INITIALIZED    TO FALSE.
  SET ASC_HDG_REF        TO 90.0.
  SET ASC_SURF_FPA_FILT  TO 0.0.
  SET ASC_HDG_CMD_FILT   TO 90.0.
  ASC_AB_ENGINES:CLEAR().
  ASC_RK_ENGINES:CLEAR().
  SET ASC_LF_MAX         TO 1.0.
  SET ASC_OX_MAX         TO 1.0.
  SET ASC_ESTIMATOR_VALIDITY TO ASC_VALID.
  SET ASC_STAR_V         TO V(1,0,0).
  SET ASC_TOP_V          TO V(0,1,0).
  SET ASC_NFWD_V         TO V(0,0,-1).
  SET ASC_AERO_ALONG_CTRL TO 0.
  SET ASC_Q_CTRL         TO 0.
  SET ASC_AOA_CTRL       TO 0.
  SET ASC_AERO_ALONG_VAL TO 0.
  SET ASC_Q_VAL          TO 0.
  SET ASC_EDOT_VAL       TO 0.
  SET ASC_APO_VAL        TO MAX(SHIP:ORBIT:APOAPSIS, 0).
  SET ASC_MDOT_AB_VAL    TO 0.
  SET ASC_MDOT_RK_VAL    TO 0.
  SET ASC_DRAG_RK_VAL    TO 0.
  SET ASC_DRAG_DDOT      TO 0.
  SET ASC_DRAG_PREV      TO 0.
  SET ASC_ENERGY_PREV    TO 0.
  SET ASC_EDOT_ORBITAL   TO 0.
  SET ASC_J_AB           TO 0.
  SET ASC_J_RK           TO 0.
  SET ASC_AOA_MAX_WIN    TO GET_AOA().
  SET ASC_AOA_MIN_WIN    TO GET_AOA().
  SET ASC_AOA_WIN_UT     TO TIME:SECONDS.
  SET ASC_APO_PREV       TO MAX(SHIP:ORBIT:APOAPSIS, 0).
  SET ASC_APO_PREV_UT    TO TIME:SECONDS.
  SET ASC_PERSIST_UT     TO -1.
  SET ASC_STEER_BLEND    TO 0.
  SET ASC_BLEND_START_UT TO -1.
  SET ASC_PITCH_BIAS     TO 0.
  SET ASC_IS_SPOOLING    TO FALSE.
  SET ASC_FLAMEOUT       TO FALSE.
  SET ASC_AB_THR_RATIO   TO 1.
  SET ASC_REGIME_MACH_CACHED TO ASC_REGIME_MACH_DEFAULT.
  SET ASC_Q_TARGET_CACHED         TO ASC_Q_TARGET_DEFAULT.
  SET ASC_Q_MAX_CACHED            TO ASC_Q_MAX_DEFAULT.
  SET ASC_Q_MIN_CACHED            TO ASC_Q_MIN_DEFAULT.
  SET ASC_HEAT_LIMIT_CACHED       TO 3.5e8.
  SET ASC_ZOOM_APO_TARGET_CACHED  TO ASC_APO_ZOOM_TARGET_DEFAULT.
  SET ASC_FINAL_APO_TARGET_CACHED TO ASC_APO_FINAL_DEFAULT.
  SET ASC_AOA_LIMIT_CACHED        TO AA_MAX_AOA.
  SET ASC_CORRIDOR_FPA_MAX_CACHED TO 25.0.
  SET ASC_DUALMODE_RK_ISP_CACHED  TO ASC_DUALMODE_RK_ISP_DEFAULT.
  SET ASC_DUALMODE_RK_THR_CACHED  TO ASC_DUALMODE_RK_THR_FRAC_DEFAULT.
  SET ASC_K_PROP_CACHED           TO ASC_K_PROP_DEFAULT.
  SET ASC_PRESSURE_CACHED         TO 0.
  ASC_RK_DUALMODE:CLEAR().
  SET ASC_CEIL_PERSIST_UT         TO -1.
  SET TELEM_ASC_J_AB     TO 0.
  SET TELEM_ASC_J_RK     TO 0.
  SET TELEM_ASC_VALIDITY TO ASC_VALID.
  SET TELEM_ASC_Q        TO 0.
  SET TELEM_ASC_Q_RAW    TO 0.
  SET TELEM_ASC_MACH     TO 0.
  SET TELEM_ASC_APO      TO 0.
  SET TELEM_ASC_DRAG_RK  TO 0.
  SET TELEM_ASC_W_PROP   TO 1.
  SET TELEM_ASC_EDOT_VAL TO 0.
  SET TELEM_ASC_EDOT_ORB TO 0.
  SET TELEM_ASC_PITCH_BIAS TO 0.
  SET TELEM_ASC_BLEND    TO 0.
  SET TELEM_ASC_SPOOLING TO FALSE.
  SET TELEM_ASC_AB_THR_RATIO TO 0.
  SET TELEM_ASC_AB_T_NOW     TO 0.
  SET TELEM_ASC_AB_T_AVAIL   TO 0.
  SET TELEM_ASC_AB_IGN_ON    TO 0.
  SET TELEM_ASC_AB_FLAMEOUTS TO 0.
  SET TELEM_ASC_RK_T_NOW     TO 0.
  SET TELEM_ASC_RK_T_AVAIL   TO 0.
  SET TELEM_ASC_RK_IGN_ON    TO 0.
  SET TELEM_ASC_RK_FLAMEOUTS TO 0.
}

// Called once after ACTIVE_PLATE and ACTIVE_AIRCRAFT are set.
FUNCTION IFC_LOAD_PLATE {
  SET ACTIVE_ILS_ID   TO ACTIVE_PLATE["ils_id"].
  LOCAL ils_bcn IS GET_BEACON(ACTIVE_ILS_ID).
  SET ACTIVE_RWY_HDG  TO ils_bcn["hdg"].
  SET ACTIVE_GS_ANGLE TO ils_bcn["gs_angle"].
  SET ACTIVE_THR_ALT  TO ils_bcn["alt_asl"].
  SET ILS_GS_TAN_CACHED TO TAN(ACTIVE_GS_ANGLE).
  SET ILS_THR_GEO_LL    TO ils_bcn["ll"].
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("v_app") {
    SET ACTIVE_V_APP  TO ACTIVE_AIRCRAFT["v_app"].
  } ELSE {
    SET ACTIVE_V_APP  TO ACTIVE_PLATE["vapp"].
  }
  SET ACTIVE_V_TGT TO ACTIVE_V_APP.
  SET APP_ON_FINAL TO FALSE.
  SET APP_FINAL_ARM_UT TO -1.
  SET APP_SPD_MODE TO "FIXES".
  SET APP_VREF_TGT TO ACTIVE_V_APP - 10.
  SET APP_VINT_TGT TO ACTIVE_V_APP.
  SET APP_BASE_V_TGT TO ACTIVE_V_APP.
  SET APP_SHORT_FINAL_FRAC TO 0.
  SET APP_LOC_CAP_OK TO 0.
  SET APP_GS_CAP_OK TO 0.
  SET APP_GS_LATCHED TO FALSE.
  SET ACTIVE_FIXES    TO ACTIVE_PLATE["fixes"].
  SET ACTIVE_ALT_AT   TO ACTIVE_PLATE["alt_at"].
  SET FIX_INDEX TO 0.
  AT_RESET().
}
