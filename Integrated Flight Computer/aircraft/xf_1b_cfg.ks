@LAZYGLOBAL OFF.

// ============================================================
// x11_c_cfg.ks  -  Integrated Flight Computer
// Aircraft config for the X11-C.
// ============================================================

// notes from test flight:
// (stall speed not yet determined — values below inherited from X10-G, needs tuning)

FUNCTION BUILD_AIRCRAFT_CONFIG {
  RETURN LEXICON(

    // ── Identity ──────────────────────────────────────────
    "name",         "XF-1",           // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        140.0,  // prev: 140,175,200  // we want ~10-15deg aoa (150 gives 10deg aoa) (140 gives 13.8deg aoa)
    "v_ref",        120.0,  // prev: 130,130,150,170
    // Approach speed schedule shaping:
    // Vint = Vapp + clamp((Vapp - Vref) * gain, min_add, max_add)
    "app_spd_intercept_gain",    0.45,
    "app_spd_intercept_min_add", 6.0,
    "app_spd_intercept_max_add", 12.0,
    "app_short_final_agl",       180.0,
    "app_speed_tgt_slew_per_s",  1.6,
    "app_short_final_cap",       1,    // cap to short-final schedule even if LOC/GS capture is noisy

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   9,  // FAR step-up detent (keyboard 9)
    "ag_flaps_step_down", 10, // FAR step-down detent (keyboard 0 / AG10)
    "ag_spoilers",       8,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     0,   // reverse thrust on touchdown
    "ag_drogue",         7,   // deploy drogue chute on touchdown (0 = not equipped)


    // ── Flap detent schedule ───────────────────────────────
    // 4 detents: 0 up, 1 climb, 2 takeoff/descent, 3 landing.
    // stall no-flap ~72 m/s - tune Vfe values from test data.
    "flaps_initial_detent", 0, // expected detent at IFC engage
    "flaps_detent_up",      0,
    "flaps_detent_climb",   1,
    "flaps_detent_takeoff", 2,
    "flaps_detent_approach",2,
    "flaps_detent_landing", 3,
    "flaps_max_detent",     3,
    "vfe_climb",          250, // m/s max IAS for climb detent
    "vfe_approach",      220, // m/s max IAS to extend approach flaps
    "vfe_landing",        210, // m/s max IAS to extend landing flaps
    "flaps_climb_km",      45, // km from threshold to allow climb detent
    "flaps_approach_km",  30, // km from threshold to deploy approach flaps
    "flaps_landing_km",    8, // km from threshold to deploy landing flaps

    // ── Gear ──────────────────────────────────────────────
    // AGL (m) at which to extend landing gear on approach.
    // Set to 0 to manage gear manually.
    "gear_down_agl", 300,

    // ── Rollout ────────────────────────────────────────────
    // Max IAS to allow wheel brakes during rollout.
    "rollout_brake_max_ias",    165,    // avoid high-speed wheel-brake instability
    "rollout_yaw_assist_ias",  210,    // above Vref so rudder assist is active from touchdown
    "rollout_yaw_kp",         0.018,
    "rollout_yaw_slew_per_s",  1.0,
    "rollout_yaw_fade_ias",     35,
    "rollout_yaw_max_cmd",    0.22,
    "rollout_roll_assist_ias",   0,    // let AA FBW own roll damping
    "rollout_steer_min_blend", 0.15,  // was 0.04; low value left aircraft with ~zero steering at high speed
    "rollout_yaw_sign",        -1,
    "rollout_touchdown_settle_s", 0.55,
    "bounce_recovery_agl_m",   3.5,
    "bounce_recovery_min_vs",  0.8,
    "bounce_recovery_confirm_s", 0.35,
    "bounce_recovery_max_s",   4.0,
    "rollout_nose_hold_cmd",   0.14,
    "rollout_nose_release_ias", 60,
    "rollout_nose_hold_min_s", 2.4,
    "rollout_nose_min_ref_deg", 4.0,
    "rollout_nose_target_pitch_deg", 0.8,
    "rollout_nose_target_slew_dps", 0.35,
    "rollout_pitch_hold_kp",   0.09,
    "rollout_pitch_max_cmd",   0.32,
    "rollout_pitch_max_down_cmd", 0.12,
    "rollout_pitch_slew_per_s", 1.4,

    // -- Stall / AoA limits --------------------------------
    "vs0",      92.0,   // m/s  stall speed in landing configuration
    "a_crit",    0.0,   // deg  FAR critical AoA (0 = protection disabled until tuned)

    // -- Spoiler arming (approach) -------------------------
    "ag_spoilers_arm",    0,   // AG to arm spoilers in-flight (0 = not used)
    "app_spoiler_arm_km", 0,   // km from threshold to arm (0 = disabled)

    // -- Takeoff --------------------------------------------
    "has_nws",            TRUE,  // TRUE = aircraft has nose wheel steering
    "v_r",                65.0,  // m/s  rotate speed
    "v2",                80.0,  // m/s  V2 climb speed
    "takeoff_pitch_tgt",  12.5,  // deg  pitch target at rotation
    "takeoff_pitch_slew_dps", 3.6, // deg/s rotate-target pitch slew
    "takeoff_rotate_pitch_kp", 0.13, // pitch cmd per deg pitch error while on wheels
    "takeoff_rotate_pitch_ff", 0.08, // baseline back-pressure during rotation
    "takeoff_rotate_pitch_min_cmd", 0.14, // minimum up-command while below rotate target
    "takeoff_rotate_pitch_max_cmd", 0.72, //PREV 0.42 // max pitch cmd while on wheels
    "takeoff_rotate_pitch_slew_per_s", 2.6, // pitch cmd slew while on wheels
    "takeoff_climb_fpa",   7.0,  // deg  climb FPA for climb-out
    "takeoff_throttle",    1.0,  // 0..1 takeoff throttle setting
    "takeoff_done_agl",  300.0,  // m AGL to end takeoff phase
    "takeoff_airborne_agl",  3.0, // m AGL threshold for airborne detect
    "takeoff_airborne_min_vs", 0.7, // m/s min VS for airborne detect
    "takeoff_autostage",      1,   // 1=auto STAGE attempts if no thrust
    "takeoff_stage_max_attempts", 1, // max autostage attempts
    "takeoff_stage_retry_s",   1.0, // s between autostage attempts
    "takeoff_engine_spool_timeout_s", 16.0, // s wait in preflight for thrust
    "takeoff_spool_thrust_frac", 0.995, // 0..1 fraction of max thrust required before brake release
    "takeoff_spool_steady_dknps", 1.0, // kN/s |d(available thrust)/dt| threshold for steady-state gate
    "takeoff_spool_steady_hold_s", 2.0, // s thrust must remain steady before brake release
    "takeoff_flap_settle_s", 8.0, // s hold after final takeoff-flap step before brake release
    "takeoff_min_avail_thrust", 5.0, // kN considered "engines lit"
    "takeoff_loc_kp",          0.050, // deg/m centerline correction gain
    "takeoff_loc_guard_m",   140.0, // m loc error clamp for steering
    "takeoff_steer_max_corr", 10.0, // deg max wheelsteering heading correction
    "takeoff_steer_hdg_rate_kd", 0.18, // steering heading-rate damping to reduce weave
    "takeoff_dir_max_corr",    6.0, // deg max director heading correction
    "takeoff_yaw_start_ias",   0.0, // m/s IAS where yaw assist starts
    "takeoff_yaw_full_ias",   45.0, // m/s IAS where yaw assist reaches full gain
    "takeoff_yaw_min_scale",   0.55, // 0..1 rudder authority floor from rollout start
    "takeoff_yaw_kp",          0.038, // rudder cmd per deg heading error
    "takeoff_yaw_kd",          0.018, // rudder cmd per deg/s heading-rate error (damping)
    "takeoff_yaw_boost_err_deg", 0.40, // heading error where yaw boost reaches +1x
    "takeoff_yaw_boost_max",   0.25, // cap extra yaw gain from heading-error boost
    "takeoff_yaw_max_cmd",     0.40, // max rudder cmd magnitude
    "takeoff_yaw_slew_per_s",  3.0, // rudder cmd slew rate
    "takeoff_yaw_sign",       -1,   // command sign for your control layout
    "takeoff_climb_min_throttle", 0.82, // throttle floor during climb
    "takeoff_climb_spd_thr_gain", 0.010, // throttle trim gain vs (V2-IAS)
    "takeoff_climb_fpa_spd_gain", 0.08, // deg FPA reduction per m/s below V2
    "takeoff_climb_fpa_min",   4.5, // deg minimum climb FPA under speed protection
    "takeoff_climb_fpa_slew_dps", 2.8, // deg/s FPA slew for rotate->climb handoff
    "takeoff_aoa_protect_frac", 0.85, // AoA protection threshold as fraction of a_crit
    "takeoff_aoa_fpa_gain", 0.90, // deg FPA pull-down per deg AoA above threshold

    // ── AA Moderators ─────────────────────────────────────
    // Per-aircraft overrides for AtmosphereAutopilot FBW limits.
    // Set to -1 to use the global default from ifc_constants.ks.
    "aa_max_aoa",      20,   // deg max AoA  (global: 12)
    "aa_max_g",        -1,   // G   max G    (global: 3.5)
    "aa_max_sideslip", -1,   // deg max sideslip (global: 5)
    "aa_max_side_g",   -1,   // G   max lateral G (global: 1.5)
    "aa_max_bank",     -1,   // deg max bank (global: 35; -1 = use default)

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",               130,  // m AGL to begin flare; was 85 but terrain rise near KSC causes ~60m effective trigger
    "flare_touchdown_vs",     -0.05, // m/s target sink rate at wheel contact
    "flare_ias_to_vs_gain",    0.010,// extra sink per m/s above Vref during flare
    "flare_roundout_agl",      8.0,  // m AGL final sink blend zone
    "flare_roundout_strength", 1.0,  // full roundout blend
    "flare_balloon_vs_trigger",0.08, // m/s VS threshold for anti-balloon push
    "flare_balloon_fpa_push",  1.4,  // deg nose-down bias when ballooning
    "flare_pitch_rate_min",    1.0,  // deg/s low-speed flare response
    "flare_pitch_rate_max",    3.0,  // deg/s high-speed flare response
    "touchdown_confirm_s",      0.16,
    "touchdown_confirm_max_abs_vs", 2.0,

    // ── Ascent guidance ───────────────────────────────────
    // All values fall back to ifc_constants.ks defaults when set to -1.
    // Tune these per-vehicle based on structural and engine characteristics.
    "ascent_q_target",          -1,  // Pa  corridor centre q              (-1 = 30000)
    "ascent_q_max",             -1,  // Pa  structural dynamic-press limit (-1 = 60000)
    "ascent_q_min",             -1,  // Pa  corridor lower bound           (-1 =  8000)
    "ascent_heat_limit",        -1,  // Pa·m/s  heating proxy limit        (-1 = 3.5e8)
    "ascent_k_prop",            -1,  // propellant equivalency coefficient (-1 = 0.5)
    "ascent_aoa_limit",         -1,  // deg  max AoA in ascent             (-1 = aa_max_aoa)
    "ascent_regime_mach",       -1,  // Mach  AB thermal regime boundary   (-1 = 4.5)
    "ascent_zoom_target_m",     -1,  // m   apoapsis target before switch  (-1 = 45000)
    "ascent_apoapsis_target_m", -1,  // m   target orbit apoapsis          (-1 = 80000)

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "X11-D"
  ).
}
