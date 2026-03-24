@LAZYGLOBAL OFF.

// ============================================================
// aircraft_template.ks  -  Integrated Flight Computer
//
// Copy this file, rename it for your aircraft (e.g. "x10d.ks"),
// fill in the values below, then pass it to IFC_MAIN like so:
//
//   RUNONCEPATH("0:/Integrated Flight Computer/aircraft/x10d.ks").
//   SET ACTIVE_AIRCRAFT TO BUILD_AIRCRAFT_CONFIG().
//
// The aircraft config lexicon is then read by the IFC to set
// approach speeds and action group assignments.
// ============================================================

// notes from test flight:
// stall speed no flaps 72m/s

FUNCTION BUILD_AIRCRAFT_CONFIG {
  RETURN LEXICON(

    // ── Identity ──────────────────────────────────────────
    "name",         "XF1-A",           // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        75.0,
    "v_ref",        65.0,
    // Approach speed schedule shaping:
    // Vint = Vapp + clamp((Vapp - Vref) * gain, min_add, max_add)
    "app_spd_intercept_gain",    0.60,
    "app_spd_intercept_min_add", 4.0,
    "app_spd_intercept_max_add", 8.0,
    "app_short_final_agl",       50.0,
    "app_speed_tgt_slew_per_s",  0.9,

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   9,  // FAR step-up detent (keyboard 9)
    "ag_flaps_step_down", 10, // FAR step-down detent (keyboard 0 / AG10)
    "ag_spoilers",       0,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     0,   // reverse thrust on touchdown

    // ── Flap detent schedule ───────────────────────────────
    // 4 detents: 0 up, 1 climb, 2 takeoff/descent, 3 landing.
    // stall no-flap ~72 m/s - tune Vfe values from test data.
    "flaps_initial_detent", 0, // expected detent at IFC engage
    "flaps_detent_up",      0,
    "flaps_detent_climb",   1,
    "flaps_detent_approach",2,
    "flaps_detent_landing", 3,
    "flaps_max_detent",     3,
    "vfe_climb",          150, // m/s max IAS for climb detent
    "vfe_approach",      120, // m/s max IAS to extend approach flaps
    "vfe_landing",        95, // m/s max IAS to extend landing flaps
    "flaps_climb_km",      45, // km from threshold to allow climb detent
    "flaps_approach_km",  30, // km from threshold to deploy approach flaps
    "flaps_landing_km",    8, // km from threshold to deploy landing flaps

    // ── Gear ──────────────────────────────────────────────
    // AGL (m) at which to extend landing gear on approach.
    // Set to 0 to manage gear manually.
    
    "gear_down_agl", 300,
    "gear_max_extend_ias", 120, // m/s max IAS to auto-extend gear

    // ── Rollout ────────────────────────────────────────────
    // Max IAS to allow wheel brakes during rollout.
    "rollout_brake_max_ias", 65,
    "rollout_yaw_assist_ias", 95,
    "rollout_roll_assist_ias", 95,

    // -- Stall / AoA limits --------------------------------
    "vs0",      50.0,   // m/s  stall speed in landing configuration
    "a_crit",    0.0,   // deg  FAR critical AoA (0 = protection disabled until tuned)

    // -- Spoiler arming (approach) -------------------------
    "ag_spoilers_arm",    0,   // AG to arm spoilers in-flight (0 = not used)
    "app_spoiler_arm_km", 0,   // km from threshold to arm (0 = disabled)

    // -- Takeoff --------------------------------------------
    "has_nws",            TRUE,  // TRUE = aircraft has nose wheel steering
    "v_r",                60.0,  // m/s  rotate speed
    "v2",                 70.0,  // m/s  V2 climb speed
    "takeoff_pitch_tgt",  10.0,  // deg  pitch target at rotation
    "takeoff_climb_fpa",   8.0,  // deg  climb FPA for climb-out
    "takeoff_throttle",    1.0,  // 0..1 takeoff throttle setting
    "takeoff_done_agl",  300.0,  // m AGL to end takeoff phase

    // ── AA Moderators ─────────────────────────────────────
    // Per-aircraft overrides for AtmosphereAutopilot FBW limits.
    // Set to -1 to use the global default from ifc_constants.ks.
    "aa_max_aoa",      -1,   // deg max AoA  (global: 12)
    "aa_max_g",        -1,   // G   max G    (global: 3.5)
    "aa_max_sideslip", -1,   // deg max sideslip (global: 5)
    "aa_max_side_g",   -1,   // G   max lateral G (global: 1.5)
    "aa_max_bank",     -1,   // deg max bank (global: 35; -1 = use default)

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",    -1,   // m AGL to begin flare  (-1 = use FLARE_AGL_M)
    "flare_touchdown_vs",      -1, // m/s  (-1 = use TOUCHDOWN_VS)
    "flare_vs_kp",               -1, // gamma correction per m/s sink-rate error (-1 = global)
    "flare_fpa_kp",              -1, // gamma correction per deg FPA error (-1 = global)
    "flare_cmd_fpa_min",         -1, // deg lower gamma clamp in flare (-1 = global)
    "flare_cmd_fpa_max",         -1, // deg upper gamma clamp in flare (-1 = global)
    "flare_cmd_rate_min_dps",    -1, // deg/s low-speed gamma slew floor (-1 = global)
    "flare_cmd_rate_max_dps",    -1, // deg/s high-speed gamma slew ceiling (-1 = global)
    "flare_roundout_start_h_m",  -1, // m runway-relative height where roundout starts (-1 = global)
    "flare_roundout_end_h_m",    -1, // m runway-relative height where roundout is full (-1 = global)
    "flare_roundout_curve",      -1, // 0 disables roundout; >0 scales roundout blend (-1 = global)
    "flare_min_throttle",        -1, // 0..1 flare throttle floor before blend/recovery (-1 = global)
    "flare_min_throttle_agl_blend",-1, // m below this height throttle floor blends down (-1 = global)
    "flare_authority_vs_err_trigger",-1, // m/s authority-limited trigger on VS error (-1 = global)
    "flare_authority_pitch_err_trigger",-1, // deg authority-limited trigger on pitch error (-1 = global)
    "flare_authority_fpa_err_trigger",-1, // deg authority-limited trigger on FPA error (-1 = global)
    "flare_authority_detect_s",  -1, // s persistence before authority-limited latch (-1 = global)
    "flare_authority_recovery_gain",-1, // 0..1 recovery gain for throttle/roundout recovery (-1 = global)
    "flare_disable_speed_bleed", -1, // 1 disable speed-driven extra sink, 0 enable, -1 = global default
    "touchdown_confirm_s",       -1, // s debounce for FLARE->TOUCHDOWN (-1 = global)
    "touchdown_confirm_max_abs_vs",-1, // m/s max |VS| at FLARE->TOUCHDOWN commit (-1 = global)

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
    "notes",        "Template - fill in before use."
  ).
}

