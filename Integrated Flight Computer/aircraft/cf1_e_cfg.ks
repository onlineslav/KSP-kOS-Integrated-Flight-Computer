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

FUNCTION BUILD_AIRCRAFT_CONFIG {
  RETURN LEXICON(

    // ── Identity ──────────────────────────────────────────
    "name",         "CF1-E Kerbmaster",     // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        76.0,
    "v_ref",        70.0,
    // Optional approach-speed schedule shaping (set to -1 to use globals):
    // Intercept target is derived as:
    //   Vint = Vapp + clamp((Vapp - Vref) * gain, min_add, max_add)
    "app_spd_intercept_gain",    -1, // -1 = use APP_SPD_INTERCEPT_GAIN
    "app_spd_intercept_min_add", -1, // m/s, -1 = use APP_SPD_INTERCEPT_MIN_ADD
    "app_spd_intercept_max_add", -1, // m/s, -1 = use APP_SPD_INTERCEPT_MAX_ADD
    "app_short_final_agl",       -1, // m AGL where Vapp blends toward Vref (-1 = global)
    "app_speed_tgt_slew_per_s",  -1, // m/s/s speed target slew limit (-1 = global)
    "app_short_final_cap",       -1, // 1=cap speed to short-final schedule even without final capture, 0=disable, -1=global

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   9,  // FAR flap detent step-up action group
    "ag_flaps_step_down", 10,  // FAR flap detent step-down action group
    "ag_spoilers",       7,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     8,   // reverse thrust on touchdown
    "ag_drogue",         0,   // deploy drogue chute on touchdown (0 = not equipped)

    // ── Flap detent schedule ───────────────────────────────
    // IFC tracks current/target detent and steps one notch at a time.
    // Distance chooses the desired detent; IAS/Vfe limits cap extension.
    "flaps_initial_detent", 0, // estimated detent when IFC starts
    "flaps_detent_up",      0, // fully retracted
    "flaps_detent_climb",   1, // climb / maneuver detent
    "flaps_detent_takeoff", 1, // takeoff flap detent (-1 = use flaps_detent_approach)
    "flaps_detent_approach",2, // takeoff/descent detent
    "flaps_detent_landing", 3, // full landing detent
    "flaps_max_detent",     3, // highest valid detent index
    "vfe_climb",          160, // m/s max IAS for climb detent
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

    // ── Stall / AoA limits ────────────────────────────────
    "vs0",      55.0,   // m/s  stall speed in landing configuration
    "a_crit",    0.0,   // deg  critical AoA from FAR data (0 = protection disabled)

    // ── Spoiler arming (approach) ─────────────────────────
    "ag_spoilers_arm",    0,  // AG to arm spoilers in-flight (0 = not used)
    "app_spoiler_arm_km", 0,  // km from threshold to arm (0 = disabled)

    // ── Takeoff ───────────────────────────────────────────
    "has_nws",            TRUE,  // TRUE = aircraft has nose wheel steering
    "v_r",                70.0,  // m/s  rotate speed
    "v2",                 80.0,  // m/s  V2 climb speed
    "takeoff_pitch_tgt",  10.0,  // deg  pitch target at rotation
    "takeoff_pitch_slew_dps",     -1, // deg/s rotate-target pitch slew (-1 = global)
    "takeoff_rotate_pitch_kp",    -1, // pitch cmd per deg pitch error while on wheels (-1 = global)
    "takeoff_rotate_pitch_ff",    -1, // baseline back-pressure during rotation (-1 = global)
    "takeoff_rotate_pitch_min_cmd",-1, // minimum nose-up pitch cmd while rotating (-1 = global)
    "takeoff_rotate_pitch_max_cmd",-1, // max pitch cmd while on wheels (-1 = global)
    "takeoff_rotate_pitch_slew_per_s",-1, // pitch cmd slew while on wheels (-1 = global)
    "takeoff_climb_fpa",   8.0,  // deg  climb FPA for climb-out
    "takeoff_throttle",    1.0,  // 0..1 takeoff throttle setting
    "takeoff_done_agl",  300.0,  // m AGL to end takeoff phase
    "takeoff_airborne_agl",   -1, // m AGL threshold for airborne detect (-1 = global)
    "takeoff_airborne_min_vs",-1, // m/s min VS for airborne detect (-1 = global)
    "takeoff_autostage",      -1, // 1=auto STAGE attempts if no thrust (-1 = global)
    "takeoff_stage_max_attempts",-1, // max autostage attempts (-1 = global)
    "takeoff_stage_retry_s",  -1, // s between autostage attempts (-1 = global)
    "takeoff_engine_spool_timeout_s",8, // s wait in preflight for thrust (-1 = global)
    "takeoff_spool_thrust_frac",  -1, // 0..1 fraction of max thrust required before brake release (-1 = global)
    "takeoff_spool_steady_dknps", -1, // kN/s |d(thrust)/dt| threshold for steady-state gate (-1 = global)
    "takeoff_spool_steady_hold_s",-1, // s thrust must remain steady before brake release (-1 = global)
    "takeoff_flap_settle_s",      -1, // s hold after final takeoff-flap step before brake release (-1 = global)
    "takeoff_min_avail_thrust",   -1, // kN considered "engines lit" (-1 = global)
    "takeoff_loc_kp",             -1, // deg/m centerline correction gain (-1 = global)
    "takeoff_loc_guard_m",        -1, // m loc error clamp for steering (-1 = global)
    "takeoff_steer_max_corr",     -1, // deg max wheelsteering heading correction (-1 = global)
    "takeoff_steer_hdg_rate_kd",  -1, // steering heading-rate damping (-1 = global)
    "takeoff_dir_max_corr",       -1, // deg max director heading correction (-1 = global)
    "takeoff_yaw_start_ias",      -1, // m/s IAS where yaw assist starts (-1 = global)
    "takeoff_yaw_full_ias",       -1, // m/s IAS where yaw assist reaches full gain (-1 = global)
    "takeoff_yaw_min_scale",      -1, // 0..1 rudder authority floor from rollout start (-1 = global)
    "takeoff_yaw_kp",             -1, // rudder cmd per deg heading error (-1 = global)
    "takeoff_yaw_kd",             -1, // rudder cmd per deg/s heading-rate error (-1 = global)
    "takeoff_yaw_boost_err_deg",  -1, // heading error where yaw boost reaches +1x (-1 = global)
    "takeoff_yaw_boost_max",      -1, // cap extra yaw gain from heading-error boost (-1 = global)
    "takeoff_yaw_max_cmd",        -1, // max rudder cmd magnitude (-1 = global)
    "takeoff_yaw_slew_per_s",     -1, // rudder cmd slew rate (-1 = global)
    "takeoff_yaw_sign",           -1, // command sign for your control layout (+1 or -1)
    "takeoff_climb_min_throttle", -1, // throttle floor during climb (-1 = global)
    "takeoff_climb_spd_thr_gain", -1, // throttle trim gain vs (V2-IAS) (-1 = global)
    "takeoff_climb_fpa_spd_gain", -1, // deg FPA reduction per m/s below V2 (-1 = global)
    "takeoff_climb_fpa_min",      -1, // deg minimum climb FPA under speed protection (-1 = global)
    "takeoff_climb_fpa_slew_dps", -1, // deg/s FPA slew for rotate->climb handoff (-1 = global)
    "takeoff_aoa_protect_frac",   -1, // AoA protection threshold as fraction of a_crit (-1 = global)
    "takeoff_aoa_fpa_gain",       -1, // deg FPA pull-down per deg AoA above threshold (-1 = global)

    // ── AA Moderators ─────────────────────────────────────
    // Per-aircraft overrides for AtmosphereAutopilot FBW limits.
    // Set to -1 to use the global default from ifc_constants.ks.
    "aa_max_aoa",      15,   // deg max AoA  (global: 12)
    "aa_max_g",        5,   // G   max G    (global: 3.5)
    "aa_max_sideslip", -1,   // deg max sideslip (global: 5)
    "aa_max_side_g",   -1,   // G   max lateral G (global: 1.5)
    "aa_max_bank",     -1,   // deg max bank (global: 35; -1 = use default)

    // ── Rollout ────────────────────────────────────────────
    // Max IAS to allow wheel brakes during rollout.
    // Lower values reduce high-speed tip-over / swerve risk.
    "rollout_brake_max_ias", 70,
    // Speeds below which aerodynamic rollout assists begin to engage.
    // rollout_roll_assist_ias:
    // 95 = assist starts below 95 m/s, 0 = disable IFC roll assist.
    // With AA FBW active, keeping this at 0 is usually best.
    "rollout_yaw_assist_ias", 95,
    "rollout_yaw_kp", -1,         // -1 = use KP_ROLLOUT_YAW global constant
    "rollout_yaw_slew_per_s", -1, // -1 = use ROLLOUT_YAW_SLEW_PER_S global constant
    "rollout_yaw_fade_ias", -1,   // -1 = use ROLLOUT_YAW_FADE_IAS global constant
    "rollout_yaw_max_cmd", -1,    // -1 = use ROLLOUT_YAW_MAX_CMD global constant
    "rollout_roll_assist_ias", 95,
    // Minimum runway-heading/centerline steering blend at high IAS.
    // 0 = hold touchdown heading at very high speed, 1 = fully command runway heading.
    "rollout_steer_min_blend", -1, // (-1 = global)
    // Rudder assist direction on rollout:
    // -1 = default IFC sign, 1 = invert sign for this aircraft.
    "rollout_yaw_sign", -1,
    // Nose-wheel protection:
    // small pitch hold after touchdown, faded out by rollout_nose_release_ias.
    // If it pushes the nose the wrong way, invert the sign.
    "rollout_nose_hold_cmd", 0,    // 0 = disabled
    "rollout_nose_release_ias", -1, // -1 = use global
    "rollout_nose_hold_min_s", -1,  // -1 = use global
    "rollout_nose_min_ref_deg", -1, // -1 = use global
    "rollout_nose_target_pitch_deg", -1, // -1 = use global
    "rollout_nose_target_slew_dps", 3.0,  // deg/s (global 1.2 — faster nose-down)
    "rollout_pitch_hold_kp", -1,    // -1 = use global
    "rollout_pitch_max_cmd", -1,    // -1 = use global
    "rollout_pitch_max_down_cmd", 0.25, // (global 0.18 — more nose-down authority)
    "rollout_pitch_slew_per_s", 3.0, // cu/s (global 1.2 — keep up with faster target slew)
    "rollout_touchdown_settle_s", -1, // -1 = use global
    "bounce_recovery_agl_m", -1,    // -1 = use global
    "bounce_recovery_min_vs", -1,   // -1 = use global
    "bounce_recovery_confirm_s", -1, // -1 = use global
    "bounce_recovery_max_s", -1,    // -1 = use global

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",    -1,   // m AGL to begin flare  (-1 = use FLARE_AGL_M)
    "flare_touchdown_vs",      -1, // m/s  (-1 = use TOUCHDOWN_VS)
    "flare_vs_kp",               -1, // gamma correction per m/s sink-rate error (-1 = global)
    "flare_fpa_kp",              -1, // gamma correction per deg FPA error (-1 = global)
    "flare_cmd_fpa_min",         -1, // deg lower gamma clamp in flare (-1 = global)
    "flare_cmd_fpa_max",         -1, // deg upper gamma clamp in flare (-1 = global)
    "flare_roundout_end_h_m",    -1, // m runway-relative height where roundout is fully blended (-1 = global)
    "flare_min_throttle",        -1, // 0..1 flare throttle floor before blend/recovery (-1 = global)
    "flare_min_throttle_agl_blend",-1, // m below this height throttle floor blends down (-1 = global)
    "flare_authority_vs_err_trigger",-1, // m/s authority-limited trigger on VS error (-1 = global)
    "flare_authority_pitch_err_trigger",-1, // deg authority-limited trigger on pitch error (-1 = global)
    "flare_authority_fpa_err_trigger",-1, // deg authority-limited trigger on FPA error (-1 = global)
    "flare_authority_detect_s",  -1, // s persistence before authority-limited latch (-1 = global)
    "flare_authority_recovery_gain",-1, // 0..1 recovery gain for throttle/roundout recovery (-1 = global)
    "flare_disable_speed_bleed", -1, // sink per m/s above Vref (-1 = global)
    "flare_roundout_start_h_m",      -1, // m AGL roundout zone (-1 = global)
    "flare_roundout_curve", -1, // 0..1 blend in roundout zone (-1 = global)
    "flare_cmd_rate_min_dps",    -1, // deg/s (-1 = global)
    "flare_cmd_rate_max_dps",    -1, // deg/s (-1 = global)
    "touchdown_confirm_s", -1, // s debounce for FLARE->TOUCHDOWN (-1 = global)
    "touchdown_confirm_max_abs_vs", -1, // m/s max |VS| at FLARE->TOUCHDOWN commit (-1 = global)

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

