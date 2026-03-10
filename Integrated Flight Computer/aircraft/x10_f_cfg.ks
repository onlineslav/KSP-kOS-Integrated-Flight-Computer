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
    "name",         "X10-F Bigboi",           // shown in telemetry

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

    // ── AA Moderators ─────────────────────────────────────
    // Per-aircraft overrides for AtmosphereAutopilot FBW limits.
    // Set to -1 to use the global default from ifc_constants.ks.
    "aa_max_aoa",      20,   // deg max AoA  (global: 12)
    "aa_max_g",        -1,   // G   max G    (global: 3.5)
    "aa_max_sideslip", -1,   // deg max sideslip (global: 5)
    "aa_max_side_g",   -1,   // G   max lateral G (global: 1.5)

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

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "X10-F Spaceplane"
  ).
}
