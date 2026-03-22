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
    "name",         "TF2-B",     // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        73.0,
    "v_ref",        69.0,
    // Approach speed schedule shaping:
    // Vint = Vapp + clamp((Vapp - Vref) * gain, min_add, max_add)
    "app_spd_intercept_gain",    0.60,
    "app_spd_intercept_min_add", 4.0,
    "app_spd_intercept_max_add", 8.0,
    "app_short_final_agl",       45.0,
    "app_speed_tgt_slew_per_s",  0.9,

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   9,  // FAR flap detent step-up action group
    "ag_flaps_step_down", 10,  // FAR flap detent step-down action group
    "ag_spoilers",       8,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     0,   // reverse thrust on touchdown (0 = not equipped)
    "ag_drogue",         0,   // deploy drogue chute on touchdown (0 = not equipped)

    // ── Flap detent schedule ───────────────────────────────
    // IFC tracks current/target detent and steps one notch at a time.
    // Distance chooses the desired detent; IAS/Vfe limits cap extension.
    "flaps_initial_detent", 0, // estimated detent when IFC starts
    "flaps_detent_up",      0, // fully retracted
    "flaps_detent_climb",   1, // climb / maneuver detent
    "flaps_detent_approach",1, // takeoff/descent detent
    "flaps_detent_landing", 1, // full landing detent
    "flaps_max_detent",     1, // highest valid detent index
    "vfe_climb",          120, // m/s max IAS for climb detent
    "vfe_approach",      110, // m/s max IAS to extend approach flaps
    "vfe_landing",        95, // m/s max IAS to extend landing flaps
    "flaps_climb_km",      45, // km from threshold to allow climb detent
    "flaps_approach_km",  30, // km from threshold to deploy approach flaps
    "flaps_landing_km",    8, // km from threshold to deploy landing flaps

    // ── Gear ──────────────────────────────────────────────
    // AGL (m) at which to extend landing gear on approach.
    // Set to 0 to manage gear manually.
    "gear_down_agl", 300,

    // ── Rollout ───────────────────────────────────────────
    // Controls ground roll behaviour after touchdown.
    // All IAS values in m/s.
    // For rollout_roll_assist_ias specifically: 0 disables IFC roll assist.
    "rollout_brake_max_ias",    60,   // only apply wheel brakes below this IAS
                                      // (high-speed braking causes instability)
    "rollout_yaw_assist_ias",   70,   // begin rudder heading-hold below this IAS
    "rollout_yaw_kp",         0.025,  // rollout rudder gain (deg heading error -> yaw cmd)
    "rollout_yaw_slew_per_s", 1.5,    // max yaw command rate change per second
    "rollout_yaw_fade_ias",   22,     // fade rudder assist down below this IAS
    "rollout_yaw_max_cmd",    0.35,   // clamp max rudder command magnitude
    "rollout_roll_assist_ias",  0,    // begin aileron wings-level assist below this IAS (0 = AA FBW handles roll)
    "rollout_steer_min_blend",  0.08, // minimum wheelsteering blend toward runway heading
                                      // (0 = pure touchdown heading, 1 = immediate runway heading)
    "rollout_nose_hold_cmd",    0.12, // moderate aft-stick hold to resist touchdown nose-drop impulse
    "rollout_nose_release_ias", 24,   // only start releasing nose hold after more deceleration
    "rollout_nose_hold_min_s",  1.8,  // keep full nose-hold through the initial touchdown transient
    "rollout_nose_min_ref_deg", 3.0,  // minimum touchdown pitch reference to hold after mains contact
    "rollout_nose_target_pitch_deg", 0.0, // final rollout pitch target once nose lowering is allowed
    "rollout_nose_target_slew_dps", 0.55, // deg/s max rate for lowering pitch target
    "rollout_pitch_hold_kp",    0.09, // closed-loop pitch hold gain (cmd per deg error)
    "rollout_pitch_max_cmd",    0.38, // clamp for closed-loop pitch hold command
    "rollout_pitch_slew_per_s", 2.0,  // allow responsive but not aggressive touchdown correction
    "rollout_touchdown_settle_s", 0.45, // keep TOUCHDOWN phase longer before entering rollout
    "rollout_yaw_sign",        -1,    // +1 or -1: flip if rudder corrects the wrong way
                                      // (depends on aircraft control axis convention)

    // ── Stall / AoA limits ────────────────────────────────
    "vs0",      54.0,   // m/s  stall speed in landing configuration
    "a_crit",   18.0,   // deg  critical AoA from FAR data (0 = protection disabled)

    // ── Spoiler arming (approach) ─────────────────────────
    "ag_spoilers_arm",   0,    // AG to arm spoilers in-flight (0 = not used)
    "app_spoiler_arm_km", 0,   // km from threshold to arm (0 = disabled)

    // ── Takeoff ───────────────────────────────────────────
    "has_nws",            TRUE,  // TRUE = aircraft has nose wheel steering
    "v_r",                75.0,  // m/s  rotate speed
    "v2",                 85.0,  // m/s  V2 climb speed
    "takeoff_pitch_tgt",  7.0,  // deg  pitch target at rotation
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
    "flare_agl",               35,   // m AGL to trigger flare
    "flare_touchdown_vs",      -0.05,// m/s target sink rate at wheel contact (neg = descending)
    "flare_ias_to_vs_gain",     0.006,// extra sink commanded per m/s above Vref (energy bleed)
    "flare_roundout_agl",       5.0, // m AGL where final roundout blend begins
    "flare_roundout_strength",  1.0, // 0..1 blend strength toward touchdown VS in roundout zone
    "flare_balloon_vs_trigger", 0.05,// m/s: if VS rises above this during flare, push nose down
    "flare_balloon_fpa_push",   1.2, // deg extra nose-down FPA added when balloon recovery fires
    "flare_pitch_rate_min",     1.2, // deg/s slowest FPA change rate (at low IAS)
    "flare_pitch_rate_max",     2.8, // deg/s fastest FPA change rate (at high IAS)

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
    "notes",        "Note for TF1-D"
  ).
}
