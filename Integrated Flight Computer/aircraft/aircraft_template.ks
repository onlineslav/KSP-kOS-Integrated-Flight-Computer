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
    "name",         "My Aircraft",     // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        75.0,
    "v_ref",        65.0,

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   0,  // FAR flap detent step-up action group
    "ag_flaps_step_down", 0,  // FAR flap detent step-down action group
    "ag_spoilers",       0,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     0,   // reverse thrust on touchdown
    "ag_drogue",         0,   // deploy drogue chute on touchdown (0 = not equipped)

    // ── Flap detent schedule ───────────────────────────────
    // IFC tracks current/target detent and steps one notch at a time.
    // Distance chooses the desired detent; IAS/Vfe limits cap extension.
    "flaps_initial_detent", 0, // estimated detent when IFC starts
    "flaps_detent_up",      0, // fully retracted
    "flaps_detent_climb",   1, // climb / maneuver detent
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
    "rollout_pitch_slew_per_s", -1, // -1 = use global

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",    -1,   // m AGL to begin flare  (-1 = use FLARE_AGL_M)
    "flare_touchdown_vs",      -1, // m/s  (-1 = use TOUCHDOWN_VS)
    "flare_ias_to_vs_gain",    -1, // sink per m/s above Vref (-1 = global)
    "flare_roundout_agl",      -1, // m AGL roundout zone (-1 = global)
    "flare_roundout_strength", -1, // 0..1 blend in roundout zone (-1 = global)
    "flare_balloon_vs_trigger",-1, // m/s trigger balloon recovery (-1 = global)
    "flare_balloon_fpa_push",  -1, // deg extra nose-down in recovery (-1 = global)
    "flare_pitch_rate_min",    -1, // deg/s (-1 = global)
    "flare_pitch_rate_max",    -1, // deg/s (-1 = global)

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "Template - fill in before use."
  ).
}
