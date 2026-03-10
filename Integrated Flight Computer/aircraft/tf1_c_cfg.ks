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
    "name",         "TF1-C",     // shown in telemetry

    // ── Approach speeds (m/s IAS) ─────────────────────────
    // Vapp: target speed from FAF to flare.
    // Vref: threshold crossing speed (used for display/logging).
    // Reduce Vapp toward Vref during the flare by cutting throttle.
    "v_app",        76.0,
    "v_ref",        65.0,

    // ── Action groups ─────────────────────────────────────
    // Set to the action group NUMBER (1-10) that controls each
    // system, or 0 to disable.
    "ag_flaps_step_up",   9,  // FAR flap detent step-up action group
    "ag_flaps_step_down", 10,  // FAR flap detent step-down action group
    "ag_spoilers",       0,   // deploy spoilers/airbrakes on touchdown
    "ag_thrust_rev",     0,   // reverse thrust on touchdown

    // ── Flap detent schedule ───────────────────────────────
    // IFC tracks current/target detent and steps one notch at a time.
    // Distance chooses the desired detent; IAS/Vfe limits cap extension.
    "flaps_initial_detent", 0, // estimated detent when IFC starts
    "flaps_detent_up",      0, // fully retracted
    "flaps_detent_climb",   1, // climb / maneuver detent
    "flaps_detent_approach",1, // takeoff/descent detent
    "flaps_detent_landing", 1, // full landing detent
    "flaps_max_detent",     1, // highest valid detent index
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
    "rollout_brake_max_ias", 30, // m/s max IAS for wheel brakes on rollout
    "rollout_yaw_assist_ias", 95,
    "rollout_roll_assist_ias", 90,
    "rollout_steer_min_blend", 0.25,
    "rollout_yaw_sign", -1,

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",    18,   // m AGL to begin flare  (-1 = use FLARE_AGL_M)
    "flare_touchdown_vs",      -0.15,
    "flare_ias_to_vs_gain",     0.01,
    "flare_balloon_vs_trigger", 0.05,
    "flare_balloon_fpa_push",   1.2,
    "flare_pitch_rate_min",     0.8,
    "flare_pitch_rate_max",     2.0,

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "Note for TF1-C"
  ).
}
