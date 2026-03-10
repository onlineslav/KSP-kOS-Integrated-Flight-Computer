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

    // ── Rollout ───────────────────────────────────────────
    // Controls ground roll behaviour after touchdown.
    // All IAS values in m/s.  Set a key to 0 (or omit) to use
    // the global defaults from ifc_constants.ks.
    "rollout_brake_max_ias",    30,   // only apply wheel brakes below this IAS
                                      // (high-speed braking causes instability)
    "rollout_yaw_assist_ias",   95,   // begin rudder heading-hold below this IAS
    "rollout_roll_assist_ias",  90,   // begin aileron wings-level assist below this IAS
    "rollout_steer_min_blend",  0.25, // minimum wheelsteering blend toward runway heading
                                      // (0 = pure touchdown heading, 1 = immediate runway heading)
    "rollout_yaw_sign",        -1,    // +1 or -1: flip if rudder corrects the wrong way
                                      // (depends on aircraft control axis convention)

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",               18,   // m AGL to trigger flare
    "flare_touchdown_vs",      -0.1, // m/s target sink rate at wheel contact (neg = descending)
    "flare_ias_to_vs_gain",     0.01,// extra sink commanded per m/s above Vref (energy bleed)
    "flare_balloon_vs_trigger", 0.05,// m/s: if VS rises above this during flare, push nose down
    "flare_balloon_fpa_push",   1.2, // deg extra nose-down FPA added when balloon recovery fires
    "flare_pitch_rate_min",     0.8, // deg/s slowest FPA change rate (at low IAS)
    "flare_pitch_rate_max",     2.0, // deg/s fastest FPA change rate (at high IAS)

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "Note for TF1-C"
  ).
}
