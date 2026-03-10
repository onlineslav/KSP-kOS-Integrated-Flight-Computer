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
    "v_app",        200.0,
    "v_ref",        170.0,
    // Approach speed schedule shaping:
    // Vint = Vapp + clamp((Vapp - Vref) * gain, min_add, max_add)
    "app_spd_intercept_gain",    0.50,
    "app_spd_intercept_min_add", 8.0,
    "app_spd_intercept_max_add", 16.0,
    "app_short_final_agl",       120.0,
    "app_speed_tgt_slew_per_s",  1.3,

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
    "vfe_approach",      170, // m/s max IAS to extend approach flaps
    "vfe_landing",        160, // m/s max IAS to extend landing flaps
    "flaps_climb_km",      45, // km from threshold to allow climb detent
    "flaps_approach_km",  30, // km from threshold to deploy approach flaps
    "flaps_landing_km",    8, // km from threshold to deploy landing flaps

    // ── Gear ──────────────────────────────────────────────
    // AGL (m) at which to extend landing gear on approach.
    // Set to 0 to manage gear manually.
    "gear_down_agl", 300,

    // ── Rollout ────────────────────────────────────────────
    // Max IAS to allow wheel brakes during rollout.
    "rollout_brake_max_ias", 160,
    "rollout_yaw_assist_ias", 120,
    "rollout_roll_assist_ias", 100,

    // ── Flare ─────────────────────────────────────────────
    // Override the global constants for this specific aircraft.
    // Set to -1 to use the global default from ifc_constants.ks.
    "flare_agl",    70,   // m AGL to begin flare  (-1 = use FLARE_AGL_M)

    // ── Notes ─────────────────────────────────────────────
    // Free-text, shown at startup for crew awareness.
    "notes",        "X10-F Spaceplane"
  ).
}
