@LAZYGLOBAL OFF.

// ============================================================
// ifc_main.ks  -  Integrated Flight Computer
//
// Usage
// -----
//   1. Copy aircraft/aircraft_template.ks to aircraft/<myplane>.ks
//      and fill in the values.
//
//   2. In your vehicle script (or the kOS terminal), run:
//
//        RUNONCEPATH("0:/Integrated Flight Computer/ifc_main.ks").
//        RUN_IFC("09", FALSE).
//
//      Parameters:
//        rwy_id          "09" or "27"
//        short_approach  TRUE = start from 30 km fix
//                        FALSE = start from 60 km fix
//
//   3. The IFC will show the FMS pre-arm screen — use the menu
//      to select procedure/runway/distance then ARM.
//
// Aircraft config
// ---------------
//   Set ACTIVE_AIRCRAFT before calling RUN_IFC, e.g.:
//
//     RUNONCEPATH("0:/Integrated Flight Computer/aircraft/x10d.ks").
//     SET ACTIVE_AIRCRAFT TO BUILD_AIRCRAFT_CONFIG().
//
//   If ACTIVE_AIRCRAFT is not set the IFC uses built-in defaults.
// ============================================================

// ── Load libraries ────────────────────────────────────────
LOCAL ifc_root IS "0:/Integrated Flight Computer/".

RUNONCEPATH(ifc_root + "lib/ifc_constants.ks").
RUNONCEPATH(ifc_root + "lib/ifc_state.ks").
RUNONCEPATH(ifc_root + "lib/ifc_helpers.ks").
RUNONCEPATH(ifc_root + "lib/ifc_ui.ks").
RUNONCEPATH(ifc_root + "lib/ifc_menu.ks").
RUNONCEPATH(ifc_root + "lib/ifc_display.ks").
RUNONCEPATH(ifc_root + "lib/ifc_aa.ks").
RUNONCEPATH(ifc_root + "lib/ifc_logger.ks").
RUNONCEPATH(ifc_root + "nav/nav_math.ks").
RUNONCEPATH(ifc_root + "nav/nav_beacons.ks").
RUNONCEPATH(ifc_root + "phases/phase_approach.ks").
RUNONCEPATH(ifc_root + "phases/phase_autoland.ks").
RUNONCEPATH(ifc_root + "phases/phase_takeoff.ks").

// ── Default aircraft config (if not set externally) ───────
// Matches aircraft_template.ks structure; safe defaults only.
LOCAL _DEFAULT_AIRCRAFT IS LEXICON(
  "name",               "Unknown",
  "v_app",              75.0,
  "v_ref",              65.0,
  "app_spd_intercept_gain", -1,
  "app_spd_intercept_min_add", -1,
  "app_spd_intercept_max_add", -1,
  "app_short_final_agl", -1,
  "app_speed_tgt_slew_per_s", -1,
  "app_short_final_cap", -1,
  "vs0",                -1,
  "a_crit",             -1,
  "ag_flaps_step_up",   0,
  "ag_flaps_step_down", 0,
  "ag_spoilers",        0,
  "ag_spoilers_arm",    0,
  "app_spoiler_arm_km", 0,
  "ag_thrust_rev",      0,
  "ag_drogue",          0,
  "flaps_initial_detent", 0,
  "flaps_detent_up",      0,
  "flaps_detent_climb",   1,
  "flaps_detent_approach",2,
  "flaps_detent_landing", 3,
  "flaps_max_detent",     3,
  "vfe_climb",          160,
  "vfe_approach",       120,
  "vfe_landing",         95,
  "flaps_climb_km",      45,
  "flaps_approach_km",   30,
  "flaps_landing_km",     8,
  "gear_down_agl",      300,
  "rollout_brake_max_ias", 70,
  "rollout_yaw_assist_ias", 95,
  "rollout_roll_assist_ias", 95,
  "rollout_steer_min_blend", -1,
  "rollout_yaw_sign", -1,
  "rollout_touchdown_settle_s", -1,
  "bounce_recovery_agl_m", -1,
  "bounce_recovery_min_vs", -1,
  "bounce_recovery_confirm_s", -1,
  "bounce_recovery_max_s", -1,
  "rollout_nose_hold_cmd", -1,
  "rollout_nose_release_ias", -1,
  "rollout_nose_hold_min_s", -1,
  "rollout_nose_min_ref_deg", -1,
  "rollout_nose_target_pitch_deg", -1,
  "rollout_nose_target_slew_dps", -1,
  "rollout_pitch_hold_kp", -1,
  "rollout_pitch_max_cmd", -1,
  "rollout_pitch_max_down_cmd", -1,
  "rollout_pitch_slew_per_s", -1,
  "v_r",                -1,
  "v2",                 -1,
  "takeoff_pitch_tgt",  -1,
  "takeoff_pitch_slew_dps",-1,
  "takeoff_rotate_pitch_kp",-1,
  "takeoff_rotate_pitch_ff",-1,
  "takeoff_rotate_pitch_min_cmd",-1,
  "takeoff_rotate_pitch_max_cmd",-1,
  "takeoff_rotate_pitch_slew_per_s",-1,
  "takeoff_climb_fpa",  -1,
  "takeoff_throttle",   -1,
  "takeoff_done_agl",   -1,
  "takeoff_airborne_agl",-1,
  "takeoff_airborne_min_vs",-1,
  "takeoff_autostage",  -1,
  "takeoff_stage_max_attempts",-1,
  "takeoff_stage_retry_s",-1,
  "takeoff_engine_spool_timeout_s",-1,
  "takeoff_spool_thrust_frac",-1,
  "takeoff_spool_steady_dknps",-1,
  "takeoff_spool_steady_hold_s",-1,
  "takeoff_flap_settle_s",-1,
  "takeoff_min_avail_thrust",-1,
  "takeoff_loc_kp",     -1,
  "takeoff_loc_guard_m",-1,
  "takeoff_steer_max_corr",-1,
  "takeoff_steer_hdg_rate_kd",-1,
  "takeoff_dir_max_corr",-1,
  "takeoff_yaw_start_ias",-1,
  "takeoff_yaw_full_ias",-1,
  "takeoff_yaw_min_scale",-1,
  "takeoff_yaw_kp",     -1,
  "takeoff_yaw_kd",     -1,
  "takeoff_yaw_boost_err_deg",-1,
  "takeoff_yaw_boost_max",-1,
  "takeoff_yaw_max_cmd",-1,
  "takeoff_yaw_slew_per_s",-1,
  "takeoff_yaw_sign",   -1,
  "takeoff_climb_min_throttle",-1,
  "takeoff_climb_spd_thr_gain",-1,
  "takeoff_climb_fpa_spd_gain",-1,
  "takeoff_climb_fpa_min",-1,
  "takeoff_climb_fpa_slew_dps",-1,
  "aa_max_aoa",          -1,
  "aa_max_g",            -1,
  "aa_max_sideslip",     -1,
  "aa_max_side_g",       -1,
  "aa_max_bank",         -1,
  "flare_agl",           -1,
  "flare_touchdown_vs",  -1,
  "flare_ias_to_vs_gain",-1,
  "flare_roundout_agl",  -1,
  "flare_roundout_strength",-1,
  "flare_balloon_vs_trigger",-1,
  "flare_balloon_fpa_push",-1,
  "flare_pitch_rate_min",-1,
  "flare_pitch_rate_max",-1,
  "touchdown_confirm_s", -1,
  "touchdown_confirm_max_abs_vs", -1,
  "notes",              "Default config."
).

// ── Auto aircraft config loader ───────────────────────────
// Normalises SHIP:NAME to a filename, then looks for a matching
// config in aircraft/.  Returns TRUE if a config was loaded.
// Mapping example: "XF1-A" → "xf1_a_cfg.ks"
FUNCTION _TRY_LOAD_AIRCRAFT_CONFIG {
  LOCAL vessel_name_key IS SHIP:NAME:TOLOWER
    :REPLACE(" ", "_")
    :REPLACE("-", "_")
    :REPLACE(".", "_").
  LOCAL cfg IS ifc_root + "aircraft/" + vessel_name_key + "_cfg.ks".

  IF EXISTS(cfg) {
    RUNONCEPATH(cfg).
    SET ACTIVE_AIRCRAFT TO BUILD_AIRCRAFT_CONFIG().
    SET IFC_ALERT_TEXT TO "Loaded config: " + SHIP:NAME.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN TRUE.
  }

  SET IFC_ALERT_TEXT TO "No config for '" + SHIP:NAME + "' — defaults".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  RETURN FALSE.
}

// ── Interactive startup (FMS pre-arm screen) ──────────────
// Shows the pre-arm page, opens the FMS menu, and waits for
// the user to ARM or QUIT.  Dispatches to RUN_IFC / RUN_TAKEOFF_IFC.
FUNCTION _IFC_INTERACTIVE_START {
  SET IFC_MISSION_START_UT TO TIME:SECONDS.
  SET IFC_PHASE    TO PHASE_PREARM.
  SET IFC_SUBPHASE TO "".
  UI_INIT().

  IF ACTIVE_AIRCRAFT = 0 {
    _TRY_LOAD_AIRCRAFT_CONFIG().
    IF ACTIVE_AIRCRAFT = 0 { SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT. }
  }

  // Draw initial static content before entering the loop.
  DISPLAY_HEADER().
  DISPLAY_BREADCRUMB().
  DISPLAY_KEY_HINTS().
  DISPLAY_PREARM(IFC_MENU_OPT_PROC, IFC_MENU_OPT_RWY, IFC_MENU_OPT_DIST).
  DISPLAY_ALERT_BAR().

  // Open FMS menu immediately so the user can navigate.
  SET IFC_MENU_OPEN TO TRUE.
  MENU_RENDER().

  LOCAL result IS "".
  UNTIL result = "ARM" OR result = "QUIT" {
    SET result TO MENU_TICK().
    // Refresh pre-arm page whenever the menu is closed.
    IF NOT IFC_MENU_OPEN {
      DISPLAY_PREARM(IFC_MENU_OPT_PROC, IFC_MENU_OPT_RWY, IFC_MENU_OPT_DIST).
    }
    DISPLAY_TICK().
    WAIT 0.05.
  }

  IF result = "QUIT" { RETURN. }

  LOCAL rwy_str IS "09".
  IF IFC_MENU_OPT_RWY = 1 { SET rwy_str TO "27". }
  LOCAL short_app IS IFC_MENU_OPT_DIST = 1.

  IF IFC_MENU_OPT_PROC = 0 {
    RUN_IFC(rwy_str, short_app).
  } ELSE {
    RUN_TAKEOFF_IFC(rwy_str).
  }
}

// Boot scripts can set GLOBAL IFC_SKIP_INTERACTIVE IS TRUE. before loading
// ifc_main.ks to suppress the interactive menu and call RUN_IFC() directly.
IF NOT (DEFINED IFC_SKIP_INTERACTIVE AND IFC_SKIP_INTERACTIVE) {
  _IFC_INTERACTIVE_START().
}

// ── Main entry point ──────────────────────────────────────
FUNCTION RUN_IFC {
  PARAMETER rwy_id, short_approach.

  // Resolve aircraft config (highest priority wins):
  //   1. Externally set before calling RUN_IFC (manual override)
  //   2. Auto-loaded from aircraft/<vessel_name>_cfg.ks
  //   3. Built-in defaults
  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }

  LOCAL plate IS GET_PLATE_FOR_RUNWAY(rwy_id, short_approach).
  IF plate = 0 {
    SET IFC_ALERT_TEXT TO "ABORT: no plate for RWY " + rwy_id.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN.
  }
  SET plate["vapp"] TO ACTIVE_AIRCRAFT["v_app"].
  SET ACTIVE_PLATE  TO plate.
  SET ACTIVE_V_APP  TO ACTIVE_AIRCRAFT["v_app"].

  IFC_INIT_STATE().
  IFC_LOAD_PLATE().
  UI_INIT().
  AA_INIT().
  LOGGER_INIT().

  SAS OFF.
  LOCK THROTTLE TO THROTTLE_CMD.

  // ── Main loop ──────────────────────────────────────────
  UNTIL IFC_PHASE = PHASE_DONE {

    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).

    LOCAL menu_result IS MENU_TICK().
    IF menu_result = "QUIT" { SET IFC_PHASE TO PHASE_DONE. }

    IF IFC_PHASE = PHASE_APPROACH {
      RUN_APPROACH().
    } ELSE IF IFC_PHASE = PHASE_FLARE     OR
              IFC_PHASE = PHASE_TOUCHDOWN OR
              IFC_PHASE = PHASE_ROLLOUT {
      RUN_AUTOLAND().
    }

    LOGGER_WRITE().
    DISPLAY_TICK().
    WAIT IFC_LOOP_DT.
  }

  // ── Shutdown ───────────────────────────────────────────
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  UNLOCK WHEELSTEERING.
  AA_DISABLE_ALL().
  BRAKES ON.
  LOGGER_CLOSE().

  // Force a final full render of the completion screen.
  SET LAST_DISPLAY_UT  TO 0.
  SET LAST_HEADER_UT   TO 0.
  SET LAST_LOGGER_UT   TO 0.
  DISPLAY_TICK().
}

// ── Takeoff entry point ────────────────────────────────────
FUNCTION RUN_TAKEOFF_IFC {
  PARAMETER rwy_id.

  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }

  LOCAL ils_id IS "KSC_ILS_" + rwy_id.
  LOCAL ils IS GET_BEACON(ils_id).
  IF NOT ils:HASKEY("ll") {
    SET IFC_ALERT_TEXT TO "ABORT: no beacon for RWY " + rwy_id.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN.
  }

  IFC_INIT_STATE().
  SET IFC_PHASE    TO PHASE_TAKEOFF.
  SET IFC_SUBPHASE TO SUBPHASE_TO_PREFLIGHT.
  SET ACTIVE_ILS_ID  TO ils_id.
  SET ACTIVE_RWY_HDG TO ils["hdg"].
  SET TO_RWY_HDG     TO ils["hdg"].

  UI_INIT().

  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_INIT("IFC_TAKEOFF_RWY_" + rwy_id).
    // Emit an initial sample immediately so a CSV exists even if AA init fails later.
    VR_PROBE_TICK().
  }
  AA_INIT().
  LOGGER_INIT().

  SAS OFF.
  LOCK THROTTLE TO THROTTLE_CMD.

  // ── Main loop ─────────────────────────────────────────
  UNTIL IFC_PHASE = PHASE_DONE {
    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).

    LOCAL menu_result IS MENU_TICK().
    IF menu_result = "QUIT" { SET IFC_PHASE TO PHASE_DONE. }

    RUN_TAKEOFF().
    IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
      VR_PROBE_TICK().
    }

    LOGGER_WRITE().
    DISPLAY_TICK().
    WAIT IFC_LOOP_DT.
  }

  // ── Shutdown ──────────────────────────────────────────
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  UNLOCK WHEELSTEERING.
  AA_DISABLE_ALL().
  LOGGER_CLOSE().
  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_FINALIZE().
  }

  // Force a final full render of the completion screen.
  SET LAST_DISPLAY_UT  TO 0.
  SET LAST_HEADER_UT   TO 0.
  SET LAST_LOGGER_UT   TO 0.
  DISPLAY_TICK().
}
