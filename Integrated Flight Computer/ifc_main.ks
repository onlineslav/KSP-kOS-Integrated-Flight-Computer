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
//   3. The IFC will ask you to confirm "ARM? (y/n)" before
//      locking controls.
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
RUNONCEPATH(ifc_root + "lib/ifc_aa.ks").
RUNONCEPATH(ifc_root + "lib/ifc_telemetry.ks").
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
  "takeoff_rotate_pitch_kp",-1,
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
  "takeoff_min_avail_thrust",-1,
  "takeoff_loc_kp",     -1,
  "takeoff_loc_guard_m",-1,
  "takeoff_steer_max_corr",-1,
  "takeoff_dir_max_corr",-1,
  "takeoff_yaw_start_ias",-1,
  "takeoff_yaw_full_ias",-1,
  "takeoff_yaw_min_scale",-1,
  "takeoff_yaw_kp",     -1,
  "takeoff_yaw_max_cmd",-1,
  "takeoff_yaw_slew_per_s",-1,
  "takeoff_yaw_sign",   -1,
  "takeoff_climb_min_throttle",-1,
  "takeoff_climb_spd_thr_gain",-1,
  "takeoff_climb_fpa_spd_gain",-1,
  "takeoff_climb_fpa_min",-1,
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
    PRINT "IFC: loaded aircraft config for '" + SHIP:NAME + "'".
    RETURN TRUE.
  }

  PRINT "IFC: no config found for '" + SHIP:NAME + "' — using defaults.".
  PRINT "     (expected: " + cfg + ")".
  RETURN FALSE.
}

// ── Interactive startup (called when file is run directly) ──
FUNCTION _IFC_INTERACTIVE_START {
  CLEARSCREEN.
  PRINT "╔══════════════════════════════════════╗".
  PRINT "║   Integrated Flight Computer  v1.0  ║".
  PRINT "╚══════════════════════════════════════╝".
  PRINT " ".
  PRINT "Select mode:".
  PRINT "  1 = KSC RWY 09  (approach, full 60 km)".
  PRINT "  2 = KSC RWY 09  (approach, short 30 km)".
  PRINT "  3 = KSC RWY 27  (approach, full 60 km)".
  PRINT "  4 = KSC RWY 27  (approach, short 30 km)".
  PRINT "  5 = KSC RWY 09  (takeoff)".
  PRINT "  6 = KSC RWY 27  (takeoff)".
  PRINT " ".

  LOCAL sel IS "".
  UNTIL sel = "1" OR sel = "2" OR sel = "3" OR sel = "4" OR sel = "5" OR sel = "6" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }

  IF      sel = "1" { RUN_IFC("09", FALSE).     }
  ELSE IF sel = "2" { RUN_IFC("09", TRUE).       }
  ELSE IF sel = "3" { RUN_IFC("27", FALSE).      }
  ELSE IF sel = "4" { RUN_IFC("27", TRUE).       }
  ELSE IF sel = "5" { RUN_TAKEOFF_IFC("09").     }
  ELSE IF sel = "6" { RUN_TAKEOFF_IFC("27").     }
}

// Boot scripts can set GLOBAL IFC_SKIP_INTERACTIVE IS TRUE. before loading
// ifc_main.ks to suppress the interactive menu and call RUN_IFC() directly.
IF NOT (DEFINED IFC_SKIP_INTERACTIVE AND IFC_SKIP_INTERACTIVE) {
  _IFC_INTERACTIVE_START().
}

// ── Main entry point ──────────────────────────────────────
FUNCTION RUN_IFC {
  PARAMETER rwy_id, short_approach.

  CLEARSCREEN.

  // Resolve aircraft config (highest priority wins):
  //   1. Externally set before calling RUN_IFC (manual override)
  //   2. Auto-loaded from aircraft/<vessel_name>_cfg.ks
  //   3. Built-in defaults
  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }

  // Override Vapp from aircraft config into the plate.
  // (Plates carry a generic Vapp; the aircraft config overrides it.)
  LOCAL plate IS GET_PLATE_FOR_RUNWAY(rwy_id, short_approach).
  IF plate = 0 { PRINT "IFC: abort - no plate for RWY " + rwy_id. RETURN. }
  SET plate["vapp"] TO ACTIVE_AIRCRAFT["v_app"].
  SET ACTIVE_PLATE  TO plate.
  SET ACTIVE_V_APP  TO ACTIVE_AIRCRAFT["v_app"].

  // ── Startup banner ──────────────────────────────────────
  PRINT "╔══════════════════════════════════════╗".
  PRINT "║   Integrated Flight Computer  v1.0  ║".
  PRINT "╚══════════════════════════════════════╝".
  PRINT "Aircraft : " + ACTIVE_AIRCRAFT["name"].
  PRINT "Approach : " + ACTIVE_PLATE["name"].
  PRINT "Vapp     : " + ACTIVE_V_APP + " m/s".
  IF ACTIVE_AIRCRAFT["notes"] <> "" {
    PRINT "Notes    : " + ACTIVE_AIRCRAFT["notes"].
  }
  PRINT " ".

  // ── Addon detection ─────────────────────────────────────
  IF ADDONS:AVAILABLE("AA") {
    PRINT "kOS-AA   : detected".
  } ELSE {
    PRINT "kOS-AA   : NOT detected (kOS steering fallback)".
  }
  IF ADDONS:AVAILABLE("FAR") {
    PRINT "FAR      : detected".
  } ELSE {
    PRINT "FAR      : not detected (using SHIP:AIRSPEED)".
  }
  PRINT " ".

  // ── Arm confirmation ────────────────────────────────────
  PRINT "ARM IFC for " + ACTIVE_PLATE["name"] + "? (y/n)".
  TERMINAL:INPUT:CLEAR().
  LOCAL reply IS "".
  UNTIL reply = "y" OR reply = "Y" OR reply = "n" OR reply = "N" {
    SET reply TO TERMINAL:INPUT:GETCHAR().
  }
  IF reply = "n" OR reply = "N" {
    PRINT "IFC: disarmed.".
    RETURN.
  }
  PRINT "IFC: ARMED.".
  PRINT " ".

  // ── Initialise state ────────────────────────────────────
  IFC_INIT_STATE().
  IFC_LOAD_PLATE().
  AA_INIT().
  LOGGER_INIT().
  PRINT "IFC: Vapp target armed at " + ROUND(ACTIVE_V_APP, 1) + " m/s (aircraft cfg)".

  SAS OFF.
  LOCK THROTTLE TO THROTTLE_CMD.

  PRINT "IFC: entering APPROACH  (subphase: FLY_TO_FIX)".
  PRINT "     First fix: " + ACTIVE_FIXES[0] + "  (" +
        ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, GET_BEACON(ACTIVE_FIXES[0])["ll"]) / 1000, 1) +
        " km)".
  PRINT " ".
  PRINT "Press CTRL+C at any time to abort and take manual control.".
  PRINT "────────────────────────────────────────".

  // ── Main loop ───────────────────────────────────────────
  UNTIL IFC_PHASE = PHASE_DONE {

    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).

    IF IFC_PHASE = PHASE_APPROACH {
      RUN_APPROACH().
    } ELSE IF IFC_PHASE = PHASE_FLARE     OR
              IFC_PHASE = PHASE_TOUCHDOWN OR
              IFC_PHASE = PHASE_ROLLOUT {
      RUN_AUTOLAND().
    }

    LOGGER_WRITE().
    PRINT_TELEMETRY().
    WAIT IFC_LOOP_DT.
  }

  // ── Shutdown ────────────────────────────────────────────
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  UNLOCK WHEELSTEERING.
  AA_DISABLE_ALL().
  BRAKES ON.
  LOGGER_CLOSE().

  PRINT " ".
  PRINT "IFC: COMPLETE — " + ACTIVE_PLATE["name"] + ".".
  PRINT "     Landed at T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1) + " s.".
}

// ── Takeoff entry point ────────────────────────────────
FUNCTION RUN_TAKEOFF_IFC {
  PARAMETER rwy_id.

  CLEARSCREEN.

  // Resolve aircraft config.
  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }

  // Look up ILS beacon for runway heading.
  LOCAL ils_id IS "KSC_ILS_" + rwy_id.
  LOCAL ils IS GET_BEACON(ils_id).
  IF ils = 0 { PRINT "IFC: abort - no beacon for RWY " + rwy_id. RETURN. }

  // ── Startup banner ──────────────────────────────────
  PRINT "╔══════════════════════════════════════╗".
  PRINT "║   Integrated Flight Computer  v1.0  ║".
  PRINT "╚══════════════════════════════════════╝".
  PRINT "Aircraft : " + ACTIVE_AIRCRAFT["name"].
  PRINT "Takeoff  : RWY " + rwy_id + "  hdg " + ils["hdg"].
  LOCAL v_r IS AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001).
  LOCAL v2  IS AC_PARAM("v2",  TAKEOFF_V2_DEFAULT,  0.001).
  PRINT "V_R " + ROUND(v_r, 1) + " m/s   V2 " + ROUND(v2, 1) + " m/s".
  IF ACTIVE_AIRCRAFT["notes"] <> "" {
    PRINT "Notes    : " + ACTIVE_AIRCRAFT["notes"].
  }
  PRINT " ".

  // ── Addon detection ─────────────────────────────────
  IF ADDONS:AVAILABLE("AA") {
    PRINT "kOS-AA   : detected".
  } ELSE {
    PRINT "kOS-AA   : NOT detected (kOS steering fallback)".
  }
  PRINT " ".

  // ── Arm confirmation ────────────────────────────────
  LOCAL auto_arm_takeoff IS FALSE.
  IF DEFINED IFC_AUTO_ARM_TAKEOFF AND IFC_AUTO_ARM_TAKEOFF {
    SET auto_arm_takeoff TO TRUE.
    // One-shot flag so test helpers don't leak into later manual runs.
    SET IFC_AUTO_ARM_TAKEOFF TO FALSE.
  }
  IF NOT auto_arm_takeoff {
    PRINT "ARM TAKEOFF for RWY " + rwy_id + "? (y/n)".
    TERMINAL:INPUT:CLEAR().
    LOCAL reply IS "".
    UNTIL reply = "y" OR reply = "Y" OR reply = "n" OR reply = "N" {
      SET reply TO TERMINAL:INPUT:GETCHAR().
    }
    IF reply = "n" OR reply = "N" {
      PRINT "IFC: disarmed.".
      RETURN.
    }
  } ELSE {
    PRINT "IFC: auto-arm enabled (IFC_AUTO_ARM_TAKEOFF).".
  }
  PRINT "IFC: ARMED.".
  PRINT " ".

  // ── Initialise state ────────────────────────────────
  IFC_INIT_STATE().
  SET IFC_PHASE    TO PHASE_TAKEOFF.
  SET IFC_SUBPHASE TO SUBPHASE_TO_PREFLIGHT.
  SET ACTIVE_ILS_ID TO ils_id.
  SET ACTIVE_RWY_HDG TO ils["hdg"].
  SET TO_RWY_HDG   TO ils["hdg"].
  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_INIT("IFC_TAKEOFF_RWY_" + rwy_id).
    // Emit an initial sample immediately so a CSV exists even if AA init fails later.
    VR_PROBE_TICK().
  }
  AA_INIT().
  LOGGER_INIT().

  SAS OFF.
  LOCK THROTTLE TO THROTTLE_CMD.

  PRINT "IFC: entering TAKEOFF  (subphase: TO_PREFLIGHT)".
  PRINT "     Runway hdg " + ROUND(ils["hdg"], 1) + "  V_R " + ROUND(v_r, 1) + " m/s".
  PRINT " ".
  PRINT "Press CTRL+C at any time to abort and take manual control.".
  PRINT "────────────────────────────────────────".

  // ── Main loop ───────────────────────────────────────
  UNTIL IFC_PHASE = PHASE_DONE {
    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_ACTUAL_DT TO CLAMP(actual_dt, 0.01, 0.5).

    RUN_TAKEOFF().
    IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
      VR_PROBE_TICK().
    }

    LOGGER_WRITE().
    PRINT_TELEMETRY().
    WAIT IFC_LOOP_DT.
  }

  // ── Shutdown ────────────────────────────────────────
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  UNLOCK WHEELSTEERING.
  AA_DISABLE_ALL().
  LOGGER_CLOSE().
  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_FINALIZE().
  }

  PRINT " ".
  PRINT "IFC: TAKEOFF COMPLETE — RWY " + rwy_id + ".".
  PRINT "     Airborne at T+" + ROUND(TIME:SECONDS - IFC_MISSION_START_UT, 1) + " s.".
}
