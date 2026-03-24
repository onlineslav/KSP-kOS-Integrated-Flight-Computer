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

RUNPATH(ifc_root + "lib/ifc_constants.ks").
RUNPATH(ifc_root + "lib/ifc_state.ks").
RUNPATH(ifc_root + "lib/ifc_helpers.ks").
RUNPATH(ifc_root + "lib/ifc_autothrottle.ks").
RUNPATH(ifc_root + "lib/ifc_autospoiler.ks").
RUNPATH(ifc_root + "lib/ifc_ui.ks").
RUNPATH(ifc_root + "lib/ifc_menu.ks").
RUNPATH(ifc_root + "lib/ifc_display.ks").
RUNPATH(ifc_root + "lib/ifc_aa.ks").
RUNPATH(ifc_root + "lib/ifc_logger.ks").
RUNPATH(ifc_root + "lib/ifc_fms.ks").
RUNPATH(ifc_root + "lib/ifc_gui.ks").
RUNPATH(ifc_root + "nav/nav_math.ks").
RUNPATH(ifc_root + "nav/nav_beacons.ks").
RUNPATH(ifc_root + "nav/nav_custom_wpts.ks").
RUNPATH(ifc_root + "phases/phase_approach.ks").
RUNPATH(ifc_root + "phases/phase_autoland.ks").
RUNPATH(ifc_root + "phases/phase_takeoff.ks").
RUNPATH(ifc_root + "lib/ifc_ascent_state.ks").
RUNPATH(ifc_root + "phases/phase_ascent.ks").
RUNPATH(ifc_root + "phases/phase_reentry.ks").
RUNPATH(ifc_root + "nav/nav_routes.ks").
RUNPATH(ifc_root + "phases/phase_cruise_alt.ks").

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
  "spoiler_tag",        "",
  "as_enabled",         -1,
  "as_thr_idle_gate",   -1,
  "as_err_deadband_mps",-1,
  "as_err_full_mps",    -1,
  "as_angle_slew_dps",  -1,
  "as_max_deflection_deg",-1,
  "as_crz_speed_lo",    -1,
  "as_crz_speed_hi",    -1,
  "as_crz_cap_deg_lo",  -1,
  "as_crz_cap_deg_hi",  -1,
  "as_app_speed_lo",    -1,
  "as_app_speed_hi",    -1,
  "as_app_cap_deg_lo",  -1,
  "as_app_cap_deg_hi",  -1,
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
  "gear_max_extend_ias", -1,
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
  "tailstrike_pitch_max_deg", -1,
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
  "flare_gear_tag",      "",
  "flare_ctrl_h_offset_max_m",-1,
  "flare_agl",           -1,
  "flare_entry_vs_min",  -1,
  "flare_touchdown_vs",  -1,
  "flare_vs_kp",         -1,
  "flare_fpa_kp",        -1,
  "flare_cmd_fpa_min",   -1,
  "flare_cmd_fpa_max",   -1,
  "flare_cmd_rate_min_dps",-1,
  "flare_cmd_rate_max_dps",-1,
  "flare_roundout_start_h_m",-1,
  "flare_roundout_end_h_m",-1,
  "flare_roundout_curve",-1,
  "flare_roundout_ttg_start_s",-1,
  "flare_roundout_ttg_end_s",-1,
  "flare_min_throttle",  -1,
  "flare_min_throttle_agl_blend",-1,
  "flare_authority_vs_err_trigger",-1,
  "flare_authority_pitch_err_trigger",-1,
  "flare_authority_fpa_err_trigger",-1,
  "flare_authority_detect_s",-1,
  "flare_authority_recovery_gain",-1,
  "flare_tecs_et_kp",     -1,
  "flare_tecs_et_ki",     -1,
  "flare_tecs_et_kd",     -1,
  "flare_tecs_eb_kp",     -1,
  "flare_tecs_eb_ki",     -1,
  "flare_tecs_eb_kd",     -1,
  "flare_tecs_edot_alpha",-1,
  "flare_tecs_et_int_lim",-1,
  "flare_tecs_eb_int_lim",-1,
  "flare_tecs_thr_trim",  -1,
  "flare_tecs_thr_bal_k", -1,
  "flare_tecs_thr_slew_per_s",-1,
  "flare_tecs_climb_vs_gate",-1,
  "flare_balloon_vs_trigger",-1,
  "flare_balloon_clear_vs",-1,
  "flare_balloon_min_h_m",-1,
  "flare_balloon_gamma_down_deg",-1,
  "flare_disable_speed_bleed",-1,
  "touchdown_confirm_s", -1,
  "touchdown_confirm_max_abs_vs", -1,
  "touchdown_fallback_max_abs_vs", -1,
  "touchdown_fallback_max_agl_m", -1,
  "touchdown_nose_hold_s", -1,
  "touchdown_nose_lower_rate_dps", -1,
  "notes",              "Default config."
).

// ── Auto aircraft config loader ───────────────────────────
// Normalises SHIP:NAME to a filename, then looks for a matching
// config in aircraft/.  Returns TRUE if a config was loaded.
// Mapping example: "XF1-A" → "xf1_a_cfg.ks"
FUNCTION _CFG_PATH_FOR_SHIP_NAME {
  LOCAL vessel_name_key IS SHIP:NAME:TOLOWER
    :REPLACE(" ", "_")
    :REPLACE("-", "_")
    :REPLACE(".", "_").
  RETURN ifc_root + "aircraft/" + vessel_name_key + "_cfg.ks".
}

// Resolve IFC_ACTIVE_CFG_PATH when ACTIVE_AIRCRAFT was preloaded externally.
// If we cannot prove the source path, keep an explicit unknown descriptor.
FUNCTION _RESOLVE_CFG_SOURCE_FOR_LOGGER {
  IF ACTIVE_AIRCRAFT = 0 OR ACTIVE_AIRCRAFT = _DEFAULT_AIRCRAFT {
    SET IFC_ACTIVE_CFG_PATH TO "INTERNAL_DEFAULT".
    RETURN.
  }

  LOCAL inferred_cfg IS _CFG_PATH_FOR_SHIP_NAME().
  IF EXISTS(inferred_cfg) {
    SET IFC_ACTIVE_CFG_PATH TO inferred_cfg + " (inferred preloaded)".
    RETURN.
  }

  SET IFC_ACTIVE_CFG_PATH TO "PRELOADED_ACTIVE_AIRCRAFT (cfg path unknown)".
}

FUNCTION _TRY_LOAD_AIRCRAFT_CONFIG {
  LOCAL cfg IS _CFG_PATH_FOR_SHIP_NAME().

  IF EXISTS(cfg) {
    RUNPATH(cfg).
    SET ACTIVE_AIRCRAFT TO BUILD_AIRCRAFT_CONFIG().
    SET IFC_ACTIVE_CFG_PATH TO cfg.
    IFC_SET_ALERT("Loaded config: " + SHIP:NAME).
    RETURN TRUE.
  }

  SET IFC_ACTIVE_CFG_PATH TO "INTERNAL_DEFAULT".
  IFC_SET_ALERT("No config for '" + SHIP:NAME + "' - defaults", "WARN").
  RETURN FALSE.
}

// ── Build flight plan from current menu state ─────────────
FUNCTION _BUILD_PLAN_FROM_MENU {
  LOCAL plan IS LIST().
  LOCAL rwy_str IS "09".
  IF IFC_MENU_OPT_RWY = 1 { SET rwy_str TO "27". }

  IF IFC_MENU_OPT_PROC = 0 {
    // Approach-only procedure
    LOCAL short_app IS IFC_MENU_OPT_DIST = 1.
    plan:ADD(LEXICON(
      "type",   LEG_APPROACH,
      "params", LEXICON("rwy_id", rwy_str, "short_approach", short_app)
    )).
  } ELSE {
    // Takeoff procedure
    plan:ADD(LEXICON(
      "type",   LEG_TAKEOFF,
      "params", LEXICON("rwy_id", rwy_str)
    )).
    // Optional cruise + approach to destination
    IF IFC_MENU_OPT_DEST = 1 {
      LOCAL route IS ROUTE_KSC_TO_ISL.
      plan:ADD(LEXICON(
        "type",   LEG_CRUISE,
        "params", LEXICON(
          "waypoints", route["waypoints"],
          "alt_m",     route["cruise_alt_m"],
          "spd",       route["cruise_spd"]
        )
      )).
      plan:ADD(LEXICON(
        "type",   LEG_APPROACH,
        "params", LEXICON("plate", route["dest_plate"])
      )).
    }
  }
  RETURN plan.
}

// ── Build flight plan from DRAFT_PLAN (plan editor state) ─
// Converts the editor representation (index-based) into the
// execution representation (string IDs) consumed by _INIT_LEG.
FUNCTION _BUILD_PLAN_FROM_DRAFT {
  LOCAL plan IS LIST().
  LOCAL i IS 0.
  UNTIL i >= DRAFT_PLAN:LENGTH {
    LOCAL dl IS DRAFT_PLAN[i].
    LOCAL t  IS dl["type"].
    LOCAL p  IS dl["params"].

    IF t = LEG_TAKEOFF {
      LOCAL rwy IS "09".
      IF ROUND(p["rwy_idx"], 0) = 1 { SET rwy TO "27". }
      plan:ADD(LEXICON("type", LEG_TAKEOFF,
        "params", LEXICON("rwy_id", rwy))).

    } ELSE IF t = LEG_CRUISE {
      LOCAL wpts IS LIST().
      LOCAL wi IS 0.
      UNTIL wi >= FMS_WPT_SLOTS {
        LOCAL wkey IS "wpt" + wi.
        LOCAL widx IS ROUND(p[wkey], 0).
        IF widx >= 0 AND widx < CUSTOM_WPT_IDS:LENGTH {
          wpts:ADD(CUSTOM_WPT_IDS[widx]).
        }
        SET wi TO wi + 1.
      }
      plan:ADD(LEXICON("type", LEG_CRUISE,
        "params", LEXICON("waypoints", wpts, "alt_m", p["alt_m"], "spd", p["spd"]))).

    } ELSE IF t = LEG_APPROACH {
      LOCAL pidx IS ROUND(p["plate_idx"], 0).
      LOCAL pid  IS "PLATE_KSC_ILS09".
      IF pidx >= 0 AND pidx < PLATE_IDS:LENGTH { SET pid TO PLATE_IDS[pidx]. }
      plan:ADD(LEXICON("type", LEG_APPROACH,
        "params", LEXICON("plate_id", pid))).
    }

    SET i TO i + 1.
  }
  RETURN plan.
}

// ── Initialise a single flight-plan leg ───────────────────
// Sets up phase state then calls SET_PHASE for the new phase.
FUNCTION _INIT_LEG {
  PARAMETER leg.
  LOCAL leg_type IS leg["type"].
  LOCAL params   IS leg["params"].

  IF leg_type = LEG_TAKEOFF {
    LOCAL rwy_id IS params["rwy_id"].
    // Compatibility: plain "27" maps to KSC's left runway beacon.
    LOCAL lookup_rwy_id IS rwy_id.
    IF lookup_rwy_id = "27" { SET lookup_rwy_id TO "27L". }
    LOCAL ils_id IS "".
    LOCAL ksc_ils_id IS "KSC_ILS_" + lookup_rwy_id.
    LOCAL daf_ils_id IS "DAF_ILS_" + rwy_id.
    LOCAL isl_ils_id IS "ISL_ILS_" + rwy_id.
    IF NAV_BEACON_DB:HASKEY(ksc_ils_id) {
      SET ils_id TO ksc_ils_id.
    } ELSE IF NAV_BEACON_DB:HASKEY(daf_ils_id) {
      SET ils_id TO daf_ils_id.
    } ELSE IF NAV_BEACON_DB:HASKEY(isl_ils_id) {
      SET ils_id TO isl_ils_id.
    }
    LOCAL ils IS GET_BEACON(ils_id).
    IF NOT ils:HASKEY("ll") {
      IFC_SET_ALERT("ABORT: no beacon for RWY " + rwy_id, "ERROR").
      SET_PHASE(PHASE_DONE).
      RETURN.
    }
    SET ACTIVE_ILS_ID  TO ils_id.
    SET ACTIVE_RWY_HDG TO ils["hdg"].
    SET TO_RWY_HDG     TO ils["hdg"].
    // Apply v-speed overrides into aircraft config
    IF IFC_MENU_OPT_VR > 0 { SET ACTIVE_AIRCRAFT["v_r"] TO IFC_MENU_OPT_VR. }
    IF IFC_MENU_OPT_V2 > 0 { SET ACTIVE_AIRCRAFT["v2"]  TO IFC_MENU_OPT_V2. }
    SET_PHASE(PHASE_TAKEOFF).
    SET IFC_SUBPHASE TO SUBPHASE_TO_PREFLIGHT.

  } ELSE IF leg_type = LEG_CRUISE {
    LOCAL wpts  IS LIST().
    LOCAL alt_m IS CRUISE_DEFAULT_ALT_M.
    LOCAL spd   IS CRUISE_DEFAULT_SPD.
    LOCAL nt    IS "waypoint".

    IF params:HASKEY("route") {
      // Legacy route wrapper
      LOCAL route IS params["route"].
      SET wpts  TO route["waypoints"].
      SET alt_m TO route["cruise_alt_m"].
      SET spd   TO route["cruise_spd"].
    } ELSE IF params:HASKEY("nav_type") {
      // FMS leg format: nav_type + wpt0/1/2 or course params
      SET nt TO params["nav_type"].
      IF params:HASKEY("alt_m") { SET alt_m TO params["alt_m"]. }
      IF params:HASKEY("spd")   { SET spd   TO params["spd"]. }

      IF nt = "course_dist" {
        LOCAL cdeg IS 90.
        LOCAL dnm  IS 100.
        IF params:HASKEY("course_deg") { SET cdeg TO params["course_deg"]. }
        IF params:HASKEY("dist_nm")    { SET dnm  TO params["dist_nm"]. }
        // Create a synthetic beacon at the destination; navigate to it normally.
        LOCAL dest_ll IS GEO_DESTINATION(SHIP:GEOPOSITION, cdeg, dnm * 1852).
        IF NAV_BEACON_DB:HASKEY("_CRUISE_TGT") { NAV_BEACON_DB:REMOVE("_CRUISE_TGT"). }
        REGISTER_BEACON(MAKE_BEACON("_CRUISE_TGT", BTYPE_WPT,
          dest_ll, alt_m, LEXICON("name", "Course target"))).
        wpts:ADD("_CRUISE_TGT").
        SET CRUISE_COURSE_DEG TO cdeg.

      } ELSE IF nt = "course_time" {
        LOCAL cdeg IS 90.
        LOCAL tmin IS 60.
        IF params:HASKEY("course_deg") { SET cdeg TO params["course_deg"]. }
        IF params:HASKEY("time_min")   { SET tmin TO params["time_min"]. }
        SET CRUISE_COURSE_DEG TO cdeg.
        SET CRUISE_END_UT     TO TIME:SECONDS + tmin * 60.

      } ELSE {
        // "waypoint" nav_type: build wpt list from wpt0/wpt1/wpt2 indices
        LOCAL slot IS 0.
        UNTIL slot >= FMS_WPT_SLOTS {
          LOCAL wkey IS "wpt" + slot.
          IF params:HASKEY(wkey) {
            LOCAL widx IS ROUND(params[wkey], 0).
            IF widx >= 0 AND widx < CUSTOM_WPT_IDS:LENGTH {
              wpts:ADD(CUSTOM_WPT_IDS[widx]).
            }
          }
          SET slot TO slot + 1.
        }
      }
    } ELSE {
      // Old flat format: params["waypoints"] is already a list of beacon IDs
      IF params:HASKEY("waypoints") { SET wpts  TO params["waypoints"]. }
      IF params:HASKEY("alt_m")     { SET alt_m TO params["alt_m"]. }
      IF params:HASKEY("spd")       { SET spd   TO params["spd"]. }
    }

    SET CRUISE_NAV_TYPE   TO nt.
    SET CRUISE_WAYPOINTS  TO wpts.
    SET CRUISE_ALT_M      TO alt_m.
    SET CRUISE_SPD_MPS    TO spd.
    SET CRUISE_DEST_PLATE TO 0.
    SET CRUISE_WP_INDEX   TO 0.
    SET THR_INTEGRAL      TO 0.
    SET PREV_IAS          TO GET_IAS().
    SET A_ACTUAL_FILT     TO 0.
    AT_RESET().
    AS_RELEASE().
    SET_PHASE(PHASE_CRUISE).

  } ELSE IF leg_type = LEG_APPROACH {
    LOCAL plate IS 0.
    IF params:HASKEY("plate_id") {
      SET plate TO GET_PLATE(params["plate_id"]).
    } ELSE IF params:HASKEY("plate") {
      SET plate TO params["plate"].
    } ELSE {
      LOCAL rwy_id    IS params["rwy_id"].
      LOCAL short_app IS params["short_approach"].
      SET plate TO GET_PLATE_FOR_RUNWAY(rwy_id, short_app).
    }
    IF plate = 0 {
      IFC_SET_ALERT("ABORT: no plate for approach leg", "ERROR").
      SET_PHASE(PHASE_DONE).
      RETURN.
    }
    SET ACTIVE_PLATE TO plate.
    IFC_LOAD_PLATE().
    // Flush speed-controller state to avoid cruise trim carry-over
    SET THR_INTEGRAL  TO 0.
    SET PREV_IAS      TO GET_IAS().
    SET A_ACTUAL_FILT TO 0.
    AT_RESET().
    AS_RELEASE().
    // Apply Vapp override after plate load
    IF IFC_MENU_OPT_VAPP > 0 {
      SET ACTIVE_V_APP   TO IFC_MENU_OPT_VAPP.
      SET ACTIVE_V_TGT   TO IFC_MENU_OPT_VAPP.
      SET APP_VREF_TGT   TO IFC_MENU_OPT_VAPP - 10.
      SET APP_VINT_TGT   TO IFC_MENU_OPT_VAPP.
      SET APP_BASE_V_TGT TO IFC_MENU_OPT_VAPP.
    }
    SET_PHASE(PHASE_APPROACH).
    SET IFC_SUBPHASE TO SUBPHASE_FLY_TO_FIX.

  } ELSE IF leg_type = LEG_ASCENT {
    SET_PHASE(PHASE_ASCENT).

  } ELSE IF leg_type = LEG_REENTRY {
    SET_PHASE(PHASE_REENTRY).
  }
}

// ── Unified flight plan executor ──────────────────────────
// Runs init, then drives the leg queue until all legs complete.
FUNCTION _RUN_FLIGHT_PLAN {
  PARAMETER plan.

  IF plan:LENGTH = 0 {
    IFC_SET_ALERT("Empty flight plan", "ERROR").
    RETURN.
  }

  IFC_INIT_STATE().
  UI_INIT().

  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_INIT("IFC_FLIGHT_PLAN").
    VR_PROBE_TICK().
  }
  IF IFC_ACTIVE_CFG_PATH = "" {
    _RESOLVE_CFG_SOURCE_FOR_LOGGER().
  }
  AA_INIT().
  AS_RESET().
  AS_DISCOVER_PARTS().
  LOGGER_INIT().

  SAS OFF.
  LOCK THROTTLE TO THROTTLE_CMD.
  // LOCK STEERING is established only when AA is unavailable (see AA_SET_DIRECTOR_PITCH).
  // When AA Director or FBW is active, kOS LOCK STEERING must NOT be held or the
  // two control writers fight each other and produce near-zero net deflection.

  SET FLIGHT_PLAN       TO plan.
  SET FLIGHT_PLAN_INDEX TO 0.
  _INIT_LEG(plan[0]).
  IFC_SET_UI_MODE(UI_MODE_AUTOFLOW).

  // ── Main loop ─────────────────────────────────────────
  UNTIL IFC_PHASE = PHASE_DONE {
    LOCAL actual_dt IS TIME:SECONDS - IFC_CYCLE_UT.
    SET IFC_CYCLE_UT  TO TIME:SECONDS.
    SET IFC_RAW_DT    TO MAX(actual_dt, 0).
    SET IFC_ACTUAL_DT TO CLAMP(IFC_RAW_DT, 0.01, 0.5).
    SET IFC_LOOP_COUNT TO IFC_LOOP_COUNT + 1.
    IF IFC_RAW_DT > IFC_RAW_DT_MAX { SET IFC_RAW_DT_MAX TO IFC_RAW_DT. }
    IF IFC_RAW_DT < IFC_RAW_DT_MIN { SET IFC_RAW_DT_MIN TO IFC_RAW_DT. }

    // ── Per-loop vector cache (P1b) ────────────────────────
    // Query each VM suffix once; all phase functions read these globals.
    LOCAL _fac IS SHIP:FACING.
    SET IFC_FACING_FWD  TO _fac:FOREVECTOR.
    SET IFC_FACING_STAR TO _fac:STARVECTOR.
    SET IFC_FACING_TOP  TO _fac:TOPVECTOR.
    SET IFC_UP_VEC      TO SHIP:UP:VECTOR.
    SET IFC_NORTH_VEC   TO SHIP:NORTH:VECTOR.
    // Telemetry: heading / pitch / bank computed once here (P1d)
    LOCAL _east IS VCRS(IFC_UP_VEC, IFC_NORTH_VEC).  // up × north = east in KSP's left-handed surface frame
    SET TELEM_PITCH_DEG   TO 90 - VECTORANGLE(IFC_FACING_FWD, IFC_UP_VEC).
    SET TELEM_COMPASS_HDG TO MOD(ARCTAN2(VDOT(IFC_FACING_FWD, _east), VDOT(IFC_FACING_FWD, IFC_NORTH_VEC)) + 360, 360).
    SET TELEM_BANK_DEG    TO VECTORANGLE(IFC_FACING_STAR, IFC_UP_VEC) - 90.
    SET TELEM_ACTUAL_FPA_DEG TO ARCTAN(SHIP:VERTICALSPEED / MAX(GET_IAS(), 1)).
    LOCAL _steer_fwd IS IFC_DESIRED_STEERING:FOREVECTOR.
    SET TELEM_KOS_STEER_PIT  TO 90 - VECTORANGLE(_steer_fwd, IFC_UP_VEC).
    SET TELEM_KOS_STEER_HDG  TO MOD(ARCTAN2(VDOT(_steer_fwd, _east), VDOT(_steer_fwd, IFC_NORTH_VEC)) + 360, 360).
    AA_POLL_TELEM().

    LOCAL menu_result IS "".
    IF IFC_FAST_MODE {
      // Fast-mode: avoid menu/UI overhead; keep a minimal quit key path.
      IF TERMINAL:INPUT:HASCHAR {
        LOCAL ch IS TERMINAL:INPUT:GETCHAR().
        IF ch = "q" OR ch = "Q" {
          SET menu_result TO "QUIT".
          SET_PHASE(PHASE_DONE).
          IFC_SET_UI_MODE(UI_MODE_COMPLETE).
        }
      }
    } ELSE {
      SET menu_result TO MENU_TICK().
      IF menu_result = "QUIT" {
        SET_PHASE(PHASE_DONE).
        IFC_SET_UI_MODE(UI_MODE_COMPLETE).
      }
    }

    IF NOT IFC_MANUAL_MODE {
      IF IFC_PHASE = PHASE_TAKEOFF {
        RUN_TAKEOFF().
        IF NOT IFC_FAST_MODE AND DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
          VR_PROBE_TICK().
        }
      } ELSE IF IFC_PHASE = PHASE_CRUISE {
        RUN_CRUISE().
      } ELSE IF IFC_PHASE = PHASE_APPROACH {
        RUN_APPROACH().
      } ELSE IF IFC_PHASE = PHASE_FLARE     OR
                IFC_PHASE = PHASE_TOUCHDOWN OR
                IFC_PHASE = PHASE_ROLLOUT {
        RUN_AUTOLAND().
      } ELSE IF IFC_PHASE = PHASE_ASCENT {
        RUN_ASCENT().
      } ELSE IF IFC_PHASE = PHASE_REENTRY {
        RUN_REENTRY().
      }
    }

    // Leg queue advancement: when a leg sets PHASE_DONE naturally, start
    // the next leg.  Skip advancement on QUIT so the loop exits cleanly.
    IF IFC_PHASE = PHASE_DONE AND menu_result <> "QUIT" {
      SET FLIGHT_PLAN_INDEX TO FLIGHT_PLAN_INDEX + 1.
      IF FLIGHT_PLAN_INDEX < FLIGHT_PLAN:LENGTH {
        _INIT_LEG(plan[FLIGHT_PLAN_INDEX]).
        IF NOT IFC_MANUAL_MODE { IFC_SET_UI_MODE(UI_MODE_AUTOFLOW). }
      }
      // If still PHASE_DONE (no more legs or _INIT_LEG aborted), loop exits.
    }

    LOGGER_WRITE().
    IF NOT IFC_FAST_MODE { DISPLAY_TICK(). }
    WAIT IFC_LOOP_DT.
  }

  // ── Shutdown ──────────────────────────────────────────
  AS_RELEASE().
  UNLOCK THROTTLE.
  UNLOCK STEERING.
  UNLOCK WHEELSTEERING.
  AA_RESTORE_FBW().
  BRAKES ON.
  IFC_GEAR_VIS_CLEAR().
  LOGGER_CLOSE().
  IF DEFINED VR_PROBE_HOOKS_READY AND VR_PROBE_HOOKS_READY {
    VR_PROBE_FINALIZE().
  }

  IFC_SET_UI_MODE(UI_MODE_COMPLETE).

  SET LAST_DISPLAY_UT  TO 0.
  SET LAST_HEADER_UT   TO 0.
  SET LAST_LOGGER_UT   TO 0.
  IF NOT IFC_FAST_MODE { DISPLAY_TICK(). }
}

// ── Interactive startup (FMS pre-arm screen) ──────────────
// Shows the plan editor, waits for the user to ARM or QUIT.
// On ARM, converts DRAFT_PLAN to an execution plan and runs it.
FUNCTION _IFC_INTERACTIVE_START {
  IFC_INIT_STATE().
  SET IFC_ACTIVE_CFG_PATH TO "".
  SET IFC_MISSION_START_UT TO TIME:SECONDS.
  SET IFC_PHASE    TO PHASE_PREARM.
  SET IFC_SUBPHASE TO "".
  IFC_SET_UI_MODE(UI_MODE_PREARM).
  UI_INIT().

  IF ACTIVE_AIRCRAFT = 0 {
    _TRY_LOAD_AIRCRAFT_CONFIG().
    IF ACTIVE_AIRCRAFT = 0 { SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT. }
  }

  // Seed DRAFT_PLAN with a default takeoff leg if it's empty.
  IF DRAFT_PLAN:LENGTH = 0 {
    DRAFT_PLAN:ADD(_FMS_DEFAULT_LEG(LEG_TAKEOFF)).
  }

  _GUI_BUILD().

  LOCAL result IS "".
  UNTIL result = "ARM" OR result = "QUIT" {
    LOCAL gui_result IS _GUI_TICK().
    IF gui_result <> "" { SET result TO gui_result. }
    ELSE { SET result TO MENU_TICK(). }
    DISPLAY_TICK().
    WAIT IFC_LOOP_DT.
  }

  _GUI_CLOSE().

  IF result = "QUIT" { RETURN. }

  _RUN_FLIGHT_PLAN(_BUILD_PLAN_FROM_DRAFT()).
}

// Boot scripts can set GLOBAL IFC_SKIP_INTERACTIVE IS TRUE. before loading
// ifc_main.ks to suppress the interactive menu and call RUN_IFC() directly.
IF NOT (DEFINED IFC_SKIP_INTERACTIVE AND IFC_SKIP_INTERACTIVE) {
  _IFC_INTERACTIVE_START().
}

// ── Legacy entry points (wrappers around _RUN_FLIGHT_PLAN) ─
// These remain for external scripts that call RUN_IFC or
// RUN_TAKEOFF_IFC directly.

FUNCTION RUN_IFC {
  PARAMETER rwy_id, short_approach.
  SET IFC_ACTIVE_CFG_PATH TO "".
  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }
  IF IFC_ACTIVE_CFG_PATH = "" {
    _RESOLVE_CFG_SOURCE_FOR_LOGGER().
  }
  LOCAL plan IS LIST(LEXICON(
    "type",   LEG_APPROACH,
    "params", LEXICON("rwy_id", rwy_id, "short_approach", short_approach)
  )).
  _RUN_FLIGHT_PLAN(plan).
}

// route: a route LEXICON from nav_routes.ks, or 0 for takeoff-only.
FUNCTION RUN_TAKEOFF_IFC {
  PARAMETER rwy_id.
  PARAMETER route IS 0.
  SET IFC_ACTIVE_CFG_PATH TO "".
  IF ACTIVE_AIRCRAFT = 0 {
    IF NOT _TRY_LOAD_AIRCRAFT_CONFIG() {
      SET ACTIVE_AIRCRAFT TO _DEFAULT_AIRCRAFT.
    }
  }
  IF IFC_ACTIVE_CFG_PATH = "" {
    _RESOLVE_CFG_SOURCE_FOR_LOGGER().
  }
  LOCAL plan IS LIST(LEXICON(
    "type",   LEG_TAKEOFF,
    "params", LEXICON("rwy_id", rwy_id)
  )).
  IF route <> 0 {
    plan:ADD(LEXICON(
      "type",   LEG_CRUISE,
      "params", LEXICON(
        "waypoints", route["waypoints"],
        "alt_m",     route["cruise_alt_m"],
        "spd",       route["cruise_spd"]
      )
    )).
    plan:ADD(LEXICON(
      "type",   LEG_APPROACH,
      "params", LEXICON("plate", route["dest_plate"])
    )).
  }
  _RUN_FLIGHT_PLAN(plan).
}
