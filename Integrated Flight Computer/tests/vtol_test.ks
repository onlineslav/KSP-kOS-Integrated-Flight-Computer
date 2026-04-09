@LAZYGLOBAL OFF.

// ============================================================
// vtol_test.ks  -  VTOL Assist standalone test script
//
// Loads the minimum IFC module stack and runs the VTOL assist
// module in a live loop so you can verify hover behaviour on
// your 4-pod test aircraft before integrating into full IFC.
//
// Setup (KSP editor — part tags, not IR group names):
//   - Tag front-left  engine part:  vtol_eng_1
//   - Tag front-right engine part:  vtol_eng_2
//   - Tag rear-left   engine part:  vtol_eng_3
//   - Tag rear-right  engine part:  vtol_eng_4
//   - Tag front-left  servo  part:  vtol_srv_1
//   - Tag front-right servo  part:  vtol_srv_2
//   - Tag rear-left   servo  part:  vtol_srv_3
//   - Tag rear-right  servo  part:  vtol_srv_4
// (Tags are set in the part right-click menu in the VAB/SPH editor)
//
// Flying (only after ARM step):
//   Throttle  →  vertical speed command (50% = hold alt)
//   Pitch     →  fore/aft thrust differential
//   Roll      →  left/right thrust differential
//   Yaw       →  differential pod tilt
//
// Flow:
//   1) Script loads, runs geometry discovery, shows report.
//   2) Any key → proceed to ARM confirmation screen.
//   3) Press A → lock throttle at 100%, enter live VTOL loop.
//      Any other key → exit without arming.
//
// ABORT  →  releases VTOL, unlocks throttle, exits
// ============================================================

LOCAL ifc_root IS "0:/Integrated Flight Computer/".

// ── Load minimum module stack ──────────────────────────────
RUNPATH(ifc_root + "lib/ifc_constants.ks").
RUNPATH(ifc_root + "lib/ifc_state.ks").
RUNPATH(ifc_root + "lib/ifc_helpers.ks").
RUNPATH(ifc_root + "lib/ifc_autobrake.ks").
RUNPATH(ifc_root + "lib/ifc_amo.ks").
RUNPATH(ifc_root + "lib/ifc_amo_vtol.ks").
RUNPATH(ifc_root + "lib/ifc_engine_model.ks").
RUNPATH(ifc_root + "nav/nav_math.ks").

// ── Aircraft config ────────────────────────────────────────
// Edit these values to match your test aircraft.
// vtol_hover_angle: servo angle (degrees) when pods point DOWN.
// vtol_cruise_angle: servo angle when pods point FORWARD.
SET ACTIVE_AIRCRAFT TO LEXICON(
  "name",                   "VTOL Test Craft",
  "has_vtol",               TRUE,
  "has_nws",                FALSE,
  "vtol_eng_tag_prefix",    "vtol_eng",
  "vtol_srv_tag_prefix",    "vtol_srv",
  "vtol_hover_angle",       90,
  "vtol_cruise_angle",      0,
  "vtol_yaw_gain",          8,
  "vtol_roll_gain",         0.25,
  "vtol_pitch_gain",        0.30,
  "vtol_pitch_mix_sign",    1,   // +1 = positive pitch_cmd lifts the nose UP (more front engine thrust).r
                                  // a negative (nose-DOWN) corrective command. Do NOT confuse command sign with error sign.
  "vtol_level_roll_kp",     0.10,
  "vtol_level_roll_kd",     0.03,
  "vtol_level_roll_ki",     0.010,
  "vtol_level_pitch_kp",    0.10,  // softer P to avoid early rail-to-rail in low-damping transients
  "vtol_level_pitch_kd",    0.08,  // stronger D for rate damping as soon as feedback engages
  "vtol_level_pitch_ki",    0.015,
  // Cascade attitude->rate->command controller gains.
  "vtol_level_roll_att2rate_kp",  0.8,
  "vtol_level_pitch_att2rate_kp", 0.9,
  "vtol_level_roll_att2rate_ki",  0.00,
  "vtol_level_pitch_att2rate_ki", 0.00,
  "vtol_level_roll_rate_kp",      0.030,
  "vtol_level_pitch_rate_kp",     0.032,
  "vtol_level_roll_rate_cmd_max_degs",  6.0,
  "vtol_level_pitch_rate_cmd_max_degs", 6.5,
  "vtol_level_i_lim",       40.0,
  "vtol_lag_filter_tau_s",  0.90,  // smooth spool-lag scheduling to prevent tick-to-tick gain/slew jumps
  "vtol_level_gain_lag_ref_s", 0.8,
  "vtol_level_kp_min_scale", 0.45,
  "vtol_level_kd_min_scale", 0.45,
  "vtol_level_ki_min_scale", 0.20,
  "vtol_level_aw_lag_s",    0.60,
  "vtol_level_aw_eff_err_min", 0.05,
  "vtol_level_cmd_slew_per_s", 1.5, // feedback-command slew (prevents rail-to-rail flips under spool lag)
  "vtol_level_cmd_slew_lag_ref_s", 0.8,
  "vtol_level_cmd_slew_min_scale", 0.35,
  "vtol_level_on_ground",   FALSE,
  "vtol_level_min_agl_m",   0.8,
  "vtol_ground_contact_agl_m", 1.5,
  "vtol_ground_contact_vs_max", 0.7,
  "vtol_static_trim_discovery", TRUE,   // enabled: computes static trim to balance pitch torque on ground
  "vtol_diff_collective_min",   0.12,
  "vtol_engine_limit_floor",    0.14,  // prevents a single pod from being driven near-zero during aggressive recovery
  "vtol_cmd_slew_per_s",        3.0,   // smooth command reversals so allocator does not jump to saturation
  "vtol_cmd_roll_max",          0.55,  // reduce roll rail-induced coupling while pitch loop is recovering
  "vtol_cmd_pitch_max",         0.60,  // reduced to avoid rail-to-rail pitch forcing under lag
  "vtol_diff_atten_min",        0.25,  // keep more authority available during moderate excursions
  "vtol_diff_soft_bank_deg",    6.0,
  "vtol_diff_hard_bank_deg",    30.0,
  "vtol_diff_soft_pitch_deg",   6.0,
  "vtol_diff_hard_pitch_deg",   16.0,
  "vtol_diff_soft_roll_rate_degs", 12.0,
  "vtol_diff_hard_roll_rate_degs", 35.0,
  "vtol_diff_soft_pitch_rate_degs", 8.0,
  "vtol_diff_hard_pitch_rate_degs", 20.0,
  "vtol_upset_bank_deg",        18.0,
  "vtol_upset_pitch_deg",       12.0,  // trigger upset slightly earlier on pitch excursions
  "vtol_upset_roll_rate_degs",  18.0,
  "vtol_upset_pitch_rate_degs", 14.0,  // enter upset before pitch-rate runaway
  "vtol_upset_exit_bank_deg",   12.0,  // hysteresis: only exit once attitude/rates are clearly recovered
  "vtol_upset_exit_pitch_deg",  8.0,
  "vtol_upset_exit_roll_rate_degs", 10.0,
  "vtol_upset_exit_pitch_rate_degs", 8.0,
  "vtol_upset_hold_s",          2.0,   // minimum upset latch time to prevent chatter
  "vtol_upset_cmd_max",         0.30,  // during upset, force low-amplitude damping commands
  "vtol_upset_cmd_roll_max",    0.60,  // keep roll from saturating the mixer during upset
  "vtol_upset_cmd_pitch_max",   0.60,  // keep upset recovery aggressive but below saturating extremes
  "vtol_upset_roll_rate_kp",    0.030,
  "vtol_upset_pitch_rate_kp",   0.035,
  "vtol_upset_pitch_slew_bypass", TRUE, // do not delay sign reversal during upset recovery
  "vtol_upset_diff_atten_min",  0.55,  // do not collapse differential authority in upset
  "vtol_upset_engine_limit_floor", 0.02, // allow deeper per-engine cut in upset for roll torque
  "vtol_upset_guard_agl_m",     20.0,  // below this AGL, upset + low throttle is clamped to preserve lift
  "vtol_upset_guard_thr_min",   0.55,  // guard minimum pilot-throttle surrogate into VS-hold
  "vtol_rate_kd_roll_accel",    0.02,  // additional D term on roll angular acceleration
  "vtol_rate_kd_pitch_accel",   0.02,  // additional D term on pitch angular acceleration
  "vtol_rate_p_alpha",          0.70,
  "vtol_rate_q_alpha",          0.70,
  "vtol_rate_pdot_alpha",       0.35,
  "vtol_rate_qdot_alpha",       0.35,
  "vtol_rate_accel_clamp_degs2", 300.0,
  "vtol_cos_att_alpha",         0.10,  // geometry correction filter speed
  "vtol_cos_att_floor",         0.25,
  "vtol_trim_min_agl_m",        0.0,   // allow adaptive trim from first collective application
  "vtol_trim_rate",             0.003,
  "vtol_trim_roll_rate",        0.001,
  "vtol_trim_rate_lead_s",      0.0,
  "vtol_trim_activity_min",     0.0,
  "vtol_trim_min_offset",       -0.85,
  "vtol_trim_max_offset",       0.0,
  "vtol_trim_active_pitch_max", 8.0,
  "vtol_trim_active_rate_max",  12.0,
  "vtol_trim_bank_clamp",       10.0,
  "vtol_trim_active_bank_max",  8.0,
  "vtol_trim_active_roll_rate_max", 16.0,
  "vtol_static_trim_base_min",  0.23,   // lowered to allow discovery to find ~0.23 aft trim (torque balance)
  "vtol_vs_kp",             0.30,
  "vtol_vs_ki",             0.04,
  "vtol_vs_cmd_up_slew_mps2", 1.0,   // long-spool friendly VS setpoint ramp-up
  "vtol_vs_cmd_dn_slew_mps2", 1.4,   // allow slightly faster descent-command ramp
  "vtol_vs_cmd_lag_ref_s",    0.8,   // scheduler reference lag for setpoint slew
  "vtol_vs_cmd_slew_min_scale", 0.35,
  "vtol_vs_gain_lag_ref_s",   0.8,   // scheduler reference lag for VS PI gains
  "vtol_vs_kp_min_scale",     0.45,
  "vtol_vs_ki_min_scale",     0.25,
  "vtol_vs_aw_alpha_min",     0.95,  // unwind VS integrator when allocator is authority-limited
  "vtol_vs_aw_lag_s",         0.60,  // unwind VS integrator under high measured spool lag
  "vtol_vs_aw_eff_err_min",   0.05,  // requires meaningful command-vs-achieved mismatch
  "vtol_vs_i_unwind_per_s",   1.8,
  "vtol_max_vs",            6.0,
  "vtol_collective_max",    0.86,  // reserve more differential headroom to delay authority collapse
  "vtol_collective_up_slew_per_s", 0.35,
  "vtol_collective_dn_slew_per_s", 3.00,
  "vtol_alt_kp",            0.40,
  "vtol_hover_collective",  0.77,
  // Test mode toggle: set TRUE to bypass VS loop and hold fixed collective.
  "vtol_test_fixed_collective_enabled", FALSE,
  "vtol_test_fixed_collective", 0.77,
  // Engine-model feed-forward (collective lag compensation).
  "vtol_em_ff_enabled",     TRUE,
  "vtol_em_ff_gain",        0.75,
  "vtol_em_ff_max_lead",    0.20,
  "vtol_em_ff_lag_min_s",   0.10,
  "vtol_em_ff_alpha_min",   0.12,
  "vtol_vel_kp",            0.15,
  "vtol_vel_ki",            0.005,
  "vtol_vel_int_lim",       2.0,
  "vtol_vel_int_deadband",  0.5,
  "vtol_max_horiz_accel",   1.5,
  "vtol_max_horiz_speed",   8.0,
  "vtol_max_fwd_pitch",     15.0,
  "vtol_max_bank",          15.0,
  "vtol_pos_kp",            0.08,
  "vtol_pos_ki",            0.002,
  "vtol_pos_int_lim",       3.0,
  "vtol_pos_int_radius",    50.0,
  "vtol_pos_capture_radius",10.0,
  "vtol_khv_capture_mps",   0.5,
  "vtol_physical_alloc_enabled", TRUE,
  "vtol_trans_start_ias",   30.0,
  "vtol_trans_end_ias",     80.0,
  "vtol_nacelle_slew_dps",  5.0,
  "vtol_nacelle_alpha_min", 10.0,
  "vtol_nacelle_sin_floor", 0.10,
  // Keep feedback enabled for upstream controller validation runs.
  "vtol_bypass_attitude_feedback", FALSE
).

// ── State init ─────────────────────────────────────────────
IFC_INIT_STATE().
SET IFC_PHASE    TO PHASE_PREARM.
SET IFC_SUBPHASE TO "".
SET IFC_CYCLE_UT TO TIME:SECONDS.
SET IFC_RAW_DT   TO 0.05.
SET IFC_ACTUAL_DT TO 0.05.
SET THROTTLE_CMD TO 0.

// ── Abort handler ──────────────────────────────────────────
// Defined before arming so ABORT always exits cleanly.
LOCAL test_running IS TRUE.
ON ABORT {
  SET test_running TO FALSE.
  VTOL_RELEASE().
  PRINT "VTOL TEST: ABORT — limiters released." AT(0, 0).
  PRESERVE.
}

// ── Helper: right-justified number string ──────────────────
FUNCTION _FMT {
  PARAMETER val, width, decimals.
  LOCAL str_val IS ROUND(val, decimals):TOSTRING.
  UNTIL str_val:LENGTH >= width { SET str_val TO " " + str_val. }
  RETURN str_val.
}

FUNCTION _FMT_PCT {
  PARAMETER frac01.
  RETURN _FMT(ROUND(frac01 * 100, 1), 6, 1) + "%".
}

FUNCTION _CFG_GET_NUM {
  PARAMETER key_name, fallback_val.
  IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY(key_name) {
    RETURN ACTIVE_AIRCRAFT[key_name].
  }
  RETURN fallback_val.
}

// ── Helper: wrap long string to multiple PRINT lines ───────
FUNCTION _PRINT_WRAP {
  PARAMETER msg, start_row, max_width.
  LOCAL cur_row IS start_row.
  UNTIL msg:LENGTH = 0 {
    LOCAL chunk IS msg:SUBSTRING(0, MIN(max_width, msg:LENGTH)).
    PRINT (chunk + SPACE_PAD(max_width - chunk:LENGTH)) AT(0, cur_row).
    SET cur_row TO cur_row + 1.
    IF msg:LENGTH <= max_width { SET msg TO "". }
    ELSE { SET msg TO msg:SUBSTRING(max_width, msg:LENGTH - max_width). }
  }
  RETURN cur_row.
}

FUNCTION SPACE_PAD {
  PARAMETER cnt.
  LOCAL pad IS "".
  UNTIL pad:LENGTH >= cnt { SET pad TO pad + " ". }
  RETURN pad.
}

// ============================================================
// PHASE 1 — Geometry & IR diagnostic report (NO throttle changes)
// ============================================================

// ── Auto-stage to activate engines ────────────────────────
// Stage once immediately to fire whatever action group/stage
// has the engines. The trim_computed logic below waits for
// actual thrust before locking in geometry.
CLEARSCREEN.
PRINT "====== VTOL TEST — STAGING ============" AT(0, 0).
PRINT "Staging to activate engines..."         AT(0, 1).
STAGE.
WAIT 0.5.

VTOL_DISCOVER().

CLEARSCREEN.
PRINT "======================================" AT(0, 0).
PRINT " VTOL ASSIST TEST — GEOMETRY REPORT  " AT(0, 1).
PRINT "======================================" AT(0, 2).

// IR diagnostics first — helps diagnose servo discovery issues.
LOCAL diag_str IS VTOL_IR_DIAG().
LOCAL diag_row IS 3.
SET diag_row TO _PRINT_WRAP(diag_str, diag_row, 38).
PRINT "" AT(0, diag_row). SET diag_row TO diag_row + 1.

IF NOT VTOL_DIFF_AVAILABLE {
  PRINT "ERROR: VTOL discovery failed."           AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT "Need >= 2 engines tagged vtol_eng_1 .."  AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT "Engine tags found: " + VTOL_ENG_LIST:LENGTH AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT ""                                         AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT "Exiting in 5 seconds..."                 AT(0, diag_row).
  WAIT 5.
  SET test_running TO FALSE.
} ELSE {

  LOCAL srv_ct IS 0.
  LOCAL srv_idx IS 0.
  UNTIL srv_idx >= VTOL_SRV_LIST:LENGTH {
    IF VTOL_SRV_LIST[srv_idx] <> 0 { SET srv_ct TO srv_ct + 1. }
    SET srv_idx TO srv_idx + 1.
  }

  PRINT ("Engines: " + VTOL_ENG_LIST:LENGTH + "   Servos: " + srv_ct) AT(0, diag_row).
  SET diag_row TO diag_row + 1.

  IF srv_ct = 0 {
    PRINT "WARNING: No servos found. Check IR"    AT(0, diag_row). SET diag_row TO diag_row + 1.
    PRINT "group names match vtol_srv_N prefix."  AT(0, diag_row). SET diag_row TO diag_row + 1.
  }

  PRINT ""                                        AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT "  N  ROLL_MIX  PITCH_MIX  YAW_SRV"      AT(0, diag_row). SET diag_row TO diag_row + 1.
  PRINT "  -  --------  ---------  -------"       AT(0, diag_row). SET diag_row TO diag_row + 1.

  LOCAL gi IS 0.
  UNTIL gi >= VTOL_ENG_LIST:LENGTH {
    LOCAL tbl_row IS diag_row + gi.
    LOCAL rmix_f IS ROUND(VTOL_ROLL_MIX[gi],  3).
    LOCAL pmix_f IS ROUND(VTOL_PITCH_MIX[gi], 3).
    LOCAL ysign  IS VTOL_YAW_SRV_MIX[gi].
    LOCAL rmix_s IS ("" + rmix_f). UNTIL rmix_s:LENGTH >= 8 { SET rmix_s TO " " + rmix_s. }
    LOCAL pmix_s IS ("" + pmix_f). UNTIL pmix_s:LENGTH >= 9 { SET pmix_s TO " " + pmix_s. }
    LOCAL ys_s   IS ("" + ysign).  UNTIL ys_s:LENGTH >= 7  { SET ys_s   TO " " + ys_s.   }
    PRINT ("  " + (gi+1) + "  " + rmix_s + "  " + pmix_s + "  " + ys_s) AT(0, tbl_row).
    SET gi TO gi + 1.
  }

  LOCAL pause_row IS diag_row + VTOL_ENG_LIST:LENGTH + 1.
  PRINT "Auto-arming in 4 seconds..."             AT(0, pause_row).
  WAIT 4.

  // ============================================================
  // PRE-PHASE 3 — Re-discover with engines running to get
  // accurate MAXTHRUST so auto-trim offsets are correct.
  // (First discovery ran before engines fully spooled → MAXTHRUST=0
  //  → tau_p=0 → no trim → front/rear imbalance uncorrected.)
  // ============================================================

  CLEARSCREEN.
  PRINT "====== VTOL — CALIBRATION =============" AT(0, 0).
  PRINT "Re-scanning engines (should be running)" AT(0, 1).
  VTOL_RESET().
  VTOL_DISCOVER().

  LOCAL cal_row IS 3.

  // Warn if engines are still not producing thrust.
  LOCAL cal_all_zero IS TRUE.
  LOCAL cal_ck IS 0.
  UNTIL cal_ck >= VTOL_MAX_THRUST:LENGTH {
    IF VTOL_MAX_THRUST[cal_ck] > 0.01 { SET cal_all_zero TO FALSE. }
    SET cal_ck TO cal_ck + 1.
  }
  IF cal_all_zero {
    PRINT "WARNING: MAXTHRUST = 0 for all engines!" AT(0, cal_row). SET cal_row TO cal_row + 1.
    PRINT "Engines still spooling — that is OK."    AT(0, cal_row). SET cal_row TO cal_row + 1.
    SET cal_row TO cal_row + 1.
  }

  // Show per-engine max thrust, trim offset, and pitch mixing.
  // Trim offset is only nonzero when MAXTHRUST > 0 at discovery.
  PRINT "  N  MAXTHRUST_kN  TRIM_OFFSET  PITCH_MIX  " AT(0, cal_row). SET cal_row TO cal_row + 1.
  PRINT "  -  ------------  -----------  ---------  " AT(0, cal_row). SET cal_row TO cal_row + 1.
  LOCAL cal_ci IS 0.
  UNTIL cal_ci >= VTOL_ENG_LIST:LENGTH {
    LOCAL cal_mt IS _FMT(VTOL_MAX_THRUST[cal_ci], 12, 2).
    LOCAL cal_tr IS _FMT(VTOL_TRIM_OFFSET[cal_ci], 11, 4).
    LOCAL cal_pm IS _FMT(VTOL_PITCH_MIX[cal_ci], 9, 3).
    PRINT ("  " + (cal_ci+1) + "  " + cal_mt + "  " + cal_tr + "  " + cal_pm + "  ") AT(0, cal_row).
    SET cal_row TO cal_row + 1.
    SET cal_ci TO cal_ci + 1.
  }

  SET cal_row TO cal_row + 1.
  PRINT "Entering live loop in 3 seconds..."       AT(0, cal_row).
  WAIT 3.

    // ============================================================
    // PHASE 3 — Live VTOL loop
    // ============================================================

    SAS OFF.
    EM_INIT().

    // ── Log init ─────────────────────────────────────────────
    LOCAL log_dir IS "0:/Integrated Flight Computer/logs".
    LOCAL ctr_path IS log_dir + "/counter.txt".
    LOCAL log_seq IS 1.
    IF EXISTS(ctr_path) {
      LOCAL ctr_str IS OPEN(ctr_path):READALL:STRING:TRIM.
      IF ctr_str:LENGTH > 0 {
        LOCAL ctr_parsed IS ctr_str:TONUMBER(0).
        IF ctr_parsed >= 1 { SET log_seq TO ROUND(ctr_parsed). }
      }
    }
    IF EXISTS(ctr_path) { DELETEPATH(ctr_path). }
    LOG (log_seq + 1) TO ctr_path.
    LOCAL log_seq_s IS "" + log_seq.
    UNTIL log_seq_s:LENGTH >= 8 { SET log_seq_s TO "0" + log_seq_s. }
    LOCAL log_file IS log_dir + "/ifc_log_" + log_seq_s + ".csv".

    // ── Engine object list for actual thrust logging ──────────
    // SHIP:ENGINES is expensive; scan once here, not in the loop.
    LOCAL vtol_eng_objs IS LIST().
    LOCAL all_ship_engs IS SHIP:ENGINES.
    LOCAL eo_tag IS "vtol_eng".
    IF ACTIVE_AIRCRAFT:HASKEY("vtol_eng_tag_prefix") {
      SET eo_tag TO ACTIVE_AIRCRAFT["vtol_eng_tag_prefix"].
    }
    LOCAL eo_ni IS 1.
    UNTIL eo_ni > VTOL_MAX_PODS {
      LOCAL eo_tagged IS SHIP:PARTSTAGGED(eo_tag + "_" + eo_ni).
      IF eo_tagged:LENGTH = 0 { BREAK. }
      LOCAL eo_part IS eo_tagged[0].
      LOCAL found_eo IS 0.
      LOCAL eo_sei IS 0.
      UNTIL eo_sei >= all_ship_engs:LENGTH {
        LOCAL eo_cand IS all_ship_engs[eo_sei].
        IF eo_cand:UID = eo_part:UID { SET found_eo TO eo_cand. }
        SET eo_sei TO eo_sei + 1.
      }
      vtol_eng_objs:ADD(found_eo).
      SET eo_ni TO eo_ni + 1.
    }

    // Header: build engine and servo column names from discovery count.
    LOCAL hdr IS "t_s,dt_s,pitch_deg,bank_deg,hdg_deg,vs_ms,vs_cmd_ms,alt_agl_m," +
                 "collective,hover_coll,vs_integral," +
                 "pilot_roll,pilot_pitch,pilot_yaw,pilot_thr," +
                 "roll_cmd,pitch_cmd,pitch_rate_degs,roll_rate_degs," +
                 "gndspd_ms,ship_status,alloc_alpha,alloc_shift," +
                 "vtol_upset,vtol_thr_guard,vtol_thr_used," +
                 "vtol_level_active,vtol_truly_airborne,vtol_is_grounded," +
                 "vtol_roll_err,vtol_pitch_err,vtol_roll_p,vtol_roll_i,vtol_roll_d," +
                 "vtol_pitch_p,vtol_pitch_i,vtol_pitch_d,vtol_roll_unsat,vtol_pitch_unsat," +
                 "vtol_cmd_roll_precap,vtol_cmd_pitch_precap,vtol_cmd_roll_postcap,vtol_cmd_pitch_postcap," +
                 "vtol_cmd_roll_postupset,vtol_cmd_pitch_postupset,vtol_cmd_roll_postslew,vtol_cmd_pitch_postslew," +
                 "vtol_diff_raw,vtol_diff_attn,vtol_diff_upset,vtol_lim_floor," +
                 "vtol_alloc_bmin,vtol_alloc_bmax,vtol_alloc_alpha_limited,vtol_clamp_low_n,vtol_clamp_high_n," +
                 "vtol_roll_moment,vtol_pitch_moment,vtol_limit_span," +
                 "vtol_vel_hold,vtol_khv,vtol_pos_hold,vtol_trans_active," +
                 "vtol_phi_cmd_deg,vtol_theta_cmd_deg,vtol_vn_actual_ms,vtol_ve_actual_ms,vtol_vn_cmd_ms,vtol_ve_cmd_ms," +
                 "vtol_pos_err_n_m,vtol_pos_err_e_m,vtol_pos_err_dist_m,vtol_target_lat,vtol_target_lng,vtol_nacelle_cmd_deg,vtol_nacelle_est_deg,vtol_hover_blend," +
                 "vtol_pdot_filt,vtol_qdot_filt,vtol_roll_d_accel,vtol_pitch_d_accel," +
                 "vtol_cos_att_filt,vtol_coll_before_corr,vtol_coll_after_corr,vtol_physical_alloc_used," +
                 "em_mach,em_q_demand_u_s,em_q_supply_u_s,em_margin_u_s," +
                 "em_la_margin_u_s,em_spool_lag_s,em_starving,em_eng_count,em_intake_count".
    LOCAL hdr_ei IS 0.
    UNTIL hdr_ei >= VTOL_ENG_LIST:LENGTH {
      SET hdr TO hdr + ",eng" + (hdr_ei+1) + "_lim".
      SET hdr_ei TO hdr_ei + 1.
    }
    LOCAL hdr_tri IS 0.
    UNTIL hdr_tri >= VTOL_TRIM_OFFSET:LENGTH {
      SET hdr TO hdr + ",trim" + (hdr_tri+1).
      SET hdr_tri TO hdr_tri + 1.
    }
    LOCAL hdr_ti IS 0.
    UNTIL hdr_ti >= vtol_eng_objs:LENGTH {
      SET hdr TO hdr + ",eng" + (hdr_ti+1) + "_thrust_kn".
      SET hdr_ti TO hdr_ti + 1.
    }
    LOCAL hdr_si IS 0.
    UNTIL hdr_si >= VTOL_SRV_LIST:LENGTH {
      SET hdr TO hdr + ",srv" + (hdr_si+1) + "_pos".
      SET hdr_si TO hdr_si + 1.
    }
    LOG "# vtol_test  craft=" + SHIP:NAME + "  UT=" + ROUND(TIME:SECONDS,1) TO log_file.
    LOG hdr TO log_file.

    // ── One-time geometry snapshot ────────────────────────────
    // Log CoM world position and per-engine offsets from CoM in
    // ship body frame (fwd=toward nose, stbd=right, up=topvector).
    // Use this to verify engine placement relative to CoM.
    LOCAL geo_com      IS SHIP:POSITION.
    LOCAL geo_fwd_vec  IS SHIP:FACING:FOREVECTOR.
    LOCAL geo_stbd_vec IS SHIP:FACING:STARVECTOR.
    LOCAL geo_up_vec   IS SHIP:FACING:TOPVECTOR.
    LOG "# CoM_world: x=" + ROUND(geo_com:X,2) + "  y=" + ROUND(geo_com:Y,2) + "  z=" + ROUND(geo_com:Z,2) TO log_file.
    LOG "# CoM_geo: lat=" + ROUND(SHIP:LATITUDE,5) + "  lng=" + ROUND(SHIP:LONGITUDE,5) + "  alt=" + ROUND(SHIP:ALTITUDE,2) TO log_file.
    LOCAL geo_eng_tag IS "vtol_eng".
    IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("vtol_eng_tag_prefix") {
      SET geo_eng_tag TO ACTIVE_AIRCRAFT["vtol_eng_tag_prefix"].
    }
    LOCAL geo_i IS 1.
    UNTIL geo_i > VTOL_ENG_LIST:LENGTH {
      LOCAL geo_tagged IS SHIP:PARTSTAGGED(geo_eng_tag + "_" + geo_i).
      IF geo_tagged:LENGTH > 0 {
        LOCAL geo_p   IS geo_tagged[0].
        LOCAL geo_off IS geo_p:POSITION - geo_com.
        LOCAL geo_fwd  IS ROUND(VDOT(geo_off, geo_fwd_vec),  3).
        LOCAL geo_stbd IS ROUND(VDOT(geo_off, geo_stbd_vec), 3).
        LOCAL geo_up   IS ROUND(VDOT(geo_off, geo_up_vec),   3).
        LOCAL geo_pmix IS ROUND(VTOL_PITCH_MIX[geo_i - 1], 4).
        LOCAL geo_rmix IS ROUND(VTOL_ROLL_MIX[geo_i - 1],  4).
        LOG ("# eng" + geo_i + ": fwd=" + geo_fwd + "m  stbd=" + geo_stbd + "m  up=" + geo_up +
             "m  pitch_mix=" + geo_pmix + "  roll_mix=" + geo_rmix) TO log_file.
      } ELSE {
        LOG "# eng" + geo_i + ": tag not found" TO log_file.
      }
      SET geo_i TO geo_i + 1.
    }
    // ─────────────────────────────────────────────────────────

    LOCAL log_last_ut IS TIME:SECONDS.
    LOCAL log_rate    IS 0.2. // 5 Hz — enough resolution for VTOL tuning
    LOCAL arm_ut      IS TIME:SECONDS.

    // ----- Fixed feedback architecture (no timed in-flight mode switching) -----
    LOCAL att_stage_name IS "CASCADE_PD".
    LOCAL airborne_ut IS -1.
    LOCAL base_roll_kp IS _CFG_GET_NUM("vtol_level_roll_kp", 0.10).
    LOCAL base_pitch_kp IS _CFG_GET_NUM("vtol_level_pitch_kp", 0.12).
    LOCAL base_roll_kd IS _CFG_GET_NUM("vtol_level_roll_kd", 0.03).
    LOCAL base_pitch_kd IS _CFG_GET_NUM("vtol_level_pitch_kd", 0.06).
    // Fixed mode at loop entry.
    SET ACTIVE_AIRCRAFT["vtol_bypass_attitude_feedback"] TO FALSE.
    SET ACTIVE_AIRCRAFT["vtol_level_roll_kp"] TO base_roll_kp.
    SET ACTIVE_AIRCRAFT["vtol_level_pitch_kp"] TO base_pitch_kp.
    SET ACTIVE_AIRCRAFT["vtol_level_roll_kd"] TO base_roll_kd.
    SET ACTIVE_AIRCRAFT["vtol_level_pitch_kd"] TO base_pitch_kd.
    SET ACTIVE_AIRCRAFT["vtol_level_roll_ki"] TO 0.
    SET ACTIVE_AIRCRAFT["vtol_level_pitch_ki"] TO 0.
    LOG "# att_mode=CASCADE_PD_FIXED  t_s=0.0" TO log_file.

    UNTIL NOT test_running {
      LOCAL now IS TIME:SECONDS.
      SET IFC_RAW_DT TO MAX(now - IFC_CYCLE_UT, 0).
      SET IFC_ACTUAL_DT TO CLAMP(IFC_RAW_DT, 0.01, IFC_ACTUAL_DT_MAX).
      SET IFC_CYCLE_UT  TO now.

      // Live-loop re-discovery removed.
      // VTOL_DISCOVER() using THRUST fallback while engines are running
      // at asymmetric limiters produces severely wrong trim offsets —
      // it reads current thrust output (already limited) rather than
      // max capability, amplifying any existing imbalance.
      // Adaptive trim (_VTOL_ADAPT_TRIM) handles convergence instead.

      // ── Pilot inputs (read once, passed to tick + used by display + log) ────
      LOCAL roll_disp  IS 0.
      LOCAL pitch_disp IS 0.
      LOCAL yaw_disp   IS 0.
      LOCAL thr_disp   IS 0.5.
      IF SHIP:HASSUFFIX("CONTROL") {
        LOCAL ctrl IS SHIP:CONTROL.
        IF ctrl:HASSUFFIX("PILOTROLL")         { SET roll_disp  TO ctrl:PILOTROLL. }
        IF ctrl:HASSUFFIX("PILOTPITCH")        { SET pitch_disp TO ctrl:PILOTPITCH. }
        IF ctrl:HASSUFFIX("PILOTYAW")          { SET yaw_disp   TO ctrl:PILOTYAW. }
        IF ctrl:HASSUFFIX("PILOTMAINTHROTTLE") { SET thr_disp   TO ctrl:PILOTMAINTHROTTLE. }
      }

      LOCAL control_event_msg IS "".
      UNTIL NOT TERMINAL:INPUT:HASCHAR {
        LOCAL key_char IS TERMINAL:INPUT:GETCHAR().
        IF key_char = "h" OR key_char = "H" {
          SET VTOL_VEL_HOLD_ACTIVE TO NOT VTOL_VEL_HOLD_ACTIVE.
          IF NOT VTOL_VEL_HOLD_ACTIVE {
            SET VTOL_KHV_ACTIVE TO FALSE.
            SET VTOL_POS_HOLD_ACTIVE TO FALSE.
          }
          SET control_event_msg TO "VEL_HOLD=" + VTOL_VEL_HOLD_ACTIVE.
        } ELSE IF key_char = "k" OR key_char = "K" {
          SET VTOL_KHV_ACTIVE TO TRUE.
          SET VTOL_POS_HOLD_ACTIVE TO FALSE.
          SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
          SET VTOL_VEL_INT_N TO 0.0.
          SET VTOL_VEL_INT_E TO 0.0.
          SET control_event_msg TO "KHV=TRUE".
        } ELSE IF key_char = "p" OR key_char = "P" {
          IF VTOL_POS_HOLD_ACTIVE {
            SET VTOL_POS_HOLD_ACTIVE TO FALSE.
            SET control_event_msg TO "POS_HOLD=FALSE".
          } ELSE {
            SET VTOL_TARGET_LAT TO SHIP:LATITUDE.
            SET VTOL_TARGET_LNG TO SHIP:LONGITUDE.
            SET VTOL_TARGET_ALT TO GET_AGL().
            SET VTOL_POS_HOLD_ACTIVE TO TRUE.
            SET VTOL_KHV_ACTIVE TO FALSE.
            SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
            SET VTOL_POS_INT_N TO 0.0.
            SET VTOL_POS_INT_E TO 0.0.
            SET control_event_msg TO "POS_HOLD=TRUE".
          }
        } ELSE IF key_char = "t" OR key_char = "T" {
          SET VTOL_TRANS_ACTIVE TO NOT VTOL_TRANS_ACTIVE.
          SET control_event_msg TO "TRANS_ACTIVE=" + VTOL_TRANS_ACTIVE.
        }
      }

      LOCAL pos_err_dist_m IS 0.0.
      LOCAL pos_err_bearing_deg IS 0.0.
      LOCAL pos_err_north_m IS 0.0.
      LOCAL pos_err_east_m IS 0.0.
      LOCAL target_geo_now IS LATLNG(VTOL_TARGET_LAT, VTOL_TARGET_LNG).
      LOCAL current_geo_now IS SHIP:GEOPOSITION.
      SET pos_err_dist_m TO GEO_DISTANCE(current_geo_now, target_geo_now).
      SET pos_err_bearing_deg TO GEO_BEARING(current_geo_now, target_geo_now).
      SET pos_err_north_m TO pos_err_dist_m * COS(pos_err_bearing_deg).
      SET pos_err_east_m TO pos_err_dist_m * SIN(pos_err_bearing_deg).

      // Update engine model from last-cycle throttle command first, so
      // VTOL_TICK_PREARM can use fresh feed-forward telemetry this cycle.
      EM_TICK().
      VTOL_TICK_PREARM().
      // THROTTLE_CMD is the model input; no throttle lock is required.
      SET THROTTLE_CMD TO CLAMP(VTOL_COLLECTIVE, 0, 1).

      IF airborne_ut < 0 AND VTOL_DIAG_TRULY_AIRBORNE {
        SET airborne_ut TO now.
      }
      // Keep one fixed attitude-controller architecture for the full run.

      // ── Per-engine limits + command telemetry (actual values from VTOL module) ──
      LOCAL disp_ang_vel IS SHIP:ANGULARVEL.
      LOCAL disp_pitch_rate IS VDOT(disp_ang_vel, SHIP:FACING:STARVECTOR) * (180 / CONSTANT:PI).
      LOCAL disp_roll_rate  IS -VDOT(disp_ang_vel, SHIP:FACING:FOREVECTOR) * (180 / CONSTANT:PI).
      LOCAL disp_roll_cmd  IS VTOL_CMD_ROLL_ACTUAL.
      LOCAL disp_pitch_cmd IS VTOL_CMD_PITCH_ACTUAL.
      LOCAL eng_limits IS LIST().
      LOCAL disp_alloc_alpha IS VTOL_ALLOC_ALPHA.
      LOCAL disp_alloc_shift IS VTOL_ALLOC_SHIFT.
      LOCAL eng_ei IS 0.
      UNTIL eng_ei >= VTOL_ENG_LIST:LENGTH {
        LOCAL eng_lim IS 0.
        IF eng_ei < VTOL_ENG_LIM_ACTUAL:LENGTH {
          SET eng_lim TO VTOL_ENG_LIM_ACTUAL[eng_ei].
        }
        eng_limits:ADD(eng_lim).
        SET eng_ei TO eng_ei + 1.
      }

      // ── Servo positions (read once, used by display + log) ─
      LOCAL srv_pos_list IS LIST().
      LOCAL srv_pi IS 0.
      UNTIL srv_pi >= VTOL_SRV_LIST:LENGTH {
        LOCAL srv_pm IS VTOL_SRV_LIST[srv_pi].
        IF srv_pm <> 0 {
          srv_pos_list:ADD(ROUND(srv_pm:GETFIELD("current position"), 2)).
        } ELSE {
          srv_pos_list:ADD(-999).
        }
        SET srv_pi TO srv_pi + 1.
      }

      // ── Display ────────────────────────────────────────────
      LOCAL disp_row IS 0.
      PRINT "====== VTOL ASSIST LIVE ===============" AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL act_s  IS "NO ".
      IF VTOL_ACTIVE    { SET act_s  TO "YES". }
      LOCAL disc_s IS "NO ".
      IF VTOL_DISCOVERED { SET disc_s TO "YES". }
      PRINT ("Active: " + act_s + "  Discovered: " + disc_s + "          ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL vs_cmd_s IS _FMT(VTOL_VS_CMD,         6, 2).
      LOCAL vs_act_s IS _FMT(SHIP:VERTICALSPEED,   6, 2).
      PRINT ("VS Cmd:" + vs_cmd_s + " m/s   VS:" + vs_act_s + " m/s  ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL coll_s  IS _FMT_PCT(VTOL_COLLECTIVE).
      LOCAL hcoll_s IS _FMT_PCT(VTOL_HOVER_COLLECTIVE).
      PRINT ("Collective:" + coll_s + "   HoverColl:" + hcoll_s + "  ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL vsi_s IS _FMT(VTOL_VS_INTEGRAL, 7, 3).
      PRINT ("VS_I:" + vsi_s + "   Thr(pilot):" + _FMT_PCT(thr_disp) + "          ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT ("Thr(used):" + _FMT_PCT(VTOL_THR_INPUT_USED) + "  Guard:" + VTOL_THR_GUARD_ACTIVE +
             "  Upset:" + VTOL_UPSET_ACTIVE + "    ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT ("Modes VH:" + VTOL_VEL_HOLD_ACTIVE + " KHV:" + VTOL_KHV_ACTIVE +
             " POS:" + VTOL_POS_HOLD_ACTIVE + " TR:" + VTOL_TRANS_ACTIVE + "    ") AT(0, disp_row).
      SET disp_row TO disp_row + 1.
      PRINT ("VN/VE act:" + _FMT(VTOL_VN_ACTUAL,6,2) + "/" + _FMT(VTOL_VE_ACTUAL,6,2) +
             " cmd:" + _FMT(VTOL_VN_CMD,6,2) + "/" + _FMT(VTOL_VE_CMD,6,2) + " ") AT(0, disp_row).
      SET disp_row TO disp_row + 1.
      PRINT ("Pos err N/E:" + _FMT(pos_err_north_m,6,2) + "/" + _FMT(pos_err_east_m,6,2) +
             " D:" + _FMT(pos_err_dist_m,6,2) + "m ") AT(0, disp_row).
      SET disp_row TO disp_row + 1.
      PRINT ("Phi/Theta cmd:" + _FMT(VTOL_PHI_CMD,6,2) + "/" + _FMT(VTOL_THETA_CMD,6,2) +
             "  CosAtt:" + _FMT(VTOL_DIAG_COS_ATT_FILT,5,3) + "   ") AT(0, disp_row).
      SET disp_row TO disp_row + 1.
      PRINT ("Nac cmd/est:" + _FMT(VTOL_NACELLE_ALPHA_CMD,6,2) + "/" + _FMT(VTOL_NACELLE_ALPHA_EST,6,2) +
             "  HB:" + _FMT(VTOL_HOVER_BLEND,5,3) + "   ") AT(0, disp_row).
      SET disp_row TO disp_row + 1.
      PRINT ("Alloc: a=" + _FMT(disp_alloc_alpha, 5, 3) + "  s=" + _FMT(disp_alloc_shift, 6, 3) + "          ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      LOCAL t_air_disp IS 0.
      IF airborne_ut >= 0 { SET t_air_disp TO now - airborne_ut. }
      PRINT ("ATT:" + att_stage_name + "  t_air:" + _FMT(t_air_disp,5,1) + " s      ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT ("EM M:" + _FMT(TELEM_EM_MACH,5,3) + " D:" + _FMT(TELEM_EM_Q_DEMAND,6,2) +
             " S:" + _FMT(TELEM_EM_Q_SUPPLY,6,2) + " d:" + _FMT(TELEM_EM_MARGIN,6,2) +
             " L:" + _FMT(TELEM_EM_WORST_SPOOL_LAG,4,2) + " " + TELEM_EM_STARVING) AT(0, disp_row).
      SET disp_row TO disp_row + 1.

      PRINT ("Roll:" + _FMT(roll_disp,6,3) + "  Pitch:" + _FMT(pitch_disp,6,3) +
             "  Yaw:" + _FMT(yaw_disp,6,3) + "  Thr:" + _FMT(thr_disp,5,2) + "  ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      PRINT "---------------------------------------" AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT "  N  LIMIT      ROLL_MIX   PITCH_MIX  " AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL di IS 0.
      UNTIL di >= eng_limits:LENGTH {
        LOCAL rmix_s  IS _FMT(VTOL_ROLL_MIX[di],  8, 3).
        LOCAL pmix_s  IS _FMT(VTOL_PITCH_MIX[di], 9, 3).
        LOCAL limit_s IS _FMT_PCT(eng_limits[di]).
        PRINT ("  " + (di+1) + "  " + limit_s + "  " + rmix_s + "  " + pmix_s + "  ") AT(0, disp_row).
        SET disp_row TO disp_row + 1.
        SET di TO di + 1.
      }

      PRINT "---------------------------------------" AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL yaw_gain_d IS VTOL_YAW_SRV_GAIN.
      IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("vtol_yaw_gain") {
        SET yaw_gain_d TO ACTIVE_AIRCRAFT["vtol_yaw_gain"].
      }

      PRINT "  N  SRV_POS    CMD_ANG    YAW_MIX    " AT(0, disp_row). SET disp_row TO disp_row + 1.
      LOCAL svi IS 0.
      UNTIL svi >= VTOL_SRV_LIST:LENGTH {
        LOCAL srv IS VTOL_SRV_LIST[svi].
        LOCAL pos_s IS "  ---   ".
        LOCAL ang_s IS "  ---   ".
        IF srv <> 0 {
          SET pos_s TO _FMT(srv_pos_list[svi], 7, 1) + " ".
          LOCAL commanded IS VTOL_NACELLE_ALPHA_CMD + yaw_disp * yaw_gain_d * VTOL_YAW_SRV_MIX[svi].
          SET ang_s TO _FMT(commanded, 7, 1) + " ".
        }
        LOCAL ym_s IS _FMT(VTOL_YAW_SRV_MIX[svi], 7, 0).
        PRINT ("  " + (svi+1) + "  " + pos_s + "  " + ang_s + "  " + ym_s + "  ") AT(0, disp_row).
        SET disp_row TO disp_row + 1.
        SET svi TO svi + 1.
      }

      LOCAL log_file_short IS log_file:SUBSTRING(log_file:LENGTH - 22, 22).
      PRINT ("  Log: " + log_file_short + "         ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT ("  Last key: " + control_event_msg + "                    ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT "  [H VHold] [K KHV] [P Pos] [T Trans] [ABORT] " AT(0, disp_row). SET disp_row TO disp_row + 1.

      // ── Log write (rate-limited to log_rate seconds) ───────
      IF now - log_last_ut >= log_rate {
        SET log_last_ut TO now.

        // Attitude — computed directly from SHIP:FACING each cycle.
        // Do NOT use GET_PITCH() / GET_COMPASS_HDG() — those read
        // IFC_FACING_FWD / IFC_UP_VEC which are cached by the main
        // IFC loop and are never updated in this test script.
        LOCAL fwd_v    IS SHIP:FACING:FOREVECTOR.
        LOCAL star_v   IS SHIP:FACING:STARVECTOR.
        LOCAL up_v     IS SHIP:UP:VECTOR.
        LOCAL north_v  IS SHIP:NORTH:VECTOR.

        LOCAL pitch_l  IS ROUND(90 - VANG(fwd_v, up_v), 3).
        LOCAL bank_l   IS ROUND(ARCSIN(CLAMP(-VDOT(star_v, up_v), -1, 1)), 3).

        LOCAL fwd_h    IS VXCL(up_v, fwd_v):NORMALIZED.
        LOCAL east_v   IS VCRS(up_v, north_v):NORMALIZED.
        LOCAL hdg_raw  IS VANG(fwd_h, north_v:NORMALIZED).
        LOCAL hdg_l    IS hdg_raw.
        IF VDOT(fwd_h, east_v) < 0 { SET hdg_l TO 360 - hdg_raw. }
        SET hdg_l TO ROUND(hdg_l, 2).

        LOCAL row IS LIST(
          ROUND(now - arm_ut,            3),
          ROUND(IFC_ACTUAL_DT,           4),
          pitch_l,
          bank_l,
          hdg_l,
          ROUND(SHIP:VERTICALSPEED,      3),
          ROUND(VTOL_VS_CMD,             3),
          ROUND(GET_AGL(),               2),
          ROUND(VTOL_COLLECTIVE,         4),
          ROUND(VTOL_HOVER_COLLECTIVE,   4),
          ROUND(VTOL_VS_INTEGRAL,        4),
          ROUND(roll_disp,               4),
          ROUND(pitch_disp,              4),
          ROUND(yaw_disp,                4),
          ROUND(thr_disp,                4),
          ROUND(disp_roll_cmd,           4),
          ROUND(disp_pitch_cmd,          4),
          ROUND(disp_pitch_rate,         3),
          ROUND(disp_roll_rate,          3),
          ROUND(SHIP:GROUNDSPEED,        3),
          SHIP:STATUS,
          ROUND(disp_alloc_alpha,        4),
          ROUND(disp_alloc_shift,        4),
          VTOL_UPSET_ACTIVE,
          VTOL_THR_GUARD_ACTIVE,
          ROUND(VTOL_THR_INPUT_USED,     4),
          VTOL_DIAG_LEVEL_ACTIVE,
          VTOL_DIAG_TRULY_AIRBORNE,
          VTOL_DIAG_IS_GROUNDED,
          ROUND(VTOL_DIAG_ROLL_ERR,          4),
          ROUND(VTOL_DIAG_PITCH_ERR,         4),
          ROUND(VTOL_DIAG_ROLL_P,            4),
          ROUND(VTOL_DIAG_ROLL_I,            4),
          ROUND(VTOL_DIAG_ROLL_D,            4),
          ROUND(VTOL_DIAG_PITCH_P,           4),
          ROUND(VTOL_DIAG_PITCH_I,           4),
          ROUND(VTOL_DIAG_PITCH_D,           4),
          ROUND(VTOL_DIAG_ROLL_UNSAT,        4),
          ROUND(VTOL_DIAG_PITCH_UNSAT,       4),
          ROUND(VTOL_DIAG_CMD_ROLL_PRECAP,   4),
          ROUND(VTOL_DIAG_CMD_PITCH_PRECAP,  4),
          ROUND(VTOL_DIAG_CMD_ROLL_POSTCAP,  4),
          ROUND(VTOL_DIAG_CMD_PITCH_POSTCAP, 4),
          ROUND(VTOL_DIAG_CMD_ROLL_POSTUPSET,  4),
          ROUND(VTOL_DIAG_CMD_PITCH_POSTUPSET, 4),
          ROUND(VTOL_DIAG_CMD_ROLL_POSTSLEW, 4),
          ROUND(VTOL_DIAG_CMD_PITCH_POSTSLEW,4),
          ROUND(VTOL_DIAG_DIFF_SCALE_RAW,    4),
          ROUND(VTOL_DIAG_DIFF_SCALE_ATTN,   4),
          ROUND(VTOL_DIAG_DIFF_SCALE_UPSET,  4),
          ROUND(VTOL_DIAG_LIM_FLOOR_USE,     4),
          ROUND(VTOL_DIAG_ALLOC_BMIN,        4),
          ROUND(VTOL_DIAG_ALLOC_BMAX,        4),
          VTOL_DIAG_ALPHA_LIMITED,
          ROUND(VTOL_DIAG_CLAMP_LOW_COUNT,   0),
          ROUND(VTOL_DIAG_CLAMP_HIGH_COUNT,  0),
          ROUND(VTOL_DIAG_ROLL_MOMENT_PROXY, 6),
          ROUND(VTOL_DIAG_PITCH_MOMENT_PROXY,6),
          ROUND(VTOL_DIAG_LIMIT_SPAN,        6),
          VTOL_VEL_HOLD_ACTIVE,
          VTOL_KHV_ACTIVE,
          VTOL_POS_HOLD_ACTIVE,
          VTOL_TRANS_ACTIVE,
          ROUND(VTOL_PHI_CMD,                4),
          ROUND(VTOL_THETA_CMD,              4),
          ROUND(VTOL_VN_ACTUAL,              4),
          ROUND(VTOL_VE_ACTUAL,              4),
          ROUND(VTOL_VN_CMD,                 4),
          ROUND(VTOL_VE_CMD,                 4),
          ROUND(pos_err_north_m,             4),
          ROUND(pos_err_east_m,              4),
          ROUND(pos_err_dist_m,              4),
          ROUND(VTOL_TARGET_LAT,             7),
          ROUND(VTOL_TARGET_LNG,             7),
          ROUND(VTOL_NACELLE_ALPHA_CMD,      4),
          ROUND(VTOL_NACELLE_ALPHA_EST,      4),
          ROUND(VTOL_HOVER_BLEND,            4),
          ROUND(VTOL_DIAG_P_DOT_FILT,        4),
          ROUND(VTOL_DIAG_Q_DOT_FILT,        4),
          ROUND(VTOL_DIAG_ROLL_D_ACCEL,      4),
          ROUND(VTOL_DIAG_PITCH_D_ACCEL,     4),
          ROUND(VTOL_DIAG_COS_ATT_FILT,      5),
          ROUND(VTOL_DIAG_COLL_BEFORE_CORR,  5),
          ROUND(VTOL_DIAG_COLL_AFTER_CORR,   5),
          VTOL_DIAG_PHYSICAL_ALLOC_USED,
          ROUND(TELEM_EM_MACH,               4),
          ROUND(TELEM_EM_Q_DEMAND,           6),
          ROUND(TELEM_EM_Q_SUPPLY,           6),
          ROUND(TELEM_EM_MARGIN,             6),
          ROUND(TELEM_EM_LOOKAHEAD_MARGIN,   6),
          ROUND(TELEM_EM_WORST_SPOOL_LAG,    4),
          TELEM_EM_STARVING,
          TELEM_EM_ENG_COUNT,
          TELEM_EM_INTAKE_COUNT
        ).
        LOCAL li IS 0.
        UNTIL li >= eng_limits:LENGTH {
          row:ADD(ROUND(eng_limits[li], 4)).
          SET li TO li + 1.
        }
        LOCAL ltri IS 0.
        UNTIL ltri >= VTOL_TRIM_OFFSET:LENGTH {
          row:ADD(ROUND(VTOL_TRIM_OFFSET[ltri], 4)).
          SET ltri TO ltri + 1.
        }
        LOCAL lti IS 0.
        UNTIL lti >= vtol_eng_objs:LENGTH {
          LOCAL eo IS vtol_eng_objs[lti].
          IF eo <> 0 { row:ADD(ROUND(eo:THRUST, 2)). }
          ELSE        { row:ADD(-1). }
          SET lti TO lti + 1.
        }
        LOCAL lsi IS 0.
        UNTIL lsi >= srv_pos_list:LENGTH {
          row:ADD(srv_pos_list[lsi]).
          SET lsi TO lsi + 1.
        }
        LOG row:JOIN(",") TO log_file.
      }

      WAIT IFC_LOOP_DT.
    }

    // ── Clean exit ─────────────────────────────────────────
    LOG "# end T+" + ROUND(TIME:SECONDS - arm_ut, 1) TO log_file.
    VTOL_RELEASE().
    PRINT "VTOL test ended. Log: " + log_file AT(0, 0).

} // end discovery OK branch
