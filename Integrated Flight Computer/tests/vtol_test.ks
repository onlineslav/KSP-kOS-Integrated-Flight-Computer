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
  "vtol_pitch_mix_sign",    1,   // +1: forward engines increase on nose-up cmd, aft engines decrease = corrective (CORRECT for fwd engines ahead of CoM)
  "vtol_level_roll_kp",     0.10,
  "vtol_level_roll_kd",     0.03,
  "vtol_level_roll_ki",     0.010,
  "vtol_level_pitch_kp",    0.12,
  "vtol_level_pitch_kd",    0.06,
  "vtol_level_pitch_ki",    0.015,
  "vtol_level_i_lim",       40.0,
  "vtol_level_on_ground",   FALSE,
  "vtol_level_min_agl_m",   0.8,
  "vtol_ground_contact_agl_m", 1.5,
  "vtol_ground_contact_vs_max", 0.7,
  "vtol_static_trim_discovery", FALSE,  // disabled: slow-spool jets can't recover from large startup offsets
  "vtol_diff_collective_min",   0.12,
  "vtol_trim_min_agl_m",        0.0,   // allow adaptive trim from first collective application
  "vtol_trim_rate",             0.003,
  "vtol_trim_roll_rate",        0.001,
  "vtol_trim_rate_lead_s",      0.25,
  "vtol_trim_activity_min",     0.35,
  "vtol_trim_min_offset",       -0.85,
  "vtol_trim_max_offset",       0.0,
  "vtol_trim_active_pitch_max", 8.0,
  "vtol_trim_active_rate_max",  12.0,
  "vtol_trim_bank_clamp",       10.0,
  "vtol_trim_active_bank_max",  8.0,
  "vtol_trim_active_roll_rate_max", 16.0,
  "vtol_static_trim_base_min",  0.30,
  "vtol_vs_kp",             0.30,
  "vtol_vs_ki",             0.04,
  "vtol_max_vs",            6.0,
  "vtol_collective_max",    1.0,   // raised: allows VS hold to compensate when differential reduces total thrust
  "vtol_collective_up_slew_per_s", 0.35,
  "vtol_collective_dn_slew_per_s", 3.00,
  "vtol_alt_kp",            0.40,
  "vtol_hover_collective",  0.77
).

// ── State init ─────────────────────────────────────────────
IFC_INIT_STATE().
SET IFC_PHASE    TO PHASE_PREARM.
SET IFC_SUBPHASE TO "".
SET IFC_CYCLE_UT TO TIME:SECONDS.

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
                 "gndspd_ms,ship_status,alloc_alpha,alloc_shift".
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

    UNTIL NOT test_running {
      LOCAL now IS TIME:SECONDS.
      SET IFC_ACTUAL_DT TO CLAMP(now - IFC_CYCLE_UT, 0.01, 0.5).
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

      VTOL_TICK_PREARM().

      // ── Per-engine limits (computed once, used by display + log) ──
      // Use the same roll_cmd / pitch_cmd that VTOL_TICK_PREARM used:
      // wings-level correction when pilot input is below deadband.
      LOCAL disp_ang_vel IS SHIP:ANGULARVEL.
      LOCAL disp_pitch_rate IS VDOT(disp_ang_vel, SHIP:FACING:STARVECTOR) * (180 / CONSTANT:PI).
      LOCAL disp_roll_rate  IS -VDOT(disp_ang_vel, SHIP:FACING:FOREVECTOR) * (180 / CONSTANT:PI).
      LOCAL disp_roll_cmd  IS roll_disp.
      LOCAL disp_pitch_cmd IS pitch_disp.
      LOCAL disp_level_roll_kp  IS _AMO_CFG_NUM("vtol_level_roll_kp", VTOL_LEVEL_ROLL_KP, 0).
      LOCAL disp_level_roll_kd  IS _AMO_CFG_NUM("vtol_level_roll_kd", VTOL_LEVEL_ROLL_KD, 0).
      LOCAL disp_level_roll_ki  IS _AMO_CFG_NUM("vtol_level_roll_ki", VTOL_LEVEL_ROLL_KI, 0).
      LOCAL disp_level_pitch_kp IS _AMO_CFG_NUM("vtol_level_pitch_kp", VTOL_LEVEL_PITCH_KP, 0).
      LOCAL disp_level_pitch_kd IS _AMO_CFG_NUM("vtol_level_pitch_kd", VTOL_LEVEL_PITCH_KD, 0).
      LOCAL disp_level_pitch_ki IS _AMO_CFG_NUM("vtol_level_pitch_ki", VTOL_LEVEL_PITCH_KI, 0).
      LOCAL disp_level_on_ground IS _AMO_CFG_BOOL("vtol_level_on_ground", VTOL_LEVEL_ON_GROUND_DEFAULT).
      LOCAL disp_level_min_agl IS _AMO_CFG_NUM("vtol_level_min_agl_m", VTOL_LEVEL_MIN_AGL_M, 0).
      LOCAL disp_ground_agl IS _AMO_CFG_NUM("vtol_ground_contact_agl_m", VTOL_GROUND_CONTACT_AGL_M, 0).
      LOCAL disp_is_grounded IS _VTOL_EFFECTIVE_GROUNDED(disp_ground_agl).
      LOCAL disp_level_active IS TRUE.
      IF NOT disp_level_on_ground {
        IF disp_is_grounded OR GET_AGL() < disp_level_min_agl {
          SET disp_level_active TO FALSE.
        }
      }
      IF disp_level_active AND ABS(roll_disp) < VTOL_TRIM_DEADBAND {
        LOCAL disp_bank IS ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1)).
        SET disp_roll_cmd TO CLAMP(
          -disp_bank * disp_level_roll_kp
          + disp_roll_rate * disp_level_roll_kd
          + VTOL_LEVEL_ROLL_INT * disp_level_roll_ki,
          -1, 1).
      }
      IF disp_level_active AND ABS(pitch_disp) < VTOL_TRIM_DEADBAND {
        LOCAL disp_pitch_ang IS 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
        SET disp_pitch_cmd TO CLAMP(
          -disp_pitch_ang * disp_level_pitch_kp
          + disp_pitch_rate * disp_level_pitch_kd
          + VTOL_LEVEL_PITCH_INT * disp_level_pitch_ki,
          -1, 1).
      }
      LOCAL eng_limits IS LIST().
      LOCAL disp_alloc_alpha IS VTOL_ALLOC_ALPHA.
      LOCAL disp_alloc_shift IS VTOL_ALLOC_SHIFT.
      LOCAL disp_diff_min IS _AMO_CFG_NUM("vtol_diff_collective_min", VTOL_DIFF_COLLECTIVE_MIN, 0).
      LOCAL disp_diff_scale IS 1.
      IF VTOL_COLLECTIVE <= disp_diff_min {
        SET disp_diff_scale TO 0.
      } ELSE {
        LOCAL disp_span IS 1.0 - disp_diff_min.
        IF disp_span < 0.01 { SET disp_span TO 0.01. }
        SET disp_diff_scale TO CLAMP((VTOL_COLLECTIVE - disp_diff_min) / disp_span, 0, 1).
      }
      LOCAL eng_ei IS 0.
      UNTIL eng_ei >= VTOL_ENG_LIST:LENGTH {
        LOCAL disp_base IS VTOL_COLLECTIVE * (1.0 + VTOL_TRIM_OFFSET[eng_ei]).
        LOCAL disp_diff IS disp_diff_scale * (disp_roll_cmd  * VTOL_ROLL_MIX[eng_ei]
                                            + disp_pitch_cmd * VTOL_PITCH_MIX[eng_ei]).
        LOCAL eng_lim IS disp_base + disp_alloc_shift + disp_alloc_alpha * disp_diff.
        eng_limits:ADD(CLAMP(eng_lim, 0, 1)).
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
      PRINT ("Alloc: a=" + _FMT(disp_alloc_alpha, 5, 3) + "  s=" + _FMT(disp_alloc_shift, 6, 3) + "          ") AT(0, disp_row). SET disp_row TO disp_row + 1.

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

      LOCAL hover_ang IS 90.
      IF ACTIVE_AIRCRAFT <> 0 AND ACTIVE_AIRCRAFT:HASKEY("vtol_hover_angle") {
        SET hover_ang TO ACTIVE_AIRCRAFT["vtol_hover_angle"].
      }
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
          LOCAL commanded IS hover_ang + yaw_disp * yaw_gain_d * VTOL_YAW_SRV_MIX[svi].
          SET ang_s TO _FMT(commanded, 7, 1) + " ".
        }
        LOCAL ym_s IS _FMT(VTOL_YAW_SRV_MIX[svi], 7, 0).
        PRINT ("  " + (svi+1) + "  " + pos_s + "  " + ang_s + "  " + ym_s + "  ") AT(0, disp_row).
        SET disp_row TO disp_row + 1.
        SET svi TO svi + 1.
      }

      LOCAL log_file_short IS log_file:SUBSTRING(log_file:LENGTH - 22, 22).
      PRINT ("  Log: " + log_file_short + "         ") AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT "  [ABORT to exit]                      " AT(0, disp_row). SET disp_row TO disp_row + 1.

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
          ROUND(disp_alloc_shift,        4)
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
