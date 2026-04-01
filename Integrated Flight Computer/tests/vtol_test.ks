@LAZYGLOBAL OFF.

// ============================================================
// vtol_test.ks  -  VTOL Assist standalone test script
//
// Loads the minimum IFC module stack and runs the VTOL assist
// module in a live loop so you can verify hover behaviour on
// your 4-pod test aircraft before integrating into full IFC.
//
// Setup (KSP editor):
//   - Tag front-left  engine part:  vtol_eng_1
//   - Tag front-right engine part:  vtol_eng_2
//   - Tag rear-left   engine part:  vtol_eng_3
//   - Tag rear-right  engine part:  vtol_eng_4
//   - Name front-left  IR servo group:  vtol_srv_1
//   - Name front-right IR servo group:  vtol_srv_2
//   - Name rear-left   IR servo group:  vtol_srv_3
//   - Name rear-right  IR servo group:  vtol_srv_4
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
  "vtol_srv_group_prefix",  "vtol_srv",
  "vtol_hover_angle",       90,
  "vtol_cruise_angle",      0,
  "vtol_yaw_gain",          8,
  "vtol_vs_kp",             0.30,
  "vtol_vs_ki",             0.04,
  "vtol_max_vs",            8.0,
  "vtol_alt_kp",            0.40
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
  UNLOCK THROTTLE.
  PRINT "VTOL TEST: ABORT — controls released." AT(0, 0).
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
  PRINT "Press any key to exit."                  AT(0, diag_row).
  WAIT UNTIL TERMINAL:INPUT:HASCHAR.
  IF TERMINAL:INPUT:HASCHAR { LOCAL _c IS TERMINAL:INPUT:GETCHAR(). }
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
  PRINT "Press any key to continue to ARM ..." AT(0, pause_row).
  WAIT UNTIL TERMINAL:INPUT:HASCHAR.
  IF TERMINAL:INPUT:HASCHAR { LOCAL _c2 IS TERMINAL:INPUT:GETCHAR(). }

  // ============================================================
  // PHASE 2 — Arm confirmation (throttle still untouched here)
  // ============================================================

  CLEARSCREEN.
  PRINT "======================================" AT(0, 0).
  PRINT "         VTOL ARM CONFIRMATION        " AT(0, 1).
  PRINT "======================================" AT(0, 2).
  PRINT ""                                       AT(0, 3).
  PRINT "Arming VTOL will:"                      AT(0, 4).
  PRINT "  - Lock physical throttle at 100%"     AT(0, 5).
  PRINT "  - Hand collective control to VTOL"    AT(0, 6).
  PRINT "  - Servos move to hover position"      AT(0, 7).
  PRINT ""                                       AT(0, 8).
  PRINT "Pilot throttle axis = VS command"       AT(0, 9).
  PRINT "  50% throttle  = hold vertical speed"  AT(0, 10).
  PRINT "  100% throttle = climb at max VS"      AT(0, 11).
  PRINT "  0%  throttle  = sink at max VS"       AT(0, 12).
  PRINT ""                                       AT(0, 13).
  PRINT "Press A to ARM VTOL."                   AT(0, 14).
  PRINT "Any other key to EXIT without arming."  AT(0, 15).

  WAIT UNTIL TERMINAL:INPUT:HASCHAR.
  LOCAL arm_key IS TERMINAL:INPUT:GETCHAR().

  IF arm_key <> "a" AND arm_key <> "A" {
    CLEARSCREEN.
    PRINT "Exiting without arming." AT(0, 0).
    WAIT 2.
    SET test_running TO FALSE.
  } ELSE {

    // ============================================================
    // PHASE 3 — Live VTOL loop (throttle locked at 100% here)
    // ============================================================

    // NOW lock throttle — not before. Physical throttle goes to
    // 100% and per-engine thrust limits carry all collective/attitude.
    SAS OFF.
    LOCK THROTTLE TO THROTTLE_CMD.
    SET THROTTLE_CMD TO 1.0.

    UNTIL NOT test_running {
      LOCAL now IS TIME:SECONDS.
      SET IFC_ACTUAL_DT TO CLAMP(now - IFC_CYCLE_UT, 0.01, 0.5).
      SET IFC_CYCLE_UT  TO now.

      VTOL_TICK_PREARM().

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
      LOCAL thr_s IS _FMT_PCT(THROTTLE_CMD).
      PRINT ("VS_I:" + vsi_s + "   Throttle:" + thr_s + " (locked)  ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      // Pilot inputs (display only — VTOL_TICK_PREARM already read them)
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
      PRINT ("Roll:" + _FMT(roll_disp,6,3) + "  Pitch:" + _FMT(pitch_disp,6,3) +
             "  Yaw:" + _FMT(yaw_disp,6,3) + "  Thr:" + _FMT(thr_disp,5,2) + "  ") AT(0, disp_row). SET disp_row TO disp_row + 1.

      PRINT "---------------------------------------" AT(0, disp_row). SET disp_row TO disp_row + 1.
      PRINT "  N  LIMIT      ROLL_MIX   PITCH_MIX  " AT(0, disp_row). SET disp_row TO disp_row + 1.

      LOCAL ei IS 0.
      UNTIL ei >= VTOL_ENG_LIST:LENGTH {
        LOCAL eng_thr IS VTOL_COLLECTIVE
                    + roll_disp  * VTOL_ROLL_MIX[ei]
                    + pitch_disp * VTOL_PITCH_MIX[ei]
                    + VTOL_TRIM_OFFSET[ei].
        SET eng_thr TO CLAMP(eng_thr, 0, 1).
        LOCAL rmix_s  IS _FMT(VTOL_ROLL_MIX[ei],  8, 3).
        LOCAL pmix_s  IS _FMT(VTOL_PITCH_MIX[ei], 9, 3).
        LOCAL limit_s IS _FMT_PCT(eng_thr).
        PRINT ("  " + (ei+1) + "  " + limit_s + "  " + rmix_s + "  " + pmix_s + "  ") AT(0, disp_row).
        SET disp_row TO disp_row + 1.
        SET ei TO ei + 1.
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

      PRINT "  N  SRV_ANGLE  YAW_MIX                " AT(0, disp_row). SET disp_row TO disp_row + 1.
      LOCAL svi IS 0.
      UNTIL svi >= VTOL_SRV_LIST:LENGTH {
        LOCAL srv IS VTOL_SRV_LIST[svi].
        LOCAL ang_s IS "  ---   ".
        IF srv <> 0 {
          LOCAL commanded IS hover_ang + yaw_disp * yaw_gain_d * VTOL_YAW_SRV_MIX[svi].
          SET ang_s TO _FMT(commanded, 7, 1) + "°".
        }
        LOCAL ym_s IS _FMT(VTOL_YAW_SRV_MIX[svi], 7, 0).
        PRINT ("  " + (svi+1) + "  " + ang_s + "  " + ym_s + "           ") AT(0, disp_row).
        SET disp_row TO disp_row + 1.
        SET svi TO svi + 1.
      }

      PRINT "  [ABORT to exit]                      " AT(0, disp_row). SET disp_row TO disp_row + 1.

      WAIT IFC_LOOP_DT.
    }

    // ── Clean exit ─────────────────────────────────────────
    VTOL_RELEASE().
    UNLOCK THROTTLE.
    PRINT "VTOL test ended." AT(0, 0).

  } // end arm branch
} // end discovery OK branch
