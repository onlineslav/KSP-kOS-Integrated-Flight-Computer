@LAZYGLOBAL OFF.

// ============================================================
// ifc_telemetry.ks  -  Integrated Flight Computer
// Rate-limited HUD display.
// ============================================================

// Call each loop cycle; internally rate-limits to IFC_TELEMETRY_PERIOD.
FUNCTION PRINT_TELEMETRY {
  LOCAL now IS TIME:SECONDS.
  IF (now - LAST_TELEM_UT) < IFC_TELEMETRY_PERIOD { RETURN. }
  SET LAST_TELEM_UT TO now.

  LOCAL t IS ROUND(now - IFC_MISSION_START_UT, 1).
  LOCAL ias IS ROUND(GET_IAS(), 1).
  LOCAL agl IS ROUND(GET_AGL(), 0).
  LOCAL vs  IS ROUND(SHIP:VERTICALSPEED, 1).
  LOCAL hdg IS ROUND(SHIP:HEADING, 1).
  LOCAL pitch_deg IS ROUND(GET_PITCH(), 1).
  LOCAL thr IS ROUND(THROTTLE_CMD, 2).

  // ── Line 1: flight state ─────────────────────────────────
  PRINT "IFC T+" + t + "  [" + IFC_PHASE + " / " + IFC_SUBPHASE + "]" AT (0,0).

  // ── Line 2: air data ─────────────────────────────────────
  PRINT "IAS " + ias + " m/s   AGL " + agl + " m   VS " + vs + " m/s   THR " + thr
      + "   FLP " + FLAPS_CURRENT_DETENT + "->" + FLAPS_TARGET_DETENT AT (0,1).

  // ── Line 3: attitude ─────────────────────────────────────
  PRINT "HDG " + hdg + " deg   PITCH " + pitch_deg + " deg                    " AT (0,2).

  // ── Line 4: ILS deviations (only meaningful during ILS_TRACK) ──
  IF IFC_SUBPHASE = SUBPHASE_ILS_TRACK OR IFC_PHASE = PHASE_FLARE {
    LOCAL loc  IS ROUND(ILS_LOC_DEV, 1).
    LOCAL gs   IS ROUND(ILS_GS_DEV, 1).
    LOCAL dist IS ROUND(ILS_DIST_M / 1000, 2).
    LOCAL loc_arrow IS ">".
    IF loc > 0 { SET loc_arrow TO "R". }
    ELSE IF loc < 0 { SET loc_arrow TO "L". }
    LOCAL gs_arrow IS "-".
    IF gs > 0 { SET gs_arrow TO "^". }
    ELSE IF gs < 0 { SET gs_arrow TO "v". }

    PRINT "LOC " + loc + " m [" + loc_arrow + "]   GS " + gs + " m [" + gs_arrow + "]   DIST " + dist + " km" AT (0,3).
  } ELSE IF IFC_SUBPHASE = SUBPHASE_FLY_TO_FIX AND ACTIVE_FIXES:LENGTH > 0 AND FIX_INDEX < ACTIVE_FIXES:LENGTH {
    LOCAL fix_id IS ACTIVE_FIXES[FIX_INDEX].
    LOCAL fix    IS GET_BEACON(fix_id).
    LOCAL dist_to_fix IS ROUND(GEO_DISTANCE(SHIP:GEOPOSITION, fix["ll"]) / 1000, 1).
    LOCAL brg_to_fix  IS ROUND(GEO_BEARING(SHIP:GEOPOSITION, fix["ll"]), 1).
    PRINT "FIX [" + fix_id + "]  dist " + dist_to_fix + " km   brg " + brg_to_fix + " deg          " AT (0,3).
  } ELSE {
    PRINT "                                                  " AT (0,3).
  }

  // ── Line 5/6: phase-specific debug internals ─────────────
  IF IFC_PHASE = PHASE_TAKEOFF {
    LOCAL v_r IS AC_PARAM("v_r", TAKEOFF_V_R_DEFAULT, 0.001).
    LOCAL v2  IS AC_PARAM("v2",  TAKEOFF_V2_DEFAULT,  0.001).
    PRINT "TO sub " + IFC_SUBPHASE
      + "  Vtgt " + ROUND(ACTIVE_V_TGT, 1)
      + "  VR " + ROUND(v_r, 1)
      + "  V2 " + ROUND(v2, 1)
      + "  FLP " + FLAPS_CURRENT_DETENT + "->" + FLAPS_TARGET_DETENT AT (0,4).
    PRINT "hdg " + ROUND(TO_RWY_HDG, 1)
      + "  steer " + ROUND(ROLLOUT_STEER_HDG, 1)
      + "  locCorr " + ROUND(TELEM_RO_LOC_CORR, 2)
      + "  pCmd " + ROUND(SHIP:CONTROL:PITCH, 2)
      + "  THR " + ROUND(THROTTLE_CMD, 2)
      + "  AGL " + agl AT (0,5).
  } ELSE IF IFC_PHASE = PHASE_APPROACH {
    LOCAL arm_left IS 0.
    IF APP_FINAL_ARM_UT >= 0 {
      SET arm_left TO MAX(APP_FINAL_CAPTURE_CONFIRM_S - (now - APP_FINAL_ARM_UT), 0).
    }
    PRINT "SPD " + APP_SPD_MODE
      + " Vtgt " + ROUND(ACTIVE_V_TGT, 1)
      + " Vbase " + ROUND(APP_BASE_V_TGT, 1)
      + " Vint " + ROUND(APP_VINT_TGT, 1)
      + " Vapp " + ROUND(ACTIVE_V_APP, 1)
      + " Vref " + ROUND(APP_VREF_TGT, 1) AT (0,4).
    PRINT "CAP L/G " + APP_LOC_CAP_OK + "/" + APP_GS_CAP_OK
      + " arm " + ROUND(arm_left, 2) + "s"
      + " SF " + ROUND(APP_SHORT_FINAL_FRAC, 2)
      + " D " + ROUND(APP_SPD_DIST_THR_M / 1000, 1) + "km"
      + " ThrI " + ROUND(THR_INTEGRAL, 2) AT (0,5).
  } ELSE IF IFC_PHASE = PHASE_FLARE {
    PRINT "FLARE tgtVS " + ROUND(TELEM_FLARE_TGT_VS, 2)
      + "  VS " + ROUND(SHIP:VERTICALSPEED, 2)
      + "  frac " + ROUND(TELEM_FLARE_FRAC, 2)
      + "  FPAcmd " + ROUND(FLARE_PITCH_CMD, 2) AT (0,4).
    PRINT "AOA " + ROUND(GET_AOA(), 2)
      + "  pitch " + pitch_deg
      + "  IAS " + ias
      + "  agl " + agl AT (0,5).
  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN OR IFC_PHASE = PHASE_ROLLOUT {
    PRINT "RO yaw tgt " + ROUND(TELEM_RO_YAW_TGT, 3)
      + " cmd " + ROUND(SHIP:CONTROL:YAW, 3)
      + " sc " + ROUND(TELEM_RO_YAW_SCALE, 2)
      + " gate " + ROUND(TELEM_RO_YAW_GATE, 0)
      + " hdgErr " + ROUND(TELEM_RO_HDG_ERR, 2) AT (0,4).
    PRINT "RO pitch tgt " + ROUND(TELEM_RO_PITCH_TGT, 2)
      + " err " + ROUND(TELEM_RO_PITCH_ERR, 2)
      + " ff " + ROUND(TELEM_RO_PITCH_FF, 3)
      + " cmd " + ROUND(SHIP:CONTROL:PITCH, 3)
      + " steerB " + ROUND(TELEM_STEER_BLEND, 2) AT (0,5).
  } ELSE {
    PRINT "                                                                                " AT (0,4).
    PRINT "                                                                                " AT (0,5).
  }

  // ── Line 7: low-level controller command summary ─────────
  IF IFC_PHASE = PHASE_APPROACH OR IFC_PHASE = PHASE_FLARE OR IFC_PHASE = PHASE_TAKEOFF {
    PRINT "AAcmd hdg " + ROUND(TELEM_AA_HDG_CMD, 1)
      + " fpa " + ROUND(TELEM_AA_FPA_CMD, 2)
      + " locCorr " + ROUND(TELEM_LOC_CORR, 2)
      + " gsCorr " + ROUND(TELEM_GS_CORR, 2) AT (0,6).
  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN OR IFC_PHASE = PHASE_ROLLOUT {
    PRINT "RO steerHdg " + ROUND(ROLLOUT_STEER_HDG, 1)
      + " blend " + ROUND(TELEM_STEER_BLEND, 2)
      + " locCorr " + ROUND(TELEM_RO_LOC_CORR, 2)
      + " rollAsst " + ROUND(TELEM_RO_ROLL_ASSIST, 0) AT (0,6).
  } ELSE {
    PRINT "                                                                                " AT (0,6).
  }

  // ── Line 8: runway / approach info ───────────────────────
  IF ACTIVE_ILS_ID <> "" {
    LOCAL v_tgt IS ACTIVE_V_APP.
    IF IFC_PHASE = PHASE_APPROACH { SET v_tgt TO ACTIVE_V_TGT. }
    PRINT "RWY " + ACTIVE_ILS_ID + "  hdg " + ACTIVE_RWY_HDG + "  GS " + ACTIVE_GS_ANGLE + " deg  Vtgt " + ROUND(v_tgt, 1) + " m/s" AT (0,7).
  } ELSE {
    PRINT "                                                                                " AT (0,7).
  }
}
