@LAZYGLOBAL OFF.

// ============================================================
// takeoff_vr_probe.ks - IFC testflight pipeline + probe hooks
//
// Pipeline:
//   ifc_testflight_bootloader.ks -> this script -> ifc_main RUN_TAKEOFF_IFC
//
// This file provides hook functions called by ifc_main during TAKEOFF:
//   VR_PROBE_INIT(tag)
//   VR_PROBE_TICK()
//   VR_PROBE_FINALIZE()
//
// It then launches IFC takeoff so probe + phase_takeoff run together.
// ============================================================

// Hook registration flag checked by ifc_main.
GLOBAL VR_PROBE_HOOKS_READY IS FALSE.
GLOBAL IFC_SKIP_INTERACTIVE IS FALSE.
GLOBAL IFC_AUTO_ARM_TAKEOFF IS FALSE.

// ----------------------------
// Probe state
// ----------------------------
GLOBAL VRP_ACTIVE         IS FALSE.
GLOBAL VRP_FAR_OK         IS FALSE.
GLOBAL VRP_LOG_PATH       IS "".
GLOBAL VRP_TAG            IS "".
GLOBAL VRP_START_UT       IS 0.

GLOBAL VRP_ROTATE_MARKED  IS FALSE.
GLOBAL VRP_ROTATE_T       IS -1.
GLOBAL VRP_ROTATE_IAS     IS -1.
GLOBAL VRP_ROTATE_AOA     IS -1.

GLOBAL VRP_AIRBORNE_ARM_UT IS -1.
GLOBAL VRP_LOF_MARKED      IS FALSE.
GLOBAL VRP_LOF_T           IS -1.
GLOBAL VRP_LOF_IAS         IS -1.
GLOBAL VRP_LOF_AOA         IS -1.
GLOBAL VRP_LOF_VS          IS -1.
GLOBAL VRP_LOF_LIFT_RATIO  IS -1.

// ----------------------------
// Tunables
// ----------------------------
GLOBAL VRP_ROTATE_AOA_TRIGGER IS 6.0.   // deg
GLOBAL VRP_ROTATE_IAS_MIN     IS 40.0.  // m/s ignore AoA spikes below this speed
GLOBAL VRP_AIRBORNE_AGL_MIN   IS 3.0.   // m
GLOBAL VRP_AIRBORNE_MIN_VS    IS 0.5.   // m/s
GLOBAL VRP_AIRBORNE_CONFIRM_S IS 0.4.   // s

GLOBAL VRP_LOF_FACTOR_ASSUMED IS 1.08.
GLOBAL VRP_VR_FACTOR_FROM_VS  IS 1.05.
GLOBAL VRP_ALPHA_CRIT_DEG     IS 18.0.  // fallback if aircraft cfg a_crit is unavailable

FUNCTION _VRP_ENSURE_LOG_DIR {
  // Use archive log directory (same pattern as IFC logger).
  IF NOT EXISTS("0:/Integrated Flight Computer") { CREATEDIR("0:/Integrated Flight Computer"). }
  IF NOT EXISTS("0:/Integrated Flight Computer/logs") { CREATEDIR("0:/Integrated Flight Computer/logs"). }
}

FUNCTION _VRP_GET_IAS {
  IF VRP_FAR_OK { RETURN ADDONS:FAR:IAS. }
  RETURN SHIP:AIRSPEED.
}

FUNCTION _VRP_GET_AOA {
  IF VRP_FAR_OK { RETURN ADDONS:FAR:AOA. }
  RETURN 0.
}

FUNCTION _VRP_GET_PHASE_LABEL {
  LOCAL label IS IFC_PHASE.
  IF IFC_SUBPHASE <> "" {
    SET label TO IFC_PHASE + ":" + IFC_SUBPHASE.
  }
  RETURN label.
}

FUNCTION VR_PROBE_INIT {
  PARAMETER tag.
  IF VRP_ACTIVE { RETURN. }

  SET VRP_TAG TO tag.
  SET VRP_FAR_OK TO ADDONS:AVAILABLE("FAR").

  // Prefer aircraft config a_crit when available; keep global as fallback.
  IF DEFINED ACTIVE_AIRCRAFT AND ACTIVE_AIRCRAFT <> 0 {
    IF ACTIVE_AIRCRAFT:HASKEY("a_crit") AND ACTIVE_AIRCRAFT["a_crit"] > 0 {
      SET VRP_ALPHA_CRIT_DEG TO ACTIVE_AIRCRAFT["a_crit"].
    }
    IF ACTIVE_AIRCRAFT:HASKEY("v_r") AND ACTIVE_AIRCRAFT["v_r"] > 0 {
      SET VRP_ROTATE_IAS_MIN TO ACTIVE_AIRCRAFT["v_r"] * 0.70.
    }
  }

  SET VRP_START_UT TO TIME:SECONDS.
  SET VRP_ROTATE_MARKED TO FALSE.
  SET VRP_ROTATE_T TO -1.
  SET VRP_ROTATE_IAS TO -1.
  SET VRP_ROTATE_AOA TO -1.

  SET VRP_AIRBORNE_ARM_UT TO -1.
  SET VRP_LOF_MARKED TO FALSE.
  SET VRP_LOF_T TO -1.
  SET VRP_LOF_IAS TO -1.
  SET VRP_LOF_AOA TO -1.
  SET VRP_LOF_VS TO -1.
  SET VRP_LOF_LIFT_RATIO TO -1.

  _VRP_ENSURE_LOG_DIR().
  LOCAL stamp IS ROUND(TIME:SECONDS, 0).
  LOCAL base IS "0:/Integrated Flight Computer/logs/vr_probe_" + stamp.
  LOCAL candidate IS base + ".csv".
  LOCAL seq IS 0.
  UNTIL NOT EXISTS(candidate) {
    SET seq TO seq + 1.
    SET candidate TO base + "_" + seq + ".csv".
  }
  SET VRP_LOG_PATH TO candidate.

  LOG "time_s,phase,ias_mps,aoa_deg,vs_mps,agl_m,lift_kN,weight_kN,lift_ratio,vs1g_est_mps" TO VRP_LOG_PATH.
  LOG "#META,tag=" + tag + ",far_ok=" + VRP_FAR_OK + ",alpha_crit_deg=" + ROUND(VRP_ALPHA_CRIT_DEG,3) TO VRP_LOG_PATH.

  SET VRP_ACTIVE TO TRUE.

  PRINT "VR PROBE: armed (" + tag + ")".
  PRINT "VR PROBE: alpha_crit = " + ROUND(VRP_ALPHA_CRIT_DEG,2) + " deg".
  PRINT "VR PROBE: logging -> " + VRP_LOG_PATH.
}

FUNCTION VR_PROBE_TICK {
  IF NOT VRP_ACTIVE { RETURN. }

  LOCAL now IS TIME:SECONDS.
  LOCAL t_s IS now - VRP_START_UT.

  LOCAL ias IS _VRP_GET_IAS().
  LOCAL aoa IS _VRP_GET_AOA().
  LOCAL vs IS SHIP:VERTICALSPEED.
  LOCAL agl IS ALT:RADAR.

  LOCAL weight_kN IS SHIP:MASS * 9.80665.
  LOCAL lift_kN IS 0.
  IF VRP_FAR_OK {
    SET lift_kN TO VDOT(ADDONS:FAR:AEROFORCE, SHIP:UP:VECTOR).
  }

  LOCAL lift_ratio IS 0.
  IF weight_kN > 0 { SET lift_ratio TO lift_kN / weight_kN. }

  LOCAL vs1g_est IS -1.
  IF lift_ratio > 0.05 {
    SET vs1g_est TO ias / SQRT(lift_ratio).
  }

  // Rotation event: either AoA trigger or explicit IFC rotate subphase.
  IF NOT VRP_ROTATE_MARKED AND ias >= VRP_ROTATE_IAS_MIN AND (aoa >= VRP_ROTATE_AOA_TRIGGER OR IFC_SUBPHASE = SUBPHASE_TO_ROTATE) {
    SET VRP_ROTATE_MARKED TO TRUE.
    SET VRP_ROTATE_T TO t_s.
    SET VRP_ROTATE_IAS TO ias.
    SET VRP_ROTATE_AOA TO aoa.
    PRINT "VR PROBE: ROTATE t=" + ROUND(t_s, 2)
      + "  IAS=" + ROUND(ias, 1)
      + "  AoA=" + ROUND(aoa, 1).
    LOG "#ROTATE,t_s=" + ROUND(t_s,3)
      + ",ias_mps=" + ROUND(ias,3)
      + ",aoa_deg=" + ROUND(aoa,3) TO VRP_LOG_PATH.
  }

  // Liftoff event with debounce.
  IF agl > VRP_AIRBORNE_AGL_MIN AND vs > VRP_AIRBORNE_MIN_VS {
    IF VRP_AIRBORNE_ARM_UT < 0 { SET VRP_AIRBORNE_ARM_UT TO now. }
    IF NOT VRP_LOF_MARKED AND now - VRP_AIRBORNE_ARM_UT >= VRP_AIRBORNE_CONFIRM_S {
      SET VRP_LOF_MARKED TO TRUE.
      SET VRP_LOF_T TO t_s.
      SET VRP_LOF_IAS TO ias.
      SET VRP_LOF_AOA TO aoa.
      SET VRP_LOF_VS TO vs.
      SET VRP_LOF_LIFT_RATIO TO lift_ratio.
      PRINT "VR PROBE: LIFTOFF t=" + ROUND(t_s, 2)
        + "  IAS=" + ROUND(ias, 1)
        + "  AoA=" + ROUND(aoa, 1)
        + "  VS=" + ROUND(vs, 1).
      LOG "#LIFTOFF,t_s=" + ROUND(t_s,3)
        + ",ias_mps=" + ROUND(ias,3)
        + ",aoa_deg=" + ROUND(aoa,3)
        + ",vs_mps=" + ROUND(vs,3)
        + ",lift_ratio=" + ROUND(lift_ratio,5) TO VRP_LOG_PATH.
    }
  } ELSE {
    SET VRP_AIRBORNE_ARM_UT TO -1.
  }

  LOG ROUND(t_s,3) + "," + _VRP_GET_PHASE_LABEL()
    + "," + ROUND(ias,3)
    + "," + ROUND(aoa,3)
    + "," + ROUND(vs,3)
    + "," + ROUND(agl,3)
    + "," + ROUND(lift_kN,3)
    + "," + ROUND(weight_kN,3)
    + "," + ROUND(lift_ratio,5)
    + "," + ROUND(vs1g_est,3) TO VRP_LOG_PATH.
}

FUNCTION VR_PROBE_FINALIZE {
  IF NOT VRP_ACTIVE { RETURN. }
  SET VRP_ACTIVE TO FALSE.

  PRINT " ".
  PRINT "VR PROBE SUMMARY".

  IF VRP_ROTATE_MARKED {
    PRINT "  Rotate:  t=" + ROUND(VRP_ROTATE_T,2)
      + "  IAS=" + ROUND(VRP_ROTATE_IAS,1)
      + "  AoA=" + ROUND(VRP_ROTATE_AOA,1).
  } ELSE {
    PRINT "  Rotate:  not detected.".
  }

  IF VRP_LOF_MARKED {
    PRINT "  Liftoff: t=" + ROUND(VRP_LOF_T,2)
      + "  IAS=" + ROUND(VRP_LOF_IAS,1)
      + "  AoA=" + ROUND(VRP_LOF_AOA,1)
      + "  VS=" + ROUND(VRP_LOF_VS,1).

    LOCAL vs_from_lof IS VRP_LOF_IAS / VRP_LOF_FACTOR_ASSUMED.

    LOCAL vs_from_lift IS -1.
    IF VRP_LOF_LIFT_RATIO > 0.05 {
      SET vs_from_lift TO VRP_LOF_IAS / SQRT(VRP_LOF_LIFT_RATIO).
    }

    LOCAL vs_from_alpha IS -1.
    IF VRP_FAR_OK AND VRP_ALPHA_CRIT_DEG > 0.1 AND VRP_LOF_AOA > 0.1 {
      SET vs_from_alpha TO VRP_LOF_IAS * SQRT(VRP_LOF_AOA / VRP_ALPHA_CRIT_DEG).
    }

    LOCAL vs_to_est IS vs_from_lof.
    IF vs_from_lift > vs_to_est { SET vs_to_est TO vs_from_lift. }
    IF vs_from_alpha > vs_to_est { SET vs_to_est TO vs_from_alpha. }

    LOCAL vr_next IS vs_to_est * VRP_VR_FACTOR_FROM_VS.

    PRINT "  Vs_TO est (V_LOF/factor): " + ROUND(vs_from_lof,2) + " m/s".
    IF vs_from_lift > 0 {
      PRINT "  Vs_TO est (lift ratio):   " + ROUND(vs_from_lift,2) + " m/s".
    }
    IF vs_from_alpha > 0 {
      PRINT "  Vs_TO est (AoA ratio):    " + ROUND(vs_from_alpha,2) + " m/s".
    }
    PRINT "  Conservative Vs_TO est:   " + ROUND(vs_to_est,2) + " m/s".
    PRINT "  Suggested next V_R:       " + ROUND(vr_next,2) + " m/s".

    LOG "#EST,vs_from_lof=" + ROUND(vs_from_lof,3)
      + ",vs_from_lift=" + ROUND(vs_from_lift,3)
      + ",vs_from_alpha=" + ROUND(vs_from_alpha,3)
      + ",vs_to_est=" + ROUND(vs_to_est,3)
      + ",vr_next=" + ROUND(vr_next,3) TO VRP_LOG_PATH.
  } ELSE {
    PRINT "  Liftoff not detected.".
    LOG "#RESULT,liftoff_not_detected=1" TO VRP_LOG_PATH.
  }

  PRINT "  Log: " + VRP_LOG_PATH.
}

FUNCTION _PROMPT_RWY {
  PRINT "Select takeoff runway:".
  PRINT "  1 = RWY 09".
  PRINT "  2 = RWY 27".
  TERMINAL:INPUT:CLEAR().
  LOCAL sel IS "".
  UNTIL sel = "1" OR sel = "2" {
    SET sel TO TERMINAL:INPUT:GETCHAR().
  }
  IF sel = "1" { RETURN "09". }
  RETURN "27".
}

FUNCTION RUN_VR_PROBE_IFC_TAKEOFF {
  CLEARSCREEN.
  PRINT "===============================================".
  PRINT "IFC TESTFLIGHT: TAKEOFF V_R PROBE + AUTOTAKEOFF".
  PRINT "===============================================".
  PRINT " ".
  PRINT "This run executes IFC takeoff and logs V_R probe data.".
  PRINT " ".

  LOCAL rwy_id IS _PROMPT_RWY().
  PRINT " ".
  PRINT "Arm testflight pipeline for RWY " + rwy_id + "? (y/n)".
  TERMINAL:INPUT:CLEAR().
  LOCAL arm IS "".
  UNTIL arm = "y" OR arm = "Y" OR arm = "n" OR arm = "N" {
    SET arm TO TERMINAL:INPUT:GETCHAR().
  }
  IF arm = "n" OR arm = "N" {
    PRINT "Testflight disarmed.".
    WAIT 0.
    QUIT.
  }

  // Load IFC without entering its interactive menu.
  SET IFC_SKIP_INTERACTIVE TO TRUE.
  LOCAL ifc_main IS "0:/Integrated Flight Computer/ifc_main.ks".
  IF NOT EXISTS(ifc_main) {
    PRINT "ERROR: missing " + ifc_main.
    WAIT 0.
    QUIT.
  }

  RUNONCEPATH(ifc_main).

  SET IFC_AUTO_ARM_TAKEOFF TO TRUE.
  SET VR_PROBE_HOOKS_READY TO TRUE.
  PRINT "Launching RUN_TAKEOFF_IFC(" + rwy_id + ")...".
  RUN_TAKEOFF_IFC(rwy_id).
  SET VR_PROBE_HOOKS_READY TO FALSE.
  SET IFC_AUTO_ARM_TAKEOFF TO FALSE.
  SET IFC_SKIP_INTERACTIVE TO FALSE.
  PRINT "Testflight pipeline complete.".
}

RUN_VR_PROBE_IFC_TAKEOFF().
