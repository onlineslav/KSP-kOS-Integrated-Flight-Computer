@LAZYGLOBAL OFF.

// ============================================================
// ifc_ascent_state.ks  -  Integrated Flight Computer
// Layer 1 (State Estimator) and Layer 2 (Mode Valuation) for
// spaceplane ascent guidance.
//
// Design constraints (see spaceplane_guidance_paper_v3.md):
//   - All vector arithmetic in the orbital (inertial) frame.
//   - Control signals and valuation signals kept in separate EMA copies.
//   - J_ab and J_rk computed exclusively from valuation-smoothed signals.
//   - Dedicated heavy filter on drag term inside J_rk.
//   - Estimator validity gates all mode-switch decisions.
//
// Public API
// ----------
//   ASC_STATE_INIT()       â€” call once when entering PHASE_ASCENT
//   ASC_STATE_UPDATE(dt)   â€” call every cycle; writes all ASC_* globals
// ============================================================

// â”€â”€ Engine classification â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

// Returns TRUE when this engine consumes intake air (is air-breathing).
// Checks CONSUMEDRESOURCES for any key containing "Air" or "Intake".
// Engines should be in their initial AB mode when this is called.
FUNCTION _ASC_IS_AB_ENGINE {
  PARAMETER eng.
  LOCAL cr IS eng:CONSUMEDRESOURCES.
  LOCAL k IS 0.
  UNTIL k >= cr:KEYS:LENGTH {
    LOCAL rname IS cr:KEYS[k]:TOUPPER.
    IF rname:FIND("AIR") >= 0 OR rname:FIND("INTAKE") >= 0 { RETURN TRUE. }
    SET k TO k + 1.
  }
  // Fallback heuristic: air-breathers have dramatically higher sea-level ISP.
  // A typical jet has ISP > 1000 s at sea level; rockets rarely exceed 350 s.
  IF eng:ISPAT(0) > 600 { RETURN TRUE. }
  RETURN FALSE.
}

// â”€â”€ Body-frame â†’ orbital-frame transform â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//
// FAR's AEROFORCE is in the ship body frame where:
//   +X = right   (STARVECTOR)
//   +Y = up      (TOPVECTOR)
//   +Z = backward (-FOREVECTOR)
//
// The cached globals ASC_STAR_V, ASC_TOP_V, ASC_NFWD_V hold these
// orbital-frame vectors and must be refreshed once per cycle before
// this function is called.
//
FUNCTION _ASC_BODY_TO_ORB {
  PARAMETER v_body.
  RETURN ASC_STAR_V * v_body:X
       + ASC_TOP_V  * v_body:Y
       + ASC_NFWD_V * v_body:Z.
}

FUNCTION _ASC_LIST_HAS_ENGINE {
  PARAMETER eng_list, eng.
  LOCAL i IS 0.
  UNTIL i >= eng_list:LENGTH {
    IF eng_list[i] = eng { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _ASC_IS_DUALMODE_AB_PRIMARY {
  PARAMETER eng.
  IF NOT eng:HASSUFFIX("MULTIMODE") OR NOT eng:MULTIMODE { RETURN FALSE. }
  IF NOT eng:HASSUFFIX("PRIMARYMODE") { RETURN FALSE. }
  IF NOT eng:PRIMARYMODE { RETURN FALSE. }
  RETURN _ASC_LIST_HAS_ENGINE(ASC_AB_ENGINES, eng).
}

// â”€â”€ Penalty functions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//
// All penalties return a scalar in [0,1].  Multiplied into J_ab or J_rk
// to reflect conditions that reduce the usefulness of each mode.

// P_q: dynamic pressure outside the optimal corridor degrades J_ab.
FUNCTION _ASC_P_Q {
  LOCAL q_dyn IS ASC_Q_CTRL.
  LOCAL q_tgt IS ASC_Q_TARGET_CACHED.
  LOCAL q_max IS ASC_Q_MAX_CACHED.
  LOCAL q_min IS ASC_Q_MIN_CACHED.

  IF q_dyn > q_max {
    // Above structural limit â€” hard penalty, degrades fast.
    LOCAL excess IS (q_dyn - q_max) / q_max.
    RETURN MAX(1 - excess * 2, 0.1).
  }
  IF q_dyn < q_min {
    // Below lower bound â€” corridor left, AB losing value.
    IF q_min > 0 { RETURN MAX(q_dyn / q_min, 0.2). }
  }
  RETURN 1.0.
}

// P_heat: aerodynamic heating proxy  q * v  approaching limit degrades J_ab.
FUNCTION _ASC_P_HEAT {
  LOCAL v_mag IS SHIP:VELOCITY:ORBIT:MAG.
  LOCAL q_dyn IS ASC_Q_CTRL.
  LOCAL proxy IS q_dyn * v_mag.
  LOCAL lim IS ASC_HEAT_LIMIT_CACHED. // PaÂ·m/s
  IF proxy <= lim * 0.75 { RETURN 1.0. }
  IF proxy >= lim { RETURN 0.4. }
  RETURN 1 - 0.6 * (proxy - lim * 0.75) / (lim * 0.25).
}

// P_ctrl: validity-gated controllability penalty.
// When estimator is degraded the mode comparison is less trustworthy.
FUNCTION _ASC_P_CTRL {
  IF ASC_ESTIMATOR_VALIDITY = ASC_INVALID  { RETURN 0.3. }
  IF ASC_ESTIMATOR_VALIDITY = ASC_DEGRADED { RETURN 0.7. }
  RETURN 1.0.
}

// P_traj: penalise J_ab when the trajectory is not growing apoapsis.
// Prevents the controller from valuing AB mode while flying flat and fast
// at low altitude without building the orbit.
FUNCTION _ASC_P_TRAJ {
  LOCAL elapsed IS MAX(TIME:SECONDS - ASC_APO_PREV_UT, 0.5).
  LOCAL growth IS (ASC_APO_VAL - ASC_APO_PREV) / elapsed. // m/s
  IF growth < -200 { RETURN 0.4. }
  IF growth <    0 { RETURN 0.75. }
  RETURN 1.0.
}

// P_traj_rk: penalise J_rk when apoapsis is far below the zoom target.
// Prevents premature rocket commit while there is still useful AB energy to
// harvest before raising the orbit.  Linear ramp: 0.5 at apo=0, 1.0 at apoâ‰¥target.
FUNCTION _ASC_P_TRAJ_RK {
  LOCAL zoom_apo IS ASC_ZOOM_APO_TARGET_CACHED.
  IF zoom_apo <= 0 { RETURN 1.0. }
  LOCAL frac IS CLAMP(ASC_APO_VAL / zoom_apo, 0, 1).
  RETURN 0.5 + 0.5 * frac.
}

// P_atm: rocket mode value is reduced at high atmospheric pressure because
// rocket ISP degrades and drag losses are larger.  This narrows as altitude
// rises â€” exactly the asymmetry that makes AB mode worth extending.
FUNCTION _ASC_P_ATM {
  // Uses ASC_PRESSURE_CACHED — must be called after ASC_STATE_UPDATE sets it.
  // AVAILABLETHRUSTAT uses atm units (0 = vacuum, 1 = sea level).
  // At sea level pressure, rocket performance is worst.
  // Above ~25 km the penalty vanishes.
  IF ASC_PRESSURE_CACHED <= 0.01 { RETURN 1.0. }
  IF ASC_PRESSURE_CACHED >= 0.80 { RETURN 0.6. }
  RETURN 1 - 0.4 * (ASC_PRESSURE_CACHED / 0.80).
}

// â”€â”€ Estimator validity update â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

FUNCTION _ASC_UPDATE_VALIDITY {
  LOCAL failing IS 0.

  // --- AoA oscillation check ---
  // Track max/min AoA in a rolling window; large amplitude â†’ degraded.
  LOCAL aoa_now IS GET_AOA().
  LOCAL now     IS TIME:SECONDS.

  IF aoa_now > ASC_AOA_MAX_WIN { SET ASC_AOA_MAX_WIN TO aoa_now. }
  IF aoa_now < ASC_AOA_MIN_WIN { SET ASC_AOA_MIN_WIN TO aoa_now. }

  IF (now - ASC_AOA_WIN_UT) >= ASC_AOA_OSC_WINDOW_S {
    // Roll the window forward.
    SET ASC_AOA_MAX_WIN TO aoa_now.
    SET ASC_AOA_MIN_WIN TO aoa_now.
    SET ASC_AOA_WIN_UT  TO now.
  }

  LOCAL aoa_amp IS ASC_AOA_MAX_WIN - ASC_AOA_MIN_WIN.
  IF aoa_amp > ASC_AOA_OSC_THRESH { SET failing TO failing + 1. }

  // --- Force-Ä– vs orbital-Ä– agreement check ---
  // Both signals must be EMA-smoothed before comparison; raw frame-to-frame
  // differences divided by dt are too noisy to use here directly.
  LOCAL v_mag IS SHIP:VELOCITY:ORBIT:MAG.
  IF v_mag > ASC_MIN_VORB AND ABS(ASC_EDOT_VAL) > 50 {
    LOCAL diff IS ABS(ASC_EDOT_VAL - ASC_EDOT_ORB_FILT).
    LOCAL ref  IS ABS(ASC_EDOT_VAL).
    IF diff / ref > ASC_EDOT_TOL { SET failing TO failing + 1. }
  }

  IF      failing = 0 { SET ASC_ESTIMATOR_VALIDITY TO ASC_VALID.   }
  ELSE IF failing = 1 { SET ASC_ESTIMATOR_VALIDITY TO ASC_DEGRADED. }
  ELSE                { SET ASC_ESTIMATOR_VALIDITY TO ASC_INVALID.  }
}

// â”€â”€ J_ab computation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

FUNCTION _ASC_COMPUTE_J_AB {
  PARAMETER t_ab_along.  // N  AB thrust along orbital velocity
  PARAMETER v_mag.       // m/s  orbital speed magnitude

  // Guard: must have mass flow and meaningful speed.
  IF ASC_MDOT_AB_VAL < ASC_MIN_MDOT OR v_mag < ASC_MIN_VORB {
    SET ASC_J_AB TO 0.
    RETURN.
  }

  // Energy gain rate: v Â· (T_ab + F_aero) / m   using valuation-smoothed aero.
  LOCAL edot_ab IS v_mag * (t_ab_along + ASC_AERO_ALONG_VAL) / SHIP:MASS.

  // Penalty multipliers.
  LOCAL p IS _ASC_P_Q() * _ASC_P_HEAT() * _ASC_P_CTRL() * _ASC_P_TRAJ().

  SET ASC_J_AB TO (edot_ab / ASC_MDOT_AB_VAL) * p.
}

// â”€â”€ J_rk computation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

FUNCTION _ASC_COMPUTE_J_RK {
  PARAMETER v_mag.   // m/s  orbital speed magnitude
  PARAMETER v_hat.   // unit vector along orbital velocity (inertial frame)
  PARAMETER in_zoom. // TRUE when currently in AB_ZOOM phase

  // Guard.
  IF ASC_MDOT_RK_VAL < ASC_MIN_MDOT OR v_mag < ASC_MIN_VORB {
    SET ASC_J_RK TO 0.
    RETURN.
  }

  // Available rocket thrust projected along velocity.
  // Per-engine projection: RAPIER-type engines may have facing that differs
  // from the ship axis (e.g., canted nozzles), so we must use eng:FACING
  // for each engine individually â€” matching how AB thrust is computed.
  LOCAL dual_thr_frac IS ASC_DUALMODE_RK_THR_CACHED.
  LOCAL t_rk_along IS 0.
  LOCAL i IS 0.
  UNTIL i >= ASC_RK_ENGINES:LENGTH {
    LOCAL eng IS ASC_RK_ENGINES[i].
    LOCAL thr_eff IS eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
    IF ASC_RK_DUALMODE[i] AND eng:PRIMARYMODE {
      SET thr_eff TO thr_eff * dual_thr_frac.
    }
    SET t_rk_along TO t_rk_along
      + thr_eff * VDOT(eng:FACING:FOREVECTOR, v_hat).
    SET i TO i + 1.
  }

  // Drag estimate: use dedicated heavy-filtered drag, projected forward during zoom.
  LOCAL d_est IS ASC_DRAG_RK_VAL.
  IF in_zoom AND ASC_DRAG_RK_VAL > 0 {
    // Project drag forward; floor at a fraction of current drag.
    LOCAL d_proj IS ASC_DRAG_RK_VAL + ASC_DRAG_DDOT * ASC_DRAG_LOOKAHEAD_S.
    LOCAL d_floor IS ASC_DRAG_RK_VAL * ASC_DRAG_FLOOR_FRAC.
    SET d_est TO MAX(MAX(d_proj, d_floor), 0).
  }

  // Propellant equivalency weight.
  LOCAL lf_frac  IS 0.
  LOCAL ox_frac  IS 0.
  IF ASC_LF_MAX > 0 { SET lf_frac TO SHIP:LIQUIDFUEL / ASC_LF_MAX. }
  IF ASC_OX_MAX > 0 { SET ox_frac TO SHIP:OXIDIZER   / ASC_OX_MAX. }
  LOCAL k_prop IS ASC_K_PROP_CACHED.
  LOCAL w_prop IS 1 + k_prop * MAX(0, ox_frac - lf_frac).
  SET TELEM_ASC_W_PROP TO w_prop.

  // Net rocket energy gain rate per weighted propellant mass.
  LOCAL edot_rk IS v_mag * (t_rk_along - d_est) / SHIP:MASS.
  LOCAL j_rk_raw IS edot_rk / (ASC_MDOT_RK_VAL * w_prop).
  SET ASC_J_RK TO j_rk_raw * _ASC_P_ATM() * _ASC_P_TRAJ_RK().
}

// â”€â”€ Fuel floor functions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//
// Estimate the rocket Î”V available from remaining LF+OX propellant.
// Uses Tsiolkovsky with current rocket ISP at current pressure.
// Returns 0 if no rocket engines are classified or mass is degenerate.
//
FUNCTION _ASC_ROCKET_DV_AVAIL {
  LOCAL g0       IS 9.80665.
  LOCAL dual_isp_cap IS ASC_DUALMODE_RK_ISP_CACHED.
  LOCAL dual_thr_frac IS ASC_DUALMODE_RK_THR_CACHED.

  // Average ISP across available rocket engines (weighted by thrust).
  LOCAL t_sum    IS 0.
  LOCAL t_isp_sum IS 0.
  LOCAL i IS 0.
  UNTIL i >= ASC_RK_ENGINES:LENGTH {
    LOCAL eng IS ASC_RK_ENGINES[i].
    LOCAL isp IS eng:ISPAT(ASC_PRESSURE_CACHED).
    LOCAL thr IS eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
    IF ASC_RK_DUALMODE[i] AND eng:PRIMARYMODE {
      SET isp TO MIN(isp, dual_isp_cap).
      SET thr TO thr * dual_thr_frac.
    }
    IF isp > 1 AND thr > 0 {
      SET t_sum    TO t_sum    + thr.
      SET t_isp_sum TO t_isp_sum + thr * isp.
    }
    SET i TO i + 1.
  }
  IF t_sum < 0.1 { RETURN 0. }
  LOCAL isp_eff IS t_isp_sum / t_sum.

  // Propellant mass remaining (LF + OX, converted from units to tonnes).
  // KSP stock: 1 unit LF = 5 kg = 0.005 t; 1 unit OX = 5 kg = 0.005 t.
  LOCAL prop_t IS (SHIP:LIQUIDFUEL + SHIP:OXIDIZER) * ASC_PROP_DENSITY / 1000.
  LOCAL m0 IS SHIP:MASS.
  LOCAL m1 IS m0 - prop_t.
  IF m1 <= 0 OR m1 >= m0 { RETURN 0. }
  RETURN isp_eff * g0 * LN(m0 / m1).
}

// Returns TRUE if rocket propellant is too low to complete circularisation.
// Triggers an immediate rocket commit in CORRIDOR and ZOOM regardless of J
// comparison, to prevent arriving at circularisation with insufficient Î”V.
//
FUNCTION _ASC_CHECK_FUEL_FLOOR {
  // Don't trigger before apoapsis has reached a meaningful fraction of the
  // zoom target — committing rockets at very low apoapsis wastes the
  // remaining Oberth opportunity in the AB phase.
  IF ASC_APO_VAL < ASC_ZOOM_APO_TARGET_CACHED * ASC_FUEL_FLOOR_MIN_APO_FRAC {
    RETURN FALSE.
  }

  LOCAL final_apo IS ASC_FINAL_APO_TARGET_CACHED.
  LOCAL v_now     IS SHIP:VELOCITY:ORBIT:MAG.

  // Circular orbit speed at the target apoapsis altitude.
  LOCAL r_tgt  IS KERBIN_R + final_apo.
  LOCAL v_circ IS SQRT(KERBIN_MU / r_tgt).

  // Needed Î”V: difference between circular speed and current speed, plus margin.
  // This is an approximation (ignores flight path angle) but is conservative.
  LOCAL needed IS MAX(v_circ - v_now, 0) + ASC_FUEL_FLOOR_DV_MARGIN.
  LOCAL avail  IS _ASC_ROCKET_DV_AVAIL().

  RETURN avail < needed.
}

// â”€â”€ Public: initialise â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

FUNCTION ASC_STATE_INIT {
  // --- Classify and cache engine lists (done once, not in the loop) ---
  ASC_AB_ENGINES:CLEAR().
  ASC_RK_ENGINES:CLEAR().

  LOCAL all_eng IS SHIP:ENGINES.
  LOCAL i IS 0.
  UNTIL i >= all_eng:LENGTH {
    LOCAL eng IS all_eng[i].
    IF eng:IGNITION OR eng:ALLOWRESTART {
      IF _ASC_IS_AB_ENGINE(eng) {
        ASC_AB_ENGINES:ADD(eng).
      } ELSE {
        ASC_RK_ENGINES:ADD(eng).
      }
    }
    SET i TO i + 1.
  }

  // For dual-mode engines (RAPIER-type): if an AB engine also consumes LF+OX
  // in its other mode, add it to RK_ENGINES too so J_rk estimation works.
  LOCAL j IS 0.
  UNTIL j >= ASC_AB_ENGINES:LENGTH {
    LOCAL eng IS ASC_AB_ENGINES[j].
    LOCAL already_rk IS FALSE.
    LOCAL k IS 0.
    UNTIL k >= ASC_RK_ENGINES:LENGTH {
      IF ASC_RK_ENGINES[k] = eng { SET already_rk TO TRUE. }
      SET k TO k + 1.
    }
    IF NOT already_rk {
      // Check if this engine has LF or LFO in its resource list at all â€”
      // if it does it is dual-mode and belongs in both lists.
      LOCAL cr IS eng:CONSUMEDRESOURCES.
      LOCAL m IS 0.
      UNTIL m >= cr:KEYS:LENGTH {
        LOCAL rname IS cr:KEYS[m]:TOUPPER.
        IF rname:FIND("LIQUID") >= 0 OR rname:FIND("OXIDIZER") >= 0 {
          ASC_RK_ENGINES:ADD(eng).
          SET m TO cr:KEYS:LENGTH. // break
        }
        SET m TO m + 1.
      }
    }
    SET j TO j + 1.
  }

  // --- Pre-compute per-engine dual-mode flag (avoids suffix queries in hot path) ---
  // ASC_RK_DUALMODE[i] = TRUE iff ASC_RK_ENGINES[i] is also in ASC_AB_ENGINES
  // (i.e., a multimode engine currently in AB mode).
  ASC_RK_DUALMODE:CLEAR().
  LOCAL dm IS 0.
  UNTIL dm >= ASC_RK_ENGINES:LENGTH {
    LOCAL dm_eng IS ASC_RK_ENGINES[dm].
    LOCAL is_dual IS dm_eng:HASSUFFIX("MULTIMODE") AND dm_eng:MULTIMODE
                  AND dm_eng:HASSUFFIX("PRIMARYMODE")
                  AND _ASC_LIST_HAS_ENGINE(ASC_AB_ENGINES, dm_eng).
    ASC_RK_DUALMODE:ADD(is_dual).
    SET dm TO dm + 1.
  }

  // --- Store propellant capacities for equivalency weight ---
  SET ASC_LF_MAX TO 1.0.
  SET ASC_OX_MAX TO 1.0.
  FOR res IN SHIP:RESOURCES {
    IF res:NAME = "LiquidFuel" { SET ASC_LF_MAX TO MAX(res:CAPACITY, 1). }
    IF res:NAME = "Oxidizer"   { SET ASC_OX_MAX TO MAX(res:CAPACITY, 1). }
  }

  // --- Seed EMA states from current measurements ---
  // Pre-seeding prevents the large startup transient that arises when
  // filters are initialised to zero and converge from far below reality.
  LOCAL v_mag IS SHIP:VELOCITY:ORBIT:MAG.
  LOCAL q_now IS 0.
  LOCAL aoa_now IS 0.
  IF FAR_AVAILABLE {
    SET q_now   TO ADDONS:FAR:DYNPRES * ASC_FAR_Q_SCALE.
    SET aoa_now TO ADDONS:FAR:AOA.
  }
  LOCAL apo_now IS MAX(SHIP:ORBIT:APOAPSIS, 0).

  SET ASC_AERO_ALONG_CTRL TO 0.
  SET ASC_Q_CTRL          TO q_now.
  SET ASC_AOA_CTRL        TO aoa_now.
  SET ASC_AERO_ALONG_VAL  TO 0.
  SET ASC_DRAG_RK_VAL     TO 0.
  SET ASC_DRAG_DDOT       TO 0.
  SET ASC_DRAG_PREV       TO 0.
  SET ASC_EDOT_VAL        TO 0.
  SET ASC_APO_VAL         TO apo_now.
  SET ASC_MDOT_AB_VAL     TO 0.
  SET ASC_MDOT_RK_VAL     TO 0.
  SET ASC_APO_PREV        TO apo_now.
  SET ASC_APO_PREV_UT     TO TIME:SECONDS.

  // Seed orbital energy from first measurement.
  LOCAL r_orb IS KERBIN_R + SHIP:ALTITUDE.
  SET ASC_ENERGY_PREV TO v_mag^2 / 2 - KERBIN_MU / r_orb.
  SET ASC_EDOT_ORBITAL  TO 0.
  SET ASC_EDOT_ORB_FILT TO 0.

  SET ASC_ESTIMATOR_VALIDITY TO ASC_VALID.
  SET ASC_AOA_MAX_WIN TO aoa_now.
  SET ASC_AOA_MIN_WIN TO aoa_now.
  SET ASC_AOA_WIN_UT  TO TIME:SECONDS.

  SET ASC_J_AB TO 0.
  SET ASC_J_RK TO 0.

  // Cache per-aircraft ascent parameters once for hot-path reuse.
  SET ASC_Q_TARGET_CACHED TO AC_PARAM("ascent_q_target", ASC_Q_TARGET_DEFAULT, 1).
  SET ASC_Q_MAX_CACHED TO AC_PARAM("ascent_q_max", ASC_Q_MAX_DEFAULT, 1).
  SET ASC_Q_MIN_CACHED TO AC_PARAM("ascent_q_min", ASC_Q_MIN_DEFAULT, 0).
  SET ASC_HEAT_LIMIT_CACHED TO AC_PARAM("ascent_heat_limit", 3.5e8, 1).
  SET ASC_ZOOM_APO_TARGET_CACHED TO AC_PARAM("ascent_zoom_target_m", ASC_APO_ZOOM_TARGET_DEFAULT, 1).
  SET ASC_FINAL_APO_TARGET_CACHED TO AC_PARAM("ascent_apoapsis_target_m", ASC_APO_FINAL_DEFAULT, 1).
  SET ASC_AOA_LIMIT_CACHED TO AC_PARAM("ascent_aoa_limit", AA_MAX_AOA, 0).
  SET ASC_CORRIDOR_FPA_MAX_CACHED TO AC_PARAM("ascent_corridor_fpa_max", 25.0, 0).
  SET ASC_DUALMODE_RK_ISP_CACHED TO AC_PARAM("ascent_dualmode_rk_isp", ASC_DUALMODE_RK_ISP_DEFAULT, 1).
  SET ASC_DUALMODE_RK_THR_CACHED TO AC_PARAM("ascent_dualmode_rk_thrust_frac", ASC_DUALMODE_RK_THR_FRAC_DEFAULT, 0.01).
  SET ASC_K_PROP_CACHED TO AC_PARAM("ascent_k_prop", ASC_K_PROP_DEFAULT, 0).

  // Cache regime Mach at init â€” avoids AC_PARAM call inside WHEN...THEN trigger.
  SET ASC_REGIME_MACH_CACHED TO AC_PARAM("ascent_regime_mach", ASC_REGIME_MACH_DEFAULT, 0).

  IFC_SET_ALERT("ASC init: " + ASC_AB_ENGINES:LENGTH + " AB  "
    + ASC_RK_ENGINES:LENGTH + " RK  LFmax " + ROUND(ASC_LF_MAX,0)
    + " OXmax " + ROUND(ASC_OX_MAX,0)).

  SET ASC_INITIALIZED TO TRUE.
}

// â”€â”€ Public: per-cycle update â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

FUNCTION ASC_STATE_UPDATE {
  PARAMETER dt. // s  measured loop dt  (= IFC_ACTUAL_DT)

  IF dt < 0.001 { RETURN. } // guard against zero or negative dt

  // Cache atmospheric pressure once per cycle — used by all downstream functions.
  SET ASC_PRESSURE_CACHED TO SHIP:BODY:ATM:ALTITUDEPRESSURE(SHIP:ALTITUDE).

  // â”€â”€ Step 1: cache facing vectors (single query per cycle) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  SET ASC_STAR_V TO SHIP:FACING:STARVECTOR.
  SET ASC_TOP_V  TO SHIP:FACING:TOPVECTOR.
  SET ASC_NFWD_V TO -SHIP:FACING:FOREVECTOR.

  // â”€â”€ Step 2: orbital velocity unit vector â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL v_orb IS SHIP:VELOCITY:ORBIT.
  LOCAL v_mag IS v_orb:MAG.
  LOCAL v_hat IS V(0,1,0). // fallback when speed too low for reliable unit vector
  IF v_mag > ASC_MIN_VORB { SET v_hat TO v_orb:NORMALIZED. }

  // â”€â”€ Step 3: FAR aeroforce â†’ orbital frame â†’ along-track component â”€â”€
  LOCAL aero_along_raw IS 0.
  IF FAR_AVAILABLE AND v_mag > ASC_MIN_VORB {
    LOCAL aero_body IS ADDONS:FAR:AEROFORCE.
    LOCAL aero_orb  IS _ASC_BODY_TO_ORB(aero_body).
    SET aero_along_raw TO VDOT(aero_orb, v_hat).
  }

  // Along-track drag magnitude (drag opposes motion â†’ negative along-track aero).
  LOCAL drag_raw IS MAX(-aero_along_raw, 0).

  // â”€â”€ Step 4: AB thrust and mass flow (iterate cached list only) â”€â”€â”€â”€â”€
  LOCAL t_ab_along  IS 0.
  LOCAL mdot_ab_raw IS 0.

  LOCAL i IS 0.
  UNTIL i >= ASC_AB_ENGINES:LENGTH {
    LOCAL eng IS ASC_AB_ENGINES[i].
    IF eng:IGNITION AND NOT eng:FLAMEOUT {
      SET t_ab_along  TO t_ab_along  + eng:THRUST * VDOT(eng:FACING:FOREVECTOR, v_hat).
      SET mdot_ab_raw TO mdot_ab_raw + eng:MASSFLOW * 1000. // tonnes/s â†’ kg/s
    }
    SET i TO i + 1.
  }

  // Update AB thrust ratio for the WHEN...THEN flameout trigger.
  // The trigger reads ASC_AB_THR_RATIO every physics tick but cannot call AC_PARAM,
  // so the ratio is pre-computed here in the main loop where AC_PARAM is safe.
  LOCAL t_ab_now_total   IS 0.
  LOCAL t_ab_avail_total IS 0.
  LOCAL ab_ign_on IS 0.
  LOCAL ab_flameouts IS 0.
  LOCAL ii IS 0.
  UNTIL ii >= ASC_AB_ENGINES:LENGTH {
    LOCAL eng IS ASC_AB_ENGINES[ii].
    IF eng:IGNITION {
      SET ab_ign_on TO ab_ign_on + 1.
      SET t_ab_now_total   TO t_ab_now_total   + eng:THRUST.
      SET t_ab_avail_total TO t_ab_avail_total + eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
      IF eng:FLAMEOUT { SET ab_flameouts TO ab_flameouts + 1. }
    }
    SET ii TO ii + 1.
  }
  IF t_ab_avail_total > 0.1 {
    SET ASC_AB_THR_RATIO TO t_ab_now_total / t_ab_avail_total.
  } ELSE {
    SET ASC_AB_THR_RATIO TO 0.
  }

  // â”€â”€ Step 5: rocket mass flow estimate (for J_rk denominator) â”€â”€â”€â”€â”€â”€â”€
  LOCAL mdot_rk_raw IS 0.
  LOCAL g0 IS 9.80665.
  LOCAL dual_isp_cap IS ASC_DUALMODE_RK_ISP_CACHED.
  LOCAL dual_thr_frac IS ASC_DUALMODE_RK_THR_CACHED.
  LOCAL t_rk_now_total   IS 0.
  LOCAL t_rk_avail_total IS 0.
  LOCAL rk_ign_on IS 0.
  LOCAL rk_flameouts IS 0.
  LOCAL k IS 0.
  UNTIL k >= ASC_RK_ENGINES:LENGTH {
    LOCAL eng IS ASC_RK_ENGINES[k].
    LOCAL thr_avail IS eng:AVAILABLETHRUSTAT(ASC_PRESSURE_CACHED).
    LOCAL isp IS eng:ISPAT(ASC_PRESSURE_CACHED).
    IF ASC_RK_DUALMODE[k] AND eng:PRIMARYMODE {
      SET isp TO MIN(isp, dual_isp_cap).
      SET thr_avail TO thr_avail * dual_thr_frac.
    }
    SET t_rk_avail_total TO t_rk_avail_total + thr_avail.
    IF eng:IGNITION {
      SET rk_ign_on TO rk_ign_on + 1.
      SET t_rk_now_total TO t_rk_now_total + eng:THRUST.
      IF eng:FLAMEOUT { SET rk_flameouts TO rk_flameouts + 1. }
    }
    IF isp > 1 {
      // Keep units aligned with AB path: eng:MASSFLOW is converted to kg/s.
      // thrust/(Isp*g0) yields tonnes/s in KSP units, so convert to kg/s.
      SET mdot_rk_raw TO mdot_rk_raw + thr_avail / (isp * g0) * 1000.
    }
    SET k TO k + 1.
  }

  // â”€â”€ Step 6: dynamic pressure and AoA â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL q_raw   IS 0.
  LOCAL aoa_raw IS 0.
  LOCAL mach_now IS 0.
  IF FAR_AVAILABLE {
    SET q_raw    TO ADDONS:FAR:DYNPRES * ASC_FAR_Q_SCALE.
    SET aoa_raw  TO ADDONS:FAR:AOA.
    SET mach_now TO ADDONS:FAR:MACH.
  }

  // â”€â”€ Step 7: EMA updates â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  // Control copies  (fast â€” for pitch commands and throttle)
  SET ASC_AERO_ALONG_CTRL TO ASC_EMA_CTRL_AERO * aero_along_raw + (1 - ASC_EMA_CTRL_AERO) * ASC_AERO_ALONG_CTRL.
  SET ASC_Q_CTRL          TO ASC_EMA_CTRL_Q    * q_raw           + (1 - ASC_EMA_CTRL_Q)    * ASC_Q_CTRL.
  SET ASC_AOA_CTRL        TO ASC_EMA_CTRL_AOA  * aoa_raw         + (1 - ASC_EMA_CTRL_AOA)  * ASC_AOA_CTRL.

  // Valuation copies  (slow â€” for mode comparison only)
  SET ASC_AERO_ALONG_VAL  TO ASC_EMA_VAL_AERO  * aero_along_raw + (1 - ASC_EMA_VAL_AERO)  * ASC_AERO_ALONG_VAL.
  SET ASC_MDOT_AB_VAL     TO ASC_EMA_VAL_MDOT  * mdot_ab_raw    + (1 - ASC_EMA_VAL_MDOT)  * ASC_MDOT_AB_VAL.
  SET ASC_MDOT_RK_VAL     TO ASC_EMA_VAL_MDOT  * mdot_rk_raw    + (1 - ASC_EMA_VAL_MDOT)  * ASC_MDOT_RK_VAL.

  // Dedicated heavy filter for drag in J_rk (Î± = 0.10 per paper Â§4.3 and Â§5.3.3)
  LOCAL drag_ddot_raw IS (drag_raw - ASC_DRAG_PREV) / dt.
  SET ASC_DRAG_PREV   TO drag_raw.
  SET ASC_DRAG_DDOT   TO ASC_EMA_DRAG_RK * drag_ddot_raw + (1 - ASC_EMA_DRAG_RK) * ASC_DRAG_DDOT.
  SET ASC_DRAG_RK_VAL TO ASC_EMA_DRAG_RK * drag_raw      + (1 - ASC_EMA_DRAG_RK) * ASC_DRAG_RK_VAL.

  // Smoothed apoapsis
  LOCAL apo_raw IS MAX(SHIP:ORBIT:APOAPSIS, 0).
  SET ASC_APO_VAL TO ASC_EMA_VAL_APO * apo_raw + (1 - ASC_EMA_VAL_APO) * ASC_APO_VAL.

  // Rolling window for apoapsis growth rate: roll every ASC_ZOOM_APO_WINDOW_S seconds.
  // Without this roll, the rate estimate degrades to a time-diluted average from
  // phase start rather than reflecting recent growth.
  IF (TIME:SECONDS - ASC_APO_PREV_UT) >= ASC_ZOOM_APO_WINDOW_S {
    SET ASC_APO_PREV    TO ASC_APO_VAL.
    SET ASC_APO_PREV_UT TO TIME:SECONDS.
  }

  // â”€â”€ Step 8: specific orbital energy â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL r_orb IS KERBIN_R + SHIP:ALTITUDE.
  LOCAL e_now IS v_mag * v_mag / 2 - KERBIN_MU / r_orb.

  // Orbital-state Ä– (validation cross-check â€” not for mode comparison).
  SET ASC_EDOT_ORBITAL TO (e_now - ASC_ENERGY_PREV) / dt.
  SET ASC_ENERGY_PREV  TO e_now.
  SET ASC_EDOT_ORB_FILT TO ASC_EMA_EDOT_ORB * ASC_EDOT_ORBITAL
                         + (1 - ASC_EMA_EDOT_ORB) * ASC_EDOT_ORB_FILT.

  // Force-projection Ä–  (valuation signal for J_ab, J_rk).
  LOCAL edot_force IS 0.
  IF v_mag > ASC_MIN_VORB {
    SET edot_force TO v_mag * (t_ab_along + aero_along_raw) / SHIP:MASS.
  }
  SET ASC_EDOT_VAL TO ASC_EMA_VAL_EDOT * edot_force + (1 - ASC_EMA_VAL_EDOT) * ASC_EDOT_VAL.

  // â”€â”€ Step 9: estimator validity â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  _ASC_UPDATE_VALIDITY().

  // â”€â”€ Step 10: mode valuations â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  LOCAL in_zoom IS IFC_SUBPHASE = SUBPHASE_ASC_AB_ZOOM.
  _ASC_COMPUTE_J_AB(t_ab_along, v_mag).
  _ASC_COMPUTE_J_RK(v_mag, v_hat, in_zoom).

  // â”€â”€ Step 11: telemetry export â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
  SET TELEM_ASC_J_AB      TO ASC_J_AB.
  SET TELEM_ASC_J_RK      TO ASC_J_RK.
  SET TELEM_ASC_VALIDITY  TO ASC_ESTIMATOR_VALIDITY.
  SET TELEM_ASC_Q         TO ASC_Q_CTRL.
  SET TELEM_ASC_Q_RAW     TO q_raw.
  SET TELEM_ASC_MACH      TO mach_now.
  SET TELEM_ASC_APO       TO ASC_APO_VAL.
  SET TELEM_ASC_DRAG_RK   TO ASC_DRAG_RK_VAL.
  SET TELEM_ASC_EDOT_VAL  TO ASC_EDOT_VAL.
  SET TELEM_ASC_EDOT_ORB  TO ASC_EDOT_ORB_FILT.
  SET TELEM_ASC_AB_THR_RATIO TO ASC_AB_THR_RATIO.
  SET TELEM_ASC_AB_T_NOW     TO t_ab_now_total.
  SET TELEM_ASC_AB_T_AVAIL   TO t_ab_avail_total.
  SET TELEM_ASC_AB_IGN_ON    TO ab_ign_on.
  SET TELEM_ASC_AB_FLAMEOUTS TO ab_flameouts.
  SET TELEM_ASC_RK_T_NOW     TO t_rk_now_total.
  SET TELEM_ASC_RK_T_AVAIL   TO t_rk_avail_total.
  SET TELEM_ASC_RK_IGN_ON    TO rk_ign_on.
  SET TELEM_ASC_RK_FLAMEOUTS TO rk_flameouts.
}

