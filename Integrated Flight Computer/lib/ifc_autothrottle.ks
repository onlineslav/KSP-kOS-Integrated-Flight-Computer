@LAZYGLOBAL OFF.

// ============================================================
// ifc_autothrottle.ks  -  Integrated Flight Computer
//
// Shared adaptive speed/throttle controller used by cruise,
// approach, and flare.
//
// Structure:
// - Outer loop: speed error -> commanded acceleration
// - Inner loop: acceleration error -> throttle
//
// Adaptation:
// - Estimates throttle->acceleration gain online (AT_GAIN_EST)
// - Estimates effective throttle lag (AT_TAU_EST)
// - Estimates current positive acceleration authority from
//   available thrust and mass (AT_A_THRUST_MAX_EST)
// - Estimates idle decel envelope (AT_IDLE_DECEL_EST)
//
// Scheduler outputs:
// - KP_ACL_THR_eff
// - KI_SPD_eff
// - THR_SLEW_eff
// - a_cmd clamps for accel/decel authority
// ============================================================

FUNCTION AT_RESET {
  SET AT_EST_INIT         TO FALSE.
  SET AT_PREV_THR_CUR     TO 0.
  SET AT_PREV_A_FILT      TO 0.
  SET AT_GAIN_EST         TO AT_GAIN_INIT.
  SET AT_TAU_EST          TO AT_TAU_INIT.
  SET AT_A_THRUST_MAX_EST TO 0.
  SET AT_IDLE_DECEL_EST   TO AT_IDLE_DECEL_INIT.
}

FUNCTION _AT_UPDATE_ESTIMATES {
  PARAMETER ias.

  LOCAL dt IS MAX(IFC_ACTUAL_DT, 0.01).
  LOCAL thr_cur IS GET_CURRENT_THROTTLE().

  // Measured longitudinal acceleration proxy from IAS derivative.
  LOCAL a_raw IS CLAMP((ias - PREV_IAS) / dt, -10, 10).
  SET PREV_IAS      TO ias.
  SET A_ACTUAL_FILT TO A_ACTUAL_FILT * ACL_FILTER_ALPHA + a_raw * (1 - ACL_FILTER_ALPHA).

  IF NOT AT_EST_INIT {
    SET AT_PREV_THR_CUR TO thr_cur.
    SET AT_PREV_A_FILT  TO A_ACTUAL_FILT.
    SET AT_EST_INIT     TO TRUE.
  }

  LOCAL dthr IS thr_cur - AT_PREV_THR_CUR.
  LOCAL da   IS A_ACTUAL_FILT - AT_PREV_A_FILT.

  // Throttle effectiveness estimate: G = d(a)/d(throttle).
  IF ABS(dthr) >= AT_GAIN_DTHR_MIN {
    LOCAL g_sample IS ABS(da / dthr).
    IF g_sample >= AT_GAIN_MIN AND g_sample <= AT_GAIN_MAX {
      SET AT_GAIN_EST TO AT_GAIN_EST * AT_GAIN_ALPHA + g_sample * (1 - AT_GAIN_ALPHA).
    }
  }

  // Effective lag estimate from command-tracking dynamics.
  IF ABS(dthr) >= AT_TAU_DTHR_MIN {
    LOCAL dthr_dt IS ABS(dthr) / dt.
    LOCAL thr_err IS ABS(THROTTLE_CMD - thr_cur).
    IF dthr_dt > 0.001 AND thr_err >= AT_TAU_ERR_MIN {
      LOCAL tau_sample IS CLAMP(thr_err / dthr_dt, AT_TAU_MIN, AT_TAU_MAX).
      SET AT_TAU_EST TO AT_TAU_EST * AT_TAU_ALPHA + tau_sample * (1 - AT_TAU_ALPHA).
    }
  }

  // Positive accel authority estimate from currently available thrust.
  LOCAL a_thrust_now IS 0.
  IF SHIP:MASS > 0 {
    SET a_thrust_now TO SHIP:AVAILABLETHRUST / SHIP:MASS.
  }
  SET AT_A_THRUST_MAX_EST TO AT_A_THRUST_MAX_EST * AT_AUTH_ALPHA
                          + a_thrust_now * (1 - AT_AUTH_ALPHA).

  // Idle-decel envelope estimate for decel clamp.
  IF thr_cur <= AT_IDLE_THR_THRESH AND THROTTLE_CMD <= AT_IDLE_THR_THRESH + 0.05 {
    LOCAL idle_sample IS CLAMP(-A_ACTUAL_FILT, 0, AT_IDLE_DECEL_MAX).
    SET AT_IDLE_DECEL_EST TO AT_IDLE_DECEL_EST * AT_IDLE_ALPHA
                          + idle_sample * (1 - AT_IDLE_ALPHA).
  }

  SET AT_PREV_THR_CUR TO thr_cur.
  SET AT_PREV_A_FILT  TO A_ACTUAL_FILT.
}

FUNCTION AT_RUN_SPEED_HOLD {
  PARAMETER v_tgt, thr_min, thr_max.
  IF thr_min > thr_max {
    LOCAL tmp_thr IS thr_min.
    SET thr_min TO thr_max.
    SET thr_max TO tmp_thr.
  }

  LOCAL ias IS GET_IAS().
  LOCAL dt  IS MAX(IFC_ACTUAL_DT, 0.01).
  _AT_UPDATE_ESTIMATES(ias).

  LOCAL spd_err IS v_tgt - ias.

  // Accel/decel authority scheduling.
  LOCAL a_up_lim IS CLAMP(
    MAX(AT_A_THRUST_MAX_EST * AT_A_UP_FRAC, ACL_MAX * AT_A_UP_MIN_FRAC),
    AT_A_UP_MIN,
    AT_A_UP_MAX
  ).
  LOCAL a_dn_lim IS CLAMP(
    MAX(AT_IDLE_DECEL_EST * AT_A_DN_GAIN, ACL_MAX * AT_A_DN_MIN_FRAC),
    AT_A_DN_MIN,
    AT_A_DN_MAX
  ).
  LOCAL a_cmd IS CLAMP(KP_SPD_ACL * spd_err, -a_dn_lim, a_up_lim).
  LOCAL a_err IS a_cmd - A_ACTUAL_FILT.

  // Gain scheduling from estimated plant gain/lag.
  LOCAL inner_bw IS AC_PARAM("at_inner_bw", AT_INNER_BW, 0.001).
  LOCAL kp_thr_eff IS CLAMP(inner_bw / MAX(AT_GAIN_EST, AT_GAIN_MIN), AT_KP_THR_MIN, AT_KP_THR_MAX).
  LOCAL ki_scale IS kp_thr_eff / MAX(KP_ACL_THR, 0.001).
  LOCAL ki_spd_eff IS CLAMP(KI_SPD * ki_scale, AT_KI_SPD_MIN, AT_KI_SPD_MAX).

  LOCAL tau_ref IS AC_PARAM("at_tau_ref_s", AT_TAU_REF_S, 0.001).
  LOCAL thr_slew_eff IS CLAMP(
    THR_SLEW_PER_S * tau_ref / MAX(AT_TAU_EST, AT_TAU_MIN),
    AT_THR_SLEW_MIN,
    AT_THR_SLEW_MAX
  ).

  // Anti-windup: suspend integral growth when saturated in the same error direction.
  LOCAL raw_unsat IS kp_thr_eff * a_err + ki_spd_eff * THR_INTEGRAL.
  LOCAL raw_thr   IS CLAMP(raw_unsat, thr_min, thr_max).
  LOCAL sat_hi IS raw_unsat >= thr_max.
  LOCAL sat_lo IS raw_unsat <= thr_min.
  LOCAL block_int IS (sat_hi AND spd_err > 0) OR (sat_lo AND spd_err < 0).
  IF NOT block_int {
    SET THR_INTEGRAL TO CLAMP(THR_INTEGRAL + spd_err * dt, -THR_INTEGRAL_LIM, THR_INTEGRAL_LIM).
  } ELSE {
    SET THR_INTEGRAL TO THR_INTEGRAL * AT_INT_BLEED.
  }

  // Re-evaluate with updated integral state.
  SET raw_unsat TO kp_thr_eff * a_err + ki_spd_eff * THR_INTEGRAL.
  SET raw_thr   TO CLAMP(raw_unsat, thr_min, thr_max).
  SET THROTTLE_CMD TO MOVE_TOWARD(THROTTLE_CMD, raw_thr, thr_slew_eff * dt).

  // Export adaptive telemetry for debug/logging.
  SET TELEM_AT_V_TGT     TO v_tgt.
  SET TELEM_AT_A_CMD     TO a_cmd.
  SET TELEM_AT_A_ACT     TO A_ACTUAL_FILT.
  SET TELEM_AT_GAIN      TO AT_GAIN_EST.
  SET TELEM_AT_TAU       TO AT_TAU_EST.
  SET TELEM_AT_A_UP_LIM  TO a_up_lim.
  SET TELEM_AT_A_DN_LIM  TO a_dn_lim.
  SET TELEM_AT_KP_THR    TO kp_thr_eff.
  SET TELEM_AT_KI_SPD    TO ki_spd_eff.
  SET TELEM_AT_THR_SLEW  TO thr_slew_eff.
}
