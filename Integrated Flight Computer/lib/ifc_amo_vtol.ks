@LAZYGLOBAL OFF.

// ============================================================
// ifc_amo_vtol.ks  -  Integrated Flight Computer
//
// AMO VTOL Assist module
// ----------------------
// Tilt-rotor / tilt-pod VTOL guidance.
// Engines and servos discovered via numbered part tags.
// Geometry + thrust weighted mixing matrix for any N-pod layout.
//
// PILOT controls the physical throttle (overall power level).
// kOS controls ONLY the per-engine thrust limiters to maintain
// pitch/roll balance and apply pilot attitude commands.
//
// How limits are computed each tick:
//   limit[i] = CLAMP(base[i] + roll*roll_mix[i] + pitch*pitch_mix[i], 0, 1)
//
//   base[i]    = 1.0 + VTOL_TRIM_OFFSET[i]
//                Auto-computed at discovery to zero out the net pitch/roll
//                torque at flat inputs, accounting for unequal engine thrust
//                and asymmetric CoM placement.
//
//   roll_mix[i]  and  pitch_mix[i]
//                Thrust-weighted: coefficients are scaled by T_mean/T[i] so
//                a weaker engine gets a larger limit change to produce the
//                same torque as a stronger one.
//
// Servo yaw: differential tilt of wing pods, keyed by yaw stick.
//
// Part tagging (KSP editor ??? part tags, not IR group names):
//   Engines: vtol_eng_1 ... vtol_eng_N
//   Servos:  vtol_srv_1 ... vtol_srv_N
//
// Public API:
//   VTOL_DISCOVER()                    -- find engines + servos, compute geometry
//   VTOL_IR_DIAG()                     -- diagnostic string (servo count)
//   VTOL_TICK_PREARM()                 -- AMO tick; reads pilot inputs directly
//   VTOL_TICK(roll, pitch, yaw, thr)   -- autopilot tick; caller provides commands
//   VTOL_RELEASE()                     -- restore limits, stop servos, clear active
//   VTOL_RESET()                       -- full state clear
// ============================================================

// ?????? Config helpers ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
FUNCTION _VTOL_ENABLED {
  RETURN _AMO_CFG_BOOL("has_vtol", FALSE).
}

// ?????? Servo discovery via part tag ??????????????????????????????????????????????????????????????????????????????
FUNCTION _VTOL_FIND_SERVO_MODULE {
  PARAMETER tag_name.
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  IF tagged:LENGTH = 0 { RETURN 0. }
  LOCAL tagged_part IS tagged[0].
  LOCAL smods IS tagged_part:MODULESNAMED("ModuleIRServo_v3").
  IF smods:LENGTH = 0 { RETURN 0. }
  RETURN smods[0].
}

FUNCTION VTOL_IR_DIAG {
  LOCAL all_srv IS SHIP:MODULESNAMED("ModuleIRServo_v3").
  IF all_srv:LENGTH = 0 { RETURN "ModuleIRServo_v3: 0 found on vessel.". }
  RETURN "ModuleIRServo_v3: " + all_srv:LENGTH + " servo(s) found.".
}

// ?????? Part offset from CoM ?????????????????????????????????????????????????????????????????????????????????????????????????????????
// Returns LEXICON("lat", lateral_m, "lng", long_m).
// lat: + = starboard,  lng: + = forward
FUNCTION _VTOL_PART_OFFSET {
  PARAMETER p.
  LOCAL offset IS p:POSITION - SHIP:POSITION.
  RETURN LEXICON(
    "lat", VDOT(offset, SHIP:FACING:STARVECTOR),
    "lng", VDOT(offset, SHIP:FACING:FOREVECTOR)
  ).
}

// ?????? Discovery ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
FUNCTION VTOL_DISCOVER {
  IF VTOL_DISCOVERED { RETURN. }

  VTOL_ENG_LIST:CLEAR().
  VTOL_SRV_LIST:CLEAR().
  VTOL_ROLL_MIX:CLEAR().
  VTOL_PITCH_MIX:CLEAR().
  VTOL_YAW_SRV_MIX:CLEAR().
  VTOL_TRIM_OFFSET:CLEAR().
  VTOL_MAX_THRUST:CLEAR().
  SET VTOL_DIFF_AVAILABLE TO FALSE.
  SET VTOL_SRV_AVAIL      TO FALSE.

  LOCAL eng_tag IS _AMO_CFG_STR("vtol_eng_tag_prefix", "vtol_eng").
  LOCAL srv_tag IS _AMO_CFG_STR("vtol_srv_tag_prefix", "vtol_srv").

  // Cache SHIP:ENGINES once ??? traversing the part tree is expensive.
  LOCAL ship_engs IS SHIP:ENGINES.

  // ?????? Collect engines by tag number ???????????????????????????????????????????????????????????????????????????
  LOCAL raw_offsets IS LIST().
  LOCAL raw_thrust  IS LIST(). // max thrust per engine (kN)
  LOCAL all_maxthrust IS TRUE. // FALSE if any engine had to use THRUST fallback
  LOCAL n IS 1.
  UNTIL n > VTOL_MAX_PODS {
    LOCAL tagged IS SHIP:PARTSTAGGED(eng_tag + "_" + n).
    IF tagged:LENGTH = 0 { BREAK. }
    LOCAL tp IS tagged[0].
    LOCAL entry IS _AMO_MAKE_PART_ENTRY(tp).
    IF entry <> 0 {
      VTOL_ENG_LIST:ADD(entry).
      raw_offsets:ADD(_VTOL_PART_OFFSET(tp)).

      // Find engine object to read max thrust.
      // NOTE: propeller/fan engines report MAXTHRUST=0 at low/zero RPM.
      // If MAXTHRUST is unavailable, record 0 and set all_maxthrust=FALSE ???
      // the torque-balance will be skipped and adaptive trim used instead.
      // We intentionally do NOT fall back to THRUST here: THRUST reflects
      // current output under whatever limiters are set, not engine capability,
      // so it produces wildly wrong trim offsets whenever limiters are unequal.
      LOCAL thr_val IS 0.
      LOCAL ei IS 0.
      UNTIL ei >= ship_engs:LENGTH {
        LOCAL cand IS ship_engs[ei].
        IF cand:UID = tp:UID {
          SET thr_val TO cand:MAXTHRUST.
          IF thr_val < 0.1 { SET all_maxthrust TO FALSE. }
        }
        SET ei TO ei + 1.
      }
      raw_thrust:ADD(thr_val).
    }
    SET n TO n + 1.
  }

  // ?????? Collect servos by part tag ????????????????????????????????????????????????????????????????????????????????????
  LOCAL si IS 1.
  UNTIL si > VTOL_ENG_LIST:LENGTH {
    LOCAL srv IS _VTOL_FIND_SERVO_MODULE(srv_tag + "_" + si).
    VTOL_SRV_LIST:ADD(srv).
    IF srv <> 0 { SET VTOL_SRV_AVAIL TO TRUE. }
    SET si TO si + 1.
  }
  UNTIL VTOL_SRV_LIST:LENGTH >= VTOL_ENG_LIST:LENGTH {
    VTOL_SRV_LIST:ADD(0).
  }

  IF VTOL_ENG_LIST:LENGTH < 2 {
    SET VTOL_DISCOVERED TO TRUE.
    IFC_SET_ALERT("VTOL: only " + VTOL_ENG_LIST:LENGTH + " engine(s) -- need 2+", "WARN").
    RETURN.
  }

  // ?????? Thrust statistics ???????????????????????????????????????????????????????????????????????????????????????????????????????????????
  LOCAL t_sum IS 0.
  LOCAL ti IS 0.
  UNTIL ti >= raw_thrust:LENGTH {
    SET t_sum TO t_sum + raw_thrust[ti].
    SET ti TO ti + 1.
  }
  LOCAL t_count IS raw_thrust:LENGTH.
  LOCAL t_mean IS t_sum.
  IF t_count > 0 { SET t_mean TO t_sum / t_count. }
  IF t_mean < 0.01 { SET t_mean TO 1. } // guard: engines not running at discovery time

  // ?????? Normalisation ranges (position only, for direction) ?????????
  LOCAL max_lat IS 0.01.
  LOCAL max_lng IS 0.01.
  LOCAL ni IS 0.
  UNTIL ni >= raw_offsets:LENGTH {
    IF ABS(raw_offsets[ni]["lat"]) > max_lat { SET max_lat TO ABS(raw_offsets[ni]["lat"]). }
    IF ABS(raw_offsets[ni]["lng"]) > max_lng { SET max_lng TO ABS(raw_offsets[ni]["lng"]). }
    SET ni TO ni + 1.
  }

  // ?????? Thrust-weighted mixing coefficients ?????????????????????????????????????????????????????????
  // Scale each engine's coefficient by T_mean/T[i] so a weaker engine
  // gets a proportionally larger limit change to produce the same torque
  // as a stronger engine.
  LOCAL roll_gain  IS _AMO_CFG_NUM("vtol_roll_gain",  VTOL_ROLL_GAIN,  0).
  LOCAL pitch_gain IS _AMO_CFG_NUM("vtol_pitch_gain", VTOL_PITCH_GAIN, 0).
  LOCAL pitch_mix_sign IS _AMO_CFG_NUM("vtol_pitch_mix_sign", VTOL_PITCH_MIX_SIGN, -1).
  IF pitch_mix_sign < 0 { SET pitch_mix_sign TO -1. } ELSE { SET pitch_mix_sign TO 1. }
  LOCAL lat_thresh IS VTOL_LAT_SIGN_THRESH.

  SET ni TO 0.
  UNTIL ni >= raw_offsets:LENGTH {
    LOCAL lat   IS raw_offsets[ni]["lat"].
    LOCAL lng   IS raw_offsets[ni]["lng"].
    LOCAL thr_i IS raw_thrust[ni].
    LOCAL t_scale IS 1.
    IF thr_i > 0.01 { SET t_scale TO t_mean / thr_i. }

    VTOL_ROLL_MIX:ADD(CLAMP(-lat / max_lat, -1, 1) * t_scale * roll_gain).
    VTOL_PITCH_MIX:ADD(CLAMP(pitch_mix_sign * lng / max_lng, -1, 1) * t_scale * pitch_gain).

    LOCAL lat_sign IS 0.
    IF lat < -lat_thresh      { SET lat_sign TO -1. }
    ELSE IF lat > lat_thresh  { SET lat_sign TO  1. }
    VTOL_YAW_SRV_MIX:ADD(lat_sign).

    VTOL_MAX_THRUST:ADD(thr_i).

    SET ni TO ni + 1.
  }

  // ?????? Auto-trim: compute per-engine base offsets that zero ??????
  // the net pitch and roll torque at flat limits (all = 1.0).
  //
  // Only valid when MAXTHRUST is available for every engine (jet engines,
  // or propeller engines at full RPM with reliable MAXTHRUST).
  // If any engine returned MAXTHRUST=0 (propeller/fan at idle/cold),
  // skip this entirely ??? all offsets start at 0 and adaptive trim
  // (_VTOL_ADAPT_TRIM) converges them in flight from observed pitch error.
  // Using THRUST fallback here is dangerous: THRUST reflects current output
  // under whatever limiters are already set, producing wildly wrong offsets
  // whenever engines are running at unequal power levels.

  LOCAL static_trim_enabled IS _AMO_CFG_BOOL(
    "vtol_static_trim_discovery",
    VTOL_STATIC_TRIM_DISCOVERY_DEFAULT
  ).

  SET ni TO 0.
  IF static_trim_enabled AND all_maxthrust {
    LOCAL tau_p IS 0.
    LOCAL tau_r IS 0.
    UNTIL ni >= raw_offsets:LENGTH {
      SET tau_p TO tau_p + raw_thrust[ni] * raw_offsets[ni]["lng"].
      SET tau_r TO tau_r + raw_thrust[ni] * raw_offsets[ni]["lat"].
      SET ni TO ni + 1.
    }

    // Pitch trim ??? sum of (T[i]*lng[i])^2 on the over-torque side.
    LOCAL denom_p IS 0.
    SET ni TO 0.
    UNTIL ni >= raw_offsets:LENGTH {
      LOCAL moment IS raw_thrust[ni] * raw_offsets[ni]["lng"].
      IF (tau_p > 0 AND moment > 0) OR (tau_p < 0 AND moment < 0) {
        SET denom_p TO denom_p + moment * moment.
      }
      SET ni TO ni + 1.
    }

    // Roll trim
    LOCAL denom_r IS 0.
    SET ni TO 0.
    UNTIL ni >= raw_offsets:LENGTH {
      LOCAL moment_r IS raw_thrust[ni] * raw_offsets[ni]["lat"].
      IF (tau_r > 0 AND moment_r > 0) OR (tau_r < 0 AND moment_r < 0) {
        SET denom_r TO denom_r + moment_r * moment_r.
      }
      SET ni TO ni + 1.
    }

    // Build VTOL_TRIM_OFFSET as pitch + roll trim combined.
    LOCAL trim_base_min IS _AMO_CFG_NUM(
      "vtol_static_trim_base_min",
      VTOL_STATIC_TRIM_BASE_MIN,
      0.01
    ).
    IF trim_base_min > 0.95 { SET trim_base_min TO 0.95. }
    SET ni TO 0.
    UNTIL ni >= raw_offsets:LENGTH {
      LOCAL moment_p IS raw_thrust[ni] * raw_offsets[ni]["lng"].
      LOCAL moment_r IS raw_thrust[ni] * raw_offsets[ni]["lat"].

      LOCAL offset_p IS 0.
      IF ABS(denom_p) > 0.001 {
        IF (tau_p > 0 AND moment_p > 0) OR (tau_p < 0 AND moment_p < 0) {
          SET offset_p TO -tau_p * moment_p / denom_p.
        }
      }

      LOCAL offset_r IS 0.
      IF ABS(denom_r) > 0.001 {
        IF (tau_r > 0 AND moment_r > 0) OR (tau_r < 0 AND moment_r < 0) {
          SET offset_r TO -tau_r * moment_r / denom_r.
        }
      }

      // Clamp so base limit stays in (0, 1], with enough floor headroom
      // to preserve bidirectional differential authority.
      LOCAL base IS CLAMP(1.0 + offset_p + offset_r, trim_base_min, 1.0).
      VTOL_TRIM_OFFSET:ADD(base - 1.0).
      SET ni TO ni + 1.
    }
  } ELSE {
    // Start from neutral offsets when static trim is disabled or unavailable.
    // Adaptive trim will converge from observed pitch error in hover.
    UNTIL ni >= raw_offsets:LENGTH {
      VTOL_TRIM_OFFSET:ADD(0.0).
      SET ni TO ni + 1.
    }
  }

  SET VTOL_DIFF_AVAILABLE TO TRUE.
  SET VTOL_DISCOVERED     TO TRUE.

  LOCAL srv_count IS 0.
  SET ni TO 0.
  UNTIL ni >= VTOL_SRV_LIST:LENGTH {
    IF VTOL_SRV_LIST[ni] <> 0 { SET srv_count TO srv_count + 1. }
    SET ni TO ni + 1.
  }
  IFC_SET_ALERT(
    "VTOL: " + VTOL_ENG_LIST:LENGTH + " engines, " + srv_count + " servos", "INFO"
  ).
}

// ?????? Engine limit application ?????????????????????????????????????????????????????????????????????????????????????????????
// Compute the feasible common-mode shift range for:
//   limit[i] = base[i] + shift + alpha * diff[i]
// with per-engine bounds 0..1.
FUNCTION _VTOL_ALLOC_SHIFT_BOUNDS {
  PARAMETER base_vec, diff_vec, alpha.
  LOCAL shift_min IS -999.
  LOCAL shift_max IS  999.
  LOCAL bi IS 0.
  UNTIL bi >= base_vec:LENGTH {
    LOCAL lo_i IS -base_vec[bi] - alpha * diff_vec[bi].
    LOCAL hi_i IS  1.0 - base_vec[bi] - alpha * diff_vec[bi].
    IF lo_i > shift_min { SET shift_min TO lo_i. }
    IF hi_i < shift_max { SET shift_max TO hi_i. }
    SET bi TO bi + 1.
  }
  RETURN LEXICON("min", shift_min, "max", shift_max).
}

// collective_in scales the torque-balanced base for all engines:
//   base[i] = collective ?? (1 + VTOL_TRIM_OFFSET[i])
// Differential terms are allocated with hard bounds:
//   1) keep full roll/pitch demand whenever feasible,
//   2) apply a common-mode shift to preserve attitude authority,
//   3) only scale differential if no bounded solution exists.
FUNCTION _VTOL_APPLY_ENGINES {
  PARAMETER collective_in, roll_cmd, pitch_cmd.
  IF VTOL_ENG_LIST:LENGTH = 0 { RETURN. }

  LOCAL diff_min IS _AMO_CFG_NUM(
    "vtol_diff_collective_min",
    VTOL_DIFF_COLLECTIVE_MIN,
    0
  ).
  LOCAL diff_scale IS 1.
  IF collective_in <= diff_min {
    SET diff_scale TO 0.
  } ELSE {
    LOCAL span IS 1.0 - diff_min.
    IF span < 0.01 { SET span TO 0.01. }
    SET diff_scale TO CLAMP((collective_in - diff_min) / span, 0, 1).
  }

  LOCAL base_vec IS LIST().
  LOCAL diff_vec IS LIST().
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL base_i IS collective_in * (1.0 + VTOL_TRIM_OFFSET[i]).
    LOCAL diff_i IS diff_scale * (roll_cmd  * VTOL_ROLL_MIX[i]
                                + pitch_cmd * VTOL_PITCH_MIX[i]).
    base_vec:ADD(base_i).
    diff_vec:ADD(diff_i).
    SET i TO i + 1.
  }

  LOCAL alpha IS 1.0.
  LOCAL alloc_bounds IS _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, alpha).
  IF alloc_bounds["min"] > alloc_bounds["max"] {
    // Full differential is infeasible under 0..1 bounds.
    // Find maximum feasible alpha in [0,1].
    LOCAL lo IS 0.0.
    LOCAL hi IS 1.0.
    LOCAL it IS 0.
    UNTIL it >= 14 {
      LOCAL mid IS (lo + hi) * 0.5.
      LOCAL b_mid IS _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, mid).
      IF b_mid["min"] <= b_mid["max"] {
        SET lo TO mid.
      } ELSE {
        SET hi TO mid.
      }
      SET it TO it + 1.
    }
    SET alpha TO lo.
    SET alloc_bounds TO _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, alpha).
  }

  // Keep commanded collective when possible; otherwise shift only as much as
  // required to keep all engines inside hard bounds.
  LOCAL shift IS CLAMP(0.0, alloc_bounds["min"], alloc_bounds["max"]).

  SET VTOL_ALLOC_ALPHA TO alpha.
  SET VTOL_ALLOC_SHIFT TO shift.

  SET i TO 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL lim IS base_vec[i] + shift + alpha * diff_vec[i].
    _AMO_SET_ENTRY_LIMIT_FRAC(VTOL_ENG_LIST[i], CLAMP(lim, 0, 1)).
    SET i TO i + 1.
  }
}

// ?????? Servo control via ModuleIRServo_v3 ???????????????????????????????????????????????????????????????
FUNCTION _VTOL_APPLY_SERVOS {
  PARAMETER yaw_cmd.
  IF NOT VTOL_SRV_AVAIL { RETURN. }
  LOCAL hover_angle IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  LOCAL yaw_gain    IS _AMO_CFG_NUM("vtol_yaw_gain", VTOL_YAW_SRV_GAIN, 0).
  LOCAL i IS 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    LOCAL srv_mod IS VTOL_SRV_LIST[i].
    IF srv_mod <> 0 {
      LOCAL yaw_mix   IS VTOL_YAW_SRV_MIX[i].
      LOCAL tgt_angle IS hover_angle + yaw_cmd * yaw_gain * yaw_mix.
      LOCAL spd IS VTOL_SRV_SPEED.
      IF yaw_mix <> 0 { SET spd TO VTOL_SRV_YAW_SPEED. }
      srv_mod:SETFIELD("max speed", spd).
      srv_mod:SETFIELD("target position", tgt_angle).
    }
    SET i TO i + 1.
  }
}

FUNCTION _VTOL_STOP_SERVOS {
  IF NOT VTOL_SRV_AVAIL { RETURN. }
  LOCAL i IS 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    LOCAL srv_mod IS VTOL_SRV_LIST[i].
    IF srv_mod <> 0 {
      LOCAL cur_pos IS srv_mod:GETFIELD("current position").
      srv_mod:SETFIELD("target position", cur_pos).
    }
    SET i TO i + 1.
  }
}

// ?????? Pilot inputs ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
FUNCTION _VTOL_PILOT_INPUTS {
  LOCAL roll_in  IS 0.
  LOCAL pitch_in IS 0.
  LOCAL yaw_in   IS 0.
  IF SHIP:HASSUFFIX("CONTROL") {
    LOCAL ctrl IS SHIP:CONTROL.
    IF ctrl:HASSUFFIX("PILOTROLL")  { SET roll_in  TO ctrl:PILOTROLL. }
    IF ctrl:HASSUFFIX("PILOTPITCH") { SET pitch_in TO ctrl:PILOTPITCH. }
    IF ctrl:HASSUFFIX("PILOTYAW")   { SET yaw_in   TO ctrl:PILOTYAW. }
  }
  RETURN LEXICON("roll", roll_in, "pitch", pitch_in, "yaw", yaw_in).
}

// ?????? VS hold (PI controller) ??? used by autopilot tick only ??????
FUNCTION _VTOL_EFFECTIVE_GROUNDED {
  PARAMETER agl_limit.
  // If VTOL is already commanding meaningful collective, treat the craft
  // as airborne for control purposes even when KSP status still reports
  // LANDED/PRELAUNCH near the pad.
  LOCAL diff_min IS _AMO_CFG_NUM(
    "vtol_diff_collective_min",
    VTOL_DIFF_COLLECTIVE_MIN,
    0
  ).
  IF VTOL_COLLECTIVE > diff_min { RETURN FALSE. }

  LOCAL status_ground IS (SHIP:STATUS = "LANDED" OR SHIP:STATUS = "PRELAUNCH").
  IF NOT status_ground { RETURN FALSE. }
  IF GET_AGL() > agl_limit { RETURN FALSE. }
  LOCAL vs_contact_max IS _AMO_CFG_NUM(
    "vtol_ground_contact_vs_max",
    VTOL_GROUND_CONTACT_VS_MAX,
    0
  ).
  IF ABS(SHIP:VERTICALSPEED) > vs_contact_max { RETURN FALSE. }
  RETURN TRUE.
}

FUNCTION _VTOL_SLEW_COLLECTIVE {
  PARAMETER target_collective.
  LOCAL up_slew IS _AMO_CFG_NUM(
    "vtol_collective_up_slew_per_s",
    VTOL_COLLECTIVE_UP_SLEW_PER_S,
    0
  ).
  LOCAL dn_slew IS _AMO_CFG_NUM(
    "vtol_collective_dn_slew_per_s",
    VTOL_COLLECTIVE_DN_SLEW_PER_S,
    0
  ).
  IF up_slew < 0 { SET up_slew TO 0. }
  IF dn_slew < 0 { SET dn_slew TO 0. }

  LOCAL slew_rate IS dn_slew.
  IF target_collective > VTOL_COLLECTIVE { SET slew_rate TO up_slew. }
  IF slew_rate <= 0 { RETURN target_collective. }

  LOCAL step IS slew_rate * IFC_ACTUAL_DT.
  RETURN CLAMP(
    target_collective,
    VTOL_COLLECTIVE - step,
    VTOL_COLLECTIVE + step
  ).
}

FUNCTION _VTOL_VS_HOLD {
  PARAMETER thr_input.
  LOCAL vs_kp  IS _AMO_CFG_NUM("vtol_vs_kp",  VTOL_VS_KP,  0).
  LOCAL vs_ki  IS _AMO_CFG_NUM("vtol_vs_ki",  VTOL_VS_KI,  0).
  LOCAL max_vs IS _AMO_CFG_NUM("vtol_max_vs", VTOL_MAX_VS, 0.1).
  LOCAL coll_max IS _AMO_CFG_NUM("vtol_collective_max", VTOL_COLLECTIVE_MAX, 0.1).
  IF coll_max > 1.0 { SET coll_max TO 1.0. }

  // Seed hover_collective from config on first call (VTOL_HOVER_COLLECTIVE
  // starts at 0.50 from VTOL_RESET; if the aircraft config provides a
  // vtol_hover_collective value it overrides the default so the controller
  // starts at the right ballpark without needing to learn it first).
  IF NOT VTOL_HOVER_COLLECTIVE_SEEDED {
    LOCAL cfg_hc IS _AMO_CFG_NUM("vtol_hover_collective", -1, 0).
    IF cfg_hc > 0.01 AND cfg_hc <= 1.0 {
      SET VTOL_HOVER_COLLECTIVE TO cfg_hc.
    }
    SET VTOL_HOVER_COLLECTIVE_SEEDED TO TRUE.
  }

  SET VTOL_VS_CMD TO (thr_input - 0.5) * 2.0 * max_vs.

  IF VTOL_ALT_HOLD {
    LOCAL alt_kp  IS _AMO_CFG_NUM("vtol_alt_kp", VTOL_ALT_KP, 0).
    LOCAL alt_err IS VTOL_ALT_CMD - GET_AGL().
    SET VTOL_VS_CMD TO CLAMP(alt_err * alt_kp, -max_vs, max_vs).
  }

  // Ground freeze: aircraft cannot respond to VS commands while landed.
  // Running the integrator while grounded pre-winds it to its limit,
  // causing collective saturation immediately at liftoff with no VS
  // regulation possible until the integral unwinds.  Reset it every
  // tick while landed so liftoff always starts with a clean integrator.
  LOCAL ground_agl IS _AMO_CFG_NUM(
    "vtol_ground_contact_agl_m",
    VTOL_GROUND_CONTACT_AGL_M,
    0
  ).
  LOCAL is_grounded IS _VTOL_EFFECTIVE_GROUNDED(ground_agl).
  IF is_grounded {
    SET VTOL_VS_INTEGRAL TO 0.
    LOCAL vs_err_g IS VTOL_VS_CMD - SHIP:VERTICALSPEED.
    LOCAL target_g IS CLAMP(VTOL_HOVER_COLLECTIVE + vs_err_g * vs_kp, 0, coll_max).
    RETURN _VTOL_SLEW_COLLECTIVE(target_g).
  }

  LOCAL vs_err IS VTOL_VS_CMD - SHIP:VERTICALSPEED.

  // Anti-windup: only accumulate the integral when the unclamped output
  // is not already saturated in the same direction as the error.
  // This prevents the integrator from driving collective to its limit
  // during initial engine spool-up (aircraft on ground, can't achieve
  // commanded VS) and keeping it there after liftoff.
  LOCAL unclamped IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp + VTOL_VS_INTEGRAL * vs_ki.
  LOCAL at_ceil IS unclamped >= coll_max AND vs_err > 0.
  LOCAL at_floor IS unclamped <= 0.0 AND vs_err < 0.
  IF NOT at_ceil AND NOT at_floor {
    SET VTOL_VS_INTEGRAL TO CLAMP(
      VTOL_VS_INTEGRAL + vs_err * IFC_ACTUAL_DT,
      -VTOL_VS_INTEGRAL_LIM,
       VTOL_VS_INTEGRAL_LIM
    ).
  }

  LOCAL target_collective IS CLAMP(unclamped, 0, coll_max).
  LOCAL collective IS _VTOL_SLEW_COLLECTIVE(target_collective).

  IF ABS(vs_err) < 0.5 {
    SET VTOL_HOVER_COLLECTIVE TO
      VTOL_HOVER_COLLECTIVE + (collective - VTOL_HOVER_COLLECTIVE) * VTOL_HOVER_LEARN_RATE.
  }

  RETURN collective.
}

// ?????? Adaptive trim integrator ?????????????????????????????????????????????????????????????????????????????????????????????
// Each tick where the pilot is not commanding pitch, measure the
// current pitch angle and nudge VTOL_TRIM_OFFSET per engine to
// drive pitch toward zero.
//
// This removes the dependency on accurate MAXTHRUST at discovery
// time and continuously compensates for CoM shifts (fuel burn).
//
// Direction rule: SIGN(VTOL_PITCH_MIX[i]) tells us which way
// increasing this engine's limit pitches the nose.  To correct
// a nose-down error we nudge each engine in its nose-UP direction.
FUNCTION _VTOL_ADAPT_TRIM {
  IF VTOL_ENG_LIST:LENGTH = 0 { RETURN. }
  IF VTOL_PITCH_MIX:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }
  IF VTOL_ROLL_MIX:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }
  IF VTOL_TRIM_OFFSET:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }

  // Only adapt while actually hovering/airborne.
  LOCAL ground_agl IS _AMO_CFG_NUM(
    "vtol_ground_contact_agl_m",
    VTOL_GROUND_CONTACT_AGL_M,
    0
  ).
  LOCAL is_grounded IS _VTOL_EFFECTIVE_GROUNDED(ground_agl).
  IF is_grounded { RETURN. }
  LOCAL min_agl IS _AMO_CFG_NUM("vtol_trim_min_agl_m", VTOL_TRIM_MIN_AGL_M, 0).
  IF GET_AGL() < min_agl { RETURN. }
  LOCAL diff_min IS _AMO_CFG_NUM("vtol_diff_collective_min", VTOL_DIFF_COLLECTIVE_MIN, 0).
  IF VTOL_COLLECTIVE <= diff_min { RETURN. }

  // Freeze integrator while pilot is commanding pitch/roll.
  LOCAL pilot_pitch_in IS 0.
  LOCAL pilot_roll_in IS 0.
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTPITCH") {
    SET pilot_pitch_in TO SHIP:CONTROL:PILOTPITCH.
  }
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTROLL") {
    SET pilot_roll_in TO SHIP:CONTROL:PILOTROLL.
  }
  IF ABS(pilot_pitch_in) > VTOL_TRIM_DEADBAND { RETURN. }
  IF ABS(pilot_roll_in) > VTOL_TRIM_DEADBAND { RETURN. }

  // Current pitch: positive = nose up.
  LOCAL pitch_ang IS 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
  LOCAL bank_ang IS ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1)).
  LOCAL roll_rate_rads IS -VDOT(SHIP:ANGULARVEL, SHIP:FACING:FOREVECTOR).
  LOCAL pitch_rate_rads IS VDOT(SHIP:ANGULARVEL, SHIP:FACING:STARVECTOR).
  LOCAL roll_rate_degs IS roll_rate_rads * (180 / CONSTANT:PI).
  LOCAL pitch_rate_degs IS pitch_rate_rads * (180 / CONSTANT:PI).
  LOCAL trim_pitch_max IS _AMO_CFG_NUM("vtol_trim_active_pitch_max", VTOL_TRIM_ACTIVE_PITCH_MAX, 0).
  LOCAL trim_bank_max IS _AMO_CFG_NUM("vtol_trim_active_bank_max", VTOL_TRIM_ACTIVE_BANK_MAX, 0).
  LOCAL trim_pitch_rate_max IS _AMO_CFG_NUM("vtol_trim_active_rate_max", VTOL_TRIM_ACTIVE_RATE_MAX, 0).
  LOCAL trim_roll_rate_max IS _AMO_CFG_NUM("vtol_trim_active_roll_rate_max", VTOL_TRIM_ACTIVE_ROLL_RATE_MAX, 0).
  LOCAL trim_rate_lead_s IS _AMO_CFG_NUM("vtol_trim_rate_lead_s", VTOL_TRIM_RATE_LEAD_S, 0).
  LOCAL trim_activity_min IS _AMO_CFG_NUM("vtol_trim_activity_min", VTOL_TRIM_ACTIVITY_MIN, 0).
  IF trim_rate_lead_s < 0 { SET trim_rate_lead_s TO 0. }
  SET trim_activity_min TO CLAMP(trim_activity_min, 0, 1).

  // Error terms include rate lead so trim starts correcting as soon as the
  // craft starts rotating away from level, not only after a large angle builds.
  LOCAL pitch_err_raw IS -pitch_ang + pitch_rate_degs * trim_rate_lead_s.
  LOCAL pitch_err IS CLAMP(pitch_err_raw, -VTOL_TRIM_PITCH_CLAMP, VTOL_TRIM_PITCH_CLAMP).
  LOCAL trim_bank_clamp IS _AMO_CFG_NUM("vtol_trim_bank_clamp", VTOL_TRIM_BANK_CLAMP, 0.1).
  LOCAL bank_err_raw IS -bank_ang + roll_rate_degs * trim_rate_lead_s.
  LOCAL bank_err IS CLAMP(bank_err_raw, -trim_bank_clamp, trim_bank_clamp).
  IF ABS(pitch_err) < 0.1 AND ABS(bank_err) < 0.1 { RETURN. }

  // Soft activity gating: keep adapting during large excursions, but reduce
  // aggressiveness when angles/rates are far beyond the nominal tuning range.
  LOCAL pitch_scale IS 1.0.
  IF trim_pitch_max > 0 AND ABS(pitch_ang) > trim_pitch_max {
    SET pitch_scale TO MIN(pitch_scale, trim_pitch_max / ABS(pitch_ang)).
  }
  IF trim_pitch_rate_max > 0 AND ABS(pitch_rate_degs) > trim_pitch_rate_max {
    SET pitch_scale TO MIN(pitch_scale, trim_pitch_rate_max / ABS(pitch_rate_degs)).
  }
  LOCAL bank_scale IS 1.0.
  IF trim_bank_max > 0 AND ABS(bank_ang) > trim_bank_max {
    SET bank_scale TO MIN(bank_scale, trim_bank_max / ABS(bank_ang)).
  }
  IF trim_roll_rate_max > 0 AND ABS(roll_rate_degs) > trim_roll_rate_max {
    SET bank_scale TO MIN(bank_scale, trim_roll_rate_max / ABS(roll_rate_degs)).
  }
  IF pitch_scale < trim_activity_min { SET pitch_scale TO trim_activity_min. }
  IF bank_scale < trim_activity_min { SET bank_scale TO trim_activity_min. }

  LOCAL pitch_drive IS pitch_err * pitch_scale.
  LOCAL bank_drive IS bank_err * bank_scale.

  LOCAL trim_rate_pitch IS _AMO_CFG_NUM("vtol_trim_rate", VTOL_TRIM_RATE, 0).
  LOCAL trim_rate_roll IS _AMO_CFG_NUM("vtol_trim_roll_rate", VTOL_TRIM_ROLL_RATE, 0).
  LOCAL rate_pitch IS trim_rate_pitch * IFC_ACTUAL_DT.
  LOCAL rate_roll IS trim_rate_roll * IFC_ACTUAL_DT.
  LOCAL trim_min_offset IS _AMO_CFG_NUM("vtol_trim_min_offset", VTOL_TRIM_MIN_OFFSET, -1).
  LOCAL trim_max_offset IS _AMO_CFG_NUM("vtol_trim_max_offset", VTOL_TRIM_MAX_OFFSET, -1).
  IF trim_max_offset > 0 { SET trim_max_offset TO 0. }
  IF trim_min_offset > trim_max_offset {
    SET trim_min_offset TO trim_max_offset - 0.05.
  }

  LOCAL ai IS 0.
  UNTIL ai >= VTOL_ENG_LIST:LENGTH {
    LOCAL pmix IS VTOL_PITCH_MIX[ai].
    LOCAL rmix IS VTOL_ROLL_MIX[ai].
    LOCAL delta IS 0.

    IF ABS(pmix) > 0.001 AND ABS(pitch_drive) >= 0.1 {
      LOCAL pmix_sign IS 1.
      IF pmix < 0 { SET pmix_sign TO -1. }
      SET delta TO delta + rate_pitch * pitch_drive * pmix_sign.
    }

    IF ABS(rmix) > 0.001 AND ABS(bank_drive) >= 0.1 {
      LOCAL rmix_sign IS 1.
      IF rmix < 0 { SET rmix_sign TO -1. }
      SET delta TO delta + rate_roll * bank_drive * rmix_sign.
    }

    IF ABS(delta) > 0 {
      SET VTOL_TRIM_OFFSET[ai] TO CLAMP(VTOL_TRIM_OFFSET[ai] + delta, trim_min_offset, trim_max_offset).
    }
    SET ai TO ai + 1.
  }
}

// ?????? Public API ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

// AMO / pre-arm tick.
// No LOCK THROTTLE ??? pilot's physical throttle is read as a VS command.
// 50 % throttle ??? 0 m/s VS command (hover).
// kOS manages thrust limiters for: collective (VS hold), trim balance,
// wings-level auto-correction, and pilot differential (roll/pitch/yaw).
FUNCTION VTOL_TICK_PREARM {
  IF NOT _VTOL_ENABLED() {
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  VTOL_DISCOVER().
  IF NOT VTOL_DIFF_AVAILABLE {
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  // ?????? Pilot inputs ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  LOCAL inputs  IS _VTOL_PILOT_INPUTS().
  LOCAL p_roll  IS CLAMP(inputs["roll"],  -1, 1).
  LOCAL p_pitch IS CLAMP(inputs["pitch"], -1, 1).
  LOCAL p_yaw   IS CLAMP(inputs["yaw"],   -1, 1).
  LOCAL p_thr   IS 0.5.
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTMAINTHROTTLE") {
    SET p_thr TO SHIP:CONTROL:PILOTMAINTHROTTLE.
  }

  // ?????? VS hold: pilot throttle ??? collective limiter level ???????????????
  // 50 % throttle ??? 0 m/s VS command.  PI controller integrates
  // to whatever collective keeps that VS. No LOCK THROTTLE.
  SET VTOL_COLLECTIVE TO _VTOL_VS_HOLD(p_thr).

  // ?????? Adaptive trim (slow pitch balance integrator) ?????????????????????????????????
  _VTOL_ADAPT_TRIM().

  // ?????? Wings-level auto-correction ???????????????????????????????????????????????????????????????????????????????????????
  // When the pilot is not commanding roll or pitch, apply a
  // proportional restoring command from current attitude.
  // Full authority at ~20?? deviation with default gains.
  LOCAL roll_cmd  IS p_roll.
  LOCAL pitch_cmd IS p_pitch.
  LOCAL level_roll_kp IS _AMO_CFG_NUM("vtol_level_roll_kp", VTOL_LEVEL_ROLL_KP, 0).
  LOCAL level_roll_kd IS _AMO_CFG_NUM("vtol_level_roll_kd", VTOL_LEVEL_ROLL_KD, 0).
  LOCAL level_roll_ki IS _AMO_CFG_NUM("vtol_level_roll_ki", VTOL_LEVEL_ROLL_KI, 0).
  LOCAL level_pitch_kp IS _AMO_CFG_NUM("vtol_level_pitch_kp", VTOL_LEVEL_PITCH_KP, 0).
  LOCAL level_pitch_kd IS _AMO_CFG_NUM("vtol_level_pitch_kd", VTOL_LEVEL_PITCH_KD, 0).
  LOCAL level_pitch_ki IS _AMO_CFG_NUM("vtol_level_pitch_ki", VTOL_LEVEL_PITCH_KI, 0).
  LOCAL level_i_lim IS _AMO_CFG_NUM("vtol_level_i_lim", VTOL_LEVEL_I_LIM, 0).
  LOCAL level_on_ground IS _AMO_CFG_BOOL("vtol_level_on_ground", VTOL_LEVEL_ON_GROUND_DEFAULT).
  LOCAL level_min_agl IS _AMO_CFG_NUM("vtol_level_min_agl_m", VTOL_LEVEL_MIN_AGL_M, 0).
  LOCAL ground_agl IS _AMO_CFG_NUM("vtol_ground_contact_agl_m", VTOL_GROUND_CONTACT_AGL_M, 0).
  LOCAL is_grounded IS _VTOL_EFFECTIVE_GROUNDED(ground_agl).
  LOCAL level_active IS TRUE.
  IF NOT level_on_ground {
    IF is_grounded OR GET_AGL() < level_min_agl {
      SET level_active TO FALSE.
    }
  }
  // Integral gate: only accumulate when truly airborne (not LANDED/PRELAUNCH).
  // _VTOL_EFFECTIVE_GROUNDED returns FALSE as soon as collective > diff_min, so
  // level_active can become TRUE while the aircraft is still on its gear.  The
  // PD terms are fine in that state; the integral must not wind up against a
  // structural pitch bias that exists only because the gear is still loaded.
  LOCAL status_str IS SHIP:STATUS.
  LOCAL truly_airborne IS (status_str <> "LANDED" AND status_str <> "PRELAUNCH").

  // Angular rates from SHIP:ANGULARVEL (world frame, rad/s).
  // Dot with ship axis vectors to get component around each axis.
  // Nose-up pitch rate  = positive VDOT with STARVECTOR.
  // Roll-right roll rate = negative VDOT with FOREVECTOR.
  LOCAL ang_vel IS SHIP:ANGULARVEL.
  LOCAL pitch_rate_rads IS VDOT(ang_vel, SHIP:FACING:STARVECTOR).
  LOCAL roll_rate_rads  IS -VDOT(ang_vel, SHIP:FACING:FOREVECTOR).
  LOCAL deg_per_rad IS 180 / CONSTANT:PI.

  IF level_active AND ABS(p_roll) < VTOL_TRIM_DEADBAND {
    LOCAL bank_ang IS ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1)).
    LOCAL roll_rate_degs IS roll_rate_rads * deg_per_rad.
    LOCAL roll_err IS -bank_ang.
    LOCAL roll_unsat IS roll_err * level_roll_kp
                     + roll_rate_degs * level_roll_kd
                     + VTOL_LEVEL_ROLL_INT * level_roll_ki.
    LOCAL roll_at_hi IS roll_unsat >= 1 AND roll_err > 0.
    LOCAL roll_at_lo IS roll_unsat <= -1 AND roll_err < 0.
    IF NOT truly_airborne {
      SET VTOL_LEVEL_ROLL_INT TO 0.
    } ELSE IF NOT roll_at_hi AND NOT roll_at_lo {
      SET VTOL_LEVEL_ROLL_INT TO CLAMP(
        VTOL_LEVEL_ROLL_INT + roll_err * IFC_ACTUAL_DT,
        -level_i_lim,
         level_i_lim
      ).
    }
    // KSP angular velocity sign convention: positive roll_rate_rads = rolling LEFT.
    // To damp a right-roll (positive bank): bank_ang > 0, roll_rate_degs > 0 while diverging.
    // KD sign must oppose the rate: +roll_rate_degs * KD adds to the left-roll correction.
    SET roll_cmd TO CLAMP(
      roll_err         * level_roll_kp
      +roll_rate_degs  * level_roll_kd
      +VTOL_LEVEL_ROLL_INT * level_roll_ki,
      -1, 1).
  } ELSE {
    SET VTOL_LEVEL_ROLL_INT TO 0.
  }
  IF level_active AND ABS(p_pitch) < VTOL_TRIM_DEADBAND {
    LOCAL pitch_ang IS 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
    LOCAL pitch_rate_degs IS pitch_rate_rads * deg_per_rad.
    LOCAL pitch_err IS -pitch_ang.
    LOCAL pitch_unsat IS pitch_err * level_pitch_kp
                      + pitch_rate_degs * level_pitch_kd
                      + VTOL_LEVEL_PITCH_INT * level_pitch_ki.
    LOCAL pitch_at_hi IS pitch_unsat >= 1 AND pitch_err > 0.
    LOCAL pitch_at_lo IS pitch_unsat <= -1 AND pitch_err < 0.
    IF NOT truly_airborne {
      SET VTOL_LEVEL_PITCH_INT TO 0.
    } ELSE IF NOT pitch_at_hi AND NOT pitch_at_lo {
      SET VTOL_LEVEL_PITCH_INT TO CLAMP(
        VTOL_LEVEL_PITCH_INT + pitch_err * IFC_ACTUAL_DT,
        -level_i_lim,
         level_i_lim
      ).
    }
    // KSP angular velocity sign convention: positive pitch_rate_rads = pitching DOWN.
    // To damp a nose-up divergence (positive pitch_ang, negative pitch_rate_degs):
    // KD sign must oppose the rate: +pitch_rate_degs * KD adds to the nose-down correction.
    SET pitch_cmd TO CLAMP(
      pitch_err         * level_pitch_kp
      +pitch_rate_degs  * level_pitch_kd
      +VTOL_LEVEL_PITCH_INT * level_pitch_ki,
      -1, 1).
  } ELSE {
    SET VTOL_LEVEL_PITCH_INT TO 0.
  }

  _VTOL_APPLY_ENGINES(VTOL_COLLECTIVE, roll_cmd, pitch_cmd).
  _VTOL_APPLY_SERVOS(p_yaw).
  SET VTOL_ACTIVE TO TRUE.
}

// Autopilot tick.
// Caller provides attitude commands; throttle managed via VS hold controller.
// roll_cmd, pitch_cmd, yaw_cmd: -1..1
// thr_input: 0..1 (50% = hold VS, mapped to collective for engine limits)
// Caller is responsible for LOCK THROTTLE TO THROTTLE_CMD and setting
// THROTTLE_CMD to 1.0 before calling this.
FUNCTION VTOL_TICK {
  PARAMETER roll_cmd, pitch_cmd, yaw_cmd, thr_input.
  IF NOT _VTOL_ENABLED() {
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  VTOL_DISCOVER().
  IF NOT VTOL_DIFF_AVAILABLE {
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  SET VTOL_COLLECTIVE TO _VTOL_VS_HOLD(thr_input).
  _VTOL_APPLY_ENGINES(
    VTOL_COLLECTIVE,
    CLAMP(roll_cmd,  -1, 1),
    CLAMP(pitch_cmd, -1, 1)
  ).
  _VTOL_APPLY_SERVOS(CLAMP(yaw_cmd, -1, 1)).
  SET VTOL_ACTIVE TO TRUE.
}

// Restore engine limits, stop servos, clear active flag.
FUNCTION VTOL_RELEASE {
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    _AMO_RESTORE_ENTRY_LIMIT(VTOL_ENG_LIST[i]).
    SET i TO i + 1.
  }
  _VTOL_STOP_SERVOS().
  SET VTOL_VS_INTEGRAL TO 0.
  SET VTOL_LEVEL_ROLL_INT TO 0.
  SET VTOL_LEVEL_PITCH_INT TO 0.
  SET VTOL_ACTIVE TO FALSE.
}

// Full state clear ??? use when re-arming or switching aircraft.
FUNCTION VTOL_RESET {
  VTOL_RELEASE().
  VTOL_ENG_LIST:CLEAR().
  VTOL_SRV_LIST:CLEAR().
  VTOL_ROLL_MIX:CLEAR().
  VTOL_PITCH_MIX:CLEAR().
  VTOL_YAW_SRV_MIX:CLEAR().
  VTOL_TRIM_OFFSET:CLEAR().
  VTOL_MAX_THRUST:CLEAR().
  SET VTOL_DISCOVERED     TO FALSE.
  SET VTOL_DIFF_AVAILABLE TO FALSE.
  SET VTOL_SRV_AVAIL      TO FALSE.
  SET VTOL_HOVER_COLLECTIVE        TO 0.50.
  SET VTOL_HOVER_COLLECTIVE_SEEDED TO FALSE.
  SET VTOL_COLLECTIVE              TO 0.
  SET VTOL_VS_CMD           TO 0.
  SET VTOL_VS_INTEGRAL      TO 0.
  SET VTOL_ALT_HOLD         TO FALSE.
  SET VTOL_ALT_CMD          TO 0.
  SET VTOL_ALLOC_ALPHA      TO 1.0.
  SET VTOL_ALLOC_SHIFT      TO 0.0.
  SET VTOL_LEVEL_ROLL_INT   TO 0.0.
  SET VTOL_LEVEL_PITCH_INT  TO 0.0.
}
