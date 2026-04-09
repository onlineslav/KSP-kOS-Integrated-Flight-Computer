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

FUNCTION _VTOL_CLEAR_DIAG {
  SET VTOL_DIAG_LEVEL_ACTIVE        TO FALSE.
  SET VTOL_DIAG_TRULY_AIRBORNE      TO FALSE.
  SET VTOL_DIAG_IS_GROUNDED         TO FALSE.
  SET VTOL_DIAG_ROLL_ERR            TO 0.0.
  SET VTOL_DIAG_PITCH_ERR           TO 0.0.
  SET VTOL_DIAG_ROLL_P              TO 0.0.
  SET VTOL_DIAG_ROLL_I              TO 0.0.
  SET VTOL_DIAG_ROLL_D              TO 0.0.
  SET VTOL_DIAG_PITCH_P             TO 0.0.
  SET VTOL_DIAG_PITCH_I             TO 0.0.
  SET VTOL_DIAG_PITCH_D             TO 0.0.
  SET VTOL_DIAG_ROLL_UNSAT          TO 0.0.
  SET VTOL_DIAG_PITCH_UNSAT         TO 0.0.
  SET VTOL_DIAG_CMD_ROLL_PRECAP     TO 0.0.
  SET VTOL_DIAG_CMD_PITCH_PRECAP    TO 0.0.
  SET VTOL_DIAG_CMD_ROLL_POSTCAP    TO 0.0.
  SET VTOL_DIAG_CMD_PITCH_POSTCAP   TO 0.0.
  SET VTOL_DIAG_CMD_ROLL_POSTUPSET  TO 0.0.
  SET VTOL_DIAG_CMD_PITCH_POSTUPSET TO 0.0.
  SET VTOL_DIAG_CMD_ROLL_POSTSLEW   TO 0.0.
  SET VTOL_DIAG_CMD_PITCH_POSTSLEW  TO 0.0.
  SET VTOL_DIAG_DIFF_SCALE_RAW      TO 0.0.
  SET VTOL_DIAG_DIFF_SCALE_ATTN     TO 0.0.
  SET VTOL_DIAG_DIFF_SCALE_UPSET    TO 0.0.
  SET VTOL_DIAG_LIM_FLOOR_USE       TO 0.0.
  SET VTOL_DIAG_ALLOC_BMIN          TO 0.0.
  SET VTOL_DIAG_ALLOC_BMAX          TO 0.0.
  SET VTOL_DIAG_ALPHA_LIMITED       TO FALSE.
  SET VTOL_DIAG_CLAMP_LOW_COUNT     TO 0.0.
  SET VTOL_DIAG_CLAMP_HIGH_COUNT    TO 0.0.
  SET VTOL_DIAG_ROLL_MOMENT_PROXY   TO 0.0.
  SET VTOL_DIAG_PITCH_MOMENT_PROXY  TO 0.0.
  SET VTOL_DIAG_LIMIT_SPAN          TO 0.0.
  SET VTOL_DIAG_P_DOT_FILT          TO 0.0.
  SET VTOL_DIAG_Q_DOT_FILT          TO 0.0.
  SET VTOL_DIAG_ROLL_D_ACCEL        TO 0.0.
  SET VTOL_DIAG_PITCH_D_ACCEL       TO 0.0.
  SET VTOL_DIAG_COS_ATT_FILT        TO VTOL_COS_ATT_FILT.
  SET VTOL_DIAG_COLL_BEFORE_CORR    TO 0.0.
  SET VTOL_DIAG_COLL_AFTER_CORR     TO 0.0.
  SET VTOL_DIAG_PHYSICAL_ALLOC_USED TO FALSE.
  SET VTOL_DIAG_INERTIA_EST_ACTIVE  TO FALSE.
  SET VTOL_DIAG_I_ROLL_EST          TO 0.0.
  SET VTOL_DIAG_I_PITCH_EST         TO 0.0.
  SET VTOL_DIAG_I_ROLL_TAU          TO 0.0.
  SET VTOL_DIAG_I_PITCH_TAU         TO 0.0.
  SET VTOL_DIAG_I_ROLL_ALPHA        TO 0.0.
  SET VTOL_DIAG_I_PITCH_ALPHA       TO 0.0.
}

GLOBAL _vtol_prev_roll_cmd IS 0.
GLOBAL _vtol_prev_pitch_cmd IS 0.
GLOBAL _vtol_em_lag_filt_s IS -1.
GLOBAL _vtol_em_lag_filt_cycle_ut IS -1.
GLOBAL _vtol_roll_lag_filt_s IS -1.
GLOBAL _vtol_pitch_lag_filt_s IS -1.
GLOBAL _vtol_prev_mean_cmd_eff IS -1. // tracks mean(throttle*lim) to remove uniform ramp from lag estimate
GLOBAL _vtol_upset_latched IS FALSE.
GLOBAL _vtol_upset_entry_ut IS -1.
GLOBAL _vtol_prev_vel_hold_active IS FALSE.
GLOBAL _vtol_prev_pos_hold_active IS FALSE.
GLOBAL _vtol_prev_khv_active IS FALSE.
GLOBAL VTOL_YAW_INPUT_OVERRIDE_ACTIVE IS FALSE.
GLOBAL VTOL_YAW_INPUT_OVERRIDE_VAL IS 0.0.

FUNCTION _VTOL_SERVO_CURRENT_FIELDS {
  RETURN LIST(
    "current position",
    "Current Position",
    "currentPosition",
    "CurrentPosition",
    "current angle",
    "Current Angle",
    "currentAngle",
    "CurrentAngle",
    "position",
    "Position",
    "current",
    "Current",
    "angle",
    "Angle"
  ).
}

FUNCTION _VTOL_SERVO_TARGET_FIELDS {
  RETURN LIST(
    "target position",
    "Target Position",
    "targetPosition",
    "TargetPosition",
    "target angle",
    "Target Angle",
    "targetAngle",
    "TargetAngle",
    "target",
    "Target",
    "goal",
    "Goal"
  ).
}

FUNCTION _VTOL_SERVO_SPEED_FIELDS {
  RETURN LIST(
    "max speed",
    "Max Speed",
    "maxSpeed",
    "MaxSpeed",
    "speed",
    "Speed",
    "motor speed",
    "Motor Speed",
    "motorSpeed",
    "MotorSpeed"
  ).
}

FUNCTION _VTOL_SERVO_MODULE_NAMES {
  RETURN LIST(
    "ModuleIRServo_v3",
    "ModuleIRServo",
    "MuMechToggle",
    "MuMechServo",
    "IRServo",
    "IRServo_v3",
    "ModuleRoboticServoHinge",
    "ModuleRoboticServoRotor",
    "ModuleRoboticServoPiston"
  ).
}

FUNCTION _VTOL_LIST_HAS_MODULE {
  PARAMETER module_list, module_ref.
  LOCAL i IS 0.
  UNTIL i >= module_list:LENGTH {
    IF module_list[i] = module_ref { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _VTOL_MOD_HAS_ANY_FIELD {
  PARAMETER module_ref, names.
  IF module_ref = 0 { RETURN FALSE. }
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    IF module_ref:HASFIELD(names[i]) { RETURN TRUE. }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _VTOL_MOD_GET_NUM_FIELD {
  PARAMETER module_ref, names, fallback_val.
  IF module_ref = 0 { RETURN fallback_val. }
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    LOCAL field_name IS names[i].
    IF module_ref:HASFIELD(field_name) {
      RETURN module_ref:GETFIELD(field_name).
    }
    SET i TO i + 1.
  }
  RETURN fallback_val.
}

FUNCTION _VTOL_MOD_SET_NUM_FIELD {
  PARAMETER module_ref, names, set_val.
  IF module_ref = 0 { RETURN FALSE. }
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    LOCAL field_name IS names[i].
    IF module_ref:HASFIELD(field_name) {
      module_ref:SETFIELD(field_name, set_val).
      RETURN TRUE.
    }
    SET i TO i + 1.
  }
  RETURN FALSE.
}

FUNCTION _VTOL_IS_SERVO_MODULE {
  PARAMETER module_ref.
  IF module_ref = 0 { RETURN FALSE. }
  IF NOT _VTOL_MOD_HAS_ANY_FIELD(module_ref, _VTOL_SERVO_CURRENT_FIELDS()) { RETURN FALSE. }
  IF NOT _VTOL_MOD_HAS_ANY_FIELD(module_ref, _VTOL_SERVO_TARGET_FIELDS()) { RETURN FALSE. }
  RETURN TRUE.
}

FUNCTION _VTOL_SERVO_CURRENT_POS {
  PARAMETER srv_mod, fallback_deg.
  LOCAL pos_deg IS _VTOL_MOD_GET_NUM_FIELD(
    srv_mod,
    _VTOL_SERVO_CURRENT_FIELDS(),
    fallback_deg
  ).
  RETURN CLAMP(pos_deg, 0, 180).
}

FUNCTION _VTOL_SERVO_SET_TARGET {
  PARAMETER srv_mod, target_deg.
  RETURN _VTOL_MOD_SET_NUM_FIELD(
    srv_mod,
    _VTOL_SERVO_TARGET_FIELDS(),
    CLAMP(target_deg, 0, 180)
  ).
}

FUNCTION _VTOL_SERVO_SET_SPEED {
  PARAMETER srv_mod, speed_val.
  RETURN _VTOL_MOD_SET_NUM_FIELD(
    srv_mod,
    _VTOL_SERVO_SPEED_FIELDS(),
    speed_val
  ).
}

FUNCTION _VTOL_COLLECT_ALL_SERVO_MODULES {
  LOCAL out IS LIST().
  LOCAL names IS _VTOL_SERVO_MODULE_NAMES().
  LOCAL ni IS 0.
  UNTIL ni >= names:LENGTH {
    LOCAL mods IS SHIP:MODULESNAMED(names[ni]).
    LOCAL mi IS 0.
    UNTIL mi >= mods:LENGTH {
      LOCAL cand IS mods[mi].
      IF _VTOL_IS_SERVO_MODULE(cand) {
        IF NOT _VTOL_LIST_HAS_MODULE(out, cand) {
          out:ADD(cand).
        }
      }
      SET mi TO mi + 1.
    }
    SET ni TO ni + 1.
  }
  RETURN out.
}

FUNCTION _VTOL_SERVO_OFFSET_FROM_MOD {
  PARAMETER srv_mod.
  IF srv_mod = 0 { RETURN LEXICON("valid", FALSE, "lat", 0, "lng", 0). }
  IF NOT srv_mod:HASSUFFIX("PART") { RETURN LEXICON("valid", FALSE, "lat", 0, "lng", 0). }
  LOCAL srv_part IS srv_mod:PART.
  IF srv_part = 0 { RETURN LEXICON("valid", FALSE, "lat", 0, "lng", 0). }
  LOCAL offset_data IS _VTOL_PART_OFFSET(srv_part).
  RETURN LEXICON("valid", TRUE, "lat", offset_data["lat"], "lng", offset_data["lng"]).
}

FUNCTION _VTOL_FILL_MISSING_SERVOS {
  PARAMETER raw_offsets.
  IF VTOL_ENG_LIST:LENGTH = 0 { RETURN. }
  IF raw_offsets:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }
  LOCAL all_srv IS _VTOL_COLLECT_ALL_SERVO_MODULES().
  IF all_srv:LENGTH = 0 { RETURN. }

  LOCAL used IS LIST().
  LOCAL i IS 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    LOCAL srv_i IS VTOL_SRV_LIST[i].
    IF srv_i <> 0 {
      used:ADD(srv_i).
      SET VTOL_SRV_AVAIL TO TRUE.
    }
    SET i TO i + 1.
  }

  SET i TO 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    IF VTOL_SRV_LIST[i] = 0 {
      LOCAL eng_off IS raw_offsets[i].
      LOCAL best_idx IS -1.
      LOCAL best_d2 IS 1.0E12.
      LOCAL ai IS 0.
      UNTIL ai >= all_srv:LENGTH {
        LOCAL cand IS all_srv[ai].
        LOCAL already_used IS _VTOL_LIST_HAS_MODULE(used, cand).
        IF NOT already_used {
          LOCAL cand_off IS _VTOL_SERVO_OFFSET_FROM_MOD(cand).
          LOCAL d2 IS 1.0E12.
          IF cand_off["valid"] {
            LOCAL d_lat IS cand_off["lat"] - eng_off["lat"].
            LOCAL d_lng IS cand_off["lng"] - eng_off["lng"].
            SET d2 TO d_lat * d_lat + d_lng * d_lng.
          }
          IF d2 < best_d2 {
            SET best_d2 TO d2.
            SET best_idx TO ai.
          }
        }
        SET ai TO ai + 1.
      }
      IF best_idx >= 0 {
        LOCAL picked_srv IS all_srv[best_idx].
        SET VTOL_SRV_LIST[i] TO picked_srv.
        used:ADD(picked_srv).
        SET VTOL_SRV_AVAIL TO TRUE.
      }
    }
    SET i TO i + 1.
  }
}

// ?????? Servo discovery via part tag ??????????????????????????????????????????????????????????????????????????????
FUNCTION _VTOL_FIND_SERVO_MODULE {
  PARAMETER tag_name.
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  IF tagged:LENGTH = 0 { RETURN 0. }
  LOCAL tagged_part IS tagged[0].
  LOCAL module_name_candidates IS _VTOL_SERVO_MODULE_NAMES().
  LOCAL name_i IS 0.
  UNTIL name_i >= module_name_candidates:LENGTH {
    LOCAL cand_name IS module_name_candidates[name_i].
    LOCAL smods IS tagged_part:MODULESNAMED(cand_name).
    IF smods:LENGTH > 0 {
      RETURN smods[0].
    }
    SET name_i TO name_i + 1.
  }
  LOCAL all_mods IS tagged_part:MODULES.
  LOCAL mod_i IS 0.
  UNTIL mod_i >= all_mods:LENGTH {
    LOCAL mod_cand IS all_mods[mod_i].
    IF _VTOL_IS_SERVO_MODULE(mod_cand) {
      RETURN mod_cand.
    }
    SET mod_i TO mod_i + 1.
  }
  RETURN 0.
}

FUNCTION VTOL_IR_DIAG {
  LOCAL names IS _VTOL_SERVO_MODULE_NAMES().
  LOCAL total IS 0.
  LOCAL i IS 0.
  UNTIL i >= names:LENGTH {
    LOCAL mods IS SHIP:MODULESNAMED(names[i]).
    SET total TO total + mods:LENGTH.
    SET i TO i + 1.
  }
  IF total = 0 {
    RETURN "VTOL servo modules: 0 found (IR/robotics names checked).".
  }
  RETURN "VTOL servo modules found: " + total + ".".
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
  VTOL_ENG_ARM_X_M:CLEAR().
  VTOL_ENG_ARM_Y_M:CLEAR().
  VTOL_ENG_EFF_ACT_EST:CLEAR().
  VTOL_ENG_LAG_EST_S:CLEAR().
  SET VTOL_EFF_LAG_WORST_S TO 0.0.
  SET _vtol_prev_mean_cmd_eff TO -1.
  SET VTOL_ARM_ROLL_M TO 0.0.
  SET VTOL_ARM_PITCH_M TO 0.0.
  SET VTOL_DIFF_AVAILABLE TO FALSE.
  SET VTOL_SRV_AVAIL      TO FALSE.
  _VTOL_RESET_INERTIA_ESTIMATOR().

  LOCAL eng_tag IS _AMO_CFG_STR("vtol_eng_tag_prefix", "vtol_eng").
  LOCAL srv_tag IS _AMO_CFG_STR("vtol_srv_tag_prefix", "vtol_srv").

  // Cache SHIP:ENGINES once ??? traversing the part tree is expensive.
  LOCAL ship_engs IS SHIP:ENGINES.

  // ?????? Collect engines by tag number ???????????????????????????????????????????????????????????????????????????
  LOCAL raw_offsets IS LIST().
  LOCAL raw_thrust  IS LIST(). // max thrust per engine (kN)
  LOCAL all_maxthrust IS TRUE. // FALSE if any engine had to use THRUST fallback
  LOCAL pod_idx IS 1.
  UNTIL pod_idx > VTOL_MAX_PODS {
    LOCAL tagged IS SHIP:PARTSTAGGED(eng_tag + "_" + pod_idx).
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
    SET pod_idx TO pod_idx + 1.
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
  LOCAL srv_missing_before IS 0.
  SET si TO 0.
  UNTIL si >= VTOL_SRV_LIST:LENGTH {
    IF VTOL_SRV_LIST[si] = 0 { SET srv_missing_before TO srv_missing_before + 1. }
    SET si TO si + 1.
  }
  IF srv_missing_before > 0 {
    _VTOL_FILL_MISSING_SERVOS(raw_offsets).
    LOCAL srv_missing_after IS 0.
    SET si TO 0.
    UNTIL si >= VTOL_SRV_LIST:LENGTH {
      IF VTOL_SRV_LIST[si] = 0 { SET srv_missing_after TO srv_missing_after + 1. }
      SET si TO si + 1.
    }
    IF srv_missing_after < srv_missing_before {
      IFC_SET_ALERT(
        "VTOL: auto-bound " + (srv_missing_before - srv_missing_after) +
        " untagged servo(s)",
        "INFO"
      ).
    }
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
  LOCAL arm_roll_max_m IS 0.0.
  LOCAL arm_pitch_max_m IS 0.0.

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
    VTOL_ENG_ARM_X_M:ADD(lng).
    VTOL_ENG_ARM_Y_M:ADD(lat).
    IF ABS(lat) > arm_roll_max_m { SET arm_roll_max_m TO ABS(lat). }
    IF ABS(lng) > arm_pitch_max_m { SET arm_pitch_max_m TO ABS(lng). }

    SET ni TO ni + 1.
  }
  SET VTOL_ARM_ROLL_M TO arm_roll_max_m.
  SET VTOL_ARM_PITCH_M TO arm_pitch_max_m.

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
  PARAMETER base_vec, diff_vec, alpha, lim_min, lim_max.
  LOCAL shift_min IS -999.
  LOCAL shift_max IS  999.
  LOCAL bi IS 0.
  UNTIL bi >= base_vec:LENGTH {
    LOCAL lo_i IS lim_min - base_vec[bi] - alpha * diff_vec[bi].
    LOCAL hi_i IS lim_max - base_vec[bi] - alpha * diff_vec[bi].
    IF lo_i > shift_min { SET shift_min TO lo_i. }
    IF hi_i < shift_max { SET shift_max TO hi_i. }
    SET bi TO bi + 1.
  }
  RETURN LEXICON("min", shift_min, "max", shift_max).
}

FUNCTION _VTOL_AUTH_AXIS_SCALE {
  PARAMETER abs_val, soft_val, hard_val.
  IF hard_val <= soft_val { RETURN 1.0. }
  IF abs_val <= soft_val { RETURN 1.0. }
  IF abs_val >= hard_val { RETURN 0.0. }
  RETURN 1.0 - ((abs_val - soft_val) / (hard_val - soft_val)).
}

FUNCTION _VTOL_DIFF_ATTENUATION {
  PARAMETER starvec, forevec, upvec, angvel.
  LOCAL state_attn_enabled IS _AMO_CFG_BOOL(
    "vtol_diff_state_atten_enabled",
    VTOL_DIFF_STATE_ATTEN_ENABLED_DEFAULT
  ).
  IF NOT state_attn_enabled { RETURN 1.0. }

  LOCAL bank_abs IS ABS(ARCSIN(CLAMP(-VDOT(starvec, upvec), -1, 1))).
  LOCAL pitch_abs IS ABS(90 - VANG(forevec, upvec)).

  LOCAL deg_per_rad IS 180 / CONSTANT:PI.
  LOCAL roll_rate_abs IS ABS(-VDOT(angvel, forevec) * deg_per_rad).
  LOCAL pitch_rate_abs IS ABS(VDOT(angvel, starvec) * deg_per_rad).

  LOCAL bank_soft IS _AMO_CFG_NUM("vtol_diff_soft_bank_deg", VTOL_DIFF_SOFT_BANK_DEG, 0).
  LOCAL bank_hard IS _AMO_CFG_NUM("vtol_diff_hard_bank_deg", VTOL_DIFF_HARD_BANK_DEG, 0).
  LOCAL pitch_soft IS _AMO_CFG_NUM("vtol_diff_soft_pitch_deg", VTOL_DIFF_SOFT_PITCH_DEG, 0).
  LOCAL pitch_hard IS _AMO_CFG_NUM("vtol_diff_hard_pitch_deg", VTOL_DIFF_HARD_PITCH_DEG, 0).
  LOCAL rr_soft IS _AMO_CFG_NUM("vtol_diff_soft_roll_rate_degs", VTOL_DIFF_SOFT_ROLL_RATE_DEGS, 0).
  LOCAL rr_hard IS _AMO_CFG_NUM("vtol_diff_hard_roll_rate_degs", VTOL_DIFF_HARD_ROLL_RATE_DEGS, 0).
  LOCAL pr_soft IS _AMO_CFG_NUM("vtol_diff_soft_pitch_rate_degs", VTOL_DIFF_SOFT_PITCH_RATE_DEGS, 0).
  LOCAL pr_hard IS _AMO_CFG_NUM("vtol_diff_hard_pitch_rate_degs", VTOL_DIFF_HARD_PITCH_RATE_DEGS, 0).
  LOCAL attn_min IS _AMO_CFG_NUM("vtol_diff_atten_min", VTOL_DIFF_ATTEN_MIN, 0).
  IF attn_min < 0 { SET attn_min TO 0. }
  IF attn_min > 1 { SET attn_min TO 1. }

  LOCAL sb IS _VTOL_AUTH_AXIS_SCALE(bank_abs, bank_soft, bank_hard).
  LOCAL sp IS _VTOL_AUTH_AXIS_SCALE(pitch_abs, pitch_soft, pitch_hard).
  LOCAL sr IS _VTOL_AUTH_AXIS_SCALE(roll_rate_abs, rr_soft, rr_hard).
  LOCAL spr IS _VTOL_AUTH_AXIS_SCALE(pitch_rate_abs, pr_soft, pr_hard).

  LOCAL out IS MIN(sb, sp).
  IF sr < out { SET out TO sr. }
  IF spr < out { SET out TO spr. }
  IF VTOL_DIAG_LEVEL_ACTIVE {
    // Do not collapse stabilization authority while the level controller is active.
    LOCAL level_attn_min IS _AMO_CFG_NUM("vtol_diff_attn_level_min", 0.70, 0).
    IF level_attn_min < 0 { SET level_attn_min TO 0. }
    IF level_attn_min > 1 { SET level_attn_min TO 1. }
    IF out < level_attn_min { SET out TO level_attn_min. }
  }
  IF out < attn_min { SET out TO attn_min. }
  RETURN CLAMP(out, 0, 1).
}

// collective_in scales the torque-balanced base for all engines:
//   base[i] = collective ?? (1 + VTOL_TRIM_OFFSET[i])
// Differential terms are allocated with hard bounds:
//   1) keep full roll/pitch demand whenever feasible,
//   2) apply a common-mode shift to preserve attitude authority,
//   3) only scale differential if no bounded solution exists.
FUNCTION _VTOL_APPLY_ENGINES {
  PARAMETER collective_in, roll_cmd, pitch_cmd, starvec, forevec, upvec, angvel.
  IF VTOL_ENG_LIST:LENGTH = 0 { RETURN. }
  VTOL_ENG_LIM_ACTUAL:CLEAR().
  SET VTOL_L_CMD_PREV TO roll_cmd.
  SET VTOL_M_CMD_PREV TO pitch_cmd.

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
  SET VTOL_DIAG_DIFF_SCALE_RAW TO diff_scale.
  SET diff_scale TO diff_scale * _VTOL_DIFF_ATTENUATION(starvec, forevec, upvec, angvel).
  SET VTOL_DIAG_DIFF_SCALE_ATTN TO diff_scale.
  IF VTOL_UPSET_ACTIVE AND collective_in > diff_min {
    LOCAL upset_diff_min IS _AMO_CFG_NUM(
      "vtol_upset_diff_atten_min",
      VTOL_UPSET_DIFF_ATTEN_MIN,
      0
    ).
    IF upset_diff_min < 0 { SET upset_diff_min TO 0. }
    IF upset_diff_min > 1 { SET upset_diff_min TO 1. }
    IF diff_scale < upset_diff_min { SET diff_scale TO upset_diff_min. }
  }
  SET VTOL_DIAG_DIFF_SCALE_UPSET TO diff_scale.

  LOCAL lim_floor_cfg IS _AMO_CFG_NUM("vtol_engine_limit_floor", VTOL_ENGINE_LIMIT_FLOOR, 0).
  IF lim_floor_cfg < 0 { SET lim_floor_cfg TO 0. }
  IF lim_floor_cfg > 0.45 { SET lim_floor_cfg TO 0.45. }
  LOCAL lim_floor_use IS 0.
  IF collective_in > diff_min {
    SET lim_floor_use TO MIN(lim_floor_cfg, collective_in * 0.60).
  }
  IF VTOL_UPSET_ACTIVE AND collective_in > diff_min {
    LOCAL upset_floor IS _AMO_CFG_NUM(
      "vtol_upset_engine_limit_floor",
      VTOL_UPSET_ENGINE_LIMIT_FLOOR,
      0
    ).
    IF upset_floor < 0 { SET upset_floor TO 0. }
    IF upset_floor > 0.45 { SET upset_floor TO 0.45. }
    IF upset_floor < lim_floor_use { SET lim_floor_use TO upset_floor. }
  }
  SET VTOL_DIAG_LIM_FLOOR_USE TO lim_floor_use.

  LOCAL base_vec IS LIST().
  LOCAL diff_vec IS LIST().
  LOCAL physical_alloc_enabled IS _AMO_CFG_BOOL(
    "vtol_physical_alloc_enabled",
    VTOL_PHYSICAL_ALLOC_ENABLED_DEFAULT
  ).
  SET VTOL_DIAG_PHYSICAL_ALLOC_USED TO physical_alloc_enabled.
  LOCAL roll_gain_cfg IS _AMO_CFG_NUM("vtol_roll_gain", VTOL_ROLL_GAIN, 0).
  LOCAL pitch_gain_cfg IS _AMO_CFG_NUM("vtol_pitch_gain", VTOL_PITCH_GAIN, 0).
  LOCAL pitch_mix_sign_cfg IS _AMO_CFG_NUM("vtol_pitch_mix_sign", VTOL_PITCH_MIX_SIGN, -1).
  IF pitch_mix_sign_cfg < 0 { SET pitch_mix_sign_cfg TO -1. } ELSE { SET pitch_mix_sign_cfg TO 1. }
  LOCAL vector_comp_enabled IS _AMO_CFG_BOOL("vtol_nacelle_vector_comp", TRUE).
  LOCAL sin_alpha_floor IS _AMO_CFG_NUM("vtol_nacelle_sin_floor", VTOL_NACELLE_SIN_FLOOR, 0.01).
  LOCAL diff_gain_max IS _AMO_CFG_NUM("vtol_nacelle_diff_gain_max", 2.0, 1.0).
  IF sin_alpha_floor < 0.01 { SET sin_alpha_floor TO 0.01. }
  IF sin_alpha_floor > 0.5 { SET sin_alpha_floor TO 0.5. }
  IF diff_gain_max < 1.0 { SET diff_gain_max TO 1.0. }
  LOCAL lift_frac_vec IS LIST().
  LOCAL diff_gain_vec IS LIST().
  LOCAL vg_i IS 0.
  UNTIL vg_i >= VTOL_ENG_LIST:LENGTH {
    LOCAL lift_frac_i IS 1.0.
    IF vector_comp_enabled {
      LOCAL alpha_i_deg IS VTOL_NACELLE_ALPHA_EST.
      IF vg_i < VTOL_SRV_LIST:LENGTH {
        LOCAL srv_i IS VTOL_SRV_LIST[vg_i].
        IF srv_i <> 0 {
          SET alpha_i_deg TO _VTOL_SERVO_CURRENT_POS(srv_i, alpha_i_deg).
        }
      }
      SET alpha_i_deg TO CLAMP(alpha_i_deg, 0, 180).
      SET lift_frac_i TO SIN(alpha_i_deg).
      IF lift_frac_i < sin_alpha_floor { SET lift_frac_i TO sin_alpha_floor. }
      IF lift_frac_i > 1.0 { SET lift_frac_i TO 1.0. }
    }
    LOCAL diff_gain_i IS 1.0.
    IF vector_comp_enabled {
      SET diff_gain_i TO 1.0 / lift_frac_i.
      IF diff_gain_i > diff_gain_max { SET diff_gain_i TO diff_gain_max. }
    }
    lift_frac_vec:ADD(lift_frac_i).
    diff_gain_vec:ADD(diff_gain_i).
    SET vg_i TO vg_i + 1.
  }
  LOCAL roll_arm_use_m IS VTOL_ARM_ROLL_M.
  LOCAL pitch_arm_use_m IS VTOL_ARM_PITCH_M.
  IF roll_arm_use_m < 0.01 { SET roll_arm_use_m TO 0.01. }
  IF pitch_arm_use_m < 0.01 { SET pitch_arm_use_m TO 0.01. }
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL base_i IS collective_in * (1.0 + VTOL_TRIM_OFFSET[i]).
    LOCAL diff_i IS 0.0.
    LOCAL diff_gain_i IS diff_gain_vec[i].
    IF physical_alloc_enabled AND VTOL_ENG_ARM_X_M:LENGTH = VTOL_ENG_LIST:LENGTH AND VTOL_ENG_ARM_Y_M:LENGTH = VTOL_ENG_LIST:LENGTH {
      LOCAL arm_x_m IS VTOL_ENG_ARM_X_M[i].
      LOCAL arm_y_m IS VTOL_ENG_ARM_Y_M[i].
      LOCAL roll_frac IS CLAMP(-arm_y_m / roll_arm_use_m, -1, 1).
      LOCAL pitch_frac IS CLAMP(pitch_mix_sign_cfg * arm_x_m / pitch_arm_use_m, -1, 1).
      SET diff_i TO diff_scale * diff_gain_i * (roll_cmd * roll_frac * roll_gain_cfg + pitch_cmd * pitch_frac * pitch_gain_cfg).
    } ELSE {
      SET diff_i TO diff_scale * diff_gain_i * (roll_cmd * VTOL_ROLL_MIX[i] + pitch_cmd * VTOL_PITCH_MIX[i]).
    }
    base_vec:ADD(base_i).
    diff_vec:ADD(diff_i).
    SET i TO i + 1.
  }

  LOCAL alpha IS 1.0.
  SET VTOL_DIAG_ALPHA_LIMITED TO FALSE.
  LOCAL alloc_bounds IS _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, alpha, lim_floor_use, 1.0).
  IF alloc_bounds["min"] > alloc_bounds["max"] {
    // Full differential is infeasible under 0..1 bounds.
    // Find maximum feasible alpha in [0,1].
    LOCAL lo IS 0.0.
    LOCAL hi IS 1.0.
    LOCAL it IS 0.
    UNTIL it >= 14 {
      LOCAL mid IS (lo + hi) * 0.5.
      LOCAL b_mid IS _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, mid, lim_floor_use, 1.0).
      IF b_mid["min"] <= b_mid["max"] {
        SET lo TO mid.
      } ELSE {
        SET hi TO mid.
      }
      SET it TO it + 1.
    }
    SET alpha TO lo.
    SET VTOL_DIAG_ALPHA_LIMITED TO TRUE.
    SET alloc_bounds TO _VTOL_ALLOC_SHIFT_BOUNDS(base_vec, diff_vec, alpha, lim_floor_use, 1.0).
  }
  SET VTOL_DIAG_ALLOC_BMIN TO alloc_bounds["min"].
  SET VTOL_DIAG_ALLOC_BMAX TO alloc_bounds["max"].

  // Keep commanded collective when possible; otherwise shift only as much as
  // required to keep all engines inside hard bounds.
  LOCAL shift IS CLAMP(0.0, alloc_bounds["min"], alloc_bounds["max"]).

  SET VTOL_ALLOC_ALPHA TO alpha.
  SET VTOL_ALLOC_SHIFT TO shift.

  LOCAL clamp_low_n IS 0.
  LOCAL clamp_high_n IS 0.
  LOCAL lim_min_seen IS 999.
  LOCAL lim_max_seen IS -999.
  LOCAL roll_moment_proxy IS 0.
  LOCAL pitch_moment_proxy IS 0.
  LOCAL roll_tau_cmd_kNm IS 0.0.
  LOCAL pitch_tau_cmd_kNm IS 0.0.
  LOCAL throttle_frac IS CLAMP(THROTTLE, 0, 1).
  SET i TO 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL lift_frac_i IS lift_frac_vec[i].
    LOCAL lim_unclamped IS base_vec[i] + shift + alpha * diff_vec[i].
    LOCAL lim_cmd IS CLAMP(lim_unclamped, lim_floor_use, 1).
    IF lim_unclamped <= lim_floor_use { SET clamp_low_n TO clamp_low_n + 1. }
    IF lim_unclamped >= 1 { SET clamp_high_n TO clamp_high_n + 1. }
    IF lim_cmd < lim_min_seen { SET lim_min_seen TO lim_cmd. }
    IF lim_cmd > lim_max_seen { SET lim_max_seen TO lim_cmd. }
    SET roll_moment_proxy TO roll_moment_proxy + lim_cmd * VTOL_ROLL_MIX[i] * lift_frac_i.
    SET pitch_moment_proxy TO pitch_moment_proxy + lim_cmd * VTOL_PITCH_MIX[i] * lift_frac_i.
    LOCAL lim_base_i IS base_vec[i] + shift.
    LOCAL diff_eff_i IS lim_cmd - lim_base_i.
    LOCAL thrust_cap_kN IS 0.0.
    IF VTOL_MAX_THRUST:LENGTH = VTOL_ENG_LIST:LENGTH {
      SET thrust_cap_kN TO VTOL_MAX_THRUST[i].
    }
    IF thrust_cap_kN < 0.01 {
      LOCAL entry_i IS VTOL_ENG_LIST[i].
      IF entry_i <> 0 AND entry_i:HASKEY("eng") {
        LOCAL eng_i IS entry_i["eng"].
        IF eng_i <> 0 {
          IF eng_i:HASSUFFIX("MAXTHRUST") { SET thrust_cap_kN TO eng_i:MAXTHRUST. }
          IF thrust_cap_kN < 0.01 AND eng_i:HASSUFFIX("THRUST") {
            SET thrust_cap_kN TO eng_i:THRUST.
          }
        }
      }
    }
    IF thrust_cap_kN < 0 { SET thrust_cap_kN TO 0. }
    LOCAL thrust_cmd_kN IS thrust_cap_kN * throttle_frac * lift_frac_i.
    IF VTOL_ENG_ARM_Y_M:LENGTH = VTOL_ENG_LIST:LENGTH {
      SET roll_tau_cmd_kNm TO roll_tau_cmd_kNm + diff_eff_i * thrust_cmd_kN * (-VTOL_ENG_ARM_Y_M[i]).
    }
    IF VTOL_ENG_ARM_X_M:LENGTH = VTOL_ENG_LIST:LENGTH {
      SET pitch_tau_cmd_kNm TO pitch_tau_cmd_kNm + diff_eff_i * thrust_cmd_kN * (pitch_mix_sign_cfg * VTOL_ENG_ARM_X_M[i]).
    }
    _AMO_SET_ENTRY_LIMIT_FRAC(VTOL_ENG_LIST[i], lim_cmd).
    VTOL_ENG_LIM_ACTUAL:ADD(lim_cmd).
    SET i TO i + 1.
  }
  SET VTOL_DIAG_CLAMP_LOW_COUNT TO clamp_low_n.
  SET VTOL_DIAG_CLAMP_HIGH_COUNT TO clamp_high_n.
  SET VTOL_DIAG_ROLL_MOMENT_PROXY TO roll_moment_proxy.
  SET VTOL_DIAG_PITCH_MOMENT_PROXY TO pitch_moment_proxy.
  IF lim_min_seen > lim_max_seen {
    SET VTOL_DIAG_LIMIT_SPAN TO 0.
  } ELSE {
    SET VTOL_DIAG_LIMIT_SPAN TO lim_max_seen - lim_min_seen.
  }
  _VTOL_UPDATE_EFFECTIVE_SPOOL_MODEL(throttle_frac, VTOL_ENG_LIM_ACTUAL).
  _VTOL_UPDATE_INERTIA_ESTIMATOR(roll_tau_cmd_kNm, pitch_tau_cmd_kNm).
}

// ?????? Servo control via ModuleIRServo_v3 ???????????????????????????????????????????????????????????????
FUNCTION _VTOL_APPLY_SERVOS {
  PARAMETER yaw_cmd.
  IF NOT VTOL_SRV_AVAIL { RETURN. }
  LOCAL hover_angle IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  LOCAL yaw_gain    IS _AMO_CFG_NUM("vtol_yaw_gain", VTOL_YAW_SRV_GAIN, 0).
  LOCAL yaw_sign_cfg IS _AMO_CFG_NUM("vtol_yaw_sign", 1, -1).
  LOCAL yaw_deadband_cmd IS _AMO_CFG_NUM("vtol_yaw_cmd_deadband", 0.0, 0).
  LOCAL yaw_min_deflection_deg IS _AMO_CFG_NUM("vtol_yaw_min_deflection_deg", 0.0, 0).
  LOCAL srv_speed_base IS _AMO_CFG_NUM("vtol_srv_speed", VTOL_SRV_SPEED, 0.1).
  LOCAL srv_speed_yaw IS _AMO_CFG_NUM("vtol_srv_yaw_speed", VTOL_SRV_YAW_SPEED, 0.1).
  IF yaw_sign_cfg < 0 { SET yaw_sign_cfg TO -1. } ELSE { SET yaw_sign_cfg TO 1. }
  IF yaw_deadband_cmd < 0 { SET yaw_deadband_cmd TO 0. }
  IF yaw_deadband_cmd > 0.30 { SET yaw_deadband_cmd TO 0.30. }
  IF yaw_min_deflection_deg < 0 { SET yaw_min_deflection_deg TO 0.0. }
  IF yaw_min_deflection_deg > 20 { SET yaw_min_deflection_deg TO 20. }
  IF srv_speed_base < 0.1 { SET srv_speed_base TO 0.1. }
  IF srv_speed_yaw < 0.1 { SET srv_speed_yaw TO 0.1. }
  LOCAL yaw_cmd_use IS CLAMP(yaw_cmd, -1, 1) * yaw_sign_cfg.
  IF ABS(yaw_cmd_use) < yaw_deadband_cmd {
    SET yaw_cmd_use TO 0.
  }
  LOCAL base_alpha_cmd IS VTOL_NACELLE_ALPHA_CMD.
  IF base_alpha_cmd < 0.1 OR base_alpha_cmd > 179.9 {
    SET base_alpha_cmd TO hover_angle.
    SET VTOL_NACELLE_ALPHA_CMD TO base_alpha_cmd.
  }
  LOCAL i IS 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    LOCAL srv_mod IS VTOL_SRV_LIST[i].
    IF srv_mod <> 0 {
      LOCAL yaw_mix   IS VTOL_YAW_SRV_MIX[i].
      LOCAL yaw_delta_deg IS yaw_cmd_use * yaw_gain * yaw_mix.
      IF ABS(yaw_delta_deg) > 0 AND ABS(yaw_delta_deg) < yaw_min_deflection_deg {
        IF yaw_delta_deg < 0 {
          SET yaw_delta_deg TO -yaw_min_deflection_deg.
        } ELSE {
          SET yaw_delta_deg TO yaw_min_deflection_deg.
        }
      }
      LOCAL tgt_angle IS CLAMP(base_alpha_cmd + yaw_delta_deg, 0, 180).
      LOCAL spd IS srv_speed_base.
      IF yaw_mix <> 0 { SET spd TO srv_speed_yaw. }
      _VTOL_SERVO_SET_SPEED(srv_mod, spd).
      _VTOL_SERVO_SET_TARGET(srv_mod, tgt_angle).
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
      LOCAL cur_pos IS _VTOL_SERVO_CURRENT_POS(srv_mod, VTOL_NACELLE_ALPHA_CMD).
      _VTOL_SERVO_SET_TARGET(srv_mod, cur_pos).
    }
    SET i TO i + 1.
  }
}

// External yaw input override for test/autopilot harnesses.
// When enabled, pre-arm tick uses this yaw command instead of SHIP:CONTROL:PILOTYAW.
FUNCTION VTOL_SET_YAW_INPUT_OVERRIDE {
  PARAMETER yaw_cmd.
  SET VTOL_YAW_INPUT_OVERRIDE_VAL TO CLAMP(yaw_cmd, -1, 1).
  SET VTOL_YAW_INPUT_OVERRIDE_ACTIVE TO TRUE.
}

FUNCTION VTOL_CLEAR_YAW_INPUT_OVERRIDE {
  SET VTOL_YAW_INPUT_OVERRIDE_ACTIVE TO FALSE.
  SET VTOL_YAW_INPUT_OVERRIDE_VAL TO 0.0.
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
  IF VTOL_YAW_INPUT_OVERRIDE_ACTIVE {
    SET yaw_in TO VTOL_YAW_INPUT_OVERRIDE_VAL.
  }
  RETURN LEXICON("roll", roll_in, "pitch", pitch_in, "yaw", yaw_in).
}

// ?????? VS hold (PI controller) ??? used by autopilot tick only ??????
FUNCTION _VTOL_EFFECTIVE_GROUNDED {
  PARAMETER agl_limit.
  // Status flags can lag a little at liftoff. Use kinematics to avoid
  // freezing controllers once upward motion is clearly established.
  LOCAL diff_min IS _AMO_CFG_NUM(
    "vtol_diff_collective_min",
    VTOL_DIFF_COLLECTIVE_MIN,
    0
  ).
  LOCAL liftoff_agl_m IS _AMO_CFG_NUM(
    "vtol_ground_liftoff_agl_m",
    VTOL_GROUND_LIFTOFF_AGL_M,
    0
  ).
  LOCAL liftoff_vs_min IS _AMO_CFG_NUM(
    "vtol_ground_liftoff_vs_min",
    VTOL_GROUND_LIFTOFF_VS_MIN,
    0
  ).
  IF liftoff_agl_m < 0 { SET liftoff_agl_m TO 0. }
  IF liftoff_vs_min < 0 { SET liftoff_vs_min TO 0. }
  IF VTOL_COLLECTIVE > diff_min {
    IF GET_AGL() > liftoff_agl_m { RETURN FALSE. }
    IF SHIP:VERTICALSPEED > liftoff_vs_min { RETURN FALSE. }
  }

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

  LOCAL slew_step IS slew_rate * IFC_ACTUAL_DT.
  RETURN CLAMP(
    target_collective,
    VTOL_COLLECTIVE - slew_step,
    VTOL_COLLECTIVE + slew_step
  ).
}

FUNCTION _VTOL_FILTER_LAG {
  PARAMETER prev_lag, raw_lag, tau_s.
  LOCAL raw_use IS raw_lag.
  IF raw_use < 0 { SET raw_use TO 0. }
  IF tau_s <= 0 { RETURN raw_use. }
  IF prev_lag < 0 { RETURN raw_use. }
  LOCAL alpha IS IFC_ACTUAL_DT / (IFC_ACTUAL_DT + tau_s).
  IF alpha < 0 { SET alpha TO 0. }
  IF alpha > 1 { SET alpha TO 1. }
  RETURN prev_lag + (raw_use - prev_lag) * alpha.
}

FUNCTION _VTOL_FILTERED_EM_LAG_S {
  IF _vtol_em_lag_filt_s >= 0 AND _vtol_em_lag_filt_cycle_ut = IFC_CYCLE_UT {
    RETURN _vtol_em_lag_filt_s.
  }
  LOCAL lag_tau_s IS _AMO_CFG_NUM("vtol_lag_filter_tau_s", VTOL_LAG_FILTER_TAU_S, 0).
  // Use the larger of global engine-model lag and limiter*throttle effective lag.
  // This matches the 4.3 model where differential limiter changes also spool.
  LOCAL raw_lag_s IS MAX(TELEM_EM_WORST_SPOOL_LAG, VTOL_EFF_LAG_WORST_S).
  SET _vtol_em_lag_filt_s TO _VTOL_FILTER_LAG(_vtol_em_lag_filt_s, raw_lag_s, lag_tau_s).
  SET _vtol_em_lag_filt_cycle_ut TO IFC_CYCLE_UT.
  RETURN _vtol_em_lag_filt_s.
}

FUNCTION _VTOL_UPDATE_EFFECTIVE_SPOOL_MODEL {
  PARAMETER throttle_frac, lim_vec.
  IF VTOL_ENG_LIST:LENGTH = 0 { RETURN. }
  IF lim_vec:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }
  IF IFC_ACTUAL_DT <= 0.0001 { RETURN. }

  LOCAL k_up IS _AMO_CFG_NUM("vtol_spool_k_up", VTOL_SPOOL_K_UP, 0.001).
  LOCAL k_dn IS _AMO_CFG_NUM("vtol_spool_k_dn", VTOL_SPOOL_K_DN, 0.001).
  LOCAL delta_min IS _AMO_CFG_NUM("vtol_spool_lag_delta_min", VTOL_SPOOL_LAG_DELTA_MIN, 0.0001).
  IF k_up < 0.001 { SET k_up TO 0.001. }
  IF k_dn < 0.001 { SET k_dn TO 0.001. }
  IF delta_min < 0.0001 { SET delta_min TO 0.0001. }

  IF VTOL_ENG_EFF_ACT_EST:LENGTH <> VTOL_ENG_LIST:LENGTH {
    VTOL_ENG_EFF_ACT_EST:CLEAR().
    // Reset prev-mean so the first differential pass starts from a clean slate.
    SET _vtol_prev_mean_cmd_eff TO -1.
    LOCAL init_i IS 0.
    UNTIL init_i >= VTOL_ENG_LIST:LENGTH {
      VTOL_ENG_EFF_ACT_EST:ADD(CLAMP(throttle_frac * lim_vec[init_i], 0, 1)).
      SET init_i TO init_i + 1.
    }
  }
  IF VTOL_ENG_LAG_EST_S:LENGTH <> VTOL_ENG_LIST:LENGTH {
    VTOL_ENG_LAG_EST_S:CLEAR().
    LOCAL lag_i_init IS 0.
    UNTIL lag_i_init >= VTOL_ENG_LIST:LENGTH {
      VTOL_ENG_LAG_EST_S:ADD(0.0).
      SET lag_i_init TO lag_i_init + 1.
    }
  }

  // Remove the uniform collective ramp from the lag estimate.
  // The slew-limited collective ramp is intentional: the engines follow it
  // without lag.  Only per-engine differential deviations (from unequal
  // limiter commands) represent true spool lag.  Pre-apply the mean cmd_eff
  // delta to all act_eff[i] so the tracker sees zero error for uniform
  // collective changes and only accumulates error for differential steps.
  LOCAL mean_cmd_eff IS 0.
  LOCAL mci IS 0.
  UNTIL mci >= VTOL_ENG_LIST:LENGTH {
    SET mean_cmd_eff TO mean_cmd_eff + CLAMP(throttle_frac * lim_vec[mci], 0, 1).
    SET mci TO mci + 1.
  }
  SET mean_cmd_eff TO mean_cmd_eff / VTOL_ENG_LIST:LENGTH.
  IF _vtol_prev_mean_cmd_eff >= 0 {
    LOCAL uniform_delta IS mean_cmd_eff - _vtol_prev_mean_cmd_eff.
    LOCAL udi IS 0.
    UNTIL udi >= VTOL_ENG_LIST:LENGTH {
      SET VTOL_ENG_EFF_ACT_EST[udi] TO CLAMP(VTOL_ENG_EFF_ACT_EST[udi] + uniform_delta, 0, 1).
      SET udi TO udi + 1.
    }
  }
  SET _vtol_prev_mean_cmd_eff TO mean_cmd_eff.

  LOCAL worst_lag IS MAX(0, TELEM_EM_WORST_SPOOL_LAG).
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL cmd_eff IS CLAMP(throttle_frac * lim_vec[i], 0, 1).
    LOCAL act_eff IS CLAMP(VTOL_ENG_EFF_ACT_EST[i], 0, 1).
    LOCAL err_eff IS cmd_eff - act_eff.

    LOCAL k_eff IS k_dn.
    IF err_eff > 0 { SET k_eff TO k_up. }

    SET act_eff TO CLAMP(act_eff + err_eff * k_eff * IFC_ACTUAL_DT, 0, 1).
    SET VTOL_ENG_EFF_ACT_EST[i] TO act_eff.

    LOCAL delta_eff IS ABS(cmd_eff - act_eff).
    LOCAL lag_eff_s IS 0.0.
    IF delta_eff > delta_min {
      LOCAL ratio IS 0.01 / delta_eff.
      IF ratio < 0.999999 {
        SET lag_eff_s TO -LN(ratio) / k_eff.
      }
    }
    IF lag_eff_s < 0 { SET lag_eff_s TO 0.0. }
    // Keep global engine-model lag as a lower bound when larger.
    IF TELEM_EM_WORST_SPOOL_LAG > lag_eff_s {
      SET lag_eff_s TO TELEM_EM_WORST_SPOOL_LAG.
    }
    SET VTOL_ENG_LAG_EST_S[i] TO lag_eff_s.
    IF lag_eff_s > worst_lag { SET worst_lag TO lag_eff_s. }
    SET i TO i + 1.
  }
  SET VTOL_EFF_LAG_WORST_S TO worst_lag.
}

FUNCTION _VTOL_AXIS_EFFECTIVE_SPOOL_LAG_S {
  PARAMETER axis_name.
  IF VTOL_ENG_LIST:LENGTH = 0 {
    RETURN MAX(MAX(0, TELEM_EM_WORST_SPOOL_LAG), VTOL_EFF_LAG_WORST_S).
  }
  LOCAL sum_w IS 0.
  LOCAL sum_lag_w IS 0.
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL mix_abs IS ABS(VTOL_ROLL_MIX[i]).
    IF axis_name = "pitch" {
      SET mix_abs TO ABS(VTOL_PITCH_MIX[i]).
    }
    IF mix_abs > 0.001 {
      LOCAL lag_i IS -1.
      IF VTOL_ENG_LAG_EST_S:LENGTH = VTOL_ENG_LIST:LENGTH {
        SET lag_i TO VTOL_ENG_LAG_EST_S[i].
      } ELSE {
        LOCAL eng_obj IS VTOL_ENG_LIST[i]["eng"].
        IF (DEFINED EM_GET_SPOOL_LAG_FOR_ENGINE) {
          SET lag_i TO EM_GET_SPOOL_LAG_FOR_ENGINE(eng_obj).
        }
        IF lag_i < 0 {
          SET lag_i TO TELEM_EM_WORST_SPOOL_LAG.
        }
      }
      IF lag_i < 0 { SET lag_i TO 0. }
      LOCAL lim_i IS VTOL_COLLECTIVE.
      IF VTOL_ENG_LIM_ACTUAL:LENGTH = VTOL_ENG_LIST:LENGTH {
        SET lim_i TO VTOL_ENG_LIM_ACTUAL[i].
      }
      LOCAL auth_i IS CLAMP(lim_i, 0.05, 1.0).
      LOCAL w_i IS mix_abs * auth_i.
      SET sum_w TO sum_w + w_i.
      SET sum_lag_w TO sum_lag_w + lag_i * w_i.
    }
    SET i TO i + 1.
  }
  IF sum_w <= 0 {
    RETURN MAX(MAX(0, TELEM_EM_WORST_SPOOL_LAG), VTOL_EFF_LAG_WORST_S).
  }
  RETURN sum_lag_w / sum_w.
}

FUNCTION _VTOL_UPDATE_RATE_FILTERS {
  PARAMETER ang_vel_vec, star_vec, fore_vec.
  LOCAL deg_per_rad IS 180 / CONSTANT:PI.
  LOCAL roll_rate_raw_deg IS -VDOT(ang_vel_vec, fore_vec) * deg_per_rad.
  LOCAL pitch_rate_raw_deg IS VDOT(ang_vel_vec, star_vec) * deg_per_rad.

  LOCAL rate_roll_alpha IS _AMO_CFG_NUM("vtol_rate_p_alpha", VTOL_RATE_P_ALPHA, 0.001).
  LOCAL rate_pitch_alpha IS _AMO_CFG_NUM("vtol_rate_q_alpha", VTOL_RATE_Q_ALPHA, 0.001).
  LOCAL accel_roll_alpha IS _AMO_CFG_NUM("vtol_rate_pdot_alpha", VTOL_RATE_PDOT_ALPHA, 0.001).
  LOCAL accel_pitch_alpha IS _AMO_CFG_NUM("vtol_rate_qdot_alpha", VTOL_RATE_QDOT_ALPHA, 0.001).
  LOCAL accel_clamp_deg_s2 IS _AMO_CFG_NUM(
    "vtol_rate_accel_clamp_degs2",
    VTOL_RATE_ACCEL_CLAMP_DEGS2,
    0.1
  ).
  SET rate_roll_alpha TO CLAMP(rate_roll_alpha, 0, 1).
  SET rate_pitch_alpha TO CLAMP(rate_pitch_alpha, 0, 1).
  SET accel_roll_alpha TO CLAMP(accel_roll_alpha, 0, 1).
  SET accel_pitch_alpha TO CLAMP(accel_pitch_alpha, 0, 1).
  IF accel_clamp_deg_s2 < 1.0 { SET accel_clamp_deg_s2 TO 1.0. }

  SET VTOL_RATE_P_FILT_PREV TO VTOL_RATE_P_FILT.
  SET VTOL_RATE_Q_FILT_PREV TO VTOL_RATE_Q_FILT.
  SET VTOL_RATE_P_FILT TO VTOL_RATE_P_FILT + (roll_rate_raw_deg - VTOL_RATE_P_FILT) * rate_roll_alpha.
  SET VTOL_RATE_Q_FILT TO VTOL_RATE_Q_FILT + (pitch_rate_raw_deg - VTOL_RATE_Q_FILT) * rate_pitch_alpha.

  LOCAL roll_accel_raw_deg_s2 IS 0.0.
  LOCAL pitch_accel_raw_deg_s2 IS 0.0.
  IF IFC_ACTUAL_DT > 0.001 {
    SET roll_accel_raw_deg_s2 TO (VTOL_RATE_P_FILT - VTOL_RATE_P_FILT_PREV) / IFC_ACTUAL_DT.
    SET pitch_accel_raw_deg_s2 TO (VTOL_RATE_Q_FILT - VTOL_RATE_Q_FILT_PREV) / IFC_ACTUAL_DT.
  }
  SET roll_accel_raw_deg_s2 TO CLAMP(roll_accel_raw_deg_s2, -accel_clamp_deg_s2, accel_clamp_deg_s2).
  SET pitch_accel_raw_deg_s2 TO CLAMP(pitch_accel_raw_deg_s2, -accel_clamp_deg_s2, accel_clamp_deg_s2).
  SET VTOL_RATE_P_DOT_FILT TO VTOL_RATE_P_DOT_FILT + (roll_accel_raw_deg_s2 - VTOL_RATE_P_DOT_FILT) * accel_roll_alpha.
  SET VTOL_RATE_Q_DOT_FILT TO VTOL_RATE_Q_DOT_FILT + (pitch_accel_raw_deg_s2 - VTOL_RATE_Q_DOT_FILT) * accel_pitch_alpha.

  SET VTOL_DIAG_P_DOT_FILT TO VTOL_RATE_P_DOT_FILT.
  SET VTOL_DIAG_Q_DOT_FILT TO VTOL_RATE_Q_DOT_FILT.
}

FUNCTION _VTOL_RESET_INERTIA_ESTIMATOR {
  LOCAL i_init IS _AMO_CFG_NUM("vtol_inertia_rls_i_init", VTOL_INERTIA_RLS_I_INIT, 0.0001).
  LOCAL p0 IS _AMO_CFG_NUM("vtol_inertia_rls_p0", VTOL_INERTIA_RLS_P0, 0.0001).
  IF i_init < 0.0001 { SET i_init TO 0.0001. }
  IF p0 < 0.0001 { SET p0 TO 0.0001. }
  SET VTOL_I_ROLL_EST TO i_init.
  SET VTOL_I_PITCH_EST TO i_init.
  SET VTOL_I_ROLL_COV TO p0.
  SET VTOL_I_PITCH_COV TO p0.
  SET VTOL_I_ROLL_VALID TO FALSE.
  SET VTOL_I_PITCH_VALID TO FALSE.
}

FUNCTION _VTOL_RLS_AXIS_UPDATE {
  PARAMETER i_est_in, p_cov_in, valid_in, tau_cmd_kNm, alpha_meas_rads2.
  PARAMETER lambda, process_q, alpha_min_rads2, tau_min_kNm, i_min, i_max.

  LOCAL i_out IS CLAMP(i_est_in, i_min, i_max).
  LOCAL p_out IS MAX(0.0001, p_cov_in).
  LOCAL valid_out IS valid_in.
  LOCAL updated_out IS FALSE.

  IF ABS(alpha_meas_rads2) >= alpha_min_rads2 {
    IF ABS(tau_cmd_kNm) >= tau_min_kNm {
      IF NOT valid_out {
        LOCAL i_seed IS ABS(tau_cmd_kNm / alpha_meas_rads2).
        SET i_out TO CLAMP(i_seed, i_min, i_max).
        SET valid_out TO TRUE.
      }
      LOCAL p_work IS p_out / lambda + process_q.
      IF p_work < 0.0001 { SET p_work TO 0.0001. }
      LOCAL denom IS 1.0 + p_work * alpha_meas_rads2 * alpha_meas_rads2.
      IF ABS(denom) < 0.000001 { SET denom TO 0.000001. }
      LOCAL gain IS p_work * alpha_meas_rads2 / denom.
      SET i_out TO CLAMP(
        i_out + gain * (tau_cmd_kNm - i_out * alpha_meas_rads2),
        i_min,
        i_max
      ).
      SET p_out TO (1.0 - gain * alpha_meas_rads2) * p_work.
      IF p_out < 0.0001 { SET p_out TO 0.0001. }
      SET updated_out TO TRUE.
    }
  }

  RETURN LEXICON(
    "i", i_out,
    "p", p_out,
    "valid", valid_out,
    "updated", updated_out
  ).
}

FUNCTION _VTOL_UPDATE_INERTIA_ESTIMATOR {
  PARAMETER roll_tau_cmd_kNm, pitch_tau_cmd_kNm.

  SET VTOL_DIAG_INERTIA_EST_ACTIVE TO FALSE.
  SET VTOL_DIAG_I_ROLL_TAU TO roll_tau_cmd_kNm.
  SET VTOL_DIAG_I_PITCH_TAU TO pitch_tau_cmd_kNm.

  LOCAL rad_per_deg IS CONSTANT:PI / 180.
  LOCAL roll_alpha_rads2 IS VTOL_RATE_P_DOT_FILT * rad_per_deg.
  LOCAL pitch_alpha_rads2 IS VTOL_RATE_Q_DOT_FILT * rad_per_deg.
  SET VTOL_DIAG_I_ROLL_ALPHA TO roll_alpha_rads2.
  SET VTOL_DIAG_I_PITCH_ALPHA TO pitch_alpha_rads2.
  SET VTOL_DIAG_I_ROLL_EST TO VTOL_I_ROLL_EST.
  SET VTOL_DIAG_I_PITCH_EST TO VTOL_I_PITCH_EST.

  LOCAL inertia_enabled IS _AMO_CFG_BOOL(
    "vtol_inertia_rls_enabled",
    VTOL_INERTIA_RLS_ENABLED_DEFAULT
  ).
  IF NOT inertia_enabled { RETURN. }
  IF IFC_ACTUAL_DT <= 0.001 { RETURN. }

  LOCAL max_ias_ms IS _AMO_CFG_NUM(
    "vtol_inertia_rls_max_ias",
    VTOL_INERTIA_RLS_MAX_IAS,
    0
  ).
  IF max_ias_ms > 0 {
    IF GET_IAS() > max_ias_ms { RETURN. }
  }

  LOCAL allow_ground IS _AMO_CFG_BOOL(
    "vtol_inertia_rls_allow_ground",
    VTOL_INERTIA_RLS_ALLOW_GROUND_DEFAULT
  ).
  IF NOT allow_ground {
    IF SHIP:STATUS = "LANDED" OR SHIP:STATUS = "PRELAUNCH" { RETURN. }
  }

  LOCAL lambda IS _AMO_CFG_NUM(
    "vtol_inertia_rls_forget_lambda",
    VTOL_INERTIA_RLS_FORGET_LAMBDA,
    0
  ).
  LOCAL process_q IS _AMO_CFG_NUM(
    "vtol_inertia_rls_process_q",
    VTOL_INERTIA_RLS_PROCESS_Q,
    0
  ).
  LOCAL alpha_min_rads2 IS _AMO_CFG_NUM(
    "vtol_inertia_rls_alpha_min_rads2",
    VTOL_INERTIA_RLS_ALPHA_MIN_RADS2,
    0
  ).
  LOCAL tau_min_kNm IS _AMO_CFG_NUM(
    "vtol_inertia_rls_tau_min_knm",
    VTOL_INERTIA_RLS_TAU_MIN_KNM,
    0
  ).
  LOCAL i_min IS _AMO_CFG_NUM(
    "vtol_inertia_rls_i_min",
    VTOL_INERTIA_RLS_I_MIN,
    0.0001
  ).
  LOCAL i_max IS _AMO_CFG_NUM(
    "vtol_inertia_rls_i_max",
    VTOL_INERTIA_RLS_I_MAX,
    0.01
  ).
  IF lambda < 0.90 { SET lambda TO 0.90. }
  IF lambda >= 1.0 { SET lambda TO 0.999999. }
  IF process_q < 0 { SET process_q TO 0. }
  IF alpha_min_rads2 < 0.001 { SET alpha_min_rads2 TO 0.001. }
  IF tau_min_kNm < 0.01 { SET tau_min_kNm TO 0.01. }
  IF i_min < 0.0001 { SET i_min TO 0.0001. }
  IF i_max < i_min + 0.001 { SET i_max TO i_min + 0.001. }

  LOCAL roll_out IS _VTOL_RLS_AXIS_UPDATE(
    VTOL_I_ROLL_EST,
    VTOL_I_ROLL_COV,
    VTOL_I_ROLL_VALID,
    roll_tau_cmd_kNm,
    roll_alpha_rads2,
    lambda,
    process_q,
    alpha_min_rads2,
    tau_min_kNm,
    i_min,
    i_max
  ).
  LOCAL pitch_out IS _VTOL_RLS_AXIS_UPDATE(
    VTOL_I_PITCH_EST,
    VTOL_I_PITCH_COV,
    VTOL_I_PITCH_VALID,
    pitch_tau_cmd_kNm,
    pitch_alpha_rads2,
    lambda,
    process_q,
    alpha_min_rads2,
    tau_min_kNm,
    i_min,
    i_max
  ).

  SET VTOL_I_ROLL_EST TO roll_out["i"].
  SET VTOL_I_ROLL_COV TO roll_out["p"].
  SET VTOL_I_ROLL_VALID TO roll_out["valid"].
  SET VTOL_I_PITCH_EST TO pitch_out["i"].
  SET VTOL_I_PITCH_COV TO pitch_out["p"].
  SET VTOL_I_PITCH_VALID TO pitch_out["valid"].

  SET VTOL_DIAG_I_ROLL_EST TO VTOL_I_ROLL_EST.
  SET VTOL_DIAG_I_PITCH_EST TO VTOL_I_PITCH_EST.
  SET VTOL_DIAG_INERTIA_EST_ACTIVE TO roll_out["updated"] OR pitch_out["updated"].
}

FUNCTION _VTOL_GET_SURFACE_VEL {
  LOCAL north_vec_use IS IFC_NORTH_VEC.
  IF VDOT(north_vec_use, north_vec_use) < 0.2 {
    SET north_vec_use TO SHIP:NORTH:VECTOR.
  }
  IF VDOT(north_vec_use, north_vec_use) < 0.2 {
    SET north_vec_use TO V(0, 0, 1).
  }
  SET north_vec_use TO north_vec_use:NORMALIZED.
  LOCAL up_vec_use IS SHIP:UP:VECTOR.
  LOCAL east_vec_use IS VCRS(up_vec_use, north_vec_use).
  IF VDOT(east_vec_use, east_vec_use) < 0.0001 {
    SET east_vec_use TO VCRS(up_vec_use, SHIP:FACING:FOREVECTOR).
  }
  IF VDOT(east_vec_use, east_vec_use) < 0.0001 {
    SET east_vec_use TO V(1, 0, 0).
  }
  SET east_vec_use TO east_vec_use:NORMALIZED.
  LOCAL surface_vel_vec IS SHIP:VELOCITY:SURFACE.
  LOCAL vel_north_ms IS VDOT(surface_vel_vec, north_vec_use).
  LOCAL vel_east_ms IS VDOT(surface_vel_vec, east_vec_use).
  RETURN LEXICON(
    "vn", vel_north_ms,
    "ve", vel_east_ms,
    "north", north_vec_use,
    "east", east_vec_use,
    "up", up_vec_use
  ).
}

FUNCTION _VTOL_CLAMP_NE_VECTOR {
  PARAMETER north_val, east_val, max_mag.
  LOCAL max_use IS max_mag.
  IF max_use < 0 { SET max_use TO 0. }

  LOCAL out_north IS north_val.
  LOCAL out_east IS east_val.
  LOCAL vec_mag IS SQRT(out_north * out_north + out_east * out_east).
  IF vec_mag > max_use AND vec_mag > 0.000001 {
    LOCAL vec_scale IS max_use / vec_mag.
    SET out_north TO out_north * vec_scale.
    SET out_east TO out_east * vec_scale.
  }
  RETURN LEXICON("north", out_north, "east", out_east).
}

FUNCTION _VTOL_POS_HOLD {
  IF NOT VTOL_POS_HOLD_ACTIVE { RETURN. }
  LOCAL pos_kp IS _AMO_CFG_NUM("vtol_pos_kp", VTOL_POS_KP, 0).
  LOCAL pos_ki IS _AMO_CFG_NUM("vtol_pos_ki", VTOL_POS_KI, 0).
  LOCAL pos_int_lim IS _AMO_CFG_NUM("vtol_pos_int_lim", VTOL_POS_INT_LIM, 0).
  LOCAL pos_int_radius_m IS _AMO_CFG_NUM("vtol_pos_int_radius", VTOL_POS_INT_RADIUS, 0.1).
  LOCAL pos_capture_radius_m IS _AMO_CFG_NUM("vtol_pos_capture_radius", VTOL_POS_CAPTURE_RADIUS, 0.1).
  LOCAL max_horiz_speed_ms IS _AMO_CFG_NUM("vtol_max_horiz_speed", VTOL_MAX_HORIZ_SPEED, 0.1).
  LOCAL em_lag_s IS _VTOL_FILTERED_EM_LAG_S().
  LOCAL gain_lag_ref_s IS _AMO_CFG_NUM("vtol_pos_gain_lag_ref_s", VTOL_POS_GAIN_LAG_REF_S, 0.01).
  LOCAL pos_kp_min_scale IS _AMO_CFG_NUM("vtol_pos_kp_min_scale", VTOL_POS_KP_MIN_SCALE, 0).
  LOCAL pos_ki_min_scale IS _AMO_CFG_NUM("vtol_pos_ki_min_scale", VTOL_POS_KI_MIN_SCALE, 0).
  LOCAL pos_speed_min_scale IS _AMO_CFG_NUM("vtol_pos_speed_min_scale", VTOL_POS_SPEED_MIN_SCALE, 0).
  LOCAL pos_capture_min_scale IS _AMO_CFG_NUM("vtol_pos_capture_min_scale", VTOL_POS_CAPTURE_MIN_SCALE, 0.05).
  LOCAL pos_aw_alpha_min IS _AMO_CFG_NUM("vtol_pos_aw_alpha_min", VTOL_POS_AW_ALPHA_MIN, 0).
  LOCAL outer_i_unwind_per_s IS _AMO_CFG_NUM(
    "vtol_outer_i_unwind_per_s",
    VTOL_OUTER_I_UNWIND_PER_S,
    0
  ).
  LOCAL pos_bw_ratio_max IS _AMO_CFG_NUM("vtol_pos_bw_ratio_max", VTOL_POS_BW_RATIO_MAX, 0).
  LOCAL pos_ki_bw_ratio_max IS _AMO_CFG_NUM("vtol_pos_ki_bw_ratio_max", VTOL_POS_KI_BW_RATIO_MAX, 0).
  LOCAL vel_kp_ref IS _AMO_CFG_NUM("vtol_vel_kp", VTOL_VEL_KP, 0).
  LOCAL vel_ki_ref IS _AMO_CFG_NUM("vtol_vel_ki", VTOL_VEL_KI, 0).
  IF pos_int_lim < 0 { SET pos_int_lim TO 0. }
  IF pos_int_radius_m < 0.1 { SET pos_int_radius_m TO 0.1. }
  IF pos_capture_radius_m < 0.1 { SET pos_capture_radius_m TO 0.1. }
  IF max_horiz_speed_ms < 0.1 { SET max_horiz_speed_ms TO 0.1. }
  IF em_lag_s < 0 { SET em_lag_s TO 0. }
  IF pos_kp_min_scale < 0 { SET pos_kp_min_scale TO 0. }
  IF pos_kp_min_scale > 1 { SET pos_kp_min_scale TO 1. }
  IF pos_ki_min_scale < 0 { SET pos_ki_min_scale TO 0. }
  IF pos_ki_min_scale > 1 { SET pos_ki_min_scale TO 1. }
  IF pos_speed_min_scale < 0 { SET pos_speed_min_scale TO 0. }
  IF pos_speed_min_scale > 1 { SET pos_speed_min_scale TO 1. }
  IF pos_capture_min_scale < 0.05 { SET pos_capture_min_scale TO 0.05. }
  IF pos_capture_min_scale > 1 { SET pos_capture_min_scale TO 1. }
  IF pos_aw_alpha_min < 0 { SET pos_aw_alpha_min TO 0. }
  IF pos_aw_alpha_min > 1 { SET pos_aw_alpha_min TO 1. }
  IF outer_i_unwind_per_s < 0 { SET outer_i_unwind_per_s TO 0. }
  IF pos_bw_ratio_max < 0 { SET pos_bw_ratio_max TO 0. }
  IF pos_ki_bw_ratio_max < 0 { SET pos_ki_bw_ratio_max TO 0. }

  LOCAL pos_lag_scale IS 1.0.
  IF gain_lag_ref_s > 0.01 AND em_lag_s > gain_lag_ref_s {
    SET pos_lag_scale TO gain_lag_ref_s / em_lag_s.
  }
  SET pos_lag_scale TO CLAMP(pos_lag_scale, 0, 1).
  LOCAL pos_kp_use IS pos_kp * CLAMP(pos_lag_scale, pos_kp_min_scale, 1.0).
  LOCAL pos_ki_use IS pos_ki * CLAMP(pos_lag_scale, pos_ki_min_scale, 1.0).
  IF pos_bw_ratio_max > 0 AND vel_kp_ref > 0 {
    LOCAL pos_kp_cap IS vel_kp_ref * pos_bw_ratio_max.
    IF pos_kp_cap > 0 AND pos_kp_use > pos_kp_cap { SET pos_kp_use TO pos_kp_cap. }
  }
  IF pos_ki_bw_ratio_max > 0 AND vel_ki_ref > 0 {
    LOCAL pos_ki_cap IS vel_ki_ref * pos_ki_bw_ratio_max.
    IF pos_ki_cap > 0 AND pos_ki_use > pos_ki_cap { SET pos_ki_use TO pos_ki_cap. }
  }
  LOCAL max_horiz_speed_use IS max_horiz_speed_ms * CLAMP(pos_lag_scale, pos_speed_min_scale, 1.0).
  IF max_horiz_speed_use < 0.1 { SET max_horiz_speed_use TO 0.1. }
  LOCAL pos_capture_radius_use IS pos_capture_radius_m / CLAMP(pos_lag_scale, pos_capture_min_scale, 1.0).
  IF pos_capture_radius_use < 0.1 { SET pos_capture_radius_use TO 0.1. }

  LOCAL target_geo IS LATLNG(VTOL_TARGET_LAT, VTOL_TARGET_LNG).
  LOCAL aircraft_geo IS SHIP:GEOPOSITION.
  LOCAL pos_err_dist_m IS GEO_DISTANCE(aircraft_geo, target_geo).
  LOCAL pos_err_bearing_deg IS GEO_BEARING(aircraft_geo, target_geo).
  LOCAL pos_err_north_m IS pos_err_dist_m * COS(pos_err_bearing_deg).
  LOCAL pos_err_east_m IS pos_err_dist_m * SIN(pos_err_bearing_deg).

  LOCAL pos_cmd_unsat_north IS pos_err_north_m * pos_kp_use + VTOL_POS_INT_N * pos_ki_use.
  LOCAL pos_cmd_unsat_east IS pos_err_east_m * pos_kp_use + VTOL_POS_INT_E * pos_ki_use.
  LOCAL pos_vec_clamped IS _VTOL_CLAMP_NE_VECTOR(
    pos_cmd_unsat_north,
    pos_cmd_unsat_east,
    max_horiz_speed_use
  ).
  LOCAL pos_clamped_north IS pos_vec_clamped["north"].
  LOCAL pos_clamped_east IS pos_vec_clamped["east"].
  LOCAL pos_saturated IS
    ABS(pos_cmd_unsat_north - pos_clamped_north) > 0.000001 OR
    ABS(pos_cmd_unsat_east - pos_clamped_east) > 0.000001.
  LOCAL alloc_limited_prev IS VTOL_ALLOC_ALPHA < pos_aw_alpha_min OR VTOL_DIAG_ALPHA_LIMITED.
  IF pos_err_dist_m <= pos_int_radius_m {
    IF alloc_limited_prev OR pos_saturated {
      LOCAL pos_unwind_fac IS CLAMP(1.0 - outer_i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
      SET VTOL_POS_INT_N TO VTOL_POS_INT_N * pos_unwind_fac.
      SET VTOL_POS_INT_E TO VTOL_POS_INT_E * pos_unwind_fac.
    } ELSE {
      SET VTOL_POS_INT_N TO CLAMP(VTOL_POS_INT_N + pos_err_north_m * IFC_ACTUAL_DT, -pos_int_lim, pos_int_lim).
      SET VTOL_POS_INT_E TO CLAMP(VTOL_POS_INT_E + pos_err_east_m * IFC_ACTUAL_DT, -pos_int_lim, pos_int_lim).
    }
  } ELSE IF alloc_limited_prev {
    LOCAL pos_unwind_fac_far IS CLAMP(1.0 - outer_i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
    SET VTOL_POS_INT_N TO VTOL_POS_INT_N * pos_unwind_fac_far.
    SET VTOL_POS_INT_E TO VTOL_POS_INT_E * pos_unwind_fac_far.
  }

  LOCAL pos_vel_cmd_north_ms IS pos_err_north_m * pos_kp_use + VTOL_POS_INT_N * pos_ki_use.
  LOCAL pos_vel_cmd_east_ms IS pos_err_east_m * pos_kp_use + VTOL_POS_INT_E * pos_ki_use.
  LOCAL capture_scale IS CLAMP(pos_err_dist_m / pos_capture_radius_use, 0, 1).
  SET pos_vel_cmd_north_ms TO pos_vel_cmd_north_ms * capture_scale.
  SET pos_vel_cmd_east_ms TO pos_vel_cmd_east_ms * capture_scale.
  LOCAL pos_vel_clamped IS _VTOL_CLAMP_NE_VECTOR(
    pos_vel_cmd_north_ms,
    pos_vel_cmd_east_ms,
    max_horiz_speed_use
  ).
  SET pos_vel_cmd_north_ms TO pos_vel_clamped["north"].
  SET pos_vel_cmd_east_ms TO pos_vel_clamped["east"].

  SET VTOL_VN_CMD TO pos_vel_cmd_north_ms.
  SET VTOL_VE_CMD TO pos_vel_cmd_east_ms.
}

FUNCTION _VTOL_VEL_HOLD {
  PARAMETER pilot_pitch_in, pilot_roll_in.
  LOCAL max_fwd_pitch_deg IS _AMO_CFG_NUM("vtol_max_fwd_pitch", VTOL_MAX_FWD_PITCH, 0.1).
  LOCAL max_bank_deg IS _AMO_CFG_NUM("vtol_max_bank", VTOL_MAX_BANK, 0.1).
  LOCAL max_horiz_speed_ms IS _AMO_CFG_NUM("vtol_max_horiz_speed", VTOL_MAX_HORIZ_SPEED, 0.1).
  LOCAL max_horiz_accel_ms2 IS _AMO_CFG_NUM("vtol_max_horiz_accel", VTOL_MAX_HORIZ_ACCEL, 0.1).
  LOCAL vel_kp IS _AMO_CFG_NUM("vtol_vel_kp", VTOL_VEL_KP, 0).
  LOCAL vel_ki IS _AMO_CFG_NUM("vtol_vel_ki", VTOL_VEL_KI, 0).
  LOCAL vel_int_lim IS _AMO_CFG_NUM("vtol_vel_int_lim", VTOL_VEL_INT_LIM, 0).
  LOCAL vel_deadband_ms IS _AMO_CFG_NUM("vtol_vel_int_deadband", VTOL_VEL_INT_DEADBAND, 0).
  LOCAL khv_capture_ms IS _AMO_CFG_NUM("vtol_khv_capture_mps", VTOL_KHV_CAPTURE_MPS, 0.05).
  LOCAL em_lag_s IS _VTOL_FILTERED_EM_LAG_S().
  LOCAL gain_lag_ref_s IS _AMO_CFG_NUM("vtol_vel_gain_lag_ref_s", VTOL_VEL_GAIN_LAG_REF_S, 0.01).
  LOCAL vel_kp_min_scale IS _AMO_CFG_NUM("vtol_vel_kp_min_scale", VTOL_VEL_KP_MIN_SCALE, 0).
  LOCAL vel_ki_min_scale IS _AMO_CFG_NUM("vtol_vel_ki_min_scale", VTOL_VEL_KI_MIN_SCALE, 0).
  LOCAL vel_accel_min_scale IS _AMO_CFG_NUM("vtol_vel_accel_min_scale", VTOL_VEL_ACCEL_MIN_SCALE, 0).
  LOCAL vel_angle_min_scale IS _AMO_CFG_NUM("vtol_vel_angle_min_scale", VTOL_VEL_ANGLE_MIN_SCALE, 0).
  LOCAL vel_aw_alpha_min IS _AMO_CFG_NUM("vtol_vel_aw_alpha_min", VTOL_VEL_AW_ALPHA_MIN, 0).
  LOCAL outer_i_unwind_per_s IS _AMO_CFG_NUM(
    "vtol_outer_i_unwind_per_s",
    VTOL_OUTER_I_UNWIND_PER_S,
    0
  ).
  LOCAL g_eff_min_scale IS _AMO_CFG_NUM(
    "vtol_vel_g_eff_min_scale",
    VTOL_VEL_G_EFF_MIN_SCALE,
    0
  ).
  LOCAL g_eff_max_scale IS _AMO_CFG_NUM(
    "vtol_vel_g_eff_max_scale",
    VTOL_VEL_G_EFF_MAX_SCALE,
    0.01
  ).
  LOCAL g_eff_min_hover_blend IS _AMO_CFG_NUM(
    "vtol_vel_g_eff_min_hover_blend",
    VTOL_VEL_G_EFF_MIN_HOVER_BLEND,
    0.01
  ).
  IF max_fwd_pitch_deg < 0.1 { SET max_fwd_pitch_deg TO 0.1. }
  IF max_bank_deg < 0.1 { SET max_bank_deg TO 0.1. }
  IF max_horiz_speed_ms < 0.1 { SET max_horiz_speed_ms TO 0.1. }
  IF max_horiz_accel_ms2 < 0.1 { SET max_horiz_accel_ms2 TO 0.1. }
  IF vel_int_lim < 0 { SET vel_int_lim TO 0. }
  IF vel_deadband_ms < 0 { SET vel_deadband_ms TO 0. }
  IF khv_capture_ms < 0.05 { SET khv_capture_ms TO 0.05. }
  IF em_lag_s < 0 { SET em_lag_s TO 0. }
  IF vel_kp_min_scale < 0 { SET vel_kp_min_scale TO 0. }
  IF vel_kp_min_scale > 1 { SET vel_kp_min_scale TO 1. }
  IF vel_ki_min_scale < 0 { SET vel_ki_min_scale TO 0. }
  IF vel_ki_min_scale > 1 { SET vel_ki_min_scale TO 1. }
  IF vel_accel_min_scale < 0 { SET vel_accel_min_scale TO 0. }
  IF vel_accel_min_scale > 1 { SET vel_accel_min_scale TO 1. }
  IF vel_angle_min_scale < 0 { SET vel_angle_min_scale TO 0. }
  IF vel_angle_min_scale > 1 { SET vel_angle_min_scale TO 1. }
  IF vel_aw_alpha_min < 0 { SET vel_aw_alpha_min TO 0. }
  IF vel_aw_alpha_min > 1 { SET vel_aw_alpha_min TO 1. }
  IF outer_i_unwind_per_s < 0 { SET outer_i_unwind_per_s TO 0. }
  IF g_eff_min_scale < 0.1 { SET g_eff_min_scale TO 0.1. }
  IF g_eff_max_scale < g_eff_min_scale { SET g_eff_max_scale TO g_eff_min_scale. }
  IF g_eff_min_hover_blend < 0.05 { SET g_eff_min_hover_blend TO 0.05. }
  IF g_eff_min_hover_blend > 1.0 { SET g_eff_min_hover_blend TO 1.0. }

  LOCAL vel_lag_scale IS 1.0.
  IF gain_lag_ref_s > 0.01 AND em_lag_s > gain_lag_ref_s {
    SET vel_lag_scale TO gain_lag_ref_s / em_lag_s.
  }
  SET vel_lag_scale TO CLAMP(vel_lag_scale, 0, 1).
  LOCAL vel_kp_use IS vel_kp * CLAMP(vel_lag_scale, vel_kp_min_scale, 1.0).
  LOCAL vel_ki_use IS vel_ki * CLAMP(vel_lag_scale, vel_ki_min_scale, 1.0).
  LOCAL max_horiz_accel_use IS max_horiz_accel_ms2 * CLAMP(vel_lag_scale, vel_accel_min_scale, 1.0).
  LOCAL max_fwd_pitch_use IS max_fwd_pitch_deg * CLAMP(vel_lag_scale, vel_angle_min_scale, 1.0).
  LOCAL max_bank_use IS max_bank_deg * CLAMP(vel_lag_scale, vel_angle_min_scale, 1.0).
  IF max_horiz_accel_use < 0.1 { SET max_horiz_accel_use TO 0.1. }
  IF max_fwd_pitch_use < 0.1 { SET max_fwd_pitch_use TO 0.1. }
  IF max_bank_use < 0.1 { SET max_bank_use TO 0.1. }

  LOCAL vel_meas IS _VTOL_GET_SURFACE_VEL().
  SET VTOL_VN_ACTUAL TO vel_meas["vn"].
  SET VTOL_VE_ACTUAL TO vel_meas["ve"].

  IF NOT VTOL_VEL_HOLD_ACTIVE {
    LOCAL pilot_vel_cmd IS _VTOL_CLAMP_NE_VECTOR(
      -pilot_pitch_in * max_horiz_speed_ms,
      pilot_roll_in * max_horiz_speed_ms,
      max_horiz_speed_ms
    ).
    SET VTOL_VN_CMD TO pilot_vel_cmd["north"].
    SET VTOL_VE_CMD TO pilot_vel_cmd["east"].
    SET VTOL_PHI_CMD TO CLAMP(pilot_roll_in * max_bank_deg, -max_bank_deg, max_bank_deg).
    SET VTOL_THETA_CMD TO CLAMP(-pilot_pitch_in * max_fwd_pitch_deg, -max_fwd_pitch_deg, max_fwd_pitch_deg).
    SET VTOL_VEL_INT_N TO 0.0.
    SET VTOL_VEL_INT_E TO 0.0.
    RETURN.
  }

  IF VTOL_KHV_ACTIVE {
    SET VTOL_VN_CMD TO 0.0.
    SET VTOL_VE_CMD TO 0.0.
  } ELSE IF NOT VTOL_POS_HOLD_ACTIVE {
    LOCAL pilot_hold_cmd IS _VTOL_CLAMP_NE_VECTOR(
      -pilot_pitch_in * max_horiz_speed_ms,
      pilot_roll_in * max_horiz_speed_ms,
      max_horiz_speed_ms
    ).
    SET VTOL_VN_CMD TO pilot_hold_cmd["north"].
    SET VTOL_VE_CMD TO pilot_hold_cmd["east"].
  }
  LOCAL vel_cmd_clamped IS _VTOL_CLAMP_NE_VECTOR(VTOL_VN_CMD, VTOL_VE_CMD, max_horiz_speed_ms).
  SET VTOL_VN_CMD TO vel_cmd_clamped["north"].
  SET VTOL_VE_CMD TO vel_cmd_clamped["east"].

  LOCAL vel_err_north_ms IS VTOL_VN_CMD - VTOL_VN_ACTUAL.
  LOCAL vel_err_east_ms IS VTOL_VE_CMD - VTOL_VE_ACTUAL.
  LOCAL vel_cmd_unsat_north IS vel_err_north_ms * vel_kp_use + VTOL_VEL_INT_N * vel_ki_use.
  LOCAL vel_cmd_unsat_east IS vel_err_east_ms * vel_kp_use + VTOL_VEL_INT_E * vel_ki_use.
  LOCAL vel_vec_clamped IS _VTOL_CLAMP_NE_VECTOR(
    vel_cmd_unsat_north,
    vel_cmd_unsat_east,
    max_horiz_accel_use
  ).
  LOCAL vel_clamped_north IS vel_vec_clamped["north"].
  LOCAL vel_clamped_east IS vel_vec_clamped["east"].
  LOCAL vel_saturated IS
    ABS(vel_cmd_unsat_north - vel_clamped_north) > 0.000001 OR
    ABS(vel_cmd_unsat_east - vel_clamped_east) > 0.000001.
  LOCAL alloc_limited_prev IS VTOL_ALLOC_ALPHA < vel_aw_alpha_min OR VTOL_DIAG_ALPHA_LIMITED.
  // Ground-contact guard: clear velocity integrators while landed/pre-launch,
  // matching the same protection the level-attitude integrators already have.
  // Without this, pre-liftoff ground-roll winds up VEL_INT and causes pitch
  // divergence immediately after gear breaks contact.
  LOCAL vel_status_str IS SHIP:STATUS.
  LOCAL vel_truly_airborne IS (vel_status_str <> "LANDED" AND vel_status_str <> "PRELAUNCH").
  IF NOT vel_truly_airborne {
    SET VTOL_VEL_INT_N TO 0.0.
    SET VTOL_VEL_INT_E TO 0.0.
  } ELSE IF alloc_limited_prev OR vel_saturated {
    LOCAL vel_unwind_fac IS CLAMP(1.0 - outer_i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
    SET VTOL_VEL_INT_N TO VTOL_VEL_INT_N * vel_unwind_fac.
    SET VTOL_VEL_INT_E TO VTOL_VEL_INT_E * vel_unwind_fac.
  } ELSE {
    IF ABS(vel_err_north_ms) > vel_deadband_ms {
      SET VTOL_VEL_INT_N TO CLAMP(VTOL_VEL_INT_N + vel_err_north_ms * IFC_ACTUAL_DT, -vel_int_lim, vel_int_lim).
    }
    IF ABS(vel_err_east_ms) > vel_deadband_ms {
      SET VTOL_VEL_INT_E TO CLAMP(VTOL_VEL_INT_E + vel_err_east_ms * IFC_ACTUAL_DT, -vel_int_lim, vel_int_lim).
    }
  }

  LOCAL accel_cmd_north_ms2 IS vel_err_north_ms * vel_kp_use + VTOL_VEL_INT_N * vel_ki_use.
  LOCAL accel_cmd_east_ms2 IS vel_err_east_ms * vel_kp_use + VTOL_VEL_INT_E * vel_ki_use.
  LOCAL accel_cmd_clamped IS _VTOL_CLAMP_NE_VECTOR(
    accel_cmd_north_ms2,
    accel_cmd_east_ms2,
    max_horiz_accel_use
  ).
  SET accel_cmd_north_ms2 TO accel_cmd_clamped["north"].
  SET accel_cmd_east_ms2 TO accel_cmd_clamped["east"].
  // Accel command is in world N/E; project it onto body-forward/starboard so
  // velocity hold remains correct when heading drifts away from North.
  LOCAL north_vec_use IS vel_meas["north"].
  LOCAL east_vec_use IS vel_meas["east"].
  LOCAL up_vec_use IS vel_meas["up"].
  LOCAL fore_vec_use IS SHIP:FACING:FOREVECTOR.
  LOCAL star_vec_use IS SHIP:FACING:STARVECTOR.
  LOCAL fore_h_vec IS fore_vec_use - up_vec_use * VDOT(fore_vec_use, up_vec_use).
  IF VDOT(fore_h_vec, fore_h_vec) < 0.0001 {
    SET fore_h_vec TO fore_vec_use.
  }
  IF VDOT(fore_h_vec, fore_h_vec) < 0.0001 {
    SET fore_h_vec TO east_vec_use.
  }
  SET fore_h_vec TO fore_h_vec:NORMALIZED.
  LOCAL star_h_vec IS star_vec_use - up_vec_use * VDOT(star_vec_use, up_vec_use).
  IF VDOT(star_h_vec, star_h_vec) < 0.0001 {
    SET star_h_vec TO star_vec_use.
  }
  IF VDOT(star_h_vec, star_h_vec) < 0.0001 {
    SET star_h_vec TO north_vec_use.
  }
  SET star_h_vec TO star_h_vec:NORMALIZED.
  LOCAL accel_cmd_fwd_ms2 IS
    accel_cmd_north_ms2 * VDOT(north_vec_use, fore_h_vec) +
    accel_cmd_east_ms2 * VDOT(east_vec_use, fore_h_vec).
  LOCAL accel_cmd_star_ms2 IS
    accel_cmd_north_ms2 * VDOT(north_vec_use, star_h_vec) +
    accel_cmd_east_ms2 * VDOT(east_vec_use, star_h_vec).
  LOCAL hover_collective_ref IS VTOL_HOVER_COLLECTIVE.
  IF hover_collective_ref < 0.05 { SET hover_collective_ref TO 0.05. }
  LOCAL g_eff_collective_scale IS VTOL_COLLECTIVE_EFF_EST / hover_collective_ref.
  SET g_eff_collective_scale TO CLAMP(g_eff_collective_scale, g_eff_min_scale, g_eff_max_scale).
  LOCAL g_eff_hover_blend_scale IS CLAMP(VTOL_HOVER_BLEND, g_eff_min_hover_blend, 1.0).
  LOCAL vertical_denom_ms2 IS 9.80665 * g_eff_collective_scale * g_eff_hover_blend_scale.
  IF vertical_denom_ms2 < 1.5 { SET vertical_denom_ms2 TO 1.5. }
  LOCAL pitch_target_deg IS -ARCTAN(accel_cmd_fwd_ms2 / vertical_denom_ms2).
  LOCAL bank_target_deg IS ARCTAN(accel_cmd_star_ms2 / vertical_denom_ms2).

  SET VTOL_THETA_CMD TO CLAMP(pitch_target_deg, -max_fwd_pitch_use, max_fwd_pitch_use).
  SET VTOL_PHI_CMD TO CLAMP(bank_target_deg, -max_bank_use, max_bank_use).

  IF VTOL_KHV_ACTIVE {
    // Only auto-clear KHV when a higher-level position objective is active.
    // This prevents entering a dead state where both KHV and POS hold are off.
    IF VTOL_POS_HOLD_ACTIVE AND
       ABS(VTOL_VN_ACTUAL) <= khv_capture_ms AND
       ABS(VTOL_VE_ACTUAL) <= khv_capture_ms {
      SET VTOL_KHV_ACTIVE TO FALSE.
    }
  }
}

FUNCTION _VTOL_UPDATE_NACELLE_SCHEDULE {
  LOCAL hover_angle_deg IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  LOCAL cruise_angle_deg IS _AMO_CFG_NUM("vtol_cruise_angle", 0, 0).
  LOCAL trans_start_ias_ms IS _AMO_CFG_NUM("vtol_trans_start_ias", VTOL_TRANS_START_IAS, 0.1).
  LOCAL trans_end_ias_ms IS _AMO_CFG_NUM("vtol_trans_end_ias", VTOL_TRANS_END_IAS, 0.2).
  LOCAL nacelle_slew_dps IS _AMO_CFG_NUM("vtol_nacelle_slew_dps", VTOL_NACELLE_SLEW_DPS, 0.1).
  LOCAL nacelle_alpha_min_deg IS _AMO_CFG_NUM("vtol_nacelle_alpha_min", VTOL_NACELLE_ALPHA_MIN, 0).
  IF trans_end_ias_ms <= trans_start_ias_ms { SET trans_end_ias_ms TO trans_start_ias_ms + 0.1. }
  IF nacelle_slew_dps < 0.1 { SET nacelle_slew_dps TO 0.1. }
  IF nacelle_alpha_min_deg < 0 { SET nacelle_alpha_min_deg TO 0. }

  IF VTOL_NACELLE_ALPHA_CMD < 0.1 OR VTOL_NACELLE_ALPHA_CMD > 179.9 {
    SET VTOL_NACELLE_ALPHA_CMD TO hover_angle_deg.
  }

  LOCAL target_alpha_deg IS VTOL_NACELLE_ALPHA_CMD.
  IF VTOL_TRANS_ACTIVE {
    LOCAL ias_now_ms IS GET_IAS().
    IF ias_now_ms <= trans_start_ias_ms {
      SET target_alpha_deg TO hover_angle_deg.
    } ELSE IF ias_now_ms >= trans_end_ias_ms {
      SET target_alpha_deg TO cruise_angle_deg.
    } ELSE {
      LOCAL trans_frac IS (ias_now_ms - trans_start_ias_ms) / (trans_end_ias_ms - trans_start_ias_ms).
      SET target_alpha_deg TO hover_angle_deg + (cruise_angle_deg - hover_angle_deg) * trans_frac.
    }
    IF target_alpha_deg < nacelle_alpha_min_deg { SET target_alpha_deg TO nacelle_alpha_min_deg. }
  }

  LOCAL alpha_step_deg IS nacelle_slew_dps * IFC_ACTUAL_DT.
  SET VTOL_NACELLE_ALPHA_CMD TO CLAMP(
    target_alpha_deg,
    VTOL_NACELLE_ALPHA_CMD - alpha_step_deg,
    VTOL_NACELLE_ALPHA_CMD + alpha_step_deg
  ).

  LOCAL estimate_alpha_deg IS VTOL_NACELLE_ALPHA_CMD.
  LOCAL estimate_lift_frac IS CLAMP(SIN(CLAMP(estimate_alpha_deg, 0, 180)), 0, 1).
  IF VTOL_SRV_AVAIL {
    LOCAL sum_alpha_deg IS 0.0.
    LOCAL sum_lift_frac IS 0.0.
    LOCAL sum_count IS 0.0.
    LOCAL idx IS 0.
    UNTIL idx >= VTOL_SRV_LIST:LENGTH {
      LOCAL srv_entry IS VTOL_SRV_LIST[idx].
      IF srv_entry <> 0 {
        LOCAL alpha_i_deg IS _VTOL_SERVO_CURRENT_POS(srv_entry, estimate_alpha_deg).
        SET sum_alpha_deg TO sum_alpha_deg + alpha_i_deg.
        SET sum_lift_frac TO sum_lift_frac + CLAMP(SIN(alpha_i_deg), 0, 1).
        SET sum_count TO sum_count + 1.0.
      }
      SET idx TO idx + 1.
    }
    IF sum_count > 0 {
      SET estimate_alpha_deg TO sum_alpha_deg / sum_count.
      SET estimate_lift_frac TO sum_lift_frac / sum_count.
    }
  }
  SET VTOL_NACELLE_ALPHA_EST TO estimate_alpha_deg.
  SET VTOL_HOVER_BLEND TO CLAMP(estimate_lift_frac, 0, 1).
}

FUNCTION _VTOL_VS_HOLD {
  PARAMETER thr_input.
  LOCAL vs_kp  IS _AMO_CFG_NUM("vtol_vs_kp",  VTOL_VS_KP,  0).
  LOCAL vs_ki  IS _AMO_CFG_NUM("vtol_vs_ki",  VTOL_VS_KI,  0).
  LOCAL max_vs IS _AMO_CFG_NUM("vtol_max_vs", VTOL_MAX_VS, 0.1).
  LOCAL coll_max IS _AMO_CFG_NUM("vtol_collective_max", VTOL_COLLECTIVE_MAX, 0.1).
  LOCAL em_lag_s IS _VTOL_FILTERED_EM_LAG_S().
  IF coll_max > 1.0 { SET coll_max TO 1.0. }
  IF coll_max < 0.0 { SET coll_max TO 0.0. }
  IF em_lag_s < 0 { SET em_lag_s TO 0. }

  // Lag-aware PI gain scheduling.
  LOCAL gain_lag_ref_s IS _AMO_CFG_NUM("vtol_vs_gain_lag_ref_s", VTOL_VS_GAIN_LAG_REF_S, 0.01).
  LOCAL kp_min_scale IS _AMO_CFG_NUM("vtol_vs_kp_min_scale", VTOL_VS_KP_MIN_SCALE, 0).
  LOCAL ki_min_scale IS _AMO_CFG_NUM("vtol_vs_ki_min_scale", VTOL_VS_KI_MIN_SCALE, 0).
  IF kp_min_scale < 0 { SET kp_min_scale TO 0. }
  IF kp_min_scale > 1 { SET kp_min_scale TO 1. }
  IF ki_min_scale < 0 { SET ki_min_scale TO 0. }
  IF ki_min_scale > 1 { SET ki_min_scale TO 1. }
  LOCAL lag_gain_scale IS 1.0.
  IF gain_lag_ref_s > 0.01 AND em_lag_s > gain_lag_ref_s {
    SET lag_gain_scale TO gain_lag_ref_s / em_lag_s.
  }
  SET lag_gain_scale TO CLAMP(lag_gain_scale, 0, 1).
  LOCAL vs_kp_use IS vs_kp * CLAMP(lag_gain_scale, kp_min_scale, 1.0).
  LOCAL vs_ki_use IS vs_ki * CLAMP(lag_gain_scale, ki_min_scale, 1.0).

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

  LOCAL vs_cmd_target IS (thr_input - 0.5) * 2.0 * max_vs.

  IF VTOL_ALT_HOLD {
    LOCAL alt_kp  IS _AMO_CFG_NUM("vtol_alt_kp", VTOL_ALT_KP, 0).
    LOCAL alt_err IS VTOL_ALT_CMD - GET_AGL().
    SET vs_cmd_target TO CLAMP(alt_err * alt_kp, -max_vs, max_vs).
  } ELSE {
    SET vs_cmd_target TO CLAMP(vs_cmd_target, -max_vs, max_vs).
  }

  // Rate-limit VS setpoint so long spool lag does not induce "instant"
  // commands the actuator cannot follow.
  LOCAL cmd_up_slew IS _AMO_CFG_NUM("vtol_vs_cmd_up_slew_mps2", VTOL_VS_CMD_UP_SLEW_MPS2, 0).
  LOCAL cmd_dn_slew IS _AMO_CFG_NUM("vtol_vs_cmd_dn_slew_mps2", VTOL_VS_CMD_DN_SLEW_MPS2, 0).
  LOCAL cmd_lag_ref_s IS _AMO_CFG_NUM("vtol_vs_cmd_lag_ref_s", VTOL_VS_CMD_LAG_REF_S, 0.01).
  LOCAL cmd_slew_min_scale IS _AMO_CFG_NUM("vtol_vs_cmd_slew_min_scale", VTOL_VS_CMD_SLEW_MIN_SCALE, 0).
  IF cmd_up_slew < 0 { SET cmd_up_slew TO 0. }
  IF cmd_dn_slew < 0 { SET cmd_dn_slew TO 0. }
  IF cmd_slew_min_scale < 0 { SET cmd_slew_min_scale TO 0. }
  IF cmd_slew_min_scale > 1 { SET cmd_slew_min_scale TO 1. }
  LOCAL cmd_scale IS 1.0.
  IF cmd_lag_ref_s > 0.01 AND em_lag_s > cmd_lag_ref_s {
    SET cmd_scale TO cmd_lag_ref_s / em_lag_s.
  }
  SET cmd_scale TO CLAMP(cmd_scale, cmd_slew_min_scale, 1.0).
  SET cmd_up_slew TO cmd_up_slew * cmd_scale.
  SET cmd_dn_slew TO cmd_dn_slew * cmd_scale.
  LOCAL cmd_slew_use IS cmd_dn_slew.
  IF vs_cmd_target > VTOL_VS_CMD { SET cmd_slew_use TO cmd_up_slew. }
  IF cmd_slew_use <= 0 {
    SET VTOL_VS_CMD TO vs_cmd_target.
  } ELSE {
    LOCAL cmd_step IS cmd_slew_use * IFC_ACTUAL_DT.
    SET VTOL_VS_CMD TO CLAMP(vs_cmd_target, VTOL_VS_CMD - cmd_step, VTOL_VS_CMD + cmd_step).
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
    LOCAL target_g IS CLAMP(VTOL_HOVER_COLLECTIVE + vs_err_g * vs_kp_use, 0, coll_max).
    LOCAL collective_g IS _VTOL_SLEW_COLLECTIVE(target_g).
    SET VTOL_COLLECTIVE_EFF_EST TO collective_g.
    RETURN collective_g.
  }

  LOCAL vs_err IS VTOL_VS_CMD - SHIP:VERTICALSPEED.

  // Anti-windup: only accumulate the integral when the unclamped output
  // is not already saturated in the same direction as the error.
  // This prevents the integrator from driving collective to its limit
  // during initial engine spool-up (aircraft on ground, can't achieve
  // commanded VS) and keeping it there after liftoff.
  LOCAL unclamped_pre IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp_use + VTOL_VS_INTEGRAL * vs_ki_use.
  LOCAL at_ceil IS unclamped_pre >= coll_max AND vs_err > 0.
  LOCAL at_floor IS unclamped_pre <= 0.0 AND vs_err < 0.
  LOCAL aw_alpha_min IS _AMO_CFG_NUM("vtol_vs_aw_alpha_min", VTOL_VS_AW_ALPHA_MIN, 0).
  LOCAL aw_lag_s IS _AMO_CFG_NUM("vtol_vs_aw_lag_s", VTOL_VS_AW_LAG_S, 0).
  LOCAL aw_eff_err_min IS _AMO_CFG_NUM("vtol_vs_aw_eff_err_min", VTOL_VS_AW_EFF_ERR_MIN, 0).
  LOCAL i_unwind_per_s IS _AMO_CFG_NUM("vtol_vs_i_unwind_per_s", VTOL_VS_I_UNWIND_PER_S, 0).
  IF aw_alpha_min < 0 { SET aw_alpha_min TO 0. }
  IF aw_alpha_min > 1 { SET aw_alpha_min TO 1. }
  IF aw_lag_s < 0 { SET aw_lag_s TO 0. }
  IF aw_eff_err_min < 0 { SET aw_eff_err_min TO 0. }
  IF i_unwind_per_s < 0 { SET i_unwind_per_s TO 0. }
  LOCAL alloc_limited_prev IS VTOL_ALLOC_ALPHA < aw_alpha_min.
  LOCAL eff_collective_err IS ABS(VTOL_COLLECTIVE - VTOL_COLLECTIVE_EFF_EST).
  LOCAL lag_limited_prev IS em_lag_s > aw_lag_s AND eff_collective_err > aw_eff_err_min.
  IF at_ceil OR at_floor OR alloc_limited_prev OR lag_limited_prev {
    LOCAL unwind_fac IS CLAMP(1.0 - i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
    SET VTOL_VS_INTEGRAL TO VTOL_VS_INTEGRAL * unwind_fac.
  } ELSE {
    SET VTOL_VS_INTEGRAL TO CLAMP(
      VTOL_VS_INTEGRAL + vs_err * IFC_ACTUAL_DT,
      -VTOL_VS_INTEGRAL_LIM,
       VTOL_VS_INTEGRAL_LIM
    ).
  }

  LOCAL unclamped IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp_use + VTOL_VS_INTEGRAL * vs_ki_use.
  LOCAL target_collective IS CLAMP(unclamped, 0, coll_max).
  SET VTOL_DIAG_COLL_BEFORE_CORR TO target_collective.

  LOCAL cos_att_alpha IS _AMO_CFG_NUM("vtol_cos_att_alpha", VTOL_COS_ATT_ALPHA, 0).
  LOCAL cos_att_floor IS _AMO_CFG_NUM("vtol_cos_att_floor", VTOL_COS_ATT_FLOOR, 0.01).
  SET cos_att_alpha TO CLAMP(cos_att_alpha, 0, 1).
  SET cos_att_floor TO CLAMP(cos_att_floor, 0.05, 1.0).
  LOCAL cos_att_raw IS VDOT(SHIP:FACING:TOPVECTOR, SHIP:UP:VECTOR).
  SET cos_att_raw TO CLAMP(cos_att_raw, cos_att_floor, 1.0).
  SET VTOL_COS_ATT_FILT TO VTOL_COS_ATT_FILT + (cos_att_raw - VTOL_COS_ATT_FILT) * cos_att_alpha.
  LOCAL cos_att_use IS CLAMP(VTOL_COS_ATT_FILT, cos_att_floor, 1.0).
  SET target_collective TO CLAMP(target_collective / cos_att_use, 0, coll_max).
  SET VTOL_DIAG_COS_ATT_FILT TO VTOL_COS_ATT_FILT.
  SET VTOL_DIAG_COLL_AFTER_CORR TO target_collective.

  // Optional feed-forward: invert a simple first-order throttle-lag model
  // using engine-model telemetry (TELEM_EM_WORST_SPOOL_LAG). This improves
  // thrust tracking when commanded collective is changing quickly.
  LOCAL ff_collective IS target_collective.
  LOCAL ff_enabled IS _AMO_CFG_BOOL("vtol_em_ff_enabled", VTOL_EM_FF_ENABLED_DEFAULT).
  IF ff_enabled {
    LOCAL ff_gain IS _AMO_CFG_NUM("vtol_em_ff_gain", VTOL_EM_FF_GAIN, 0).
    LOCAL ff_max_lead IS _AMO_CFG_NUM("vtol_em_ff_max_lead", VTOL_EM_FF_MAX_LEAD, 0).
    LOCAL ff_lag_min_s IS _AMO_CFG_NUM("vtol_em_ff_lag_min_s", VTOL_EM_FF_LAG_MIN_S, 0).
    LOCAL ff_alpha_min IS _AMO_CFG_NUM("vtol_em_ff_alpha_min", VTOL_EM_FF_ALPHA_MIN, 0).
    IF ff_gain < 0 { SET ff_gain TO 0. }
    IF ff_gain > 1 { SET ff_gain TO 1. }
    IF ff_max_lead < 0 { SET ff_max_lead TO 0. }
    IF ff_lag_min_s < 0 { SET ff_lag_min_s TO 0. }
    IF ff_alpha_min <= 0 OR ff_alpha_min > 1 { SET ff_alpha_min TO VTOL_EM_FF_ALPHA_MIN. }

    LOCAL em_count IS ROUND(TELEM_EM_ENG_COUNT, 0).
    IF em_count > 0 AND em_lag_s > ff_lag_min_s {
      // Engine-model lag is time-to-near-settled; map to first-order tau.
      LOCAL tau_s IS em_lag_s / 4.6.
      IF tau_s < 0.01 { SET tau_s TO 0.01. }
      LOCAL alpha IS IFC_ACTUAL_DT / (IFC_ACTUAL_DT + tau_s).
      IF alpha < ff_alpha_min { SET alpha TO ff_alpha_min. }
      IF alpha > 1 { SET alpha TO 1. }

      // Predict achieved (lagged) collective from last command.
      IF VTOL_COLLECTIVE_EFF_EST < 0 OR VTOL_COLLECTIVE_EFF_EST > 1 {
        SET VTOL_COLLECTIVE_EFF_EST TO VTOL_COLLECTIVE.
      }
      SET VTOL_COLLECTIVE_EFF_EST TO CLAMP(
        VTOL_COLLECTIVE_EFF_EST + (VTOL_COLLECTIVE - VTOL_COLLECTIVE_EFF_EST) * alpha,
        0, 1
      ).

      LOCAL inv_target IS target_collective.
      IF alpha > 0.0001 {
        SET inv_target TO (target_collective - (1 - alpha) * VTOL_COLLECTIVE_EFF_EST) / alpha.
      }
      LOCAL blended_target IS target_collective + (inv_target - target_collective) * ff_gain.
      SET ff_collective TO CLAMP(
        blended_target,
        target_collective - ff_max_lead,
        target_collective + ff_max_lead
      ).
    } ELSE {
      // No reliable engine-model signal this tick: keep estimator in sync.
      SET VTOL_COLLECTIVE_EFF_EST TO VTOL_COLLECTIVE.
    }
  } ELSE {
    SET VTOL_COLLECTIVE_EFF_EST TO VTOL_COLLECTIVE.
  }

  LOCAL sin_alpha_floor IS _AMO_CFG_NUM("vtol_nacelle_sin_floor", VTOL_NACELLE_SIN_FLOOR, 0.01).
  SET sin_alpha_floor TO CLAMP(sin_alpha_floor, 0.01, 0.5).
  // Use mean per-engine lift fraction (from current servo positions) so yaw
  // differential nacelle tilt is reflected in collective geometry correction.
  LOCAL sin_alpha_lift IS CLAMP(VTOL_HOVER_BLEND, 0, 1).
  IF sin_alpha_lift > sin_alpha_floor {
    SET ff_collective TO CLAMP(ff_collective / sin_alpha_lift, 0, coll_max).
  }

  LOCAL collective IS _VTOL_SLEW_COLLECTIVE(CLAMP(ff_collective, 0, coll_max)).

  IF ABS(vs_err) < 0.5 {
    SET VTOL_HOVER_COLLECTIVE TO
      VTOL_HOVER_COLLECTIVE + (collective - VTOL_HOVER_COLLECTIVE) * VTOL_HOVER_LEARN_RATE.
  }

  RETURN collective.
}

FUNCTION _VTOL_COLLECTIVE_CMD {
  PARAMETER thr_input.

  LOCAL fixed_enabled IS _AMO_CFG_BOOL("vtol_test_fixed_collective_enabled", FALSE).
  IF fixed_enabled {
    LOCAL fixed_collective IS _AMO_CFG_NUM("vtol_test_fixed_collective", VTOL_HOVER_COLLECTIVE, 0).
    LOCAL coll_max IS _AMO_CFG_NUM("vtol_collective_max", VTOL_COLLECTIVE_MAX, 0.1).
    IF coll_max > 1.0 { SET coll_max TO 1.0. }
    IF coll_max < 0.0 { SET coll_max TO 0.0. }
    SET fixed_collective TO CLAMP(fixed_collective, 0, coll_max).

    // VS loop bypass for controller isolation tests.
    SET VTOL_VS_CMD TO 0.
    SET VTOL_VS_INTEGRAL TO 0.
    SET VTOL_COLLECTIVE_EFF_EST TO fixed_collective.
    RETURN _VTOL_SLEW_COLLECTIVE(fixed_collective).
  }

  RETURN _VTOL_VS_HOLD(thr_input).
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
  PARAMETER starvec, forevec, upvec, angvel.
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
  LOCAL status_str IS SHIP:STATUS.
  IF status_str = "LANDED" OR status_str = "PRELAUNCH" { RETURN. }

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
  LOCAL pitch_ang IS 90 - VANG(forevec, upvec).
  LOCAL bank_ang IS ARCSIN(CLAMP(-VDOT(starvec, upvec), -1, 1)).
  LOCAL roll_rate_rads IS -VDOT(angvel, forevec).
  LOCAL pitch_rate_rads IS VDOT(angvel, starvec).
  LOCAL roll_rate_degs IS roll_rate_rads * (180 / CONSTANT:PI).
  LOCAL pitch_rate_degs IS pitch_rate_rads * (180 / CONSTANT:PI).
  LOCAL trim_pitch_max IS _AMO_CFG_NUM("vtol_trim_active_pitch_max", VTOL_TRIM_ACTIVE_PITCH_MAX, 0).
  LOCAL trim_bank_max IS _AMO_CFG_NUM("vtol_trim_active_bank_max", VTOL_TRIM_ACTIVE_BANK_MAX, 0).
  LOCAL trim_pitch_rate_max IS _AMO_CFG_NUM("vtol_trim_active_rate_max", VTOL_TRIM_ACTIVE_RATE_MAX, 0).
  LOCAL trim_roll_rate_max IS _AMO_CFG_NUM("vtol_trim_active_roll_rate_max", VTOL_TRIM_ACTIVE_ROLL_RATE_MAX, 0).
  // Do not adapt trim during dynamic excursions.
  // Trim is for static torque-bias learning, not transient rate damping.
  IF trim_pitch_max > 0 AND ABS(pitch_ang) > trim_pitch_max { RETURN. }
  IF trim_bank_max > 0 AND ABS(bank_ang) > trim_bank_max { RETURN. }
  IF trim_pitch_rate_max > 0 AND ABS(pitch_rate_degs) > trim_pitch_rate_max { RETURN. }
  IF trim_roll_rate_max > 0 AND ABS(roll_rate_degs) > trim_roll_rate_max { RETURN. }

  LOCAL pitch_err IS CLAMP(-pitch_ang, -VTOL_TRIM_PITCH_CLAMP, VTOL_TRIM_PITCH_CLAMP).
  LOCAL trim_bank_clamp IS _AMO_CFG_NUM("vtol_trim_bank_clamp", VTOL_TRIM_BANK_CLAMP, 0.1).
  LOCAL bank_err IS CLAMP(-bank_ang, -trim_bank_clamp, trim_bank_clamp).
  IF ABS(pitch_err) < 0.1 AND ABS(bank_err) < 0.1 { RETURN. }
  LOCAL pitch_drive IS pitch_err.
  LOCAL bank_drive IS bank_err.

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

FUNCTION _VTOL_HANDLE_MODE_EDGES {
  PARAMETER starvec, forevec, upvec.
  // Keep velocity hold latched on whenever a higher-level horizontal mode is active.
  IF VTOL_KHV_ACTIVE OR VTOL_POS_HOLD_ACTIVE {
    SET VTOL_VEL_HOLD_ACTIVE TO TRUE.
  }

  LOCAL vel_engaged IS VTOL_VEL_HOLD_ACTIVE AND NOT _vtol_prev_vel_hold_active.
  LOCAL vel_disengaged IS NOT VTOL_VEL_HOLD_ACTIVE AND _vtol_prev_vel_hold_active.
  LOCAL pos_engaged IS VTOL_POS_HOLD_ACTIVE AND NOT _vtol_prev_pos_hold_active.
  LOCAL pos_disengaged IS NOT VTOL_POS_HOLD_ACTIVE AND _vtol_prev_pos_hold_active.
  LOCAL khv_engaged IS VTOL_KHV_ACTIVE AND NOT _vtol_prev_khv_active.

  IF vel_engaged {
    LOCAL vel_now IS _VTOL_GET_SURFACE_VEL().
    LOCAL max_fwd_pitch_deg IS _AMO_CFG_NUM("vtol_max_fwd_pitch", VTOL_MAX_FWD_PITCH, 0.1).
    LOCAL max_bank_deg IS _AMO_CFG_NUM("vtol_max_bank", VTOL_MAX_BANK, 0.1).
    IF max_fwd_pitch_deg < 0.1 { SET max_fwd_pitch_deg TO 0.1. }
    IF max_bank_deg < 0.1 { SET max_bank_deg TO 0.1. }
    SET VTOL_VN_ACTUAL TO vel_now["vn"].
    SET VTOL_VE_ACTUAL TO vel_now["ve"].
    SET VTOL_VN_CMD TO VTOL_VN_ACTUAL.
    SET VTOL_VE_CMD TO VTOL_VE_ACTUAL.
    SET VTOL_PHI_CMD TO CLAMP(ARCSIN(CLAMP(-VDOT(starvec, upvec), -1, 1)), -max_bank_deg, max_bank_deg).
    SET VTOL_THETA_CMD TO CLAMP(90 - VANG(forevec, upvec), -max_fwd_pitch_deg, max_fwd_pitch_deg).
    SET VTOL_VEL_INT_N TO 0.0.
    SET VTOL_VEL_INT_E TO 0.0.
  } ELSE IF vel_disengaged {
    SET VTOL_VEL_INT_N TO 0.0.
    SET VTOL_VEL_INT_E TO 0.0.
  }

  IF pos_engaged {
    SET VTOL_POS_INT_N TO 0.0.
    SET VTOL_POS_INT_E TO 0.0.
  } ELSE IF pos_disengaged {
    SET VTOL_POS_INT_N TO 0.0.
    SET VTOL_POS_INT_E TO 0.0.
  }

  IF khv_engaged {
    SET VTOL_VEL_INT_N TO 0.0.
    SET VTOL_VEL_INT_E TO 0.0.
  }

  SET _vtol_prev_vel_hold_active TO VTOL_VEL_HOLD_ACTIVE.
  SET _vtol_prev_pos_hold_active TO VTOL_POS_HOLD_ACTIVE.
  SET _vtol_prev_khv_active TO VTOL_KHV_ACTIVE.
}

// ?????? Public API ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????

// AMO / pre-arm tick.
// No LOCK THROTTLE ??? pilot's physical throttle is read as a VS command.
// 50 % throttle ??? 0 m/s VS command (hover).
// kOS manages thrust limiters for: collective (VS hold), trim balance,
// wings-level auto-correction, and pilot differential (roll/pitch/yaw).
FUNCTION VTOL_TICK_PREARM {
  IF NOT _VTOL_ENABLED() {
    _VTOL_CLEAR_DIAG().
    SET VTOL_CMD_ROLL_ACTUAL TO 0.
    SET VTOL_CMD_PITCH_ACTUAL TO 0.
    VTOL_ENG_LIM_ACTUAL:CLEAR().
    SET VTOL_UPSET_ACTIVE TO FALSE.
    SET VTOL_THR_GUARD_ACTIVE TO FALSE.
    SET VTOL_THR_INPUT_USED TO 0.5.
    SET _vtol_prev_vel_hold_active TO FALSE.
    SET _vtol_prev_pos_hold_active TO FALSE.
    SET _vtol_prev_khv_active TO FALSE.
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  VTOL_DISCOVER().
  IF NOT VTOL_DIFF_AVAILABLE {
    _VTOL_CLEAR_DIAG().
    SET VTOL_CMD_ROLL_ACTUAL TO 0.
    SET VTOL_CMD_PITCH_ACTUAL TO 0.
    VTOL_ENG_LIM_ACTUAL:CLEAR().
    SET VTOL_UPSET_ACTIVE TO FALSE.
    SET VTOL_THR_GUARD_ACTIVE TO FALSE.
    SET VTOL_THR_INPUT_USED TO 0.5.
    SET _vtol_prev_vel_hold_active TO FALSE.
    SET _vtol_prev_pos_hold_active TO FALSE.
    SET _vtol_prev_khv_active TO FALSE.
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }
  _VTOL_CLEAR_DIAG().

  // ?????? Pilot inputs ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  LOCAL inputs  IS _VTOL_PILOT_INPUTS().
  LOCAL p_roll  IS CLAMP(inputs["roll"],  -1, 1).
  LOCAL p_pitch IS CLAMP(inputs["pitch"], -1, 1).
  LOCAL p_yaw   IS CLAMP(inputs["yaw"],   -1, 1).
  LOCAL p_thr   IS 0.5.
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTMAINTHROTTLE") {
    SET p_thr TO SHIP:CONTROL:PILOTMAINTHROTTLE.
  }

  // Cache facing vectors and angular velocity once per tick.
  // SHIP:FACING:* and SHIP:ANGULARVEL traverse the part tree on each access;
  // reading them once here eliminates repeated redundant reads across the
  // upset guard, leveling PID, upset detection, upset recovery, and sub-functions.
  LOCAL _starvec IS SHIP:FACING:STARVECTOR.
  LOCAL _forevec IS SHIP:FACING:FOREVECTOR.
  LOCAL _upvec   IS SHIP:UP:VECTOR.
  LOCAL _angvel  IS SHIP:ANGULARVEL.
  _VTOL_UPDATE_RATE_FILTERS(_angvel, _starvec, _forevec).
  _VTOL_UPDATE_NACELLE_SCHEDULE().

  // Low-alt upset throttle guard:
  // when badly banked/pitched near the ground, prevent pilot throttle
  // from collapsing to zero (which maps to strong descent command in VS-hold).
  SET VTOL_THR_GUARD_ACTIVE TO FALSE.
  LOCAL guard_agl_m IS _AMO_CFG_NUM("vtol_upset_guard_agl_m", VTOL_UPSET_GUARD_AGL_M, 0).
  LOCAL guard_thr_min IS _AMO_CFG_NUM("vtol_upset_guard_thr_min", VTOL_UPSET_GUARD_THR_MIN, 0).
  IF guard_thr_min < 0 { SET guard_thr_min TO 0. }
  IF guard_thr_min > 1 { SET guard_thr_min TO 1. }
  IF guard_agl_m > 0 AND GET_AGL() <= guard_agl_m {
    LOCAL guard_bank_abs IS ABS(ARCSIN(CLAMP(-VDOT(_starvec, _upvec), -1, 1))).
    LOCAL guard_pitch_abs IS ABS(90 - VANG(_forevec, _upvec)).
    LOCAL guard_deg_per_rad IS 180 / CONSTANT:PI.
    LOCAL guard_roll_rate_abs IS ABS(-VDOT(_angvel, _forevec) * guard_deg_per_rad).
    LOCAL guard_pitch_rate_abs IS ABS(VDOT(_angvel, _starvec) * guard_deg_per_rad).
    LOCAL upset_bank_deg_g IS _AMO_CFG_NUM("vtol_upset_bank_deg", VTOL_UPSET_BANK_DEG, 0).
    LOCAL upset_pitch_deg_g IS _AMO_CFG_NUM("vtol_upset_pitch_deg", VTOL_UPSET_PITCH_DEG, 0).
    LOCAL upset_roll_rate_degs_g IS _AMO_CFG_NUM("vtol_upset_roll_rate_degs", VTOL_UPSET_ROLL_RATE_DEGS, 0).
    LOCAL upset_pitch_rate_degs_g IS _AMO_CFG_NUM("vtol_upset_pitch_rate_degs", VTOL_UPSET_PITCH_RATE_DEGS, 0).
    LOCAL guard_upset IS FALSE.
    IF guard_bank_abs > upset_bank_deg_g { SET guard_upset TO TRUE. }
    IF guard_pitch_abs > upset_pitch_deg_g { SET guard_upset TO TRUE. }
    IF guard_roll_rate_abs > upset_roll_rate_degs_g { SET guard_upset TO TRUE. }
    IF guard_pitch_rate_abs > upset_pitch_rate_degs_g { SET guard_upset TO TRUE. }
    IF guard_upset AND p_thr < guard_thr_min {
      SET p_thr TO guard_thr_min.
      SET VTOL_THR_GUARD_ACTIVE TO TRUE.
    }
  }
  SET VTOL_THR_INPUT_USED TO p_thr.
  _VTOL_HANDLE_MODE_EDGES(_starvec, _forevec, _upvec).

  // ?????? VS hold: pilot throttle ??? collective limiter level ???????????????
  // 50 % throttle ??? 0 m/s VS command.  PI controller integrates
  // to whatever collective keeps that VS. No LOCK THROTTLE.
  SET VTOL_COLLECTIVE TO _VTOL_COLLECTIVE_CMD(p_thr).

  // Outer loops: position hold -> velocity hold -> attitude targets.
  _VTOL_POS_HOLD().
  _VTOL_VEL_HOLD(p_pitch, p_roll).

  // ?????? Adaptive trim (slow pitch balance integrator) ?????????????????????????????????
  _VTOL_ADAPT_TRIM(_starvec, _forevec, _upvec, _angvel).

  // ?????? Wings-level auto-correction ???????????????????????????????????????????????????????????????????????????????????????
  // When the pilot is not commanding roll or pitch, apply a
  // proportional restoring command from current attitude.
  // Full authority at ~20?? deviation with default gains.
  LOCAL roll_cmd  IS p_roll.
  LOCAL pitch_cmd IS p_pitch.
  LOCAL roll_input_for_level IS p_roll.
  LOCAL pitch_input_for_level IS p_pitch.
  IF VTOL_VEL_HOLD_ACTIVE {
    SET roll_cmd TO 0.0.
    SET pitch_cmd TO 0.0.
    SET roll_input_for_level TO 0.0.
    SET pitch_input_for_level TO 0.0.
  }
  LOCAL level_roll_kp IS _AMO_CFG_NUM("vtol_level_roll_kp", VTOL_LEVEL_ROLL_KP, 0).
  LOCAL level_roll_kd IS _AMO_CFG_NUM("vtol_level_roll_kd", VTOL_LEVEL_ROLL_KD, 0).
  LOCAL level_roll_ki IS _AMO_CFG_NUM("vtol_level_roll_ki", VTOL_LEVEL_ROLL_KI, 0).
  LOCAL level_pitch_kp IS _AMO_CFG_NUM("vtol_level_pitch_kp", VTOL_LEVEL_PITCH_KP, 0).
  LOCAL level_pitch_kd IS _AMO_CFG_NUM("vtol_level_pitch_kd", VTOL_LEVEL_PITCH_KD, 0).
  LOCAL level_pitch_ki IS _AMO_CFG_NUM("vtol_level_pitch_ki", VTOL_LEVEL_PITCH_KI, 0).
  LOCAL level_i_lim IS _AMO_CFG_NUM("vtol_level_i_lim", VTOL_LEVEL_I_LIM, 0).
  LOCAL em_lag_s IS _VTOL_FILTERED_EM_LAG_S().
  IF em_lag_s < 0 { SET em_lag_s TO 0. }
  LOCAL level_gain_lag_ref_s IS _AMO_CFG_NUM(
    "vtol_level_gain_lag_ref_s",
    VTOL_LEVEL_GAIN_LAG_REF_S,
    0.01
  ).
  LOCAL level_kp_min_scale IS _AMO_CFG_NUM("vtol_level_kp_min_scale", VTOL_LEVEL_KP_MIN_SCALE, 0).
  LOCAL level_kd_min_scale IS _AMO_CFG_NUM("vtol_level_kd_min_scale", VTOL_LEVEL_KD_MIN_SCALE, 0).
  LOCAL level_ki_min_scale IS _AMO_CFG_NUM("vtol_level_ki_min_scale", VTOL_LEVEL_KI_MIN_SCALE, 0).
  LOCAL roll_att2rate_kp IS _AMO_CFG_NUM(
    "vtol_level_roll_att2rate_kp",
    VTOL_LEVEL_ROLL_ATT2RATE_KP,
    0
  ).
  LOCAL pitch_att2rate_kp IS _AMO_CFG_NUM(
    "vtol_level_pitch_att2rate_kp",
    VTOL_LEVEL_PITCH_ATT2RATE_KP,
    0
  ).
  LOCAL roll_att2rate_ki IS _AMO_CFG_NUM(
    "vtol_level_roll_att2rate_ki",
    VTOL_LEVEL_ROLL_ATT2RATE_KI,
    0
  ).
  LOCAL pitch_att2rate_ki IS _AMO_CFG_NUM(
    "vtol_level_pitch_att2rate_ki",
    VTOL_LEVEL_PITCH_ATT2RATE_KI,
    0
  ).
  LOCAL roll_rate_kp IS _AMO_CFG_NUM("vtol_level_roll_rate_kp", VTOL_LEVEL_ROLL_RATE_KP, 0).
  LOCAL pitch_rate_kp IS _AMO_CFG_NUM("vtol_level_pitch_rate_kp", VTOL_LEVEL_PITCH_RATE_KP, 0).
  LOCAL kd_roll_accel_cfg IS _AMO_CFG_NUM("vtol_rate_kd_roll_accel", VTOL_RATE_KD_ROLL_ACCEL, 0).
  LOCAL kd_pitch_accel_cfg IS _AMO_CFG_NUM("vtol_rate_kd_pitch_accel", VTOL_RATE_KD_PITCH_ACCEL, 0).
  LOCAL roll_rate_cmd_max_degs IS _AMO_CFG_NUM(
    "vtol_level_roll_rate_cmd_max_degs",
    VTOL_LEVEL_ROLL_RATE_CMD_MAX_DEGS,
    0
  ).
  LOCAL pitch_rate_cmd_max_degs IS _AMO_CFG_NUM(
    "vtol_level_pitch_rate_cmd_max_degs",
    VTOL_LEVEL_PITCH_RATE_CMD_MAX_DEGS,
    0
  ).
  IF level_kp_min_scale < 0 { SET level_kp_min_scale TO 0. }
  IF level_kp_min_scale > 1 { SET level_kp_min_scale TO 1. }
  IF level_kd_min_scale < 0 { SET level_kd_min_scale TO 0. }
  IF level_kd_min_scale > 1 { SET level_kd_min_scale TO 1. }
  IF level_ki_min_scale < 0 { SET level_ki_min_scale TO 0. }
  IF level_ki_min_scale > 1 { SET level_ki_min_scale TO 1. }
  IF roll_att2rate_kp < 0 { SET roll_att2rate_kp TO 0. }
  IF pitch_att2rate_kp < 0 { SET pitch_att2rate_kp TO 0. }
  IF roll_att2rate_ki < 0 { SET roll_att2rate_ki TO 0. }
  IF pitch_att2rate_ki < 0 { SET pitch_att2rate_ki TO 0. }
  IF roll_rate_kp < 0 { SET roll_rate_kp TO 0. }
  IF pitch_rate_kp < 0 { SET pitch_rate_kp TO 0. }
  IF kd_roll_accel_cfg < 0 { SET kd_roll_accel_cfg TO 0. }
  IF kd_pitch_accel_cfg < 0 { SET kd_pitch_accel_cfg TO 0. }
  IF roll_rate_cmd_max_degs < 0 { SET roll_rate_cmd_max_degs TO 0. }
  IF pitch_rate_cmd_max_degs < 0 { SET pitch_rate_cmd_max_degs TO 0. }
  LOCAL level_gain_scale IS 1.0.
  IF level_gain_lag_ref_s > 0.01 AND em_lag_s > level_gain_lag_ref_s {
    SET level_gain_scale TO level_gain_lag_ref_s / em_lag_s.
  }
  SET level_gain_scale TO CLAMP(level_gain_scale, 0, 1).
  SET level_roll_kp TO level_roll_kp * CLAMP(level_gain_scale, level_kp_min_scale, 1.0).
  SET level_roll_kd TO level_roll_kd * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
  SET level_roll_ki TO level_roll_ki * CLAMP(level_gain_scale, level_ki_min_scale, 1.0).
  SET level_pitch_kp TO level_pitch_kp * CLAMP(level_gain_scale, level_kp_min_scale, 1.0).
  SET level_pitch_kd TO level_pitch_kd * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
  SET level_pitch_ki TO level_pitch_ki * CLAMP(level_gain_scale, level_ki_min_scale, 1.0).
  SET roll_att2rate_kp TO roll_att2rate_kp * CLAMP(level_gain_scale, level_kp_min_scale, 1.0).
  SET pitch_att2rate_kp TO pitch_att2rate_kp * CLAMP(level_gain_scale, level_kp_min_scale, 1.0).
  SET roll_att2rate_ki TO roll_att2rate_ki * CLAMP(level_gain_scale, level_ki_min_scale, 1.0).
  SET pitch_att2rate_ki TO pitch_att2rate_ki * CLAMP(level_gain_scale, level_ki_min_scale, 1.0).
  SET roll_rate_kp TO roll_rate_kp * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
  SET pitch_rate_kp TO pitch_rate_kp * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
  LOCAL inertia_gain_sched_enabled IS _AMO_CFG_BOOL(
    "vtol_inertia_gain_sched_enabled",
    VTOL_INERTIA_GAIN_SCHED_ENABLED_DEFAULT
  ).
  LOCAL inertia_gain_nominal IS _AMO_CFG_NUM("vtol_inertia_gain_nominal", VTOL_INERTIA_GAIN_NOMINAL, 0.0001).
  LOCAL inertia_gain_min_scale IS _AMO_CFG_NUM(
    "vtol_inertia_gain_min_scale",
    VTOL_INERTIA_GAIN_MIN_SCALE,
    0.01
  ).
  LOCAL inertia_gain_max_scale IS _AMO_CFG_NUM(
    "vtol_inertia_gain_max_scale",
    VTOL_INERTIA_GAIN_MAX_SCALE,
    0.02
  ).
  IF inertia_gain_nominal < 0.0001 { SET inertia_gain_nominal TO 0.0001. }
  IF inertia_gain_min_scale < 0.05 { SET inertia_gain_min_scale TO 0.05. }
  IF inertia_gain_max_scale < inertia_gain_min_scale { SET inertia_gain_max_scale TO inertia_gain_min_scale. }
  IF inertia_gain_sched_enabled {
    LOCAL roll_inertia_scale IS 1.0.
    LOCAL pitch_inertia_scale IS 1.0.
    IF VTOL_I_ROLL_VALID AND VTOL_I_ROLL_EST > 0.0001 {
      SET roll_inertia_scale TO inertia_gain_nominal / VTOL_I_ROLL_EST.
    }
    IF VTOL_I_PITCH_VALID AND VTOL_I_PITCH_EST > 0.0001 {
      SET pitch_inertia_scale TO inertia_gain_nominal / VTOL_I_PITCH_EST.
    }
    SET roll_inertia_scale TO CLAMP(roll_inertia_scale, inertia_gain_min_scale, inertia_gain_max_scale).
    SET pitch_inertia_scale TO CLAMP(pitch_inertia_scale, inertia_gain_min_scale, inertia_gain_max_scale).
    SET roll_rate_kp TO roll_rate_kp * roll_inertia_scale.
    SET pitch_rate_kp TO pitch_rate_kp * pitch_inertia_scale.
    SET kd_roll_accel_cfg TO kd_roll_accel_cfg * roll_inertia_scale.
    SET kd_pitch_accel_cfg TO kd_pitch_accel_cfg * pitch_inertia_scale.
  }
  LOCAL kd_roll_accel_use IS kd_roll_accel_cfg * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
  LOCAL kd_pitch_accel_use IS kd_pitch_accel_cfg * CLAMP(level_gain_scale, level_kd_min_scale, 1.0).
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
  LOCAL bypass_att_feedback IS _AMO_CFG_BOOL(
    "vtol_bypass_attitude_feedback",
    VTOL_BYPASS_ATTITUDE_FEEDBACK_DEFAULT
  ).
  IF bypass_att_feedback {
    // FF-only test mode: bypass all automatic attitude feedback loops.
    SET level_active TO FALSE.
    SET VTOL_LEVEL_ROLL_INT TO 0.
    SET VTOL_LEVEL_PITCH_INT TO 0.
  }
  // Integral gate: only accumulate when truly airborne (not LANDED/PRELAUNCH).
  // _VTOL_EFFECTIVE_GROUNDED returns FALSE as soon as collective > diff_min, so
  // level_active can become TRUE while the aircraft is still on its gear.  The
  // PD terms are fine in that state; the integral must not wind up against a
  // structural pitch bias that exists only because the gear is still loaded.
  LOCAL status_str IS SHIP:STATUS.
  LOCAL truly_airborne IS (status_str <> "LANDED" AND status_str <> "PRELAUNCH").
  SET VTOL_DIAG_LEVEL_ACTIVE TO level_active.
  SET VTOL_DIAG_TRULY_AIRBORNE TO truly_airborne.
  SET VTOL_DIAG_IS_GROUNDED TO is_grounded.

  // Angular rates from SHIP:ANGULARVEL (world frame, rad/s).
  // Dot with ship axis vectors to get component around each axis.
  // Nose-down pitch rate = positive VDOT with STARVECTOR.
  // Roll-right roll rate = negative VDOT with FOREVECTOR.
  LOCAL ang_vel IS _angvel.
  LOCAL pitch_rate_rads IS VDOT(ang_vel, _starvec).
  LOCAL roll_rate_rads  IS -VDOT(ang_vel, _forevec).
  LOCAL deg_per_rad IS 180 / CONSTANT:PI.
  LOCAL roll_cap IS _AMO_CFG_NUM("vtol_cmd_roll_max", VTOL_CMD_ROLL_MAX, 0).
  LOCAL pitch_cap IS _AMO_CFG_NUM("vtol_cmd_pitch_max", VTOL_CMD_PITCH_MAX, 0).
  IF roll_cap <= 0 { SET roll_cap TO 1.0. }
  IF pitch_cap <= 0 { SET pitch_cap TO 1.0. }
  LOCAL level_aw_alpha_min IS _AMO_CFG_NUM(
    "vtol_level_aw_alpha_min",
    VTOL_LEVEL_AW_ALPHA_MIN,
    0
  ).
  IF level_aw_alpha_min < 0 { SET level_aw_alpha_min TO 0. }
  IF level_aw_alpha_min > 1 { SET level_aw_alpha_min TO 1. }
  LOCAL level_i_unwind_per_s IS _AMO_CFG_NUM(
    "vtol_level_i_unwind_per_s",
    VTOL_LEVEL_I_UNWIND_PER_S,
    0
  ).
  LOCAL level_aw_lag_s IS _AMO_CFG_NUM("vtol_level_aw_lag_s", VTOL_LEVEL_AW_LAG_S, 0).
  LOCAL level_aw_eff_err_min IS _AMO_CFG_NUM(
    "vtol_level_aw_eff_err_min",
    VTOL_LEVEL_AW_EFF_ERR_MIN,
    0
  ).
  IF level_i_unwind_per_s < 0 { SET level_i_unwind_per_s TO 0. }
  IF level_aw_lag_s < 0 { SET level_aw_lag_s TO 0. }
  IF level_aw_eff_err_min < 0 { SET level_aw_eff_err_min TO 0. }
  LOCAL alloc_limited_prev IS VTOL_ALLOC_ALPHA < level_aw_alpha_min.
  LOCAL eff_collective_err IS ABS(VTOL_COLLECTIVE - VTOL_COLLECTIVE_EFF_EST).
  LOCAL lag_limited_prev IS em_lag_s > level_aw_lag_s AND eff_collective_err > level_aw_eff_err_min.
  LOCAL roll_auto_feedback IS FALSE.
  LOCAL pitch_auto_feedback IS FALSE.

  IF level_active AND ABS(roll_input_for_level) < VTOL_TRIM_DEADBAND {
    SET roll_auto_feedback TO TRUE.
    LOCAL bank_ang IS ARCSIN(CLAMP(-VDOT(_starvec, _upvec), -1, 1)).
    LOCAL roll_rate_degs IS roll_rate_rads * deg_per_rad.
    LOCAL roll_target_deg IS 0.0.
    IF VTOL_VEL_HOLD_ACTIVE { SET roll_target_deg TO VTOL_PHI_CMD. }
    LOCAL roll_err IS roll_target_deg - bank_ang.
    LOCAL roll_rate_cmd_p IS roll_err * roll_att2rate_kp.
    LOCAL roll_rate_cmd_i IS VTOL_LEVEL_ROLL_INT * roll_att2rate_ki.
    LOCAL roll_rate_cmd IS CLAMP(
      roll_rate_cmd_p + roll_rate_cmd_i,
      -roll_rate_cmd_max_degs,
       roll_rate_cmd_max_degs
    ).
    LOCAL roll_p_term IS roll_rate_cmd_p * roll_rate_kp.
    LOCAL roll_i_term IS roll_rate_cmd_i * roll_rate_kp.
    LOCAL roll_d_term IS -roll_rate_degs * roll_rate_kp.
    LOCAL roll_unsat IS (roll_rate_cmd - roll_rate_degs) * roll_rate_kp.
    LOCAL roll_at_hi IS roll_unsat >= roll_cap AND roll_err > 0.
    LOCAL roll_at_lo IS roll_unsat <= -roll_cap AND roll_err < 0.
    IF NOT truly_airborne {
      SET VTOL_LEVEL_ROLL_INT TO 0.
    } ELSE IF alloc_limited_prev OR lag_limited_prev OR roll_at_hi OR roll_at_lo {
      LOCAL roll_unwind_fac IS CLAMP(1.0 - level_i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
      SET VTOL_LEVEL_ROLL_INT TO VTOL_LEVEL_ROLL_INT * roll_unwind_fac.
    } ELSE {
      SET VTOL_LEVEL_ROLL_INT TO CLAMP(
        VTOL_LEVEL_ROLL_INT + roll_err * IFC_ACTUAL_DT,
        -level_i_lim,
         level_i_lim
      ).
      SET roll_rate_cmd_i TO VTOL_LEVEL_ROLL_INT * roll_att2rate_ki.
      SET roll_rate_cmd TO CLAMP(
        roll_rate_cmd_p + roll_rate_cmd_i,
        -roll_rate_cmd_max_degs,
         roll_rate_cmd_max_degs
      ).
    }
    SET roll_p_term TO roll_rate_cmd_p * roll_rate_kp.
    SET roll_i_term TO roll_rate_cmd_i * roll_rate_kp.
    SET roll_d_term TO -roll_rate_degs * roll_rate_kp.
    SET roll_unsat TO (roll_rate_cmd - roll_rate_degs) * roll_rate_kp.
    LOCAL roll_d_accel_cmd IS -VTOL_RATE_P_DOT_FILT * kd_roll_accel_use.
    SET roll_unsat TO roll_unsat + roll_d_accel_cmd.
    SET roll_cmd TO CLAMP(roll_unsat, -1, 1).
    SET VTOL_DIAG_ROLL_ERR TO roll_err.
    SET VTOL_DIAG_ROLL_P TO roll_p_term.
    SET VTOL_DIAG_ROLL_I TO roll_i_term.
    SET VTOL_DIAG_ROLL_D TO roll_d_term + roll_d_accel_cmd.
    SET VTOL_DIAG_ROLL_UNSAT TO roll_unsat.
    SET VTOL_DIAG_ROLL_D_ACCEL TO roll_d_accel_cmd.
  } ELSE {
    SET VTOL_LEVEL_ROLL_INT TO 0.
    SET VTOL_DIAG_ROLL_D_ACCEL TO 0.0.
  }
  IF level_active AND ABS(pitch_input_for_level) < VTOL_TRIM_DEADBAND {
    SET pitch_auto_feedback TO TRUE.
    LOCAL pitch_ang IS 90 - VANG(_forevec, _upvec).
    LOCAL pitch_rate_degs IS pitch_rate_rads * deg_per_rad.
    LOCAL pitch_target_deg IS 0.0.
    IF VTOL_VEL_HOLD_ACTIVE { SET pitch_target_deg TO VTOL_THETA_CMD. }
    LOCAL pitch_err IS pitch_target_deg - pitch_ang.
    LOCAL pitch_rate_cmd_p IS -pitch_err * pitch_att2rate_kp.
    LOCAL pitch_rate_cmd_i IS -VTOL_LEVEL_PITCH_INT * pitch_att2rate_ki.
    LOCAL pitch_rate_cmd IS CLAMP(
      pitch_rate_cmd_p + pitch_rate_cmd_i,
      -pitch_rate_cmd_max_degs,
       pitch_rate_cmd_max_degs
    ).
    LOCAL pitch_p_term IS -pitch_rate_cmd_p * pitch_rate_kp.
    LOCAL pitch_i_term IS -pitch_rate_cmd_i * pitch_rate_kp.
    LOCAL pitch_d_term IS pitch_rate_degs * pitch_rate_kp.
    // Pitch command convention: positive pitch_cmd requests nose-up torque.
    LOCAL pitch_unsat IS -(pitch_rate_cmd - pitch_rate_degs) * pitch_rate_kp.
    LOCAL pitch_at_hi IS pitch_unsat >= pitch_cap AND pitch_err > 0.
    LOCAL pitch_at_lo IS pitch_unsat <= -pitch_cap AND pitch_err < 0.
    IF NOT truly_airborne {
      SET VTOL_LEVEL_PITCH_INT TO 0.
    } ELSE IF alloc_limited_prev OR lag_limited_prev OR pitch_at_hi OR pitch_at_lo {
      LOCAL pitch_unwind_fac IS CLAMP(1.0 - level_i_unwind_per_s * IFC_ACTUAL_DT, 0, 1).
      SET VTOL_LEVEL_PITCH_INT TO VTOL_LEVEL_PITCH_INT * pitch_unwind_fac.
    } ELSE {
      SET VTOL_LEVEL_PITCH_INT TO CLAMP(
        VTOL_LEVEL_PITCH_INT + pitch_err * IFC_ACTUAL_DT,
        -level_i_lim,
         level_i_lim
      ).
      SET pitch_rate_cmd_i TO -VTOL_LEVEL_PITCH_INT * pitch_att2rate_ki.
      SET pitch_rate_cmd TO CLAMP(
        pitch_rate_cmd_p + pitch_rate_cmd_i,
        -pitch_rate_cmd_max_degs,
         pitch_rate_cmd_max_degs
      ).
    }
    SET pitch_p_term TO -pitch_rate_cmd_p * pitch_rate_kp.
    SET pitch_i_term TO -pitch_rate_cmd_i * pitch_rate_kp.
    SET pitch_d_term TO pitch_rate_degs * pitch_rate_kp.
    SET pitch_unsat TO -(pitch_rate_cmd - pitch_rate_degs) * pitch_rate_kp.
    // Positive q_dot means nose-down acceleration; oppose it with nose-up
    // command, which is positive pitch_cmd in this convention.
    LOCAL pitch_d_accel_cmd IS VTOL_RATE_Q_DOT_FILT * kd_pitch_accel_use.
    SET pitch_unsat TO pitch_unsat + pitch_d_accel_cmd.
    SET pitch_cmd TO CLAMP(pitch_unsat, -1, 1).
    SET VTOL_DIAG_PITCH_ERR TO pitch_err.
    SET VTOL_DIAG_PITCH_P TO pitch_p_term.
    SET VTOL_DIAG_PITCH_I TO pitch_i_term.
    SET VTOL_DIAG_PITCH_D TO pitch_d_term + pitch_d_accel_cmd.
    SET VTOL_DIAG_PITCH_UNSAT TO pitch_unsat.
    SET VTOL_DIAG_PITCH_D_ACCEL TO pitch_d_accel_cmd.
  } ELSE {
    SET VTOL_LEVEL_PITCH_INT TO 0.
    SET VTOL_DIAG_PITCH_D_ACCEL TO 0.0.
  }

  // Hard command caps keep the differential mixer out of bang-bang saturation.
  SET VTOL_DIAG_CMD_ROLL_PRECAP TO roll_cmd.
  SET VTOL_DIAG_CMD_PITCH_PRECAP TO pitch_cmd.
  SET roll_cmd TO CLAMP(roll_cmd, -roll_cap, roll_cap).
  SET pitch_cmd TO CLAMP(pitch_cmd, -pitch_cap, pitch_cap).
  SET VTOL_DIAG_CMD_ROLL_POSTCAP TO roll_cmd.
  SET VTOL_DIAG_CMD_PITCH_POSTCAP TO pitch_cmd.

  // Upset mode: if attitude/rates are large, force conservative damping
  // commands and clear integrators to avoid runaway oscillation.
  LOCAL bank_abs_u IS ABS(ARCSIN(CLAMP(-VDOT(_starvec, _upvec), -1, 1))).
  LOCAL pitch_abs_u IS ABS(90 - VANG(_forevec, _upvec)).
  LOCAL roll_rate_abs_u IS ABS(roll_rate_rads * deg_per_rad).
  LOCAL pitch_rate_abs_u IS ABS(pitch_rate_rads * deg_per_rad).
  LOCAL upset_bank_deg IS _AMO_CFG_NUM("vtol_upset_bank_deg", VTOL_UPSET_BANK_DEG, 0).
  LOCAL upset_pitch_deg IS _AMO_CFG_NUM("vtol_upset_pitch_deg", VTOL_UPSET_PITCH_DEG, 0).
  LOCAL upset_roll_rate_degs IS _AMO_CFG_NUM("vtol_upset_roll_rate_degs", VTOL_UPSET_ROLL_RATE_DEGS, 0).
  LOCAL upset_pitch_rate_degs IS _AMO_CFG_NUM("vtol_upset_pitch_rate_degs", VTOL_UPSET_PITCH_RATE_DEGS, 0).
  LOCAL upset_exit_bank_deg IS _AMO_CFG_NUM("vtol_upset_exit_bank_deg", VTOL_UPSET_EXIT_BANK_DEG, 0).
  LOCAL upset_exit_pitch_deg IS _AMO_CFG_NUM("vtol_upset_exit_pitch_deg", VTOL_UPSET_EXIT_PITCH_DEG, 0).
  LOCAL upset_exit_roll_rate_degs IS _AMO_CFG_NUM(
    "vtol_upset_exit_roll_rate_degs",
    VTOL_UPSET_EXIT_ROLL_RATE_DEGS,
    0
  ).
  LOCAL upset_exit_pitch_rate_degs IS _AMO_CFG_NUM(
    "vtol_upset_exit_pitch_rate_degs",
    VTOL_UPSET_EXIT_PITCH_RATE_DEGS,
    0
  ).
  LOCAL upset_hold_s IS _AMO_CFG_NUM("vtol_upset_hold_s", VTOL_UPSET_HOLD_S, 0).
  IF upset_exit_bank_deg <= 0 { SET upset_exit_bank_deg TO upset_bank_deg * 0.70. }
  IF upset_exit_pitch_deg <= 0 { SET upset_exit_pitch_deg TO upset_pitch_deg * 0.70. }
  IF upset_exit_roll_rate_degs <= 0 { SET upset_exit_roll_rate_degs TO upset_roll_rate_degs * 0.70. }
  IF upset_exit_pitch_rate_degs <= 0 { SET upset_exit_pitch_rate_degs TO upset_pitch_rate_degs * 0.70. }
  IF upset_hold_s < 0 { SET upset_hold_s TO 0. }

  LOCAL upset_enter IS FALSE.
  LOCAL upset_exit_ok IS FALSE.
  IF NOT bypass_att_feedback {
    IF bank_abs_u > upset_bank_deg { SET upset_enter TO TRUE. }
    IF pitch_abs_u > upset_pitch_deg { SET upset_enter TO TRUE. }
    IF roll_rate_abs_u > upset_roll_rate_degs { SET upset_enter TO TRUE. }
    IF pitch_rate_abs_u > upset_pitch_rate_degs { SET upset_enter TO TRUE. }

    IF bank_abs_u < upset_exit_bank_deg {
      IF pitch_abs_u < upset_exit_pitch_deg {
        IF roll_rate_abs_u < upset_exit_roll_rate_degs {
          IF pitch_rate_abs_u < upset_exit_pitch_rate_degs {
            SET upset_exit_ok TO TRUE.
          }
        }
      }
    }
  } ELSE {
    SET _vtol_upset_latched TO FALSE.
    SET _vtol_upset_entry_ut TO -1.
  }

  IF _vtol_upset_latched {
    IF _vtol_upset_entry_ut < 0 { SET _vtol_upset_entry_ut TO TIME:SECONDS. }
    LOCAL hold_elapsed IS TRUE.
    IF upset_hold_s > 0 {
      SET hold_elapsed TO (TIME:SECONDS - _vtol_upset_entry_ut) >= upset_hold_s.
    }
    IF hold_elapsed AND upset_exit_ok {
      SET _vtol_upset_latched TO FALSE.
      SET _vtol_upset_entry_ut TO -1.
    }
  } ELSE IF upset_enter {
    SET _vtol_upset_latched TO TRUE.
    SET _vtol_upset_entry_ut TO TIME:SECONDS.
  }
  LOCAL upset_active IS _vtol_upset_latched.
  SET VTOL_UPSET_ACTIVE TO upset_active.
  IF upset_active {
    LOCAL upset_cmd_roll_max IS _AMO_CFG_NUM("vtol_upset_cmd_roll_max", VTOL_UPSET_CMD_MAX_ROLL, 0).
    LOCAL upset_cmd_pitch_max IS _AMO_CFG_NUM("vtol_upset_cmd_pitch_max", VTOL_UPSET_CMD_MAX_PITCH, 0).
    LOCAL upset_cmd_legacy IS _AMO_CFG_NUM("vtol_upset_cmd_max", VTOL_UPSET_CMD_MAX, 0).
    IF upset_cmd_legacy <= 0 { SET upset_cmd_legacy TO 0.35. }
    IF upset_cmd_legacy > 1 { SET upset_cmd_legacy TO 1.0. }
    IF upset_cmd_roll_max <= 0 { SET upset_cmd_roll_max TO upset_cmd_legacy. }
    IF upset_cmd_pitch_max <= 0 { SET upset_cmd_pitch_max TO upset_cmd_legacy. }
    IF upset_cmd_roll_max > 1 { SET upset_cmd_roll_max TO 1.0. }
    IF upset_cmd_pitch_max > 1 { SET upset_cmd_pitch_max TO 1.0. }
    LOCAL upset_roll_rate_kp IS _AMO_CFG_NUM(
      "vtol_upset_roll_rate_kp",
      VTOL_UPSET_ROLL_RATE_KP,
      0
    ).
    LOCAL upset_pitch_rate_kp IS _AMO_CFG_NUM(
      "vtol_upset_pitch_rate_kp",
      VTOL_UPSET_PITCH_RATE_KP,
      0
    ).
    IF upset_roll_rate_kp <= 0 { SET upset_roll_rate_kp TO level_roll_kd. }
    IF upset_pitch_rate_kp <= 0 { SET upset_pitch_rate_kp TO level_pitch_kd. }
    LOCAL roll_rate_u_degs IS roll_rate_rads * deg_per_rad.
    LOCAL pitch_rate_u_degs IS pitch_rate_rads * deg_per_rad.
    SET VTOL_LEVEL_ROLL_INT TO 0.
    SET VTOL_LEVEL_PITCH_INT TO 0.
    SET roll_auto_feedback TO TRUE.
    SET pitch_auto_feedback TO TRUE.
    // Upset recovery is rate-priority: damp rates first, avoid attitude-P flipovers.
    SET roll_cmd TO CLAMP(
      -roll_rate_u_degs * upset_roll_rate_kp,
      -upset_cmd_roll_max, upset_cmd_roll_max).
    SET pitch_cmd TO CLAMP(
      pitch_rate_u_degs * upset_pitch_rate_kp,
      -upset_cmd_pitch_max, upset_cmd_pitch_max).
  }
  SET VTOL_DIAG_CMD_ROLL_POSTUPSET TO roll_cmd.
  SET VTOL_DIAG_CMD_PITCH_POSTUPSET TO pitch_cmd.

  LOCAL cmd_slew IS _AMO_CFG_NUM("vtol_cmd_slew_per_s", VTOL_CMD_SLEW_PER_S, 0).
  LOCAL level_cmd_slew IS _AMO_CFG_NUM("vtol_level_cmd_slew_per_s", VTOL_LEVEL_CMD_SLEW_PER_S, 0).
  LOCAL cmd_phys_slew_min IS _AMO_CFG_NUM("vtol_cmd_phys_slew_min", VTOL_CMD_PHYS_SLEW_MIN, 0).
  LOCAL upset_pitch_phys_slew_scale IS _AMO_CFG_NUM(
    "vtol_upset_pitch_phys_slew_scale",
    VTOL_UPSET_PITCH_PHYS_SLEW_SCALE,
    0
  ).
  LOCAL upset_pitch_slew_bypass IS _AMO_CFG_BOOL(
    "vtol_upset_pitch_slew_bypass",
    VTOL_UPSET_PITCH_SLEW_BYPASS
  ).
  LOCAL lag_tau_s IS _AMO_CFG_NUM("vtol_lag_filter_tau_s", VTOL_LAG_FILTER_TAU_S, 0).
  LOCAL level_cmd_lag_ref_s IS _AMO_CFG_NUM(
    "vtol_level_cmd_slew_lag_ref_s",
    VTOL_LEVEL_CMD_SLEW_LAG_REF_S,
    0.01
  ).
  LOCAL level_cmd_slew_min_scale IS _AMO_CFG_NUM(
    "vtol_level_cmd_slew_min_scale",
    VTOL_LEVEL_CMD_SLEW_MIN_SCALE,
    0
  ).
  IF level_cmd_slew <= 0 { SET level_cmd_slew TO cmd_slew. }
  IF cmd_phys_slew_min < 0 { SET cmd_phys_slew_min TO 0. }
  IF upset_pitch_phys_slew_scale < 1.0 { SET upset_pitch_phys_slew_scale TO 1.0. }
  IF level_cmd_slew_min_scale < 0 { SET level_cmd_slew_min_scale TO 0. }
  IF level_cmd_slew_min_scale > 1 { SET level_cmd_slew_min_scale TO 1. }
  LOCAL roll_slew_lag_s IS em_lag_s.
  LOCAL pitch_slew_lag_s IS em_lag_s.
  IF roll_auto_feedback {
    SET roll_slew_lag_s TO _VTOL_AXIS_EFFECTIVE_SPOOL_LAG_S("roll").
  }
  IF pitch_auto_feedback {
    SET pitch_slew_lag_s TO _VTOL_AXIS_EFFECTIVE_SPOOL_LAG_S("pitch").
  }
  IF roll_slew_lag_s < 0 { SET roll_slew_lag_s TO 0. }
  IF pitch_slew_lag_s < 0 { SET pitch_slew_lag_s TO 0. }
  SET _vtol_roll_lag_filt_s TO _VTOL_FILTER_LAG(_vtol_roll_lag_filt_s, roll_slew_lag_s, lag_tau_s).
  SET _vtol_pitch_lag_filt_s TO _VTOL_FILTER_LAG(_vtol_pitch_lag_filt_s, pitch_slew_lag_s, lag_tau_s).
  SET roll_slew_lag_s TO _vtol_roll_lag_filt_s.
  SET pitch_slew_lag_s TO _vtol_pitch_lag_filt_s.
  LOCAL roll_level_cmd_slew_scale IS 1.0.
  LOCAL pitch_level_cmd_slew_scale IS 1.0.
  IF level_cmd_lag_ref_s > 0.01 AND roll_slew_lag_s > level_cmd_lag_ref_s {
    SET roll_level_cmd_slew_scale TO level_cmd_lag_ref_s / roll_slew_lag_s.
  }
  IF level_cmd_lag_ref_s > 0.01 AND pitch_slew_lag_s > level_cmd_lag_ref_s {
    SET pitch_level_cmd_slew_scale TO level_cmd_lag_ref_s / pitch_slew_lag_s.
  }
  SET roll_level_cmd_slew_scale TO CLAMP(roll_level_cmd_slew_scale, level_cmd_slew_min_scale, 1.0).
  SET pitch_level_cmd_slew_scale TO CLAMP(pitch_level_cmd_slew_scale, level_cmd_slew_min_scale, 1.0).
  LOCAL roll_level_cmd_slew_use IS level_cmd_slew * roll_level_cmd_slew_scale.
  LOCAL pitch_level_cmd_slew_use IS level_cmd_slew * pitch_level_cmd_slew_scale.
  LOCAL roll_cmd_slew_use IS cmd_slew.
  LOCAL pitch_cmd_slew_use IS cmd_slew.
  IF roll_auto_feedback { SET roll_cmd_slew_use TO roll_level_cmd_slew_use. }
  IF pitch_auto_feedback { SET pitch_cmd_slew_use TO pitch_level_cmd_slew_use. }
  // Section 11.2: command pre-filtering must respect actuator lag bandwidth.
  // For first-order lag tau, useful command slew is bounded by ~1/tau.
  IF roll_slew_lag_s > 0.05 {
    LOCAL roll_phys_slew_max IS MAX(cmd_phys_slew_min, 1.0 / roll_slew_lag_s).
    IF roll_cmd_slew_use <= 0 OR roll_cmd_slew_use > roll_phys_slew_max {
      SET roll_cmd_slew_use TO roll_phys_slew_max.
    }
  }
  IF pitch_slew_lag_s > 0.05 {
    LOCAL pitch_phys_slew_max IS 1.0 / pitch_slew_lag_s.
    IF upset_active {
      SET pitch_phys_slew_max TO pitch_phys_slew_max * upset_pitch_phys_slew_scale.
    }
    IF pitch_phys_slew_max < cmd_phys_slew_min {
      SET pitch_phys_slew_max TO cmd_phys_slew_min.
    }
    IF pitch_cmd_slew_use <= 0 OR pitch_cmd_slew_use > pitch_phys_slew_max {
      SET pitch_cmd_slew_use TO pitch_phys_slew_max.
    }
  }
  IF upset_active AND upset_pitch_slew_bypass {
    SET pitch_cmd_slew_use TO 0.
  }
  IF roll_cmd_slew_use > 0 {
    LOCAL roll_cmd_step IS roll_cmd_slew_use * IFC_ACTUAL_DT.
    SET roll_cmd TO CLAMP(roll_cmd, _vtol_prev_roll_cmd - roll_cmd_step, _vtol_prev_roll_cmd + roll_cmd_step).
  }
  IF pitch_cmd_slew_use > 0 {
    LOCAL pitch_cmd_step IS pitch_cmd_slew_use * IFC_ACTUAL_DT.
    SET pitch_cmd TO CLAMP(pitch_cmd, _vtol_prev_pitch_cmd - pitch_cmd_step, _vtol_prev_pitch_cmd + pitch_cmd_step).
  }
  SET VTOL_DIAG_CMD_ROLL_POSTSLEW TO roll_cmd.
  SET VTOL_DIAG_CMD_PITCH_POSTSLEW TO pitch_cmd.
  SET _vtol_prev_roll_cmd TO roll_cmd.
  SET _vtol_prev_pitch_cmd TO pitch_cmd.
  SET VTOL_CMD_ROLL_ACTUAL TO roll_cmd.
  SET VTOL_CMD_PITCH_ACTUAL TO pitch_cmd.

  LOCAL collective_apply IS VTOL_COLLECTIVE.
  LOCAL upset_collective_cap IS _AMO_CFG_NUM(
    "vtol_upset_collective_cap",
    VTOL_UPSET_COLLECTIVE_CAP,
    -1
  ).
  IF upset_active AND upset_collective_cap >= 0 {
    IF upset_collective_cap > 1 { SET upset_collective_cap TO 1.0. }
    SET collective_apply TO MIN(collective_apply, upset_collective_cap).
  }
  SET VTOL_COLLECTIVE TO collective_apply.
  _VTOL_APPLY_ENGINES(collective_apply, roll_cmd, pitch_cmd, _starvec, _forevec, _upvec, _angvel).
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
    _VTOL_CLEAR_DIAG().
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }

  VTOL_DISCOVER().
  IF NOT VTOL_DIFF_AVAILABLE {
    _VTOL_CLEAR_DIAG().
    IF VTOL_ACTIVE { VTOL_RELEASE(). }
    RETURN.
  }
  _VTOL_CLEAR_DIAG().

  LOCAL _starvec IS SHIP:FACING:STARVECTOR.
  LOCAL _forevec IS SHIP:FACING:FOREVECTOR.
  LOCAL _upvec   IS SHIP:UP:VECTOR.
  LOCAL _angvel  IS SHIP:ANGULARVEL.
  _VTOL_UPDATE_RATE_FILTERS(_angvel, _starvec, _forevec).
  _VTOL_UPDATE_NACELLE_SCHEDULE().

  SET VTOL_COLLECTIVE TO _VTOL_COLLECTIVE_CMD(thr_input).
  SET VTOL_THR_INPUT_USED TO thr_input.
  SET VTOL_THR_GUARD_ACTIVE TO FALSE.
  SET VTOL_UPSET_ACTIVE TO FALSE.
  LOCAL roll_use IS CLAMP(roll_cmd,  -1, 1).
  LOCAL pitch_use IS CLAMP(pitch_cmd, -1, 1).
  SET VTOL_DIAG_CMD_ROLL_PRECAP TO roll_cmd.
  SET VTOL_DIAG_CMD_PITCH_PRECAP TO pitch_cmd.
  SET VTOL_DIAG_CMD_ROLL_POSTCAP TO roll_use.
  SET VTOL_DIAG_CMD_PITCH_POSTCAP TO pitch_use.
  SET VTOL_DIAG_CMD_ROLL_POSTUPSET TO roll_use.
  SET VTOL_DIAG_CMD_PITCH_POSTUPSET TO pitch_use.
  SET VTOL_DIAG_CMD_ROLL_POSTSLEW TO roll_use.
  SET VTOL_DIAG_CMD_PITCH_POSTSLEW TO pitch_use.
  SET VTOL_CMD_ROLL_ACTUAL TO roll_use.
  SET VTOL_CMD_PITCH_ACTUAL TO pitch_use.
  _VTOL_APPLY_ENGINES(
    VTOL_COLLECTIVE,
    roll_use,
    pitch_use,
    _starvec, _forevec, _upvec, _angvel
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
  _VTOL_CLEAR_DIAG().
  SET VTOL_CMD_ROLL_ACTUAL TO 0.
  SET VTOL_CMD_PITCH_ACTUAL TO 0.
  VTOL_ENG_LIM_ACTUAL:CLEAR().
  SET VTOL_UPSET_ACTIVE TO FALSE.
  SET VTOL_THR_GUARD_ACTIVE TO FALSE.
  SET VTOL_THR_INPUT_USED TO 0.5.
  SET VTOL_COLLECTIVE_EFF_EST TO VTOL_COLLECTIVE.
  SET VTOL_L_CMD_PREV TO 0.0.
  SET VTOL_M_CMD_PREV TO 0.0.
  SET VTOL_RATE_P_FILT TO 0.0.
  SET VTOL_RATE_Q_FILT TO 0.0.
  SET VTOL_RATE_P_FILT_PREV TO 0.0.
  SET VTOL_RATE_Q_FILT_PREV TO 0.0.
  SET VTOL_RATE_P_DOT_FILT TO 0.0.
  SET VTOL_RATE_Q_DOT_FILT TO 0.0.
  SET VTOL_COS_ATT_FILT TO 1.0.
  VTOL_CLEAR_YAW_INPUT_OVERRIDE().
  VTOL_ENG_EFF_ACT_EST:CLEAR().
  VTOL_ENG_LAG_EST_S:CLEAR().
  SET VTOL_EFF_LAG_WORST_S TO 0.0.
  SET VTOL_VEL_INT_N TO 0.0.
  SET VTOL_VEL_INT_E TO 0.0.
  SET VTOL_VN_ACTUAL TO 0.0.
  SET VTOL_VE_ACTUAL TO 0.0.
  SET VTOL_VN_CMD TO 0.0.
  SET VTOL_VE_CMD TO 0.0.
  SET VTOL_PHI_CMD TO 0.0.
  SET VTOL_THETA_CMD TO 0.0.
  SET VTOL_POS_INT_N TO 0.0.
  SET VTOL_POS_INT_E TO 0.0.
  SET VTOL_VEL_HOLD_ACTIVE TO FALSE.
  SET VTOL_KHV_ACTIVE TO FALSE.
  SET VTOL_POS_HOLD_ACTIVE TO FALSE.
  SET VTOL_HOVER_BLEND TO 1.0.
  SET VTOL_TRANS_ACTIVE TO FALSE.
  SET VTOL_DIAG_INERTIA_EST_ACTIVE TO FALSE.
  SET VTOL_DIAG_I_ROLL_EST TO VTOL_I_ROLL_EST.
  SET VTOL_DIAG_I_PITCH_EST TO VTOL_I_PITCH_EST.
  SET VTOL_DIAG_I_ROLL_TAU TO 0.0.
  SET VTOL_DIAG_I_PITCH_TAU TO 0.0.
  SET VTOL_DIAG_I_ROLL_ALPHA TO 0.0.
  SET VTOL_DIAG_I_PITCH_ALPHA TO 0.0.
  LOCAL hover_angle_rel IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  SET VTOL_NACELLE_ALPHA_CMD TO hover_angle_rel.
  SET VTOL_NACELLE_ALPHA_EST TO hover_angle_rel.
  SET _vtol_em_lag_filt_s TO -1.
  SET _vtol_em_lag_filt_cycle_ut TO -1.
  SET _vtol_roll_lag_filt_s TO -1.
  SET _vtol_pitch_lag_filt_s TO -1.
  SET _vtol_upset_latched TO FALSE.
  SET _vtol_upset_entry_ut TO -1.
  SET _vtol_prev_vel_hold_active TO FALSE.
  SET _vtol_prev_pos_hold_active TO FALSE.
  SET _vtol_prev_khv_active TO FALSE.
  SET _vtol_prev_mean_cmd_eff TO -1.
  SET VTOL_ACTIVE TO FALSE.
}

// Full state clear ??? use when re-arming or switching aircraft.
FUNCTION VTOL_RESET {
  VTOL_RELEASE().
  VTOL_CLEAR_YAW_INPUT_OVERRIDE().
  VTOL_ENG_LIST:CLEAR().
  VTOL_SRV_LIST:CLEAR().
  VTOL_ROLL_MIX:CLEAR().
  VTOL_PITCH_MIX:CLEAR().
  VTOL_YAW_SRV_MIX:CLEAR().
  VTOL_TRIM_OFFSET:CLEAR().
  VTOL_MAX_THRUST:CLEAR().
  VTOL_ENG_ARM_X_M:CLEAR().
  VTOL_ENG_ARM_Y_M:CLEAR().
  VTOL_ENG_EFF_ACT_EST:CLEAR().
  VTOL_ENG_LAG_EST_S:CLEAR().
  SET VTOL_DISCOVERED     TO FALSE.
  SET VTOL_DIFF_AVAILABLE TO FALSE.
  SET VTOL_SRV_AVAIL      TO FALSE.
  SET VTOL_ARM_ROLL_M     TO 0.0.
  SET VTOL_ARM_PITCH_M    TO 0.0.
  SET VTOL_EFF_LAG_WORST_S TO 0.0.
  SET VTOL_HOVER_COLLECTIVE        TO 0.50.
  SET VTOL_HOVER_COLLECTIVE_SEEDED TO FALSE.
  SET VTOL_COLLECTIVE              TO 0.
  SET VTOL_COLLECTIVE_EFF_EST      TO 0.50.
  SET VTOL_VS_CMD           TO 0.
  SET VTOL_VS_INTEGRAL      TO 0.
  SET VTOL_ALT_HOLD         TO FALSE.
  SET VTOL_ALT_CMD          TO 0.
  SET VTOL_ALLOC_ALPHA      TO 1.0.
  SET VTOL_ALLOC_SHIFT      TO 0.0.
  SET VTOL_LEVEL_ROLL_INT   TO 0.0.
  SET VTOL_LEVEL_PITCH_INT  TO 0.0.
  SET VTOL_RATE_P_FILT      TO 0.0.
  SET VTOL_RATE_Q_FILT      TO 0.0.
  SET VTOL_RATE_P_FILT_PREV TO 0.0.
  SET VTOL_RATE_Q_FILT_PREV TO 0.0.
  SET VTOL_RATE_P_DOT_FILT  TO 0.0.
  SET VTOL_RATE_Q_DOT_FILT  TO 0.0.
  SET VTOL_COS_ATT_FILT     TO 1.0.
  SET VTOL_L_CMD_PREV       TO 0.0.
  SET VTOL_M_CMD_PREV       TO 0.0.
  SET VTOL_VEL_INT_N        TO 0.0.
  SET VTOL_VEL_INT_E        TO 0.0.
  SET VTOL_VN_ACTUAL        TO 0.0.
  SET VTOL_VE_ACTUAL        TO 0.0.
  SET VTOL_VN_CMD           TO 0.0.
  SET VTOL_VE_CMD           TO 0.0.
  SET VTOL_VEL_HOLD_ACTIVE  TO FALSE.
  SET VTOL_KHV_ACTIVE       TO FALSE.
  SET VTOL_PHI_CMD          TO 0.0.
  SET VTOL_THETA_CMD        TO 0.0.
  SET VTOL_POS_INT_N        TO 0.0.
  SET VTOL_POS_INT_E        TO 0.0.
  SET VTOL_TARGET_LAT       TO SHIP:LATITUDE.
  SET VTOL_TARGET_LNG       TO SHIP:LONGITUDE.
  SET VTOL_TARGET_ALT       TO 0.0.
  SET VTOL_POS_HOLD_ACTIVE  TO FALSE.
  _VTOL_RESET_INERTIA_ESTIMATOR().
  LOCAL hover_angle_res IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  SET VTOL_NACELLE_ALPHA_CMD TO hover_angle_res.
  SET VTOL_NACELLE_ALPHA_EST TO hover_angle_res.
  SET VTOL_HOVER_BLEND       TO 1.0.
  SET VTOL_TRANS_ACTIVE      TO FALSE.
  _VTOL_CLEAR_DIAG().
  SET _vtol_prev_roll_cmd   TO 0.0.
  SET _vtol_prev_pitch_cmd  TO 0.0.
  SET _vtol_em_lag_filt_s   TO -1.
  SET _vtol_em_lag_filt_cycle_ut TO -1.
  SET _vtol_roll_lag_filt_s TO -1.
  SET _vtol_pitch_lag_filt_s TO -1.
  SET _vtol_upset_latched TO FALSE.
  SET _vtol_upset_entry_ut TO -1.
  SET _vtol_prev_vel_hold_active TO FALSE.
  SET _vtol_prev_pos_hold_active TO FALSE.
  SET _vtol_prev_khv_active TO FALSE.
}
