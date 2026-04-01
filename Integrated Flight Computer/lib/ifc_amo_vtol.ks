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
// Part tagging (KSP editor — part tags, not IR group names):
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

// ── Config helpers ────────────────────────────────────────
FUNCTION _VTOL_ENABLED {
  RETURN _AMO_CFG_BOOL("has_vtol", FALSE).
}

// ── Servo discovery via part tag ──────────────────────────
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

// ── Part offset from CoM ───────────────────────────────────
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

// ── Discovery ─────────────────────────────────────────────
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

  // Cache SHIP:ENGINES once — traversing the part tree is expensive.
  LOCAL ship_engs IS SHIP:ENGINES.

  // ── Collect engines by tag number ─────────────────────────
  LOCAL raw_offsets IS LIST().
  LOCAL raw_thrust  IS LIST(). // max thrust per engine (kN)
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
      // Fall back to actual THRUST when MAXTHRUST is unusable — the
      // thrust RATIO between engines is what matters for trim, and the
      // ratio is stable across the throttle range.
      LOCAL thr_val IS 0.
      LOCAL ei IS 0.
      UNTIL ei >= ship_engs:LENGTH {
        LOCAL cand IS ship_engs[ei].
        IF cand:UID = tp:UID {
          SET thr_val TO cand:MAXTHRUST.
          IF thr_val < 0.1 AND cand:HASSUFFIX("THRUST") {
            SET thr_val TO cand:THRUST.
          }
        }
        SET ei TO ei + 1.
      }
      raw_thrust:ADD(thr_val).
    }
    SET n TO n + 1.
  }

  // ── Collect servos by part tag ────────────────────────────
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

  // ── Thrust statistics ─────────────────────────────────────
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

  // ── Normalisation ranges (position only, for direction) ───
  LOCAL max_lat IS 0.01.
  LOCAL max_lng IS 0.01.
  LOCAL ni IS 0.
  UNTIL ni >= raw_offsets:LENGTH {
    IF ABS(raw_offsets[ni]["lat"]) > max_lat { SET max_lat TO ABS(raw_offsets[ni]["lat"]). }
    IF ABS(raw_offsets[ni]["lng"]) > max_lng { SET max_lng TO ABS(raw_offsets[ni]["lng"]). }
    SET ni TO ni + 1.
  }

  // ── Thrust-weighted mixing coefficients ───────────────────
  // Scale each engine's coefficient by T_mean/T[i] so a weaker engine
  // gets a proportionally larger limit change to produce the same torque
  // as a stronger engine.
  LOCAL roll_gain  IS _AMO_CFG_NUM("vtol_roll_gain",  VTOL_ROLL_GAIN,  0).
  LOCAL pitch_gain IS _AMO_CFG_NUM("vtol_pitch_gain", VTOL_PITCH_GAIN, 0).
  LOCAL lat_thresh IS VTOL_LAT_SIGN_THRESH.

  SET ni TO 0.
  UNTIL ni >= raw_offsets:LENGTH {
    LOCAL lat   IS raw_offsets[ni]["lat"].
    LOCAL lng   IS raw_offsets[ni]["lng"].
    LOCAL thr_i IS raw_thrust[ni].
    LOCAL t_scale IS 1.
    IF thr_i > 0.01 { SET t_scale TO t_mean / thr_i. }

    VTOL_ROLL_MIX:ADD(CLAMP(-lat / max_lat, -1, 1) * t_scale * roll_gain).
    VTOL_PITCH_MIX:ADD(CLAMP( lng / max_lng, -1, 1) * t_scale * pitch_gain).

    LOCAL lat_sign IS 0.
    IF lat < -lat_thresh      { SET lat_sign TO -1. }
    ELSE IF lat > lat_thresh  { SET lat_sign TO  1. }
    VTOL_YAW_SRV_MIX:ADD(lat_sign).

    VTOL_MAX_THRUST:ADD(thr_i).

    SET ni TO ni + 1.
  }

  // ── Auto-trim: compute per-engine base offsets that zero ──
  // the net pitch and roll torque at flat limits (all = 1.0).
  //
  // Net pitch torque at flat: tau_p = SUM(T[i] * lng[i])
  // Net roll  torque at flat: tau_r = SUM(T[i] * lat[i])
  //
  // Correction is applied only to the engines on the over-torque
  // side (so the other side stays at 1.0 = maximum lift).
  // Offset[i] = -tau_p * (T[i]*lng[i]) / SUM_same_sign(T[i]*lng[i])^2
  // This distributes the correction proportionally to each engine's
  // share of the imbalance.

  LOCAL tau_p IS 0.
  LOCAL tau_r IS 0.
  SET ni TO 0.
  UNTIL ni >= raw_offsets:LENGTH {
    SET tau_p TO tau_p + raw_thrust[ni] * raw_offsets[ni]["lng"].
    SET tau_r TO tau_r + raw_thrust[ni] * raw_offsets[ni]["lat"].
    SET ni TO ni + 1.
  }

  // Pitch trim
  // Sum of (T[i]*lng[i])^2 on the over-torque side (for proportional distribution).
  LOCAL denom_p IS 0.
  SET ni TO 0.
  UNTIL ni >= raw_offsets:LENGTH {
    LOCAL moment IS raw_thrust[ni] * raw_offsets[ni]["lng"].
    // Only correct engines whose moment is same-sign as the imbalance.
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

    // Clamp so base limit stays in (0, 1].
    LOCAL base IS CLAMP(1.0 + offset_p + offset_r, 0.05, 1.0).
    VTOL_TRIM_OFFSET:ADD(base - 1.0).

    SET ni TO ni + 1.
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

// ── Engine limit application ───────────────────────────────
// base[i] = 1.0 + VTOL_TRIM_OFFSET[i]   (auto-balanced at discovery)
// Pilot throttle controls overall thrust; limiters handle differential only.
FUNCTION _VTOL_APPLY_ENGINES {
  PARAMETER roll_cmd, pitch_cmd.
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL base IS 1.0 + VTOL_TRIM_OFFSET[i].
    LOCAL lim  IS base
               + roll_cmd  * VTOL_ROLL_MIX[i]
               + pitch_cmd * VTOL_PITCH_MIX[i].
    _AMO_SET_ENTRY_LIMIT_FRAC(VTOL_ENG_LIST[i], CLAMP(lim, 0, 1)).
    SET i TO i + 1.
  }
}

// ── Servo control via ModuleIRServo_v3 ─────────────────────
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

// ── Pilot inputs ───────────────────────────────────────────
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

// ── VS hold (PI controller) — used by autopilot tick only ──
FUNCTION _VTOL_VS_HOLD {
  PARAMETER thr_input.
  LOCAL vs_kp   IS _AMO_CFG_NUM("vtol_vs_kp",  VTOL_VS_KP,  0).
  LOCAL vs_ki   IS _AMO_CFG_NUM("vtol_vs_ki",  VTOL_VS_KI,  0).
  LOCAL max_vs  IS _AMO_CFG_NUM("vtol_max_vs", VTOL_MAX_VS, 0.1).

  SET VTOL_VS_CMD TO (thr_input - 0.5) * 2.0 * max_vs.

  IF VTOL_ALT_HOLD {
    LOCAL alt_kp  IS _AMO_CFG_NUM("vtol_alt_kp", VTOL_ALT_KP, 0).
    LOCAL alt_err IS VTOL_ALT_CMD - GET_AGL().
    SET VTOL_VS_CMD TO CLAMP(alt_err * alt_kp, -max_vs, max_vs).
  }

  LOCAL vs_err IS VTOL_VS_CMD - SHIP:VERTICALSPEED.
  SET VTOL_VS_INTEGRAL TO CLAMP(
    VTOL_VS_INTEGRAL + vs_err * IFC_ACTUAL_DT,
    -VTOL_VS_INTEGRAL_LIM,
     VTOL_VS_INTEGRAL_LIM
  ).

  LOCAL collective IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp + VTOL_VS_INTEGRAL * vs_ki.
  SET collective TO CLAMP(collective, 0, 1).

  IF ABS(vs_err) < 0.5 {
    SET VTOL_HOVER_COLLECTIVE TO
      VTOL_HOVER_COLLECTIVE + (collective - VTOL_HOVER_COLLECTIVE) * VTOL_HOVER_LEARN_RATE.
  }

  RETURN collective.
}

// ── Public API ─────────────────────────────────────────────

// AMO / pre-arm tick.
// Pilot controls physical throttle directly — kOS only adjusts limiters.
// No LOCK THROTTLE. No VS hold. Pure attitude stabilisation via differential limits.
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

  LOCAL inputs IS _VTOL_PILOT_INPUTS().
  _VTOL_APPLY_ENGINES(
    CLAMP(inputs["roll"],  -1, 1),
    CLAMP(inputs["pitch"], -1, 1)
  ).
  _VTOL_APPLY_SERVOS(CLAMP(inputs["yaw"], -1, 1)).
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

  // In autopilot mode, the collective shifts all base limits uniformly.
  // This overrides the simple 1.0+trim base used in VTOL_TICK_PREARM.
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL lim IS VTOL_COLLECTIVE
              + VTOL_TRIM_OFFSET[i]
              + CLAMP(roll_cmd,  -1, 1) * VTOL_ROLL_MIX[i]
              + CLAMP(pitch_cmd, -1, 1) * VTOL_PITCH_MIX[i].
    _AMO_SET_ENTRY_LIMIT_FRAC(VTOL_ENG_LIST[i], CLAMP(lim, 0, 1)).
    SET i TO i + 1.
  }
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
  SET VTOL_ACTIVE TO FALSE.
}

// Full state clear — use when re-arming or switching aircraft.
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
  SET VTOL_HOVER_COLLECTIVE TO 0.50.
  SET VTOL_COLLECTIVE       TO 0.
  SET VTOL_VS_CMD           TO 0.
  SET VTOL_VS_INTEGRAL      TO 0.
  SET VTOL_ALT_HOLD         TO FALSE.
  SET VTOL_ALT_CMD          TO 0.
}
