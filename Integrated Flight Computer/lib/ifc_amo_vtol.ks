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
      // If MAXTHRUST is unavailable, record 0 and set all_maxthrust=FALSE —
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
  // Only valid when MAXTHRUST is available for every engine (jet engines,
  // or propeller engines at full RPM with reliable MAXTHRUST).
  // If any engine returned MAXTHRUST=0 (propeller/fan at idle/cold),
  // skip this entirely — all offsets start at 0 and adaptive trim
  // (_VTOL_ADAPT_TRIM) converges them in flight from observed pitch error.
  // Using THRUST fallback here is dangerous: THRUST reflects current output
  // under whatever limiters are already set, producing wildly wrong offsets
  // whenever engines are running at unequal power levels.

  SET ni TO 0.
  IF all_maxthrust {
    LOCAL tau_p IS 0.
    LOCAL tau_r IS 0.
    UNTIL ni >= raw_offsets:LENGTH {
      SET tau_p TO tau_p + raw_thrust[ni] * raw_offsets[ni]["lng"].
      SET tau_r TO tau_r + raw_thrust[ni] * raw_offsets[ni]["lat"].
      SET ni TO ni + 1.
    }

    // Pitch trim — sum of (T[i]*lng[i])^2 on the over-torque side.
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
  } ELSE {
    // MAXTHRUST unavailable — start all offsets at zero.
    // Adaptive trim will converge from observed pitch error in flight.
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

// ── Engine limit application ───────────────────────────────
// collective_in scales the torque-balanced base for all engines:
//   base[i] = collective × (1 + VTOL_TRIM_OFFSET[i])
// Multiplying by collective preserves the torque balance ratio at
// any throttle level — a 2:1 front/rear trim stays 2:1 at 50% collective.
// Roll/pitch differential is layered additively on top.
FUNCTION _VTOL_APPLY_ENGINES {
  PARAMETER collective_in, roll_cmd, pitch_cmd.
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL base IS collective_in * (1.0 + VTOL_TRIM_OFFSET[i]).
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
  LOCAL vs_kp  IS _AMO_CFG_NUM("vtol_vs_kp",  VTOL_VS_KP,  0).
  LOCAL vs_ki  IS _AMO_CFG_NUM("vtol_vs_ki",  VTOL_VS_KI,  0).
  LOCAL max_vs IS _AMO_CFG_NUM("vtol_max_vs", VTOL_MAX_VS, 0.1).

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
  LOCAL is_grounded IS (SHIP:STATUS = "LANDED" OR SHIP:STATUS = "PRELAUNCH").
  IF is_grounded {
    SET VTOL_VS_INTEGRAL TO 0.
    LOCAL vs_err_g IS VTOL_VS_CMD - SHIP:VERTICALSPEED.
    RETURN CLAMP(VTOL_HOVER_COLLECTIVE + vs_err_g * vs_kp, 0, 1).
  }

  LOCAL vs_err IS VTOL_VS_CMD - SHIP:VERTICALSPEED.

  // Anti-windup: only accumulate the integral when the unclamped output
  // is not already saturated in the same direction as the error.
  // This prevents the integrator from driving collective to its limit
  // during initial engine spool-up (aircraft on ground, can't achieve
  // commanded VS) and keeping it there after liftoff.
  LOCAL unclamped IS VTOL_HOVER_COLLECTIVE + vs_err * vs_kp + VTOL_VS_INTEGRAL * vs_ki.
  LOCAL at_ceil IS unclamped >= 1.0 AND vs_err > 0.
  LOCAL at_floor IS unclamped <= 0.0 AND vs_err < 0.
  IF NOT at_ceil AND NOT at_floor {
    SET VTOL_VS_INTEGRAL TO CLAMP(
      VTOL_VS_INTEGRAL + vs_err * IFC_ACTUAL_DT,
      -VTOL_VS_INTEGRAL_LIM,
       VTOL_VS_INTEGRAL_LIM
    ).
  }

  LOCAL collective IS CLAMP(unclamped, 0, 1).

  IF ABS(vs_err) < 0.5 {
    SET VTOL_HOVER_COLLECTIVE TO
      VTOL_HOVER_COLLECTIVE + (collective - VTOL_HOVER_COLLECTIVE) * VTOL_HOVER_LEARN_RATE.
  }

  RETURN collective.
}

// ── Adaptive trim integrator ───────────────────────────────
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
  IF VTOL_TRIM_OFFSET:LENGTH <> VTOL_ENG_LIST:LENGTH { RETURN. }

  // Freeze integrator while pilot is commanding pitch.
  LOCAL pilot_pitch_in IS 0.
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTPITCH") {
    SET pilot_pitch_in TO SHIP:CONTROL:PILOTPITCH.
  }
  IF ABS(pilot_pitch_in) > VTOL_TRIM_DEADBAND { RETURN. }

  // Current pitch: positive = nose up.
  LOCAL pitch_ang IS 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
  // Error: positive when nose is below level — need nose-up correction.
  LOCAL pitch_err IS CLAMP(-pitch_ang, -VTOL_TRIM_PITCH_CLAMP, VTOL_TRIM_PITCH_CLAMP).
  IF ABS(pitch_err) < 0.1 { RETURN. } // within 0.1° of level — done

  LOCAL rate IS VTOL_TRIM_RATE * IFC_ACTUAL_DT.

  LOCAL ai IS 0.
  UNTIL ai >= VTOL_ENG_LIST:LENGTH {
    LOCAL pmix IS VTOL_PITCH_MIX[ai].
    IF ABS(pmix) > 0.001 {
      LOCAL pmix_sign IS 1.
      IF pmix < 0 { SET pmix_sign TO -1. }
      // Nose-down error → positive pitch_err → nudge trim in nose-up direction.
      // Front engines (pmix > 0): positive delta → higher limit → more nose-up.
      // Rear  engines (pmix < 0): negative delta → lower  limit → less nose-down.
      LOCAL delta IS rate * pitch_err * pmix_sign.
      SET VTOL_TRIM_OFFSET[ai] TO CLAMP(VTOL_TRIM_OFFSET[ai] + delta, -0.95, 0.0).
    }
    SET ai TO ai + 1.
  }
}

// ── Public API ─────────────────────────────────────────────

// AMO / pre-arm tick.
// No LOCK THROTTLE — pilot's physical throttle is read as a VS command.
// 50 % throttle → 0 m/s VS command (hover).
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

  // ── Pilot inputs ────────────────────────────────────────────
  LOCAL inputs  IS _VTOL_PILOT_INPUTS().
  LOCAL p_roll  IS CLAMP(inputs["roll"],  -1, 1).
  LOCAL p_pitch IS CLAMP(inputs["pitch"], -1, 1).
  LOCAL p_yaw   IS CLAMP(inputs["yaw"],   -1, 1).
  LOCAL p_thr   IS 0.5.
  IF SHIP:HASSUFFIX("CONTROL") AND SHIP:CONTROL:HASSUFFIX("PILOTMAINTHROTTLE") {
    SET p_thr TO SHIP:CONTROL:PILOTMAINTHROTTLE.
  }

  // ── VS hold: pilot throttle → collective limiter level ─────
  // 50 % throttle → 0 m/s VS command.  PI controller integrates
  // to whatever collective keeps that VS. No LOCK THROTTLE.
  SET VTOL_COLLECTIVE TO _VTOL_VS_HOLD(p_thr).

  // ── Adaptive trim (slow pitch balance integrator) ───────────
  _VTOL_ADAPT_TRIM().

  // ── Wings-level auto-correction ─────────────────────────────
  // When the pilot is not commanding roll or pitch, apply a
  // proportional restoring command from current attitude.
  // Full authority at ~20° deviation with default gains.
  LOCAL roll_cmd  IS p_roll.
  LOCAL pitch_cmd IS p_pitch.

  // Angular rates from SHIP:ANGULARVEL (world frame, rad/s).
  // Dot with ship axis vectors to get component around each axis.
  // Nose-up pitch rate  = positive VDOT with STARVECTOR.
  // Roll-right roll rate = negative VDOT with FOREVECTOR.
  LOCAL ang_vel IS SHIP:ANGULARVEL.
  LOCAL pitch_rate_rads IS VDOT(ang_vel, SHIP:FACING:STARVECTOR).
  LOCAL roll_rate_rads  IS -VDOT(ang_vel, SHIP:FACING:FOREVECTOR).
  LOCAL deg_per_rad IS 180 / CONSTANT:PI.

  IF ABS(p_roll) < VTOL_TRIM_DEADBAND {
    LOCAL bank_ang IS ARCSIN(CLAMP(-VDOT(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR), -1, 1)).
    LOCAL roll_rate_degs IS roll_rate_rads * deg_per_rad.
    // KSP angular velocity sign convention: positive roll_rate_rads = rolling LEFT.
    // To damp a right-roll (positive bank): bank_ang > 0, roll_rate_degs > 0 while diverging.
    // KD sign must oppose the rate: +roll_rate_degs * KD adds to the left-roll correction.
    SET roll_cmd TO CLAMP(
      -bank_ang        * VTOL_LEVEL_ROLL_KP
      +roll_rate_degs  * VTOL_LEVEL_ROLL_KD,
      -1, 1).
  }
  IF ABS(p_pitch) < VTOL_TRIM_DEADBAND {
    LOCAL pitch_ang IS 90 - VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:VECTOR).
    LOCAL pitch_rate_degs IS pitch_rate_rads * deg_per_rad.
    // KSP angular velocity sign convention: positive pitch_rate_rads = pitching DOWN.
    // To damp a nose-up divergence (positive pitch_ang, negative pitch_rate_degs):
    // KD sign must oppose the rate: +pitch_rate_degs * KD adds to the nose-down correction.
    SET pitch_cmd TO CLAMP(
      -pitch_ang        * VTOL_LEVEL_PITCH_KP
      +pitch_rate_degs  * VTOL_LEVEL_PITCH_KD,
      -1, 1).
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
  SET VTOL_HOVER_COLLECTIVE        TO 0.50.
  SET VTOL_HOVER_COLLECTIVE_SEEDED TO FALSE.
  SET VTOL_COLLECTIVE              TO 0.
  SET VTOL_VS_CMD           TO 0.
  SET VTOL_VS_INTEGRAL      TO 0.
  SET VTOL_ALT_HOLD         TO FALSE.
  SET VTOL_ALT_CMD          TO 0.
}
