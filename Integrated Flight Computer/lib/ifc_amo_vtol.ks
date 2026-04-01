@LAZYGLOBAL OFF.

// ============================================================
// ifc_amo_vtol.ks  -  Integrated Flight Computer
//
// AMO VTOL Assist module
// ----------------------
// Tilt-rotor / tilt-pod VTOL guidance for AMO (manual) mode.
// Engines and IR servos discovered via numbered part tags.
// Geometry-based mixing matrix works for any N-pod layout.
//
// Physical throttle is locked at 1.0 while active.
// Per-engine thrust limits carry all collective/attitude control.
//
// NOTE: Does NOT use ADDONS:IR. Servos are controlled directly
// via ModuleIRServo_v3 SETFIELD("target position", angle).
// This works with Infernal Robotics v3 (meirumeiru) regardless
// of whether the kOS-IR addon is installed.
//
// Part tagging (KSP editor):
//   Engines: vtol_eng_1, vtol_eng_2 ... vtol_eng_N
//   Servos:  vtol_srv_1, vtol_srv_2 ... vtol_srv_N  (part tag, not IR group name)
//
// Pilot input mapping:
//   Main throttle  ->  VS command (50% = hold, 0% = max sink, 100% = max climb)
//   Pitch stick    ->  fore/aft differential thrust
//   Roll stick     ->  left/right differential thrust
//   Yaw stick      ->  differential servo tilt (wing pods toe in/out)
//
// Public API:
//   VTOL_DISCOVER()                    -- find engines + servos, compute geometry
//   VTOL_IR_DIAG()                     -- returns diagnostic string (servo count)
//   VTOL_TICK_PREARM()                 -- AMO tick; reads pilot inputs directly
//   VTOL_TICK(roll, pitch, yaw, thr)   -- autopilot tick; caller provides commands
//   VTOL_RELEASE()                     -- restore limits, stop servos, clear active
//   VTOL_RESET()                       -- full state clear
// ============================================================

// ── Config helpers ────────────────────────────────────────
// Reuse _AMO_CFG_* from ifc_amo.ks (loads before this file).

FUNCTION _VTOL_ENABLED {
  RETURN _AMO_CFG_BOOL("has_vtol", FALSE).
}

// ── Servo discovery via part tag ──────────────────────────
// Returns the ModuleIRServo_v3 from the part tagged tag_name, or 0.
FUNCTION _VTOL_FIND_SERVO_MODULE {
  PARAMETER tag_name.
  LOCAL tagged IS SHIP:PARTSTAGGED(tag_name).
  IF tagged:LENGTH = 0 { RETURN 0. }
  LOCAL tagged_part IS tagged[0].
  LOCAL smods IS tagged_part:MODULESNAMED("ModuleIRServo_v3").
  IF smods:LENGTH = 0 { RETURN 0. }
  RETURN smods[0].
}

// Returns a string describing servo availability on this vessel.
FUNCTION VTOL_IR_DIAG {
  LOCAL all_srv IS SHIP:MODULESNAMED("ModuleIRServo_v3").
  IF all_srv:LENGTH = 0 {
    RETURN "ModuleIRServo_v3: 0 found. Check IR v3 installed.".
  }
  RETURN "ModuleIRServo_v3: " + all_srv:LENGTH + " servo(s) on vessel.".
}

// ── Part offset from CoM ───────────────────────────────────
// Returns LEXICON("lat", lateral_m, "lng", long_m).
// lateral_m: + = starboard (right)
// long_m:    + = forward (nose direction)
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
  SET VTOL_DIFF_AVAILABLE TO FALSE.
  SET VTOL_SRV_AVAIL      TO FALSE.

  LOCAL eng_tag IS _AMO_CFG_STR("vtol_eng_tag_prefix",   "vtol_eng").
  LOCAL srv_tag IS _AMO_CFG_STR("vtol_srv_tag_prefix",   "vtol_srv").

  // ── Collect engines by tag number ─────────────────────────
  LOCAL raw_offsets IS LIST(). // parallel list: LEXICON("lat","lng")
  LOCAL n IS 1.
  UNTIL n > VTOL_MAX_PODS {
    LOCAL tagged IS SHIP:PARTSTAGGED(eng_tag + "_" + n).
    IF tagged:LENGTH = 0 { BREAK. }
    LOCAL tp IS tagged[0].
    LOCAL entry IS _AMO_MAKE_PART_ENTRY(tp).
    IF entry <> 0 {
      VTOL_ENG_LIST:ADD(entry).
      raw_offsets:ADD(_VTOL_PART_OFFSET(tp)).
    }
    SET n TO n + 1.
  }

  // ── Collect servos by part tag ────────────────────────────
  // One servo module per engine, same index numbering.
  LOCAL si IS 1.
  UNTIL si > VTOL_ENG_LIST:LENGTH {
    LOCAL srv IS _VTOL_FIND_SERVO_MODULE(srv_tag + "_" + si).
    VTOL_SRV_LIST:ADD(srv).
    IF srv <> 0 { SET VTOL_SRV_AVAIL TO TRUE. }
    SET si TO si + 1.
  }
  // Pad to same length as engine list (safety).
  UNTIL VTOL_SRV_LIST:LENGTH >= VTOL_ENG_LIST:LENGTH {
    VTOL_SRV_LIST:ADD(0).
  }

  IF VTOL_ENG_LIST:LENGTH < 2 {
    SET VTOL_DISCOVERED TO TRUE.
    IFC_SET_ALERT("VTOL: only " + VTOL_ENG_LIST:LENGTH + " engine(s) tagged -- need 2+", "WARN").
    RETURN.
  }

  // ── Compute normalisation ranges ──────────────────────────
  LOCAL max_lat IS 0.01. // guard against zero
  LOCAL max_lng IS 0.01.
  LOCAL i IS 0.
  UNTIL i >= raw_offsets:LENGTH {
    LOCAL pos IS raw_offsets[i].
    IF ABS(pos["lat"]) > max_lat { SET max_lat TO ABS(pos["lat"]). }
    IF ABS(pos["lng"]) > max_lng { SET max_lng TO ABS(pos["lng"]). }
    SET i TO i + 1.
  }

  // ── Build mixing coefficients ──────────────────────────────
  LOCAL roll_gain  IS _AMO_CFG_NUM("vtol_roll_gain",  VTOL_ROLL_GAIN,  0).
  LOCAL pitch_gain IS _AMO_CFG_NUM("vtol_pitch_gain", VTOL_PITCH_GAIN, 0).
  LOCAL lat_thresh IS VTOL_LAT_SIGN_THRESH.

  SET i TO 0.
  UNTIL i >= raw_offsets:LENGTH {
    LOCAL lat IS raw_offsets[i]["lat"].
    LOCAL lng IS raw_offsets[i]["lng"].

    // roll_mix:  -lat/max -> left engine gets + coefficient for roll-right cmd
    VTOL_ROLL_MIX:ADD(CLAMP(-lat / max_lat, -1, 1) * roll_gain).

    // pitch_mix: +lng/max -> front engine gets + coefficient for pitch-up cmd
    VTOL_PITCH_MIX:ADD(CLAMP(lng / max_lng, -1, 1) * pitch_gain).

    // yaw_srv_mix: +/-1 by side; centerline pods get 0
    LOCAL lat_sign IS 0.
    IF lat < -lat_thresh      { SET lat_sign TO -1. }
    ELSE IF lat > lat_thresh  { SET lat_sign TO  1. }
    VTOL_YAW_SRV_MIX:ADD(lat_sign).

    // trim_offset: 0 for MVP (symmetric CoM assumed)
    VTOL_TRIM_OFFSET:ADD(0).

    SET i TO i + 1.
  }

  SET VTOL_DIFF_AVAILABLE TO TRUE.
  SET VTOL_DISCOVERED     TO TRUE.

  LOCAL srv_count IS 0.
  SET i TO 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    IF VTOL_SRV_LIST[i] <> 0 { SET srv_count TO srv_count + 1. }
    SET i TO i + 1.
  }
  IFC_SET_ALERT(
    "VTOL: " + VTOL_ENG_LIST:LENGTH + " engines, " + srv_count + " servos found", "INFO"
  ).
}

// ── Engine limit application ───────────────────────────────
FUNCTION _VTOL_APPLY_ENGINES {
  PARAMETER roll_cmd, pitch_cmd, collective.
  LOCAL i IS 0.
  UNTIL i >= VTOL_ENG_LIST:LENGTH {
    LOCAL thr IS collective
              + roll_cmd  * VTOL_ROLL_MIX[i]
              + pitch_cmd * VTOL_PITCH_MIX[i]
              + VTOL_TRIM_OFFSET[i].
    _AMO_SET_ENTRY_LIMIT_FRAC(VTOL_ENG_LIST[i], CLAMP(thr, 0, 1)).
    SET i TO i + 1.
  }
}

// ── Servo control via ModuleIRServo_v3 ─────────────────────
// Sets "target position" and "max speed" directly on the part module.
// Does not require ADDONS:IR.
FUNCTION _VTOL_APPLY_SERVOS {
  PARAMETER yaw_cmd.
  IF NOT VTOL_SRV_AVAIL { RETURN. }
  LOCAL hover_angle IS _AMO_CFG_NUM("vtol_hover_angle", 90, 0).
  LOCAL yaw_gain    IS _AMO_CFG_NUM("vtol_yaw_gain", VTOL_YAW_SRV_GAIN, 0).
  LOCAL i IS 0.
  UNTIL i >= VTOL_SRV_LIST:LENGTH {
    LOCAL srv_mod IS VTOL_SRV_LIST[i].
    IF srv_mod <> 0 {
      LOCAL yaw_mix    IS VTOL_YAW_SRV_MIX[i].
      LOCAL tgt_angle  IS hover_angle + yaw_cmd * yaw_gain * yaw_mix.
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
      // Hold position by commanding current angle as target.
      LOCAL cur_pos IS srv_mod:GETFIELD("current position").
      srv_mod:SETFIELD("target position", cur_pos).
    }
    SET i TO i + 1.
  }
}

// ── Pilot inputs ───────────────────────────────────────────
// Returns LEXICON("roll","pitch","yaw","thr").
// thr = PILOTMAINTHROTTLE (0..1); used as VS command, not physical throttle.
FUNCTION _VTOL_PILOT_INPUTS {
  LOCAL roll_in  IS 0.
  LOCAL pitch_in IS 0.
  LOCAL yaw_in   IS 0.
  LOCAL thr_in   IS 0.5. // default = hover hold
  IF SHIP:HASSUFFIX("CONTROL") {
    LOCAL ctrl IS SHIP:CONTROL.
    IF ctrl:HASSUFFIX("PILOTROLL")          { SET roll_in  TO ctrl:PILOTROLL. }
    IF ctrl:HASSUFFIX("PILOTPITCH")         { SET pitch_in TO ctrl:PILOTPITCH. }
    IF ctrl:HASSUFFIX("PILOTYAW")           { SET yaw_in   TO ctrl:PILOTYAW. }
    IF ctrl:HASSUFFIX("PILOTMAINTHROTTLE")  { SET thr_in   TO ctrl:PILOTMAINTHROTTLE. }
  }
  RETURN LEXICON("roll", roll_in, "pitch", pitch_in, "yaw", yaw_in, "thr", thr_in).
}

// ── VS hold (PI controller) ────────────────────────────────
// thr_input: 0..1; 0.5 = hold, 0 = max sink, 1 = max climb.
// Returns collective command (0..1).
FUNCTION _VTOL_VS_HOLD {
  PARAMETER thr_input.
  LOCAL vs_kp   IS _AMO_CFG_NUM("vtol_vs_kp",  VTOL_VS_KP,  0).
  LOCAL vs_ki   IS _AMO_CFG_NUM("vtol_vs_ki",  VTOL_VS_KI,  0).
  LOCAL max_vs  IS _AMO_CFG_NUM("vtol_max_vs", VTOL_MAX_VS, 0.1).

  // Map throttle axis to VS command.
  SET VTOL_VS_CMD TO (thr_input - 0.5) * 2.0 * max_vs.

  // Altitude hold overrides VS command when armed.
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

  // Slowly track hover collective when VS error is small.
  IF ABS(vs_err) < 0.5 {
    SET VTOL_HOVER_COLLECTIVE TO
      VTOL_HOVER_COLLECTIVE + (collective - VTOL_HOVER_COLLECTIVE) * VTOL_HOVER_LEARN_RATE.
  }

  RETURN collective.
}

// ── Public API ─────────────────────────────────────────────

// AMO/pre-arm tick. Reads pilot inputs directly.
// Called from AMO_TICK_PREARM when has_vtol is TRUE.
// Caller is responsible for LOCK THROTTLE TO THROTTLE_CMD
// and setting THROTTLE_CMD to 1.0 before calling this.
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
  SET VTOL_COLLECTIVE TO _VTOL_VS_HOLD(inputs["thr"]).
  _VTOL_APPLY_ENGINES(
    CLAMP(inputs["roll"],  -1, 1),
    CLAMP(inputs["pitch"], -1, 1),
    VTOL_COLLECTIVE
  ).
  _VTOL_APPLY_SERVOS(CLAMP(inputs["yaw"], -1, 1)).
  SET VTOL_ACTIVE TO TRUE.
}

// Autopilot tick. Caller provides all commands.
// roll_cmd, pitch_cmd, yaw_cmd: -1..1
// thr_input: 0..1 (maps to VS command via VS hold controller)
// Caller is responsible for setting THROTTLE_CMD to 1.0.
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
    CLAMP(roll_cmd,  -1, 1),
    CLAMP(pitch_cmd, -1, 1),
    VTOL_COLLECTIVE
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
  SET VTOL_ACTIVE TO FALSE.
}

// Full state clear -- use when re-arming or switching aircraft.
FUNCTION VTOL_RESET {
  VTOL_RELEASE().
  VTOL_ENG_LIST:CLEAR().
  VTOL_SRV_LIST:CLEAR().
  VTOL_ROLL_MIX:CLEAR().
  VTOL_PITCH_MIX:CLEAR().
  VTOL_YAW_SRV_MIX:CLEAR().
  VTOL_TRIM_OFFSET:CLEAR().
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
