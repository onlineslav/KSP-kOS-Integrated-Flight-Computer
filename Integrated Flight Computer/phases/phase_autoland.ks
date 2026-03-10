@LAZYGLOBAL OFF.

// ============================================================
// phase_autoland.ks  -  Integrated Flight Computer
//
// Handles post-glideslope phases:
//   PHASE_FLARE      - arrest sink rate, idle throttle
//   PHASE_TOUCHDOWN  - detect ground contact, deploy brakes/spoilers
//   PHASE_ROLLOUT    - decelerate to a stop on runway heading
// ============================================================

// ── Public entry point ────────────────────────────────────
FUNCTION RUN_AUTOLAND {
  IF IFC_PHASE = PHASE_FLARE {
    _RUN_FLARE().
  } ELSE IF IFC_PHASE = PHASE_TOUCHDOWN {
    _RUN_TOUCHDOWN().
  } ELSE IF IFC_PHASE = PHASE_ROLLOUT {
    _RUN_ROLLOUT().
  }
}

// ─────────────────────────────────────────────────────────
// FLARE
// Sink-rate targeting with AoA ceiling.
//
// As AGL decreases from FLARE_ENTRY_AGL toward 0, the target
// vertical speed is linearly interpolated from the entry sink
// rate to TOUCHDOWN_VS (-0.3 m/s).  That target VS is
// converted to a flight path angle and fed to AA Director.
// If FAR AoA approaches MAX_FLARE_AOA the pitch-up is frozen.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLARE {
  LOCAL agl IS GET_AGL().
  LOCAL ias IS MAX(GET_IAS(), 10).  // floor prevents divide-by-zero at very low speed

  // Progress: 0 = just entered flare, 1 = at ground level.
  LOCAL frac IS CLAMP(1 - agl / FLARE_ENTRY_AGL, 0, 1).

  // Interpolate target sink rate from entry VS down to touchdown VS.
  LOCAL tgt_vs IS FLARE_ENTRY_VS + (TOUCHDOWN_VS - FLARE_ENTRY_VS) * frac.

  // Convert target VS to flight path angle (degrees).
  LOCAL tgt_fpa IS ARCTAN(tgt_vs / ias).

  // AoA ceiling: if FAR is available and AoA is near the limit,
  // stop pitching up (freeze tgt_fpa at current commanded value).
  IF FAR_AVAILABLE {
    LOCAL aoa IS GET_AOA().
    IF aoa > MAX_FLARE_AOA * 0.85 {
      SET tgt_fpa TO MIN(tgt_fpa, FLARE_PITCH_CMD).
    }
  }

  // Smooth the FPA transition at FLARE_PITCH_RATE deg/s.
  SET FLARE_PITCH_CMD TO MOVE_TOWARD(
    FLARE_PITCH_CMD, tgt_fpa,
    FLARE_PITCH_RATE * IFC_LOOP_DT
  ).

  AA_SET_DIRECTOR(ACTIVE_RWY_HDG, FLARE_PITCH_CMD).

  // Idle throttle during flare.
  SET THROTTLE_CMD TO 0.

  // Transition: gear contact detected.
  IF SHIP:STATUS = "LANDED" OR agl < TOUCHDOWN_AGL_M {
    SET_PHASE(PHASE_TOUCHDOWN).
  }
}

// ─────────────────────────────────────────────────────────
// TOUCHDOWN
// Deploy spoilers/reversers, brakes.  Shuts AA down cleanly
// and hands over to raw control inputs for the ground roll.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_TOUCHDOWN {
  SET THROTTLE_CMD TO 0.

  // Spoilers (action group from aircraft config).
  LOCAL ag_sp IS ACTIVE_AIRCRAFT["ag_spoilers"].
  IF ag_sp > 0 { TRIGGER_AG(ag_sp, TRUE). }

  // Reverse thrust (action group from aircraft config).
  LOCAL ag_tr IS ACTIVE_AIRCRAFT["ag_thrust_rev"].
  IF ag_tr > 0 { TRIGGER_AG(ag_tr, TRUE). }

  BRAKES ON.

  // Shut down AA — it fights gear loads on the ground.
  AA_DISABLE_ALL().
  UNLOCK STEERING.

  // Clear any accumulated control inputs from the flare.
  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.

  // Nosewheel steering for directional control.
  LOCK WHEELSTEERING TO ACTIVE_RWY_HDG.

  SET_PHASE(PHASE_ROLLOUT).
}

// ─────────────────────────────────────────────────────────
// ROLLOUT
// Raw control inputs — AA is not used on the ground.
//   SHIP:CONTROL:ROLL  → ailerons for wings level
//   SHIP:CONTROL:YAW   → rudder supplementing wheelsteering
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_ROLLOUT {
  BRAKES ON.
  LOCK WHEELSTEERING TO ACTIVE_RWY_HDG.
  SET THROTTLE_CMD TO 0.

  // Bank angle: positive = right wing down.
  // When level, STARVECTOR ⊥ UP → VECTORANGLE = 90 → bank = 0.
  LOCAL bank IS 90 - VECTORANGLE(SHIP:FACING:STARVECTOR, SHIP:UP:VECTOR).

  // Heading error: positive = pointing right of runway heading.
  LOCAL hdg_err IS WRAP_180(SHIP:HEADING - ACTIVE_RWY_HDG).

  // Aileron: roll left when right wing is down, and vice versa.
  SET SHIP:CONTROL:ROLL TO CLAMP(-bank * KP_ROLLOUT_ROLL, -1, 1).

  // Rudder: yaw left when heading is right of runway, and vice versa.
  SET SHIP:CONTROL:YAW  TO CLAMP(-hdg_err * KP_ROLLOUT_YAW,  -1, 1).

  IF GET_IAS() < ROLLOUT_DONE_IAS {
    BRAKES ON.
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    UNLOCK WHEELSTEERING.
    UNLOCK THROTTLE.
    SET_PHASE(PHASE_DONE).
  }
}
