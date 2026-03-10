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
// Pitch up to arrest the sink rate while cutting throttle.
// AA Director still drives attitude; we ramp FLARE_PITCH_CMD
// up at FLARE_PITCH_RATE deg/s toward the target flare pitch.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_FLARE {
  // Choose flare pitch target from aircraft config or global default.
  LOCAL tgt_pitch IS FLARE_PITCH.
  IF ACTIVE_AIRCRAFT["flare_pitch"] >= 0 { SET tgt_pitch TO ACTIVE_AIRCRAFT["flare_pitch"]. }

  // Ramp pitch up toward target.
  SET FLARE_PITCH_CMD TO MOVE_TOWARD(
    FLARE_PITCH_CMD, tgt_pitch,
    FLARE_PITCH_RATE * IFC_LOOP_DT
  ).

  // Hold runway heading, commanded pitch.
  AA_SET_DIRECTOR(ACTIVE_RWY_HDG, FLARE_PITCH_CMD).

  // Idle throttle during flare.
  SET THROTTLE_CMD TO 0.
  LOCK THROTTLE TO 0.

  // Transition: gear contact detected.
  // SHIP:STATUS becomes "LANDED" or "SPLASHED", or radar alt is very low.
  IF SHIP:STATUS = "LANDED" OR GET_AGL() < TOUCHDOWN_AGL_M {
    SET_PHASE(PHASE_TOUCHDOWN).
  }
}

// ─────────────────────────────────────────────────────────
// TOUCHDOWN
// Aircraft is on the ground.  Deploy spoilers/reversers,
// apply wheel brakes, switch to wheelsteering.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_TOUCHDOWN {
  // Throttle off.
  SET THROTTLE_CMD TO 0.
  LOCK THROTTLE TO 0.

  // Spoilers (action group from aircraft config).
  LOCAL ag_sp IS ACTIVE_AIRCRAFT["ag_spoilers"].
  IF ag_sp > 0 { TRIGGER_AG(ag_sp, TRUE). }

  // Reverse thrust (action group from aircraft config).
  LOCAL ag_tr IS ACTIVE_AIRCRAFT["ag_thrust_rev"].
  IF ag_tr > 0 { TRIGGER_AG(ag_tr, TRUE). }

  // Brakes.
  BRAKES ON.

  // Stop commanding attitude; hand off to wheelsteering for directional control.
  AA_DISABLE_ALL().
  UNLOCK STEERING.
  LOCK WHEELSTEERING TO ACTIVE_RWY_HDG.

  SET_PHASE(PHASE_ROLLOUT).
}

// ─────────────────────────────────────────────────────────
// ROLLOUT
// Stay on runway heading, full brakes, until speed is low
// enough to declare the landing complete.
// ─────────────────────────────────────────────────────────
FUNCTION _RUN_ROLLOUT {
  BRAKES ON.
  LOCK WHEELSTEERING TO ACTIVE_RWY_HDG.
  SET THROTTLE_CMD TO 0.

  IF GET_IAS() < ROLLOUT_DONE_IAS {
    BRAKES ON.
    UNLOCK WHEELSTEERING.
    UNLOCK THROTTLE.
    AA_DISABLE_ALL().
    SET_PHASE(PHASE_DONE).
  }
}
