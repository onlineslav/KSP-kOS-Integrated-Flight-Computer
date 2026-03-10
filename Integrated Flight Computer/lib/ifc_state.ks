@LAZYGLOBAL OFF.

// ============================================================
// ifc_state.ks  -  Integrated Flight Computer
// All mutable runtime globals.  Reset via IFC_INIT_STATE().
// ============================================================

// ----------------------------
// Phase state
// ----------------------------
GLOBAL IFC_PHASE          IS PHASE_APPROACH.
GLOBAL IFC_SUBPHASE       IS SUBPHASE_FLY_TO_FIX.
GLOBAL IFC_PHASE_START_UT IS 0.
GLOBAL IFC_MISSION_START_UT IS 0.

// ----------------------------
// Active approach
// ----------------------------
// Set these before starting the approach loop.
// ACTIVE_PLATE is the approach plate lexicon from nav_beacons.ks.
// ACTIVE_AIRCRAFT is the aircraft config lexicon from an aircraft/*.ks file.
GLOBAL ACTIVE_PLATE    IS 0.  // set before run
GLOBAL ACTIVE_AIRCRAFT IS 0.  // set before run

// Derived from ACTIVE_PLATE at approach start (cached for speed):
GLOBAL ACTIVE_ILS_ID   IS "".
GLOBAL ACTIVE_RWY_HDG  IS 90.
GLOBAL ACTIVE_GS_ANGLE IS 3.0.
GLOBAL ACTIVE_THR_ALT  IS 69.
GLOBAL ACTIVE_V_APP    IS 80.
GLOBAL ACTIVE_FIXES    IS LIST().  // ordered list of beacon IDs
GLOBAL ACTIVE_ALT_AT   IS LEXICON(). // beacon_id -> target altitude (m)

// Index into ACTIVE_FIXES for FLY_TO_FIX sub-phase
GLOBAL FIX_INDEX IS 0.

// ----------------------------
// Throttle command
// ----------------------------
GLOBAL THROTTLE_CMD IS 0.
GLOBAL THR_INTEGRAL IS 0.   // accumulated speed error for PI autothrottle

// ----------------------------
// ILS deviation state  (updated each cycle in ILS_TRACK)
// ----------------------------
GLOBAL ILS_LOC_DEV  IS 0.   // m, + = right of centerline
GLOBAL ILS_GS_DEV   IS 0.   // m, + = above glideslope
GLOBAL ILS_DIST_M   IS 0.   // m  horizontal from threshold (+ = on approach)

// Previous-cycle values for derivative term
GLOBAL PREV_LOC_DEV IS 0.
GLOBAL PREV_GS_DEV  IS 0.

// ----------------------------
// AA state flags
// ----------------------------
GLOBAL AA_AVAILABLE    IS FALSE.
GLOBAL AA_DIRECTOR_ON  IS FALSE.
GLOBAL AA_FBW_ON       IS FALSE.

// ----------------------------
// FAR state flag
// ----------------------------
GLOBAL FAR_AVAILABLE IS FALSE.

// ----------------------------
// Telemetry
// ----------------------------
GLOBAL LAST_TELEM_UT IS 0.

// ----------------------------
// Flap deployment state
// ----------------------------
GLOBAL FLAPS_APPROACH_DEPLOYED IS FALSE.
GLOBAL FLAPS_LANDING_DEPLOYED  IS FALSE.

// ----------------------------
// Autoland phase state
// ----------------------------
GLOBAL FLARE_PITCH_CMD IS 0.   // current commanded pitch during flare (ramped up)

// ----------------------------
// Init / reset
// ----------------------------
FUNCTION IFC_INIT_STATE {
  SET IFC_PHASE          TO PHASE_APPROACH.
  SET IFC_SUBPHASE       TO SUBPHASE_FLY_TO_FIX.
  SET IFC_PHASE_START_UT TO TIME:SECONDS.
  SET IFC_MISSION_START_UT TO TIME:SECONDS.

  SET ACTIVE_ILS_ID   TO "".
  SET ACTIVE_RWY_HDG  TO 90.
  SET ACTIVE_GS_ANGLE TO 3.0.
  SET ACTIVE_THR_ALT  TO 69.
  SET ACTIVE_V_APP    TO 80.
  SET ACTIVE_FIXES    TO LIST().
  SET ACTIVE_ALT_AT   TO LEXICON().
  SET FIX_INDEX TO 0.

  SET THROTTLE_CMD TO 0.
  SET THR_INTEGRAL TO 0.

  SET ILS_LOC_DEV  TO 0.
  SET ILS_GS_DEV   TO 0.
  SET ILS_DIST_M   TO 0.
  SET PREV_LOC_DEV TO 0.
  SET PREV_GS_DEV  TO 0.

  SET AA_AVAILABLE   TO FALSE.
  SET AA_DIRECTOR_ON TO FALSE.
  SET AA_FBW_ON      TO FALSE.
  SET FAR_AVAILABLE  TO FALSE.

  SET LAST_TELEM_UT TO TIME:SECONDS.
  SET FLARE_PITCH_CMD TO 0.

  SET FLAPS_APPROACH_DEPLOYED TO FALSE.
  SET FLAPS_LANDING_DEPLOYED  TO FALSE.
}

// Called once after ACTIVE_PLATE and ACTIVE_AIRCRAFT are set.
FUNCTION IFC_LOAD_PLATE {
  SET ACTIVE_ILS_ID   TO ACTIVE_PLATE["ils_id"].
  SET ACTIVE_RWY_HDG  TO GET_BEACON(ACTIVE_ILS_ID)["hdg"].
  SET ACTIVE_GS_ANGLE TO GET_BEACON(ACTIVE_ILS_ID)["gs_angle"].
  SET ACTIVE_THR_ALT  TO GET_BEACON(ACTIVE_ILS_ID)["alt_asl"].
  SET ACTIVE_V_APP    TO ACTIVE_PLATE["vapp"].
  SET ACTIVE_FIXES    TO ACTIVE_PLATE["fixes"].
  SET ACTIVE_ALT_AT   TO ACTIVE_PLATE["alt_at"].
  SET FIX_INDEX TO 0.
}
