@LAZYGLOBAL OFF.

// ============================================================
// ifc_aa.ks  -  Integrated Flight Computer
// AtmosphereAutopilot (kOS-AA) interface.
//
// Wraps all ADDONS:AA calls behind availability guards so the
// rest of the IFC never needs to check AA_AVAILABLE itself.
// ============================================================

// Resolve a valid AA addon handle.
// Returns 0 when AA is unavailable or not fully initialised.
FUNCTION _AA_GET_HANDLE {
  IF NOT ADDONS:AVAILABLE("AA") { RETURN 0. }
  IF NOT ADDONS:HASSUFFIX("AA") { RETURN 0. }
  LOCAL aa_handle IS ADDONS:AA.
  IF aa_handle = 0 { RETURN 0. }
  RETURN aa_handle.
}

// Call once at startup.  Detects AA and FAR, enables FBW with
// the limits defined in ifc_constants.ks.
FUNCTION AA_INIT {
  LOCAL aa IS _AA_GET_HANDLE().
  LOCAL aa_tries IS 0.
  // Startup race guard: AA can report available before its handle is ready.
  UNTIL aa <> 0 OR aa_tries >= 20 {
    WAIT 0.
    SET aa TO _AA_GET_HANDLE().
    SET aa_tries TO aa_tries + 1.
  }
  SET AA_AVAILABLE  TO aa <> 0.
  SET FAR_AVAILABLE TO ADDONS:AVAILABLE("FAR").

  // Allow kOS to take control even if another autopilot (SAS etc.) was active.
  SET CONFIG:SUPPRESSAUTOPILOT TO FALSE.

  IF AA_AVAILABLE {
    // Enable Fly-By-Wire for stability augmentation.
    SET aa:FBW TO TRUE.
    SET AA_FBW_ON TO TRUE.

    // Resolve moderator limits: aircraft config overrides global constants.
    LOCAL lim_aoa      IS AA_MAX_AOA.
    LOCAL lim_g        IS AA_MAX_G.
    LOCAL lim_sideslip IS AA_MAX_SIDESLIP.
    LOCAL lim_side_g   IS AA_MAX_SIDE_G.
    IF ACTIVE_AIRCRAFT <> 0 {
      IF ACTIVE_AIRCRAFT:HASKEY("aa_max_aoa")      AND ACTIVE_AIRCRAFT["aa_max_aoa"]      > 0 { SET lim_aoa      TO ACTIVE_AIRCRAFT["aa_max_aoa"].      }
      IF ACTIVE_AIRCRAFT:HASKEY("aa_max_g")        AND ACTIVE_AIRCRAFT["aa_max_g"]        > 0 { SET lim_g        TO ACTIVE_AIRCRAFT["aa_max_g"].        }
      IF ACTIVE_AIRCRAFT:HASKEY("aa_max_sideslip") AND ACTIVE_AIRCRAFT["aa_max_sideslip"] > 0 { SET lim_sideslip TO ACTIVE_AIRCRAFT["aa_max_sideslip"]. }
      IF ACTIVE_AIRCRAFT:HASKEY("aa_max_side_g")   AND ACTIVE_AIRCRAFT["aa_max_side_g"]   > 0 { SET lim_side_g   TO ACTIVE_AIRCRAFT["aa_max_side_g"].   }
    }

    // Apply structural / comfort limits.
    IF aa:HASSUFFIX("MODERATEAOA")     { SET aa:MODERATEAOA     TO TRUE. }
    IF aa:HASSUFFIX("MAXAOA")          { SET aa:MAXAOA          TO lim_aoa. }
    IF aa:HASSUFFIX("MODERATEG")       { SET aa:MODERATEG       TO TRUE. }
    IF aa:HASSUFFIX("MAXG")            { SET aa:MAXG            TO lim_g. }
    IF aa:HASSUFFIX("MODERATESIDESLIP"){ SET aa:MODERATESIDESLIP TO TRUE. }
    IF aa:HASSUFFIX("MAXSIDESLIP")     { SET aa:MAXSIDESLIP     TO lim_sideslip. }
    IF aa:HASSUFFIX("MODERATESIDEG")   { SET aa:MODERATESIDEG   TO TRUE. }
    IF aa:HASSUFFIX("MAXSIDEG")        { SET aa:MAXSIDEG        TO lim_side_g. }
    IF aa:HASSUFFIX("COORDTURN")       { SET aa:COORDTURN       TO TRUE. }

    // Attempt to set bank angle limit (not supported in all AA versions; safe no-op).
    LOCAL lim_bank IS AA_MAX_BANK.
    IF ACTIVE_AIRCRAFT <> 0 {
      IF ACTIVE_AIRCRAFT:HASKEY("aa_max_bank") AND ACTIVE_AIRCRAFT["aa_max_bank"] > 0 {
        SET lim_bank TO ACTIVE_AIRCRAFT["aa_max_bank"].
      }
    }
    IF aa:HASSUFFIX("MAXROLL") { SET aa:MAXROLL TO lim_bank. }
    IF aa:HASSUFFIX("MAXBANK") { SET aa:MAXBANK TO lim_bank. }

    LOCAL ready_str IS "".
    IF aa_tries > 0 { SET ready_str TO " (+" + aa_tries + "t)". }
    SET IFC_ALERT_TEXT TO "AA: FBW ok" + ready_str + "  AoA<=" + lim_aoa + "  G<=" + lim_g.
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  } ELSE {
    SET IFC_ALERT_TEXT TO "AA: unavailable - kOS steering fallback".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
  }
}

// Point the AA Director at (heading_deg, fpa_deg).
// fpa_deg is the flight path angle: negative = descending.
// If AA is unavailable, falls back to kOS LOCK STEERING.
FUNCTION AA_SET_DIRECTOR {
  PARAMETER hdg_deg, fpa_deg.

  // AA Director controls nose attitude, not flight path angle.
  // To achieve the desired FPA, command pitch = FPA + AoA so that
  // the nose is pointed high enough for the velocity vector to match fpa_deg.
  // GET_AOA() returns 0 when FAR is unavailable (graceful fallback).
  LOCAL pitch_cmd IS fpa_deg + GET_AOA().

  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 {
    SET AA_AVAILABLE TO FALSE.
    // AA unavailable: fall back to kOS LOCK STEERING.
    // Lock to the global so the expression evaluated each physics tick is trivial.
    SET IFC_DESIRED_STEERING TO HEADING(hdg_deg, pitch_cmd).
    LOCK STEERING TO IFC_DESIRED_STEERING.
    RETURN.
  }
  SET AA_AVAILABLE TO TRUE.
  // AA is handling the controls — release kOS LOCK STEERING so the two
  // control writers do not fight each other and cancel out roll commands.
  UNLOCK STEERING.

  LOCAL dir_vec IS HEADING(hdg_deg, pitch_cmd):VECTOR.
  SET TELEM_AA_DIR_VX TO dir_vec:X.
  SET TELEM_AA_DIR_VY TO dir_vec:Y.
  SET TELEM_AA_DIR_VZ TO dir_vec:Z.
  // Compute the actual heading and pitch this vector represents (same formula as
  // TELEM_COMPASS_HDG / TELEM_PITCH_DEG in ifc_main.ks) so we can verify the
  // vector matches what we commanded.
  LOCAL _up   IS IFC_UP_VEC.
  LOCAL _nrth IS IFC_NORTH_VEC.
  LOCAL _est  IS VCRS(_up, _nrth).
  SET TELEM_AA_DIR_PITCH_DEG TO 90 - VECTORANGLE(dir_vec, _up).
  SET TELEM_AA_DIR_HDG_DEG   TO MOD(ARCTAN2(VDOT(dir_vec, _est), VDOT(dir_vec, _nrth)) + 360, 360).
  // Mirror the command into IFC_DESIRED_STEERING for telemetry display only;
  // this variable is NOT locked to STEERING while AA is active.
  SET IFC_DESIRED_STEERING TO HEADING(hdg_deg, pitch_cmd).
  SET aa:DIRECTION TO dir_vec.

  // Ensure Director mode is active; leave FBW running for inner-loop stability.
  IF aa:HASSUFFIX("CRUISE")   AND aa:CRUISE   { SET aa:CRUISE   TO FALSE. }
  IF aa:HASSUFFIX("DIRECTOR") AND NOT aa:DIRECTOR {
    SET aa:DIRECTOR TO TRUE.
    SET AA_DIRECTOR_ON TO TRUE.
  }
}

// Switch to AA Cruise mode (FPA + heading hold).
// Used if you want AA to own the vertical profile instead of the Director.
FUNCTION AA_SET_CRUISE {
  PARAMETER hdg_deg, fpa_deg.
  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 {
    SET AA_AVAILABLE TO FALSE.
    RETURN.
  }
  SET AA_AVAILABLE TO TRUE.

  SET aa:HEADING  TO hdg_deg.
  SET aa:FPANGLE  TO fpa_deg.

  IF aa:HASSUFFIX("DIRECTOR") AND aa:DIRECTOR { SET aa:DIRECTOR TO FALSE. }
  IF aa:HASSUFFIX("FBW")      AND aa:FBW      { SET aa:FBW      TO FALSE. }
  IF aa:HASSUFFIX("CRUISE")   AND NOT aa:CRUISE {
    SET aa:CRUISE TO TRUE.
  }
}

// Turn off all AA modes cleanly.
FUNCTION AA_DISABLE_ALL {
  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 {
    SET AA_AVAILABLE TO FALSE.
    RETURN.
  }
  SET AA_AVAILABLE TO TRUE.
  IF aa:HASSUFFIX("CRUISE")   AND aa:CRUISE   { SET aa:CRUISE   TO FALSE. }
  IF aa:HASSUFFIX("DIRECTOR") AND aa:DIRECTOR { SET aa:DIRECTOR TO FALSE. }
  IF aa:HASSUFFIX("FBW")      AND aa:FBW      { SET aa:FBW      TO FALSE. }
  SET AA_DIRECTOR_ON TO FALSE.
  SET AA_FBW_ON      TO FALSE.
}

// Read actual FBW and Director states from the AA object into TELEM variables.
// Call once per loop cycle so the logger captures real mode state, not the
// tracked flags (AA_FBW_ON / AA_DIRECTOR_ON), which are not always updated.
FUNCTION AA_POLL_TELEM {
  IF NOT AA_AVAILABLE { RETURN. }
  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 { RETURN. }
  IF aa:HASSUFFIX("FBW")      { SET TELEM_AA_FBW_ON TO CHOOSE 1 IF aa:FBW      ELSE 0. }
  IF aa:HASSUFFIX("DIRECTOR") { SET TELEM_AA_DIR_ON TO CHOOSE 1 IF aa:DIRECTOR ELSE 0. }
}

// Re-assert all safety moderator flags.
// Called during preflight to guard against AA resetting its own state
// between AA_INIT and brake release (e.g. due to mode switches).
FUNCTION AA_REAPPLY_MODERATORS {
  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 { RETURN. }
  IF aa:HASSUFFIX("MODERATEAOA")     { SET aa:MODERATEAOA     TO TRUE. }
  IF aa:HASSUFFIX("MODERATEG")       { SET aa:MODERATEG       TO TRUE. }
  IF aa:HASSUFFIX("MODERATESIDESLIP"){ SET aa:MODERATESIDESLIP TO TRUE. }
  IF aa:HASSUFFIX("MODERATESIDEG")   { SET aa:MODERATESIDEG   TO TRUE. }
}

// Re-enable FBW after it was switched off (e.g. after Director was active).
FUNCTION AA_RESTORE_FBW {
  LOCAL aa IS _AA_GET_HANDLE().
  IF aa = 0 {
    SET AA_AVAILABLE TO FALSE.
    RETURN.
  }
  SET AA_AVAILABLE TO TRUE.
  IF aa:HASSUFFIX("DIRECTOR") AND aa:DIRECTOR { SET aa:DIRECTOR TO FALSE. }
  IF aa:HASSUFFIX("CRUISE")   AND aa:CRUISE   { SET aa:CRUISE   TO FALSE. }
  IF NOT aa:FBW { SET aa:FBW TO TRUE. }
  SET AA_FBW_ON TO TRUE.
  SET AA_DIRECTOR_ON TO FALSE.
}
