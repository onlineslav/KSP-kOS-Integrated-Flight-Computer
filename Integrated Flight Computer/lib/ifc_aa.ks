@LAZYGLOBAL OFF.

// ============================================================
// ifc_aa.ks  -  Integrated Flight Computer
// AtmosphereAutopilot (kOS-AA) interface.
//
// Wraps all ADDONS:AA calls behind availability guards so the
// rest of the IFC never needs to check AA_AVAILABLE itself.
// ============================================================

// Call once at startup.  Detects AA and FAR, enables FBW with
// the limits defined in ifc_constants.ks.
FUNCTION AA_INIT {
  SET AA_AVAILABLE  TO ADDONS:AVAILABLE("AA").
  SET FAR_AVAILABLE TO ADDONS:AVAILABLE("FAR").

  // Allow kOS to take control even if another autopilot (SAS etc.) was active.
  SET CONFIG:SUPPRESSAUTOPILOT TO FALSE.

  IF AA_AVAILABLE {
    // Enable Fly-By-Wire for stability augmentation.
    SET ADDONS:AA:FBW TO TRUE.
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
    IF ADDONS:AA:HASSUFFIX("MODERATEAOA")     { SET ADDONS:AA:MODERATEAOA     TO TRUE. }
    IF ADDONS:AA:HASSUFFIX("MAXAOA")          { SET ADDONS:AA:MAXAOA          TO lim_aoa. }
    IF ADDONS:AA:HASSUFFIX("MODERATEG")       { SET ADDONS:AA:MODERATEG       TO TRUE. }
    IF ADDONS:AA:HASSUFFIX("MAXG")            { SET ADDONS:AA:MAXG            TO lim_g. }
    IF ADDONS:AA:HASSUFFIX("MODERATESIDESLIP"){ SET ADDONS:AA:MODERATESIDESLIP TO TRUE. }
    IF ADDONS:AA:HASSUFFIX("MAXSIDESLIP")     { SET ADDONS:AA:MAXSIDESLIP     TO lim_sideslip. }
    IF ADDONS:AA:HASSUFFIX("MODERATESIDEG")   { SET ADDONS:AA:MODERATESIDEG   TO TRUE. }
    IF ADDONS:AA:HASSUFFIX("MAXSIDEG")        { SET ADDONS:AA:MAXSIDEG        TO lim_side_g. }
    IF ADDONS:AA:HASSUFFIX("COORDTURN")       { SET ADDONS:AA:COORDTURN       TO TRUE. }

    PRINT "AA: FBW enabled  AoA<=" + lim_aoa + " G<=" + lim_g + " slip<=" + lim_sideslip + " sideG<=" + lim_side_g.
  } ELSE {
    PRINT "AA: addon not available - kOS steering fallback active.".
  }
}

// Point the AA Director at (heading_deg, fpa_deg).
// fpa_deg is the flight path angle: negative = descending.
// If AA is unavailable, falls back to kOS LOCK STEERING.
FUNCTION AA_SET_DIRECTOR {
  PARAMETER hdg_deg, fpa_deg.

  IF NOT AA_AVAILABLE {
    // Fallback: kOS built-in steering.
    LOCK STEERING TO HEADING(hdg_deg, fpa_deg).
    RETURN.
  }

  LOCAL dir_vec IS HEADING(hdg_deg, fpa_deg):VECTOR.
  SET ADDONS:AA:DIRECTION TO dir_vec.

  // Ensure Director mode is active, others off.
  IF ADDONS:AA:HASSUFFIX("CRUISE")   AND ADDONS:AA:CRUISE   { SET ADDONS:AA:CRUISE   TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("FBW")      AND ADDONS:AA:FBW      { SET ADDONS:AA:FBW      TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("DIRECTOR") AND NOT ADDONS:AA:DIRECTOR {
    SET ADDONS:AA:DIRECTOR TO TRUE.
    SET AA_DIRECTOR_ON TO TRUE.
  }
}

// Switch to AA Cruise mode (FPA + heading hold).
// Used if you want AA to own the vertical profile instead of the Director.
FUNCTION AA_SET_CRUISE {
  PARAMETER hdg_deg, fpa_deg.
  IF NOT AA_AVAILABLE { RETURN. }

  SET ADDONS:AA:HEADING  TO hdg_deg.
  SET ADDONS:AA:FPANGLE  TO fpa_deg.

  IF ADDONS:AA:HASSUFFIX("DIRECTOR") AND ADDONS:AA:DIRECTOR { SET ADDONS:AA:DIRECTOR TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("FBW")      AND ADDONS:AA:FBW      { SET ADDONS:AA:FBW      TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("CRUISE")   AND NOT ADDONS:AA:CRUISE {
    SET ADDONS:AA:CRUISE TO TRUE.
  }
}

// Turn off all AA modes cleanly.
FUNCTION AA_DISABLE_ALL {
  IF NOT AA_AVAILABLE { RETURN. }
  IF ADDONS:AA:HASSUFFIX("CRUISE")   AND ADDONS:AA:CRUISE   { SET ADDONS:AA:CRUISE   TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("DIRECTOR") AND ADDONS:AA:DIRECTOR { SET ADDONS:AA:DIRECTOR TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("FBW")      AND ADDONS:AA:FBW      { SET ADDONS:AA:FBW      TO FALSE. }
  SET AA_DIRECTOR_ON TO FALSE.
  SET AA_FBW_ON      TO FALSE.
}

// Re-enable FBW after it was switched off (e.g. after Director was active).
FUNCTION AA_RESTORE_FBW {
  IF NOT AA_AVAILABLE { RETURN. }
  IF ADDONS:AA:HASSUFFIX("DIRECTOR") AND ADDONS:AA:DIRECTOR { SET ADDONS:AA:DIRECTOR TO FALSE. }
  IF ADDONS:AA:HASSUFFIX("CRUISE")   AND ADDONS:AA:CRUISE   { SET ADDONS:AA:CRUISE   TO FALSE. }
  IF NOT ADDONS:AA:FBW { SET ADDONS:AA:FBW TO TRUE. }
  SET AA_FBW_ON TO TRUE.
  SET AA_DIRECTOR_ON TO FALSE.
}
