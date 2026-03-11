@LAZYGLOBAL OFF.

// ============================================================
// ifc_fms.ks  -  Integrated Flight Computer
// Flight plan save / load helpers.
//
// Plans are stored as JSON on the kOS volume.  Each plan is a
// LEXICON with keys:
//   "name"  : human-readable string
//   "proc"  : IFC_MENU_OPT_PROC value  (0=approach, 1=takeoff)
//   "rwy"   : IFC_MENU_OPT_RWY value   (0=09, 1=27)
//   "dist"  : IFC_MENU_OPT_DIST value  (0=long, 1=short)
//   "dest"  : IFC_MENU_OPT_DEST value  (0=KSC, 1=Island)
//   "v_r"   : VR override in m/s
//   "v2"    : V2 override in m/s
//   "vapp"  : Vapp override in m/s
//
// Only the menu state is serialised.  The full leg list is
// always rebuilt from these options at ARM time, so plates and
// waypoints resolve correctly at runtime without embedding
// kOS objects in the JSON.
// ============================================================

GLOBAL FMS_SAVE_PATH IS "0:/ifc_plan.json".

// ── Save current menu options to disk ─────────────────────
FUNCTION FMS_SAVE_PLAN {
  LOCAL snapshot IS LEXICON(
    "name",  "IFC Plan",
    "proc",  IFC_MENU_OPT_PROC,
    "rwy",   IFC_MENU_OPT_RWY,
    "dist",  IFC_MENU_OPT_DIST,
    "dest",  IFC_MENU_OPT_DEST,
    "v_r",   IFC_MENU_OPT_VR,
    "v2",    IFC_MENU_OPT_V2,
    "vapp",  IFC_MENU_OPT_VAPP
  ).
  WRITEJSON(snapshot, FMS_SAVE_PATH).
  SET IFC_ALERT_TEXT TO "PLAN SAVED  →  " + FMS_SAVE_PATH.
  SET IFC_ALERT_UT   TO TIME:SECONDS.
}

// ── Load menu options from disk and apply them ────────────
// Returns TRUE on success, FALSE if no file exists.
FUNCTION FMS_LOAD_PLAN {
  IF NOT EXISTS(FMS_SAVE_PATH) {
    SET IFC_ALERT_TEXT TO "NO SAVED PLAN".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN FALSE.
  }
  LOCAL p IS READJSON(FMS_SAVE_PATH).
  IF p:HASKEY("proc")  { SET IFC_MENU_OPT_PROC TO p["proc"]. }
  IF p:HASKEY("rwy")   { SET IFC_MENU_OPT_RWY  TO p["rwy"].  }
  IF p:HASKEY("dist")  { SET IFC_MENU_OPT_DIST TO p["dist"]. }
  IF p:HASKEY("dest")  { SET IFC_MENU_OPT_DEST TO p["dest"]. }
  IF p:HASKEY("v_r")   { SET IFC_MENU_OPT_VR   TO p["v_r"].  }
  IF p:HASKEY("v2")    { SET IFC_MENU_OPT_V2   TO p["v2"].   }
  IF p:HASKEY("vapp")  { SET IFC_MENU_OPT_VAPP TO p["vapp"]. }
  SET IFC_ALERT_TEXT TO "PLAN LOADED".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  RETURN TRUE.
}
