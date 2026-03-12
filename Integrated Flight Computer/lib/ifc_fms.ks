@LAZYGLOBAL OFF.

// ============================================================
// ifc_fms.ks  -  Integrated Flight Computer
// Save/load menu presets by slot.
// ============================================================

GLOBAL FMS_SLOT_MIN IS 1.
GLOBAL FMS_SLOT_MAX IS 5.
GLOBAL FMS_SLOT_PREFIX IS "0:/ifc_plan_".
GLOBAL FMS_SLOT_SUFFIX IS ".json".

FUNCTION _FMS_CLAMP_SLOT {
  PARAMETER slot.
  RETURN CLAMP(ROUND(slot, 0), FMS_SLOT_MIN, FMS_SLOT_MAX).
}

FUNCTION _FMS_PATH {
  PARAMETER slot.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  RETURN FMS_SLOT_PREFIX + s + FMS_SLOT_SUFFIX.
}

FUNCTION FMS_SAVE_PLAN {
  PARAMETER slot IS 1.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  LOCAL slot_path IS _FMS_PATH(s).

  LOCAL snapshot IS LEXICON(
    "name",  "IFC Plan Slot " + s,
    "slot",  s,
    "proc",  IFC_MENU_OPT_PROC,
    "rwy",   IFC_MENU_OPT_RWY,
    "dist",  IFC_MENU_OPT_DIST,
    "dest",  IFC_MENU_OPT_DEST,
    "v_r",   IFC_MENU_OPT_VR,
    "v2",    IFC_MENU_OPT_V2,
    "vapp",  IFC_MENU_OPT_VAPP
  ).
  WRITEJSON(snapshot, slot_path).
  IFC_SET_ALERT("Plan saved: slot " + s + " -> " + slot_path).
  RETURN TRUE.
}

FUNCTION FMS_LOAD_PLAN {
  PARAMETER slot IS 1.
  LOCAL s IS _FMS_CLAMP_SLOT(slot).
  LOCAL slot_path IS _FMS_PATH(s).

  IF NOT EXISTS(slot_path) {
    IFC_SET_ALERT("No saved plan in slot " + s, "WARN").
    RETURN FALSE.
  }

  LOCAL p IS READJSON(slot_path).
  IF p:HASKEY("proc")  { SET IFC_MENU_OPT_PROC TO p["proc"]. }
  IF p:HASKEY("rwy")   { SET IFC_MENU_OPT_RWY  TO p["rwy"]. }
  IF p:HASKEY("dist")  { SET IFC_MENU_OPT_DIST TO p["dist"]. }
  IF p:HASKEY("dest")  { SET IFC_MENU_OPT_DEST TO p["dest"]. }
  IF p:HASKEY("v_r")   { SET IFC_MENU_OPT_VR   TO MAX(0, ROUND(p["v_r"], 0)). }
  IF p:HASKEY("v2")    { SET IFC_MENU_OPT_V2   TO MAX(0, ROUND(p["v2"], 0)). }
  IF p:HASKEY("vapp")  { SET IFC_MENU_OPT_VAPP TO MAX(0, ROUND(p["vapp"], 0)). }

  SET IFC_MENU_OPT_PROC TO MOD(MAX(ROUND(IFC_MENU_OPT_PROC, 0), 0), 2).
  SET IFC_MENU_OPT_RWY  TO MOD(MAX(ROUND(IFC_MENU_OPT_RWY, 0), 0), 2).
  SET IFC_MENU_OPT_DIST TO MOD(MAX(ROUND(IFC_MENU_OPT_DIST, 0), 0), 2).
  SET IFC_MENU_OPT_DEST TO MOD(MAX(ROUND(IFC_MENU_OPT_DEST, 0), 0), 2).

  IFC_SET_ALERT("Plan loaded: slot " + s).
  RETURN TRUE.
}

FUNCTION FMS_SLOT_EXISTS {
  PARAMETER slot.
  RETURN EXISTS(_FMS_PATH(slot)).
}
