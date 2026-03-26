@LAZYGLOBAL OFF.

// ============================================================
// export_ifc_nav_for_web.ks
//
// Exports IFC navigation data for the kerbin_map web viewer.
//
// Outputs:
//   0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_beacons.csv
//   0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_plates.csv
// ============================================================

GLOBAL IFC_SKIP_INTERACTIVE IS TRUE.

FUNCTION _CSV_SAN {
  PARAMETER val.
  LOCAL s IS "" + val.
  SET s TO s:REPLACE(",", ";").
  RETURN s.
}

FUNCTION _ENSURE_DIRS {
  IF NOT EXISTS("0:/Integrated Flight Computer/tools") {
    CREATEDIR("0:/Integrated Flight Computer/tools").
  }
  IF NOT EXISTS("0:/Integrated Flight Computer/tools/kerbin_map") {
    CREATEDIR("0:/Integrated Flight Computer/tools/kerbin_map").
  }
  IF NOT EXISTS("0:/Integrated Flight Computer/tools/kerbin_map/web") {
    CREATEDIR("0:/Integrated Flight Computer/tools/kerbin_map/web").
  }
  IF NOT EXISTS("0:/Integrated Flight Computer/tools/kerbin_map/web/data") {
    CREATEDIR("0:/Integrated Flight Computer/tools/kerbin_map/web/data").
  }
}

FUNCTION _EXPORT_BEACONS {
  PARAMETER out_path.
  IF EXISTS(out_path) { DELETEPATH(out_path). }
  LOG "id,type,lat,lon,alt_asl,name,rwy,hdg,gs_angle" TO out_path.

  LOCAL count IS 0.
  FOR id IN NAV_BEACON_DB:KEYS {
    LOCAL b IS NAV_BEACON_DB[id].
    LOCAL ll IS b["ll"].
    LOCAL name IS id.
    LOCAL rwy IS "".
    LOCAL hdg IS 0.
    LOCAL gs_angle IS 0.
    IF b:HASKEY("name")     { SET name     TO b["name"]. }
    IF b:HASKEY("runway")   { SET rwy      TO b["runway"]. }
    IF b:HASKEY("rwy")      { SET rwy      TO b["rwy"]. }
    IF b:HASKEY("hdg")      { SET hdg      TO b["hdg"]. }
    IF b:HASKEY("gs_angle") { SET gs_angle TO b["gs_angle"]. }

    LOG _CSV_SAN(id) + ","
      + _CSV_SAN(b["type"]) + ","
      + ROUND(ll:LAT, 7) + ","
      + ROUND(ll:LNG, 7) + ","
      + ROUND(b["alt_asl"], 2) + ","
      + _CSV_SAN(name) + ","
      + _CSV_SAN(rwy) + ","
      + ROUND(hdg, 4) + ","
      + ROUND(gs_angle, 4) TO out_path.

    SET count TO count + 1.
  }
  RETURN count.
}

FUNCTION _EXPORT_PLATES {
  PARAMETER out_path.
  IF EXISTS(out_path) { DELETEPATH(out_path). }
  LOG "plate_id,sequence,beacon_id,vapp,ils_id" TO out_path.

  LOCAL count IS 0.
  FOR plate_id IN PLATE_IDS {
    LOCAL plate IS GET_PLATE(plate_id).
    IF plate = 0 { CONTINUE. }

    LOCAL fixes  IS plate["fixes"].
    LOCAL ils_id IS plate["ils_id"].
    LOCAL seq IS 0.
    UNTIL seq >= fixes:LENGTH {
      LOG _CSV_SAN(plate_id) + ","
        + ROUND(seq, 0) + ","
        + _CSV_SAN(fixes[seq]) + ","
        + ROUND(plate["vapp"], 3) + ","
        + _CSV_SAN(ils_id) TO out_path.
      SET seq TO seq + 1.
      SET count TO count + 1.
    }
  }
  RETURN count.
}

FUNCTION RUN_EXPORT_IFC_NAV_FOR_WEB {
  PRINT "IFC nav export -> web CSV".
  LOCAL ifc_main IS "0:/Integrated Flight Computer/ifc_main.ks".
  IF NOT EXISTS(ifc_main) {
    PRINT "ERROR: missing " + ifc_main.
    RETURN.
  }

  RUNONCEPATH(ifc_main).
  _ENSURE_DIRS().

  LOCAL beacons_out IS "0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_beacons.csv".
  LOCAL plates_out  IS "0:/Integrated Flight Computer/tools/kerbin_map/web/data/ifc_plates.csv".

  LOCAL b_count IS _EXPORT_BEACONS(beacons_out).
  LOCAL p_count IS _EXPORT_PLATES(plates_out).

  PRINT "Beacon rows: " + b_count.
  PRINT "Plate rows:  " + p_count.
  PRINT "Wrote: " + beacons_out.
  PRINT "Wrote: " + plates_out.
}

RUN_EXPORT_IFC_NAV_FOR_WEB().
