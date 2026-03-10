@LAZYGLOBAL OFF.

// ============================================================
// nav_beacons.ks  -  Integrated Flight Computer
//
// Navigation beacon database and approach plate definitions.
//
// KSC coordinates sourced from the NavInstruments mod by linuxgurugamer:
//   https://github.com/linuxgurugamer/NavInstruments
//   GameData/NavInstruments/PluginData/defaultRunways.cfg
//
// Coordinate convention:
//   lat  - degrees, negative = South
//   lng  - degrees, negative = West  (NavInstruments stores 0-360; subtract 360)
//   alt  - metres MSL
//
// KSC runway notes:
//   Runs roughly East-West near the equator.
//   RWY 09 threshold = western end  (landing heading 090.4°)
//   RWY 27 threshold = eastern end  (landing heading 270.4°)
//   The two ends have slightly different latitudes.
//   Runway elevation = 70 m MSL.
// ============================================================

// ----------------------------
// Beacon constructor
// ----------------------------
// type  : one of the BTYPE_* constants from ifc_constants.ks
// ll    : LATLNG object
// alt_asl : elevation in metres above mean sea level
// data    : LEXICON of type-specific extra fields
FUNCTION MAKE_BEACON {
  PARAMETER id, type, ll, alt_asl, data.
  LOCAL b IS LEXICON(
    "id",      id,
    "type",    type,
    "ll",      ll,
    "alt_asl", alt_asl
  ).
  FOR k IN data:KEYS { SET b[k] TO data[k]. }
  RETURN b.
}

// ----------------------------
// Approach plate constructor
// ----------------------------
// name        : human-readable string
// ils_id      : beacon ID of the ILS/LOC for this runway
// fixes       : LIST of beacon IDs in approach order  (IAF -> FAF -> ILS)
// vapp        : target approach speed in m/s
// alt_at      : LEXICON of beacon_id -> target crossing altitude (m MSL)
FUNCTION MAKE_PLATE {
  PARAMETER name, ils_id, fixes, vapp, alt_at.
  RETURN LEXICON(
    "name",   name,
    "ils_id", ils_id,
    "fixes",  fixes,
    "vapp",   vapp,
    "alt_at", alt_at
  ).
}

// ----------------------------
// Beacon registry
// ----------------------------
// Beacons are stored here; use GET_BEACON(id) to retrieve them.
GLOBAL NAV_BEACON_DB IS LEXICON().

FUNCTION REGISTER_BEACON {
  PARAMETER b.
  SET NAV_BEACON_DB[b["id"]] TO b.
}

FUNCTION GET_BEACON {
  PARAMETER id.
  IF NOT NAV_BEACON_DB:HASKEY(id) {
    PRINT "NAV ERROR: unknown beacon '" + id + "'".
    RETURN LEXICON().
  }
  RETURN NAV_BEACON_DB[id].
}

// ----------------------------
// KSC ILS beacons
// Coordinates from NavInstruments defaultRunways.cfg (gsLatitude/gsLongitude
// is the threshold/glideslope antenna position for each runway).
// Longitudes converted: NavInstruments 285.xxx - 360 = -74.xxx
// ----------------------------
LOCAL ksc_rwy09_thr IS LATLNG(-0.04877658, -74.70026463).  // 285.29973537 - 360
LOCAL ksc_rwy27_thr IS LATLNG(-0.05005861, -74.51751840).  // 285.48248160 - 360
LOCAL ksc_elev IS 70.  // altMSL from NavInstruments

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_09", BTYPE_ILS,
  ksc_rwy09_thr, ksc_elev,
  LEXICON("hdg", 90.4, "gs_angle", 3.0, "rwy", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_27", BTYPE_ILS,
  ksc_rwy27_thr, ksc_elev,
  LEXICON("hdg", 270.4, "gs_angle", 3.0, "rwy", "27")
)).

// ----------------------------
// KSC VOR/DME  (general area navigation)
// Positioned roughly at the Vehicle Assembly Building.
// ----------------------------
REGISTER_BEACON(MAKE_BEACON(
  "KSC_VOR", BTYPE_VOR,
  LATLNG(-0.097, -74.557), 70,
  LEXICON("name", "KSC Area VOR")
)).

// ----------------------------
// KSC Island Airstrip ILS beacons
// Also from NavInstruments defaultRunways.cfg (IS09 / IS27).
// Elevation 134 m MSL.  Heading ~089° / ~269°.
// ----------------------------
LOCAL isl_rwy09_thr IS LATLNG(-1.517254, -71.9515).  // 288.0485 - 360
LOCAL isl_rwy27_thr IS LATLNG(-1.516002, -71.8566).  // 288.1434 - 360
LOCAL isl_elev IS 134.

REGISTER_BEACON(MAKE_BEACON(
  "ISL_ILS_09", BTYPE_ILS,
  isl_rwy09_thr, isl_elev,
  LEXICON("hdg", 89.0, "gs_angle", 3.0, "rwy", "IS09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "ISL_ILS_27", BTYPE_ILS,
  isl_rwy27_thr, isl_elev,
  LEXICON("hdg", 269.0, "gs_angle", 3.0, "rwy", "IS27")
)).

// ----------------------------
// KSC RWY 09 approach fixes
// IAF-60 and IAF-30 are on the extended centreline west of the threshold.
// FAF is the glideslope intercept point ~8 km out.
// ----------------------------
// GEO_DESTINATION is available because nav_math.ks is loaded before this file.

LOCAL ksc09_iaf60_ll IS GEO_DESTINATION(ksc_rwy09_thr, 270, 60000).
LOCAL ksc09_iaf30_ll IS GEO_DESTINATION(ksc_rwy09_thr, 270, 30000).
LOCAL ksc09_faf_ll   IS GEO_DESTINATION(ksc_rwy09_thr, 270,  8000).

// FAF crossing altitude: on the 3° glideslope at 8 km = 69 + 8000*TAN(3°) ≈ 488 m
LOCAL ksc09_faf_alt IS ksc_elev + ROUND(8000 * TAN(3.0), 0).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09_60", BTYPE_IAF,
  ksc09_iaf60_ll, 3000,
  LEXICON("name", "KSC RWY09 IAF 60km", "runway", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09_30", BTYPE_IAF,
  ksc09_iaf30_ll, 1500,
  LEXICON("name", "KSC RWY09 IAF 30km", "runway", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_09", BTYPE_FAF,
  ksc09_faf_ll, ksc09_faf_alt,
  LEXICON("name", "KSC RWY09 FAF 8km", "runway", "09")
)).

// ----------------------------
// KSC RWY 27 approach fixes
// ----------------------------
LOCAL ksc27_iaf60_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 60000).
LOCAL ksc27_iaf30_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 30000).
LOCAL ksc27_faf_ll   IS GEO_DESTINATION(ksc_rwy27_thr, 90,  8000).

LOCAL ksc27_faf_alt IS ksc_elev + ROUND(8000 * TAN(3.0), 0).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27_60", BTYPE_IAF,
  ksc27_iaf60_ll, 3000,
  LEXICON("name", "KSC RWY27 IAF 60km", "runway", "27")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27_30", BTYPE_IAF,
  ksc27_iaf30_ll, 1500,
  LEXICON("name", "KSC RWY27 IAF 30km", "runway", "27")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_27", BTYPE_FAF,
  ksc27_faf_ll, ksc27_faf_alt,
  LEXICON("name", "KSC RWY27 FAF 8km", "runway", "27")
)).

// ----------------------------
// Approach plates
// ----------------------------
// Full ILS approach: fly IAF-60, IAF-30, FAF, then ILS tracking.
GLOBAL PLATE_KSC_ILS09 IS MAKE_PLATE(
  "KSC ILS RWY 09",
  "KSC_ILS_09",
  LIST("KSC_IAF_09_60", "KSC_IAF_09_30", "KSC_FAF_09"),
  75,   // Vapp m/s — override in aircraft config if needed
  LEXICON(
    "KSC_IAF_09_60", 3000,
    "KSC_IAF_09_30", 1500,
    "KSC_FAF_09",    ksc09_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27 IS MAKE_PLATE(
  "KSC ILS RWY 27",
  "KSC_ILS_27",
  LIST("KSC_IAF_27_60", "KSC_IAF_27_30", "KSC_FAF_27"),
  75,
  LEXICON(
    "KSC_IAF_27_60", 3000,
    "KSC_IAF_27_30", 1500,
    "KSC_FAF_27",    ksc27_faf_alt
  )
).

// Short approach: start from 30 km out (aircraft already in the area).
GLOBAL PLATE_KSC_ILS09_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 09 (short)",
  "KSC_ILS_09",
  LIST("KSC_IAF_09_30", "KSC_FAF_09"),
  75,
  LEXICON(
    "KSC_IAF_09_30", 1500,
    "KSC_FAF_09",    ksc09_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 27 (short)",
  "KSC_ILS_27",
  LIST("KSC_IAF_27_30", "KSC_FAF_27"),
  75,
  LEXICON(
    "KSC_IAF_27_30", 1500,
    "KSC_FAF_27",    ksc27_faf_alt
  )
).

// ----------------------------
// Approach plate lookup
// ----------------------------
FUNCTION GET_PLATE_FOR_RUNWAY {
  PARAMETER rwy_id, short_approach.
  IF rwy_id = "09" {
    IF short_approach { RETURN PLATE_KSC_ILS09_SHORT. }
    RETURN PLATE_KSC_ILS09.
  } ELSE IF rwy_id = "27" {
    IF short_approach { RETURN PLATE_KSC_ILS27_SHORT. }
    RETURN PLATE_KSC_ILS27.
  }
  PRINT "NAV ERROR: no plate for runway '" + rwy_id + "'".
  RETURN 0.
}
