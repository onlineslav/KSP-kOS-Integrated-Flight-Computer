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
//
// Van's KSC notes:
//   The long parallel runway is exposed here as 09R/27R approach options.
//   Threshold coordinates were derived from Van's KSC static configs:
//     - VansKSC/Statics/VansKSC_center.cfg
//     - VansKSC/Statics/KK_2500m_RL_runway.cfg
//   using Kerbal Konstructs group/instance transform math.
//   These values may need minor local tuning if the pack updates.
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
    SET IFC_ALERT_TEXT TO "NAV: unknown beacon '" + id + "'".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN LEXICON().
  }
  RETURN NAV_BEACON_DB[id].
}

// ----------------------------
// Shared glideslope geometry
// ----------------------------
// All IAF fix altitudes are set BELOW_GS_M below the 3° glideslope at their
// fix distance.  This guarantees the aircraft always intercepts the GS from
// below, which is required for the pre-capture altitude-hold logic in
// phase_approach.ks (_RUN_ILS_TRACK).  FAF is placed exactly on the GS.
LOCAL GS_ANG   IS 3.0.                              // deg, shared by all approaches
LOCAL BELOW_GS_M IS 300.                            // m below GS at each IAF
LOCAL GS_HGT_15KM IS ROUND(15000 * TAN(GS_ANG), 0). // 787 m above field at FAF
LOCAL GS_HGT_30KM IS ROUND(30000 * TAN(GS_ANG), 0). // 1571 m above field at IAF-30
LOCAL GS_HGT_60KM IS ROUND(60000 * TAN(GS_ANG), 0). // 3144 m above field at IAF-60

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

// Van's KSC long-runway thresholds (09R/27R), derived from local KK statics.
LOCAL ksc_rwy09r_thr IS LATLNG(0.04285637, -75.31379059).
LOCAL ksc_rwy27r_thr IS LATLNG(0.04283534, -74.83633283).
LOCAL ksc_rwy_r_elev IS 93.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_09R", BTYPE_ILS,
  ksc_rwy09r_thr, ksc_rwy_r_elev,
  LEXICON("hdg", 90.0, "gs_angle", 3.0, "rwy", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_27R", BTYPE_ILS,
  ksc_rwy27r_thr, ksc_rwy_r_elev,
  LEXICON("hdg", 270.0, "gs_angle", 3.0, "rwy", "27R")
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
// Desert Airfield ILS beacons (Making History)
// NavInstruments makingHistoryRunways.cfg:
//   DAF 36 gs: lat -6.5825    lon 215.9594444 (-> -144.0405556)
//   DA18 18 gs: lat -6.4666667 lon 215.9611111 (-> -144.0388889)
// Elevation 822 m MSL.
// ----------------------------
LOCAL daf_rwy36_thr IS LATLNG(-6.5825,    -144.0405556).
LOCAL daf_rwy18_thr IS LATLNG(-6.4666667, -144.0388889).
LOCAL daf_elev IS 822.

REGISTER_BEACON(MAKE_BEACON(
  "DAF_ILS_36", BTYPE_ILS,
  daf_rwy36_thr, daf_elev,
  LEXICON("hdg", 0.4, "gs_angle", 3.0, "rwy", "DA36")
)).

REGISTER_BEACON(MAKE_BEACON(
  "DAF_ILS_18", BTYPE_ILS,
  daf_rwy18_thr, daf_elev,
  LEXICON("hdg", 180.4, "gs_angle", 3.0, "rwy", "DA18")
)).

// ----------------------------
// KSC Island Airstrip approach fixes
// Same geometry as KSC: IAF30 at 30 km, FAF at 8 km on extended centreline.
// Elevation 134 m MSL.
// ----------------------------
LOCAL isl09_iaf30_ll IS GEO_DESTINATION(isl_rwy09_thr, 270, 30000).
LOCAL isl09_faf_ll   IS GEO_DESTINATION(isl_rwy09_thr, 270, 15000).
LOCAL isl27_iaf30_ll IS GEO_DESTINATION(isl_rwy27_thr,  90, 30000).
LOCAL isl27_faf_ll   IS GEO_DESTINATION(isl_rwy27_thr,  90, 15000).
LOCAL isl_faf_alt   IS isl_elev + GS_HGT_15KM.              // on GS
LOCAL isl_iaf30_alt IS isl_elev + GS_HGT_30KM - BELOW_GS_M. // 300 m below GS

REGISTER_BEACON(MAKE_BEACON(
  "ISL_IAF_09_30", BTYPE_IAF,
  isl09_iaf30_ll, isl_iaf30_alt,
  LEXICON("name", "ISL RWY09 IAF 30km", "runway", "IS09")
)).
REGISTER_BEACON(MAKE_BEACON(
  "ISL_FAF_09", BTYPE_FAF,
  isl09_faf_ll, isl_faf_alt,
  LEXICON("name", "ISL RWY09 FAF 15km", "runway", "IS09")
)).
REGISTER_BEACON(MAKE_BEACON(
  "ISL_IAF_27_30", BTYPE_IAF,
  isl27_iaf30_ll, isl_iaf30_alt,
  LEXICON("name", "ISL RWY27 IAF 30km", "runway", "IS27")
)).
REGISTER_BEACON(MAKE_BEACON(
  "ISL_FAF_27", BTYPE_FAF,
  isl27_faf_ll, isl_faf_alt,
  LEXICON("name", "ISL RWY27 FAF 15km", "runway", "IS27")
)).

// ----------------------------
// ISL approach plates
// ----------------------------
GLOBAL PLATE_ISL_ILS09 IS MAKE_PLATE(
  "ISL ILS RWY IS09",
  "ISL_ILS_09",
  LIST("ISL_IAF_09_30", "ISL_FAF_09"),
  70,
  LEXICON(
    "ISL_IAF_09_30", isl_iaf30_alt,
    "ISL_FAF_09",    isl_faf_alt
  )
).
GLOBAL PLATE_ISL_ILS27 IS MAKE_PLATE(
  "ISL ILS RWY IS27",
  "ISL_ILS_27",
  LIST("ISL_IAF_27_30", "ISL_FAF_27"),
  70,
  LEXICON(
    "ISL_IAF_27_30", isl_iaf30_alt,
    "ISL_FAF_27",    isl_faf_alt
  )
).

// ----------------------------
// Desert Airfield approach fixes
// Geometry mirrors KSC setup: IAF-60, IAF-30, FAF-15.
// ----------------------------
LOCAL daf36_iaf60_ll IS GEO_DESTINATION(daf_rwy36_thr, 180, 60000).
LOCAL daf36_iaf30_ll IS GEO_DESTINATION(daf_rwy36_thr, 180, 30000).
LOCAL daf36_faf_ll   IS GEO_DESTINATION(daf_rwy36_thr, 180, 15000).
LOCAL daf18_iaf60_ll IS GEO_DESTINATION(daf_rwy18_thr,   0, 60000).
LOCAL daf18_iaf30_ll IS GEO_DESTINATION(daf_rwy18_thr,   0, 30000).
LOCAL daf18_faf_ll   IS GEO_DESTINATION(daf_rwy18_thr,   0, 15000).
LOCAL daf_faf_alt   IS daf_elev + GS_HGT_15KM.              // on GS:    1609 m MSL
LOCAL daf_iaf30_alt IS daf_elev + GS_HGT_30KM - BELOW_GS_M. // 300 m below GS: 2093 m MSL
LOCAL daf_iaf60_alt IS daf_elev + GS_HGT_60KM - BELOW_GS_M. // 300 m below GS: 3666 m MSL

REGISTER_BEACON(MAKE_BEACON(
  "DAF_IAF_36_60", BTYPE_IAF,
  daf36_iaf60_ll, daf_iaf60_alt,
  LEXICON("name", "DAF RWY36 IAF 60km", "runway", "DA36")
)).
REGISTER_BEACON(MAKE_BEACON(
  "DAF_IAF_36_30", BTYPE_IAF,
  daf36_iaf30_ll, daf_iaf30_alt,
  LEXICON("name", "DAF RWY36 IAF 30km", "runway", "DA36")
)).
REGISTER_BEACON(MAKE_BEACON(
  "DAF_FAF_36", BTYPE_FAF,
  daf36_faf_ll, daf_faf_alt,
  LEXICON("name", "DAF RWY36 FAF 15km", "runway", "DA36")
)).

REGISTER_BEACON(MAKE_BEACON(
  "DAF_IAF_18_60", BTYPE_IAF,
  daf18_iaf60_ll, daf_iaf60_alt,
  LEXICON("name", "DAF RWY18 IAF 60km", "runway", "DA18")
)).
REGISTER_BEACON(MAKE_BEACON(
  "DAF_IAF_18_30", BTYPE_IAF,
  daf18_iaf30_ll, daf_iaf30_alt,
  LEXICON("name", "DAF RWY18 IAF 30km", "runway", "DA18")
)).
REGISTER_BEACON(MAKE_BEACON(
  "DAF_FAF_18", BTYPE_FAF,
  daf18_faf_ll, daf_faf_alt,
  LEXICON("name", "DAF RWY18 FAF 15km", "runway", "DA18")
)).

// ----------------------------
// KSC RWY 09 approach fixes
// IAF-60 and IAF-30 are on the extended centreline west of the threshold.
// FAF is the glideslope intercept point ~8 km out.
// ----------------------------
// GEO_DESTINATION is available because nav_math.ks is loaded before this file.

LOCAL ksc09_iaf60_ll IS GEO_DESTINATION(ksc_rwy09_thr, 270, 60000).
LOCAL ksc09_iaf30_ll IS GEO_DESTINATION(ksc_rwy09_thr, 270, 30000).
LOCAL ksc09_faf_ll   IS GEO_DESTINATION(ksc_rwy09_thr, 270, 15000).

LOCAL ksc_faf_alt   IS ksc_elev + GS_HGT_15KM.              // on GS:    857 m MSL
LOCAL ksc_iaf30_alt IS ksc_elev + GS_HGT_30KM - BELOW_GS_M. // 300 m below GS: 1341 m MSL
LOCAL ksc_iaf60_alt IS ksc_elev + GS_HGT_60KM - BELOW_GS_M. // 300 m below GS: 2914 m MSL

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09_60", BTYPE_IAF,
  ksc09_iaf60_ll, ksc_iaf60_alt,
  LEXICON("name", "KSC RWY09 IAF 60km", "runway", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09_30", BTYPE_IAF,
  ksc09_iaf30_ll, ksc_iaf30_alt,
  LEXICON("name", "KSC RWY09 IAF 30km", "runway", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_09", BTYPE_FAF,
  ksc09_faf_ll, ksc_faf_alt,
  LEXICON("name", "KSC RWY09 FAF 15km", "runway", "09")
)).

// ----------------------------
// KSC RWY 27 approach fixes
// ----------------------------
LOCAL ksc27_iaf60_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 60000).
LOCAL ksc27_iaf30_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 30000).
LOCAL ksc27_faf_ll   IS GEO_DESTINATION(ksc_rwy27_thr, 90, 15000).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27_60", BTYPE_IAF,
  ksc27_iaf60_ll, ksc_iaf60_alt,
  LEXICON("name", "KSC RWY27 IAF 60km", "runway", "27")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27_30", BTYPE_IAF,
  ksc27_iaf30_ll, ksc_iaf30_alt,
  LEXICON("name", "KSC RWY27 IAF 30km", "runway", "27")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_27", BTYPE_FAF,
  ksc27_faf_ll, ksc_faf_alt,
  LEXICON("name", "KSC RWY27 FAF 15km", "runway", "27")
)).

// ----------------------------
// KSC RWY 09R / 27R approach fixes (Van's KSC long runway)
// ----------------------------
LOCAL ksc09r_iaf60_ll IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 60000).
LOCAL ksc09r_iaf30_ll IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 30000).
LOCAL ksc09r_faf_ll   IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 15000).
LOCAL ksc27r_iaf60_ll IS GEO_DESTINATION(ksc_rwy27r_thr,  90, 60000).
LOCAL ksc27r_iaf30_ll IS GEO_DESTINATION(ksc_rwy27r_thr,  90, 30000).
LOCAL ksc27r_faf_ll   IS GEO_DESTINATION(ksc_rwy27r_thr,  90, 15000).

LOCAL ksc_r_faf_alt   IS ksc_rwy_r_elev + GS_HGT_15KM.
LOCAL ksc_r_iaf30_alt IS ksc_rwy_r_elev + GS_HGT_30KM - BELOW_GS_M.
LOCAL ksc_r_iaf60_alt IS ksc_rwy_r_elev + GS_HGT_60KM - BELOW_GS_M.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09R_60", BTYPE_IAF,
  ksc09r_iaf60_ll, ksc_r_iaf60_alt,
  LEXICON("name", "KSC RWY09R IAF 60km", "runway", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09R_30", BTYPE_IAF,
  ksc09r_iaf30_ll, ksc_r_iaf30_alt,
  LEXICON("name", "KSC RWY09R IAF 30km", "runway", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_09R", BTYPE_FAF,
  ksc09r_faf_ll, ksc_r_faf_alt,
  LEXICON("name", "KSC RWY09R FAF 15km", "runway", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27R_60", BTYPE_IAF,
  ksc27r_iaf60_ll, ksc_r_iaf60_alt,
  LEXICON("name", "KSC RWY27R IAF 60km", "runway", "27R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27R_30", BTYPE_IAF,
  ksc27r_iaf30_ll, ksc_r_iaf30_alt,
  LEXICON("name", "KSC RWY27R IAF 30km", "runway", "27R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_27R", BTYPE_FAF,
  ksc27r_faf_ll, ksc_r_faf_alt,
  LEXICON("name", "KSC RWY27R FAF 15km", "runway", "27R")
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
    "KSC_IAF_09_60", ksc_iaf60_alt,
    "KSC_IAF_09_30", ksc_iaf30_alt,
    "KSC_FAF_09",    ksc_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27 IS MAKE_PLATE(
  "KSC ILS RWY 27",
  "KSC_ILS_27",
  LIST("KSC_IAF_27_60", "KSC_IAF_27_30", "KSC_FAF_27"),
  75,
  LEXICON(
    "KSC_IAF_27_60", ksc_iaf60_alt,
    "KSC_IAF_27_30", ksc_iaf30_alt,
    "KSC_FAF_27",    ksc_faf_alt
  )
).

// Short approach: start from 30 km out (aircraft already in the area).
GLOBAL PLATE_KSC_ILS09_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 09 (short)",
  "KSC_ILS_09",
  LIST("KSC_IAF_09_30", "KSC_FAF_09"),
  75,
  LEXICON(
    "KSC_IAF_09_30", ksc_iaf30_alt,
    "KSC_FAF_09",    ksc_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 27 (short)",
  "KSC_ILS_27",
  LIST("KSC_IAF_27_30", "KSC_FAF_27"),
  75,
  LEXICON(
    "KSC_IAF_27_30", ksc_iaf30_alt,
    "KSC_FAF_27",    ksc_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS09R IS MAKE_PLATE(
  "KSC ILS RWY 09R",
  "KSC_ILS_09R",
  LIST("KSC_IAF_09R_60", "KSC_IAF_09R_30", "KSC_FAF_09R"),
  75,
  LEXICON(
    "KSC_IAF_09R_60", ksc_r_iaf60_alt,
    "KSC_IAF_09R_30", ksc_r_iaf30_alt,
    "KSC_FAF_09R",    ksc_r_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27R IS MAKE_PLATE(
  "KSC ILS RWY 27R",
  "KSC_ILS_27R",
  LIST("KSC_IAF_27R_60", "KSC_IAF_27R_30", "KSC_FAF_27R"),
  75,
  LEXICON(
    "KSC_IAF_27R_60", ksc_r_iaf60_alt,
    "KSC_IAF_27R_30", ksc_r_iaf30_alt,
    "KSC_FAF_27R",    ksc_r_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS09R_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 09R (short)",
  "KSC_ILS_09R",
  LIST("KSC_IAF_09R_30", "KSC_FAF_09R"),
  75,
  LEXICON(
    "KSC_IAF_09R_30", ksc_r_iaf30_alt,
    "KSC_FAF_09R",    ksc_r_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS27R_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 27R (short)",
  "KSC_ILS_27R",
  LIST("KSC_IAF_27R_30", "KSC_FAF_27R"),
  75,
  LEXICON(
    "KSC_IAF_27R_30", ksc_r_iaf30_alt,
    "KSC_FAF_27R",    ksc_r_faf_alt
  )
).

GLOBAL PLATE_DAF_ILS36 IS MAKE_PLATE(
  "DAF ILS RWY 36",
  "DAF_ILS_36",
  LIST("DAF_IAF_36_60", "DAF_IAF_36_30", "DAF_FAF_36"),
  70,
  LEXICON(
    "DAF_IAF_36_60", daf_iaf60_alt,
    "DAF_IAF_36_30", daf_iaf30_alt,
    "DAF_FAF_36",    daf_faf_alt
  )
).

GLOBAL PLATE_DAF_ILS18 IS MAKE_PLATE(
  "DAF ILS RWY 18",
  "DAF_ILS_18",
  LIST("DAF_IAF_18_60", "DAF_IAF_18_30", "DAF_FAF_18"),
  70,
  LEXICON(
    "DAF_IAF_18_60", daf_iaf60_alt,
    "DAF_IAF_18_30", daf_iaf30_alt,
    "DAF_FAF_18",    daf_faf_alt
  )
).

GLOBAL PLATE_DAF_ILS36_SHORT IS MAKE_PLATE(
  "DAF ILS RWY 36 (short)",
  "DAF_ILS_36",
  LIST("DAF_IAF_36_30", "DAF_FAF_36"),
  70,
  LEXICON(
    "DAF_IAF_36_30", daf_iaf30_alt,
    "DAF_FAF_36",    daf_faf_alt
  )
).

GLOBAL PLATE_DAF_ILS18_SHORT IS MAKE_PLATE(
  "DAF ILS RWY 18 (short)",
  "DAF_ILS_18",
  LIST("DAF_IAF_18_30", "DAF_FAF_18"),
  70,
  LEXICON(
    "DAF_IAF_18_30", daf_iaf30_alt,
    "DAF_FAF_18",    daf_faf_alt
  )
).

// ----------------------------
// Plate registry  (for serialisable plan save/load)
// Maps string ID -> plate LEXICON so legs can store plate_id as a string.
// ----------------------------
GLOBAL PLATE_REGISTRY IS LEXICON().
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09",       PLATE_KSC_ILS09).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27",       PLATE_KSC_ILS27).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09_SHORT", PLATE_KSC_ILS09_SHORT).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27_SHORT", PLATE_KSC_ILS27_SHORT).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09R",       PLATE_KSC_ILS09R).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27R",       PLATE_KSC_ILS27R).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09R_SHORT", PLATE_KSC_ILS09R_SHORT).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27R_SHORT", PLATE_KSC_ILS27R_SHORT).
PLATE_REGISTRY:ADD("PLATE_ISL_ILS09",       PLATE_ISL_ILS09).
PLATE_REGISTRY:ADD("PLATE_ISL_ILS27",       PLATE_ISL_ILS27).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS36",       PLATE_DAF_ILS36).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS18",       PLATE_DAF_ILS18).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS36_SHORT", PLATE_DAF_ILS36_SHORT).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS18_SHORT", PLATE_DAF_ILS18_SHORT).

FUNCTION GET_PLATE {
  PARAMETER id.
  IF NOT PLATE_REGISTRY:HASKEY(id) {
    SET IFC_ALERT_TEXT TO "NAV: unknown plate '" + id + "'".
    SET IFC_ALERT_UT   TO TIME:SECONDS.
    RETURN 0.
  }
  RETURN PLATE_REGISTRY[id].
}

// ----------------------------
// Approach plate lookup
// ----------------------------
// Ordered list of plate IDs for FMS plate picker (A/D to cycle).
GLOBAL PLATE_IDS IS LIST(
  "PLATE_KSC_ILS09",
  "PLATE_KSC_ILS09_SHORT",
  "PLATE_KSC_ILS09R",
  "PLATE_KSC_ILS09R_SHORT",
  "PLATE_KSC_ILS27",
  "PLATE_KSC_ILS27_SHORT",
  "PLATE_KSC_ILS27R",
  "PLATE_KSC_ILS27R_SHORT",
  "PLATE_ISL_ILS09",
  "PLATE_ISL_ILS27",
  "PLATE_DAF_ILS36",
  "PLATE_DAF_ILS36_SHORT",
  "PLATE_DAF_ILS18",
  "PLATE_DAF_ILS18_SHORT"
).

FUNCTION GET_PLATE_FOR_RUNWAY {
  PARAMETER rwy_id, short_approach.
  IF rwy_id = "09" {
    IF short_approach { RETURN PLATE_KSC_ILS09_SHORT. }
    RETURN PLATE_KSC_ILS09.
  } ELSE IF rwy_id = "09R" {
    IF short_approach { RETURN PLATE_KSC_ILS09R_SHORT. }
    RETURN PLATE_KSC_ILS09R.
  } ELSE IF rwy_id = "27" {
    IF short_approach { RETURN PLATE_KSC_ILS27_SHORT. }
    RETURN PLATE_KSC_ILS27.
  } ELSE IF rwy_id = "27R" {
    IF short_approach { RETURN PLATE_KSC_ILS27R_SHORT. }
    RETURN PLATE_KSC_ILS27R.
  } ELSE IF rwy_id = "36" {
    IF short_approach { RETURN PLATE_DAF_ILS36_SHORT. }
    RETURN PLATE_DAF_ILS36.
  } ELSE IF rwy_id = "18" {
    IF short_approach { RETURN PLATE_DAF_ILS18_SHORT. }
    RETURN PLATE_DAF_ILS18.
  }
  SET IFC_ALERT_TEXT TO "NAV: no plate for runway '" + rwy_id + "'".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  RETURN 0.
}
