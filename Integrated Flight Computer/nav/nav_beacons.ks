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
//   RWY 27 threshold = eastern end  (landing heading 270.0°)
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
LOCAL GS_ANG_STEEP IS 5.0.                               // deg, terrain-clearance approaches
LOCAL GS4_HGT_15KM IS ROUND(15000 * TAN(GS_ANG_STEEP), 0).
LOCAL GS4_HGT_30KM IS ROUND(30000 * TAN(GS_ANG_STEEP), 0).
LOCAL GS4_HGT_60KM IS ROUND(60000 * TAN(GS_ANG_STEEP), 0).

// ----------------------------
// KSC ILS beacons
// Coordinates from NavInstruments defaultRunways.cfg (gsLatitude/gsLongitude
// is the threshold/glideslope antenna position for each runway).
// Longitudes converted: NavInstruments 285.xxx - 360 = -74.xxx
// Note: RWY 27L threshold/elevation was updated from in-game sampling
// for this Van's KSC install.
// ----------------------------
LOCAL ksc_rwy09_thr IS LATLNG(-0.04877658, -74.70026463).  // 285.29973537 - 360
LOCAL ksc_rwy09_elev IS 70.  // altMSL from NavInstruments
LOCAL ksc_rwy27_thr IS LATLNG(0.042795, -74.510267).
LOCAL ksc_rwy27l_elev IS 74.29.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_09", BTYPE_ILS,
  ksc_rwy09_thr, ksc_rwy09_elev,
  LEXICON("hdg", 90.4, "gs_angle", 3.0, "rwy", "09")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_27L", BTYPE_ILS,
  ksc_rwy27_thr, ksc_rwy27l_elev,
  LEXICON("hdg", 270.0, "gs_angle", GS_ANG, "rwy", "27L")
)).

// Van's KSC long-runway threshold (09R), derived from local KK statics.
LOCAL ksc_rwy09r_thr IS LATLNG(0.042762, -74.948327).
LOCAL ksc_rwy09r_elev IS 81.27.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_ILS_09R", BTYPE_ILS,
  ksc_rwy09r_thr, ksc_rwy09r_elev,
  LEXICON("hdg", 90.0, "gs_angle", GS_ANG_STEEP, "rwy", "09R")
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
// Polar Research Alpha ILS beacon (KerbinSide Remastered via NavInstruments)
// NavInstruments ModuleManagerCfgs/KerbinSideRemastered.cfg:
//   Polar 34 shortID POR34 hdg 337
//   gsLatitude 72.5169754, gsLongitude -78.4949417, altMSL 31.5
// ----------------------------
LOCAL pol_rwy34_thr IS LATLNG(72.5169754, -78.4949417).
LOCAL pol_rwy34_elev IS 31.5.

REGISTER_BEACON(MAKE_BEACON(
  "POL_ILS_34", BTYPE_ILS,
  pol_rwy34_thr, pol_rwy34_elev,
  LEXICON("hdg", 337.0, "gs_angle", 3.0, "rwy", "POR34")
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
// Polar RWY 34 approach fixes
// Inbound course 337, so fixes are placed on reciprocal bearing 157.
// Geometry mirrors KSC/DAF setup: IAF-60, IAF-30, FAF-15.
// ----------------------------
LOCAL pol34_iaf60_ll IS GEO_DESTINATION(pol_rwy34_thr, 157, 60000).
LOCAL pol34_iaf30_ll IS GEO_DESTINATION(pol_rwy34_thr, 157, 30000).
LOCAL pol34_faf_ll   IS GEO_DESTINATION(pol_rwy34_thr, 157, 15000).
LOCAL pol_faf_alt   IS pol_rwy34_elev + GS_HGT_15KM.
LOCAL pol_iaf30_alt IS pol_rwy34_elev + GS_HGT_30KM - BELOW_GS_M.
LOCAL pol_iaf60_alt IS pol_rwy34_elev + GS_HGT_60KM - BELOW_GS_M.

REGISTER_BEACON(MAKE_BEACON(
  "POL_IAF_34_60", BTYPE_IAF,
  pol34_iaf60_ll, pol_iaf60_alt,
  LEXICON("name", "POL RWY34 IAF 60km", "runway", "POR34")
)).
REGISTER_BEACON(MAKE_BEACON(
  "POL_IAF_34_30", BTYPE_IAF,
  pol34_iaf30_ll, pol_iaf30_alt,
  LEXICON("name", "POL RWY34 IAF 30km", "runway", "POR34")
)).
REGISTER_BEACON(MAKE_BEACON(
  "POL_FAF_34", BTYPE_FAF,
  pol34_faf_ll, pol_faf_alt,
  LEXICON("name", "POL RWY34 FAF 15km", "runway", "POR34")
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

LOCAL ksc_faf_alt   IS ksc_rwy09_elev + GS_HGT_15KM.              // on GS
LOCAL ksc_iaf30_alt IS ksc_rwy09_elev + GS_HGT_30KM - BELOW_GS_M. // 300 m below GS
LOCAL ksc_iaf60_alt IS ksc_rwy09_elev + GS_HGT_60KM - BELOW_GS_M. // 300 m below GS

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
// KSC RWY 27L approach fixes (3 deg GS)
// ----------------------------
LOCAL ksc27l_iaf60_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 60000).
LOCAL ksc27l_iaf30_ll IS GEO_DESTINATION(ksc_rwy27_thr, 90, 30000).
LOCAL ksc27l_faf_ll   IS GEO_DESTINATION(ksc_rwy27_thr, 90, 15000).

LOCAL ksc_27l_faf_alt   IS ksc_rwy27l_elev + GS_HGT_15KM.
LOCAL ksc_27l_iaf30_alt IS ksc_rwy27l_elev + GS_HGT_30KM - BELOW_GS_M.
LOCAL ksc_27l_iaf60_alt IS ksc_rwy27l_elev + GS_HGT_60KM - BELOW_GS_M.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27L_60", BTYPE_IAF,
  ksc27l_iaf60_ll, ksc_27l_iaf60_alt,
  LEXICON("name", "KSC RWY27L IAF 60km", "runway", "27L")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_27L_30", BTYPE_IAF,
  ksc27l_iaf30_ll, ksc_27l_iaf30_alt,
  LEXICON("name", "KSC RWY27L IAF 30km", "runway", "27L")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_27L", BTYPE_FAF,
  ksc27l_faf_ll, ksc_27l_faf_alt,
  LEXICON("name", "KSC RWY27L FAF 15km", "runway", "27L")
)).

// ----------------------------
// KSC RWY 09R approach fixes (Van's KSC long runway)
// ----------------------------
LOCAL ksc09r_iaf60_ll IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 60000).
LOCAL ksc09r_iaf30_ll IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 30000).
LOCAL ksc09r_faf_ll   IS GEO_DESTINATION(ksc_rwy09r_thr, 270, 15000).

LOCAL ksc_09r_faf_alt   IS ksc_rwy09r_elev + GS4_HGT_15KM.
LOCAL ksc_09r_iaf30_alt IS ksc_rwy09r_elev + GS4_HGT_30KM - BELOW_GS_M.
LOCAL ksc_09r_iaf60_alt IS ksc_rwy09r_elev + GS4_HGT_60KM - BELOW_GS_M.

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09R_60", BTYPE_IAF,
  ksc09r_iaf60_ll, ksc_09r_iaf60_alt,
  LEXICON("name", "KSC RWY09R IAF 60km", "runway", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_IAF_09R_30", BTYPE_IAF,
  ksc09r_iaf30_ll, ksc_09r_iaf30_alt,
  LEXICON("name", "KSC RWY09R IAF 30km", "runway", "09R")
)).

REGISTER_BEACON(MAKE_BEACON(
  "KSC_FAF_09R", BTYPE_FAF,
  ksc09r_faf_ll, ksc_09r_faf_alt,
  LEXICON("name", "KSC RWY09R FAF 15km", "runway", "09R")
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

GLOBAL PLATE_KSC_ILS27L IS MAKE_PLATE(
  "KSC ILS RWY 27L",
  "KSC_ILS_27L",
  LIST("KSC_IAF_27L_60", "KSC_IAF_27L_30", "KSC_FAF_27L"),
  75,
  LEXICON(
    "KSC_IAF_27L_60", ksc_27l_iaf60_alt,
    "KSC_IAF_27L_30", ksc_27l_iaf30_alt,
    "KSC_FAF_27L",    ksc_27l_faf_alt
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

GLOBAL PLATE_KSC_ILS27L_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 27L (short)",
  "KSC_ILS_27L",
  LIST("KSC_IAF_27L_30", "KSC_FAF_27L"),
  75,
  LEXICON(
    "KSC_IAF_27L_30", ksc_27l_iaf30_alt,
    "KSC_FAF_27L",    ksc_27l_faf_alt
  )
).

GLOBAL PLATE_KSC_ILS09R IS MAKE_PLATE(
  "KSC ILS RWY 09R",
  "KSC_ILS_09R",
  LIST("KSC_IAF_09R_60", "KSC_IAF_09R_30", "KSC_FAF_09R"),
  75,
  LEXICON(
    "KSC_IAF_09R_60", ksc_09r_iaf60_alt,
    "KSC_IAF_09R_30", ksc_09r_iaf30_alt,
    "KSC_FAF_09R",    ksc_09r_faf_alt
  )
).


GLOBAL PLATE_KSC_ILS09R_SHORT IS MAKE_PLATE(
  "KSC ILS RWY 09R (short)",
  "KSC_ILS_09R",
  LIST("KSC_IAF_09R_30", "KSC_FAF_09R"),
  75,
  LEXICON(
    "KSC_IAF_09R_30", ksc_09r_iaf30_alt,
    "KSC_FAF_09R",    ksc_09r_faf_alt
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

GLOBAL PLATE_POL_ILS34 IS MAKE_PLATE(
  "POL ILS RWY 34",
  "POL_ILS_34",
  LIST("POL_IAF_34_60", "POL_IAF_34_30", "POL_FAF_34"),
  70,
  LEXICON(
    "POL_IAF_34_60", pol_iaf60_alt,
    "POL_IAF_34_30", pol_iaf30_alt,
    "POL_FAF_34",    pol_faf_alt
  )
).

GLOBAL PLATE_POL_ILS34_SHORT IS MAKE_PLATE(
  "POL ILS RWY 34 (short)",
  "POL_ILS_34",
  LIST("POL_IAF_34_30", "POL_FAF_34"),
  70,
  LEXICON(
    "POL_IAF_34_30", pol_iaf30_alt,
    "POL_FAF_34",    pol_faf_alt
  )
).

// ----------------------------
// Plate registry  (for serialisable plan save/load)
// Maps string ID -> plate LEXICON so legs can store plate_id as a string.
// ----------------------------
GLOBAL PLATE_REGISTRY IS LEXICON().
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09",       PLATE_KSC_ILS09).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27L",      PLATE_KSC_ILS27L).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09_SHORT", PLATE_KSC_ILS09_SHORT).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS27L_SHORT", PLATE_KSC_ILS27L_SHORT).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09R",       PLATE_KSC_ILS09R).
PLATE_REGISTRY:ADD("PLATE_KSC_ILS09R_SHORT", PLATE_KSC_ILS09R_SHORT).
PLATE_REGISTRY:ADD("PLATE_ISL_ILS09",       PLATE_ISL_ILS09).
PLATE_REGISTRY:ADD("PLATE_ISL_ILS27",       PLATE_ISL_ILS27).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS36",       PLATE_DAF_ILS36).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS18",       PLATE_DAF_ILS18).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS36_SHORT", PLATE_DAF_ILS36_SHORT).
PLATE_REGISTRY:ADD("PLATE_DAF_ILS18_SHORT", PLATE_DAF_ILS18_SHORT).
PLATE_REGISTRY:ADD("PLATE_POL_ILS34",       PLATE_POL_ILS34).
PLATE_REGISTRY:ADD("PLATE_POL_ILS34_SHORT", PLATE_POL_ILS34_SHORT).

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
  "PLATE_KSC_ILS27L",
  "PLATE_KSC_ILS27L_SHORT",
  "PLATE_ISL_ILS09",
  "PLATE_ISL_ILS27",
  "PLATE_DAF_ILS36",
  "PLATE_DAF_ILS36_SHORT",
  "PLATE_DAF_ILS18",
  "PLATE_DAF_ILS18_SHORT",
  "PLATE_POL_ILS34",
  "PLATE_POL_ILS34_SHORT"
).

// ----------------------------
// NavInstruments supplemental runway import
// ----------------------------
// Adds missing ILS + IAF/FAF + approach plates from NavInstruments:
// - PluginData/defaultRunways.cfg
// - PluginData/makingHistoryRunways.cfg
// - ModuleManagerCfgs/KerbinSideRemastered.cfg
//
// Existing handcrafted IFC approaches above are kept as source-of-truth.
// This section only registers runways that are not already in NAV_BEACON_DB.
FUNCTION _LIST_HAS_VALUE {
  PARAMETER items, target.
  FOR value IN items {
    IF value = target { RETURN TRUE. }
  }
  RETURN FALSE.
}

FUNCTION _ADD_DYNAMIC_PLATE {
  PARAMETER plate_id, plate.
  IF NOT PLATE_REGISTRY:HASKEY(plate_id) {
    PLATE_REGISTRY:ADD(plate_id, plate).
  }
  IF NOT _LIST_HAS_VALUE(PLATE_IDS, plate_id) {
    PLATE_IDS:ADD(plate_id).
  }
}

FUNCTION REGISTER_NAVUTIL_ILS_SUITE {
  PARAMETER apt, rwy, hdg, thr_lat, thr_lng, alt_msl, gs_angle, vapp, src_sid.

  LOCAL ils_id IS apt + "_ILS_" + rwy.
  IF NAV_BEACON_DB:HASKEY(ils_id) { RETURN. }

  LOCAL thr_ll IS LATLNG(thr_lat, thr_lng).
  REGISTER_BEACON(MAKE_BEACON(
    ils_id, BTYPE_ILS,
    thr_ll, alt_msl,
    LEXICON("hdg", hdg, "gs_angle", gs_angle, "rwy", rwy, "src_sid", src_sid)
  )).

  LOCAL inbound_recip IS hdg + 180.
  IF inbound_recip >= 360 { SET inbound_recip TO inbound_recip - 360. }

  LOCAL iaf60_id IS apt + "_IAF_" + rwy + "_60".
  LOCAL iaf30_id IS apt + "_IAF_" + rwy + "_30".
  LOCAL faf_id   IS apt + "_FAF_" + rwy.

  LOCAL iaf60_ll IS GEO_DESTINATION(thr_ll, inbound_recip, 60000).
  LOCAL iaf30_ll IS GEO_DESTINATION(thr_ll, inbound_recip, 30000).
  LOCAL faf_ll   IS GEO_DESTINATION(thr_ll, inbound_recip, 15000).

  LOCAL gs_hgt_15km IS ROUND(15000 * TAN(gs_angle), 0).
  LOCAL gs_hgt_30km IS ROUND(30000 * TAN(gs_angle), 0).
  LOCAL gs_hgt_60km IS ROUND(60000 * TAN(gs_angle), 0).

  LOCAL faf_alt   IS alt_msl + gs_hgt_15km.
  LOCAL iaf30_alt IS alt_msl + gs_hgt_30km - BELOW_GS_M.
  LOCAL iaf60_alt IS alt_msl + gs_hgt_60km - BELOW_GS_M.

  REGISTER_BEACON(MAKE_BEACON(
    iaf60_id, BTYPE_IAF,
    iaf60_ll, iaf60_alt,
    LEXICON("name", apt + " RWY" + rwy + " IAF 60km", "runway", rwy)
  )).
  REGISTER_BEACON(MAKE_BEACON(
    iaf30_id, BTYPE_IAF,
    iaf30_ll, iaf30_alt,
    LEXICON("name", apt + " RWY" + rwy + " IAF 30km", "runway", rwy)
  )).
  REGISTER_BEACON(MAKE_BEACON(
    faf_id, BTYPE_FAF,
    faf_ll, faf_alt,
    LEXICON("name", apt + " RWY" + rwy + " FAF 15km", "runway", rwy)
  )).

  LOCAL plate_id IS "PLATE_" + apt + "_ILS" + rwy.
  LOCAL plate_short_id IS plate_id + "_SHORT".

  LOCAL plate_full IS MAKE_PLATE(
    apt + " ILS RWY " + rwy,
    ils_id,
    LIST(iaf60_id, iaf30_id, faf_id),
    vapp,
    LEXICON(
      iaf60_id, iaf60_alt,
      iaf30_id, iaf30_alt,
      faf_id,   faf_alt
    )
  ).

  LOCAL plate_short IS MAKE_PLATE(
    apt + " ILS RWY " + rwy + " (short)",
    ils_id,
    LIST(iaf30_id, faf_id),
    vapp,
    LEXICON(
      iaf30_id, iaf30_alt,
      faf_id,   faf_alt
    )
  ).

  _ADD_DYNAMIC_PLATE(plate_id, plate_full).
  _ADD_DYNAMIC_PLATE(plate_short_id, plate_short).
}

LOCAL NAVUTIL_MISSING_RUNWAYS IS LIST(
  LEXICON("apt", "KSC", "rwy", "27",  "hdg", 270.4000, "alt",   70.0000, "lat", -0.0500586, "lng",  -74.5175184, "vapp", 75, "sid", "SC27"),
  LEXICON("apt", "BAI", "rwy", "20",  "hdg", 198.0000, "alt",  414.4000, "lat", 20.6890488, "lng", -146.4836880, "vapp", 70, "sid", "BAI20"),
  LEXICON("apt", "BAI", "rwy", "02",  "hdg",  18.0000, "alt",  415.8000, "lat", 20.5199947, "lng", -146.5410610, "vapp", 70, "sid", "BAI02"),
  LEXICON("apt", "CAK", "rwy", "33",  "hdg", 330.0000, "alt",   74.3000, "lat", 24.8293533, "lng",  -83.5805283, "vapp", 70, "sid", "CAK33"),
  LEXICON("apt", "CAK", "rwy", "15",  "hdg", 150.0000, "alt",   74.4000, "lat", 24.9831390, "lng",  -83.6768723, "vapp", 70, "sid", "CAK15"),
  LEXICON("apt", "DUA", "rwy", "27",  "hdg", 266.0000, "alt",  455.7000, "lat", -39.2922478, "lng", 116.3527370, "vapp", 70, "sid", "DUA27"),
  LEXICON("apt", "DUA", "rwy", "09",  "hdg",  87.0000, "alt",  454.9000, "lat", -39.3039322, "lng", 116.1250460, "vapp", 70, "sid", "DUA09"),
  LEXICON("apt", "DUA", "rwy", "03",  "hdg",  26.0000, "alt",  454.9000, "lat", -39.3772469, "lng", 116.1888120, "vapp", 70, "sid", "DUA03"),
  LEXICON("apt", "DUA", "rwy", "21",  "hdg", 206.0000, "alt",  455.8000, "lat", -39.2186279, "lng", 116.2897030, "vapp", 70, "sid", "DUA21"),
  LEXICON("apt", "HAA", "rwy", "08",  "hdg",  79.0000, "alt", 3953.1001, "lat", -56.3064957, "lng",  -11.0708122, "vapp", 70, "sid", "HAA08"),
  LEXICON("apt", "HAA", "rwy", "26",  "hdg", 259.0000, "alt", 3952.0000, "lat", -56.2874184, "lng",  -10.8933182, "vapp", 70, "sid", "HAA26"),
  LEXICON("apt", "HAZ", "rwy", "05",  "hdg",  50.0000, "alt",   24.9000, "lat", -14.2588758, "lng",  155.2005160, "vapp", 70, "sid", "HAS05"),
  LEXICON("apt", "HAZ", "rwy", "23",  "hdg", 230.0000, "alt",   22.3000, "lat", -14.1945810, "lng",  155.2807010, "vapp", 70, "sid", "HAS23"),
  LEXICON("apt", "JEJ", "rwy", "01",  "hdg",  12.0000, "alt",  758.8000, "lat",   6.8674359, "lng",  -77.9567795, "vapp", 70, "sid", "JEB01"),
  LEXICON("apt", "JEJ", "rwy", "19",  "hdg", 192.0000, "alt",  759.1000, "lat",   7.0415063, "lng",  -77.9192657, "vapp", 70, "sid", "JEB19"),
  LEXICON("apt", "KAM", "rwy", "27",  "hdg", 272.0000, "alt",  620.3000, "lat",  36.1776314, "lng",   10.7111731, "vapp", 70, "sid", "KAG27"),
  LEXICON("apt", "KAM", "rwy", "09",  "hdg",  92.0000, "alt",  619.6000, "lat",  36.1856995, "lng",   10.4924726, "vapp", 70, "sid", "KAG09"),
  LEXICON("apt", "KAT", "rwy", "11L", "hdg", 111.0000, "alt",  204.9000, "lat", -37.0405540, "lng",  -71.1069031, "vapp", 70, "sid", "KEA11L"),
  LEXICON("apt", "KAT", "rwy", "11R", "hdg", 111.0000, "alt",  204.5000, "lat", -37.0826569, "lng",  -71.1321030, "vapp", 70, "sid", "KEA11R"),
  LEXICON("apt", "KAT", "rwy", "29R", "hdg", 291.0000, "alt",  204.9000, "lat", -37.1039238, "lng",  -70.9000702, "vapp", 70, "sid", "KEA29R"),
  LEXICON("apt", "KAT", "rwy", "29L", "hdg", 291.0000, "alt",  204.6000, "lat", -37.1460075, "lng",  -70.9248199, "vapp", 70, "sid", "KEA29L"),
  LEXICON("apt", "KER", "rwy", "19",  "hdg", 185.0000, "alt",   34.0000, "lat", -89.8201675, "lng", -170.2836150, "vapp", 70, "sid", "KER19"),
  LEXICON("apt", "KER", "rwy", "02",  "hdg",  19.0000, "alt",   33.5000, "lat", -89.9244385, "lng", -177.1661680, "vapp", 70, "sid", "KER02"),
  LEXICON("apt", "KOJ", "rwy", "22L", "hdg", 218.0000, "alt",  764.4000, "lat",   6.1200619, "lng", -141.9233090, "vapp", 70, "sid", "KOS22L"),
  LEXICON("apt", "KOJ", "rwy", "22R", "hdg", 218.0000, "alt",  763.3000, "lat",   6.1106548, "lng", -141.9902500, "vapp", 70, "sid", "KOS22R"),
  LEXICON("apt", "KOJ", "rwy", "04R", "hdg",  38.0000, "alt",  763.6000, "lat",   5.9807258, "lng", -142.0324400, "vapp", 70, "sid", "KOS04R"),
  LEXICON("apt", "KOJ", "rwy", "04L", "hdg",  38.0000, "alt",  763.0000, "lat",   6.0317116, "lng", -142.0523680, "vapp", 70, "sid", "KOS04L"),
  LEXICON("apt", "KOL", "rwy", "02",  "hdg",  16.0000, "alt",   25.1000, "lat",  -4.2447028, "lng",  -72.1335983, "vapp", 70, "sid", "KOI02"),
  LEXICON("apt", "KOL", "rwy", "20",  "hdg", 196.0000, "alt",   22.1000, "lat",  -4.0743165, "lng",  -72.0850906, "vapp", 70, "sid", "KOI20"),
  LEXICON("apt", "MEE", "rwy", "31",  "hdg", 313.0000, "alt",   22.0000, "lat",  37.8082504, "lng", -109.7625890, "vapp", 70, "sid", "MEE31"),
  LEXICON("apt", "MEE", "rwy", "13",  "hdg", 133.0000, "alt",   20.3000, "lat",  37.9301147, "lng", -109.9253690, "vapp", 70, "sid", "MEE13"),
  LEXICON("apt", "MEE", "rwy", "07",  "hdg",  73.0000, "alt",   22.0000, "lat",  37.8440514, "lng", -109.9510040, "vapp", 70, "sid", "MEE07"),
  LEXICON("apt", "MEE", "rwy", "25",  "hdg", 253.0000, "alt",   20.3000, "lat",  37.8942909, "lng", -109.7363430, "vapp", 70, "sid", "MEE25"),
  LEXICON("apt", "NYE", "rwy", "33",  "hdg", 329.0000, "alt",  320.6000, "lat",   5.7032723, "lng",  108.7760470, "vapp", 70, "sid", "NYI33"),
  LEXICON("apt", "NYE", "rwy", "15",  "hdg", 149.0000, "alt",  320.2000, "lat",   5.7888746, "lng",  108.7238310, "vapp", 70, "sid", "NYI15"),
  LEXICON("apt", "POL", "rwy", "16",  "hdg", 157.0000, "alt",   31.6000, "lat",  72.6096497, "lng",  -78.6251831, "vapp", 70, "sid", "POR16"),
  LEXICON("apt", "RND", "rwy", "28R", "hdg", 282.0000, "alt", 1185.9000, "lat",  -6.0184669, "lng",   99.5971069, "vapp", 70, "sid", "ROR28R"),
  LEXICON("apt", "RND", "rwy", "28L", "hdg", 282.0000, "alt", 1185.1000, "lat",  -6.0575519, "lng",   99.5562973, "vapp", 70, "sid", "ROR28L"),
  LEXICON("apt", "RND", "rwy", "10L", "hdg", 102.0000, "alt", 1186.1000, "lat",  -5.9808364, "lng",   99.4235535, "vapp", 70, "sid", "ROR10L"),
  LEXICON("apt", "RND", "rwy", "10R", "hdg", 102.0000, "alt", 1185.1000, "lat",  -6.0359821, "lng",   99.4573364, "vapp", 70, "sid", "ROR10R"),
  LEXICON("apt", "SND", "rwy", "04",  "hdg",  44.0000, "alt",   23.1000, "lat",  -8.1883078, "lng",  -42.4340630, "vapp", 70, "sid", "SAI04"),
  LEXICON("apt", "SND", "rwy", "22",  "hdg", 224.0000, "alt",   26.2000, "lat",  -8.1163788, "lng",  -42.3633995, "vapp", 70, "sid", "SAI22"),
  LEXICON("apt", "SOF", "rwy", "27",  "hdg", 271.0000, "alt",   79.6000, "lat", -47.0086441, "lng", -140.8494570, "vapp", 70, "sid", "SOF27"),
  LEXICON("apt", "SOF", "rwy", "09",  "hdg",  91.0000, "alt",   79.6000, "lat", -47.0042114, "lng", -141.1089780, "vapp", 70, "sid", "SOF09"),
  LEXICON("apt", "SLK", "rwy", "25",  "hdg", 251.0000, "alt",   72.3000, "lat", -37.2554588, "lng",   52.6673737, "vapp", 70, "sid", "SOL25"),
  LEXICON("apt", "SLK", "rwy", "07",  "hdg",  71.0000, "alt",   70.0000, "lat", -37.3114128, "lng",   52.4563026, "vapp", 70, "sid", "SOL07"),
  LEXICON("apt", "SLK", "rwy", "04",  "hdg",  41.0000, "alt",   70.3000, "lat", -37.2941704, "lng",   52.3529968, "vapp", 70, "sid", "SOL04"),
  LEXICON("apt", "SLK", "rwy", "22",  "hdg", 221.0000, "alt",   70.3000, "lat", -37.1608047, "lng",   52.4987793, "vapp", 70, "sid", "SOL22"),
  LEXICON("apt", "UBD", "rwy", "04",  "hdg",  41.0000, "alt",  503.2000, "lat",  38.4407425, "lng", -149.7023470, "vapp", 70, "sid", "UBA04"),
  LEXICON("apt", "UBD", "rwy", "22",  "hdg", 221.0000, "alt",  499.2000, "lat",  38.5745163, "lng", -149.5554810, "vapp", 70, "sid", "UBA22"),
  LEXICON("apt", "TSC", "rwy", "16",  "hdg", 160.0000, "alt",   62.0000, "lat",  17.3383942, "lng",   88.6990891, "vapp", 70, "sid", "sID4"),
  LEXICON("apt", "TSC", "rwy", "34",  "hdg", 340.0000, "alt",   61.1000, "lat",  17.1416798, "lng",   88.7741318, "vapp", 70, "sid", "sID4"),
  LEXICON("apt", "XXX", "rwy", "31",  "hdg", 311.0000, "alt", 1373.5000, "lat", -60.5925903, "lng",   39.3401833, "vapp", 70, "sid", "XXX31"),
  LEXICON("apt", "XXX", "rwy", "13",  "hdg", 131.0000, "alt", 1369.1000, "lat", -60.4769745, "lng",   39.0680542, "vapp", 70, "sid", "XXX13")
).

FOR nav_rwy IN NAVUTIL_MISSING_RUNWAYS {
  REGISTER_NAVUTIL_ILS_SUITE(
    nav_rwy["apt"],
    nav_rwy["rwy"],
    nav_rwy["hdg"],
    nav_rwy["lat"],
    nav_rwy["lng"],
    nav_rwy["alt"],
    3.0,
    nav_rwy["vapp"],
    nav_rwy["sid"]
  ).
}

// ----------------------------
// Runway alias lookup
// ----------------------------
// Allows direct lookup by runway token (for example: "CAK33", "KAT11L",
// "SC27", "09", "27L") in addition to cycling PLATE_IDS.
GLOBAL RUNWAY_ALIAS_TO_PLATE IS LEXICON().

FUNCTION _NORMALIZE_RUNWAY_KEY {
  PARAMETER raw.
  LOCAL key IS ("" + raw):TOUPPER.
  SET key TO key:REPLACE(" ", "").
  SET key TO key:REPLACE("-", "").
  RETURN key.
}

FUNCTION REGISTER_RUNWAY_ALIAS {
  PARAMETER alias, plate_id.
  LOCAL key IS _NORMALIZE_RUNWAY_KEY(alias).
  IF key = "" { RETURN. }
  SET RUNWAY_ALIAS_TO_PLATE[key] TO plate_id.
}

FUNCTION REGISTER_RUNWAY_FAMILY_ALIASES {
  PARAMETER apt, rwy, nav_sid.
  LOCAL plate_id IS "PLATE_" + apt + "_ILS" + rwy.
  REGISTER_RUNWAY_ALIAS(apt + rwy, plate_id).
  REGISTER_RUNWAY_ALIAS(apt + "_" + rwy, plate_id).
  IF nav_sid <> "" { REGISTER_RUNWAY_ALIAS(nav_sid, plate_id). }
}

FUNCTION _GET_PLATE_BY_ID_WITH_SHORT_OPTION {
  PARAMETER plate_id, short_approach.

  IF short_approach {
    LOCAL short_id IS plate_id.
    IF short_id:LENGTH < 6 OR short_id:SUBSTRING(short_id:LENGTH - 6, 6) <> "_SHORT" {
      SET short_id TO plate_id + "_SHORT".
    }
    IF PLATE_REGISTRY:HASKEY(short_id) { RETURN PLATE_REGISTRY[short_id]. }
  }

  IF PLATE_REGISTRY:HASKEY(plate_id) { RETURN PLATE_REGISTRY[plate_id]. }
  RETURN 0.
}

// Historical direct runway IDs
REGISTER_RUNWAY_ALIAS("09",    "PLATE_KSC_ILS09").
REGISTER_RUNWAY_ALIAS("09R",   "PLATE_KSC_ILS09R").
REGISTER_RUNWAY_ALIAS("27L",   "PLATE_KSC_ILS27L").
REGISTER_RUNWAY_ALIAS("27",    "PLATE_KSC_ILS27L"). // backwards-compatible plain "27"
REGISTER_RUNWAY_ALIAS("36",    "PLATE_DAF_ILS36").
REGISTER_RUNWAY_ALIAS("18",    "PLATE_DAF_ILS18").
REGISTER_RUNWAY_ALIAS("POR34", "PLATE_POL_ILS34").
REGISTER_RUNWAY_ALIAS("POL34", "PLATE_POL_ILS34").

// Structured airport/runway aliases
REGISTER_RUNWAY_FAMILY_ALIASES("KSC", "09",  "SC09").
REGISTER_RUNWAY_FAMILY_ALIASES("KSC", "09R", "").
REGISTER_RUNWAY_FAMILY_ALIASES("KSC", "27L", "").
REGISTER_RUNWAY_FAMILY_ALIASES("ISL", "09",  "IS09").
REGISTER_RUNWAY_FAMILY_ALIASES("ISL", "27",  "IS27").
REGISTER_RUNWAY_FAMILY_ALIASES("DAF", "36",  "DA36").
REGISTER_RUNWAY_FAMILY_ALIASES("DAF", "18",  "DA18").
REGISTER_RUNWAY_FAMILY_ALIASES("POL", "34",  "POR34").

FOR nav_rwy IN NAVUTIL_MISSING_RUNWAYS {
  REGISTER_RUNWAY_FAMILY_ALIASES(
    nav_rwy["apt"],
    nav_rwy["rwy"],
    nav_rwy["sid"]
  ).
}

FUNCTION GET_PLATE_FOR_RUNWAY {
  PARAMETER rwy_id, short_approach.
  LOCAL key IS _NORMALIZE_RUNWAY_KEY(rwy_id).
  LOCAL plate IS 0.

  // Allow direct plate ID lookup too, e.g. "PLATE_CAK_ILS33".
  IF PLATE_REGISTRY:HASKEY(key) {
    SET plate TO _GET_PLATE_BY_ID_WITH_SHORT_OPTION(key, short_approach).
    IF plate <> 0 { RETURN plate. }
  }

  IF RUNWAY_ALIAS_TO_PLATE:HASKEY(key) {
    SET plate TO _GET_PLATE_BY_ID_WITH_SHORT_OPTION(RUNWAY_ALIAS_TO_PLATE[key], short_approach).
    IF plate <> 0 { RETURN plate. }
  }
  SET IFC_ALERT_TEXT TO "NAV: no plate for runway '" + rwy_id + "'".
  SET IFC_ALERT_UT   TO TIME:SECONDS.
  RETURN 0.
}
