@LAZYGLOBAL OFF.

// ============================================================
// nav_custom_wpts.ks  -  Integrated Flight Computer
// User-defined waypoints for cruise leg routing.
//
// HOW TO ADD A WAYPOINT
// ---------------------
// Copy any REGISTER_BEACON block below and give it:
//   id      - unique ALL_CAPS string, used in flight plans
//   ll      - LATLNG(lat, lng)  (negative lat = South, negative lng = West)
//   alt_asl - cruising altitude in metres MSL to cross this point
//   name    - human-readable label shown on the FMS display
//
// The id must be unique across all beacons (nav_beacons.ks + this file).
// After saving, the waypoint is available to use in cruise legs
// and will appear in the FMS waypoint picker.
//
// Loaded by ifc_main.ks after nav_beacons.ks.
// ============================================================

// ----------------------------
// KSC area waypoints
// ----------------------------

// Mid-ocean between KSC and Island Airstrip.
// Useful as a cruise waypoint when routing KSC -> Island at altitude.
REGISTER_BEACON(MAKE_BEACON(
  "WPT_KSC_ISLAND_MID", BTYPE_WPT,
  LATLNG(-0.78, -73.22), 1500,
  LEXICON("name", "KSC-Island Mid")
)).

// Departure fix north of KSC.
// Use as a first cruise waypoint when departing KSC RWY27 and turning north.
REGISTER_BEACON(MAKE_BEACON(
  "WPT_KSC_NORTH", BTYPE_WPT,
  LATLNG(0.50, -74.57), 1500,
  LEXICON("name", "KSC North")
)).

// High-altitude cruise point over the ocean (use for spaceplane profiles).
// Same position as KSC-Island Mid but tagged at 6000 m for high cruise.
REGISTER_BEACON(MAKE_BEACON(
  "WPT_KSC_ISLAND_MID_HIGH", BTYPE_WPT,
  LATLNG(-0.78, -73.22), 6000,
  LEXICON("name", "KSC-Island Mid High")
)).

// ----------------------------
// NavInstruments Updated airport waypoints (Kerbin)
// Source files in this install:
// - PluginData/defaultRunways.cfg
// - PluginData/makingHistoryRunways.cfg
// - ModuleManagerCfgs/KerbinSideRemastered.cfg
//
// One waypoint per airport (averaged from runway GS points).
// ----------------------------

REGISTER_BEACON(MAKE_BEACON("WPT_APT_BAI", BTYPE_WPT, LATLNG(20.604522,  -146.512375), 415.1,  LEXICON("name", "Airport Baikerbanur"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_CAK", BTYPE_WPT, LATLNG(24.906246,   -83.628700),  74.4,  LEXICON("name", "Airport Cape Kerman"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_DAF", BTYPE_WPT, LATLNG(-6.522210,  -144.039633), 821.8,  LEXICON("name", "Airport Desert Airfield"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_DUA", BTYPE_WPT, LATLNG(-39.298014,  116.239074), 455.3,  LEXICON("name", "Airport Dununda"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_HAA", BTYPE_WPT, LATLNG(-56.296957,  -10.982065), 3952.6, LEXICON("name", "Airport Harvester"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_HAZ", BTYPE_WPT, LATLNG(-14.226728,  155.240608),  23.6,  LEXICON("name", "Airport Hazard"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_ISL", BTYPE_WPT, LATLNG(-1.516628,   -71.904050), 134.0,  LEXICON("name", "Airport Island Airstrip"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_JEJ", BTYPE_WPT, LATLNG(6.954471,    -77.938023), 758.9,  LEXICON("name", "Airport Jeb's Junkyard"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KAM", BTYPE_WPT, LATLNG(36.181665,    10.601823), 619.9,  LEXICON("name", "Airport Kamberwick"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KAT", BTYPE_WPT, LATLNG(-37.093286,  -71.015974), 204.7,  LEXICON("name", "Airport Kerman Atoll"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KER", BTYPE_WPT, LATLNG(-89.872303, -173.724892),  33.8,  LEXICON("name", "Airport Kermundsen"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KOJ", BTYPE_WPT, LATLNG(6.060789,   -141.999592), 763.6,  LEXICON("name", "Airport Kojave Sands"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KOL", BTYPE_WPT, LATLNG(-4.159510,   -72.109344),  23.6,  LEXICON("name", "Airport Kola Island"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KSC", BTYPE_WPT, LATLNG(-0.049418,   -74.608892),  70.0,  LEXICON("name", "Airport Kerbal Space Center"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_KSC2", BTYPE_WPT, LATLNG(20.582899, -146.511597), 425.0,  LEXICON("name", "Airport KSC2"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_MEE", BTYPE_WPT, LATLNG(37.869177,  -109.843826),  21.1,  LEXICON("name", "Airport Meeda"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_NYE", BTYPE_WPT, LATLNG(5.746073,    108.749939), 320.4,  LEXICON("name", "Airport Nye"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_POL", BTYPE_WPT, LATLNG(72.563313,   -78.560062),  31.6,  LEXICON("name", "Airport Polar"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_RND", BTYPE_WPT, LATLNG(-6.023209,    99.508574), 1185.5, LEXICON("name", "Airport Round"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_SND", BTYPE_WPT, LATLNG(-8.152343,   -42.398731),  24.7,  LEXICON("name", "Airport Sandy Island"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_SOF", BTYPE_WPT, LATLNG(-47.006428, -140.979218),  79.6,  LEXICON("name", "Airport South Field"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_SLK", BTYPE_WPT, LATLNG(-37.255462,   52.493863),  70.7,  LEXICON("name", "Airport South Lake"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_TSC", BTYPE_WPT, LATLNG(17.240037,    88.736610),  61.5,  LEXICON("name", "Airport TSC"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_UBD", BTYPE_WPT, LATLNG(38.507629,  -149.628914), 501.2,  LEXICON("name", "Airport Uberdam"))).
REGISTER_BEACON(MAKE_BEACON("WPT_APT_XXX", BTYPE_WPT, LATLNG(-60.534782,   39.204119), 1371.3, LEXICON("name", "Airport XXX"))).

// ----------------------------
// Custom waypoint registry
// Maps id -> beacon for FMS waypoint picker enumeration.
// All BTYPE_WPT beacons registered above are added here automatically
// when you call REGISTER_BEACON; this list is used by the FMS UI to
// cycle through available waypoints.
// ----------------------------
GLOBAL CUSTOM_WPT_IDS IS LIST(
  "WPT_APT_BAI",
  "WPT_APT_CAK",
  "WPT_APT_DAF",
  "WPT_APT_DUA",
  "WPT_APT_HAA",
  "WPT_APT_HAZ",
  "WPT_APT_ISL",
  "WPT_APT_JEJ",
  "WPT_APT_KAM",
  "WPT_APT_KAT",
  "WPT_APT_KER",
  "WPT_APT_KOJ",
  "WPT_APT_KOL",
  "WPT_APT_KSC",
  "WPT_APT_KSC2",
  "WPT_APT_MEE",
  "WPT_APT_NYE",
  "WPT_APT_POL",
  "WPT_APT_RND",
  "WPT_APT_SND",
  "WPT_APT_SOF",
  "WPT_APT_SLK",
  "WPT_APT_TSC",
  "WPT_APT_UBD",
  "WPT_APT_XXX",
  "WPT_KSC_ISLAND_MID",
  "WPT_KSC_NORTH",
  "WPT_KSC_ISLAND_MID_HIGH"
).
