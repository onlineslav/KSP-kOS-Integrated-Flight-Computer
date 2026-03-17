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
// Custom waypoint registry
// Maps id -> beacon for FMS waypoint picker enumeration.
// All BTYPE_WPT beacons registered above are added here automatically
// when you call REGISTER_BEACON; this list is used by the FMS UI to
// cycle through available waypoints.
// ----------------------------
GLOBAL CUSTOM_WPT_IDS IS LIST(
  "WPT_KSC_ISLAND_MID",
  "WPT_KSC_NORTH",
  "WPT_KSC_ISLAND_MID_HIGH"
).
