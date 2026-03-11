@LAZYGLOBAL OFF.

// ============================================================
// nav_routes.ks  -  Integrated Flight Computer
// Pre-defined cruise routes.
//
// A route LEXICON contains:
//   "waypoints"    : LIST of beacon IDs to navigate in order
//   "dest_plate"   : approach plate LEXICON for the destination
//   "cruise_alt_m" : target cruise altitude MSL (m)
//   "cruise_spd"   : target cruise IAS (m/s)
//
// Loaded after nav_beacons.ks so all plate globals are available.
// ============================================================

// Route: KSC → Island Airstrip, ILS RWY IS09
// Fly east at 1500 m MSL, capture ISL RWY09 IAF at 30 km,
// then execute the ILS IS09 approach.
GLOBAL ROUTE_KSC_TO_ISL IS LEXICON(
  "waypoints",    LIST("ISL_IAF_09_30"),
  "dest_plate",   PLATE_ISL_ILS09,
  "cruise_alt_m", 1500,
  "cruise_spd",   CRUISE_DEFAULT_SPD
).
