@LAZYGLOBAL OFF.

// ============================================================
// nav_math.ks  -  Integrated Flight Computer
// Spherical navigation math for Kerbin.
//
// All angles in degrees unless the function name says RAD.
// Uses Kerbin body radius dynamically (SHIP:BODY:RADIUS).
//
// NOTE: kOS trig functions (SIN, COS, TAN) take DEGREES.
// ARCSIN/ARCCOS/ARCTAN2 return DEGREES.
// The only place CONSTANT:DEGTORAD appears here is where we
// convert a degree-valued arc-angle into radians for the
// final  d = R * c  distance formula.
// ============================================================

// Great-circle distance in metres between two LatLng objects.
FUNCTION GEO_DISTANCE {
  PARAMETER ll1, ll2.
  LOCAL BR IS SHIP:BODY:RADIUS.
  LOCAL dlat IS ll2:LAT - ll1:LAT.         // degrees
  LOCAL dlng IS ll2:LNG - ll1:LNG.         // degrees
  LOCAL a IS SIN(dlat/2)^2 + COS(ll1:LAT)*COS(ll2:LAT)*SIN(dlng/2)^2.
  // Clamp 'a' to avoid domain error in SQRT when rounding pushes it slightly above 1.
  SET a TO CLAMP(a, 0, 1).
  // ARCTAN2 returns degrees; convert to radians for R*c.
  LOCAL c IS 2 * ARCTAN2(SQRT(a), SQRT(1 - a)) * CONSTANT:DEGTORAD.
  RETURN BR * c.
}

// Initial bearing (degrees) from ll1 to ll2 (0 = North, 90 = East).
FUNCTION GEO_BEARING {
  PARAMETER ll1, ll2.
  LOCAL dlng IS ll2:LNG - ll1:LNG.         // degrees
  LOCAL y IS SIN(dlng) * COS(ll2:LAT).
  LOCAL x IS COS(ll1:LAT)*SIN(ll2:LAT) - SIN(ll1:LAT)*COS(ll2:LAT)*COS(dlng).
  // ARCTAN2 returns degrees; WRAP_360 keeps it in [0, 360).
  RETURN WRAP_360(ARCTAN2(y, x)).
}

// Destination LatLng when travelling bearing_deg for dist_m from start_ll.
FUNCTION GEO_DESTINATION {
  PARAMETER start_ll, bearing_deg, dist_m.
  LOCAL BR IS SHIP:BODY:RADIUS.
  // Angular distance in degrees (divide radians by DEGTORAD).
  LOCAL ang IS dist_m / BR / CONSTANT:DEGTORAD.
  LOCAL lat2 IS ARCSIN(CLAMP(
    SIN(start_ll:LAT)*COS(ang) + COS(start_ll:LAT)*SIN(ang)*COS(bearing_deg),
    -1, 1)).
  LOCAL lon2 IS start_ll:LNG + ARCTAN2(
    SIN(bearing_deg)*SIN(ang)*COS(start_ll:LAT),
    COS(ang) - SIN(start_ll:LAT)*SIN(lat2)).
  RETURN LATLNG(lat2, lon2).
}

// Cross-track distance (metres) of the aircraft from the great-circle track
// defined by from_ll -> to_ll.
// Positive = aircraft is to the right of the track.
// Negative = aircraft is to the left.
FUNCTION CROSS_TRACK_ERROR {
  PARAMETER ac_ll, from_ll, to_ll.
  LOCAL BR IS SHIP:BODY:RADIUS.
  // Angular distance from_ll -> ac in degrees.
  LOCAL d13_deg IS GEO_DISTANCE(from_ll, ac_ll) / BR / CONSTANT:DEGTORAD.
  LOCAL brng13 IS GEO_BEARING(from_ll, ac_ll).  // degrees
  LOCAL brng12 IS GEO_BEARING(from_ll, to_ll).  // degrees
  // ARCSIN returns degrees; convert to radians for R*angle.
  RETURN BR * ARCSIN(CLAMP(SIN(d13_deg) * SIN(brng13 - brng12), -1, 1)) * CONSTANT:DEGTORAD.
}

// Along-track distance (metres) – how far along from_ll->to_ll the aircraft is.
// Positive = aircraft is between from_ll and to_ll (or past to_ll).
FUNCTION ALONG_TRACK_DIST {
  PARAMETER ac_ll, from_ll, to_ll.
  LOCAL BR IS SHIP:BODY:RADIUS.
  LOCAL d13_deg IS GEO_DISTANCE(from_ll, ac_ll) / BR / CONSTANT:DEGTORAD.
  LOCAL xt_deg  IS CROSS_TRACK_ERROR(ac_ll, from_ll, to_ll) / BR / CONSTANT:DEGTORAD.
  // Guard: if cross-track ≈ 90° COS(xt_deg) ≈ 0; clamp prevents divide explosion.
  LOCAL cos_xt IS COS(xt_deg).
  IF ABS(cos_xt) < 0.0001 { RETURN 0. }
  RETURN BR * ARCCOS(CLAMP(COS(d13_deg) / cos_xt, -1, 1)) * CONSTANT:DEGTORAD.
}
