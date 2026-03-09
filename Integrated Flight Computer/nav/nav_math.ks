@LAZYGLOBAL OFF.

// ============================================================
// nav_math.ks  -  Integrated Flight Computer
// Spherical navigation math for Kerbin.
//
// All angles in degrees unless the function name says RAD.
// Uses Kerbin body radius dynamically (SHIP:BODY:RADIUS).
// ============================================================

// Great-circle distance in metres between two LatLng objects.
FUNCTION GEO_DISTANCE {
  PARAMETER ll1, ll2.
  LOCAL R IS SHIP:BODY:RADIUS.
  LOCAL lat1 IS ll1:LAT * CONSTANT:DEGTORAD.
  LOCAL lat2 IS ll2:LAT * CONSTANT:DEGTORAD.
  LOCAL dlat IS (ll2:LAT - ll1:LAT) * CONSTANT:DEGTORAD.
  LOCAL dlng IS (ll2:LNG - ll1:LNG) * CONSTANT:DEGTORAD.
  LOCAL a IS SIN(dlat/2)^2 + COS(lat1)*COS(lat2)*SIN(dlng/2)^2.
  // Clamp 'a' to avoid domain error in SQRT when rounding pushes it slightly above 1.
  SET a TO CLAMP(a, 0, 1).
  LOCAL c IS 2 * ARCTAN2(SQRT(a), SQRT(1 - a)) * CONSTANT:DEGTORAD.
  RETURN R * c.
}

// Initial bearing (degrees) from ll1 to ll2 (0 = North, 90 = East).
FUNCTION GEO_BEARING {
  PARAMETER ll1, ll2.
  LOCAL lat1 IS ll1:LAT * CONSTANT:DEGTORAD.
  LOCAL lat2 IS ll2:LAT * CONSTANT:DEGTORAD.
  LOCAL dlng IS (ll2:LNG - ll1:LNG) * CONSTANT:DEGTORAD.
  LOCAL y IS SIN(dlng) * COS(lat2).
  LOCAL x IS COS(lat1)*SIN(lat2) - SIN(lat1)*COS(lat2)*COS(dlng).
  RETURN WRAP_360(ARCTAN2(y, x) / CONSTANT:DEGTORAD).
}

// Destination LatLng when travelling bearing_deg for dist_m from start_ll.
FUNCTION GEO_DESTINATION {
  PARAMETER start_ll, bearing_deg, dist_m.
  LOCAL R IS SHIP:BODY:RADIUS.
  LOCAL ang IS dist_m / R.   // angular distance (radians)
  LOCAL lat1 IS start_ll:LAT * CONSTANT:DEGTORAD.
  LOCAL lon1 IS start_ll:LNG * CONSTANT:DEGTORAD.
  LOCAL brng IS bearing_deg * CONSTANT:DEGTORAD.
  LOCAL lat2 IS ARCSIN(CLAMP(SIN(lat1)*COS(ang) + COS(lat1)*SIN(ang)*COS(brng), -1, 1)).
  LOCAL lon2 IS lon1 + ARCTAN2(SIN(brng)*SIN(ang)*COS(lat1),
                                COS(ang) - SIN(lat1)*SIN(lat2)).
  RETURN LATLNG(lat2 / CONSTANT:DEGTORAD, lon2 / CONSTANT:DEGTORAD).
}

// Cross-track distance (metres) of the aircraft from the great-circle track
// defined by from_ll -> to_ll.
// Positive = aircraft is to the right of the track.
// Negative = aircraft is to the left.
FUNCTION CROSS_TRACK_ERROR {
  PARAMETER ac_ll, from_ll, to_ll.
  LOCAL R IS SHIP:BODY:RADIUS.
  LOCAL d13 IS GEO_DISTANCE(from_ll, ac_ll) / R.        // angular dist from_ll -> ac
  LOCAL brng13 IS GEO_BEARING(from_ll, ac_ll) * CONSTANT:DEGTORAD.
  LOCAL brng12 IS GEO_BEARING(from_ll, to_ll) * CONSTANT:DEGTORAD.
  RETURN R * ARCSIN(CLAMP(SIN(d13) * SIN(brng13 - brng12), -1, 1)) * CONSTANT:DEGTORAD.
}

// Along-track distance (metres) – how far along from_ll->to_ll the aircraft is.
// Positive = aircraft is between from_ll and to_ll (or past to_ll).
FUNCTION ALONG_TRACK_DIST {
  PARAMETER ac_ll, from_ll, to_ll.
  LOCAL R IS SHIP:BODY:RADIUS.
  LOCAL d13 IS GEO_DISTANCE(from_ll, ac_ll) / R.
  LOCAL xt  IS CROSS_TRACK_ERROR(ac_ll, from_ll, to_ll) / R.
  RETURN R * ARCCOS(CLAMP(COS(d13) / COS(xt), -1, 1)) * CONSTANT:DEGTORAD.
}
