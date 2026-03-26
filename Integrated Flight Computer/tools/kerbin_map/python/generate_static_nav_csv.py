#!/usr/bin/env python3
"""
generate_static_nav_csv.py

Generates ifc_beacons.csv and ifc_plates.csv from hardcoded IFC nav data.
Use this to seed the web viewer without needing KSP/kOS running.

Output: ../web/data/ifc_beacons.csv  and  ../web/data/ifc_plates.csv
"""

from __future__ import annotations
import csv
import math
from pathlib import Path

# ---------------------------------------------------------------------------
# Kerbin constants (must match ifc_constants.ks / nav_beacons.ks)
# ---------------------------------------------------------------------------
KERBIN_RADIUS_M = 600_000.0
GS_ANG          = 3.0   # standard glideslope (deg)
GS_ANG_STEEP    = 4.0   # terrain-clearance glideslope (deg)
BELOW_GS_M      = 300.0 # IAF placed this far below glideslope altitude


def geo_destination(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
    """Compute destination on Kerbin sphere given start, bearing, and distance."""
    d   = dist_m / KERBIN_RADIUS_M
    b   = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(b))
    lon2 = lon1 + math.atan2(
        math.sin(b) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2)
    )
    return math.degrees(lat2), (math.degrees(lon2) + 540) % 360 - 180


def gs_hgt(dist_m: float, gs_deg: float) -> float:
    """Glideslope height (m) above field at a given distance from threshold."""
    return round(dist_m * math.tan(math.radians(gs_deg)))


# ---------------------------------------------------------------------------
# ILS beacon definitions
# Columns: id, lat, lon, elev_m, hdg, gs_angle, rwy, approach_bearing
# approach_bearing matches the rounded value used in nav_beacons.ks GEO_DESTINATION calls.
# ---------------------------------------------------------------------------
ILS_BEACONS: list[tuple] = [
    # id                  lat            lon              elev   hdg    gs_angle       rwy      app_brng
    ("KSC_ILS_09",  -0.04877658, -74.70026463,   70.00,  90.4, GS_ANG,       "09",    270),
    ("KSC_ILS_27L",  0.042795,   -74.510267,     74.29, 270.0, GS_ANG,       "27L",    90),
    ("KSC_ILS_09R",  0.042762,   -74.948327,     81.27,  90.0, GS_ANG_STEEP, "09R",   270),
    ("KSC_ILS_27R",  0.04283534, -74.83633283,   93.00, 270.0, GS_ANG_STEEP, "27R",    90),
    ("ISL_ILS_09",  -1.517254,   -71.9515,      134.00,  89.0, GS_ANG,       "IS09",  270),
    ("ISL_ILS_27",  -1.516002,   -71.8566,      134.00, 269.0, GS_ANG,       "IS27",   90),
    ("DAF_ILS_36",  -6.5825,    -144.0405556,   822.00,   0.4, GS_ANG,       "DA36",  180),
    ("DAF_ILS_18",  -6.4666667, -144.0388889,   822.00, 180.4, GS_ANG,       "DA18",    0),
    ("POL_ILS_34",  72.5169754,  -78.4949417,    31.50, 337.0, GS_ANG,       "POR34", 157),
]

# Approach fix distances per ILS (km), matching nav_beacons.ks
APPROACH_DISTS: dict[str, list[int]] = {
    "KSC_ILS_09":  [60, 30, 15],
    "KSC_ILS_27L": [60, 30, 15],
    "KSC_ILS_09R": [60, 30, 15],
    "KSC_ILS_27R": [60, 30, 15],
    "ISL_ILS_09":  [30, 15],
    "ISL_ILS_27":  [30, 15],
    "DAF_ILS_36":  [60, 30, 15],
    "DAF_ILS_18":  [60, 30, 15],
    "POL_ILS_34":  [60, 30, 15],
}

# Approach fix ID templates.  Keys must match REGISTER_BEACON calls in nav_beacons.ks.
# Format: {apt}_{IAF|FAF}_{rwy_tag}[_{dist_km}]
# rwy_tag is the runway ID without airport prefix (09, 27L, IS09 → 09, etc.)
APPROACH_FIX_IDS: dict[str, dict[int, str]] = {
    # ILS id         dist_km → beacon_id
    "KSC_ILS_09":  {60: "KSC_IAF_09_60",  30: "KSC_IAF_09_30",  15: "KSC_FAF_09"},
    "KSC_ILS_27L": {60: "KSC_IAF_27L_60", 30: "KSC_IAF_27L_30", 15: "KSC_FAF_27L"},
    "KSC_ILS_09R": {60: "KSC_IAF_09R_60", 30: "KSC_IAF_09R_30", 15: "KSC_FAF_09R"},
    "KSC_ILS_27R": {60: "KSC_IAF_27R_60", 30: "KSC_IAF_27R_30", 15: "KSC_FAF_27R"},
    "ISL_ILS_09":  {30: "ISL_IAF_09_30",  15: "ISL_FAF_09"},
    "ISL_ILS_27":  {30: "ISL_IAF_27_30",  15: "ISL_FAF_27"},
    "DAF_ILS_36":  {60: "DAF_IAF_36_60",  30: "DAF_IAF_36_30",  15: "DAF_FAF_36"},
    "DAF_ILS_18":  {60: "DAF_IAF_18_60",  30: "DAF_IAF_18_30",  15: "DAF_FAF_18"},
    "POL_ILS_34":  {60: "POL_IAF_34_60",  30: "POL_IAF_34_30",  15: "POL_FAF_34"},
}

APPROACH_FIX_NAMES: dict[str, dict[int, str]] = {
    "KSC_ILS_09":  {60: "KSC RWY09 IAF 60km",  30: "KSC RWY09 IAF 30km",  15: "KSC RWY09 FAF 15km"},
    "KSC_ILS_27L": {60: "KSC RWY27L IAF 60km", 30: "KSC RWY27L IAF 30km", 15: "KSC RWY27L FAF 15km"},
    "KSC_ILS_09R": {60: "KSC RWY09R IAF 60km", 30: "KSC RWY09R IAF 30km", 15: "KSC RWY09R FAF 15km"},
    "KSC_ILS_27R": {60: "KSC RWY27R IAF 60km", 30: "KSC RWY27R IAF 30km", 15: "KSC RWY27R FAF 15km"},
    "ISL_ILS_09":  {30: "ISL RWY09 IAF 30km",  15: "ISL RWY09 FAF 15km"},
    "ISL_ILS_27":  {30: "ISL RWY27 IAF 30km",  15: "ISL RWY27 FAF 15km"},
    "DAF_ILS_36":  {60: "DAF RWY36 IAF 60km",  30: "DAF RWY36 IAF 30km",  15: "DAF RWY36 FAF 15km"},
    "DAF_ILS_18":  {60: "DAF RWY18 IAF 60km",  30: "DAF RWY18 IAF 30km",  15: "DAF RWY18 FAF 15km"},
    "POL_ILS_34":  {60: "POL RWY34 IAF 60km",  30: "POL RWY34 IAF 30km",  15: "POL RWY34 FAF 15km"},
}

# ---------------------------------------------------------------------------
# VOR beacons
# ---------------------------------------------------------------------------
VOR_BEACONS: list[tuple] = [
    # id         lat      lon       elev   name
    ("KSC_VOR", -0.097, -74.557,   70.0,  "KSC Area VOR"),
]

# ---------------------------------------------------------------------------
# Airport waypoints (WPT_APT_*)
# Source: NavInstruments PluginData/defaultRunways.cfg and mod configs.
# ---------------------------------------------------------------------------
AIRPORT_WPTS: list[tuple] = [
    # id                 lat           lon         elev    name
    ("WPT_APT_BAI",  20.604522,  -146.512375,  415.1,  "Airport Baikerbanur"),
    ("WPT_APT_CAK",  24.906246,   -83.628700,   74.4,  "Airport Cape Kerman"),
    ("WPT_APT_DAF",  -6.522210,  -144.039633,  821.8,  "Airport Desert Airfield"),
    ("WPT_APT_DUA", -39.298014,   116.239074,  455.3,  "Airport Dununda"),
    ("WPT_APT_HAA", -56.296957,   -10.982065, 3952.6,  "Airport Harvester"),
    ("WPT_APT_HAZ", -14.226728,   155.240608,   23.6,  "Airport Hazard"),
    ("WPT_APT_ISL",  -1.516628,   -71.904050,  134.0,  "Airport Island Airstrip"),
    ("WPT_APT_JEJ",   6.954471,   -77.938023,  758.9,  "Airport Jeb's Junkyard"),
    ("WPT_APT_KAM",  36.181665,    10.601823,  619.9,  "Airport Kamberwick"),
    ("WPT_APT_KAT", -37.093286,   -71.015974,  204.7,  "Airport Kerman Atoll"),
    ("WPT_APT_KER", -89.872303,  -173.724892,   33.8,  "Airport Kermundsen"),
    ("WPT_APT_KOJ",   6.060789,  -141.999592,  763.6,  "Airport Kojave Sands"),
    ("WPT_APT_KOL",  -4.159510,   -72.109344,   23.6,  "Airport Kola Island"),
    ("WPT_APT_KSC",  -0.049418,   -74.608892,   70.0,  "Airport Kerbal Space Center"),
    ("WPT_APT_KSC2", 20.582899,  -146.511597,  425.0,  "Airport KSC2"),
    ("WPT_APT_MEE",  37.869177,  -109.843826,   21.1,  "Airport Meeda"),
    ("WPT_APT_NYE",   5.746073,   108.749939,  320.4,  "Airport Nye"),
    ("WPT_APT_POL",  72.563313,   -78.560062,   31.6,  "Airport Polar"),
    ("WPT_APT_RND",  -6.023209,    99.508574, 1185.5,  "Airport Round"),
    ("WPT_APT_SND",  -8.152343,   -42.398731,   24.7,  "Airport Sandy Island"),
    ("WPT_APT_SOF", -47.006428,  -140.979218,   79.6,  "Airport South Field"),
    ("WPT_APT_SLK", -37.255462,    52.493863,   70.7,  "Airport South Lake"),
    ("WPT_APT_TSC",  17.240037,    88.736610,   61.5,  "Airport TSC"),
    ("WPT_APT_UBD",  38.507629,  -149.628914,  501.2,  "Airport Uberdam"),
    ("WPT_APT_XXX", -60.534782,    39.204119, 1371.3,  "Airport XXX"),
]

# ---------------------------------------------------------------------------
# Custom waypoints (WPT_KSC_*)
# ---------------------------------------------------------------------------
CUSTOM_WPTS: list[tuple] = [
    # id                          lat     lon      elev   name
    ("WPT_KSC_ISLAND_MID",      -0.78, -73.22, 1500.0,  "KSC-Island Mid"),
    ("WPT_KSC_NORTH",            0.50, -74.57, 1500.0,  "KSC North"),
    ("WPT_KSC_ISLAND_MID_HIGH", -0.78, -73.22, 6000.0,  "KSC-Island Mid High"),
]

# ---------------------------------------------------------------------------
# Approach plate definitions
# Matches PLATE_IDS order and fix sequences from nav_beacons.ks.
# ---------------------------------------------------------------------------
PLATES: list[dict] = [
    {"id": "PLATE_KSC_ILS09",        "ils": "KSC_ILS_09",  "vapp": 75,
     "fixes": ["KSC_IAF_09_60",  "KSC_IAF_09_30",  "KSC_FAF_09"]},
    {"id": "PLATE_KSC_ILS09_SHORT",  "ils": "KSC_ILS_09",  "vapp": 75,
     "fixes": ["KSC_IAF_09_30",  "KSC_FAF_09"]},
    {"id": "PLATE_KSC_ILS09R",       "ils": "KSC_ILS_09R", "vapp": 75,
     "fixes": ["KSC_IAF_09R_60", "KSC_IAF_09R_30", "KSC_FAF_09R"]},
    {"id": "PLATE_KSC_ILS09R_SHORT", "ils": "KSC_ILS_09R", "vapp": 75,
     "fixes": ["KSC_IAF_09R_30", "KSC_FAF_09R"]},
    {"id": "PLATE_KSC_ILS27L",       "ils": "KSC_ILS_27L", "vapp": 75,
     "fixes": ["KSC_IAF_27L_60", "KSC_IAF_27L_30", "KSC_FAF_27L"]},
    {"id": "PLATE_KSC_ILS27L_SHORT", "ils": "KSC_ILS_27L", "vapp": 75,
     "fixes": ["KSC_IAF_27L_30", "KSC_FAF_27L"]},
    {"id": "PLATE_KSC_ILS27R",       "ils": "KSC_ILS_27R", "vapp": 75,
     "fixes": ["KSC_IAF_27R_60", "KSC_IAF_27R_30", "KSC_FAF_27R"]},
    {"id": "PLATE_KSC_ILS27R_SHORT", "ils": "KSC_ILS_27R", "vapp": 75,
     "fixes": ["KSC_IAF_27R_30", "KSC_FAF_27R"]},
    {"id": "PLATE_ISL_ILS09",        "ils": "ISL_ILS_09",  "vapp": 70,
     "fixes": ["ISL_IAF_09_30",  "ISL_FAF_09"]},
    {"id": "PLATE_ISL_ILS27",        "ils": "ISL_ILS_27",  "vapp": 70,
     "fixes": ["ISL_IAF_27_30",  "ISL_FAF_27"]},
    {"id": "PLATE_DAF_ILS36",        "ils": "DAF_ILS_36",  "vapp": 70,
     "fixes": ["DAF_IAF_36_60",  "DAF_IAF_36_30",  "DAF_FAF_36"]},
    {"id": "PLATE_DAF_ILS36_SHORT",  "ils": "DAF_ILS_36",  "vapp": 70,
     "fixes": ["DAF_IAF_36_30",  "DAF_FAF_36"]},
    {"id": "PLATE_DAF_ILS18",        "ils": "DAF_ILS_18",  "vapp": 70,
     "fixes": ["DAF_IAF_18_60",  "DAF_IAF_18_30",  "DAF_FAF_18"]},
    {"id": "PLATE_DAF_ILS18_SHORT",  "ils": "DAF_ILS_18",  "vapp": 70,
     "fixes": ["DAF_IAF_18_30",  "DAF_FAF_18"]},
    {"id": "PLATE_POL_ILS34",        "ils": "POL_ILS_34",  "vapp": 70,
     "fixes": ["POL_IAF_34_60",  "POL_IAF_34_30",  "POL_FAF_34"]},
    {"id": "PLATE_POL_ILS34_SHORT",  "ils": "POL_ILS_34",  "vapp": 70,
     "fixes": ["POL_IAF_34_30",  "POL_FAF_34"]},
]


# ---------------------------------------------------------------------------
# Build full beacon row list
# ---------------------------------------------------------------------------
def build_beacon_rows() -> list[dict]:
    rows: list[dict] = []

    # ILS beacons
    for (id_, lat, lon, elev, hdg, gs_angle, rwy, _) in ILS_BEACONS:
        rows.append(dict(id=id_, type="ILS", lat=round(lat, 7), lon=round(lon, 7),
                         alt_asl=round(elev, 2), name=id_, rwy=rwy,
                         hdg=round(hdg, 4), gs_angle=round(gs_angle, 4)))

    # VOR
    for (id_, lat, lon, elev, name) in VOR_BEACONS:
        rows.append(dict(id=id_, type="VOR", lat=round(lat, 7), lon=round(lon, 7),
                         alt_asl=round(elev, 2), name=name, rwy="", hdg=0, gs_angle=0))

    # Approach fixes (computed via geo_destination)
    for (ils_id, thr_lat, thr_lon, thr_elev, _, gs_angle, rwy, app_brng) in ILS_BEACONS:
        dists_km = APPROACH_DISTS[ils_id]
        for dist_km in dists_km:
            dist_m    = dist_km * 1000
            is_faf    = (dist_km == dists_km[-1])
            fix_id    = APPROACH_FIX_IDS[ils_id][dist_km]
            fix_name  = APPROACH_FIX_NAMES[ils_id][dist_km]
            fix_type  = "FAF" if is_faf else "IAF"
            dest_lat, dest_lon = geo_destination(thr_lat, thr_lon, app_brng, dist_m)
            if is_faf:
                alt = thr_elev + gs_hgt(dist_m, gs_angle)
            else:
                alt = thr_elev + gs_hgt(dist_m, gs_angle) - BELOW_GS_M
            rows.append(dict(id=fix_id, type=fix_type,
                             lat=round(dest_lat, 7), lon=round(dest_lon, 7),
                             alt_asl=round(alt, 2), name=fix_name, rwy=rwy,
                             hdg=0, gs_angle=0))

    # Airport waypoints
    for (id_, lat, lon, elev, name) in AIRPORT_WPTS:
        rows.append(dict(id=id_, type="WPT", lat=round(lat, 7), lon=round(lon, 7),
                         alt_asl=round(elev, 2), name=name, rwy="", hdg=0, gs_angle=0))

    # Custom waypoints
    for (id_, lat, lon, elev, name) in CUSTOM_WPTS:
        rows.append(dict(id=id_, type="WPT", lat=round(lat, 7), lon=round(lon, 7),
                         alt_asl=round(elev, 2), name=name, rwy="", hdg=0, gs_angle=0))

    return rows


def build_plate_rows() -> list[dict]:
    rows: list[dict] = []
    for plate in PLATES:
        for seq, fix_id in enumerate(plate["fixes"]):
            rows.append(dict(plate_id=plate["id"], sequence=seq,
                             beacon_id=fix_id, vapp=plate["vapp"],
                             ils_id=plate["ils"]))
    return rows


def write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> None:
    script_dir = Path(__file__).parent
    out_dir    = script_dir / ".." / "web" / "data"

    beacon_rows = build_beacon_rows()
    plate_rows  = build_plate_rows()

    beacons_path = out_dir / "ifc_beacons.csv"
    plates_path  = out_dir / "ifc_plates.csv"

    n_b = write_csv(beacons_path, ["id", "type", "lat", "lon", "alt_asl", "name", "rwy", "hdg", "gs_angle"], beacon_rows)
    n_p = write_csv(plates_path,  ["plate_id", "sequence", "beacon_id", "vapp", "ils_id"],                   plate_rows)

    print(f"Wrote {n_b} beacons -> {beacons_path.resolve()}")
    print(f"Wrote {n_p} plate rows -> {plates_path.resolve()}")

    # Summary
    ils_count = sum(1 for r in beacon_rows if r["type"] == "ILS")
    iaf_count = sum(1 for r in beacon_rows if r["type"] == "IAF")
    faf_count = sum(1 for r in beacon_rows if r["type"] == "FAF")
    apt_count = sum(1 for r in beacon_rows if r["type"] == "WPT" and r["id"].startswith("WPT_APT_"))
    wpt_count = sum(1 for r in beacon_rows if r["type"] == "WPT" and not r["id"].startswith("WPT_APT_"))
    print(f"  ILS={ils_count}  IAF={iaf_count}  FAF={faf_count}  airports={apt_count}  waypoints={wpt_count}")


if __name__ == "__main__":
    main()
