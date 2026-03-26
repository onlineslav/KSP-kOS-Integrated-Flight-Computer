#!/usr/bin/env python3
"""
generate_static_nav_csv.py

Parses nav_beacons.ks and nav_custom_wpts.ks directly to generate
ifc_beacons.csv and ifc_plates.csv for the web viewer.

No hardcoded nav data — any changes to the .ks source files are picked
up automatically when this script runs (or when Reload is clicked in
the web viewer via server.py).
"""

from __future__ import annotations
import csv
import math
import re
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_SCRIPT_DIR  = Path(__file__).parent
_WEB_DATA    = _SCRIPT_DIR / ".." / "web" / "data"
_IFC_ROOT    = _SCRIPT_DIR / ".." / ".." / ".."  # -> Integrated Flight Computer/
_NAV_BEACONS = _IFC_ROOT / "nav" / "nav_beacons.ks"
_NAV_WPTS    = _IFC_ROOT / "nav" / "nav_custom_wpts.ks"

KERBIN_R_M = 600_000.0


# ---------------------------------------------------------------------------
# Geodesic math  (matches kOS GEO_DESTINATION exactly)
# ---------------------------------------------------------------------------
def _geo_destination(lat: float, lon: float, bearing_deg: float, dist_m: float):
    d    = dist_m / KERBIN_R_M
    b    = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) +
                     math.cos(lat1) * math.sin(d) * math.cos(b))
    lon2 = lon1 + math.atan2(
        math.sin(b) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), (math.degrees(lon2) + 540) % 360 - 180


# ---------------------------------------------------------------------------
# kOS source pre-processing
# ---------------------------------------------------------------------------
def _preprocess(path: Path) -> str:
    """Strip comments, collapse newlines inside parens → one logical line each."""
    lines = []
    for line in path.read_text(encoding="utf-8").splitlines():
        for marker in ("//", "\\"):
            idx = line.find(marker)
            if idx >= 0:
                line = line[:idx]
        lines.append(line)
    text = "\n".join(lines)

    # Collapse whitespace inside parentheses so multi-line calls become one line.
    result, depth = [], 0
    for ch in text:
        if ch == "(":
            depth += 1
            result.append(ch)
        elif ch == ")":
            depth -= 1
            result.append(ch)
        elif ch in "\n\r\t" and depth > 0:
            if result and result[-1] != " ":
                result.append(" ")
        else:
            result.append(ch)
    return "".join(result)


# ---------------------------------------------------------------------------
# Symbol-table evaluator
# Understands: numeric literals, LATLNG, GEO_DESTINATION, ROUND, TAN,
#              and simple +/- arithmetic over known variables.
# ---------------------------------------------------------------------------
class _Env:
    def __init__(self):
        self._num: dict[str, float] = {}
        self._ll:  dict[str, tuple] = {}

    # ---- setters ----
    def set_num(self, name: str, v: float):        self._num[name.lower()] = v
    def set_ll(self, name: str, lat, lon):         self._ll[name.lower()]  = (lat, lon)

    # ---- getters ----
    def get_num(self, name: str):                  return self._num.get(name.lower())
    def get_ll(self, name: str):                   return self._ll.get(name.lower())

    # ---- expression evaluators ----
    def eval_num(self, expr: str):
        expr = expr.strip()
        # Literal float
        try:
            return float(expr)
        except ValueError:
            pass
        # Known variable
        v = self.get_num(expr)
        if v is not None:
            return v
        # ROUND(expr, digits)
        m = re.match(r"ROUND\s*\((.+),\s*(\d+)\s*\)$", expr, re.IGNORECASE)
        if m:
            inner = self.eval_num(m.group(1))
            if inner is not None:
                return round(inner, int(m.group(2)))
        # TAN(expr)
        m = re.match(r"TAN\s*\((.+)\)$", expr, re.IGNORECASE)
        if m:
            inner = self.eval_num(m.group(1))
            if inner is not None:
                return math.tan(math.radians(inner))
        # Multiplication: a * b
        m = re.match(r"^(.+?)\s*\*\s*(.+)$", expr)
        if m:
            a, b = self.eval_num(m.group(1)), self.eval_num(m.group(2))
            if a is not None and b is not None:
                return a * b
        # Additive chain: a + b - c  (split on +/- not inside a number)
        tokens = re.split(r"(?<!\d)(?=[+\-])", expr)
        if len(tokens) > 1:
            total = 0.0
            for tok in tokens:
                tok = tok.strip()
                if not tok:
                    continue
                sign = -1.0 if tok.startswith("-") else 1.0
                tok  = tok.lstrip("+-").strip()
                v    = self.eval_num(tok)
                if v is None:
                    return None
                total += sign * v
            return total
        return None

    def eval_ll(self, expr: str):
        expr = expr.strip()
        m = re.match(r"LATLNG\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)$", expr, re.IGNORECASE)
        if m:
            return float(m.group(1)), float(m.group(2))
        return self.get_ll(expr)


# ---------------------------------------------------------------------------
# LOCAL variable parser
# ---------------------------------------------------------------------------
_LOCAL_RE = re.compile(
    r"^\s*LOCAL\s+(\w+)\s+IS\s+(.+)\.\s*$",
    re.IGNORECASE | re.MULTILINE,
)

def _parse_locals(text: str, env: _Env):
    for m in _LOCAL_RE.finditer(text):
        name = m.group(1)
        rhs  = m.group(2).strip()

        # Try LATLNG or known ll variable
        ll = env.eval_ll(rhs)
        if ll is not None:
            env.set_ll(name, *ll)
            continue

        # GEO_DESTINATION(ll_var, bearing, dist)
        gd = re.match(
            r"GEO_DESTINATION\s*\(\s*(\w+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)$",
            rhs, re.IGNORECASE,
        )
        if gd:
            src = env.get_ll(gd.group(1))
            if src is not None:
                lat2, lon2 = _geo_destination(
                    src[0], src[1], float(gd.group(2)), float(gd.group(3))
                )
                env.set_ll(name, lat2, lon2)
            continue

        # Numeric expression
        v = env.eval_num(rhs)
        if v is not None:
            env.set_num(name, v)


# ---------------------------------------------------------------------------
# REGISTER_BEACON extractor
# ---------------------------------------------------------------------------
# Matches: REGISTER_BEACON(MAKE_BEACON("ID", BTYPE_*, <ll>, <alt>, LEXICON(...)))
_BEACON_RE = re.compile(
    r'REGISTER_BEACON\s*\(\s*MAKE_BEACON\s*\(\s*"([^"]+)"\s*,\s*(\w+)\s*,\s*'
    r'(LATLNG\s*\([^)]+\)|\w+)\s*,\s*([^,]+?)\s*,\s*LEXICON\s*\(([^)]*)\)\s*\)\s*\)',
    re.IGNORECASE,
)
# Matches key-value pairs inside a LEXICON body
_LEX_KV_RE = re.compile(r'"([^"]+)"\s*,\s*([^,)]+)', re.IGNORECASE)


def _parse_lexicon(body: str, env: _Env) -> dict:
    out = {}
    for m in _LEX_KV_RE.finditer(body):
        key   = m.group(1).strip().lower()
        val_s = m.group(2).strip()
        v     = env.eval_num(val_s)
        out[key] = v if v is not None else val_s.strip('"')
    return out


def _extract_beacons(text: str, env: _Env) -> list[dict]:
    beacons = []
    for m in _BEACON_RE.finditer(text):
        bid   = m.group(1)
        btype = m.group(2).upper().replace("BTYPE_", "")
        ll    = env.eval_ll(m.group(3).strip())
        alt   = env.eval_num(m.group(4).strip()) or 0.0
        kv    = _parse_lexicon(m.group(5), env)

        if ll is None:
            continue  # can't resolve position — skip

        beacons.append({
            "id":       bid,
            "type":     btype,
            "lat":      round(ll[0], 7),
            "lon":      round(ll[1], 7),
            "alt_asl":  round(alt, 2),
            "name":     kv.get("name", bid),
            "rwy":      kv.get("rwy", kv.get("runway", "")),
            "hdg":      round(float(kv.get("hdg", 0) or 0), 4),
            "gs_angle": round(float(kv.get("gs_angle", 0) or 0), 4),
        })
    return beacons


# ---------------------------------------------------------------------------
# MAKE_PLATE / PLATE_IDS extractor
# ---------------------------------------------------------------------------
# Matches: GLOBAL PLATE_X IS MAKE_PLATE("name", "ils_id", LIST(...), vapp, ...)
_PLATE_RE = re.compile(
    r'GLOBAL\s+(\w+)\s+IS\s+MAKE_PLATE\s*\(\s*"([^"]+)"\s*,\s*"([^"]+)"\s*,'
    r'\s*LIST\s*\(([^)]*)\)\s*,\s*([\d.]+)',
    re.IGNORECASE,
)
_PLATE_IDS_RE = re.compile(
    r"GLOBAL\s+PLATE_IDS\s+IS\s+LIST\s*\(([^)]*)\)",
    re.IGNORECASE,
)


def _extract_plates(text: str) -> tuple[list[dict], list[str]]:
    plates = []
    for m in _PLATE_RE.finditer(text):
        fixes = [f.strip().strip('"') for f in m.group(4).split(",") if f.strip().strip('"')]
        plates.append({
            "var":    m.group(1).upper(),
            "ils_id": m.group(3),
            "vapp":   float(m.group(5)),
            "fixes":  fixes,
        })

    plate_ids = []
    m = _PLATE_IDS_RE.search(text)
    if m:
        plate_ids = [p.strip().strip('"') for p in m.group(1).split(",") if p.strip().strip('"')]

    return plates, plate_ids


# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------
def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main() -> None:
    env = _Env()

    ks_files = [p for p in (_NAV_BEACONS, _NAV_WPTS) if p.exists()]
    missing  = [p for p in (_NAV_BEACONS, _NAV_WPTS) if not p.exists()]
    for p in missing:
        print(f"Warning: {p} not found — skipping")

    # Pre-process all files and parse LOCAL variable declarations first
    # so computed positions (GEO_DESTINATION) are available when extracting beacons.
    texts = {p: _preprocess(p) for p in ks_files}
    for p in ks_files:
        _parse_locals(texts[p], env)

    # Extract beacons from all files, deduplicated by id
    all_beacons: list[dict] = []
    seen: set[str] = set()
    for p in ks_files:
        for b in _extract_beacons(texts[p], env):
            if b["id"] not in seen:
                all_beacons.append(b)
                seen.add(b["id"])

    # Extract plates and plate ordering from nav_beacons.ks only
    plates_raw, plate_ids = _extract_plates(texts[_NAV_BEACONS]) if _NAV_BEACONS in texts else ([], [])
    plate_by_var = {p["var"]: p for p in plates_raw}

    plate_rows: list[dict] = []
    for pid in plate_ids:
        plate = plate_by_var.get(pid.upper())
        if not plate:
            continue
        for seq, fix_id in enumerate(plate["fixes"]):
            plate_rows.append({
                "plate_id":  pid,
                "sequence":  seq,
                "beacon_id": fix_id,
                "vapp":      plate["vapp"],
                "ils_id":    plate["ils_id"],
            })

    # Write CSVs
    beacons_path = _WEB_DATA / "ifc_beacons.csv"
    plates_path  = _WEB_DATA / "ifc_plates.csv"

    n_b = _write_csv(
        beacons_path,
        ["id", "type", "lat", "lon", "alt_asl", "name", "rwy", "hdg", "gs_angle"],
        all_beacons,
    )
    n_p = _write_csv(
        plates_path,
        ["plate_id", "sequence", "beacon_id", "vapp", "ils_id"],
        plate_rows,
    )

    ils = sum(1 for b in all_beacons if b["type"] == "ILS")
    iaf = sum(1 for b in all_beacons if b["type"] == "IAF")
    faf = sum(1 for b in all_beacons if b["type"] == "FAF")
    apt = sum(1 for b in all_beacons if b["type"] == "WPT" and b["id"].startswith("WPT_APT_"))
    wpt = sum(1 for b in all_beacons if b["type"] == "WPT" and not b["id"].startswith("WPT_APT_"))

    print(f"Wrote {n_b} beacons -> {beacons_path.resolve()}")
    print(f"Wrote {n_p} plate rows -> {plates_path.resolve()}")
    print(f"  ILS={ils}  IAF={iaf}  FAF={faf}  airports={apt}  waypoints={wpt}")


if __name__ == "__main__":
    main()
