"use strict";

import * as THREE from "three";
import { OrbitControls }                from "three/addons/controls/OrbitControls.js";
import { CSS2DRenderer, CSS2DObject }   from "three/addons/renderers/CSS2DRenderer.js";

// ============================================================
// Kerbin Navigation Chart — Three.js 3D viewer
// ============================================================

const KERBIN_R_M = 600_000;
const WORLD_W    = 3600;          // world units spanning lon -180..+180
const WORLD_H    = 1800;          // world units spanning lat  +90..-90
// True-to-scale: 1 world unit = Kerbin equatorial circumference / WORLD_W metres
// At height exaggeration = 1 this is geometrically correct (1m up = 1m on the map).
const METRES_PER_WORLD_UNIT = (2 * Math.PI * KERBIN_R_M) / WORLD_W; // ≈ 1047 m/unit
const ALT_SCALE = 1 / METRES_PER_WORLD_UNIT;                         // ≈ 0.000955
const PLANE_Y    = 0;             // Y of the flat map plane
const LIFT       = 0.8;           // slight lift to avoid Z-fighting

// ---- DOM handles ----
const mapPathInput     = document.getElementById("mapPath");
const beaconsPathInput = document.getElementById("beaconsPath");
const platesPathInput  = document.getElementById("platesPath");
const reloadBtn        = document.getElementById("reloadBtn");
const showLabelsEl     = document.getElementById("showLabels");
const showAltitudeEl   = document.getElementById("showAltitude");
const showAltStemsEl   = document.getElementById("showAltStems");
const heightExagEl     = document.getElementById("heightExaggeration");
const markerSizeEl     = document.getElementById("markerSize");
const plateSelectEl    = document.getElementById("plateSelect");
const showProfileEl    = document.getElementById("showPlateProfile");
const profileCanvas    = document.getElementById("profileCanvas");
const profileCtx       = profileCanvas ? profileCanvas.getContext("2d") : null;
const profileStatusEl  = document.getElementById("profileStatus");
const cursorReadoutEl  = document.getElementById("cursorReadout");
const statusEl         = document.getElementById("status");
const tooltipEl        = document.getElementById("tooltip");
const showHideAllBtn   = document.getElementById("showHideAllBtn");
const logFileInput     = document.getElementById("logFileInput");
const logLayerToggle   = document.getElementById("logLayerToggle");
const logStatusEl      = document.getElementById("logStatus");
const planModeBtn      = document.getElementById("planModeBtn");
const planClearBtn     = document.getElementById("planClearBtn");
const planExportBtn    = document.getElementById("planExportBtn");
const planListEl       = document.getElementById("planList");
const planStatusEl     = document.getElementById("planStatus");
const mapContainer     = document.getElementById("mapContainer");

const layerCbs = {
  ILS:         document.getElementById("layerILS"),
  IAF:         document.getElementById("layerIAF"),
  FAF:         document.getElementById("layerFAF"),
  VOR:         document.getElementById("layerVOR"),
  APT:         document.getElementById("layerAPT"),
  WPT:         document.getElementById("layerWPT"),
  centerlines: document.getElementById("layerCenterlines"),
};

// ---- App state ----
const state = {
  beacons:       [],
  beaconById:    new Map(),
  plates:        new Map(),
  selectedPlate: "",
  hovered:       null,
};
const logState  = { rows: [] };
const planState = { waypoints: [], active: false };

// ============================================================
// Three.js scene setup
// ============================================================

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1620);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 25000);
camera.position.set(0, 2600, 700);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(mapContainer.clientWidth, mapContainer.clientHeight);
mapContainer.appendChild(renderer.domElement);

// CSS2D overlay for labels
const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.cssText =
  "position:absolute;top:0;left:0;pointer-events:none;";
mapContainer.appendChild(labelRenderer.domElement);

// Orbit controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping    = true;
controls.dampingFactor    = 0.07;
controls.screenSpacePanning = false;  // pan stays in the XZ map plane
controls.minDistance      = 40;
controls.maxDistance      = 9000;
controls.maxPolarAngle    = Math.PI / 2 - 0.01;
controls.target.set(0, PLANE_Y, 0);
controls.update();

// Keep the orbit target locked to the map plane at all times
controls.addEventListener("change", () => {
  if (controls.target.y !== PLANE_Y) {
    controls.target.y = PLANE_Y;
  }
});

// Lights (for future terrain normals)
scene.add(new THREE.AmbientLight(0xffffff, 1.0));
const sunLight = new THREE.DirectionalLight(0xffffff, 0.4);
sunLight.position.set(600, 1200, -400);
scene.add(sunLight);

// Scene groups
const mapGroup    = new THREE.Group();
const gridGroup   = new THREE.Group();
const clGroup     = new THREE.Group();   // approach centerlines (ground track)
const gsGroup     = new THREE.Group();   // ILS glideslope paths (3D)
const beaconGroup = new THREE.Group();
const stemGroup   = new THREE.Group();
const logGroup    = new THREE.Group();
const plateGroup  = new THREE.Group();
const planGroup   = new THREE.Group();
scene.add(mapGroup, gridGroup, clGroup, gsGroup, beaconGroup, stemGroup,
          logGroup, plateGroup, planGroup);

// Raycaster for plane intersection + hover
const raycaster  = new THREE.Raycaster();
const mouseNDC   = new THREE.Vector2();
const mapPlane   = new THREE.Plane(new THREE.Vector3(0, 1, 0), -PLANE_Y);

// ============================================================
// Coordinate utilities
// ============================================================

function lonLatAltToVec3(lon, lat, altM = 0) {
  const exag = clamp(Number(heightExagEl?.value) || 6, 1, 20);
  return new THREE.Vector3(
    (lon / 180) * (WORLD_W / 2),
    PLANE_Y + altM * ALT_SCALE * exag,
    -(lat / 90) * (WORLD_H / 2),
  );
}

function vec3ToLonLat(v) {
  return {
    lon: (v.x / (WORLD_W / 2)) * 180,
    lat: -(v.z / (WORLD_H / 2)) * 90,
  };
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function geoDestination(lat, lon, bearingDeg, distM) {
  const d  = distM / KERBIN_R_M;
  const b  = bearingDeg * Math.PI / 180;
  const φ1 = lat * Math.PI / 180;
  const λ1 = lon * Math.PI / 180;
  const φ2 = Math.asin(Math.sin(φ1) * Math.cos(d) + Math.cos(φ1) * Math.sin(d) * Math.cos(b));
  const λ2 = λ1 + Math.atan2(
    Math.sin(b) * Math.sin(d) * Math.cos(φ1),
    Math.cos(d) - Math.sin(φ1) * Math.sin(φ2),
  );
  return {
    lat: φ2 * 180 / Math.PI,
    lon: ((λ2 * 180 / Math.PI) + 540) % 360 - 180,
  };
}

function geoDistanceM(lat1, lon1, lat2, lon2) {
  const p = Math.PI / 180;
  const a = Math.sin((lat2 - lat1) * p / 2) ** 2
    + Math.cos(lat1 * p) * Math.cos(lat2 * p) * Math.sin((lon2 - lon1) * p / 2) ** 2;
  return 2 * KERBIN_R_M * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

// ============================================================
// Beacon appearance helpers
// ============================================================

function layerVisible(type, id) {
  switch (type) {
    case "ILS": return layerCbs.ILS.checked;
    case "IAF": return layerCbs.IAF.checked;
    case "FAF": return layerCbs.FAF.checked;
    case "VOR": return layerCbs.VOR.checked;
    case "WPT":
      if (id && id.startsWith("WPT_APT_")) return layerCbs.APT.checked;
      return layerCbs.WPT.checked;
    default: return true;
  }
}

function beaconColor(type, id) {
  switch (type) {
    case "ILS": return "#ffd166";
    case "IAF": return "#90be6d";
    case "FAF": return "#ff9f1c";
    case "VOR": return "#4cc9f0";
    case "WPT": return (id && id.startsWith("WPT_APT_")) ? "#a8ff78" : "#c9b8ff";
    default:    return "#e0e0e0";
  }
}

// ============================================================
// Sprite texture cache
// ============================================================

const _spriteCache = new Map();

function makeBeaconTexture(type, id) {
  const isApt = id && id.startsWith("WPT_APT_");
  const key   = type + (isApt ? "_APT" : "");
  if (_spriteCache.has(key)) return _spriteCache.get(key);

  const SZ  = 96;
  const c   = document.createElement("canvas");
  c.width   = SZ;
  c.height  = SZ;
  const cx  = SZ / 2;
  const cy  = SZ / 2;
  const r   = SZ * 0.34;
  const k   = c.getContext("2d");

  k.clearRect(0, 0, SZ, SZ);

  switch (type) {
    case "ILS": {
      k.beginPath();
      k.moveTo(cx,          cy - r * 1.25);
      k.lineTo(cx + r * 0.8, cy);
      k.lineTo(cx,          cy + r * 1.25);
      k.lineTo(cx - r * 0.8, cy);
      k.closePath();
      k.fillStyle = "#ffd166";
      k.fill();
      k.strokeStyle = "rgba(0,0,0,0.45)";
      k.lineWidth = 1.5;
      k.stroke();
      break;
    }
    case "IAF": {
      k.beginPath();
      k.moveTo(cx,              cy - r);
      k.lineTo(cx + r * 0.866,  cy + r * 0.5);
      k.lineTo(cx - r * 0.866,  cy + r * 0.5);
      k.closePath();
      k.strokeStyle = "#90be6d";
      k.lineWidth = 4;
      k.stroke();
      break;
    }
    case "FAF": {
      k.beginPath();
      k.arc(cx, cy, r, 0, Math.PI * 2);
      k.fillStyle = "#ff9f1c";
      k.fill();
      k.beginPath();
      k.moveTo(cx - r * 0.7, cy); k.lineTo(cx + r * 0.7, cy);
      k.moveTo(cx, cy - r * 0.7); k.lineTo(cx, cy + r * 0.7);
      k.strokeStyle = "rgba(0,0,0,0.5)";
      k.lineWidth = 3;
      k.stroke();
      break;
    }
    case "VOR": {
      k.beginPath();
      for (let i = 0; i < 6; i++) {
        const a  = (i / 6) * Math.PI * 2 - Math.PI / 2;
        const px = cx + Math.cos(a) * r;
        const py = cy + Math.sin(a) * r;
        if (i === 0) k.moveTo(px, py); else k.lineTo(px, py);
      }
      k.closePath();
      k.fillStyle = "rgba(76,201,240,0.12)";
      k.fill();
      k.strokeStyle = "#4cc9f0";
      k.lineWidth = 4;
      k.stroke();
      break;
    }
    case "WPT": {
      if (isApt) {
        const rs = r * 1.2;
        k.beginPath();
        k.moveTo(cx,             cy - rs);
        k.lineTo(cx + rs * 0.866, cy + rs * 0.5);
        k.lineTo(cx - rs * 0.866, cy + rs * 0.5);
        k.closePath();
        k.fillStyle = "#a8ff78";
        k.fill();
        k.strokeStyle = "rgba(0,0,0,0.3)";
        k.lineWidth = 1.5;
        k.stroke();
      } else {
        k.beginPath();
        k.arc(cx, cy, r * 0.85, 0, Math.PI * 2);
        k.fillStyle = "#c9b8ff";
        k.fill();
      }
      break;
    }
    default: {
      k.beginPath();
      k.arc(cx, cy, r, 0, Math.PI * 2);
      k.fillStyle = "#e0e0e0";
      k.fill();
    }
  }

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  _spriteCache.set(key, tex);
  return tex;
}

// Halo texture for hover highlight
let _haloTex = null;
function getHaloTexture() {
  if (_haloTex) return _haloTex;
  const SZ = 64;
  const c  = document.createElement("canvas");
  c.width  = SZ; c.height = SZ;
  const k  = c.getContext("2d");
  const g  = k.createRadialGradient(SZ/2, SZ/2, 0, SZ/2, SZ/2, SZ/2);
  g.addColorStop(0,   "rgba(255,255,255,0.25)");
  g.addColorStop(0.5, "rgba(255,255,255,0.08)");
  g.addColorStop(1,   "rgba(255,255,255,0)");
  k.fillStyle = g;
  k.fillRect(0, 0, SZ, SZ);
  _haloTex = new THREE.CanvasTexture(c);
  return _haloTex;
}

// ============================================================
// Grid
// ============================================================

function buildGrid() {
  gridGroup.clear();
  const pts = [];
  for (let lon = -180; lon <= 180; lon += 30) {
    const x = (lon / 180) * (WORLD_W / 2);
    pts.push(x, LIFT * 0.3, -WORLD_H / 2,  x, LIFT * 0.3, WORLD_H / 2);
  }
  for (let lat = -90; lat <= 90; lat += 30) {
    const z = -(lat / 90) * (WORLD_H / 2);
    pts.push(-WORLD_W / 2, LIFT * 0.3, z,  WORLD_W / 2, LIFT * 0.3, z);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  const mat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.07 });
  gridGroup.add(new THREE.LineSegments(geo, mat));
}

// ============================================================
// Map plane
// ============================================================

let _mapMaterial = null;

function setMapTexture(image) {
  mapGroup.clear();
  if (_mapMaterial) { _mapMaterial.dispose(); _mapMaterial = null; }

  if (image) {
    const tex = new THREE.Texture(image);
    tex.needsUpdate  = true;
    tex.colorSpace   = THREE.SRGBColorSpace;
    _mapMaterial = new THREE.MeshBasicMaterial({ map: tex });
  } else {
    _mapMaterial = new THREE.MeshBasicMaterial({ color: 0x0f1722 });
  }

  // PlaneGeometry in XY → rotateX(-PI/2) → lies in XZ (Y-up)
  // 256x128 segments so a heightmap displacement can be added later
  const geo  = new THREE.PlaneGeometry(WORLD_W, WORLD_H, 256, 128);
  geo.rotateX(-Math.PI / 2);
  mapGroup.add(new THREE.Mesh(geo, _mapMaterial));
}

// ============================================================
// Approach centerlines
// ============================================================

function updateCenterlines() {
  clGroup.clear();
  if (!layerCbs.centerlines.checked || !layerCbs.ILS.checked) return;

  const positions = [];
  for (const b of state.beacons) {
    if (b.type !== "ILS" || !b.hdg) continue;
    const appBrng = (b.hdg + 180) % 360;
    const far     = geoDestination(b.lat, b.lon, appBrng, 66000);
    const p1 = lonLatAltToVec3(b.lon, b.lat, 0);
    const p2 = lonLatAltToVec3(far.lon, far.lat, 0);
    p1.y += LIFT * 0.5;
    p2.y += LIFT * 0.5;
    positions.push(p1.x, p1.y, p1.z,  p2.x, p2.y, p2.z);
  }
  if (!positions.length) return;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const line = new THREE.LineSegments(geo,
    new THREE.LineDashedMaterial({
      color: 0xffc846,
      transparent: true,
      opacity: 0.25,
      dashSize: 6,
      gapSize:  5,
    }),
  );
  line.computeLineDistances();
  clGroup.add(line);
}

// ============================================================
// ILS glideslope paths
// ============================================================

function updateGlideslopes() {
  gsGroup.clear();
  if (!layerCbs.ILS.checked) return;
  if (!showAltitudeEl?.checked) return;   // flat map has no meaningful GS angle

  for (const b of state.beacons) {
    if (b.type !== "ILS" || !b.hdg || !b.gs_angle) continue;

    const gsRad   = b.gs_angle * Math.PI / 180;
    const appBrng = (b.hdg + 180) % 360;
    const maxDist = 32000;   // 32 km — typical final + base leg distance
    const N       = 40;
    const positions = [];

    for (let i = 0; i <= N; i++) {
      const d    = (i / N) * maxDist;
      const dest = geoDestination(b.lat, b.lon, appBrng, d);
      // Altitude climbs linearly away from threshold at the glideslope angle
      const altM = (Number(b.alt_asl) || 0) + d * Math.tan(gsRad);
      const v    = lonLatAltToVec3(dest.lon, dest.lat, altM);
      v.y += LIFT * 0.7;
      positions.push(v.x, v.y, v.z);
    }

    const geo  = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    gsGroup.add(new THREE.Line(geo,
      new THREE.LineBasicMaterial({ color: 0xffe08a, transparent: true, opacity: 0.55 }),
    ));
  }
}

// ============================================================
// Beacons
// ============================================================

// We track sprites so we can project them for hover
let _beaconSprites = [];  // [{beacon, sprite}]
let _labelObjects  = [];  // CSS2DObjects to remove

function updateBeacons() {
  // Remove old label objects properly
  for (const { obj } of _labelObjects) obj.parent && obj.parent.remove(obj);
  _labelObjects  = [];
  _beaconSprites = [];
  beaconGroup.clear();
  stemGroup.clear();

  const showAlt   = showAltitudeEl?.checked;
  const showStems = showAltStemsEl?.checked && showAlt;
  const showLabels = showLabelsEl?.checked;
  const mSize     = clamp(Number(markerSizeEl.value) || 6, 0.5, 16);
  const stemPts   = [];

  for (const b of state.beacons) {
    if (!layerVisible(b.type, b.id)) continue;

    const altM = showAlt ? (Number(b.alt_asl) || 0) : 0;
    const pos  = lonLatAltToVec3(b.lon, b.lat, altM);
    pos.y += LIFT;

    // Sprite marker
    const tex  = makeBeaconTexture(b.type, b.id);
    const mat  = new THREE.SpriteMaterial({ map: tex });
    const sprite = new THREE.Sprite(mat);
    sprite.position.copy(pos);
    sprite.scale.setScalar(mSize * 3);   // world-space units, perspective-attenuated
    sprite.userData.beacon = b;
    beaconGroup.add(sprite);
    _beaconSprites.push({ beacon: b, sprite });

    // Altitude stem
    if (showStems && altM > 1) {
      const base = lonLatAltToVec3(b.lon, b.lat, 0);
      base.y += LIFT * 0.5;
      stemPts.push(base.x, base.y, base.z,  pos.x, pos.y, pos.z);
    }

    // CSS2D label
    if (showLabels) {
      const div    = document.createElement("div");
      div.className = "beacon-label";
      div.textContent = b.id.replace(/^WPT_APT_/, "").replace(/^WPT_/, "");
      div.style.color = beaconColor(b.type, b.id);
      const obj = new CSS2DObject(div);
      obj.position.copy(pos);
      obj.position.x += 0.5;  // slight right offset
      beaconGroup.add(obj);
      _labelObjects.push({ obj });
    }
  }

  // Batch stems into one draw call
  if (stemPts.length) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(stemPts, 3));
    stemGroup.add(new THREE.LineSegments(geo,
      new THREE.LineBasicMaterial({ color: 0xd2e6f5, transparent: true, opacity: 0.35 }),
    ));
  }

  updateHoverHalo();
}

// Separate halo sprite for the hovered beacon
let _haloSprite = null;

function updateHoverHalo() {
  if (_haloSprite) { beaconGroup.remove(_haloSprite); _haloSprite = null; }
  if (!state.hovered) return;

  const b    = state.hovered;
  const altM = showAltitudeEl?.checked ? (Number(b.alt_asl) || 0) : 0;
  const pos  = lonLatAltToVec3(b.lon, b.lat, altM);
  pos.y += LIFT;

  const mSize = clamp(Number(markerSizeEl.value) || 6, 0.5, 16);
  _haloSprite = new THREE.Sprite(
    new THREE.SpriteMaterial({ map: getHaloTexture(), transparent: true }),
  );
  _haloSprite.position.copy(pos);
  _haloSprite.scale.setScalar(mSize * 25);   // world units, larger halo ring
  beaconGroup.add(_haloSprite);
}

// ============================================================
// Log path
// ============================================================

const PHASE_COLORS = {
  TAKEOFF:   "#44ff88",
  CRUISE:    "#5dc2ff",
  APPROACH:  "#ffd166",
  FLARE:     "#ff9f1c",
  TOUCHDOWN: "#ff6b6b",
  ROLLOUT:   "#ff6b6b",
  ASCENT:    "#c084fc",
};

function phaseColor(phase) {
  if (!phase) return "#88aacc";
  const u = phase.toUpperCase();
  for (const [k, col] of Object.entries(PHASE_COLORS)) {
    if (u.includes(k)) return col;
  }
  return "#88aacc";
}

function updateLogPath() {
  logGroup.clear();
  if (!logState.rows.length) return;
  if (logLayerToggle && !logLayerToggle.checked) return;

  const showAlt = showAltitudeEl?.checked;
  const positions = [];
  const colors    = [];
  let   prev      = null;

  for (const row of logState.rows) {
    if (!Number.isFinite(row.lat) || !Number.isFinite(row.lon)) { prev = null; continue; }
    const altM = showAlt ? (row.alt || 0) : 0;
    const p    = lonLatAltToVec3(row.lon, row.lat, altM);
    p.y += LIFT * 0.6;
    if (prev) {
      positions.push(prev.x, prev.y, prev.z,  p.x, p.y, p.z);
      const col = new THREE.Color(phaseColor(row.phase));
      colors.push(col.r, col.g, col.b,  col.r, col.g, col.b);
    }
    prev = p;
  }

  if (!positions.length) return;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute("color",    new THREE.Float32BufferAttribute(colors, 3));
  logGroup.add(new THREE.LineSegments(geo,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9, linewidth: 2 }),
  ));

  // Start / end dots
  const validRows = logState.rows.filter(r => Number.isFinite(r.lat) && Number.isFinite(r.lon));
  if (validRows.length >= 2) {
    addDot(logGroup, validRows[0],  0x44ff88, showAlt);
    addDot(logGroup, validRows[validRows.length - 1], 0xff6b6b, showAlt);
  }
}

function addDot(group, row, colorHex, showAlt) {
  const altM = showAlt ? (row.alt || 0) : 0;
  const pos  = lonLatAltToVec3(row.lon, row.lat, altM);
  pos.y += LIFT * 0.6 + 1;
  const geo  = new THREE.SphereGeometry(4, 8, 8);
  const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: colorHex }));
  mesh.position.copy(pos);
  group.add(mesh);
}

// ============================================================
// Plate route
// ============================================================

function getPlateSequence(plate) {
  if (!plate) return [];
  const seq = [...plate.fixes];
  if (plate.ils_id && plate.ils_id !== seq[seq.length - 1]) seq.push(plate.ils_id);
  return seq;
}

function updatePlateRoute() {
  plateGroup.clear();
  if (!state.selectedPlate) return;
  const plate = state.plates.get(state.selectedPlate);
  if (!plate) return;

  const showAlt = showAltitudeEl?.checked;
  const seq     = getPlateSequence(plate);
  if (!seq.length) return;

  const positions = [];
  const dots      = [];
  for (const fixId of seq) {
    const b = state.beaconById.get(fixId);
    if (!b) continue;
    const altM = showAlt ? (Number(b.alt_asl) || 0) : 0;
    const p    = lonLatAltToVec3(b.lon, b.lat, altM);
    p.y += LIFT * 0.7;
    positions.push(p.x, p.y, p.z);
    dots.push({ pos: p, isILS: b.type === "ILS" });
  }

  // Route line
  if (positions.length >= 6) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const line = new THREE.Line(geo,
      new THREE.LineDashedMaterial({ color: 0xff4646, dashSize: 8, gapSize: 5, transparent: true, opacity: 0.9 }),
    );
    line.computeLineDistances();
    plateGroup.add(line);
  }

  // Fix dots
  for (const { pos, isILS } of dots) {
    const geo  = new THREE.SphereGeometry(3.5, 8, 8);
    const mesh = new THREE.Mesh(geo,
      new THREE.MeshBasicMaterial({ color: isILS ? 0xffd166 : 0xff5a5a }),
    );
    mesh.position.copy(pos);
    plateGroup.add(mesh);
  }
}

// ============================================================
// Flight plan route
// ============================================================

function updatePlanRoute() {
  planGroup.clear();
  if (!planState.waypoints.length) return;

  const showAlt   = showAltitudeEl?.checked;
  const positions = [];

  for (const wp of planState.waypoints) {
    const altM = showAlt ? (wp.alt || 0) : 0;
    const p    = lonLatAltToVec3(wp.lon, wp.lat, altM);
    p.y += LIFT * 0.7;
    positions.push(p.x, p.y, p.z);
  }

  // Connecting line
  if (positions.length >= 6) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const line = new THREE.Line(geo,
      new THREE.LineDashedMaterial({ color: 0x00e6c8, dashSize: 6, gapSize: 5, transparent: true, opacity: 0.9 }),
    );
    line.computeLineDistances();
    planGroup.add(line);
  }

  // Waypoint dots
  for (let i = 0; i < planState.waypoints.length; i++) {
    const wp   = planState.waypoints[i];
    const altM = showAlt ? (wp.alt || 0) : 0;
    const pos  = lonLatAltToVec3(wp.lon, wp.lat, altM);
    pos.y += LIFT * 0.7 + 1;
    const geo  = new THREE.SphereGeometry(4, 8, 8);
    const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x00e6c8 }));
    mesh.position.copy(pos);
    planGroup.add(mesh);
  }
}

// ============================================================
// Hover / hit test (project sprites to screen, 2D distance)
// ============================================================

function hitTest(clientX, clientY) {
  let closest = null;
  let minDist = 22;  // pixels
  const rect  = renderer.domElement.getBoundingClientRect();

  for (const { beacon, sprite } of _beaconSprites) {
    const ndc = sprite.position.clone().project(camera);
    const sx  = (ndc.x + 1) / 2 * rect.width  + rect.left;
    const sy  = (-ndc.y + 1) / 2 * rect.height + rect.top;
    const d   = Math.hypot(clientX - sx, clientY - sy);
    if (d < minDist) { minDist = d; closest = beacon; }
  }
  return closest;
}

// ============================================================
// Tooltip
// ============================================================

function showTooltip(b, clientX, clientY) {
  if (!b) { tooltipEl.style.display = "none"; return; }
  const latStr = `${Math.abs(b.lat).toFixed(5)}° ${b.lat >= 0 ? "N" : "S"}`;
  const lonStr = `${Math.abs(b.lon).toFixed(5)}° ${b.lon >= 0 ? "E" : "W"}`;

  let html = `<div class="tt-id">${b.id}</div>`;
  html += `<div class="tt-type">${b.type}${b.rwy ? " · RWY " + b.rwy : ""}</div>`;
  if (b.name && b.name !== b.id) html += `<div class="tt-name">${b.name}</div>`;
  html += `<div class="tt-coords">${latStr} &nbsp; ${lonStr}</div>`;
  if (b.alt_asl) html += `<div class="tt-coords">ELEV ${b.alt_asl.toFixed(0)} m MSL</div>`;
  if (b.type === "ILS" && b.hdg) {
    html += `<div class="tt-coords">HDG ${b.hdg.toFixed(1)}° &nbsp;·&nbsp; GS ${b.gs_angle.toFixed(1)}°</div>`;
  }

  tooltipEl.innerHTML = html;
  tooltipEl.style.display = "block";
  const tw = tooltipEl.offsetWidth + 20;
  const th = tooltipEl.offsetHeight + 20;
  tooltipEl.style.left = (clientX + 14 + tw > window.innerWidth  ? clientX - tw : clientX + 14) + "px";
  tooltipEl.style.top  = (clientY + 14 + th > window.innerHeight ? clientY - th : clientY + 14) + "px";
}

// ============================================================
// Plate profile (2D sidebar canvas — unchanged from original)
// ============================================================

function drawPlateProfile() {
  if (!profileCtx || !profileCanvas || !profileStatusEl) return;

  const dpr = window.devicePixelRatio || 1;
  const w   = Math.max(200, Math.floor(profileCanvas.clientWidth || 420));
  const h   = Math.max(110, Math.floor(profileCanvas.clientHeight || 170));
  if (profileCanvas.width  !== Math.floor(w * dpr) ||
      profileCanvas.height !== Math.floor(h * dpr)) {
    profileCanvas.width  = Math.floor(w * dpr);
    profileCanvas.height = Math.floor(h * dpr);
  }
  profileCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  profileCtx.clearRect(0, 0, w, h);

  if (showProfileEl && !showProfileEl.checked) {
    profileStatusEl.textContent = "Profile hidden.";
    return;
  }
  if (!state.selectedPlate) {
    profileStatusEl.textContent = "Select a plate to view its altitude profile.";
    return;
  }
  const plate = state.plates.get(state.selectedPlate);
  if (!plate) { profileStatusEl.textContent = "Selected plate data not found."; return; }

  const sequence = getPlateSequence(plate);
  if (!sequence.length) { profileStatusEl.textContent = "Selected plate has no fixes."; return; }

  const points = [];
  let distM = 0;
  let prev  = null;
  for (const id of sequence) {
    const b = state.beaconById.get(id);
    if (!b) continue;
    if (prev) distM += geoDistanceM(prev.lat, prev.lon, b.lat, b.lon);
    points.push({ id, lat: b.lat, lon: b.lon, alt: Number(b.alt_asl) || 0, dist: distM / 1000 });
    prev = b;
  }
  if (points.length < 2) {
    profileStatusEl.textContent = "Need at least two valid beacons for profile.";
    return;
  }

  const minAltRaw = Math.min(...points.map(p => p.alt));
  const maxAltRaw = Math.max(...points.map(p => p.alt));
  const yMin      = Math.max(0, minAltRaw - 200);
  const yMax      = Math.max(yMin + 500, maxAltRaw + 250);
  const xMax      = Math.max(1, points[points.length - 1].dist);
  const m         = { l: 42, r: 10, t: 10, b: 22 };
  const pw        = Math.max(20, w - m.l - m.r);
  const ph        = Math.max(20, h - m.t - m.b);

  const xFor = dKm  => m.l + (dKm / xMax) * pw;
  const yFor = altM => m.t + (1 - (altM - yMin) / (yMax - yMin)) * ph;

  profileCtx.fillStyle = "#0f171f";
  profileCtx.fillRect(0, 0, w, h);

  profileCtx.strokeStyle = "rgba(170,200,220,0.22)";
  profileCtx.lineWidth   = 1;
  profileCtx.beginPath();
  profileCtx.moveTo(m.l, m.t);
  profileCtx.lineTo(m.l, m.t + ph);
  profileCtx.lineTo(m.l + pw, m.t + ph);
  profileCtx.stroke();

  profileCtx.fillStyle = "rgba(175,205,225,0.8)";
  profileCtx.font      = "11px Consolas";
  for (let i = 0; i <= 4; i++) {
    const t      = i / 4;
    const altTick = yMin + (1 - t) * (yMax - yMin);
    const y      = m.t + t * ph;
    profileCtx.strokeStyle = "rgba(170,200,220,0.12)";
    profileCtx.beginPath();
    profileCtx.moveTo(m.l, y);
    profileCtx.lineTo(m.l + pw, y);
    profileCtx.stroke();
    profileCtx.fillStyle = "rgba(175,205,225,0.8)";
    profileCtx.fillText(`${Math.round(altTick)}m`, 3, y + 3);
  }
  profileCtx.fillText(`${xMax.toFixed(1)}km`, m.l + pw - 44, h - 5);
  profileCtx.fillText("0km", m.l - 4, h - 5);

  profileCtx.strokeStyle = "rgba(255,120,120,0.95)";
  profileCtx.lineWidth   = 2;
  profileCtx.beginPath();
  points.forEach((p, i) => {
    const x = xFor(p.dist);
    const y = yFor(p.alt);
    if (i === 0) profileCtx.moveTo(x, y); else profileCtx.lineTo(x, y);
  });
  profileCtx.stroke();

  points.forEach((p, i) => {
    const x = xFor(p.dist);
    const y = yFor(p.alt);
    profileCtx.beginPath();
    profileCtx.arc(x, y, 3, 0, Math.PI * 2);
    profileCtx.fillStyle = i === points.length - 1 ? "#ffd166" : "#ff7a7a";
    profileCtx.fill();
    if (pw > 280) {
      profileCtx.fillStyle = "rgba(230,240,250,0.9)";
      profileCtx.font      = "10px Consolas";
      profileCtx.fillText(p.id, x + 4, y - 4);
    }
  });

  profileStatusEl.textContent =
    `${state.selectedPlate}  |  Dist ${xMax.toFixed(1)} km  |  Alt ${Math.round(minAltRaw)}-${Math.round(maxAltRaw)} m MSL`;
}

// ============================================================
// Plate select
// ============================================================

function rebuildPlateSelect() {
  plateSelectEl.innerHTML = "";
  const none = document.createElement("option");
  none.value = ""; none.textContent = "(none)";
  plateSelectEl.appendChild(none);
  Array.from(state.plates.keys()).sort().forEach(id => {
    const opt = document.createElement("option");
    opt.value = id; opt.textContent = id;
    plateSelectEl.appendChild(opt);
  });
  plateSelectEl.value = state.selectedPlate;
}

// ============================================================
// "Fit" / reset camera
// ============================================================

function fitView() {
  camera.position.set(0, 2600, 700);
  controls.target.set(0, 0, 0);
  controls.update();
}

// ============================================================
// Full scene refresh (call after data load or setting changes)
// ============================================================

function refreshScene() {
  updateCenterlines();
  updateGlideslopes();
  updateBeacons();
  updateLogPath();
  updatePlateRoute();
  updatePlanRoute();
  drawPlateProfile();
}

// ============================================================
// Log replay
// ============================================================

function parseCsv(text) {
  const lines = text.split(/\r?\n/).map(l => l.trim()).filter(l => l && !l.startsWith("#"));
  if (!lines.length) return [];
  const headers = lines[0].split(",").map(s => s.trim());
  return lines.slice(1).map(line => {
    const cols = line.split(",");
    const row  = {};
    for (let i = 0; i < headers.length; i++) row[headers[i]] = (cols[i] || "").trim();
    return row;
  });
}

function parseLogCsv(text) {
  return parseCsv(text).map(r => ({
    t:     Number(r.t_s)    || 0,
    phase: r.phase          || "",
    lat:   Number(r.lat_deg),
    lon:   Number(r.lon_deg),
    alt:   Number(r.alt_asl_m),
    ias:   Number(r.ias_ms) || 0,
  })).filter(r => Number.isFinite(r.lat) && Number.isFinite(r.lon) && r.lat !== 0);
}

function loadLogFile(file) {
  const reader = new FileReader();
  reader.onload = e => {
    logState.rows = parseLogCsv(e.target.result);
    if (logState.rows.length) {
      const n    = logState.rows.length;
      const tEnd = logState.rows[n - 1].t.toFixed(0);
      if (logStatusEl) logStatusEl.textContent = `${n} pts · T+0–${tEnd}s · ${file.name}`;
      updateLogPath();
    } else {
      if (logStatusEl) logStatusEl.textContent =
        "No position data. Ensure ifc_logger.ks has lat_deg/lon_deg columns.";
    }
  };
  reader.readAsText(file);
}

// ============================================================
// Flight plan editor
// ============================================================

function planRebuildList() {
  if (!planListEl) return;
  planListEl.innerHTML = "";
  if (!planState.waypoints.length) {
    planListEl.innerHTML = "<div class='plan-empty'>No waypoints. Enter plan mode and click map.</div>";
    return;
  }
  planState.waypoints.forEach((wp, i) => {
    const row     = document.createElement("div");
    row.className = "plan-row";
    const latStr  = `${Math.abs(wp.lat).toFixed(4)}°${wp.lat >= 0 ? "N" : "S"}`;
    const lonStr  = `${Math.abs(wp.lon).toFixed(4)}°${wp.lon >= 0 ? "E" : "W"}`;
    const label   = wp.id || `WPT${i + 1}`;
    row.innerHTML =
      `<span class="plan-idx">${i + 1}</span>` +
      `<span class="plan-id">${label}</span>` +
      `<span class="plan-coord">${latStr} ${lonStr}</span>` +
      `<button class="plan-del" data-idx="${i}" title="Remove">✕</button>`;
    planListEl.appendChild(row);
  });
  planListEl.querySelectorAll(".plan-del").forEach(btn => {
    btn.addEventListener("click", () => {
      planState.waypoints.splice(Number(btn.dataset.idx), 1);
      planRebuildList();
      updatePlanRoute();
    });
  });
}

function planAddWaypoint(lat, lon, altM, id) {
  planState.waypoints.push({ lat, lon, alt: altM || 0, id: id || "", type: "DIRECT", speed: 0 });
  planRebuildList();
  updatePlanRoute();
}

function planClear() {
  planState.waypoints = [];
  planRebuildList();
  updatePlanRoute();
}

function planExport() {
  if (!planState.waypoints.length) {
    if (planStatusEl) planStatusEl.textContent = "Nothing to export.";
    return;
  }
  const plan = {
    name:    "Unnamed Plan",
    created: new Date().toISOString(),
    legs:    planState.waypoints.map((wp, i) => ({
      seq:      i,
      type:     wp.type || "DIRECT",
      id:       wp.id || `WPT${i + 1}`,
      lat:      Number(wp.lat.toFixed(6)),
      lon:      Number(wp.lon.toFixed(6)),
      alt_m:    Number(wp.alt.toFixed(0)),
      speed_ms: wp.speed || 0,
    })),
  };
  const blob = new Blob([JSON.stringify(plan, null, 2)], { type: "application/json" });
  const a    = document.createElement("a");
  a.href     = URL.createObjectURL(blob);
  a.download = "kerbin_plan.json";
  a.click();
  URL.revokeObjectURL(a.href);
}

// ============================================================
// Show / Hide All layers
// ============================================================

function updateShowHideAllLabel() {
  if (!showHideAllBtn) return;
  const allOn = Object.values(layerCbs).every(cb => cb.checked);
  showHideAllBtn.textContent = allOn ? "Hide All" : "Show All";
}

// ============================================================
// Data loading
// ============================================================

async function loadImage(url) {
  return new Promise((resolve, reject) => {
    const img     = new Image();
    img.onload    = () => resolve(img);
    img.onerror   = () => reject(new Error(`Failed to load: ${url}`));
    img.src       = `${url}?t=${Date.now()}`;
  });
}

async function loadCsv(url) {
  const resp = await fetch(`${url}?t=${Date.now()}`);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}: ${url}`);
  return parseCsv(await resp.text());
}

function setStatus(msg) { if (statusEl) statusEl.textContent = msg; }

async function reloadAll() {
  reloadBtn.disabled = true;

  // Ask the server to re-export CSVs (silently no-ops against plain http.server)
  setStatus("Exporting nav data...");
  try {
    const resp   = await fetch("/api/export", { method: "POST" });
    const result = await resp.json();
    if (!result.ok) setStatus(`Export warning: ${result.message}`);
  } catch { /* not running server.py */ }

  setStatus("Loading...");
  try {
    // Basemap
    let mapLoaded = false;
    try {
      const img = await loadImage(mapPathInput.value.trim());
      setMapTexture(img);
      mapLoaded = true;
    } catch {
      setMapTexture(null);
    }

    // Beacons
    const beaconRows = await loadCsv(beaconsPathInput.value.trim());
    state.beacons = beaconRows.map(r => ({
      id:       r.id       || "",
      type:     r.type     || "",
      lat:      Number(r.lat),
      lon:      Number(r.lon),
      alt_asl:  Number(r.alt_asl) || 0,
      name:     r.name     || "",
      rwy:      r.rwy      || "",
      hdg:      Number(r.hdg)       || 0,
      gs_angle: Number(r.gs_angle)  || 0,
    })).filter(b => Number.isFinite(b.lat) && Number.isFinite(b.lon) && b.id);
    state.beaconById = new Map(state.beacons.map(b => [b.id, b]));

    // Plates
    const plateRows = await loadCsv(platesPathInput.value.trim());
    const grouped   = new Map();
    plateRows.forEach(r => {
      const pid    = r.plate_id  || "";
      const fix    = r.beacon_id || "";
      const seq    = Number(r.sequence);
      const ils_id = r.ils_id    || "";
      const vapp   = Number(r.vapp) || 0;
      if (!pid || !fix || !Number.isFinite(seq)) return;
      if (!grouped.has(pid)) grouped.set(pid, { fixes: [], ils_id: "", vapp: 0 });
      grouped.get(pid).fixes.push({ seq, fix });
      if (ils_id) grouped.get(pid).ils_id = ils_id;
      grouped.get(pid).vapp = vapp;
    });
    state.plates = new Map();
    grouped.forEach((data, pid) => {
      data.fixes.sort((a, b) => a.seq - b.seq);
      state.plates.set(pid, { fixes: data.fixes.map(x => x.fix), ils_id: data.ils_id, vapp: data.vapp });
    });

    rebuildPlateSelect();
    buildGrid();
    refreshScene();
    fitView();

    const ilsCount = state.beacons.filter(b => b.type === "ILS").length;
    const aptCount = state.beacons.filter(b => b.type === "WPT" && b.id.startsWith("WPT_APT_")).length;
    setStatus(
      `${mapLoaded ? "Map loaded" : "No map image"}  ·  `
      + `${state.beacons.length} beacons  ·  ${ilsCount} ILS  ·  `
      + `${aptCount} airports  ·  ${state.plates.size} plates`,
    );
  } catch (err) {
    console.error(err);
    setStatus(`Error: ${err.message}`);
  } finally {
    reloadBtn.disabled = false;
  }
}

// ============================================================
// Resize
// ============================================================

function onResize() {
  const w = mapContainer.clientWidth;
  const h = mapContainer.clientHeight;
  if (w <= 0 || h <= 0) return;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
  labelRenderer.setSize(w, h);
}

// ============================================================
// Event wiring
// ============================================================

reloadBtn.addEventListener("click", reloadAll);

showLabelsEl.addEventListener("change", () => updateBeacons());

showAltitudeEl.addEventListener("change", () => { updateGlideslopes(); refreshScene(); });
if (showAltStemsEl) showAltStemsEl.addEventListener("change", () => updateBeacons());
if (heightExagEl)   heightExagEl.addEventListener("input",    () => refreshScene());
if (showProfileEl)  showProfileEl.addEventListener("change",  () => drawPlateProfile());

markerSizeEl.addEventListener("input", () => updateBeacons());

plateSelectEl.addEventListener("change", () => {
  state.selectedPlate = plateSelectEl.value || "";
  updatePlateRoute();
  drawPlateProfile();
});

Object.values(layerCbs).forEach(cb => cb.addEventListener("change", () => {
  updateShowHideAllLabel();
  updateCenterlines();
  updateGlideslopes();
  updateBeacons();
}));

if (showHideAllBtn) {
  showHideAllBtn.addEventListener("click", () => {
    const allOn = Object.values(layerCbs).every(cb => cb.checked);
    Object.values(layerCbs).forEach(cb => { cb.checked = !allOn; });
    updateShowHideAllLabel();
    updateCenterlines();
    updateGlideslopes();
    updateBeacons();
  });
}

if (logFileInput)    logFileInput.addEventListener("change", e => { if (e.target.files[0]) loadLogFile(e.target.files[0]); });
if (logLayerToggle)  logLayerToggle.addEventListener("change", () => updateLogPath());

if (planModeBtn) {
  planModeBtn.addEventListener("click", () => {
    planState.active = !planState.active;
    planModeBtn.textContent = planState.active ? "Exit Plan Mode" : "Enter Plan Mode";
    planModeBtn.classList.toggle("active", planState.active);
    renderer.domElement.style.cursor = planState.active ? "crosshair" : "default";
    if (planStatusEl) planStatusEl.textContent = planState.active
      ? "Click map to add waypoint. Click a beacon to add it."
      : "";
  });
}
if (planClearBtn)  planClearBtn.addEventListener("click",  () => { planClear();  if (planStatusEl) planStatusEl.textContent = "Plan cleared."; });
if (planExportBtn) planExportBtn.addEventListener("click", planExport);

// Hover + cursor readout (on Three.js canvas)
renderer.domElement.addEventListener("mousemove", e => {
  const rect = renderer.domElement.getBoundingClientRect();
  mouseNDC.set(
    ((e.clientX - rect.left) / rect.width)  * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(mouseNDC, camera);

  // Cursor lat/lon from plane intersection
  const hit = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(mapPlane, hit)) {
    const lon = (hit.x / (WORLD_W / 2)) * 180;
    const lat = -(hit.z / (WORLD_H / 2)) * 90;
    if (cursorReadoutEl) {
      const ns = lat >= 0 ? "N" : "S";
      const ew = lon >= 0 ? "E" : "W";
      cursorReadoutEl.textContent =
        `${Math.abs(lat).toFixed(4)}° ${ns}   ${Math.abs(lon).toFixed(4)}° ${ew}`;
    }
  }

  // Beacon hover
  const hovered = hitTest(e.clientX, e.clientY);
  if (hovered !== state.hovered) {
    state.hovered = hovered;
    if (!planState.active) renderer.domElement.style.cursor = hovered ? "crosshair" : "default";
    updateHoverHalo();
  }
  showTooltip(hovered, e.clientX, e.clientY);
});

renderer.domElement.addEventListener("mouseleave", () => {
  state.hovered = null;
  tooltipEl.style.display = "none";
  updateHoverHalo();
});

// Plan mode click — raycast against the flat plane
renderer.domElement.addEventListener("click", e => {
  if (!planState.active) return;
  const rect = renderer.domElement.getBoundingClientRect();
  mouseNDC.set(
    ((e.clientX - rect.left) / rect.width)  * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(mouseNDC, camera);
  const hit = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(mapPlane, hit)) return;

  // Prefer clicking an existing beacon
  const hov = hitTest(e.clientX, e.clientY);
  if (hov) {
    planAddWaypoint(hov.lat, hov.lon, hov.alt_asl, hov.id);
  } else {
    const lon = (hit.x / (WORLD_W / 2)) * 180;
    const lat = -(hit.z / (WORLD_H / 2)) * 90;
    planAddWaypoint(lat, lon, 0, "");
  }
});

// Double-click on a beacon: pivot around that point. Double-click on empty: reset view.
renderer.domElement.addEventListener("dblclick", e => {
  const hov = hitTest(e.clientX, e.clientY);
  if (hov) {
    const mapPos = lonLatAltToVec3(hov.lon, hov.lat, 0);
    controls.target.set(mapPos.x, PLANE_Y, mapPos.z);
    controls.update();
  } else {
    fitView();
  }
});

window.addEventListener("resize", onResize);

// ============================================================
// Animation loop
// ============================================================

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
}

// ============================================================
// Boot
// ============================================================

updateShowHideAllLabel();
onResize();
animate();
reloadAll();
