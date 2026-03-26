"use strict";

// ============================================================
// Kerbin Navigation Chart Viewer
// ============================================================

const KERBIN_R_M = 600_000; // Kerbin mean radius (m)

/** Compute destination given start lat/lon, bearing (deg), and distance (m). */
function geoDestination(lat, lon, bearingDeg, distM) {
  const d = distM / KERBIN_R_M;
  const b = bearingDeg * Math.PI / 180;
  const lat1 = lat * Math.PI / 180;
  const lon1 = lon * Math.PI / 180;
  const lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(b));
  const lon2 = lon1 + Math.atan2(
    Math.sin(b) * Math.sin(d) * Math.cos(lat1),
    Math.cos(d) - Math.sin(lat1) * Math.sin(lat2)
  );
  return {
    lat: lat2 * 180 / Math.PI,
    lon: ((lon2 * 180 / Math.PI) + 540) % 360 - 180,
  };
}

// ---- DOM handles ----
const canvas           = document.getElementById("mapCanvas");
const ctx              = canvas.getContext("2d");
const mapPathInput     = document.getElementById("mapPath");
const beaconsPathInput = document.getElementById("beaconsPath");
const platesPathInput  = document.getElementById("platesPath");
const reloadBtn        = document.getElementById("reloadBtn");
const showLabelsEl     = document.getElementById("showLabels");
const showPseudo3DEl   = document.getElementById("showPseudo3D");
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
  mapImage:      null,
  mapWidth:      3600,
  mapHeight:     1800,
  beacons:       [],
  beaconById:    new Map(),
  plates:        new Map(),
  selectedPlate: "",
  view:          { scale: 1, tx: 0, ty: 0 },
  dragging:      false,
  dragMode:      "",
  dragStart:     { x: 0, y: 0, tx: 0, ty: 0, azimuth: 232, strength: 1.0 },
  tilt:          { azimuth: 232, strength: 1.0 },
  hovered:       null,
};

// ---- Utilities ----
function setStatus(msg) { statusEl.textContent = msg; }

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
    case "WPT":
      if (id && id.startsWith("WPT_APT_")) return "#a8ff78";
      return "#c9b8ff";
    default: return "#e0e0e0";
  }
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width  = Math.max(1, Math.floor(canvas.clientWidth  * dpr));
  canvas.height = Math.max(1, Math.floor(canvas.clientHeight * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

function lonLatToWorld(lon, lat) {
  return {
    x: ((lon + 180.0) / 360.0) * state.mapWidth,
    y: ((90.0 - lat)  / 180.0) * state.mapHeight,
  };
}

function worldToLonLat(x, y) {
  return {
    lon: (x / state.mapWidth)  * 360.0 - 180.0,
    lat: 90.0 - (y / state.mapHeight) * 180.0,
  };
}

function screenToWorld(sx, sy) {
  return {
    x: (sx - state.view.tx) / state.view.scale,
    y: (sy - state.view.ty) / state.view.scale,
  };
}

function worldToScreen(wx, wy) {
  return {
    x: wx * state.view.scale + state.view.tx,
    y: wy * state.view.scale + state.view.ty,
  };
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function geoDistanceM(lat1, lon1, lat2, lon2) {
  const p = Math.PI / 180;
  const aLat = lat1 * p;
  const bLat = lat2 * p;
  const dLat = (lat2 - lat1) * p;
  const dLon = (lon2 - lon1) * p;
  const s = Math.sin(dLat * 0.5) ** 2
    + Math.cos(aLat) * Math.cos(bLat) * Math.sin(dLon * 0.5) ** 2;
  return 2 * KERBIN_R_M * Math.atan2(Math.sqrt(s), Math.sqrt(1 - s));
}

function metersPerWorldUnit() {
  return (2 * Math.PI * KERBIN_R_M) / state.mapWidth;
}

function pseudo3dEnabled() {
  return !!(showPseudo3DEl && showPseudo3DEl.checked);
}

function getHeightExaggeration() {
  if (!heightExagEl) return 1;
  return clamp(Number(heightExagEl.value) || 1, 1, 20);
}

function getBeaconWorldBase(beacon) {
  return lonLatToWorld(beacon.lon, beacon.lat);
}

function getBeaconWorldTop(beacon) {
  const p = getBeaconWorldBase(beacon);
  if (!pseudo3dEnabled()) return p;

  const metersPerWorld = metersPerWorldUnit();
  const altM = Math.max(0, Number(beacon.alt_asl) || 0);
  const worldLift = (altM / metersPerWorld) * getHeightExaggeration() * clamp(state.tilt.strength, 0.15, 3.0);

  const az = ((state.tilt.azimuth % 360) + 360) % 360;
  const rad = az * Math.PI / 180;
  const ux = Math.cos(rad);
  const uy = Math.sin(rad);
  return { x: p.x + worldLift * ux, y: p.y + worldLift * uy };
}

function markerAltitudeAlpha(altM) {
  if (!pseudo3dEnabled()) return 1.0;
  return clamp(0.75 + (Math.max(0, altM) / 8000), 0.75, 1.0);
}

function fitView() {
  const vw = canvas.clientWidth;
  const vh = canvas.clientHeight;
  if (vw <= 0 || vh <= 0) return;
  const scale = Math.min(vw / state.mapWidth, vh / state.mapHeight);
  state.view.scale = Math.max(scale, 0.01);
  state.view.tx = (vw - state.mapWidth  * state.view.scale) * 0.5;
  state.view.ty = (vh - state.mapHeight * state.view.scale) * 0.5;
  draw();
}

// ---- Loaders ----
async function loadImage(url) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload  = () => resolve(img);
    img.onerror = () => reject(new Error(`Failed to load: ${url}`));
    img.src = `${url}?t=${Date.now()}`;
  });
}

function parseCsv(text) {
  const lines = text.split(/\r?\n/).map(l => l.trim()).filter(l => l && !l.startsWith("#"));
  if (!lines.length) return [];
  const headers = lines[0].split(",").map(s => s.trim());
  return lines.slice(1).map(line => {
    const cols = line.split(",");
    const row = {};
    for (let i = 0; i < headers.length; i++) row[headers[i]] = (cols[i] || "").trim();
    return row;
  });
}

async function loadCsv(url) {
  const resp = await fetch(`${url}?t=${Date.now()}`);
  if (!resp.ok) throw new Error(`HTTP ${resp.status}: ${url}`);
  return parseCsv(await resp.text());
}

function getPlateSequence(plate) {
  if (!plate) return [];
  const seq = [...plate.fixes];
  if (plate.ils_id && plate.ils_id !== seq[seq.length - 1]) seq.push(plate.ils_id);
  return seq;
}

// ---- Marker rendering ----
/**
 * Draw a type-specific beacon marker centered at (cx, cy) in world-space.
 * r = radius in world-space units (= markerSize / scale, giving constant screen size).
 * Call this inside a ctx that already has the world transform applied.
 */
function drawMarkerShape(cx, cy, type, id, r) {
  ctx.save();
  ctx.translate(cx, cy);
  const lw = 1.5 / state.view.scale;

  switch (type) {

    case "ILS": {
      // Filled amber diamond
      ctx.beginPath();
      ctx.moveTo(0, -r * 1.25);
      ctx.lineTo(r * 0.75, 0);
      ctx.lineTo(0,  r * 1.25);
      ctx.lineTo(-r * 0.75, 0);
      ctx.closePath();
      ctx.fillStyle = "#ffd166";
      ctx.fill();
      ctx.strokeStyle = "rgba(0,0,0,0.5)";
      ctx.lineWidth = 0.5 / state.view.scale;
      ctx.stroke();
      break;
    }

    case "IAF": {
      // Open green upward triangle
      ctx.beginPath();
      ctx.moveTo(0, -r);
      ctx.lineTo(r * 0.866,  r * 0.5);
      ctx.lineTo(-r * 0.866, r * 0.5);
      ctx.closePath();
      ctx.strokeStyle = "#90be6d";
      ctx.lineWidth = lw;
      ctx.stroke();
      break;
    }

    case "FAF": {
      // Filled orange circle with black cross
      ctx.beginPath();
      ctx.arc(0, 0, r, 0, Math.PI * 2);
      ctx.fillStyle = "#ff9f1c";
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(-r, 0); ctx.lineTo(r, 0);
      ctx.moveTo(0, -r); ctx.lineTo(0, r);
      ctx.strokeStyle = "rgba(0,0,0,0.55)";
      ctx.lineWidth = lw * 0.7;
      ctx.stroke();
      break;
    }

    case "VOR": {
      // Cyan hexagon (open, with faint fill)
      ctx.beginPath();
      for (let i = 0; i < 6; i++) {
        const a  = (i / 6) * Math.PI * 2 - Math.PI / 2;
        const px = Math.cos(a) * r;
        const py = Math.sin(a) * r;
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      }
      ctx.closePath();
      ctx.fillStyle = "rgba(76,201,240,0.12)";
      ctx.fill();
      ctx.strokeStyle = "#4cc9f0";
      ctx.lineWidth = lw;
      ctx.stroke();
      break;
    }

    case "WPT": {
      if (id && id.startsWith("WPT_APT_")) {
        // Solid chartreuse upward triangle (airports)
        const rs = r * 1.2;
        ctx.beginPath();
        ctx.moveTo(0, -rs);
        ctx.lineTo(rs * 0.866,  rs * 0.5);
        ctx.lineTo(-rs * 0.866, rs * 0.5);
        ctx.closePath();
        ctx.fillStyle = "#a8ff78";
        ctx.fill();
        ctx.strokeStyle = "rgba(0,0,0,0.35)";
        ctx.lineWidth = 0.5 / state.view.scale;
        ctx.stroke();
      } else {
        // Lavender filled circle (custom waypoints)
        ctx.beginPath();
        ctx.arc(0, 0, r * 0.85, 0, Math.PI * 2);
        ctx.fillStyle = "#c9b8ff";
        ctx.fill();
      }
      break;
    }

    default: {
      ctx.beginPath();
      ctx.arc(0, 0, r, 0, Math.PI * 2);
      ctx.fillStyle = "#e0e0e0";
      ctx.fill();
    }
  }
  ctx.restore();
}

// ---- Hit testing ----
function hitTest(sx, sy) {
  let closest = null;
  let minDist = 16; // screen pixels
  for (const b of state.beacons) {
    if (!layerVisible(b.type, b.id)) continue;
    const wp = getBeaconWorldTop(b);
    const sc = worldToScreen(wp.x, wp.y);
    const d  = Math.hypot(sx - sc.x, sy - sc.y);
    if (d < minDist) { minDist = d; closest = b; }
  }
  return closest;
}

// ---- Drawing functions ----
function drawGrid() {
  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);
  ctx.strokeStyle = "rgba(255,255,255,0.07)";
  ctx.lineWidth = 1 / state.view.scale;

  for (let lon = -180; lon <= 180; lon += 30) {
    const x = ((lon + 180) / 360) * state.mapWidth;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, state.mapHeight); ctx.stroke();
  }
  for (let lat = -90; lat <= 90; lat += 30) {
    const y = ((90 - lat) / 180) * state.mapHeight;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(state.mapWidth, y); ctx.stroke();
  }

  // Lat/lon labels when zoomed in enough
  if (state.view.scale > 0.3) {
    const fs = Math.round(10 / state.view.scale);
    ctx.font = `${fs}px Consolas`;
    ctx.fillStyle = "rgba(255,255,255,0.22)";
    ctx.textAlign = "left";
    for (let lat = -60; lat <= 60; lat += 30) {
      const y = ((90 - lat) / 180) * state.mapHeight;
      ctx.fillText(`${lat >= 0 ? "+" : ""}${lat}°`, 4 / state.view.scale, y - 3 / state.view.scale);
    }
    for (let lon = -150; lon <= 150; lon += 30) {
      const x = ((lon + 180) / 360) * state.mapWidth;
      ctx.fillText(`${lon}°`, x + 3 / state.view.scale, 13 / state.view.scale);
    }
  }
  ctx.restore();
}

function drawCenterlines() {
  if (!layerCbs.centerlines.checked) return;

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  for (const b of state.beacons) {
    if (b.type !== "ILS" || !layerCbs.ILS.checked) continue;
    if (!b.hdg) continue;

    const appBrng = (b.hdg + 180) % 360;
    const far     = geoDestination(b.lat, b.lon, appBrng, 66000);
    const p1 = lonLatToWorld(b.lon, b.lat);
    const p2 = lonLatToWorld(far.lon, far.lat);

    ctx.beginPath();
    ctx.setLineDash([6 / state.view.scale, 5 / state.view.scale]);
    ctx.strokeStyle = "rgba(255,200,70,0.22)";
    ctx.lineWidth = 1.2 / state.view.scale;
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.restore();
}

function drawBeacons() {
  const markerSize = Number(markerSizeEl.value) || 6;
  const showLabels = showLabelsEl.checked;
  const drawStems = !!(showAltStemsEl && showAltStemsEl.checked && pseudo3dEnabled());

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  for (const b of state.beacons) {
    if (!layerVisible(b.type, b.id)) continue;

    const pBase = getBeaconWorldBase(b);
    const pTop  = getBeaconWorldTop(b);
    const r  = markerSize / state.view.scale;
    const isHovered = state.hovered === b;
    const alpha = markerAltitudeAlpha(b.alt_asl);

    if (drawStems && (pTop.x !== pBase.x || pTop.y !== pBase.y)) {
      ctx.beginPath();
      ctx.moveTo(pBase.x, pBase.y);
      ctx.lineTo(pTop.x, pTop.y);
      ctx.strokeStyle = "rgba(210,230,245,0.35)";
      ctx.lineWidth = 1.1 / state.view.scale;
      ctx.stroke();
    }

    if (isHovered) {
      ctx.beginPath();
      ctx.arc(pTop.x, pTop.y, r * 2.4, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255,255,255,0.1)";
      ctx.fill();
    }

    ctx.save();
    ctx.globalAlpha = alpha;
    drawMarkerShape(pTop.x, pTop.y, b.type, b.id, r);
    ctx.restore();

    if (showLabels && state.view.scale >= 0.4) {
      const label = b.id.replace(/^WPT_APT_/, "").replace(/^WPT_/, "");
      const color = isHovered ? "#ffffff" : beaconColor(b.type, b.id);
      const fs    = 9 / state.view.scale;
      ctx.font     = `${fs}px Consolas`;
      ctx.fillStyle = color;
      ctx.shadowBlur  = 3 / state.view.scale;
      ctx.shadowColor = "#000000";
      ctx.fillText(label, pTop.x + r + 2 / state.view.scale, pTop.y + 3 / state.view.scale);
      ctx.shadowBlur = 0;
    }
  }
  ctx.restore();
}

function drawPlateRoute() {
  if (!state.selectedPlate) return;
  const plate = state.plates.get(state.selectedPlate);
  if (!plate) return;

  // Full fix sequence: IAF(s) → FAF → ILS threshold
  const allFixes = getPlateSequence(plate);
  if (!allFixes.length) return;

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  // Connecting polyline
  ctx.beginPath();
  ctx.setLineDash([5 / state.view.scale, 3 / state.view.scale]);
  ctx.strokeStyle = "rgba(255, 70, 70, 0.9)";
  ctx.lineWidth = 2 / state.view.scale;
  let first = true;
  for (const fixId of allFixes) {
    const b = state.beaconById.get(fixId);
    if (!b) continue;
    const p = getBeaconWorldTop(b);
    if (first) { ctx.moveTo(p.x, p.y); first = false; } else ctx.lineTo(p.x, p.y);
  }
  if (!first) ctx.stroke();
  ctx.setLineDash([]);

  // Fix dots + labels
  allFixes.forEach((fixId, i) => {
    const b = state.beaconById.get(fixId);
    if (!b) return;
    const p = getBeaconWorldTop(b);
    const r = 4 / state.view.scale;
    const isILS = i === allFixes.length - 1 && b.type === "ILS";

    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle = isILS ? "#ffd166" : "rgba(255, 90, 90, 0.95)";
    ctx.fill();

    if (showLabelsEl.checked) {
      ctx.font      = `${10 / state.view.scale}px Consolas`;
      ctx.fillStyle = isILS ? "#ffd166" : "rgba(255,220,220,0.95)";
      ctx.shadowBlur  = 3 / state.view.scale;
      ctx.shadowColor = "#000";
      ctx.fillText(
        `${i + 1}: ${fixId}`,
        p.x + 5 / state.view.scale,
        p.y - 5 / state.view.scale
      );
      ctx.shadowBlur = 0;
    }
  });

  ctx.restore();
}

function drawPlateProfile() {
  if (!profileCtx || !profileCanvas || !profileStatusEl) return;

  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(200, Math.floor(profileCanvas.clientWidth || 420));
  const h = Math.max(110, Math.floor(profileCanvas.clientHeight || 170));
  if (profileCanvas.width !== Math.floor(w * dpr) || profileCanvas.height !== Math.floor(h * dpr)) {
    profileCanvas.width = Math.floor(w * dpr);
    profileCanvas.height = Math.floor(h * dpr);
  }
  profileCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  profileCtx.clearRect(0, 0, w, h);

  const panelOff = showProfileEl && !showProfileEl.checked;
  if (panelOff) {
    profileStatusEl.textContent = "Profile hidden.";
    return;
  }
  if (!state.selectedPlate) {
    profileStatusEl.textContent = "Select a plate to view its altitude profile.";
    return;
  }

  const plate = state.plates.get(state.selectedPlate);
  if (!plate) {
    profileStatusEl.textContent = "Selected plate data not found.";
    return;
  }

  const sequence = getPlateSequence(plate);
  if (!sequence.length) {
    profileStatusEl.textContent = "Selected plate has no fixes.";
    return;
  }

  const points = [];
  let distM = 0;
  let prev = null;
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
  const yMin = Math.max(0, minAltRaw - 200);
  const yMax = Math.max(yMin + 500, maxAltRaw + 250);
  const xMax = Math.max(1, points[points.length - 1].dist);

  const m = { l: 42, r: 10, t: 10, b: 22 };
  const pw = Math.max(20, w - m.l - m.r);
  const ph = Math.max(20, h - m.t - m.b);

  const xFor = dKm => m.l + (dKm / xMax) * pw;
  const yFor = altM => m.t + (1 - ((altM - yMin) / (yMax - yMin))) * ph;

  profileCtx.fillStyle = "#0f171f";
  profileCtx.fillRect(0, 0, w, h);

  // Grid + axes
  profileCtx.strokeStyle = "rgba(170,200,220,0.22)";
  profileCtx.lineWidth = 1;
  profileCtx.beginPath();
  profileCtx.moveTo(m.l, m.t);
  profileCtx.lineTo(m.l, m.t + ph);
  profileCtx.lineTo(m.l + pw, m.t + ph);
  profileCtx.stroke();

  profileCtx.fillStyle = "rgba(175,205,225,0.8)";
  profileCtx.font = "11px Consolas";
  for (let i = 0; i <= 4; i++) {
    const t = i / 4;
    const altTick = yMin + (1 - t) * (yMax - yMin);
    const y = m.t + t * ph;
    profileCtx.strokeStyle = "rgba(170,200,220,0.12)";
    profileCtx.beginPath();
    profileCtx.moveTo(m.l, y);
    profileCtx.lineTo(m.l + pw, y);
    profileCtx.stroke();
    profileCtx.fillText(`${Math.round(altTick)}m`, 3, y + 3);
  }
  profileCtx.fillText(`${xMax.toFixed(1)}km`, m.l + pw - 44, h - 5);
  profileCtx.fillText("0km", m.l - 4, h - 5);

  // Profile path
  profileCtx.strokeStyle = "rgba(255,120,120,0.95)";
  profileCtx.lineWidth = 2;
  profileCtx.beginPath();
  points.forEach((p, i) => {
    const x = xFor(p.dist);
    const y = yFor(p.alt);
    if (i === 0) profileCtx.moveTo(x, y); else profileCtx.lineTo(x, y);
  });
  profileCtx.stroke();

  // Markers + labels
  points.forEach((p, i) => {
    const x = xFor(p.dist);
    const y = yFor(p.alt);
    profileCtx.beginPath();
    profileCtx.arc(x, y, 3, 0, Math.PI * 2);
    profileCtx.fillStyle = i === points.length - 1 ? "#ffd166" : "#ff7a7a";
    profileCtx.fill();
    if (pw > 280) {
      profileCtx.fillStyle = "rgba(230,240,250,0.9)";
      profileCtx.font = "10px Consolas";
      profileCtx.fillText(p.id, x + 4, y - 4);
    }
  });

  profileStatusEl.textContent =
    `${state.selectedPlate}  |  Dist ${xMax.toFixed(1)} km  |  Alt ${Math.round(minAltRaw)}-${Math.round(maxAltRaw)} m MSL`;
}

function draw() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  if (state.mapImage) {
    ctx.save();
    ctx.translate(state.view.tx, state.view.ty);
    ctx.scale(state.view.scale, state.view.scale);
    ctx.drawImage(state.mapImage, 0, 0, state.mapWidth, state.mapHeight);
    ctx.restore();
  } else {
    ctx.fillStyle = "#0f1722";
    ctx.fillRect(0, 0, w, h);
  }

  drawGrid();
  drawCenterlines();
  drawPlateRoute();
  drawBeacons();
  drawPlateProfile();
}

// ---- Plate select ----
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

// ---- Tooltip ----
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
    html += `<div class="tt-coords">HDG ${b.hdg.toFixed(1)}°  ·  GS ${b.gs_angle.toFixed(1)}°</div>`;
  }

  tooltipEl.innerHTML = html;
  tooltipEl.style.display = "block";

  const tw = tooltipEl.offsetWidth + 20;
  const th = tooltipEl.offsetHeight + 20;
  tooltipEl.style.left = (clientX + 14 + tw > window.innerWidth  ? clientX - tw : clientX + 14) + "px";
  tooltipEl.style.top  = (clientY + 14 + th > window.innerHeight ? clientY - th : clientY + 14) + "px";
}

// ---- Data load ----
async function reloadAll() {
  reloadBtn.disabled = true;

  // Ask the server to re-export the CSVs from generate_static_nav_csv.py.
  // Falls back silently if served by plain `python -m http.server`.
  setStatus("Exporting nav data...");
  try {
    const resp   = await fetch("/api/export", { method: "POST" });
    const result = await resp.json();
    if (!result.ok) setStatus(`Export warning: ${result.message}`);
  } catch {
    // Not running server.py — skip export, reload whatever is on disk.
  }

  setStatus("Loading...");
  try {
    // Basemap (optional)
    let mapLoaded = false;
    try {
      state.mapImage  = await loadImage(mapPathInput.value.trim());
      state.mapWidth  = state.mapImage.width;
      state.mapHeight = state.mapImage.height;
      mapLoaded = true;
    } catch {
      state.mapImage = null;
      state.mapWidth  = 3600;
      state.mapHeight = 1800;
    }

    // Beacons
    const beaconRows = await loadCsv(beaconsPathInput.value.trim());
    state.beacons = beaconRows.map(r => ({
      id:       r.id      || "",
      type:     r.type    || "",
      lat:      Number(r.lat),
      lon:      Number(r.lon),
      alt_asl:  Number(r.alt_asl) || 0,
      name:     r.name    || "",
      rwy:      r.rwy     || "",
      hdg:      Number(r.hdg)      || 0,
      gs_angle: Number(r.gs_angle) || 0,
    })).filter(b => Number.isFinite(b.lat) && Number.isFinite(b.lon) && b.id);
    state.beaconById = new Map(state.beacons.map(b => [b.id, b]));

    // Plates
    const plateRows = await loadCsv(platesPathInput.value.trim());
    const grouped = new Map();
    plateRows.forEach(r => {
      const pid    = r.plate_id  || "";
      const fix    = r.beacon_id || "";
      const seq    = Number(r.sequence);
      const ils_id = r.ils_id   || "";
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
      state.plates.set(pid, {
        fixes:  data.fixes.map(x => x.fix),
        ils_id: data.ils_id,
        vapp:   data.vapp,
      });
    });

    rebuildPlateSelect();
    fitView();

    const ilsCount = state.beacons.filter(b => b.type === "ILS").length;
    const aptCount = state.beacons.filter(b => b.type === "WPT" && b.id.startsWith("WPT_APT_")).length;
    setStatus(
      `${mapLoaded ? "Map loaded" : "No map image"}  ·  `
      + `${state.beacons.length} beacons  ·  `
      + `${ilsCount} ILS  ·  ${aptCount} airports  ·  `
      + `${state.plates.size} plates`
    );
  } catch (err) {
    console.error(err);
    setStatus(`Error: ${err.message}`);
    draw();
  } finally {
    reloadBtn.disabled = false;
  }
}

// ---- Event wiring ----
reloadBtn.addEventListener("click", reloadAll);
showLabelsEl.addEventListener("change", draw);
if (showPseudo3DEl) showPseudo3DEl.addEventListener("change", draw);
if (showAltStemsEl) showAltStemsEl.addEventListener("change", draw);
if (heightExagEl) heightExagEl.addEventListener("input", draw);
if (showProfileEl) showProfileEl.addEventListener("change", draw);
markerSizeEl.addEventListener("input", draw);
plateSelectEl.addEventListener("change", () => {
  state.selectedPlate = plateSelectEl.value || "";
  draw();
});
Object.values(layerCbs).forEach(cb => cb.addEventListener("change", draw));

// Pan
canvas.addEventListener("mousedown", e => {
  if (e.button !== 0) return;
  e.preventDefault();
  state.dragging = true;

  if (e.shiftKey) {
    state.dragMode = "tilt";
  } else {
    state.dragMode = "pan";
  }

  state.dragStart = {
    x: e.clientX,
    y: e.clientY,
    tx: state.view.tx,
    ty: state.view.ty,
    azimuth: state.tilt.azimuth,
    strength: state.tilt.strength,
  };
});
window.addEventListener("mouseup", () => {
  state.dragging = false;
  state.dragMode = "";
});

window.addEventListener("mousemove", e => {
  if (state.dragging) {
    if (state.dragMode === "tilt") {
      const dx = e.clientX - state.dragStart.x;
      const dy = e.clientY - state.dragStart.y;
      state.tilt.azimuth = state.dragStart.azimuth + dx * 0.35;
      state.tilt.strength = clamp(state.dragStart.strength - dy * 0.01, 0.15, 3.0);
    } else {
      state.view.tx = state.dragStart.tx + (e.clientX - state.dragStart.x);
      state.view.ty = state.dragStart.ty + (e.clientY - state.dragStart.y);
    }
    draw();
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const sx   = e.clientX - rect.left;
  const sy   = e.clientY - rect.top;
  const wpos = screenToWorld(sx, sy);
  const ll   = worldToLonLat(wpos.x, wpos.y);
  const ns   = ll.lat >= 0 ? "N" : "S";
  const ew   = ll.lon >= 0 ? "E" : "W";
  cursorReadoutEl.textContent = `${Math.abs(ll.lat).toFixed(4)}° ${ns}   ${Math.abs(ll.lon).toFixed(4)}° ${ew}`;

  const hovered = hitTest(sx, sy);
  if (hovered !== state.hovered) {
    state.hovered = hovered;
    canvas.style.cursor = hovered ? "crosshair" : "default";
    draw();
  }
  showTooltip(hovered, e.clientX, e.clientY);
});

canvas.addEventListener("mouseleave", () => {
  state.hovered = null;
  tooltipEl.style.display = "none";
  draw();
});

// Zoom toward cursor
canvas.addEventListener("wheel", e => {
  e.preventDefault();
  const rect   = canvas.getBoundingClientRect();
  const sx     = e.clientX - rect.left;
  const sy     = e.clientY - rect.top;
  const before = screenToWorld(sx, sy);
  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  state.view.scale = Math.min(200, Math.max(0.01, state.view.scale * factor));
  state.view.tx = sx - before.x * state.view.scale;
  state.view.ty = sy - before.y * state.view.scale;
  draw();
}, { passive: false });

canvas.addEventListener("dblclick", () => fitView());
window.addEventListener("resize", resizeCanvas);

resizeCanvas();
reloadAll();
