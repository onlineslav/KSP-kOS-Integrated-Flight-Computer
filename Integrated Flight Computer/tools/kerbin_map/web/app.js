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
const markerSizeEl     = document.getElementById("markerSize");
const plateSelectEl    = document.getElementById("plateSelect");
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
  dragStart:     { x: 0, y: 0, tx: 0, ty: 0 },
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
    const wp = lonLatToWorld(b.lon, b.lat);
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

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  for (const b of state.beacons) {
    if (!layerVisible(b.type, b.id)) continue;

    const p  = lonLatToWorld(b.lon, b.lat);
    const r  = markerSize / state.view.scale;
    const isHovered = state.hovered === b;

    if (isHovered) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, r * 2.4, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255,255,255,0.1)";
      ctx.fill();
    }

    drawMarkerShape(p.x, p.y, b.type, b.id, r);

    if (showLabels && state.view.scale >= 0.4) {
      const label = b.id.replace(/^WPT_APT_/, "").replace(/^WPT_/, "");
      const color = isHovered ? "#ffffff" : beaconColor(b.type, b.id);
      const fs    = 9 / state.view.scale;
      ctx.font     = `${fs}px Consolas`;
      ctx.fillStyle = color;
      ctx.shadowBlur  = 3 / state.view.scale;
      ctx.shadowColor = "#000000";
      ctx.fillText(label, p.x + r + 2 / state.view.scale, p.y + 3 / state.view.scale);
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
  const allFixes = [...plate.fixes];
  if (plate.ils_id && plate.ils_id !== allFixes[allFixes.length - 1]) {
    allFixes.push(plate.ils_id);
  }

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
    const p = lonLatToWorld(b.lon, b.lat);
    if (first) { ctx.moveTo(p.x, p.y); first = false; } else ctx.lineTo(p.x, p.y);
  }
  if (!first) ctx.stroke();
  ctx.setLineDash([]);

  // Fix dots + labels
  allFixes.forEach((fixId, i) => {
    const b = state.beaconById.get(fixId);
    if (!b) return;
    const p = lonLatToWorld(b.lon, b.lat);
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
  }
}

// ---- Event wiring ----
reloadBtn.addEventListener("click", reloadAll);
showLabelsEl.addEventListener("change", draw);
markerSizeEl.addEventListener("input", draw);
plateSelectEl.addEventListener("change", () => {
  state.selectedPlate = plateSelectEl.value || "";
  draw();
});
Object.values(layerCbs).forEach(cb => cb.addEventListener("change", draw));

// Pan
canvas.addEventListener("mousedown", e => {
  state.dragging  = true;
  state.dragStart = { x: e.clientX, y: e.clientY, tx: state.view.tx, ty: state.view.ty };
});
window.addEventListener("mouseup", () => { state.dragging = false; });

window.addEventListener("mousemove", e => {
  if (state.dragging) {
    state.view.tx = state.dragStart.tx + (e.clientX - state.dragStart.x);
    state.view.ty = state.dragStart.ty + (e.clientY - state.dragStart.y);
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
