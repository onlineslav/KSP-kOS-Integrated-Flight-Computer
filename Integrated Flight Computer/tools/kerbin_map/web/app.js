"use strict";

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

const mapPathInput = document.getElementById("mapPath");
const beaconsPathInput = document.getElementById("beaconsPath");
const platesPathInput = document.getElementById("platesPath");
const reloadBtn = document.getElementById("reloadBtn");

const showBeaconsEl = document.getElementById("showBeacons");
const showLabelsEl = document.getElementById("showLabels");
const markerSizeEl = document.getElementById("markerSize");
const plateSelectEl = document.getElementById("plateSelect");
const cursorReadoutEl = document.getElementById("cursorReadout");
const statusEl = document.getElementById("status");

const state = {
  mapImage: null,
  mapWidth: 3600,
  mapHeight: 1800,
  beacons: [],
  beaconById: new Map(),
  plates: new Map(),
  selectedPlate: "",
  view: { scale: 1, tx: 0, ty: 0 },
  dragging: false,
  dragStart: { x: 0, y: 0, tx: 0, ty: 0 },
};

function setStatus(msg) {
  statusEl.textContent = msg;
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  canvas.width = Math.max(1, Math.floor(w * dpr));
  canvas.height = Math.max(1, Math.floor(h * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

function lonLatToWorld(lon, lat) {
  const x = ((lon + 180.0) / 360.0) * state.mapWidth;
  const y = ((90.0 - lat) / 180.0) * state.mapHeight;
  return { x, y };
}

function worldToLonLat(x, y) {
  const lon = (x / state.mapWidth) * 360.0 - 180.0;
  const lat = 90.0 - (y / state.mapHeight) * 180.0;
  return { lon, lat };
}

function screenToWorld(sx, sy) {
  return {
    x: (sx - state.view.tx) / state.view.scale,
    y: (sy - state.view.ty) / state.view.scale,
  };
}

function fitView() {
  const vw = canvas.clientWidth;
  const vh = canvas.clientHeight;
  if (vw <= 0 || vh <= 0) return;
  const scale = Math.min(vw / state.mapWidth, vh / state.mapHeight);
  state.view.scale = Math.max(scale, 0.01);
  state.view.tx = (vw - state.mapWidth * state.view.scale) * 0.5;
  state.view.ty = (vh - state.mapHeight * state.view.scale) * 0.5;
  draw();
}

async function loadImage(url) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => reject(new Error(`Failed to load image: ${url}`));
    img.src = `${url}?t=${Date.now()}`;
  });
}

function parseCsv(text) {
  const lines = text
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter((l) => l && !l.startsWith("#"));
  if (!lines.length) return [];
  const headers = lines[0].split(",").map((s) => s.trim());
  return lines.slice(1).map((line) => {
    const cols = line.split(",");
    const row = {};
    for (let i = 0; i < headers.length; i += 1) {
      row[headers[i]] = (cols[i] || "").trim();
    }
    return row;
  });
}

async function loadCsv(url) {
  const resp = await fetch(`${url}?t=${Date.now()}`);
  if (!resp.ok) throw new Error(`Failed to fetch ${url} (${resp.status})`);
  return parseCsv(await resp.text());
}

function beaconColor(type) {
  const t = (type || "").toUpperCase();
  if (t.includes("ILS")) return "#ffd166";
  if (t.includes("FAF")) return "#ff9f1c";
  if (t.includes("IAF")) return "#90be6d";
  if (t.includes("VOR")) return "#4cc9f0";
  if (t.includes("WPT")) return "#bdb2ff";
  return "#e0e0e0";
}

function drawGrid() {
  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  ctx.strokeStyle = "rgba(255,255,255,0.08)";
  ctx.lineWidth = 1 / state.view.scale;

  for (let lon = -180; lon <= 180; lon += 30) {
    const x = ((lon + 180) / 360) * state.mapWidth;
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, state.mapHeight);
    ctx.stroke();
  }
  for (let lat = -90; lat <= 90; lat += 30) {
    const y = ((90 - lat) / 180) * state.mapHeight;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(state.mapWidth, y);
    ctx.stroke();
  }

  ctx.restore();
}

function drawPlateRoute() {
  if (!state.selectedPlate) return;
  const route = state.plates.get(state.selectedPlate);
  if (!route || !route.length) return;

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  ctx.strokeStyle = "rgba(255, 70, 70, 0.95)";
  ctx.lineWidth = 2 / state.view.scale;
  ctx.beginPath();

  let drewAny = false;
  route.forEach((fixId, i) => {
    const b = state.beaconById.get(fixId);
    if (!b) return;
    const p = lonLatToWorld(b.lon, b.lat);
    if (!drewAny) {
      ctx.moveTo(p.x, p.y);
      drewAny = true;
    } else {
      ctx.lineTo(p.x, p.y);
    }

    ctx.fillStyle = "rgba(255,80,80,0.95)";
    const r = 4 / state.view.scale;
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fill();

    if (showLabelsEl.checked) {
      ctx.fillStyle = "rgba(255,220,220,0.95)";
      ctx.font = `${12 / state.view.scale}px Consolas`;
      ctx.fillText(`${i + 1}:${fixId}`, p.x + 6 / state.view.scale, p.y - 6 / state.view.scale);
    }
  });

  if (drewAny) ctx.stroke();
  ctx.restore();
}

function drawBeacons() {
  if (!showBeaconsEl.checked) return;
  const markerSize = Number(markerSizeEl.value) || 6;

  ctx.save();
  ctx.translate(state.view.tx, state.view.ty);
  ctx.scale(state.view.scale, state.view.scale);

  state.beacons.forEach((b) => {
    const p = lonLatToWorld(b.lon, b.lat);
    const r = markerSize / state.view.scale;

    ctx.fillStyle = beaconColor(b.type);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fill();

    if (showLabelsEl.checked) {
      ctx.fillStyle = "rgba(255,255,255,0.95)";
      ctx.font = `${12 / state.view.scale}px Consolas`;
      ctx.fillText(b.id, p.x + 8 / state.view.scale, p.y - 8 / state.view.scale);
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
  drawPlateRoute();
  drawBeacons();
}

function rebuildPlateSelect() {
  plateSelectEl.innerHTML = "";
  const none = document.createElement("option");
  none.value = "";
  none.textContent = "(none)";
  plateSelectEl.appendChild(none);

  const ids = Array.from(state.plates.keys()).sort();
  ids.forEach((id) => {
    const opt = document.createElement("option");
    opt.value = id;
    opt.textContent = id;
    plateSelectEl.appendChild(opt);
  });
  plateSelectEl.value = state.selectedPlate;
}

async function reloadAll() {
  setStatus("Loading...");
  try {
    let mapLoaded = false;
    try {
      state.mapImage = await loadImage(mapPathInput.value.trim());
      state.mapWidth = state.mapImage.width;
      state.mapHeight = state.mapImage.height;
      mapLoaded = true;
    } catch (err) {
      state.mapImage = null;
      state.mapWidth = 3600;
      state.mapHeight = 1800;
      console.warn(err);
    }

    const beaconsRows = await loadCsv(beaconsPathInput.value.trim());
    state.beacons = beaconsRows
      .map((r) => ({
        id: r.id || "",
        type: r.type || "",
        lat: Number(r.lat),
        lon: Number(r.lon),
        alt_asl: Number(r.alt_asl),
        name: r.name || "",
        runway: r.runway || "",
      }))
      .filter((b) => Number.isFinite(b.lat) && Number.isFinite(b.lon) && b.id);

    state.beaconById = new Map(state.beacons.map((b) => [b.id, b]));

    const plateRows = await loadCsv(platesPathInput.value.trim());
    const grouped = new Map();
    plateRows.forEach((r) => {
      const pid = r.plate_id || "";
      const fix = r.beacon_id || "";
      const seq = Number(r.sequence);
      if (!pid || !fix || !Number.isFinite(seq)) return;
      if (!grouped.has(pid)) grouped.set(pid, []);
      grouped.get(pid).push({ seq, fix });
    });
    state.plates = new Map();
    grouped.forEach((items, pid) => {
      items.sort((a, b) => a.seq - b.seq);
      state.plates.set(pid, items.map((x) => x.fix));
    });

    rebuildPlateSelect();
    fitView();
    draw();

    setStatus(
      `${mapLoaded ? "Map OK" : "Map missing"}  |  `
      + `${state.beacons.length} beacons  |  `
      + `${state.plates.size} plates`
    );
  } catch (err) {
    console.error(err);
    setStatus(`Load error: ${err.message}`);
    draw();
  }
}

reloadBtn.addEventListener("click", reloadAll);
showBeaconsEl.addEventListener("change", draw);
showLabelsEl.addEventListener("change", draw);
markerSizeEl.addEventListener("input", draw);
plateSelectEl.addEventListener("change", () => {
  state.selectedPlate = plateSelectEl.value || "";
  draw();
});

canvas.addEventListener("mousedown", (e) => {
  state.dragging = true;
  state.dragStart = {
    x: e.clientX,
    y: e.clientY,
    tx: state.view.tx,
    ty: state.view.ty,
  };
});
window.addEventListener("mouseup", () => {
  state.dragging = false;
});
window.addEventListener("mousemove", (e) => {
  if (state.dragging) {
    state.view.tx = state.dragStart.tx + (e.clientX - state.dragStart.x);
    state.view.ty = state.dragStart.ty + (e.clientY - state.dragStart.y);
    draw();
  }

  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;
  const wpos = screenToWorld(sx, sy);
  const ll = worldToLonLat(wpos.x, wpos.y);
  cursorReadoutEl.textContent = `lat ${ll.lat.toFixed(5)}, lon ${ll.lon.toFixed(5)}`;
});

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;

  const before = screenToWorld(sx, sy);
  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  const newScale = Math.min(200, Math.max(0.02, state.view.scale * factor));
  state.view.scale = newScale;
  state.view.tx = sx - before.x * state.view.scale;
  state.view.ty = sy - before.y * state.view.scale;
  draw();
}, { passive: false });

canvas.addEventListener("dblclick", () => fitView());
window.addEventListener("resize", resizeCanvas);

resizeCanvas();
reloadAll();
