// The robot's site and missions, in a browser. Talks only to web_api.py.
// Positions on the wire are GeoJSON [lon, lat]; Leaflet wants [lat, lon].
"use strict";

const KIND_LABEL = { area: "Alan", keepout: "Girilmez", obstacle: "Engel",
                     station: "İstasyon", route: "Rota" };
const KIND_STYLE = {
  area: { color: "#2fb35a", weight: 2, fillOpacity: 0.15 },
  keepout: { color: "#d43a3a", weight: 2, fillOpacity: 0.3 },
  obstacle: { color: "#e07b00", weight: 2, fillOpacity: 0.4 },
  route: { color: "#7a3fd1", weight: 3 },
};

const $ = (id) => document.getElementById(id);
const features = [];           // {kind, name, props, layer}
let selected = null;
let armedKind = null;
let taskTypes = {};
let gotoTarget = null;          // [lon, lat]
let gotoMarker = null;
let centredOnRobot = false;     // an empty site: first fix centres the map

// --- map ---------------------------------------------------------------------
const map = L.map("map", { maxZoom: 22 }).setView([41.0, 29.0], 19);
const satellite = L.tileLayer(
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  { maxZoom: 22, maxNativeZoom: 19, attribution: "Esri World Imagery" }).addTo(map);
const streets = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png",
  { maxZoom: 22, maxNativeZoom: 19, attribution: "© OpenStreetMap" });
L.control.layers({ "Uydu": satellite, "Harita": streets }).addTo(map);
L.control.scale({ imperial: false }).addTo(map);
map.pm.setGlobalOptions({ snappable: true });

const robotMarker = L.circleMarker([0, 0], { radius: 8, color: "#fff", weight: 2,
  fillColor: "#2463eb", fillOpacity: 1 });
const robotHeading = L.polyline([], { color: "#2463eb", weight: 3 });
const detectedLayer = L.layerGroup().addTo(map);

// --- helpers -----------------------------------------------------------------
async function api(method, path, body) {
  const options = { method, headers: {} };
  if (body !== undefined) {
    options.headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(body);
  }
  const response = await fetch(path, options);
  let data = {};
  try { data = await response.json(); } catch (e) { /* empty body */ }
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}

function say(id, text, kind) {
  const el = $(id);
  el.textContent = text;
  el.className = "message" + (kind ? " " + kind : "");
}

function offsetLatLng(latlng, bearingDeg, meters) {
  // Local spherical offset — only to draw a heading arrow.
  const R = 6371000;
  const b = bearingDeg * Math.PI / 180;
  const dLat = (meters * Math.cos(b)) / R * 180 / Math.PI;
  const dLon = (meters * Math.sin(b)) / (R * Math.cos(latlng.lat * Math.PI / 180)) * 180 / Math.PI;
  return L.latLng(latlng.lat + dLat, latlng.lng + dLon);
}

function uniqueName(kind) {
  const base = { area: "alan", keepout: "girilmez", obstacle: "engel",
                 station: "home", route: "rota" }[kind];
  if (kind === "station" && !features.some((f) => f.kind === "station" && f.name === "home")) {
    return "home";        // battery_supervisor's dock_id: the station it docks at
  }
  for (let i = 1; ; i++) {
    const name = base + i;
    if (!features.some((f) => f.kind === kind && f.name === name)) return name;
  }
}

// --- site: layers <-> features ---------------------------------------------
function styleFor(feature) {
  if (feature.kind === "station") return;
  feature.layer.setStyle(KIND_STYLE[feature.kind]);
}

function stationArrow(feature) {
  if (feature.arrow) map.removeLayer(feature.arrow);
  const at = feature.layer.getLatLng();
  feature.arrow = L.polyline([at, offsetLatLng(at, feature.props.heading_deg || 0, 1.5)],
                             { color: "#0b7285", weight: 4 }).addTo(map);
}

function addFeature(kind, layer, name, props) {
  const feature = { kind, name, props: props || {}, layer };
  features.push(feature);
  layer.addTo(map);
  styleFor(feature);
  if (kind === "station") stationArrow(feature);
  layer.on("click", (e) => { L.DomEvent.stopPropagation(e); select(feature); });
  layer.on("pm:edit pm:dragend", () => { if (kind === "station") stationArrow(feature); });
  renderList();
  return feature;
}

function layerFromGeoJSON(feature) {
  const kind = feature.properties.kind;
  const g = feature.geometry;
  if (g.type === "Polygon") {
    return L.polygon(g.coordinates.map((ring) => ring.slice(0, -1).map(([lon, lat]) => [lat, lon])));
  }
  if (g.type === "LineString") return L.polyline(g.coordinates.map(([lon, lat]) => [lat, lon]));
  if (g.type === "Point") {
    const at = [g.coordinates[1], g.coordinates[0]];
    if (kind === "obstacle") return L.circle(at, { radius: feature.properties.radius_m || 0.5 });
    return L.marker(at, { draggable: false });
  }
  throw new Error("unsupported geometry " + g.type);
}

function toGeoJSON(feature) {
  const properties = Object.assign({}, feature.props, { kind: feature.kind, name: feature.name });
  let geometry;
  if (feature.kind === "obstacle" && feature.layer instanceof L.Circle) {
    const c = feature.layer.getLatLng();
    geometry = { type: "Point", coordinates: [c.lng, c.lat] };
    properties.radius_m = Math.round(feature.layer.getRadius() * 100) / 100;
  } else {
    geometry = feature.layer.toGeoJSON().geometry;
  }
  return { type: "Feature", geometry, properties };
}

function clearSite() {
  for (const f of features) {
    map.removeLayer(f.layer);
    if (f.arrow) map.removeLayer(f.arrow);
  }
  features.length = 0;
  select(null);
}

async function loadSite(fit) {
  const doc = await api("GET", "/api/site");
  clearSite();
  for (const f of doc.features || []) {
    const props = Object.assign({}, f.properties);
    const kind = props.kind, name = props.name;
    delete props.kind; delete props.name;
    try { addFeature(kind, layerFromGeoJSON(f), name, props); } catch (e) { console.warn(e); }
  }
  if (fit && features.length) {
    const group = L.featureGroup(features.map((f) => f.layer));
    map.fitBounds(group.getBounds().pad(0.3), { maxZoom: 20 });
  }
  renderTargets();
}

function renderList() {
  const list = $("feature-list");
  list.innerHTML = "";
  for (const f of features) {
    const li = document.createElement("li");
    li.innerHTML = `<span class="kind">${KIND_LABEL[f.kind] || f.kind}</span>`;
    li.appendChild(document.createTextNode(f.name));
    li.onclick = () => { select(f); if (f.layer.getBounds) map.fitBounds(f.layer.getBounds(), { maxZoom: 21 }); else map.panTo(f.layer.getLatLng()); };
    list.appendChild(li);
  }
  renderTargets();
}

function select(feature) {
  if (selected && selected.layer.pm) selected.layer.pm.disable();
  selected = feature;
  const editor = $("feature-editor");
  if (!feature) { editor.hidden = true; return; }
  editor.hidden = false;
  $("fe-title").textContent = KIND_LABEL[feature.kind];
  $("fe-name").value = feature.name;
  $("fe-radius-row").hidden = feature.kind !== "obstacle";
  $("fe-heading-row").hidden = feature.kind !== "station";
  if (feature.kind === "obstacle") $("fe-radius").value = feature.layer.getRadius().toFixed(2);
  if (feature.kind === "station") $("fe-heading").value = feature.props.heading_deg || 0;
  if (feature.layer.pm) {
    if (feature.kind === "station") feature.layer.pm.enableLayerDrag();
    else feature.layer.pm.enable({ allowSelfIntersection: false });
  }
}

$("fe-apply").onclick = () => {
  if (!selected) return;
  const name = $("fe-name").value.trim();
  if (!name) return say("site-message", "Ad boş olamaz.", "error");
  selected.name = name;
  if (selected.kind === "obstacle") selected.layer.setRadius(parseFloat($("fe-radius").value) || 0.5);
  if (selected.kind === "station") {
    selected.props.heading_deg = ((parseFloat($("fe-heading").value) || 0) % 360 + 360) % 360;
    stationArrow(selected);
  }
  renderList();
  say("site-message", "Uygulandı — robota göndermek için “Sahayı kaydet”.", "");
};

$("fe-delete").onclick = () => {
  if (!selected) return;
  map.removeLayer(selected.layer);
  if (selected.arrow) map.removeLayer(selected.arrow);
  features.splice(features.indexOf(selected), 1);
  select(null);
  renderList();
};

for (const button of document.querySelectorAll("[data-draw]")) {
  button.onclick = () => {
    armedKind = button.dataset.draw;
    document.querySelectorAll("[data-draw]").forEach((b) => b.classList.toggle("armed", b === button));
    map.pm.enableDraw(button.dataset.shape, { allowSelfIntersection: false,
      pathOptions: KIND_STYLE[armedKind] || {} });
  };
}

map.on("pm:create", (e) => {
  const kind = armedKind;
  armedKind = null;
  document.querySelectorAll("[data-draw]").forEach((b) => b.classList.remove("armed"));
  map.removeLayer(e.layer);
  let layer = e.layer;
  const props = {};
  if (kind === "obstacle") layer = L.circle(e.layer.getLatLng(), { radius: 0.5 });
  if (kind === "station") props.heading_deg = 0;
  const feature = addFeature(kind, layer, uniqueName(kind), props);
  select(feature);
});

$("site-save").onclick = async () => {
  const doc = { type: "FeatureCollection", features: features.map(toGeoJSON) };
  try {
    const r = await api("PUT", "/api/site", doc);
    say("site-message", r.reloaded ? `Kaydedildi (${r.features} öğe), robot yeniden yükledi.`
                                   : `Kaydedildi, ama robot yeniden yükleyemedi: ${r.reload_message}`,
        r.reloaded ? "ok" : "error");
  } catch (e) {
    say("site-message", e.message, "error");
  }
};
$("site-reload").onclick = () => loadSite(true).then(() => say("site-message", "Yüklendi.", ""),
                                                     (e) => say("site-message", e.message, "error"));

// --- tabs ---------------------------------------------------------------------
for (const button of document.querySelectorAll("nav button")) {
  button.onclick = () => {
    document.querySelectorAll("nav button").forEach((b) => b.classList.toggle("active", b === button));
    document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.id === "tab-" + button.dataset.tab));
  };
}

// --- missions -----------------------------------------------------------------
function renderTargets() {
  const type = taskTypes[$("m-type").value];
  if (!type) return;
  const select = $("m-target");
  const keep = select.value;
  select.innerHTML = "";
  const pool = type.on === "area" ? "area" : type.on === "route" ? "route" : null;
  $("m-target-row").hidden = !pool;
  $("m-goto-hint").hidden = type.on !== "target";
  if (!pool) return;
  for (const f of features.filter((x) => x.kind === pool)) {
    const option = document.createElement("option");
    option.value = option.textContent = f.name;
    select.appendChild(option);
  }
  if ([...select.options].some((o) => o.value === keep)) select.value = keep;
}

function renderParams() {
  const type = taskTypes[$("m-type").value];
  $("m-describe").textContent = { area: "Sahadaki bir alanda çalışır.",
    route: "Sahadaki bir rotayı sürer.", target: "Haritada tıkladığın noktaya gider." }[type.on] || "";
  const box = $("m-params");
  box.innerHTML = "";
  for (const [key, p] of Object.entries(type.params)) {
    const label = document.createElement("label");
    let input;
    if (p.kind === "bool") {
      label.className = "check";
      input = document.createElement("input");
      input.type = "checkbox";
      input.checked = !!p.default;
      label.append(input, document.createTextNode(" " + key));
    } else if (p.choices.length) {
      input = document.createElement("select");
      for (const c of p.choices) input.append(new Option(c, c));
      input.value = p.default;
      label.append(document.createTextNode(key), input);
    } else {
      input = document.createElement("input");
      input.type = "number";
      input.step = p.kind === "int" ? "1" : "any";
      if (p.low !== null) input.min = p.low;
      if (p.high !== null) input.max = p.high;
      input.placeholder = p.optional ? "otomatik" : "";
      input.value = p.default === null ? "" : p.default;
      label.append(document.createTextNode(key), input);
    }
    input.dataset.key = key;
    input.dataset.kind = p.kind;
    input.dataset.optional = p.optional;
    box.appendChild(label);
  }
  renderTargets();
}

function readParams() {
  const params = {};
  for (const input of $("m-params").querySelectorAll("[data-key]")) {
    const kind = input.dataset.kind;
    if (kind === "bool") params[input.dataset.key] = input.checked;
    else if (kind === "str") params[input.dataset.key] = input.value;
    else if (input.value === "") {
      if (input.dataset.optional !== "true") throw new Error(input.dataset.key + " boş olamaz");
    } else {
      const v = Number(input.value);
      params[input.dataset.key] = kind === "int" ? Math.round(v) : v;
    }
  }
  return params;
}

$("m-type").onchange = renderParams;

map.on("click", (e) => {
  const type = taskTypes[$("m-type").value];
  if (!type || type.on !== "target" || !$("tab-mission").classList.contains("active")) return;
  gotoTarget = [e.latlng.lng, e.latlng.lat];
  if (gotoMarker) map.removeLayer(gotoMarker);
  gotoMarker = L.marker(e.latlng).addTo(map);
});

$("m-start").onclick = async () => {
  const name = $("m-type").value;
  const type = taskTypes[name];
  try {
    const spec = { type: name, params: readParams() };
    if (type.on === "target") {
      if (!gotoTarget) throw new Error("Önce haritada hedef noktaya tıkla.");
      spec.target = gotoTarget;
    } else {
      if (!$("m-target").value) throw new Error("Bu görev için sahada uygun bir " +
        (type.on === "area" ? "alan" : "rota") + " yok — önce çiz ve kaydet.");
      spec[type.on] = $("m-target").value;
    }
    const r = await api("POST", "/api/mission", spec);
    say("mission-message", "Başladı: " + r.message, "ok");
  } catch (e) {
    say("mission-message", e.message, "error");
  }
};
for (const command of ["pause", "resume", "abort"]) {
  $("m-" + command).onclick = () => api("POST", "/api/mission/" + command)
    .then((r) => say("mission-message", r.message, "ok"), (e) => say("mission-message", e.message, "error"));
}

// --- live status --------------------------------------------------------------
const STATE_TR = { IDLE: "boşta", PLANNING: "planlıyor", RUNNING: "çalışıyor",
  WAITING_DOCK: "istasyonda bekliyor", PAUSED: "duraklatıldı", DONE: "bitti", FAILED: "başarısız" };

async function pollStatus() {
  try {
    const s = await api("GET", "/api/status");
    if (s.robot) {
      const at = L.latLng(s.robot.lat, s.robot.lon);
      robotMarker.setLatLng(at).addTo(map);
      if (!centredOnRobot && !features.length) map.setView(at, 20);
      centredOnRobot = true;
      if (s.robot.heading_deg !== null && s.robot.heading_deg !== undefined) {
        robotHeading.setLatLngs([at, offsetLatLng(at, s.robot.heading_deg, 1.0)]).addTo(map);
      }
    }
    const m = s.mission;
    const mission = m ? `${STATE_TR[m.state] || m.state}` +
      (m.total ? ` · ${Math.min(m.index + 1, m.total)}/${m.total}` : "") +
      (m.work ? " · biçiyor" : "") : "görev yürütücü yok";
    $("status-line").textContent = `${mission} · ${s.supervisor || "supervisor ?"}`;
    $("battery").textContent = s.battery && s.battery.percentage !== null
      ? `🔋 %${Math.round(s.battery.percentage * 100)}` : "";
    $("mission-status").textContent = m ? JSON.stringify(m, null, 1) : "";
  } catch (e) {
    $("status-line").textContent = "Robota ulaşılamıyor: " + e.message;
  }
}

// --- detections ---------------------------------------------------------------
async function pollDetected() {
  let items = [];
  try { items = await api("GET", "/api/detected"); } catch (e) { return; }
  $("detected-count").textContent = items.length ? `(${items.length})` : "";
  detectedLayer.clearLayers();
  const list = $("detected-list");
  const focused = document.activeElement && list.contains(document.activeElement);
  for (const d of items) {
    if (d.lat === null || d.lon === null) continue;
    L.circle([d.lat, d.lon], { radius: d.radius_m, color: "#c4314b", dashArray: "4 3" })
      .bindTooltip(`Tespit #${d.index} · ${d.count} basış`).addTo(detectedLayer);
  }
  if (focused) return;            // do not rebuild while someone is typing
  list.innerHTML = "";
  for (const d of items) {
    const li = document.createElement("li");
    li.innerHTML = `<div><b>#${d.index}</b> · ${d.count} basış · ${new Date(d.last_seen * 1000).toLocaleString("tr-TR")}</div>`;
    const name = document.createElement("input"); name.placeholder = "ad (ör. agac)";
    name.title = "sahadaki engelin adı";
    const radius = document.createElement("input"); radius.type = "number"; radius.step = "0.05";
    radius.min = "0.05"; radius.value = d.radius_m; radius.title = "gerçek yarıçap (m)";
    const ok = document.createElement("button"); ok.textContent = "Onayla"; ok.className = "primary";
    const drop = document.createElement("button"); drop.textContent = "Sil"; drop.className = "danger";
    const show = document.createElement("button"); show.textContent = "Göster";
    show.onclick = () => d.lat !== null && map.setView([d.lat, d.lon], 21);
    ok.onclick = () => api("POST", `/api/detected/${d.index}/confirm`,
                           { name: name.value.trim(), radius_m: Number(radius.value) })
      .then(() => { say("detected-message", "Sahaya eklendi.", "ok"); loadSite(false); pollDetected(); },
            (e) => say("detected-message", e.message, "error"));
    drop.onclick = () => api("POST", `/api/detected/${d.index}/drop`)
      .then(() => { say("detected-message", "Silindi.", "ok"); pollDetected(); },
            (e) => say("detected-message", e.message, "error"));
    const row = document.createElement("div"); row.className = "row";
    row.append(name, radius, ok, drop, show);
    li.appendChild(row);
    list.appendChild(li);
  }
}

// --- start --------------------------------------------------------------------
(async function main() {
  try {
    taskTypes = await api("GET", "/api/task_types");
    for (const [name, t] of Object.entries(taskTypes)) $("m-type").append(new Option(t.describe, name));
    renderParams();
    await loadSite(true);
  } catch (e) {
    say("site-message", "Yüklenemedi: " + e.message, "error");
  }
  pollStatus(); pollDetected();
  setInterval(pollStatus, 1000);
  setInterval(pollDetected, 3000);
})();
