// Learning mode (docs/learning.md): start / mark / stop an episode, list the
// recorded ones. The robot's episode_recorder does the recording; its status
// comes with /api/status (app.js calls onEpisodeStatus every second).
"use strict";

const MODE_TR = { demo: "gösterim", intervention: "müdahale", autonomous: "otonom" };
const OUTCOME_TR = { success: "başarılı", failure: "başarısız", aborted: "iptal",
                     disk_full: "disk doldu", crashed: "kaydedici çöktü", shutdown: "kapatıldı" };
let learnRecording = null;       // the episode name while recording

function bytes(n) {
  if (n === null || n === undefined) return "?";
  return n >= 1e9 ? (n / 1e9).toFixed(1) + " GB" : (n / 1e6).toFixed(1) + " MB";
}

function clock(seconds) {
  const s = Math.floor(seconds || 0);
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
}

function onEpisodeStatus(e) {
  const recording = !!(e && e.recording);
  $("rec").hidden = !recording;
  // The recorder takes seconds to subscribe; what is driven before READY is
  // missing from the bag (docs/learning.md).
  $("rec").textContent = recording && !e.ready ? "○ HAZIRLANIYOR" : "● REC";
  $("rec").classList.toggle("warming", recording && !e.ready);
  $("learn-idle").hidden = recording;
  $("learn-recording").hidden = !recording;
  if (e && e.free_bytes !== undefined) $("learn-free").textContent = `(boş: ${bytes(e.free_bytes)})`;
  if (recording && !e.ready) {
    $("learn-live").textContent = "○ Kaydedici hazırlanıyor — henüz sürme" +
      (e.waiting_for && e.waiting_for.length ? ` (bekleniyor: ${e.waiting_for.join(", ")})` : "");
  } else if (recording) {
    $("learn-live").textContent = `● ${e.task} · ${MODE_TR[e.mode] || e.mode} · ` +
      `${clock(e.elapsed_s)} · ${bytes(e.size_bytes)} · ${e.markers} işaret`;
  } else if (e && e.last_error && learnRecording) {
    say("learn-message", "Kayıt kendiliğinden bitti: " + e.last_error, "error");
  }
  const was = learnRecording;
  learnRecording = recording ? e.episode : null;
  if (was !== learnRecording) loadEpisodes();
}

async function loadEpisodes() {
  let list = [];
  try { list = (await api("GET", "/api/episodes")).episodes; } catch (e) { return; }
  const ul = $("learn-list");
  ul.innerHTML = "";
  for (const ep of list) {
    const li = document.createElement("li");
    const title = document.createElement("div");
    title.innerHTML = "<b></b> <span class='meta'></span>";
    title.querySelector("b").textContent = ep.task || ep.name;
    title.querySelector(".meta").textContent = [
      ep.started ? new Date(ep.started).toLocaleString("tr-TR") : "",
      MODE_TR[ep.mode] || ep.mode, ep.status === "recording" ? "kaydediliyor"
        : OUTCOME_TR[ep.outcome] || ep.outcome, ep.duration_s !== null ? clock(ep.duration_s) : "",
      bytes(ep.size_bytes), `${ep.markers} işaret`, ep.source === "sim" ? "sim" : ""]
      .filter(Boolean).join(" · ");
    li.appendChild(title);
    const row = document.createElement("div");
    row.className = "row";
    const download = document.createElement("a");
    download.href = `/api/episodes/${encodeURIComponent(ep.name)}.zip`;
    download.textContent = "İndir";
    const drop = document.createElement("button");
    drop.textContent = "Sil";
    drop.className = "danger";
    drop.disabled = ep.name === learnRecording;
    drop.onclick = () => {
      if (!confirm(`${ep.task} (${ep.name}) silinsin mi? Geri alınamaz.`)) return;
      api("POST", `/api/episodes/${encodeURIComponent(ep.name)}/delete`, {})
        .then(() => { say("learn-message", "Silindi.", "ok"); loadEpisodes(); },
              (e) => say("learn-message", e.message, "error"));
    };
    // A recording episode's zip would be a half-written bag.
    if (ep.name !== learnRecording) row.append(download);
    row.append(drop);
    li.appendChild(row);
    ul.appendChild(li);
  }
}

$("learn-start").onclick = () => api("POST", "/api/episode/start", {
  task: $("learn-task").value.trim(), mode: $("learn-mode").value, note: $("learn-note").value })
  .then((r) => say("learn-message", "Kayıt başladı: " + r.episode, "ok"),
        (e) => say("learn-message", e.message, "error"));
$("learn-mark").onclick = () => {
  const label = $("learn-mark-label").value.trim();
  if (!label) return say("learn-message", "İşaretin bir adı olsun.", "error");
  api("POST", "/api/episode/mark", { label })
    .then((r) => { say("learn-message", r.message, "ok"); $("learn-mark-label").value = ""; },
          (e) => say("learn-message", e.message, "error"));
};
$("learn-stop").onclick = () => {
  say("learn-message", "Kayıt kapatılıyor…", "");
  api("POST", "/api/episode/stop", { outcome: $("learn-outcome").value,
                                     note: $("learn-stop-note").value })
    .then((r) => { say("learn-message", r.message, "ok"); $("learn-stop-note").value = ""; },
          (e) => say("learn-message", e.message, "error"));
};
loadEpisodes();
