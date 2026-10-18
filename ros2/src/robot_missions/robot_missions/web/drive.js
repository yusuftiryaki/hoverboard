// Driving from the page (docs/missions.md 5a). Uses $ / api / say from app.js.
//
// While driving, a command goes out every 100 ms — zero when nothing is held —
// and the ROBOT decides what that means (teleop.py): it scales the ratios to
// its own limits, stops the wheels 0.3 s after the last command, and drops the
// lease after 2 s. This page only has to keep talking, and to stop talking the
// moment it is no longer in front of someone (tab hidden, window closed).
"use strict";

const drive = { token: null, timer: null, inFlight: false, pad: [0, 0], keys: new Set() };

const KEY_AXES = { ArrowUp: [1, 0], KeyW: [1, 0], ArrowDown: [-1, 0], KeyS: [-1, 0],
                   ArrowLeft: [0, 1], KeyA: [0, 1], ArrowRight: [0, -1], KeyD: [0, -1] };

function driveCommand() {
  // Keys win over the pad when any is held; both are ratios in [-1, 1].
  let linear = drive.pad[0], angular = drive.pad[1];
  if (drive.keys.size) {
    linear = 0; angular = 0;
    for (const code of drive.keys) { linear += KEY_AXES[code][0]; angular += KEY_AXES[code][1]; }
    linear = Math.max(-1, Math.min(1, linear)); angular = Math.max(-1, Math.min(1, angular));
  }
  const scale = Number($("drive-speed").value) / 100;
  return { linear: linear * scale, angular: angular * scale };
}

function renderDrive(text, kind) {
  const driving = drive.token !== null;
  $("drive-start").hidden = driving;
  $("drive-stop").hidden = !driving;
  $("drive-pad").classList.toggle("live", driving);
  if (text !== undefined) say("drive-message", text, kind);
}

async function driveSend() {
  if (drive.token === null || drive.inFlight) return;   // never queue stale commands
  drive.inFlight = true;
  try {
    await api("POST", "/api/teleop", { token: drive.token, ...driveCommand() });
    $("drive-link").textContent = "bağlı";
  } catch (e) {
    if (/acquire|not driving/.test(e.message)) {
      driveStop(false);
      renderDrive("Sürüş kirası düştü (bağlantı koptu mu?). Yeniden 'Sürüşü al'.", "error");
    } else {
      // The robot stops by itself 0.3 s after the last command it heard.
      $("drive-link").textContent = "bağlantı yok — robot kendiliğinden durur";
    }
  } finally {
    drive.inFlight = false;
  }
}

async function driveStart() {
  try {
    const lease = await api("POST", "/api/teleop/acquire", {});
    drive.token = lease.token;
    $("drive-limits").textContent =
      `tavan ${lease.max_linear} m/s · ${lease.max_angular} rad/s (robotun ayarı)`;
    drive.timer = setInterval(driveSend, 100);
    renderDrive("Sürüyorsun. Bırakınca robot durur.", "ok");
  } catch (e) {
    renderDrive(e.message, "error");
  }
}

function driveStop(release = true) {
  clearInterval(drive.timer);
  drive.timer = null;
  drive.pad = [0, 0];
  drive.keys.clear();
  setKnob(0, 0);
  const token = drive.token;
  drive.token = null;
  if (release && token !== null) {
    // sendBeacon survives the page closing; the robot reads the body as JSON.
    navigator.sendBeacon("/api/teleop/release", JSON.stringify({ token }));
  }
  renderDrive("Sürüş bırakıldı.", "");
}

// --- the pad: drag the knob; up = forward, left = turn left ---------------------
function setKnob(x, y) {
  const pad = $("drive-pad"), knob = $("drive-knob");
  const r = pad.clientWidth / 2;
  knob.style.transform = `translate(${x * r * 0.8}px, ${y * r * 0.8}px)`;
}

function padFromEvent(event) {
  const rect = $("drive-pad").getBoundingClientRect();
  let x = (event.clientX - rect.left) / (rect.width / 2) - 1;
  let y = (event.clientY - rect.top) / (rect.height / 2) - 1;
  const length = Math.hypot(x, y);
  if (length > 1) { x /= length; y /= length; }
  setKnob(x, y);
  drive.pad = [-y, -x];
}

$("drive-pad").addEventListener("pointerdown", (event) => {
  if (drive.token === null) return;
  $("drive-pad").setPointerCapture(event.pointerId);
  padFromEvent(event);
});
$("drive-pad").addEventListener("pointermove", (event) => {
  if (drive.token !== null && $("drive-pad").hasPointerCapture(event.pointerId)) padFromEvent(event);
});
for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
  $("drive-pad").addEventListener(name, () => { drive.pad = [0, 0]; setKnob(0, 0); });
}

// --- keys: only while driving and not typing into a field ----------------------
document.addEventListener("keydown", (event) => {
  if (drive.token === null || !(event.code in KEY_AXES)) return;
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName || "")) return;
  event.preventDefault();
  drive.keys.add(event.code);
});
document.addEventListener("keyup", (event) => drive.keys.delete(event.code));
// A key released while the window had no focus never sends keyup.
window.addEventListener("blur", () => { drive.keys.clear(); drive.pad = [0, 0]; setKnob(0, 0); });

// A hidden tab's timers slow to ~1 Hz — the robot would lurch between command
// and deadman. Hidden = not driving.
document.addEventListener("visibilitychange", () => {
  if (document.hidden && drive.token !== null) driveStop();
});
window.addEventListener("pagehide", () => { if (drive.token !== null) driveStop(); });

$("drive-start").onclick = driveStart;
$("drive-stop").onclick = () => driveStop();
$("drive-speed").oninput = () => { $("drive-speed-value").textContent = $("drive-speed").value + "%"; };
renderDrive();
