// Record page controller: wires KettleBle -> Recorder -> DOM. Pure helpers live in format.js.
import { KettleBle } from "./ble.js";
import { Recorder } from "./recorder.js";
import { decodeStatus, encodeCommand } from "./protocol.js";
import { MARKERS, buildMeta, formatDuration, isLowBattery, measuredRateHz, pkrawFilename } from "./format.js";

const $ = (id) => document.getElementById(id);
const el = {
  btMissing: $("bt-missing"), lowBattery: $("low-battery"),
  connState: $("conn-state"), connect: $("connect"),
  start: $("start"), stop: $("stop"), message: $("message"),
  markers: $("markers"), markerForm: $("marker-free"), markerText: $("marker-text"), markerAdd: $("marker-add"),
  battery: $("r-battery"), samples: $("r-samples"), rate: $("r-rate"),
  gaps: $("r-gaps"), dropped: $("r-dropped"), duration: $("r-duration"),
};

const STOP_WRITE_TIMEOUT_MS = 2000;
const phoneNow = () => performance.timeOrigin + performance.now();

const ble = new KettleBle();
let recorder = null;      // exists from Start until the file has been saved
let recording = false;    // streaming in progress (false after Stop while the file is unsaved)
let startedAt = 0;        // performance.now() at Start
let stoppedAt = 0;
let linkState = "disconnected"; // disconnected | connecting | connected | reconnecting
let lastStatus = null;
let wakeLock = null;
let ticker = null;

function say(text, isError = false) {
  el.message.textContent = text;
  el.message.classList.toggle("error", isError);
}

const hasBluetooth = !!navigator.bluetooth;
el.btMissing.hidden = hasBluetooth;

// ---- rendering -------------------------------------------------------------------------------

const LINK_TEXT = {
  disconnected: "Not connected",
  connecting: "Connecting...",
  connected: "Connected to PowerKettle",
  reconnecting: "Connection lost, reconnecting...",
};

function render() {
  el.connState.textContent = LINK_TEXT[linkState];
  el.connect.disabled = !hasBluetooth || linkState === "connecting" || linkState === "connected" || linkState === "reconnecting";
  el.start.disabled = !(linkState === "connected" && recorder === null);
  el.stop.disabled = recorder === null;
  const canMark = recording;
  for (const b of el.markers.querySelectorAll("button")) b.disabled = !canMark;
  el.markerAdd.disabled = !canMark;

  el.battery.textContent = lastStatus ? `${lastStatus.batteryMv} mV` : "-";
  el.lowBattery.hidden = !isLowBattery(lastStatus);
  const stats = recorder?.stats;
  el.samples.textContent = stats ? String(stats.samples) : "0";
  el.rate.textContent = stats && stats.durationS > 0 ? measuredRateHz(stats.samples, stats.durationS).toFixed(1) : "-";
  el.gaps.textContent = stats ? String(stats.gaps) : "0";
  el.dropped.textContent = stats?.lastStatus ? String(stats.lastStatus.dropped) : "-";
  const elapsedMs = recorder ? (recording ? performance.now() : stoppedAt) - startedAt : 0;
  el.duration.textContent = formatDuration(elapsedMs / 1000);
}

// ---- wake lock -------------------------------------------------------------------------------

async function acquireWakeLock() {
  if (!("wakeLock" in navigator) || wakeLock) return;
  try {
    wakeLock = await navigator.wakeLock.request("screen");
    wakeLock.addEventListener("release", () => { wakeLock = null; });
  } catch (err) {
    console.warn("wake lock unavailable", err);
    say("Could not keep the screen awake; keep the phone unlocked.", true);
  }
}

async function releaseWakeLock() {
  const lock = wakeLock;
  wakeLock = null;
  try { await lock?.release(); } catch { /* already released */ }
}

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible" && recording) acquireWakeLock();
});

// ---- BLE events ------------------------------------------------------------------------------

function logEvent(obj) {
  recorder?.event(obj, phoneNow());
}

ble.addEventListener("imu", (e) => {
  if (!recording) return;
  try {
    recorder.onImu(e.detail.bytes, e.detail.phoneMs);
  } catch (err) {
    console.warn("bad IMU packet", err);
    say(`Bad IMU packet skipped: ${err.message}`, true);
  }
});

ble.addEventListener("status", (e) => {
  const { bytes, phoneMs } = e.detail;
  try {
    if (recording) recorder.onStatus(bytes, phoneMs); // validates, then logs
    lastStatus = decodeStatus(new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength));
  } catch (err) {
    console.warn("bad status packet", err);
    say(`Bad status packet skipped: ${err.message}`, true);
  }
  if (!recording) render();
});

ble.addEventListener("connected", async (e) => {
  linkState = "connected";
  if (recorder && recording) {
    logEvent({ e: e.detail.reconnect ? "reconnect" : "connect" });
    if (!e.detail.reconnect) {
      // Manual reconnect after giving up: ble.js only auto-resumes on its own retries.
      try { await ble.send(encodeCommand("start")); logEvent({ e: "start" }); } catch (err) { say(`Could not resume: ${err.message}`, true); }
    }
    say(e.detail.reconnect ? "Reconnected. Streaming resumed (new run in the log)." : "Reconnected.");
  } else {
    say("");
  }
  render();
});

ble.addEventListener("disconnected", (e) => {
  linkState = e.detail.willRetry ? "reconnecting" : "disconnected";
  if (recorder && recording) {
    logEvent({ e: "disconnect" });
    if (e.detail.willRetry) say("Connection lost, retrying...", true);
  }
  render();
});

ble.addEventListener("reconnecting", (e) => {
  say(`Reconnecting (attempt ${e.detail.attempt} of 5)...`, true);
});

ble.addEventListener("gaveup", () => {
  linkState = "disconnected";
  say(recording ? "Could not reconnect. Tap Connect to resume, or Stop & save to keep what you have." : "Could not reconnect.", true);
  render();
});

// ---- actions ---------------------------------------------------------------------------------

el.connect.addEventListener("click", async () => {
  linkState = "connecting";
  say("");
  render();
  try {
    await ble.connect();
  } catch (err) {
    linkState = "disconnected";
    if (err.name !== "NotFoundError") say(`Connect failed: ${err.message}`, true); // NotFoundError = chooser cancelled
    render();
  }
});

el.start.addEventListener("click", async () => {
  recorder = new Recorder(buildMeta(navigator.userAgent));
  recording = true;
  startedAt = performance.now();
  say("");
  logEvent({ e: "start" });
  try {
    // Notifications are already enabled by ble.connect(); only now is `start` written.
    await ble.send(encodeCommand("start"));
  } catch (err) {
    recorder = null;
    recording = false;
    say(`Could not start: ${err.message}`, true);
    render();
    return;
  }
  acquireWakeLock();
  ticker = setInterval(render, 250);
  render();
});

function withTimeout(promise, ms) {
  return Promise.race([promise, new Promise((_, rej) => setTimeout(() => rej(new Error("timed out")), ms))]);
}

async function stopStreaming() {
  recording = false;
  stoppedAt = performance.now();
  clearInterval(ticker);
  ticker = null;
  recorder.event({ e: "stop" }, phoneNow());
  releaseWakeLock();
  if (linkState === "connected") {
    try { await withTimeout(ble.send(encodeCommand("stop")), STOP_WRITE_TIMEOUT_MS); } catch (err) { console.warn("stop write failed", err); }
  }
}

function download(file) {
  const url = URL.createObjectURL(file);
  const a = document.createElement("a");
  a.href = url;
  a.download = file.name;
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}

/** Returns true once the file left the page (shared or downloaded), false if the user cancelled. */
async function saveFile(file) {
  if (navigator.canShare?.({ files: [file] })) {
    try {
      await navigator.share({ files: [file], title: file.name });
      return true;
    } catch (err) {
      if (err.name === "AbortError") return false;
      console.warn("share failed, downloading instead", err);
    }
  }
  download(file);
  return true;
}

el.stop.addEventListener("click", async () => {
  el.stop.disabled = true;
  if (recording) await stopStreaming();
  const name = pkrawFilename(new Date());
  const file = new File([recorder.bytes()], name, { type: "application/octet-stream" });
  let saved = false;
  try {
    saved = await saveFile(file);
  } catch (err) {
    say(`Save failed: ${err.message}. Tap Stop & save to try again.`, true);
  }
  if (saved) {
    say(`Saved ${name} (${(file.size / 1024).toFixed(0)} kB).`);
    recorder = null;
  } else if (recorder) {
    say("Not saved yet. Tap Stop & save to try again.", true);
  }
  render();
});

function addMarker(text) {
  if (!recording || !text) return;
  recorder.marker(text, phoneNow());
}

function flash(button) {
  button.classList.add("flash");
  setTimeout(() => button.classList.remove("flash"), 400);
}

for (const label of MARKERS) {
  const b = document.createElement("button");
  b.type = "button";
  b.textContent = label;
  b.addEventListener("click", () => { addMarker(label); flash(b); });
  el.markers.append(b);
}

el.markerForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const text = el.markerText.value.trim();
  if (!text) return;
  addMarker(text);
  flash(el.markerAdd);
  el.markerText.value = "";
});

window.addEventListener("beforeunload", (e) => {
  if (recorder) e.preventDefault(); // unsaved recording
});

render();
