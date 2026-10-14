// Pure helpers for the record page (no DOM, no BLE) so they can be unit-tested in node.

export const MARKERS = Object.freeze([
  "snatch L", "snatch R", "swing 2H", "swing L", "swing R", "rest", "overhead hold", "table lift",
]);

const LOW_BATTERY_FLAG = 0b10;
const STREAMING_FLAG = 0b01;
// A 28-byte status notified at the default ATT MTU of 23 arrives cut to MTU - 3 = 20 bytes.
const TRUNCATED_STATUS_BYTES = 20;
export const START_ACK_TIMEOUT_MS = 2000;

const pad = (n, w = 2) => String(n).padStart(w, "0");

/** `pk-YYYYMMDD-HHMMSS.pkraw` in the phone's local time. */
export function pkrawFilename(d) {
  const date = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}`;
  const time = `${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
  return `pk-${date}-${time}.pkraw`;
}

/** m:ss, or h:mm:ss from one hour up. */
export function formatDuration(seconds) {
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return h > 0 ? `${h}:${pad(m)}:${pad(s)}` : `${m}:${pad(s)}`;
}

export function measuredRateHz(samples, elapsedS) {
  return elapsedS > 0 ? samples / elapsedS : 0;
}

export function buildMeta(userAgent, now = new Date()) {
  return { app: "powerkettle-record", v: 1, ua: userAgent, created: now.toISOString() };
}

export function isLowBattery(status) {
  return status != null && (status.flags & LOW_BATTERY_FLAG) !== 0;
}

/** True for a status notification cut short by a too-small Bluetooth MTU. */
export function isTruncatedStatus(bytes) {
  return bytes.length === TRUNCATED_STATUS_BYTES;
}

/** Notices a `start` the device ignores: status bit0 (streaming) still clear 2 s after it. */
export class StreamingWatch {
  #sentAt = null; // ms of the last start; null when not watching
  #seen = false;

  started(nowMs) {
    this.#sentAt = nowMs;
    this.#seen = false;
  }

  stop() {
    this.#sentAt = null;
  }

  status(flags) {
    if (flags & STREAMING_FLAG) this.#seen = true;
  }

  stalled(nowMs) {
    return this.#sentAt !== null && !this.#seen && nowMs - this.#sentAt > START_ACK_TIMEOUT_MS;
  }
}
