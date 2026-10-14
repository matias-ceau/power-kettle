import test from "node:test";
import assert from "node:assert/strict";
import { pkrawFilename, formatDuration, measuredRateHz, buildMeta, isLowBattery, MARKERS } from "../js/format.js";

test("pkrawFilename uses local time, zero padded", () => {
  assert.equal(pkrawFilename(new Date(2026, 0, 5, 7, 8, 9)), "pk-20260105-070809.pkraw");
  assert.equal(pkrawFilename(new Date(2026, 11, 31, 23, 59, 59)), "pk-20261231-235959.pkraw");
});

test("formatDuration is m:ss, h:mm:ss from an hour", () => {
  assert.equal(formatDuration(0), "0:00");
  assert.equal(formatDuration(59.9), "0:59");
  assert.equal(formatDuration(65), "1:05");
  assert.equal(formatDuration(3600), "1:00:00");
  assert.equal(formatDuration(3725), "1:02:05");
  assert.equal(formatDuration(-3), "0:00");
});

test("measuredRateHz is samples per second, 0 when no time elapsed", () => {
  assert.equal(measuredRateHz(4000, 10), 400);
  assert.equal(measuredRateHz(5, 0), 0);
  assert.equal(measuredRateHz(0, 0), 0);
});

test("buildMeta matches the recorder meta contract", () => {
  const m = buildMeta("UA/1", new Date(Date.UTC(2026, 9, 14, 12, 0, 0)));
  assert.deepEqual(m, { app: "powerkettle-record", v: 1, ua: "UA/1", created: "2026-10-14T12:00:00.000Z" });
});

test("isLowBattery reads status flag bit1", () => {
  assert.equal(isLowBattery({ flags: 0b01 }), false);
  assert.equal(isLowBattery({ flags: 0b10 }), true);
  assert.equal(isLowBattery(null), false);
});

test("MARKERS lists the brief's exercise markers", () => {
  assert.deepEqual(MARKERS, ["snatch L", "snatch R", "swing 2H", "swing L", "swing R", "rest", "overhead hold", "table lift"]);
});
