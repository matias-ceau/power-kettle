import test from "node:test";
import assert from "node:assert/strict";
import { Recorder } from "../js/recorder.js";
import { STATUS_GOLDEN } from "./golden.mjs";

// Build an IMU packet: seq, firstIndex, n, flags, then n zeroed 12-byte samples.
function imu(seq, firstIndex, n, flags = 0) {
  const b = new Uint8Array(8 + 12 * n);
  const dv = new DataView(b.buffer);
  dv.setUint16(0, seq, true);
  dv.setUint32(2, firstIndex, true);
  dv.setUint8(6, n);
  dv.setUint8(7, flags);
  return b;
}
const status = () => Uint8Array.from(Buffer.from(STATUS_GOLDEN, "hex"));

// Parse the raw log bytes into records.
function parse(bytes) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let pos = 7;
  const metaLen = dv.getUint32(pos, true);
  pos += 4 + metaLen;
  const recs = [];
  while (pos < bytes.length) {
    const type = dv.getUint8(pos);
    const phoneMs = dv.getFloat64(pos + 1, true);
    const len = dv.getUint16(pos + 9, true);
    recs.push({ type, phoneMs, payload: bytes.slice(pos + 11, pos + 11 + len) });
    pos += 11 + len;
  }
  return recs;
}

test("three contiguous packets: no gaps, 57 samples", () => {
  const r = new Recorder();
  r.onImu(imu(0, 0, 19), 1000);
  r.onImu(imu(1, 19, 19), 1050);
  r.onImu(imu(2, 38, 19), 1100);
  const s = r.stats;
  assert.equal(s.gaps, 0);
  assert.equal(s.samples, 57);
  assert.equal(s.packets, 3);
  assert.equal(s.lastStatus, null);
  assert.ok(Math.abs(s.durationS - 0.1) < 1e-9);
});

test("a jump in first_index counts one gap", () => {
  const r = new Recorder();
  r.onImu(imu(0, 0, 19), 1000);
  r.onImu(imu(1, 19, 19), 1050);
  r.onImu(imu(2, 100, 19), 1100);
  r.onImu(imu(3, 119, 19), 1150);
  assert.equal(r.stats.gaps, 1);
  assert.equal(r.stats.samples, 76);
});

test("first_index going backwards starts a new run, not a gap", () => {
  const r = new Recorder();
  r.onImu(imu(0, 5000, 19), 1000);
  r.onImu(imu(1, 5019, 19), 1050);
  r.onImu(imu(0, 0, 19), 2000); // device restarted
  r.onImu(imu(1, 19, 19), 2050);
  assert.equal(r.stats.gaps, 0);
  assert.equal(r.stats.samples, 76);
});

test("a gap right after a restart is still detected", () => {
  const r = new Recorder();
  r.onImu(imu(0, 5000, 19), 1000);
  r.onImu(imu(0, 0, 19), 2000);
  r.onImu(imu(1, 50, 19), 2050);
  assert.equal(r.stats.gaps, 1);
});

test("seq wrap 65535 -> 0 is not a gap", () => {
  const r = new Recorder();
  r.onImu(imu(65534, 1000, 19), 1000);
  r.onImu(imu(65535, 1019, 19), 1050);
  r.onImu(imu(0, 1038, 19), 1100);
  assert.equal(r.stats.gaps, 0);
  assert.equal(r.stats.samples, 57);
});

test("disconnect event then more packets: every record retained, in order", () => {
  const r = new Recorder({ app: "t" });
  const p1 = imu(0, 0, 19), p2 = imu(1, 19, 19), p3 = imu(2, 38, 19);
  r.onStatus(status(), 999);
  r.onImu(p1, 1000);
  r.onImu(p2, 1050);
  r.event({ e: "disconnect" }, 1060);
  r.marker("snatch L", 1070);
  r.onImu(p3, 1100);
  const recs = parse(r.bytes());
  assert.deepEqual(recs.map((x) => x.type), [2, 1, 1, 4, 3, 1]);
  assert.deepEqual(recs.map((x) => x.phoneMs), [999, 1000, 1050, 1060, 1070, 1100]);
  assert.deepEqual(recs[1].payload, p1);
  assert.deepEqual(recs[2].payload, p2);
  assert.deepEqual(recs[5].payload, p3);
  assert.equal(new TextDecoder().decode(recs[3].payload), '{"e":"disconnect"}');
  assert.equal(new TextDecoder().decode(recs[4].payload), "snatch L");
  assert.equal(r.stats.gaps, 0);
  assert.equal(r.stats.samples, 57);
});

test("status is recorded and exposed as lastStatus", () => {
  const r = new Recorder();
  r.onStatus(status(), 1000);
  assert.equal(r.stats.lastStatus.batteryMv, 3987);
  assert.equal(r.stats.lastStatus.timeUs, 9876543210n);
});

test("malformed IMU packet throws and records nothing", () => {
  const r = new Recorder();
  assert.throws(() => r.onImu(imu(0, 0, 19).subarray(0, 20), 1000), /length/);
  assert.equal(r.stats.packets, 0);
  assert.equal(parse(r.bytes()).length, 0);
});

test("malformed status throws and is not recorded", () => {
  const r = new Recorder();
  assert.throws(() => r.onStatus(status().subarray(0, 10), 1000));
  assert.equal(parse(r.bytes()).length, 0);
});

test("handles Uint8Array views with a byteOffset", () => {
  const p = imu(7, 42, 3);
  const big = new Uint8Array(p.length + 5);
  big.set(p, 5);
  const r = new Recorder();
  r.onImu(big.subarray(5), 1000);
  assert.equal(r.stats.samples, 3);
  assert.deepEqual(parse(r.bytes())[0].payload, p);
});
