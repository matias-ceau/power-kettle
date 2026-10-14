import test from "node:test";
import assert from "node:assert/strict";
import {
  SERVICE_UUID, IMU_UUID, CONTROL_UUID, STATUS_UUID,
  decodeImuHeader, decodeStatus, encodeCommand,
} from "../js/protocol.js";

import { IMU_GOLDEN, STATUS_GOLDEN } from "./golden.mjs";

const view = (hex) => {
  const b = Uint8Array.from(Buffer.from(hex, "hex"));
  return new DataView(b.buffer, b.byteOffset, b.byteLength);
};
const hex = (u8) => Buffer.from(u8).toString("hex");

test("UUID constants", () => {
  assert.equal(SERVICE_UUID, "f3a10001-6c1e-4b8e-9d2a-5e7c0b4a1d00");
  assert.equal(IMU_UUID, "f3a10002-6c1e-4b8e-9d2a-5e7c0b4a1d00");
  assert.equal(CONTROL_UUID, "f3a10003-6c1e-4b8e-9d2a-5e7c0b4a1d00");
  assert.equal(STATUS_UUID, "f3a10004-6c1e-4b8e-9d2a-5e7c0b4a1d00");
});

test("golden IMU header decodes", () => {
  assert.deepEqual(decodeImuHeader(view(IMU_GOLDEN)), {
    seq: 0x1234, firstIndex: 1000, n: 2, flags: 1,
  });
});

test("IMU header honours byteOffset of the DataView", () => {
  const b = Uint8Array.from([0xaa, 0xbb, ...Buffer.from(IMU_GOLDEN, "hex")]);
  const dv = new DataView(b.buffer, 2);
  assert.equal(decodeImuHeader(dv).firstIndex, 1000);
});

test("IMU header too short throws", () => {
  assert.throws(() => decodeImuHeader(view("341200")), /short/);
});

test("golden status decodes", () => {
  const s = decodeStatus(view(STATUS_GOLDEN));
  assert.deepEqual(s, {
    version: 1, flags: 1, batteryMv: 3987, odrHz: 400, accRangeG: 16,
    gyroRangeDps: 2000, tempRaw: -512, dropped: 7, sampleIndex: 123456,
    timeUs: 9876543210n,
  });
  assert.equal(typeof s.timeUs, "bigint");
});

test("status with wrong length throws", () => {
  assert.throws(() => decodeStatus(view(STATUS_GOLDEN.slice(0, -2))), /length/);
});

test("status with unsupported version throws", () => {
  assert.throws(() => decodeStatus(view("02" + STATUS_GOLDEN.slice(2))), /version/);
});

test("simple commands", () => {
  assert.equal(hex(encodeCommand("start")), "01");
  assert.equal(hex(encodeCommand("stop")), "02");
  assert.equal(hex(encodeCommand("ping")), "04");
});

test("golden set_config", () => {
  const out = encodeCommand("set_config", { odrHz: 400, accRangeG: 16, gyroRangeDps: 2000 });
  assert.ok(out instanceof Uint8Array);
  assert.equal(hex(out), "03900110d007");
});

test("invalid set_config values throw", () => {
  const ok = { odrHz: 400, accRangeG: 16, gyroRangeDps: 2000 };
  assert.throws(() => encodeCommand("set_config", { ...ok, odrHz: 300 }), /odrHz/);
  assert.throws(() => encodeCommand("set_config", { ...ok, accRangeG: 3 }), /accRangeG/);
  assert.throws(() => encodeCommand("set_config", { ...ok, gyroRangeDps: 100 }), /gyroRangeDps/);
  assert.throws(() => encodeCommand("set_config", { odrHz: 400 }), /accRangeG/);
  assert.throws(() => encodeCommand("set_config"), /odrHz/);
});

test("every valid set member encodes", () => {
  for (const odrHz of [100, 200, 400, 800])
    for (const accRangeG of [2, 4, 8, 16])
      for (const gyroRangeDps of [125, 250, 500, 1000, 2000])
        assert.equal(encodeCommand("set_config", { odrHz, accRangeG, gyroRangeDps }).length, 6);
});

test("unknown command throws", () => {
  assert.throws(() => encodeCommand("reboot"), /unknown/);
});
