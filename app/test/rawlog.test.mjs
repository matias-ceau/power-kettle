import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { RawLogWriter, TYPE } from "../js/rawlog.js";
import { IMU_GOLDEN, STATUS_GOLDEN } from "./golden.mjs";

const GOLDEN = new URL("../../tests/fixtures/golden.pkraw", import.meta.url);
const bytes = (hex) => Uint8Array.from(Buffer.from(hex, "hex"));
const enc = (s) => new TextEncoder().encode(s);

function goldenWriter() {
  const w = new RawLogWriter({ app: "golden", v: 1 });
  w.add(TYPE.STATUS, 1000.0, bytes(STATUS_GOLDEN));
  w.add(TYPE.IMU, 1001.5, bytes(IMU_GOLDEN));
  w.add(TYPE.MARKER, 1002.0, enc("snatch L"));
  w.add(TYPE.EVENT, 1003.0, enc('{"e":"disconnect"}'));
  return w;
}

test("TYPE constants", () => {
  assert.deepEqual({ ...TYPE }, { IMU: 1, STATUS: 2, MARKER: 3, EVENT: 4 });
});

test("writer output is byte-identical to golden.pkraw", () => {
  const expected = new Uint8Array(readFileSync(GOLDEN));
  assert.deepEqual(goldenWriter().toBytes(), expected);
});

test("toBlob has the same bytes", async () => {
  const blob = goldenWriter().toBlob();
  const got = new Uint8Array(await blob.arrayBuffer());
  assert.deepEqual(got, new Uint8Array(readFileSync(GOLDEN)));
});

test("toBytes is repeatable and add can continue after it", () => {
  const w = goldenWriter();
  const a = w.toBytes();
  w.add(TYPE.MARKER, 1004.0, enc("x"));
  const b = w.toBytes();
  assert.equal(b.length, a.length + 1 + 8 + 2 + 1);
  assert.deepEqual(b.subarray(0, a.length), a);
});

test("payload longer than u16 throws", () => {
  const w = new RawLogWriter({});
  assert.throws(() => w.add(TYPE.IMU, 0, new Uint8Array(0x10000)), /long/);
});

test("add copies the payload", () => {
  const w = new RawLogWriter({});
  const p = Uint8Array.of(1, 2, 3);
  w.add(TYPE.IMU, 0, p);
  p[0] = 99;
  assert.equal(w.toBytes().at(-3), 1);
});
