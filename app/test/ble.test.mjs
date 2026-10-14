import test from "node:test";
import assert from "node:assert/strict";
import { KettleBle } from "../js/ble.js";
import { SERVICE_UUID, IMU_UUID, CONTROL_UUID, STATUS_UUID } from "../js/protocol.js";

// A fake Web Bluetooth stack that records the order of notification/write calls.
function makeFake({ failConnects = 0, reuseChars = false } = {}) {
  const log = [];
  let connectsLeftToFail = failConnects;
  const char = (uuid) => {
    const t = new EventTarget();
    t.uuid = uuid;
    t.startNotifications = async () => { log.push(`notify:${uuid.slice(0, 8)}`); };
    t.writeValueWithResponse = async (b) => { log.push(`write:${Buffer.from(b).toString("hex")}`); };
    // GATT read: resolves with readResult; firesEvent mimics the spec (and Chrome), which also
    // fires characteristicvaluechanged with the same DataView before resolving.
    t.readValue = async () => {
      log.push(`read:${uuid.slice(0, 8)}`);
      t.value = new DataView(t.readResult.slice().buffer);
      if (t.firesEvent) t.dispatchEvent(new Event("characteristicvaluechanged"));
      return t.value;
    };
    return t;
  };
  const device = new EventTarget();
  device.chars = {};
  device.gatt = {
    connected: false,
    async connect() {
      if (connectsLeftToFail > 0) { connectsLeftToFail--; throw new Error("connect failed"); }
      this.connected = true;
      return {
        getPrimaryService: async (u) => {
          assert.equal(u, SERVICE_UUID);
          return {
            getCharacteristic: async (cu) => (reuseChars && device.chars[cu]) || (device.chars[cu] = char(cu)),
          };
        },
      };
    },
    disconnect() {
      if (!this.connected) return;
      this.connected = false;
      device.dispatchEvent(new Event("gattserverdisconnected"));
    },
  };
  const bluetooth = { requestDevice: async () => device };
  return { log, device, bluetooth };
}

const instant = async () => {};
const tick = () => new Promise((r) => setImmediate(r));

test("subscribes to imu_stream and status before any control write", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth });
  await ble.connect();
  await ble.send(Uint8Array.of(1));
  assert.deepEqual(f.log.slice(0, 2).sort(), ["notify:f3a10002", "notify:f3a10004"]);
  assert.equal(f.log[2], "write:01");
});

test("emits imu/status with copied bytes and phoneMs", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth });
  await ble.connect();
  const got = [];
  ble.addEventListener("imu", (e) => got.push(e.detail));
  const buf = Uint8Array.of(9, 8, 7, 6).buffer;
  f.device.chars[IMU_UUID].value = new DataView(buf, 1, 2);
  f.device.chars[IMU_UUID].dispatchEvent(new Event("characteristicvaluechanged"));
  assert.equal(got.length, 1);
  assert.deepEqual([...got[0].bytes], [8, 7]);
  assert.ok(Number.isFinite(got[0].phoneMs));
});

test("reconnect re-subscribes both characteristics, then re-sends start", async () => {
  const f = makeFake();
  const delays = [];
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: async (ms) => { delays.push(ms); } });
  const events = [];
  for (const t of ["connected", "disconnected", "reconnecting", "gaveup", "restarted"]) {
    ble.addEventListener(t, (e) => events.push([t, e.detail]));
  }
  await ble.connect();
  await ble.send(Uint8Array.of(1));
  f.log.length = 0;
  f.device.gatt.disconnect(); // link drop
  await tick(); await tick();
  assert.deepEqual(f.log.slice(0, 2).sort(), ["notify:f3a10002", "notify:f3a10004"]);
  assert.equal(f.log[2], "write:01");
  assert.deepEqual(delays, [1000]);
  // "restarted" follows "connected" so the page logs {"e":"reconnect"} then {"e":"start"},
  // the same events as a manual reconnect.
  assert.deepEqual(events.map((e) => e[0]), ["connected", "disconnected", "reconnecting", "connected", "restarted"]);
  assert.equal(events.at(-2)[1].reconnect, true);
});

test("a reconnect while not streaming emits no restarted", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: instant });
  const events = [];
  for (const t of ["connected", "restarted"]) ble.addEventListener(t, () => events.push(t));
  await ble.connect();
  f.device.gatt.disconnect();
  await tick(); await tick();
  assert.deepEqual(events, ["connected", "connected"]);
});

test("no start is re-sent after a stop", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: instant });
  await ble.connect();
  await ble.send(Uint8Array.of(1));
  await ble.send(Uint8Array.of(2));
  f.log.length = 0;
  f.device.gatt.disconnect();
  await tick(); await tick();
  assert.ok(!f.log.some((l) => l.startsWith("write:")));
});

test("backoff is 1/2/4/8/16 s and gives up after 5 failed attempts", async () => {
  const f = makeFake();
  const delays = [];
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: async (ms) => { delays.push(ms); } });
  const warn = console.warn;
  console.warn = () => {}; // the expected failed attempts are logged by ble.js
  let gaveUp = 0;
  ble.addEventListener("gaveup", () => gaveUp++);
  await ble.connect();
  // every reconnect connect() fails
  f.device.gatt.connect = async () => { throw new Error("nope"); };
  f.device.gatt.disconnect();
  for (let i = 0; i < 10; i++) await tick();
  assert.deepEqual(delays, [1000, 2000, 4000, 8000, 16000]);
  console.warn = warn;
  assert.equal(gaveUp, 1);
});

test("user disconnect does not reconnect", async () => {
  const f = makeFake();
  const delays = [];
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: async (ms) => { delays.push(ms); } });
  const events = [];
  ble.addEventListener("disconnected", (e) => events.push(e.detail));
  await ble.connect();
  ble.disconnect();
  await tick(); await tick();
  assert.deepEqual(delays, []);
  assert.deepEqual(events, [{ willRetry: false }]);
  await assert.rejects(ble.send(Uint8Array.of(1)), /not connected/);
});

test("control writes use writeValueWithResponse", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth });
  await ble.connect();
  const c = f.device.chars[CONTROL_UUID];
  c.writeValue = async () => assert.fail("must not use writeValue");
  await ble.send(Uint8Array.of(4));
  assert.ok(f.log.includes("write:04"));
});

test("a reconnect that reuses characteristic objects does not duplicate packets", async () => {
  const f = makeFake({ reuseChars: true });
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: instant });
  let n = 0;
  ble.addEventListener("imu", () => n++);
  await ble.connect();
  f.device.gatt.disconnect();
  await tick(); await tick();
  const c = f.device.chars[IMU_UUID];
  c.value = new DataView(new ArrayBuffer(8));
  c.dispatchEvent(new Event("characteristicvaluechanged"));
  assert.equal(n, 1);
});

test("stop requested while the retry loop sleeps: reconnect does not re-send start", async () => {
  const f = makeFake();
  let release;
  const gate = new Promise((r) => { release = r; });
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: () => gate });
  await ble.connect();
  await ble.send(Uint8Array.of(1));
  f.device.gatt.disconnect(); // link drop; retry loop now sleeping
  await tick();
  await assert.rejects(ble.send(Uint8Array.of(2)), /not connected/); // stop cannot be delivered
  f.log.length = 0;
  release();
  await tick(); await tick(); await tick();
  assert.deepEqual(f.log.slice(0, 2).sort(), ["notify:f3a10002", "notify:f3a10004"]);
  assert.ok(!f.log.some((l) => l.startsWith("write:")), JSON.stringify(f.log));
});

test("a failed stop write still clears the resume-streaming intent", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth, sleepFn: instant });
  await ble.connect();
  await ble.send(Uint8Array.of(1));
  const c = f.device.chars[CONTROL_UUID];
  const ok = c.writeValueWithResponse;
  c.writeValueWithResponse = async () => { throw new Error("gatt error"); };
  await assert.rejects(ble.send(Uint8Array.of(2)), /gatt error/);
  c.writeValueWithResponse = ok;
  f.log.length = 0;
  f.device.gatt.disconnect();
  await tick(); await tick();
  assert.ok(!f.log.some((l) => l.startsWith("write:")), JSON.stringify(f.log));
});

const STATUS_28 = Uint8Array.from({ length: 28 }, (_, i) => i);

test("status notifications are tagged source notify", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth });
  await ble.connect();
  const got = [];
  ble.addEventListener("status", (e) => got.push(e.detail));
  f.device.chars[STATUS_UUID].value = new DataView(new ArrayBuffer(20));
  f.device.chars[STATUS_UUID].dispatchEvent(new Event("characteristicvaluechanged"));
  assert.equal(got.length, 1);
  assert.equal(got[0].source, "notify");
  assert.equal(got[0].bytes.length, 20);
});

test("readStatus emits the full status read once, tagged source read", async () => {
  for (const firesEvent of [false, true]) {
    const f = makeFake();
    const ble = new KettleBle({ bluetooth: f.bluetooth });
    await ble.connect();
    Object.assign(f.device.chars[STATUS_UUID], { readResult: STATUS_28, firesEvent });
    const got = [];
    ble.addEventListener("status", (e) => got.push(e.detail));
    await ble.readStatus();
    assert.equal(got.length, 1, `firesEvent=${firesEvent}`);
    assert.equal(got[0].source, "read");
    assert.deepEqual([...got[0].bytes], [...STATUS_28]);
    assert.ok(Number.isFinite(got[0].phoneMs));
    assert.ok(f.log.includes("read:f3a10004"));
  }
});

test("readStatus does not overlap reads and rejects when not connected", async () => {
  const f = makeFake();
  const ble = new KettleBle({ bluetooth: f.bluetooth });
  await assert.rejects(ble.readStatus(), /not connected/);
  await ble.connect();
  f.device.chars[STATUS_UUID].readResult = STATUS_28;
  await Promise.all([ble.readStatus(), ble.readStatus()]);
  assert.equal(f.log.filter((l) => l.startsWith("read:")).length, 1);
});
