// Web Bluetooth link to the PowerKettle. Emits (CustomEvent, detail):
//   imu / status   {bytes: Uint8Array (copy), phoneMs}
//   connected      {reconnect: boolean}
//   disconnected   {willRetry: boolean}
//   reconnecting   {attempt, delayMs}
//   gaveup         {}            all reconnect attempts failed
// The firmware notifies without checking subscriptions, so notifications on imu_stream AND
// status are enabled (and awaited) before `start` is ever written, on every (re)connect.
import { SERVICE_UUID, IMU_UUID, CONTROL_UUID, STATUS_UUID, encodeCommand } from "./protocol.js";

export const RECONNECT_DELAYS_MS = Object.freeze([1000, 2000, 4000, 8000, 16000]);
const DEVICE_NAME = "PowerKettle";

const phoneNow = () => performance.timeOrigin + performance.now();
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export class KettleBle extends EventTarget {
  #device = null;
  #control = null;
  #streaming = false; // a `start` was sent and no `stop` since: re-send it after a reconnect
  #userClosed = false;
  #generation = 0; // bumped on every connect/disconnect so stale retry loops stop
  #retrying = false; // a reconnect loop is running; drops during it are its own failures
  #delays;
  #sleep;
  #bluetooth;

  /** Options exist for tests; the page uses the defaults. */
  constructor({ delaysMs = RECONNECT_DELAYS_MS, sleepFn = sleep, bluetooth = null } = {}) {
    super();
    this.#delays = delaysMs;
    this.#sleep = sleepFn;
    this.#bluetooth = bluetooth;
  }

  get connected() {
    return !!this.#device?.gatt?.connected && this.#control !== null;
  }

  #emit(type, detail = {}) {
    this.dispatchEvent(new CustomEvent(type, { detail }));
  }

  /** Opens the chooser (needs a user gesture), connects and subscribes. */
  async connect() {
    this.#userClosed = false;
    const gen = ++this.#generation;
    const bluetooth = this.#bluetooth ?? navigator.bluetooth;
    const device = await bluetooth.requestDevice({
      filters: [{ name: DEVICE_NAME }],
      optionalServices: [SERVICE_UUID],
    });
    if (this.#device && this.#device !== device) {
      this.#device.removeEventListener("gattserverdisconnected", this.#onDrop);
    }
    this.#device = device;
    device.addEventListener("gattserverdisconnected", this.#onDrop);
    try {
      await this.#setup();
    } catch (err) {
      // A half-open first connection is not worth retrying behind the user's back.
      this.#userClosed = true;
      this.#control = null;
      try { device.gatt.disconnect(); } catch { /* already down */ }
      throw err;
    }
    if (gen === this.#generation) this.#emit("connected", { reconnect: false });
  }

  disconnect() {
    this.#userClosed = true;
    this.#streaming = false;
    this.#generation++;
    this.#control = null;
    if (!this.#device) return;
    this.#device.gatt?.disconnect();
    this.#emit("disconnected", { willRetry: false });
  }

  /** Control write (with response). Tracks start/stop so a reconnect can resume streaming. */
  async send(cmdBytes) {
    if (!this.#control) throw new Error("not connected");
    await this.#control.writeValueWithResponse(cmdBytes);
    if (cmdBytes[0] === 0x01) this.#streaming = true;
    else if (cmdBytes[0] === 0x02) this.#streaming = false;
  }

  async #setup() {
    const server = await this.#device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE_UUID);
    const [imu, control, status] = await Promise.all([
      service.getCharacteristic(IMU_UUID),
      service.getCharacteristic(CONTROL_UUID),
      service.getCharacteristic(STATUS_UUID),
    ]);
    // Stable handler references: if a browser hands back the same characteristic object after a
    // reconnect, addEventListener de-duplicates instead of delivering every packet twice.
    imu.addEventListener("characteristicvaluechanged", this.#onImu);
    status.addEventListener("characteristicvaluechanged", this.#onStatus);
    await Promise.all([imu.startNotifications(), status.startNotifications()]);
    this.#control = control;
  }

  #onImu = (e) => this.#onValue("imu", e);
  #onStatus = (e) => this.#onValue("status", e);

  #onValue(type, e) {
    const dv = e.target.value;
    const bytes = new Uint8Array(dv.buffer.slice(dv.byteOffset, dv.byteOffset + dv.byteLength));
    this.#emit(type, { bytes, phoneMs: phoneNow() });
  }

  #onDrop = () => {
    if (this.#userClosed || this.#retrying) return;
    this.#control = null;
    const gen = ++this.#generation;
    this.#emit("disconnected", { willRetry: true });
    this.#reconnectLoop(gen);
  };

  async #reconnectLoop(gen) {
    this.#retrying = true;
    try {
      await this.#attemptReconnects(gen);
    } finally {
      this.#retrying = false;
    }
  }

  async #attemptReconnects(gen) {
    for (let i = 0; i < this.#delays.length; i++) {
      const delayMs = this.#delays[i];
      this.#emit("reconnecting", { attempt: i + 1, delayMs });
      await this.#sleep(delayMs);
      if (gen !== this.#generation) return;
      try {
        await this.#setup();
        if (gen !== this.#generation) return;
        if (this.#streaming) await this.#control.writeValueWithResponse(encodeCommand("start"));
        this.#emit("connected", { reconnect: true });
        return;
      } catch (err) {
        console.warn(`reconnect attempt ${i + 1} failed`, err);
        this.#control = null;
        try { this.#device?.gatt?.disconnect(); } catch { /* already down */ }
      }
    }
    if (gen === this.#generation) this.#emit("gaveup");
  }
}
