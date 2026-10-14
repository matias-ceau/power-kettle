// Pure recording state: no BLE, no DOM. Feeds a RawLogWriter and tracks live stats.
import { decodeImuHeader, decodeStatus, IMU_HEADER_BYTES, IMU_SAMPLE_BYTES } from "./protocol.js";
import { RawLogWriter, TYPE } from "./rawlog.js";

const dvOf = (bytes) => new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

export class Recorder {
  #writer;
  #samples = 0;
  #packets = 0;
  #gaps = 0;
  #runs = 0;
  #prevFirst = null; // first_index of the previous packet
  #nextIndex = null; // first_index expected next; null before the first packet
  #lastStatus = null;
  #firstImuMs = null;
  #lastImuMs = null;

  constructor(meta = {}) {
    this.#writer = new RawLogWriter(meta);
  }

  /**
   * Gaps are judged from first_index continuity (same rule as the Python reader), so the u16
   * `seq` wrapping is irrelevant. A first_index that goes backwards means the device restarted:
   * that begins a new run and is not a gap. Runs are split exactly as the Python reader does
   * (first_index below the previous packet's), so `runs` matches `powerkettle info`.
   */
  onImu(bytes, phoneMs) {
    const dv = dvOf(bytes);
    const h = decodeImuHeader(dv);
    const expected = IMU_HEADER_BYTES + IMU_SAMPLE_BYTES * h.n;
    if (bytes.length !== expected) {
      throw new Error(`IMU packet length ${bytes.length} != ${expected} for n=${h.n}`);
    }
    if (this.#nextIndex !== null && h.firstIndex > this.#nextIndex) this.#gaps++;
    if (this.#prevFirst === null || h.firstIndex < this.#prevFirst) this.#runs++;
    this.#prevFirst = h.firstIndex;
    this.#nextIndex = h.firstIndex + h.n;
    this.#packets++;
    this.#samples += h.n;
    this.#firstImuMs ??= phoneMs;
    this.#lastImuMs = phoneMs;
    this.#writer.add(TYPE.IMU, phoneMs, bytes);
  }

  onStatus(bytes, phoneMs) {
    this.#lastStatus = decodeStatus(dvOf(bytes)); // validates before anything is recorded
    this.#writer.add(TYPE.STATUS, phoneMs, bytes);
  }

  marker(text, phoneMs) {
    this.#writer.add(TYPE.MARKER, phoneMs, new TextEncoder().encode(text));
  }

  event(obj, phoneMs) {
    this.#writer.add(TYPE.EVENT, phoneMs, new TextEncoder().encode(JSON.stringify(obj)));
  }

  /** durationS spans the first to the last IMU packet's phone time. */
  get stats() {
    return {
      samples: this.#samples,
      packets: this.#packets,
      gaps: this.#gaps,
      runs: this.#runs,
      lastStatus: this.#lastStatus,
      durationS: this.#firstImuMs === null ? 0 : (this.#lastImuMs - this.#firstImuMs) / 1000,
    };
  }

  bytes() {
    return this.#writer.toBytes();
  }
}
