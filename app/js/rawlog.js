// `.pkraw` raw log writer. Format (see powerkettle/rawlog.py for the reader):
//   magic "PKRAW1\n", u32 meta_len, meta JSON (compact UTF-8),
//   then per record: u8 type, f64 phone_ms, u16 len, payload. All little-endian.

export const TYPE = Object.freeze({ IMU: 1, STATUS: 2, MARKER: 3, EVENT: 4 });

const MAGIC = new TextEncoder().encode("PKRAW1\n");
const REC_HEADER_BYTES = 11;
const MAX_PAYLOAD = 0xffff;

export class RawLogWriter {
  #chunks = [];
  #size = 0;

  constructor(meta) {
    const metaBytes = new TextEncoder().encode(JSON.stringify(meta));
    const head = new Uint8Array(MAGIC.length + 4 + metaBytes.length);
    head.set(MAGIC, 0);
    new DataView(head.buffer).setUint32(MAGIC.length, metaBytes.length, true);
    head.set(metaBytes, MAGIC.length + 4);
    this.#push(head);
  }

  #push(chunk) {
    this.#chunks.push(chunk);
    this.#size += chunk.length;
  }

  /** Append one record. The payload is copied, so callers may reuse their buffer. */
  add(type, phoneMs, payloadBytes) {
    if (payloadBytes.length > MAX_PAYLOAD) {
      throw new Error(`record payload too long: ${payloadBytes.length} bytes`);
    }
    const rec = new Uint8Array(REC_HEADER_BYTES + payloadBytes.length);
    const dv = new DataView(rec.buffer);
    dv.setUint8(0, type);
    dv.setFloat64(1, phoneMs, true);
    dv.setUint16(9, payloadBytes.length, true);
    rec.set(payloadBytes, REC_HEADER_BYTES);
    this.#push(rec);
  }

  toBytes() {
    const out = new Uint8Array(this.#size);
    let pos = 0;
    for (const c of this.#chunks) {
      out.set(c, pos);
      pos += c.length;
    }
    return out;
  }

  toBlob() {
    return new Blob(this.#chunks, { type: "application/octet-stream" });
  }
}
