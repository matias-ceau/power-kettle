// Power Kettle BLE wire protocol (JS side). All multi-byte fields are little-endian.
// Mirrors powerkettle/protocol.py. Raw counts only; no scaling here.

export const SERVICE_UUID = "f3a10001-6c1e-4b8e-9d2a-5e7c0b4a1d00";
export const IMU_UUID = "f3a10002-6c1e-4b8e-9d2a-5e7c0b4a1d00";
export const CONTROL_UUID = "f3a10003-6c1e-4b8e-9d2a-5e7c0b4a1d00";
export const STATUS_UUID = "f3a10004-6c1e-4b8e-9d2a-5e7c0b4a1d00";

export const IMU_HEADER_BYTES = 8;
export const IMU_SAMPLE_BYTES = 12;
export const MAX_SAMPLES_PER_PACKET = 19;
export const STATUS_BYTES = 28;
export const STATUS_VERSION = 1;

export const VALID_ODR_HZ = [100, 200, 400, 800];
export const VALID_ACC_RANGE_G = [2, 4, 8, 16];
export const VALID_GYRO_RANGE_DPS = [125, 250, 500, 1000, 2000];

const CMD_START = 0x01;
const CMD_STOP = 0x02;
const CMD_SET_CONFIG = 0x03;
const CMD_PING = 0x04;

/** IMU packet header: u16 seq, u32 first_index, u8 n, u8 flags (bit0 = overflow). */
export function decodeImuHeader(dv) {
  if (dv.byteLength < IMU_HEADER_BYTES) {
    throw new Error(`IMU packet too short: ${dv.byteLength} bytes`);
  }
  return {
    seq: dv.getUint16(0, true),
    firstIndex: dv.getUint32(2, true),
    n: dv.getUint8(6),
    flags: dv.getUint8(7),
  };
}

/** Status (28 bytes). `timeUs` is a BigInt (u64). */
export function decodeStatus(dv) {
  if (dv.byteLength !== STATUS_BYTES) {
    throw new Error(`status length ${dv.byteLength} != ${STATUS_BYTES}`);
  }
  const version = dv.getUint8(0);
  if (version !== STATUS_VERSION) throw new Error(`unsupported status version ${version}`);
  return {
    version,
    flags: dv.getUint8(1),
    batteryMv: dv.getUint16(2, true),
    odrHz: dv.getUint16(4, true),
    accRangeG: dv.getUint8(6),
    // byte 7 reserved
    gyroRangeDps: dv.getUint16(8, true),
    tempRaw: dv.getInt16(10, true),
    dropped: dv.getUint32(12, true),
    sampleIndex: dv.getUint32(16, true),
    timeUs: dv.getBigUint64(20, true),
  };
}

function requireMember(name, value, valid) {
  if (!valid.includes(value)) throw new Error(`invalid ${name} ${value}`);
}

/** Encode a control-characteristic write. Throws on unknown ops or invalid config values. */
export function encodeCommand(op, opts = {}) {
  switch (op) {
    case "start": return Uint8Array.of(CMD_START);
    case "stop": return Uint8Array.of(CMD_STOP);
    case "ping": return Uint8Array.of(CMD_PING);
    case "set_config": {
      const { odrHz, accRangeG, gyroRangeDps } = opts;
      requireMember("odrHz", odrHz, VALID_ODR_HZ);
      requireMember("accRangeG", accRangeG, VALID_ACC_RANGE_G);
      requireMember("gyroRangeDps", gyroRangeDps, VALID_GYRO_RANGE_DPS);
      const out = new Uint8Array(6);
      const dv = new DataView(out.buffer);
      dv.setUint8(0, CMD_SET_CONFIG);
      dv.setUint16(1, odrHz, true);
      dv.setUint8(3, accRangeG);
      dv.setUint16(4, gyroRangeDps, true);
      return out;
    }
    default:
      throw new Error(`unknown command ${JSON.stringify(op)}`);
  }
}
