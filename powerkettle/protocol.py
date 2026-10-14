"""Power Kettle BLE wire protocol: packet decoding and command encoding.

All multi-byte fields are little-endian. Stdlib + numpy only (runs in Pyodide).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

SERVICE_UUID = "f3a10001-6c1e-4b8e-9d2a-5e7c0b4a1d00"
IMU_UUID = "f3a10002-6c1e-4b8e-9d2a-5e7c0b4a1d00"
CONTROL_UUID = "f3a10003-6c1e-4b8e-9d2a-5e7c0b4a1d00"
STATUS_UUID = "f3a10004-6c1e-4b8e-9d2a-5e7c0b4a1d00"

MAX_SAMPLES_PER_PACKET = 19
_IMU_HEADER = struct.Struct("<HIBB")  # seq, first_index, n, flags
_IMU_SAMPLE_BYTES = 12  # 6 x i16
_STATUS = struct.Struct("<BBHHBBHhIIQ")
STATUS_VERSION = 1

VALID_ODR_HZ = (100, 200, 400, 800)
VALID_ACC_RANGE_G = (2, 4, 8, 16)
VALID_GYRO_RANGE_DPS = (125, 250, 500, 1000, 2000)

_CMD_START = 0x01
_CMD_STOP = 0x02
_CMD_SET_CONFIG = 0x03
_CMD_PING = 0x04


@dataclass(frozen=True)
class ImuPacket:
    seq: int
    first_index: int
    flags: int
    samples: np.ndarray  # (n, 6) int16: ax, ay, az, gx, gy, gz


@dataclass(frozen=True)
class Status:
    version: int
    flags: int
    battery_mv: int
    odr_hz: int
    acc_range_g: int
    gyro_range_dps: int
    temp_raw: int
    dropped_samples: int
    sample_index: int
    time_us: int

    @property
    def temperature_c(self) -> float:
        return 23 + self.temp_raw / 512


def decode_imu_packet(b: bytes) -> ImuPacket:
    if len(b) < _IMU_HEADER.size:
        raise ValueError(f"IMU packet too short: {len(b)} bytes")
    seq, first_index, n, flags = _IMU_HEADER.unpack_from(b)
    if n > MAX_SAMPLES_PER_PACKET:
        raise ValueError(f"IMU packet claims {n} samples (max {MAX_SAMPLES_PER_PACKET})")
    expected = _IMU_HEADER.size + _IMU_SAMPLE_BYTES * n
    if len(b) != expected:
        raise ValueError(f"IMU packet length {len(b)} != {expected} for n={n}")
    samples = np.frombuffer(b, dtype="<i2", count=6 * n, offset=_IMU_HEADER.size).reshape(n, 6)
    return ImuPacket(seq, first_index, flags, samples.astype(np.int16))


def decode_status(b: bytes) -> Status:
    if len(b) != _STATUS.size:
        raise ValueError(f"status length {len(b)} != {_STATUS.size}")
    (version, flags, battery_mv, odr_hz, acc, _reserved, gyro,
     temp_raw, dropped, sample_index, time_us) = _STATUS.unpack(b)
    if version != STATUS_VERSION:
        raise ValueError(f"unsupported status version {version}")
    return Status(version, flags, battery_mv, odr_hz, acc, gyro, temp_raw,
                  dropped, sample_index, time_us)


def encode_command(op: str, **kw: int) -> bytes:
    if op == "start":
        return bytes([_CMD_START])
    if op == "stop":
        return bytes([_CMD_STOP])
    if op == "ping":
        return bytes([_CMD_PING])
    if op == "set_config":
        try:
            odr, acc, gyro = kw["odr_hz"], kw["acc_range_g"], kw["gyro_range_dps"]
        except KeyError as e:
            raise ValueError(f"set_config missing {e.args[0]}") from None
        if odr not in VALID_ODR_HZ:
            raise ValueError(f"invalid odr_hz {odr}")
        if acc not in VALID_ACC_RANGE_G:
            raise ValueError(f"invalid acc_range_g {acc}")
        if gyro not in VALID_GYRO_RANGE_DPS:
            raise ValueError(f"invalid gyro_range_dps {gyro}")
        return struct.pack("<BHBH", _CMD_SET_CONFIG, odr, acc, gyro)
    raise ValueError(f"unknown command {op!r}")
