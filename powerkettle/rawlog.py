"""Raw `.pkraw` log reader/writer and assembly of a log into a Recording.

Stdlib + numpy only (runs in Pyodide).
"""
from __future__ import annotations

import json
import os
import struct
from dataclasses import dataclass, field

import numpy as np

from .protocol import ImuPacket, Status, decode_imu_packet, decode_status

MAGIC = b"PKRAW1\n"
REC_IMU, REC_STATUS, REC_MARKER, REC_EVENT = 1, 2, 3, 4

_META_LEN = struct.Struct("<I")
_REC_HEADER = struct.Struct("<BdH")  # type, phone_ms, payload len
_MAX_PAYLOAD = 0xFFFF

DEFAULT_ACC_RANGE_G = 16
DEFAULT_GYRO_RANGE_DPS = 2000
DEFAULT_ODR_HZ = 400
MIN_RATE_SPAN_S = 5.0


@dataclass
class RawRecord:
    type: int
    phone_ms: float
    payload: bytes


@dataclass
class RawLog:
    meta: dict
    records: list[RawRecord]


@dataclass
class Run:
    sample_index: np.ndarray  # int64 (N,)
    raw: np.ndarray  # int16 (N, 6)
    gaps: list[tuple[int, int]]  # (last index before gap, first index after)
    statuses: list[Status]
    rate_hz: float  # estimated true sample rate
    acc_range_g: int
    gyro_range_dps: int


@dataclass
class Recording:
    runs: list[Run]
    markers: list[tuple[float, str]]  # (phone_ms, text)
    events: list[tuple[float, dict]]
    meta: dict


def read_rawlog(path_or_bytes: str | os.PathLike | bytes) -> RawLog:
    if isinstance(path_or_bytes, (bytes, bytearray, memoryview)):
        data = bytes(path_or_bytes)
    else:
        with open(path_or_bytes, "rb") as f:
            data = f.read()
    if data[:len(MAGIC)] != MAGIC:
        raise ValueError("bad magic: not a .pkraw file")
    pos = len(MAGIC)
    if len(data) < pos + _META_LEN.size:
        raise ValueError("truncated .pkraw: missing meta length")
    (meta_len,) = _META_LEN.unpack_from(data, pos)
    pos += _META_LEN.size
    if len(data) < pos + meta_len:
        raise ValueError("truncated .pkraw: meta cut short")
    try:
        meta = json.loads(data[pos:pos + meta_len].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise ValueError(f"invalid .pkraw meta: {e}") from None
    pos += meta_len

    records: list[RawRecord] = []
    while pos < len(data):
        if len(data) < pos + _REC_HEADER.size:
            raise ValueError("truncated .pkraw: record header cut short")
        rtype, phone_ms, n = _REC_HEADER.unpack_from(data, pos)
        pos += _REC_HEADER.size
        if len(data) < pos + n:
            raise ValueError("truncated .pkraw: record payload cut short")
        records.append(RawRecord(rtype, phone_ms, data[pos:pos + n]))
        pos += n
    return RawLog(meta, records)


def write_rawlog(path: str | os.PathLike, meta: dict, records: list[RawRecord]) -> None:
    meta_bytes = json.dumps(meta, separators=(",", ":")).encode("utf-8")
    parts = [MAGIC, _META_LEN.pack(len(meta_bytes)), meta_bytes]
    for r in records:
        if len(r.payload) > _MAX_PAYLOAD:
            raise ValueError(f"record payload too long: {len(r.payload)} bytes")
        parts.append(_REC_HEADER.pack(r.type, r.phone_ms, len(r.payload)))
        parts.append(r.payload)
    with open(path, "wb") as f:
        f.write(b"".join(parts))


def estimate_rate(statuses: list[Status], nominal_hz: float) -> float:
    """Least-squares slope of sample_index vs time_us/1e6 (samples per second)."""
    if len(statuses) < 2:
        return nominal_hz
    t = np.array([s.time_us for s in statuses], dtype=np.float64) / 1e6
    idx = np.array([s.sample_index for s in statuses], dtype=np.float64)
    if t.max() - t.min() < MIN_RATE_SPAN_S:
        return nominal_hz
    slope = np.polyfit(t, idx, 1)[0]
    return float(slope)


@dataclass
class _RunBuilder:
    acc_range_g: int
    gyro_range_dps: int
    nominal_hz: float
    packets: list[ImuPacket] = field(default_factory=list)
    gaps: list[tuple[int, int]] = field(default_factory=list)
    statuses: list[Status] = field(default_factory=list)
    next_index: int = 0

    def add(self, p: ImuPacket) -> None:
        n = p.samples.shape[0]
        if self.packets and p.first_index != self.next_index:
            self.gaps.append((self.next_index - 1, p.first_index))
        self.packets.append(p)
        self.next_index = p.first_index + n

    def accepts(self, st: Status) -> bool:
        """Whether a status plausibly belongs to this run.

        Status notifies at 1 Hz even when idle, and after a device reset it may arrive
        before the new run's first packet. Require the streaming flag, a sample_index inside
        the run's received range (inclusive of the next expected index) and monotonicity.
        """
        if not st.flags & 1:
            return False
        if not self.packets[0].first_index <= st.sample_index <= self.next_index:
            return False
        return not self.statuses or st.sample_index >= self.statuses[-1].sample_index

    def build(self) -> Run:
        raw = np.concatenate([p.samples for p in self.packets]).astype(np.int16)
        sample_index = np.concatenate([
            p.first_index + np.arange(p.samples.shape[0], dtype=np.int64) for p in self.packets
        ])
        return Run(sample_index, raw, self.gaps, self.statuses,
                   estimate_rate(self.statuses, self.nominal_hz),
                   self.acc_range_g, self.gyro_range_dps)


def to_recording(log: RawLog) -> Recording:
    """Assemble records into runs (split where first_index goes backwards), markers and events.

    Gaps are detected from first_index continuity, so the u16 `seq` (which wraps) is not needed.
    A run's ranges come from the latest status seen before its first packet. Statuses seen
    after its first packet are attached to it only if they plausibly belong to it (see
    _RunBuilder.accepts), so idle or post-reset statuses do not corrupt rate_hz.
    """
    builders: list[_RunBuilder] = []
    markers: list[tuple[float, str]] = []
    events: list[tuple[float, dict]] = []
    latest: Status | None = None
    current: _RunBuilder | None = None
    prev_first = -1

    for rec in log.records:
        if rec.type == REC_IMU:
            p = decode_imu_packet(rec.payload)
            if current is None or p.first_index < prev_first:
                current = _RunBuilder(
                    latest.acc_range_g if latest else DEFAULT_ACC_RANGE_G,
                    latest.gyro_range_dps if latest else DEFAULT_GYRO_RANGE_DPS,
                    float(latest.odr_hz) if latest else float(DEFAULT_ODR_HZ),
                )
                builders.append(current)
            current.add(p)
            prev_first = p.first_index
        elif rec.type == REC_STATUS:
            latest = decode_status(rec.payload)
            if current is not None and current.accepts(latest):
                current.statuses.append(latest)
        elif rec.type == REC_MARKER:
            markers.append((rec.phone_ms, rec.payload.decode("utf-8")))
        elif rec.type == REC_EVENT:
            events.append((rec.phone_ms, json.loads(rec.payload.decode("utf-8"))))
        # unknown record types are ignored (forward compatibility)

    return Recording([b.build() for b in builders], markers, events, log.meta)
