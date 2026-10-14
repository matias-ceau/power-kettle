"""Scale a raw `Run` (int16 counts) to SI `ImuData` on a uniform sample grid.

Stdlib + numpy only (runs in Pyodide).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .rawlog import Run

G = 9.80665  # m/s^2
GAP_GUARD_SAMPLES = 2  # samples marked invalid on each side of a gap
_FULL_SCALE = 32768.0
_INT16_MIN, _INT16_MAX = -32768, 32767


@dataclass
class ImuData:
    t: np.ndarray  # (N,) s, (index - first_index) / fs
    acc: np.ndarray  # (N, 3) m/s^2 (specific force); NaN where a sample is missing
    gyro: np.ndarray  # (N, 3) rad/s; NaN where a sample is missing
    valid: np.ndarray  # (N,) bool: sample present and not within the guard of a gap
    saturated: np.ndarray  # (N,) bool: any axis (accelerometer or gyro) at an int16 limit
    fs: float  # sample rate, Hz
    first_index: int  # sensor sample_index of row 0


def scale_run(run: Run) -> ImuData:
    first = int(run.sample_index[0])
    n = int(run.sample_index[-1]) - first + 1
    rows = (run.sample_index - first).astype(np.intp)

    acc = np.full((n, 3), np.nan)
    gyro = np.full((n, 3), np.nan)
    valid = np.zeros(n, dtype=bool)
    saturated = np.zeros(n, dtype=bool)

    raw = run.raw.astype(np.float64)
    acc[rows] = raw[:, :3] * (run.acc_range_g * G / _FULL_SCALE)
    gyro[rows] = raw[:, 3:] * (run.gyro_range_dps / _FULL_SCALE * math.pi / 180.0)
    valid[rows] = True
    saturated[rows] = np.any((run.raw == _INT16_MIN) | (run.raw == _INT16_MAX), axis=1)

    for last_before, first_after in run.gaps:
        lo = max(0, last_before - first - (GAP_GUARD_SAMPLES - 1))
        hi = min(n, first_after - first + GAP_GUARD_SAMPLES)
        valid[lo:hi] = False

    t = np.arange(n, dtype=np.float64) / run.rate_hz
    return ImuData(t, acc, gyro, valid, saturated, float(run.rate_hz), first)
