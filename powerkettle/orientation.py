"""Orientation (world <- sensor) from gyro integration with a static gravity correction.

The bias-corrected gyro is integrated exactly (exponential map, midpoint rule: each step uses
the mean of its two end-point rates). Only while the bell is static does the measured
specific-force direction pull the estimate's tilt towards gravity; the correction is a rotation
about an axis perpendicular to gravity, so heading (yaw), which gravity cannot observe, is never
changed. Stdlib + numpy only.
"""
from __future__ import annotations

import math

import numpy as np

from . import quat
from .calibration import Calibration
from .imu import G, ImuData

STATIC_HOLD_SECONDS = 0.1  # a static run must last at least this long
_EPS = 1e-12


def _runs(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Start (inclusive) and stop (exclusive) indices of the True runs of `mask`."""
    edges = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)


def static_mask(imu: ImuData, gyro_max: float = 0.3, acc_tol: float = 0.05) -> np.ndarray:
    """Samples where the bell is static: |omega| < gyro_max (rad/s) and | |f| - g | < acc_tol * g,
    in a run of valid samples that lasts at least 0.1 s."""
    with np.errstate(invalid="ignore"):
        calm = (np.linalg.norm(imu.gyro, axis=1) < gyro_max)
        calm &= np.abs(np.linalg.norm(imu.acc, axis=1) - G) < acc_tol * G
    calm &= imu.valid  # NaN compares False, so missing samples are excluded already
    out = np.zeros(len(calm), dtype=bool)
    need = max(1, int(math.ceil(STATIC_HOLD_SECONDS * imu.fs - 1e-9)))
    for a, b in zip(*_runs(calm)):
        if b - a >= need:
            out[a:b] = True
    return out


def _cumulative_product(dq: np.ndarray) -> np.ndarray:
    """Left-to-right running Hamilton product of the rows of dq (Hillis-Steele scan)."""
    p = dq.copy()
    d = 1
    while d < len(p):
        p[d:] = quat.mul(p[:-d], p[d:])
        d *= 2
    return p


def _delta_quaternions(omega: np.ndarray, step_ok: np.ndarray, dt: float) -> np.ndarray:
    """(N-1, 4) rotation of each step, k -> k+1, from the mean of the two end-point rates;
    the identity where either end is missing."""
    w = 0.5 * (omega[:-1] + omega[1:])
    w = np.where(step_ok[:, None], w, 0.0)
    theta = np.linalg.norm(w, axis=1) * dt
    half = 0.5 * theta
    k = np.where(theta < 1e-12, 0.5 * dt, np.sin(half) / np.where(theta < 1e-12, 1.0, theta) * dt)
    return np.concatenate([np.cos(half)[:, None], k[:, None] * w], axis=1)


def _static_stretch(q_prev, dq, f_hat, rate):
    """Sequentially integrate and level one static stretch; returns its (m, 4) orientations.

    After each gyro step the predicted gravity direction in the sensor frame, u = R(q)^T z, is
    rotated a fraction `rate` of the way onto the measured direction f_hat, by
    q <- q (x) conj(delta), delta being the shortest arc u -> f_hat scaled by `rate`.
    The axis u x f_hat is perpendicular to u, so in the world frame the update is horizontal.
    """
    w, x, y, z = q_prev
    out = []
    atan2, sqrt, sin, cos = math.atan2, math.sqrt, math.sin, math.cos
    for (dw, dx, dy, dz), (fx, fy, fz) in zip(dq, f_hat):
        w, x, y, z = (w * dw - x * dx - y * dy - z * dz,
                      w * dx + x * dw + y * dz - z * dy,
                      w * dy - x * dz + y * dw + z * dx,
                      w * dz + x * dy - y * dx + z * dw)
        ux, uy, uz = 2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)
        ax, ay, az = uy * fz - uz * fy, uz * fx - ux * fz, ux * fy - uy * fx
        s = sqrt(ax * ax + ay * ay + az * az)
        if s > _EPS:
            half = 0.5 * rate * atan2(s, ux * fx + uy * fy + uz * fz)
            c, k = cos(half), -sin(half) / s
            ex, ey, ez = k * ax, k * ay, k * az
            w, x, y, z = (w * c - x * ex - y * ey - z * ez,
                          w * ex + x * c + y * ez - z * ey,
                          w * ey - x * ez + y * c + z * ex,
                          w * ez + x * ey - y * ex + z * c)
        n = 1.0 / sqrt(w * w + x * x + y * y + z * z)
        w, x, y, z = w * n, x * n, y * n, z * n
        out.append((w, x, y, z))
    return np.array(out)


def working_bias(imu: ImuData, bias0: np.ndarray, static: np.ndarray, start: int = 0,
                 min_seconds: float = 1.0, max_std: float = 0.01,
                 max_jump: float = 0.03) -> np.ndarray:
    """(N, 3) gyro bias used for integration: `bias0`, replaced from the start of every accepted
    static run by the mean raw gyro over that run (valid, finite samples only), until the next
    accepted run. A static run is accepted if it lasts >= `min_seconds`, the per-axis std of the
    raw gyro over it is < `max_std` (rad/s) and its mean lies within `max_jump` (rad/s, vector
    norm) of the bias in force; otherwise a slow genuine rotation would be absorbed as bias.
    Runs ending at or before sample `start` are ignored. Offline: the mean is applied from the
    start of the run, so it uses samples later than the ones it corrects."""
    n = len(imu.t)
    bias = np.tile(np.asarray(bias0, float), (n, 1))
    need = max(1, int(math.ceil(min_seconds * imu.fs - 1e-9)))
    good = imu.valid & np.all(np.isfinite(imu.gyro), axis=1)
    current = np.asarray(bias0, float).copy()
    seg_start = 0
    for a, b in zip(*_runs(static)):
        if b - a < need or b <= start:
            continue
        w = imu.gyro[a:b][good[a:b]]
        if len(w) == 0 or np.any(w.std(axis=0) >= max_std):
            continue
        mean = w.mean(axis=0)
        if np.linalg.norm(mean - current) >= max_jump:
            continue
        bias[seg_start:a] = current
        current, seg_start = mean, a
    bias[seg_start:] = current
    return bias


def estimate_orientation(imu: ImuData, calib: Calibration, gain: float = 2.0,
                         bias_update_min_s: float = 1.0, bias_max_std: float = 0.01,
                         bias_max_jump: float = 0.03, return_bias: bool = False):
    """(N, 4) world <- sensor orientation, scalar-first; with `return_bias` also the (N, 3)
    working gyro bias that was subtracted at each sample.

    The gyro bias starts at `calib.gyro_bias` and is re-estimated offline (non-causally): a
    static window lasting at least `bias_update_min_s` seconds, with per-axis raw-gyro std below
    `bias_max_std` and a mean within `bias_max_jump` (rad/s) of the bias in force, replaces it
    by that window mean, applied from the window's start (the true rate of a still bell is
    zero). This absorbs bias drift between sets; windows failing the std/jump checks (slow
    genuine rotation) are ignored.

    Starts from `calib.q0` at the start of the calibration window (earlier samples hold it) and
    integrates the bias-corrected gyro. Where `static_mask` is set, the tilt is corrected towards
    the measured gravity direction at rate `gain` (rad/s, per rad of tilt error; a step is capped
    at the full error). Steps touching an invalid or missing sample do not rotate, so invalid
    samples hold the last orientation."""
    n = len(imu.t)
    dt = 1.0 / imu.fs
    s0 = min(max(int(calib.still.start or 0), 0), n - 1)
    q = np.empty((n, 4))
    q[: s0 + 1] = quat.normalize(calib.q0)
    if n == 1 or s0 == n - 1:
        return (q, np.tile(np.asarray(calib.gyro_bias, float), (n, 1))) if return_bias else q

    static_all = static_mask(imu)
    bias = working_bias(imu, calib.gyro_bias, static_all, s0, bias_update_min_s,
                        bias_max_std, bias_max_jump)
    omega = imu.gyro - bias
    ok = imu.valid & np.all(np.isfinite(omega), axis=1)
    ok_s = ok[s0:]
    dq = _delta_quaternions(omega[s0:], ok_s[:-1] & ok_s[1:], dt)  # dq[j]: sample s0+j -> s0+j+1
    static = static_all[s0 + 1:]  # applies to the sample each step lands on
    with np.errstate(invalid="ignore", divide="ignore"):
        f_hat = imu.acc[s0 + 1:] / np.linalg.norm(imu.acc[s0 + 1:], axis=1, keepdims=True)
    rate = min(gain * dt, 1.0)

    starts, stops = _runs(static)
    bounds = sorted({0, len(static), *starts.tolist(), *stops.tolist()})
    for a, b in zip(bounds[:-1], bounds[1:]):
        prev = q[s0 + a]  # orientation of the sample before this stretch (step index a)
        if static[a]:
            q[s0 + a + 1: s0 + b + 1] = _static_stretch(
                prev, dq[a:b].tolist(), f_hat[a:b].tolist(), rate)
        else:
            q[s0 + a + 1: s0 + b + 1] = quat.normalize(
                quat.mul(prev, _cumulative_product(dq[a:b])))
    return (q, bias) if return_bias else q
