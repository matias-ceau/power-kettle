"""Session calibration from a still window (bell standing on the floor).

From the first still window of a recording: gyro bias, bell-up and outward axes in the sensor
frame, the COM -> sensor lever arm in the sensor frame, and the initial orientation
(world <- sensor, world z up, athlete forward = world +x).

The magnitude of the mean specific force at rest, ``g_measured``, should be g; beyond
``g_tolerance`` a warning is issued, and the pipeline divides it out (``correct_acc_scale``)
before every later stage, so an accelerometer scale error does not bias power and the
static tests (|f| within 5 % of g) keep working. Stdlib + numpy only.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np

from . import quat
from .bell import BellProfile, lever_arm_bell_m
from .imu import G, ImuData

SMOOTH_SECONDS = 0.1  # averaging length of the per-sample stillness test (suppresses noise)
G_ERROR_MAX = 0.10  # fraction of g; beyond this |f| at rest is a scale/range error, not a warning
MIN_OUT_PERP = 0.1  # |out - (out.up) up| below this: outward axis is (anti)parallel to up
SEED_SECONDS = 0.5  # q0 of a run without a still window: gravity averaged over this much


class CalibrationError(ValueError):
    pass


@dataclass
class CalibrationConfig:
    still_seconds: float = 2.0
    gyro_still_max: float = 0.05  # rad/s
    acc_still_tol: float = 0.03  # fraction of g
    g_tolerance: float = 0.02  # fraction of g, beyond which a warning is issued
    normal_axis: int = 2  # sensor axis normal to the box base (= bell outward)
    normal_sign: int = +1


@dataclass
class Calibration:
    gyro_bias: np.ndarray  # (3,) rad/s, sensor frame
    up_s: np.ndarray  # (3,) unit, bell-up in the sensor frame
    out_s: np.ndarray  # (3,) unit, bell outward in the sensor frame
    lever_arm_s: np.ndarray  # (3,) m, COM -> sensor, sensor frame
    q0: np.ndarray  # (4,) world <- sensor; forward (world +x) = -horizontal(out)
    still: slice
    g_measured: float  # m/s^2
    warnings: list[str] = field(default_factory=list)


def _moving_mean(x: np.ndarray, k: int) -> np.ndarray:
    """Mean over every k consecutive rows; element j covers rows j .. j+k-1."""
    c = np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(x, axis=0)])
    return (c[k:] - c[:-k]) / k


def _sliding(x: np.ndarray, w: int, op) -> np.ndarray:
    """Sliding max/min (op = np.maximum / np.minimum) over every w consecutive samples of a 1-D
    array (van Herk / Gil-Werman): element j covers x[j .. j+w-1]."""
    n = len(x)
    pad = (-n) % w
    fill = x[-1]
    blocks = np.concatenate([x, np.full(pad, fill)]).reshape(-1, w)
    prefix = op.accumulate(blocks, axis=1).ravel()
    suffix = op.accumulate(blocks[:, ::-1], axis=1)[:, ::-1].ravel()
    return op(suffix[: n - w + 1], prefix[w - 1: n])


def find_still_window(imu: ImuData, cfg: CalibrationConfig = CalibrationConfig()) -> slice:
    """First run of `still_seconds` where every sample is valid, the (0.1 s averaged) |gyro| stays
    below `gyro_still_max` and the averaged |acc| stays within `acc_still_tol` of the window's own
    mean |acc| (stability, not closeness to g: a mis-scaled accelerometer is reported by
    `calibrate`). Returns a slice of exactly `still_seconds`."""
    n_need = int(math.ceil(cfg.still_seconds * imu.fs - 1e-9))
    n = len(imu.t)
    k = max(1, int(round(SMOOTH_SECONDS * imu.fs)))
    if n >= n_need and n_need >= k:
        ok_sample = imu.valid & np.all(np.isfinite(imu.acc), 1) & np.all(np.isfinite(imu.gyro), 1)
        all_valid = _moving_mean(ok_sample.astype(float), k) > 1.0 - 1e-9
        acc = _moving_mean(np.nan_to_num(imu.acc), k)
        gyro = _moving_mean(np.nan_to_num(imu.gyro), k)
        mag = np.linalg.norm(acc, axis=1)
        calm = all_valid & (np.linalg.norm(gyro, axis=1) < cfg.gyro_still_max)
        w = n_need - k + 1  # consecutive smoothed flags that cover n_need samples
        if len(mag) >= w:
            run_calm = _moving_mean(calm.astype(float), w) > 1.0 - 1e-9
            mean = _moving_mean(mag, w)
            hi, lo = _sliding(mag, w, np.maximum), _sliding(mag, w, np.minimum)
            tol = cfg.acc_still_tol * mean
            good = np.flatnonzero(run_calm & (hi - mean < tol) & (mean - lo < tol))
            if len(good):
                s = int(good[0])
                return slice(s, s + n_need)
    raise CalibrationError(
        f"no still window of {cfg.still_seconds:.1f} s found (|gyro| < {cfg.gyro_still_max} rad/s, "
        f"|acc| steady within {100 * cfg.acc_still_tol:.0f} %): start the recording with the bell "
        "standing still on the floor")


def with_mounting(calib: Calibration, prior: Calibration) -> Calibration:
    """`calib` (from its own still window) with the mounting of `prior` (a previous run of the
    session: up_s, out_s, lever_arm_s, which do not change while the box stays on the bell);
    q0 is rebuilt from the window's gravity direction (``calib.up_s``) and prior's out_s."""
    return dataclasses.replace(
        calib, up_s=np.array(prior.up_s, float), out_s=np.array(prior.out_s, float),
        lever_arm_s=np.array(prior.lever_arm_s, float),
        q0=_initial_orientation(calib.up_s, np.asarray(prior.out_s, float)))


def seeded_from_prior(imu: ImuData, prior: Calibration, static: np.ndarray) -> Calibration:
    """A calibration for a run without a still window, from a previous run's `prior`: its
    mounting, gyro bias and |g|; q0 levelled on the mean specific-force direction over the first
    run of `static` samples (at most SEED_SECONDS of it; heading from prior's out_s) and
    ``still`` = that run's first sample, or, without static samples, prior's q0 at sample 0."""
    starts, stops = _true_runs(static)
    if len(starts) == 0:
        return dataclasses.replace(prior, still=slice(0, 1), warnings=[])
    a = int(starts[0])
    b = min(int(stops[0]), a + max(1, int(round(SEED_SECONDS * imu.fs))))
    f = np.nanmean(imu.acc[a:b], axis=0)
    q0 = _initial_orientation(f / np.linalg.norm(f), np.asarray(prior.out_s, float))
    return dataclasses.replace(prior, q0=q0, still=slice(a, a + 1), warnings=[])


def _true_runs(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    edges = np.diff(np.concatenate([[0], np.asarray(mask, np.int8), [0]]))
    return np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)


def correct_acc_scale(imu: ImuData, calib: Calibration) -> ImuData:
    """`imu` with the accelerometer scaled by g / ``calib.g_measured`` (|f| at rest becomes g);
    the input is not modified."""
    return dataclasses.replace(imu, acc=imu.acc * (G / calib.g_measured))


def _initial_orientation(up_s: np.ndarray, out_s: np.ndarray) -> np.ndarray:
    """World <- sensor with up_s -> +z and out_s -> -x (forward = -horizontal(out))."""
    q_up = quat.from_two_vectors(up_s, [0.0, 0.0, 1.0])
    h = quat.rotate(q_up, out_s)
    yaw = math.pi - math.atan2(h[1], h[0])
    return quat.mul(quat.from_axis_angle([0.0, 0.0, 1.0], yaw), q_up)


def calibrate(imu: ImuData, bell: BellProfile, cfg: CalibrationConfig = CalibrationConfig(),
              window: slice | None = None) -> Calibration:
    win = find_still_window(imu, cfg) if window is None else window
    ok = imu.valid[win] & np.all(np.isfinite(imu.acc[win]), 1) & np.all(np.isfinite(imu.gyro[win]), 1)
    if not np.any(ok):
        raise CalibrationError("calibration window holds no valid samples")
    f_mean = imu.acc[win][ok].mean(axis=0)
    gyro_bias = imu.gyro[win][ok].mean(axis=0)

    g_measured = float(np.linalg.norm(f_mean))
    off = abs(g_measured - G) / G
    if off > G_ERROR_MAX:
        raise CalibrationError(
            f"bell is still but |f| is {100 * off:.1f} % off g - check accelerometer range/scale")
    up_s = f_mean / g_measured  # at floor rest specific force points along bell-up

    normal = np.zeros(3)
    normal[cfg.normal_axis] = float(cfg.normal_sign)
    out_s = normal - (normal @ up_s) * up_s
    perp = float(np.linalg.norm(out_s))
    if perp < MIN_OUT_PERP:
        raise CalibrationError(
            "box normal axis is parallel to gravity at rest: the bell is not standing upright "
            "or normal_axis is wrong")
    out_s = out_s / perp

    lateral = np.cross(up_s, out_s)  # bell y = z x x
    r_sb = np.stack([out_s, lateral, up_s], axis=1)  # columns: bell axes in sensor coordinates
    lever_arm_s = r_sb @ lever_arm_bell_m(bell)

    warnings: list[str] = []
    if off > cfg.g_tolerance:
        warnings.append(f"accelerometer magnitude off by {100 * off:.1f} %")

    return Calibration(gyro_bias, up_s, out_s, lever_arm_s, _initial_orientation(up_s, out_s),
                       win, g_measured, warnings)
