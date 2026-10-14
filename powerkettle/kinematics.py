"""Centre-of-mass acceleration from IMU data, orientation and the COM -> sensor lever arm.

The accelerometer sits off the bell's centre of mass, so it measures the acceleration of the
sensor point. With R = world <- sensor, f the specific force and g_w = (0, 0, -g)::

    a_s   = R f + g_w                                   (sensor-point acceleration, world)
    a_com = a_s - alpha x r_w - omega x (omega x r_w)   (r_w = R lever_arm_s, COM -> sensor)

Samples that are invalid or hold NaN propagate NaN (nothing is invented); because the angular
acceleration is a Savitzky-Golay derivative, NaN also spreads over the derivative window around
such a sample (+-window//2 samples). Stdlib + numpy only.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import quat
from .calibration import Calibration
from .imu import G, ImuData

G_W = np.array([0.0, 0.0, -G])


@dataclass
class Kinematics:
    a_com: np.ndarray  # (N, 3) m/s^2, world, COM acceleration (not specific force)
    omega_w: np.ndarray  # (N, 3) rad/s, world, bias-corrected angular velocity
    alpha_w: np.ndarray  # (N, 3) rad/s^2, world, angular acceleration
    f_norm: np.ndarray  # (N,) m/s^2, specific-force magnitude |f| in the sensor frame


def _derivative_coeffs(half: int, order: int) -> np.ndarray:
    """Weights w (2*half+1,) with d/dm of the least-squares polynomial fit at m = 0 equal to
    sum_k w[k] x[k] (unit sample spacing); the fit order is capped at 2*half."""
    m = np.arange(-half, half + 1, dtype=float)
    p = min(order, 2 * half)
    if p < 1:
        return np.zeros(2 * half + 1)
    return np.linalg.pinv(np.vander(m, p + 1, increasing=True))[1]


def savgol_derivative(x: np.ndarray, dt: float, window: int = 11, order: int = 3) -> np.ndarray:
    """First derivative along axis 0 by a Savitzky-Golay local least-squares polynomial fit
    (`window` odd samples, polynomial `order`), in units of x per second.

    Rows within window//2 of either end use a symmetric window shrunk to fit (a row `i` from the
    edge uses half-width `i`; the fit order is capped at the window size - 1, so the outermost
    row, a one-sample window, has derivative 0). NaN in a row's window gives NaN for that row.
    Inputs shorter than 3 rows return zeros (NaN where the input is NaN)."""
    x = np.asarray(x, dtype=float)
    if window % 2 == 0 or window < 1:
        raise ValueError("window must be a positive odd integer")
    n = len(x)
    out = np.empty_like(x)
    half = window // 2
    for i in range(min(half, n)):
        for row in {i, n - 1 - i}:
            h = min(row, n - 1 - row, half)
            c = _derivative_coeffs(h, order)
            out[row] = np.tensordot(c, x[row - h: row + h + 1], axes=(0, 0))
    if n > 2 * half:
        c = _derivative_coeffs(half, order)
        view = np.lib.stride_tricks.sliding_window_view(x, window, axis=0)  # (n-w+1, ..., w)
        out[half: n - half] = view @ c
    return out / dt


def com_acceleration(imu: ImuData, quats: np.ndarray, calib: Calibration,
                     gyro_bias: np.ndarray | None = None) -> Kinematics:
    """Lever-arm-corrected COM acceleration in the world frame.

    `quats` (N, 4) is world <- sensor (`estimate_orientation`). `gyro_bias` is the sensor-frame
    bias subtracted from the gyro: (3,) or (N, 3); pass the *working* bias returned by
    `estimate_orientation(..., return_bias=True)`. Default: `calib.gyro_bias`. Samples with
    `imu.valid` false get NaN omega and hence NaN alpha / a_com in the derivative window."""
    n = len(imu.t)
    quats = np.asarray(quats, dtype=float)
    if quats.shape != (n, 4):
        raise ValueError(f"quats must have shape ({n}, 4), got {quats.shape}")
    bias = calib.gyro_bias if gyro_bias is None else gyro_bias
    bias = np.broadcast_to(np.asarray(bias, dtype=float), (n, 3))

    bad = ~imu.valid
    acc = np.where(bad[:, None], np.nan, imu.acc)
    omega_s = np.where(bad[:, None], np.nan, imu.gyro - bias)

    r_mat = quat.to_matrix(quats)  # (N, 3, 3), world <- sensor
    a_s = np.einsum("nij,nj->ni", r_mat, acc) + G_W
    omega_w = np.einsum("nij,nj->ni", r_mat, omega_s)
    alpha_w = savgol_derivative(omega_w, 1.0 / imu.fs)
    r_w = np.einsum("nij,j->ni", r_mat, np.asarray(calib.lever_arm_s, dtype=float))

    a_com = a_s - np.cross(alpha_w, r_w) - np.cross(omega_w, np.cross(omega_w, r_w))
    return Kinematics(a_com, omega_w, alpha_w, np.linalg.norm(imu.acc, axis=1))
