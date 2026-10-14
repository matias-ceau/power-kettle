"""Quaternion utilities. Scalar-first [w, x, y, z], vectorised over leading axes.

``rotate(q, v)`` maps body -> world (v' = q v q*). ``integrate`` applies body-frame
angular rates by right-multiplication: q_next = q (x) exp(omega * dt / 2).
"""
from __future__ import annotations

import numpy as np


def _q(a) -> np.ndarray:
    return np.asarray(a, dtype=float)


def mul(a, b) -> np.ndarray:
    """Hamilton product a (x) b."""
    a, b = _q(a), _q(b)
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def conj(q) -> np.ndarray:
    q = _q(q)
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def normalize(q) -> np.ndarray:
    q = _q(q)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def rotate(q, v) -> np.ndarray:
    """Rotate vector(s) v by quaternion(s) q (body -> world)."""
    q, v = _q(q), _q(v)
    qv = q[..., 1:]
    t = 2.0 * np.cross(qv, v)
    return v + q[..., :1] * t + np.cross(qv, t)


def from_axis_angle(axis, angle) -> np.ndarray:
    axis = _q(axis)
    angle = _q(angle)
    axis = axis / np.linalg.norm(axis, axis=-1, keepdims=True)
    half = 0.5 * angle
    return np.concatenate(
        [np.cos(half)[..., None], np.sin(half)[..., None] * axis], axis=-1
    )


def from_two_vectors(a, b) -> np.ndarray:
    """Shortest-arc quaternion rotating direction a onto direction b."""
    a, b = _q(a), _q(b)
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    a, b = np.broadcast_arrays(a, b)
    d = np.sum(a * b, axis=-1)
    c = np.cross(a, b)
    q = np.concatenate([(1.0 + d)[..., None], c], axis=-1)
    # Antiparallel: rotate pi about any axis perpendicular to a.
    anti = d < -1.0 + 1e-12
    if np.any(anti):
        helper = np.where(
            (np.abs(a[..., 0]) < 0.9)[..., None],
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
        )
        perp = np.cross(a, helper)
        perp = perp / np.linalg.norm(perp, axis=-1, keepdims=True)
        flip = np.concatenate([np.zeros(d.shape + (1,)), perp], axis=-1)
        q = np.where(anti[..., None], flip, q)
    return normalize(q)


def to_matrix(q) -> np.ndarray:
    """Rotation matrix (body -> world), shape (..., 3, 3)."""
    q = normalize(q)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    m = np.stack(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
        ],
        axis=-1,
    )
    return m.reshape(q.shape[:-1] + (3, 3))


def angle_between(a, b):
    """Angle in rad between 3-vectors a and b (numerically stable atan2 form)."""
    a, b = _q(a), _q(b)
    return np.arctan2(np.linalg.norm(np.cross(a, b), axis=-1), np.sum(a * b, axis=-1))


def integrate(q, omega_body, dt) -> np.ndarray:
    """One exact exponential-map step with body-frame rates (rad/s) over dt (s)."""
    q, omega = _q(q), _q(omega_body)
    theta_vec = omega * _q(dt)[..., None] if np.ndim(dt) else omega * float(dt)
    theta = np.linalg.norm(theta_vec, axis=-1, keepdims=True)
    half = 0.5 * theta
    small = theta < 1e-12
    safe = np.where(small, 1.0, theta)
    k = np.where(small, 0.5, np.sin(half) / safe)
    dq = np.concatenate([np.cos(half), k * theta_vec], axis=-1)
    return normalize(mul(q, dq))
