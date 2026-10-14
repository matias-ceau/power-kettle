"""Drift-free velocity, relative position and power of the bell's centre of mass (spec §5.6).

Velocity
--------
Anchors are zero-velocity points (low, top_swing) or windows (rest, top_lockout); v ≡ 0 on
them, except at the edges of a rest (below). Between two consecutive anchors (a segment, from
the zero-velocity end of one to that of the next) a_com is integrated with the trapezoid rule
from v = 0, then the linear ramp in time that brings the end value back to zero is subtracted::

    v(t) = ∫ a dt − (t − t0)/(t1 − t0) · ∫_{t0}^{t1} a dt

so any constant acceleration error (a small tilt error leaking gravity, accelerometer bias) is
removed exactly. Any NaN in a_com over a segment (invalid samples propagate NaN through
``com_acceleration``) makes the whole segment NaN, including the rest edges it would have
integrated (rest cores and lockout windows stay 0): no velocity is guessed across a gap.
Samples before the first anchor and after the last have no zero-velocity end: NaN.

Rest edges: ``static_mask`` accepts |a| up to 0.05 g, so a rest window detected by
``detect_anchors`` also covers the first and last ~0.1 s of a slow lift or set-down (a smooth
start from rest accelerates gently). Forcing v = 0 there leaves the onset speed (~0.025 m/s for a
2 s table lift) as a constant offset over the whole lift (−4.5 cm of height over 0.74 m). So the
zero-velocity point of a rest is its *core* (``rest_core``): the window less `REST_EDGE_S` at each
end that borders a segment; the edges are integrated with the segment (|v| stays at noise level
there when the bell is truly still). A rest edge that borders no segment (before the first or
after the last anchor) stays at 0.

Position
--------
The trapezoid integral of v, restarting at 0 at every low anchor and at the end of the core of
every rest window. Before that restart, inside the rest it stays at the height reached on
entering the rest (v ≈ 0), i.e. the height of the rest above the preceding low (or rest). It is a
relative position, for apex heights and per-rep paths, not a trajectory over the session. NaN
velocity propagates until the next restart; there is no origin (NaN) before the first restart.

Power
-----
P = m · (a_com + g ẑ) · v, the power the lifter delivers to the bell, lifting against gravity
included. Rep power = concentric work / concentric duration (``concentric_power``).

Pipeline: ``detect_anchors`` cannot tell a quiet (tremor-free) overhead hold from a floor rest;
callers run ``segment_velocity`` and ``segment_position``, then ``anchors.refine_rests`` (which
needs the height), then ``segment_velocity`` / ``segment_position`` once more on the refined
anchors before ``power``. Stdlib + numpy only.
"""
from __future__ import annotations

import numpy as np

from .anchors import Anchor
from .imu import G

REST_EDGE_S = 0.2  # integrated (not forced to zero) at each segment-facing end of a rest window
_UP = np.array([0.0, 0.0, 1.0])


def _cumtrapz(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Cumulative trapezoid integral along axis 0, starting at 0 (same length as y)."""
    dt = np.diff(t)[:, None] if y.ndim == 2 else np.diff(t)
    steps = 0.5 * (y[1:] + y[:-1]) * dt
    return np.concatenate([np.zeros_like(y[:1]), np.cumsum(steps, axis=0)])


def rest_core(a: Anchor, fs: float) -> tuple[int, int]:
    """(c0, c1) inclusive: the zero-velocity part of an anchor. For a rest, the window less
    `REST_EDGE_S` at each end, never past its middle sample; the whole window otherwise."""
    if a.kind != "rest":
        return a.i0, a.i1
    m, mid = round(REST_EDGE_S * fs), (a.i0 + a.i1) // 2
    return min(a.i0 + m, mid), max(a.i1 - m, mid)


def _fs(t: np.ndarray) -> float:
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 1.0


def segment_velocity(a_com: np.ndarray, t: np.ndarray, anchors: list[Anchor]) -> np.ndarray:
    """(N, 3) COM velocity, m/s: zero on every anchor (points and windows; rest edges facing a
    segment are integrated, see the module docstring), integrated and linearly detrended
    between consecutive anchors; NaN over a segment (its rest edges included) that holds any NaN
    acceleration, and before the first / after the last anchor."""
    a_com = np.asarray(a_com, dtype=float)
    t = np.asarray(t, dtype=float)
    fs = _fs(t)
    v = np.full(a_com.shape, np.nan)
    anchors = sorted(anchors)
    for a in anchors:
        v[a.i0:a.i1 + 1] = 0.0
    for a, b in zip(anchors[:-1], anchors[1:]):
        s, e = rest_core(a, fs)[1], rest_core(b, fs)[0]
        if e - s < 2:
            continue  # adjacent zero-velocity samples: nothing between them
        seg_a, seg_t = a_com[s:e + 1], t[s:e + 1]
        if not np.all(np.isfinite(seg_a)):
            v[s + 1:e] = np.nan  # its rest edges too; rest cores and lockouts stay at 0
            continue
        raw = _cumtrapz(seg_a, seg_t)
        ramp = (seg_t - seg_t[0]) / (seg_t[-1] - seg_t[0])
        v[s + 1:e] = (raw - ramp[:, None] * raw[-1])[1:-1]
    return v


def segment_position(v: np.ndarray, t: np.ndarray, anchors: list[Anchor]) -> np.ndarray:
    """(N, 3) relative COM position, m: trapezoid integral of `v`, restarting at 0 at every low
    anchor and at the end of every rest core (``rest_core``). NaN before the first restart and
    from a NaN velocity to the next restart."""
    v = np.asarray(v, dtype=float)
    t = np.asarray(t, dtype=float)
    fs = _fs(t)
    n = len(v)
    restarts = sorted({a.i0 for a in anchors if a.kind == "low"}
                      | {rest_core(a, fs)[1] for a in anchors if a.kind == "rest"})
    pos = np.full(v.shape, np.nan)
    for k, s in enumerate(restarts):
        e = restarts[k + 1] if k + 1 < len(restarts) else n - 1
        pos[s:e + 1] = _cumtrapz(v[s:e + 1], t[s:e + 1])
    return pos


def power(a_com: np.ndarray, v: np.ndarray, mass_kg: float) -> np.ndarray:
    """(N,) power delivered to the bell, W: m · (a_com + g ẑ) · v."""
    return mass_kg * np.sum((np.asarray(a_com, float) + G * _UP) * np.asarray(v, float), axis=1)


def concentric_power(P: np.ndarray, t: np.ndarray, i_low: int, i_top: int) -> float:
    """Mean power over [i_low, i_top] (inclusive), W: trapezoid work / duration (the
    simulator's truth definition of rep power). Requires i_top > i_low; a degenerate window
    (i_top <= i_low, zero duration) gives NaN."""
    if i_top <= i_low:
        return float("nan")
    p, tt = np.asarray(P, float)[i_low:i_top + 1], np.asarray(t, float)[i_low:i_top + 1]
    work = np.sum(0.5 * (p[1:] + p[:-1]) * np.diff(tt))
    return float(work / (tt[-1] - tt[0]))
