"""Zero-velocity anchors: backswing turnarounds (lows), swing tops, snatch lockouts, floor rests.

Velocity is integrated between consecutive anchors and detrended to zero at both ends (spec
§5.5), so a false or missed anchor corrupts the reps around it. Stdlib + numpy only.

Sagittal tilt θ
---------------
θ is the angle of the world bell-up vector R·up_s inside the vertical plane that contains the
athlete's forward direction, positive when bell-up leans forward (the simulator's θ)::

    θ = atan2(u · forward, u · ẑ)

Swing: θ ≈ +40° at the backswing, −90° at the top. Snatch: +40° at the backswing, −30° mid-pull
and on the drop, +20° at the lockout.

Forward
-------
At a floor rest the front flat faces the lifter, so forward = −horizontal(R·out_s), averaged over
the rest window (``rest_forward``). Each stretch between rests uses the forward of the rest
before it (the calibration window before the first rest). The axis is always re-read at the
rest (absorbing gyro heading drift), but its sign is kept continuous with the previous forward:
a forward that turns by more than 90° between rests is taken to be the bell parked turned round
(or a hold without tremor, where the hand has twisted the bell 180°, provisionally taken as a
rest), not the athlete turning round. A rest whose outward axis is close to vertical gives no
forward and the previous one is kept.

Sets in anchor terms
--------------------
A set is the run of anchors between two rest anchors. It opens with its first low (the lift
from the floor to the first backswing is not a rep) and closes with its last low (the descent
to the floor is not a rep): a top anchor is only placed between two consecutive lows with no
rest between them, so a rest is always next to a low or another rest, never a top.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from . import quat
from .calibration import Calibration
from .imu import G, ImuData
from .kinematics import Kinematics
from .orientation import _runs, static_mask

MIN_FORWARD_NORM = 0.5  # |mean horizontal(R·out)| below this: outward axis too close to vertical


@dataclass
class AnchorConfig:
    prominence_deg: float = 30.0  # a low must stand out of θ by this much (hysteresis)
    low_min_f_g: float = 1.3  # |f| at a low, in g (the arm pulls the bell out of the backswing)
    static_min_s: float = 0.15  # shortest static window considered (lockout / rest)
    rest_min_s: float = 2.0
    tremor_band: tuple = (4.0, 12.0)  # Hz, physiological tremor
    tremor_rms_max: float = 0.02  # m/s², band RMS of |f| above which a static window is hand-held
    floor_max_height_m: float = 0.8  # used by Task 8 (refine_rests)


@dataclass(order=True)
class Anchor:
    i0: int
    i1: int  # inclusive; i0 == i1 for point anchors (low, top_swing)
    kind: Literal["low", "top_swing", "top_lockout", "rest"]


def tremor_rms(acc: np.ndarray, fs: float, band) -> float:
    """RMS (m/s²) of the `band` (Hz, inclusive) content of |f| over a window of specific force
    `acc` (n, 3): |f| is linearly detrended and Hann-tapered (the taper keeps the motion tails at
    the window edges out of the band), and the band power is read from the FFT and corrected for
    the taper's power loss."""
    mag = np.linalg.norm(np.asarray(acc, dtype=float), axis=1)
    n = len(mag)
    if n < 3:
        return 0.0
    x = np.arange(n, dtype=float)
    resid = mag - np.polyval(np.polyfit(x, mag, 1), x)
    w = np.hanning(n)
    spec = np.fft.rfft(resid * w)
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (freq >= band[0]) & (freq <= band[1]) & (freq > 0) & (freq < fs / 2)
    power = 2.0 * np.sum(np.abs(spec[sel]) ** 2) / n**2 / np.mean(w**2)
    return float(math.sqrt(power))


def sagittal_tilt(quats: np.ndarray, calib: Calibration, forward_w: np.ndarray) -> np.ndarray:
    """(N,) signed tilt θ (rad) of bell-up in the vertical plane containing `forward_w` (unit,
    horizontal), positive when bell-up leans forward; in (−π, π]."""
    up = quat.rotate(quats, calib.up_s)
    return np.arctan2(up @ np.asarray(forward_w, dtype=float), up[:, 2])


def _window_forward(quats: np.ndarray, calib: Calibration, i0: int, i1: int) -> np.ndarray:
    out = quat.rotate(quats[i0:i1 + 1], calib.out_s).mean(axis=0)
    h = np.array([-out[0], -out[1], 0.0])
    norm = float(np.linalg.norm(h))
    if norm < MIN_FORWARD_NORM:
        raise ValueError(f"rest {i0}..{i1}: outward axis near vertical, forward undefined")
    return h / norm


def rest_forward(quats: np.ndarray, calib: Calibration, rest: Anchor) -> np.ndarray:
    """Unit horizontal forward −horizontal(R·out_s), averaged over the rest window. Raises
    ValueError if the outward axis is too close to vertical (bell lying on a flat)."""
    return _window_forward(quats, calib, rest.i0, rest.i1)


def _forward_chain(quats: np.ndarray, calib: Calibration,
                   rests: list[tuple[int, int]]) -> tuple[np.ndarray, list[np.ndarray]]:
    """Forward of the calibration window, then of each rest (i0, i1) in order: the rest's own
    axis with its sign kept continuous with the previous forward; the previous forward where the
    rest gives none."""
    still = calib.still
    try:
        prev = _window_forward(quats, calib, int(still.start or 0), int(still.stop) - 1)
    except ValueError:
        prev = np.array([1.0, 0.0, 0.0])  # calibration defines world +x as forward
    first = prev
    chain = []
    for i0, i1 in rests:
        try:
            f = _window_forward(quats, calib, i0, i1)
            if f @ prev < 0.0:
                f = -f
        except ValueError:
            f = prev
        chain.append(f)
        prev = f
    return first, chain


def set_forwards(quats: np.ndarray, calib: Calibration, anchors: list[Anchor]) -> list[np.ndarray]:
    """One unit horizontal forward per set (run of non-rest anchors between rests), read from the
    rest before the set (the calibration window if no rest precedes it), sign-continuous."""
    anchors = sorted(anchors)
    rests = [(a.i0, a.i1) for a in anchors if a.kind == "rest"]
    first, chain = _forward_chain(quats, calib, rests)
    out: list[np.ndarray] = []
    current, k, in_set = first, 0, False
    for a in anchors:
        if a.kind == "rest":
            current, in_set = chain[k], False
            k += 1
        elif not in_set:
            out.append(current.copy())
            in_set = True
    return out


def _static_windows(imu: ImuData, cfg: AnchorConfig) -> tuple[list, list]:
    """Static runs [a, b) of at least `static_min_s`, split into rests and hand-held windows."""
    need = max(1, int(math.ceil(cfg.static_min_s * imu.fs - 1e-9)))
    need_rest = int(math.ceil(cfg.rest_min_s * imu.fs - 1e-9))
    rests, held = [], []
    for a, b in zip(*_runs(static_mask(imu))):
        a, b = int(a), int(b)
        if b - a < need:
            continue
        quiet = tremor_rms(imu.acc[a:b], imu.fs, cfg.tremor_band) <= cfg.tremor_rms_max
        (rests if quiet and b - a >= need_rest else held).append((a, b))
    return rests, held


def _prominence(th: np.ndarray, i: int) -> float:
    """Topographic prominence of the peak th[i] within the 1-D array th (edges bound the search,
    as for scipy.signal.peak_prominences)."""
    higher = np.flatnonzero(th[:i] > th[i])
    start = int(higher[-1]) + 1 if len(higher) else 0
    higher = np.flatnonzero(th[i + 1:] > th[i])
    stop = i + 1 + int(higher[0]) if len(higher) else len(th)
    return float(th[i] - max(th[start:i + 1].min(), th[i:stop].min()))


def _lows(th: np.ndarray, f_ok: np.ndarray, held: np.ndarray, min_prom: float) -> list[int]:
    """Indices (into th) of the local maxima of θ that are not hand-held static, have enough
    specific force and stand out by `min_prom`."""
    if len(th) < 3:
        return []
    c = np.flatnonzero((th[1:-1] > th[:-2]) & (th[1:-1] >= th[2:])) + 1
    c = c[f_ok[c] & ~held[c]]
    return [int(i) for i in c if _prominence(th, int(i)) >= min_prom]


def detect_anchors(imu: ImuData, quats: np.ndarray, kin: Kinematics, calib: Calibration,
                   cfg: AnchorConfig = AnchorConfig()) -> list[Anchor]:
    """Rests, lows, swing tops and lockouts, sorted by position.

    1. Static windows (`static_mask`) of at least `static_min_s`: a rest if the tremor-band RMS of
       |f| is at most `tremor_rms_max` and it lasts `rest_min_s`; otherwise hand-held.
    2. Between rests, θ uses the forward of the rest before (see the module docstring).
    3. Lows: local maxima of θ with prominence ≥ `prominence_deg` (within the stretch between
       rests), outside hand-held static windows, at a valid sample with |f| ≥ `low_min_f_g`·g.
    4. Between consecutive lows of a stretch: the longest hand-held static window inside the
       interval (`top_lockout`), else the θ minimum (`top_swing`).
    """
    n = len(imu.t)
    rests, held_wins = _static_windows(imu, cfg)
    held = np.zeros(n, dtype=bool)
    for a, b in held_wins:
        held[a:b] = True
    with np.errstate(invalid="ignore"):
        f_ok = imu.valid & (kin.f_norm >= cfg.low_min_f_g * G)
    first, chain = _forward_chain(quats, calib, [(a, b - 1) for a, b in rests])

    anchors = [Anchor(a, b - 1, "rest") for a, b in rests]
    bounds = [0] + [x for a, b in rests for x in (a, b)] + [n]
    forwards = [first] + chain
    min_prom = math.radians(cfg.prominence_deg)
    for (s0, s1), fwd in zip(zip(bounds[::2], bounds[1::2]), forwards):
        if s1 - s0 < 3:
            continue
        th = sagittal_tilt(quats[s0:s1], calib, fwd)
        lows = _lows(th, f_ok[s0:s1], held[s0:s1], min_prom)
        anchors += [Anchor(s0 + i, s0 + i, "low") for i in lows]
        for l0, l1 in zip(lows[:-1], lows[1:]):
            inside = [(a, b) for a, b in held_wins if a >= s0 + l0 and b - 1 <= s0 + l1]
            if inside:
                a, b = max(inside, key=lambda w: w[1] - w[0])
                anchors.append(Anchor(a, b - 1, "top_lockout"))
            else:
                i = s0 + l0 + int(np.argmin(th[l0:l1 + 1]))
                anchors.append(Anchor(i, i, "top_swing"))
    return sorted(anchors)
