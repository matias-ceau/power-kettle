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

Forward (the θ plane)
---------------------
Per stretch between rests (``motion_forward``), the sagittal plane comes from the motion: in a
swing or snatch the world bell-up vector u = R·up_s sweeps the sagittal plane (the snatch twist
turns the bell about u itself), so the forward axis is the principal axis (uncentred) of the
horizontal part of u over the stretch's valid samples. (The principal horizontal axis of ω,
the lateral axis, was tried first: the snatch twist about a tilted bell-up puts horizontal ω off
that axis and turned the plane by ~8° in simulation.) Its sign comes from the motion too: at a
backswing low the bell is reversed hard from moving back to moving forward, so the mean COM
acceleration along the forward at the θ maxima (prominence ≥ ``prominence_deg``, outside
hand-held windows) is large and positive (≈ 8–15 m/s² in simulation) under the right sign;
under the wrong sign the θ maxima are the swing tops, mid-pulls and drops, where it is ≈ 0. The
sign whose value exceeds the other's by ``SIGN_MIN_ACC`` wins. (The asymmetry of θ itself, the
most negative θ larger in magnitude than the most positive, holds for swings, 40° / −90°, but
not for snatches, 40° / −30°.)

The rest-based forward (``rest_forward``) is the fallback: for a stretch with fewer than
``MOTION_MIN_SAMPLES`` samples rotating faster than ``MOTION_OMEGA_MIN`` (the axis), and for the
sign when the accelerations do not decide it. At a floor rest the front flat faces the lifter,
so forward = −horizontal(R·out_s), averaged over the rest window; each stretch's fallback is the
forward of the rest before it (the calibration window before the first rest). Its axis is
re-read at every rest, its sign kept continuous with the previous fallback (a forward that
turns by more than 90° between rests is taken to be the bell parked turned round, or a hold
without tremor provisionally taken as a rest); a rest where the bell does not stand upright
(mean bell-up within ``forward_upright_max_deg`` of vertical) keeps the previous one. A bell
parked turned about its own axis (any yaw) or a lifter who turns round between sets is
handled by the motion; the fallback is only right when the bell is parked facing the lifter.

Sets in anchor terms
--------------------
A set is the run of anchors between two rest anchors. It opens with its first low (the lift
from the floor to the first backswing is not a rep) and closes with its last low (the descent
to the floor is not a rep): a top anchor is only placed between two consecutive lows with no
rest between them, so a rest is always next to a low or another rest, never a top.

Known limits (to validate on real data)
---------------------------------------
- A set ends only at a rest (≥ ``rest_min_s``, no tremor). If the bell only touches the floor
  between two sets, the two lows around the touch are consecutive and a top is placed at the
  touch: a ``top_lockout`` if the bell sat static for ≥ ``static_min_s`` (but < ``rest_min_s``),
  else a ``top_swing`` at the θ minimum. A floor rest split by a BLE gap into pieces shorter than
  ``rest_min_s`` behaves the same. The classifier labels such a "rep" other (negative apex).
- If a snatch lockout is too short or unsteady for ``static_mask``, no hand-held window lies
  between its lows and the θ minimum is taken as a ``top_swing``: that is the mid-pull (or drop)
  minimum, not a zero-velocity point.
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

LOW_GATE_HALF_S = 0.025  # the |f| gate of a low looks this far (±10 samples at 400 Hz) around it
HELD_MARGIN_S = 0.05  # no low this close to a hand-held static window (θ plateau edges)
MOTION_OMEGA_MIN = 1.0  # rad/s: a sample rotating this fast counts towards the motion axis
MOTION_MIN_SAMPLES = 50  # fewer such samples in a stretch: the rest-based forward is used
SIGN_MIN_ACC = 2.0  # m/s²: margin by which the motion decides the forward's sign


@dataclass
class AnchorConfig:
    prominence_deg: float = 30.0  # a low must stand out of θ by this much (hysteresis)
    low_min_f_g: float = 1.3  # |f| at a low, in g (the arm pulls the bell out of the backswing)
    static_min_s: float = 0.15  # shortest static window considered (lockout / rest)
    rest_min_s: float = 2.0
    tremor_band: tuple = (4.0, 12.0)  # Hz, physiological tremor
    tremor_rms_max: float = 0.02  # m/s², band RMS of f above which a static window is hand-held
    floor_max_height_m: float = 0.8  # a rest this high above its low is a lockout
    forward_upright_max_deg: float = 20.0  # a rest gives a forward only if the bell is this upright


@dataclass(order=True)
class Anchor:
    i0: int
    i1: int  # inclusive; i0 == i1 for point anchors (low, top_swing)
    kind: Literal["low", "top_swing", "top_lockout", "rest"]


def tremor_rms(acc: np.ndarray, fs: float, band) -> float:
    """RMS (m/s²) of the `band` (Hz, inclusive) content of the specific-force vector over a
    window `acc` (n, 3), combined over the three axes (so independent of the tremor's direction
    and of the sensor mounting): each axis is linearly detrended and Hann-tapered (the taper
    keeps the motion tails at the window edges out of the band), and the band power is read from
    the FFT and corrected for the taper's power loss."""
    acc = np.asarray(acc, dtype=float)
    n = len(acc)
    if n < 3:
        return 0.0
    x = np.arange(n, dtype=float)
    coef = np.polyfit(x, acc, 1)  # (2, 3): slope and intercept per axis
    resid = acc - (np.outer(x, coef[0]) + coef[1])
    w = np.hanning(n)
    spec = np.fft.rfft(resid * w[:, None], axis=0)
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (freq >= band[0]) & (freq <= band[1]) & (freq > 0) & (freq < fs / 2)
    power = 2.0 * np.sum(np.abs(spec[sel]) ** 2) / n**2 / np.mean(w**2)
    return float(math.sqrt(power))


def sagittal_tilt(quats: np.ndarray, calib: Calibration, forward_w: np.ndarray) -> np.ndarray:
    """(N,) signed tilt θ (rad) of bell-up in the vertical plane containing `forward_w` (unit,
    horizontal), positive when bell-up leans forward; in (−π, π]."""
    up = quat.rotate(quats, calib.up_s)
    return np.arctan2(up @ np.asarray(forward_w, dtype=float), up[:, 2])


def _window_forward(quats: np.ndarray, calib: Calibration, i0: int, i1: int,
                    upright_max_deg: float) -> np.ndarray:
    up = quat.rotate(quats[i0:i1 + 1], calib.up_s).mean(axis=0)
    tilt = math.degrees(math.atan2(math.hypot(up[0], up[1]), up[2]))
    if tilt > upright_max_deg:
        raise ValueError(f"rest {i0}..{i1}: bell-up {tilt:.0f}° from vertical (not standing "
                         f"upright), forward undefined")
    out = quat.rotate(quats[i0:i1 + 1], calib.out_s).mean(axis=0)
    h = np.array([-out[0], -out[1], 0.0])
    return h / np.linalg.norm(h)  # upright => outward within upright_max_deg of horizontal


def rest_forward(quats: np.ndarray, calib: Calibration, rest: Anchor,
                 upright_max_deg: float = AnchorConfig.forward_upright_max_deg) -> np.ndarray:
    """Unit horizontal forward −horizontal(R·out_s), averaged over the rest window. Raises
    ValueError unless the bell stands upright there (mean bell-up within `upright_max_deg` of
    vertical); a bell lying on its side or a flat gives no forward."""
    return _window_forward(quats, calib, rest.i0, rest.i1, upright_max_deg)


def _forward_chain(quats: np.ndarray, calib: Calibration, rests: list[tuple[int, int]],
                   upright_max_deg: float,
                   first: np.ndarray | None = None) -> tuple[np.ndarray, list[np.ndarray]]:
    """Forward of the calibration window (or `first`, when given), then of each rest (i0, i1)
    in order: the rest's own axis with its sign kept continuous with the previous forward; the
    previous forward where the rest gives none (bell not upright)."""
    still = calib.still
    if first is not None:
        prev = np.asarray(first, dtype=float)
    else:
        try:
            prev = _window_forward(quats, calib, int(still.start or 0), int(still.stop) - 1,
                                   upright_max_deg)
        except ValueError:
            prev = np.array([1.0, 0.0, 0.0])  # calibration defines world +x as forward
    first = prev
    chain = []
    for i0, i1 in rests:
        try:
            f = _window_forward(quats, calib, i0, i1, upright_max_deg)
            if f @ prev < 0.0:
                f = -f
        except ValueError:
            f = prev
        chain.append(f)
        prev = f
    return first, chain


@dataclass
class StretchMotion:
    """Sums over (the first part of) a stretch between rests that its forward comes from."""
    uu: np.ndarray  # (2, 2) Σ u_h u_hᵀ, u_h = horizontal part of world bell-up, valid samples
    moving: int  # valid samples with |ω| > MOTION_OMEGA_MIN
    sign_ref: np.ndarray | None  # (3,) forward whose sign the motion decided; None: undecided


@dataclass
class Stretch:
    """Samples [s0, s1) between rests and their forward (θ plane)."""
    s0: int
    s1: int
    hint: np.ndarray  # rest-based forward (fallback)
    forward: np.ndarray
    motion: StretchMotion


def _horizontal(v: np.ndarray) -> np.ndarray:
    h = np.array([v[0], v[1], 0.0], dtype=float)
    return h / np.linalg.norm(h)


def _peak_sign(th: np.ndarray, a_fwd: np.ndarray, held: np.ndarray, min_prom: float) -> int:
    """+1 / −1 for the sign of the forward under which θ = `th` (computed with the + sign) is
    right, from the mean COM acceleration `a_fwd` along that forward at the θ maxima (module
    docstring); 0 when undecided."""
    cue = []
    for sgn in (1.0, -1.0):
        ts = sgn * th
        c = np.flatnonzero((ts[1:-1] > ts[:-2]) & (ts[1:-1] >= ts[2:])) + 1
        c = c[~held[c] & np.isfinite(a_fwd[c])]
        peaks = [int(i) for i in c if _prominence(ts, int(i)) >= min_prom]
        cue.append(float(np.mean(sgn * a_fwd[peaks])) if peaks else -math.inf)
    if cue[0] >= cue[1] + SIGN_MIN_ACC:
        return 1
    if cue[1] >= cue[0] + SIGN_MIN_ACC:
        return -1
    return 0


def motion_sums(quats: np.ndarray, calib: Calibration, omega_w: np.ndarray, valid: np.ndarray,
                prior: StretchMotion | None = None) -> StretchMotion:
    """``StretchMotion`` sums over samples (`quats`, world `omega_w`, `valid`) of a stretch, plus
    those of `prior` (an earlier part of the same stretch, whose sign decision is kept)."""
    up = quat.rotate(quats, calib.up_s)
    ok = valid & np.all(np.isfinite(up), axis=1)
    uh = up[ok, :2]
    with np.errstate(invalid="ignore"):
        fast = ok & (np.linalg.norm(omega_w, axis=1) > MOTION_OMEGA_MIN)
    if prior is None:
        return StretchMotion(uh.T @ uh, int(np.sum(fast)), None)
    return StretchMotion(uh.T @ uh + prior.uu, int(np.sum(fast)) + prior.moving, prior.sign_ref)


def motion_forward(quats: np.ndarray, calib: Calibration, kin: Kinematics, valid: np.ndarray,
                   held: np.ndarray, hint: np.ndarray, min_prom: float,
                   prior: StretchMotion | None = None) -> tuple[np.ndarray, StretchMotion]:
    """(unit horizontal forward, motion sums) of a stretch from its samples (`quats`, `kin`,
    `valid`, `held` = hand-held static mask, all over the stretch) and the rest-based `hint`
    (module docstring). `prior` holds the sums over an earlier part of the same stretch (live
    analysis): they are added, and a sign it has decided is kept."""
    m = motion_sums(quats, calib, kin.omega_w, valid, prior)
    hint = _horizontal(hint)
    if m.moving < MOTION_MIN_SAMPLES:
        return hint, m
    _, vecs = np.linalg.eigh(m.uu)  # ascending: the last is the principal axis
    f = np.array([vecs[0, -1], vecs[1, -1], 0.0])
    if m.sign_ref is None:
        up = quat.rotate(quats, calib.up_s)
        th = np.where(valid, np.arctan2(up @ f, up[:, 2]), np.nan)
        sgn = _peak_sign(th, kin.a_com @ f, held, min_prom)
        if sgn:
            m.sign_ref = sgn * f
    ref = hint if m.sign_ref is None else m.sign_ref
    return (f if f @ ref >= 0.0 else -f), m


def _held_mask(imu: ImuData, held_wins: list[tuple[int, int]]) -> np.ndarray:
    """Hand-held static windows widened by HELD_MARGIN_S (the θ maximum of a lockout plateau can
    fall a few samples outside the window)."""
    held = np.zeros(len(imu.t), dtype=bool)
    margin = round(HELD_MARGIN_S * imu.fs)
    for a, b in held_wins:
        held[max(0, a - margin):b + margin] = True
    return held


def stretch_forwards(imu: ImuData, quats: np.ndarray, kin: Kinematics, calib: Calibration,
                     rests: list[tuple[int, int]], held: np.ndarray, cfg: AnchorConfig,
                     forward0: np.ndarray | None = None,
                     motion0: StretchMotion | None = None) -> list[Stretch]:
    """One ``Stretch`` per stretch between the rest windows `rests` ((i0, i1) inclusive,
    sorted): before the first, between consecutive ones, after the last (empty stretches
    included). `forward0` / `motion0`: the fallback forward and motion sums carried into the
    stretch holding sample 0 (live analysis; default: the calibration window's forward)."""
    first, chain = _forward_chain(quats, calib, rests, cfg.forward_upright_max_deg, forward0)
    bounds = [0] + [x for a, b in rests for x in (a, b + 1)] + [len(imu.t)]
    out = []
    min_prom = math.radians(cfg.prominence_deg)
    for k, (s0, s1) in enumerate(zip(bounds[::2], bounds[1::2])):
        hint = first if k == 0 else chain[k - 1]
        sl = slice(s0, s1)
        part = Kinematics(kin.a_com[sl], kin.omega_w[sl], kin.alpha_w[sl], kin.f_norm[sl])
        fwd, motion = motion_forward(quats[sl], calib, part, imu.valid[sl], held[sl], hint,
                                     min_prom, motion0 if k == 0 else None)
        out.append(Stretch(s0, s1, hint, fwd, motion))
    return out


def set_forwards(quats: np.ndarray, calib: Calibration, anchors: list[Anchor],
                 cfg: AnchorConfig = AnchorConfig(), imu: ImuData | None = None,
                 kin: Kinematics | None = None) -> list[np.ndarray]:
    """One unit horizontal forward per set (run of non-rest anchors between rests): with `imu`
    and `kin`, the forward of the set's stretch from the motion, as ``detect_anchors`` (module
    docstring); without, the rest-based forward of the rest before the set (the calibration
    window if no rest precedes it), sign-continuous, a rest where the bell is not upright
    keeping the previous forward."""
    anchors = sorted(anchors)
    rests = [(a.i0, a.i1) for a in anchors if a.kind == "rest"]
    if imu is not None and kin is not None:
        _, held_wins = _static_windows(imu, cfg)
        stretches = stretch_forwards(imu, quats, kin, calib, rests,
                                     _held_mask(imu, held_wins), cfg)
        fwds = [st.forward for st in stretches]
    else:
        first, chain = _forward_chain(quats, calib, rests, cfg.forward_upright_max_deg)
        fwds = [first] + chain
    out: list[np.ndarray] = []
    k, in_set = 0, False
    for a in anchors:
        if a.kind == "rest":
            k, in_set = k + 1, False
        elif not in_set:
            out.append(fwds[k].copy())
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


def _f_gate(f_norm: np.ndarray, valid: np.ndarray, half: int, f_min: float) -> np.ndarray:
    """(N,) bool: the max |f| over the valid samples within ±`half` samples reaches `f_min`
    (False where that neighbourhood holds no valid sample). A window rather than the θ-peak
    sample alone, so one invalid (gap-guard) sample at a backswing apex does not drop the low."""
    f = np.where(valid & np.isfinite(f_norm), f_norm, -np.inf)
    padded = np.concatenate([np.full(half, -np.inf), f, np.full(half, -np.inf)])
    return np.lib.stride_tricks.sliding_window_view(padded, 2 * half + 1).max(axis=1) >= f_min


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
                   cfg: AnchorConfig = AnchorConfig(),
                   forward0: np.ndarray | None = None) -> list[Anchor]:
    """Rests, lows, swing tops and lockouts, sorted by position (``detect_anchors_and_forwards``
    without the stretches)."""
    return detect_anchors_and_forwards(imu, quats, kin, calib, cfg, forward0)[0]


def detect_anchors_and_forwards(imu: ImuData, quats: np.ndarray, kin: Kinematics,
                                calib: Calibration, cfg: AnchorConfig = AnchorConfig(),
                                forward0: np.ndarray | None = None,
                                motion0: StretchMotion | None = None
                                ) -> tuple[list[Anchor], list[Stretch]]:
    """Rests, lows, swing tops and lockouts, sorted by position, and the stretches between the
    rests with their forwards (``stretch_forwards``).

    `forward0` (unit, horizontal) replaces the calibration window's forward as the fallback
    forward in force at sample 0, and `motion0` holds the motion sums of the stretch's part
    before sample 0, for data that starts after the calibration window (``live``).

    1. Static windows (`static_mask`) of at least `static_min_s`: a rest if the tremor-band RMS of
       f (`tremor_rms`) is at most `tremor_rms_max` and it lasts `rest_min_s`; otherwise hand-held.
    2. Between rests, θ uses the stretch's forward from the motion (see the module docstring).
    3. Lows: local maxima of θ with prominence ≥ `prominence_deg` (within the stretch between
       rests), outside hand-held static windows (widened by ±50 ms), with |f| ≥ `low_min_f_g`·g at some valid sample
       within ±25 ms.
    4. Between consecutive lows of a stretch: the longest hand-held static window inside the
       interval (`top_lockout`), else the θ minimum (`top_swing`).
    """
    rests, held_wins = _static_windows(imu, cfg)
    held = _held_mask(imu, held_wins)
    f_ok = _f_gate(kin.f_norm, imu.valid, max(0, round(LOW_GATE_HALF_S * imu.fs)),
                   cfg.low_min_f_g * G)
    stretches = stretch_forwards(imu, quats, kin, calib, [(a, b - 1) for a, b in rests], held,
                                 cfg, forward0, motion0)

    anchors = [Anchor(a, b - 1, "rest") for a, b in rests]
    min_prom = math.radians(cfg.prominence_deg)
    for st in stretches:
        s0, s1 = st.s0, st.s1
        if s1 - s0 < 3:
            continue
        th = sagittal_tilt(quats[s0:s1], calib, st.forward)
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
    return sorted(anchors), stretches


def refine_rests(anchors: list[Anchor], pos: np.ndarray,
                 cfg: AnchorConfig = AnchorConfig()) -> list[Anchor]:
    """Re-label as ``top_lockout`` every rest window held at least `floor_max_height_m` above the
    preceding low anchor of its set (a tremor-free overhead hold, which ``detect_anchors`` cannot
    tell from a floor rest). Height = pos z at the window's middle sample minus pos z at that
    low, with `pos` from ``integrate.segment_position`` on these anchors (inside a rest it stays
    at the height reached on entering it). Position restarts at 0 at the end of every rest core,
    so after a rest is re-labelled, later rests of the set add its height (running offset, reset
    at each low). A rest whose set has no low before it (the anchor before it is a rest, or
    none) or whose height is NaN stays a rest. Returns a new sorted list; the input is not
    modified. Callers then re-run ``segment_velocity`` /
    ``segment_position`` once on the result (a lockout is a zero-velocity window over its full
    length and position no longer restarts after it)."""
    anchors = sorted(anchors)
    out: list[Anchor] = []
    base: float | None = None  # height of the current position origin above the set's low
    for a in anchors:
        if a.kind == "low":
            base = -float(pos[a.i0, 2])  # 0 for segment_position (it restarts at lows)
        elif a.kind == "rest":
            if base is not None:
                height = base + float(pos[(a.i0 + a.i1) // 2, 2])
                if height >= cfg.floor_max_height_m:  # False for NaN
                    out.append(Anchor(a.i0, a.i1, "top_lockout"))
                    base = height  # segment_position restarted at 0 at this rest's core end
                    continue
            base = None  # a (floor) rest ends the set
        out.append(a)
    return out
