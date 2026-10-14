"""Rep and set segmentation and per-rep metrics (spec §6.1, §7.1).

Reps
----
A rep is a run of three consecutive anchors low → top → low (``segment_reps``): (i_low, i_top,
i_top_end, i_end) with i_top / i_top_end the first / last sample of the top anchor (a lockout
window; i_top_end = i_top for a ``top_swing``) and i_end the next low. Concentric phase =
[i_low, i_top], eccentric phase = [i_top_end, i_end]. A final top with no closing backswing
(the bell put down from the top: low → top → rest) is not counted as a rep, nor is a rep whose
next low lies past the recorded samples.

Metrics (``rep_metrics``)
-------------------------
- ``power_w`` (headline): concentric work / concentric duration (``integrate.concentric_power``).
- ``work_con_j``: trapezoid ∫P dt over the concentric phase (so power_w = work_con_j /
  con_duration_s); ``work_ecc_j``: ∫P dt over the eccentric phase (negative: the lifter absorbs
  the drop). P = 0 inside a lockout window (v = 0 there).
- ``peak_power_w``: max over the finite concentric samples of P smoothed by a 25 ms centred
  moving average of finite samples (a gap just outside the rep does not poison it; NaN if the
  concentric phase holds no finite P); ``peak_velocity_ms``: max ‖v‖ over the concentric phase.
- ``apex_m``: max position z over [i_low, i_top_end] above the low (as ``classify``).
- ``con_duration_s`` = t[i_top] − t[i_low]; ``cycle_s`` = t[i_end] − t[i_low].
- ``flags``: "gap" if any sample in [i_low, i_end] is invalid or has a non-finite velocity
  (metrics there are NaN or unreliable), "saturated" if any is saturated.

Sets
----
A set is a run of non-rest anchors between rests (``segment_sets``): from the sample after the
rest before it (the bell leaving the floor) to the sample before the rest after it (back on the
floor); without a rest on that side, the run's first / last anchor bounds it. ``build_sets``
assembles classified ``Rep`` objects into ``SetResult`` (``Rep.path`` is left None; paths are
added later), dropping sets without reps (their time goes to the next set's rest). Stdlib +
numpy only.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .anchors import Anchor
from .calibration import Calibration
from .classify import ClassifierConfig, classify, rep_features
from .imu import ImuData
from .integrate import concentric_power

PEAK_SMOOTH_S = 0.025  # moving-average window for peak power


@dataclass
class Rep:
    i_low: int
    i_top: int
    i_top_end: int  # last sample of the lockout window; = i_top for a swing top
    i_end: int  # next low
    kind: str  # snatch | swing | other
    hand: str  # L | R | ? (snatch), - (swing, other)
    confidence: float
    features: dict
    flags: set[str]
    power_w: float
    peak_power_w: float
    peak_velocity_ms: float
    work_con_j: float
    work_ecc_j: float
    apex_m: float
    con_duration_s: float
    cycle_s: float
    path: np.ndarray | None = None


@dataclass
class SetResult:
    reps: list[Rep]
    i_start: int
    i_end: int
    t_start: float
    t_end: float
    rest_before_s: float  # rest window(s) + dropped empty sets before the set, 0 if none


def segment_reps(anchors: list[Anchor], n_samples: int) -> list[tuple[int, int, int, int]]:
    """(i_low, i_top, i_top_end, i_end) for every low → top → low run of consecutive anchors
    (in position order) whose next low lies inside the `n_samples` recorded samples."""
    a = sorted(anchors)
    out = []
    for lo, top, nxt in zip(a[:-2], a[1:-1], a[2:]):
        if (lo.kind == "low" and top.kind in ("top_swing", "top_lockout") and nxt.kind == "low"
                and nxt.i0 < n_samples):
            out.append((lo.i0, top.i0, top.i1, nxt.i0))
    return out


def _set_runs(anchors: list[Anchor]) -> list[tuple[Anchor | None, list[Anchor], Anchor | None]]:
    """(rest before or None, run of non-rest anchors, rest after or None) per set."""
    out, before, run = [], None, []
    for a in sorted(anchors):
        if a.kind == "rest":
            if run:
                out.append((before, run, a))
            before, run = a, []
        else:
            run.append(a)
    if run:
        out.append((before, run, None))
    return out


def _bounds(before: Anchor | None, run: list[Anchor], after: Anchor | None) -> tuple[int, int]:
    return (before.i1 + 1 if before else run[0].i0, after.i0 - 1 if after else run[-1].i1)


def segment_sets(anchors: list[Anchor]) -> list[tuple[int, int]]:
    """(i_start, i_end) inclusive per set (run of non-rest anchors between rests): the sample
    after the rest before to the sample before the rest after; the run's first / last anchor
    where no rest bounds it."""
    return [_bounds(*r) for r in _set_runs(anchors)]


def moving_average(x: np.ndarray, k: int) -> np.ndarray:
    """Centred moving average over `k` samples (window [i − k//2, i + k − k//2)), truncated at
    the array ends, of the finite samples only (sum of finite / count of finite): NaN where a
    window holds no finite sample."""
    x = np.asarray(x, dtype=float)
    k = max(1, int(k))
    idx = np.arange(len(x)) + k - 1 - k // 2  # full-convolution index of each window's end
    kernel = np.ones(k)
    finite = np.isfinite(x)
    sums = np.convolve(np.where(finite, x, 0.0), kernel)[idx]
    counts = np.convolve(finite.astype(float), kernel)[idx]
    return np.divide(sums, counts, out=np.full(len(x), np.nan), where=counts > 0.5)


def _work(P: np.ndarray, t: np.ndarray, i0: int, i1: int) -> float:
    p, tt = P[i0:i1 + 1], t[i0:i1 + 1]
    return float(np.sum(0.5 * (p[1:] + p[:-1]) * np.diff(tt)))


def rep_metrics(seg: tuple[int, int, int, int], t: np.ndarray, v: np.ndarray, pos: np.ndarray,
                P: np.ndarray, valid: np.ndarray, saturated: np.ndarray) -> dict:
    """Numeric ``Rep`` fields and ``flags`` of the rep `seg` (see the module docstring), from
    the ``integrate`` arrays `v`, `pos`, `P` and the ``ImuData`` masks `valid`, `saturated`."""
    i_low, i_top, i_top_end, i_end = (int(i) for i in seg)
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    P = np.asarray(P, dtype=float)
    fs = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 1.0
    k = max(1, round(PEAK_SMOOTH_S * fs))
    s0, s1 = max(0, i_low - k), min(len(P), i_top + k + 1)  # padded: full windows at the ends
    smooth = moving_average(P[s0:s1], k)[i_low - s0:i_top - s0 + 1]
    smooth = smooth[np.isfinite(P[i_low:i_top + 1]) & np.isfinite(smooth)]  # in-rep samples
    span = slice(i_low, i_end + 1)
    flags = set()
    if not (np.all(valid[span]) and np.all(np.isfinite(v[span]))):
        flags.add("gap")
    if np.any(saturated[span]):
        flags.add("saturated")
    z = np.asarray(pos, dtype=float)[:, 2]
    return {
        "power_w": concentric_power(P, t, i_low, i_top),
        "peak_power_w": float(np.max(smooth)) if len(smooth) else float("nan"),
        "peak_velocity_ms": float(np.max(np.linalg.norm(v[i_low:i_top + 1], axis=1))),
        "work_con_j": _work(P, t, i_low, i_top),
        "work_ecc_j": _work(P, t, i_top_end, i_end),
        "apex_m": float(np.max(z[i_low:i_top_end + 1]) - z[i_low]),
        "con_duration_s": float(t[i_top] - t[i_low]),
        "cycle_s": float(t[i_end] - t[i_low]),
        "flags": flags,
    }


def build_sets(anchors: list[Anchor], imu: ImuData, quats: np.ndarray, calib: Calibration,
               v: np.ndarray, pos: np.ndarray, P: np.ndarray,
               cfg: ClassifierConfig = ClassifierConfig()) -> list[SetResult]:
    """Segment, measure (``rep_metrics``), describe (``rep_features``) and classify every rep,
    grouped into sets; a rep belongs to the set whose span holds its low. Reps whose confidence
    is below ``cfg.low_conf`` get the flag "low_conf". A set without reps (bell picked up and
    put down) is dropped and its time merged into the next set's ``rest_before_s``, which runs
    from the start of the rest before (the earliest one, after dropped sets) to the set's
    first sample."""
    t = imu.t
    reps = []
    for seg in segment_reps(anchors, len(t)):
        m = rep_metrics(seg, t, v, pos, P, imu.valid, imu.saturated)
        feats = rep_features(seg, quats, calib, pos, t)
        kind, hand, conf = classify(feats, cfg)
        flags = m.pop("flags")
        if conf < cfg.low_conf:
            flags.add("low_conf")
        reps.append(Rep(*seg, kind=kind, hand=hand, confidence=conf, features=feats,
                        flags=flags, **m))
    sets = []
    since = None  # start of the stretch without a kept set: rests and dropped (empty) sets
    for before, run, after in _set_runs(anchors):
        i0, i1 = _bounds(before, run, after)
        if since is None:
            since = before.i0 if before else i0
        in_set = [r for r in reps if i0 <= r.i_low <= i1]
        if not in_set:
            continue  # no rep (e.g. picked up and put down): its time counts as rest
        sets.append(SetResult(in_set, i0, i1, float(t[i0]), float(t[i1]),
                              float(t[i0] - t[since])))
        since = None
    return sets
