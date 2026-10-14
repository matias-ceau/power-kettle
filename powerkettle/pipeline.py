"""End-to-end analysis of one run (spec §5–7): the production chain of every stage.

``analyse_run`` scales the run, calibrates (or carries a previous run's calibration over),
divides the accelerometer by the |g| measured at rest (``calibration.correct_acc_scale``),
estimates orientation and COM acceleration, finds anchors, integrates velocity / position (twice:
before and after ``refine_rests``), computes power, builds and classifies reps and sets, applies
label corrections, fills each rep's athlete-frame path and computes set and session metrics.

Labels
------
``labels`` maps a rep's session-wide index (0-based, in time order across sets) to
``{"kind": …, "hand": …}`` (either key optional). A label overrides the classifier and adds the
flag "relabelled". A kind other than snatch without a hand gets hand "-"; a snatch without a hand
keeps the classifier's hand if it was already a snatch, else "?". A hand must fit the kind:
L / R / ? for a snatch, "-" for swing and other; anything else (e.g. a hand-only label "L" on a
rep classified swing) raises ValueError. Labels for indices past the reps found are ignored.

Markers
-------
``map_markers`` places phone-clock markers (``Recording.markers``) on a run's sample index: the
nearest packet received at or before the marker (the first packet if none) plus the elapsed
phone time × ``rate_hz``.

``SessionResult.summary()`` is JSON-safe: plain Python types only, NaN / ±inf → None, flags as
sorted lists, dict keys as strings, no arrays (paths are left out). Stdlib + numpy only.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np

from .anchors import (
    Anchor,
    AnchorConfig,
    Stretch,
    StretchMotion,
    detect_anchors_and_forwards,
    refine_rests,
    set_forwards,
)
from .bell import BellProfile
from .calibration import (
    Calibration,
    CalibrationConfig,
    CalibrationError,
    calibrate,
    correct_acc_scale,
    seeded_from_prior,
    with_mounting,
)
from .classify import ClassifierConfig
from .imu import ImuData, scale_run
from .integrate import power, segment_position, segment_velocity
from .kinematics import Kinematics, com_acceleration
from .metrics import SessionMetrics, SetMetrics, session_metrics, set_metrics
from .orientation import estimate_orientation, static_mask
from .paths import athlete_forward, rep_path
from .rawlog import Run
from .reps import Rep, SetResult, build_sets, segment_sets

ALGORITHM_VERSION = "1"
KINDS = ("snatch", "swing", "other")
SNATCH_HANDS = ("L", "R", "?")  # a snatch's hand; swing and other always have "-"
WARN_NOMINAL_RATE = "sample rate nominal (no status data)"
WARN_CARRIED_CALIBRATION = "calibration carried over from previous run"
PRE_CALIBRATION_WARN_S = 1.0  # more data than this before the calibration window: warning


def pre_calibration_warning(calib: Calibration, fs: float) -> list[str]:
    """[warning] when the orientation reference (``calib.still.start``) lies more than
    PRE_CALIBRATION_WARN_S into the run (the samples before it are integrated backwards)."""
    s = (calib.still.start or 0) / fs
    if s <= PRE_CALIBRATION_WARN_S:
        return []
    return [f"{s:.1f} s of data before the calibration window (orientation integrated "
            "backwards from it)"]


def calibrate_run(imu: ImuData, bell: BellProfile, ccfg: CalibrationConfig,
                  prior: Calibration | None = None) -> tuple[Calibration, ImuData, list[str]]:
    """(calibration, `imu` with the accelerometer scale corrected, warnings) of a run (see
    ``analyse_run``)."""
    warnings: list[str] = []
    try:
        calib = calibrate(imu, bell, ccfg)
        if prior is not None:
            calib = with_mounting(calib, prior)
        imu = correct_acc_scale(imu, calib)
    except CalibrationError:
        if prior is None:
            raise
        imu = correct_acc_scale(imu, prior)
        calib = seeded_from_prior(imu, prior, static_mask(imu))
        warnings.append(WARN_CARRIED_CALIBRATION)
    return calib, imu, warnings + calib.warnings + pre_calibration_warning(calib, imu.fs)


@dataclass
class AnalysisConfig:
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    classifier: ClassifierConfig = field(default_factory=ClassifierConfig)
    power_floor_w: float | None = None


def _jsonable(x):
    """Plain JSON value of `x`: NaN / inf → None, numpy scalars / arrays → Python, sets → sorted
    lists, dataclasses → dicts, dict keys → str."""
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (set, frozenset)):
        return [_jsonable(v) for v in sorted(x)]
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_jsonable(v) for v in x]
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: _jsonable(getattr(x, f.name)) for f in dataclasses.fields(x)}
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x) if math.isfinite(x) else None
    return x


_REP_FIELDS = ("i_low", "i_top", "i_top_end", "i_end", "kind", "hand", "confidence", "power_w",
               "peak_power_w", "peak_velocity_ms", "work_con_j", "work_ecc_j", "apex_m",
               "con_duration_s", "cycle_s", "flags", "features")


@dataclass
class SessionResult:
    """Result of ``analyse_run``. Beyond the session-level fields, ``set_metrics`` holds one
    ``SetMetrics`` per set (computed with the config's power floor, so callers need not
    recompute them) and ``t`` the run's sample times in s (rep start times in ``summary``
    and the CLI)."""
    calib: Calibration
    sets: list[SetResult]
    metrics: SessionMetrics
    duration_s: float
    warnings: list[str]
    algorithm_version: str
    set_metrics: list[SetMetrics] = field(default_factory=list)  # one per set
    t: np.ndarray | None = None  # (N,) s, sample times (for rep start times)

    def summary(self) -> dict:
        """JSON-safe summary: session metrics, per-set metrics and per-rep scalar fields."""
        reps, sets = [], []
        for k, s in enumerate(self.sets):
            sm = self.set_metrics[k] if k < len(self.set_metrics) else None
            sets.append({"i_start": s.i_start, "i_end": s.i_end, "t_start": s.t_start,
                         "t_end": s.t_end, "rest_before_s": s.rest_before_s,
                         "n_reps": len(s.reps), "metrics": sm})
            for r in s.reps:
                d = {"index": len(reps), "set": k,
                     "t_low": float(self.t[r.i_low]) if self.t is not None else None}
                d.update({name: getattr(r, name) for name in _REP_FIELDS})
                reps.append(d)
        return _jsonable({
            "algorithm_version": self.algorithm_version,
            "duration_s": self.duration_s,
            "warnings": list(self.warnings),
            "calibration": {"g_measured": self.calib.g_measured,
                            "gyro_bias": self.calib.gyro_bias, "up_s": self.calib.up_s,
                            "out_s": self.calib.out_s},
            "metrics": self.metrics,
            "sets": sets,
            "reps": reps,
        })


@dataclass
class Motion:
    """Outputs of the stages from orientation to power (``motion_stages``)."""
    q: np.ndarray  # (N, 4) world <- sensor
    bias: np.ndarray  # (N, 3) working gyro bias, rad/s
    kin: Kinematics
    raw_anchors: list[Anchor]  # detect_anchors (rests not yet refined)
    stretches: list[Stretch]  # between the raw rests, with their forwards (θ planes)
    anchors: list[Anchor]  # after refine_rests
    v: np.ndarray  # (N, 3) m/s
    pos: np.ndarray  # (N, 3) m
    P: np.ndarray  # (N,) W


def motion_stages(imu: ImuData, calib: Calibration, bell: BellProfile, cfg: AnalysisConfig,
                  forward0: np.ndarray | None = None,
                  motion0: StretchMotion | None = None) -> Motion:
    """Orientation, COM acceleration, anchors, velocity / position, ``refine_rests``, velocity /
    position again on the refined anchors, and power (the chain of ``analyse_run``; `forward0`
    and `motion0` as in ``detect_anchors_and_forwards``)."""
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    raw, stretches = detect_anchors_and_forwards(imu, q, kin, calib, cfg.anchors, forward0,
                                                 motion0)
    v = segment_velocity(kin.a_com, imu.t, raw)
    pos = segment_position(v, imu.t, raw)
    anchors = refine_rests(raw, pos, cfg.anchors)
    v = segment_velocity(kin.a_com, imu.t, anchors)
    pos = segment_position(v, imu.t, anchors)
    return Motion(q, bias, kin, raw, stretches, anchors, v, pos,
                  power(kin.a_com, v, bell.mass_kg))


def map_markers(run: Run, markers: list[tuple[float, str]]) -> list[tuple[int, str]]:
    """(sample_index, text) per marker (see the module docstring); [] without packet times."""
    pt = np.asarray(run.packet_times, dtype=float).reshape(-1, 2)
    if len(pt) == 0:
        return []
    out = []
    for ms, text in markers:
        k = max(0, int(np.searchsorted(pt[:, 0], ms, side="right")) - 1)
        pkt_ms, first = pt[k]
        out.append((int(round(first + (ms - pkt_ms) / 1000.0 * run.rate_hz)), text))
    return out


def _apply_labels(reps: list[Rep], labels: dict) -> None:
    for key, lab in labels.items():
        i = int(key)
        if not 0 <= i < len(reps):
            continue  # a label for a rep this analysis did not find
        r = reps[i]
        kind = lab.get("kind", r.kind)
        if kind not in KINDS:
            raise ValueError(f"label for rep {i}: kind {kind!r} not in {KINDS}")
        if "hand" in lab:
            hand = lab["hand"]
        elif kind == "snatch":
            hand = r.hand if r.kind == "snatch" else "?"
        else:
            hand = "-"
        allowed = SNATCH_HANDS if kind == "snatch" else ("-",)
        if hand not in allowed:
            raise ValueError(f"label for rep {i}: hand {hand!r} invalid for kind {kind!r} "
                             f"(allowed: {allowed})")
        r.kind, r.hand = kind, hand
        r.flags.add("relabelled")


def analyse_run(run: Run, bell: BellProfile = BellProfile(),
                cfg: AnalysisConfig = AnalysisConfig(), labels: dict | None = None,
                prior_calib: Calibration | None = None) -> SessionResult:
    """Analyse one run (every stage of spec §5–7).

    Calibration: from the run's first still window. `prior_calib` is the calibration of a
    previous run of the session (a reconnect): its mounting (up_s, out_s, lever_arm_s) is kept
    and only gyro bias and q0 come from the run's still window (``with_mounting``); without a
    still window, its gyro bias and |g| too, with q0 levelled on the first static samples
    (``seeded_from_prior``; prior's q0 at sample 0 if there are none) and a warning. Raises
    ``CalibrationError`` when the run has no still window and no `prior_calib` is given. The
    orientation is integrated backwards over the samples before that reference (reps before
    the first rest are analysed); beyond 1 s a warning says how long they last."""
    imu = scale_run(run)
    calib, imu, warnings = calibrate_run(imu, bell, cfg.calibration, prior_calib)
    if not run.rate_estimated:
        warnings.append(WARN_NOMINAL_RATE)

    m = motion_stages(imu, calib, bell, cfg)
    q, v, pos, anchors = m.q, m.v, m.pos, m.anchors

    sets = build_sets(anchors, imu, q, calib, v, pos, m.P, cfg.classifier)
    reps = [r for s in sets for r in s.reps]
    if labels:
        _apply_labels(reps, labels)

    bounds = segment_sets(anchors)  # one per run of non-rest anchors, as set_forwards
    hints = set_forwards(q, calib, anchors, cfg.anchors, imu, m.kin)
    for s in sets:
        hint = hints[bounds.index((s.i_start, s.i_end))]
        fwd = athlete_forward(v, slice(s.i_start, s.i_end + 1), hint)
        for r in s.reps:
            r.path = rep_path(pos, (r.i_low, r.i_top, r.i_top_end, r.i_end), fwd, fs=imu.fs)

    duration = float(imu.t[-1] - imu.t[0]) if len(imu.t) else 0.0
    return SessionResult(
        calib=calib, sets=sets,
        metrics=session_metrics(sets, duration, cfg.power_floor_w),
        duration_s=duration, warnings=warnings, algorithm_version=ALGORITHM_VERSION,
        set_metrics=[set_metrics(s, cfg.power_floor_w) for s in sets], t=imu.t,
    )
