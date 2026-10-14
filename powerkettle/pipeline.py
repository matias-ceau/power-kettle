"""End-to-end analysis of one run (spec §5–7): the production chain of every stage.

``analyse_run`` scales the run, calibrates (or carries a previous run's calibration over),
estimates orientation and COM acceleration, finds anchors, integrates velocity / position (twice:
before and after ``refine_rests``), computes power, builds and classifies reps and sets, applies
label corrections, fills each rep's athlete-frame path and computes set and session metrics.

Labels
------
``labels`` maps a rep's session-wide index (0-based, in time order across sets) to
``{"kind": …, "hand": …}`` (either key optional). A label overrides the classifier and adds the
flag "relabelled". A kind other than snatch without a hand gets hand "-"; a snatch without a hand
keeps the classifier's hand if it was already a snatch, else "?". Labels for indices past the
reps found are ignored.

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

from .anchors import AnchorConfig, detect_anchors, refine_rests, set_forwards
from .bell import BellProfile
from .calibration import Calibration, CalibrationConfig, CalibrationError, calibrate
from .classify import ClassifierConfig
from .imu import scale_run
from .integrate import power, segment_position, segment_velocity
from .kinematics import com_acceleration
from .metrics import SessionMetrics, SetMetrics, session_metrics, set_metrics
from .orientation import estimate_orientation
from .paths import athlete_forward, rep_path
from .rawlog import Run
from .reps import Rep, SetResult, build_sets, segment_sets

ALGORITHM_VERSION = "1"
KINDS = ("snatch", "swing", "other")
HANDS = ("L", "R", "?", "-")
WARN_NOMINAL_RATE = "sample rate nominal (no status data)"
WARN_CARRIED_CALIBRATION = "calibration carried over from previous run"


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
        if hand not in HANDS:
            raise ValueError(f"label for rep {i}: hand {hand!r} not in {HANDS}")
        r.kind, r.hand = kind, hand
        r.flags.add("relabelled")


def analyse_run(run: Run, bell: BellProfile = BellProfile(),
                cfg: AnalysisConfig = AnalysisConfig(), labels: dict | None = None,
                prior_calib: Calibration | None = None) -> SessionResult:
    """Analyse one run (every stage of spec §5–7). Raises ``CalibrationError`` when the run has
    no still window and no `prior_calib` is given; with one, it is used instead (its still
    window moved to the run's first sample) and a warning is added."""
    imu = scale_run(run)
    warnings: list[str] = []
    try:
        calib = calibrate(imu, bell, cfg.calibration)
    except CalibrationError:
        if prior_calib is None:
            raise
        calib = dataclasses.replace(prior_calib, still=slice(0, 1), warnings=[])
        warnings.append(WARN_CARRIED_CALIBRATION)
    warnings += calib.warnings
    if not run.rate_estimated:
        warnings.append(WARN_NOMINAL_RATE)

    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    anchors = detect_anchors(imu, q, kin, calib, cfg.anchors)
    v = segment_velocity(kin.a_com, imu.t, anchors)
    pos = segment_position(v, imu.t, anchors)
    anchors = refine_rests(anchors, pos, cfg.anchors)
    v = segment_velocity(kin.a_com, imu.t, anchors)
    pos = segment_position(v, imu.t, anchors)
    P = power(kin.a_com, v, bell.mass_kg)

    sets = build_sets(anchors, imu, q, calib, v, pos, P, cfg.classifier)
    reps = [r for s in sets for r in s.reps]
    if labels:
        _apply_labels(reps, labels)

    bounds = segment_sets(anchors)  # one per run of non-rest anchors, as set_forwards
    hints = set_forwards(q, calib, anchors, cfg.anchors)
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
