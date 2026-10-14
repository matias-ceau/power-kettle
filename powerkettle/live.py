"""Incremental analysis while recording (on the phone, in Pyodide): ``LiveAnalyser``.

``feed`` takes the samples of each BLE packet and returns the events that became certain;
``finish`` returns the batch result (``pipeline.analyse_run`` on everything fed). That result is
authoritative (it is what gets stored); live events are a preview of it.

Approach
--------
All samples are kept, scaled as they arrive. Until the first still window is found
(``calibration.find_still_window``, searched incrementally) nothing else happens; from then
on the accelerometer is also corrected by g / |g measured| (as ``analyse_run`` does). Then, whenever
at least ``PROCESS_EVERY_S`` of new data has arrived, the stages from orientation to power
(``pipeline.motion_stages``) are re-run on a window from a *checkpoint* to the newest sample,
never on the whole session. The window's orientation starts from the orientation, working gyro
bias and forward carried over from the run that set the checkpoint, so within the window the
estimate is the batch one. A checkpoint is final data only, the latest of:

- ``CONTEXT_S`` before the low of the pending rep (before the newest sample when no rep is
  pending), moved back to a sample outside every static window (so no gyro-bias window and no
  static correction straddles it);
- the last sample of a floor rest the bell has left (its static window has ended);
- with no rep pending, ``MAX_WINDOW_S`` before the newest sample when that lies inside a
  hand-held static window (a long overhead hold, which never becomes a rest): the window then
  starts inside the hold, carrying the working bias in force there, and re-estimates the bias
  from the part of the hold it holds rather than from the whole hold as the batch does (a
  residual difference of the order of gyro noise / √samples, ~1e-4 rad/s in simulation).

Every checkpoint is a valid sample with no invalid sample within ``GAP_INTERP_MAX_S`` plus the
gap guards (+2) on either side: a window starting in or next to a gap could not bridge the gap
(``orientation._bridge_gaps`` needs a valid sample before it) and would hold the orientation
still across it, an error every later window would inherit. Without such a sample the window
keeps its start.

While the bell rests on the floor after a confirmed rest and every new sample is static, nothing
can change and the stages are not re-run; the first tick after the bell leaves a rest re-runs
them over the whole rest once (a one-off cost proportional to the rest's length).

Events (JSON-safe dicts, in this order)
---------------------------------------
- ``{"type": "calibration", "ok": True, "warnings": [...]}`` once, when the first still window
  is found (``ok`` False, with the reason as the only warning, if that window cannot calibrate,
  e.g. |f| far off g; nothing else follows then and ``finish`` raises like ``analyse_run``).
  Without a still window there is no event; ``finish`` raises ``CalibrationError`` (or, with
  `prior_calib`, analyses the run with it, as ``analyse_run``). The samples before the window
  are analysed too (orientation integrated backwards from it): their reps are reported at the
  first ticks after the event, which then carries the warning saying how long they last. With
  `prior_calib` (a reconnect), its mounting is kept, as in ``analyse_run``.
- ``{"type": "set_start", "set": k, "t": s}`` just before the first rep of set k (0-based, as
  ``SessionResult.sets``); ``t`` (s since the first sample) is when the bell left the floor. It
  is not sent when the bell leaves the floor, since a set without reps (picked up and put down)
  is no set in the batch result and would shift the numbering.
- ``{"type": "rep", "set": k, "rep": i, "kind", "hand", "power_w", "peak_power_w",
  "peak_velocity_ms", "apex_m", "flags", "path"}``: rep i (0-based within the set), sent once,
  as soon as its top is confirmed. ``path`` is the side view [[x, z], ...] (m, athlete frame,
  ``paths.rep_path``) from the low to the end of the top known so far, 3 decimals.
- ``{"type": "set_end", "set": k, "t": s, "metrics": {...}}`` when the floor rest after the set
  is confirmed (``rest_min_s`` still without tremor and below ``floor_max_height_m``); ``t`` is
  the set's last sample, ``metrics`` the ``metrics.SetMetrics`` of its live reps.
- ``{"type": "warning", "message": ...}`` for each gap in the sample indices.

Confirming a top
----------------
A rep is low → top (→ next low in the batch result). Lows are taken from ``detect_anchors`` on
the window (a low is reported there only once θ has fallen ``prominence_deg`` after it, and
stays one as data arrives). After the pending low, the top is confirmed:

- lockout: a static window of at least ``static_min_s`` (not a confirmed rest) whose height above
  the low is at least ``floor_max_height_m`` (the bell set down on the floor after the final
  backswing is not a top), once it has ended or lasted ``LOCKOUT_CONFIRM_S`` (so a lockout
  that is still going on is not cut below the classifier's minimum static time);
- swing top: θ has risen ``prominence_deg`` above its minimum since the low *and* the bell is
  falling (vertical velocity integrated from the low below ``-DESCENT_MIN_MS``): on the way into
  a snatch lockout θ also rises that much above the mid-pull minimum, but the bell still rises;
- otherwise, late, when the window's anchors hold a top after the low (the next low has come).

The rep is then measured as in ``reps.build_sets`` with that top added to the window's anchors.

Known differences from the batch result
---------------------------------------
- A rep's ``flags`` cover [low, end of the top so far]; the batch rep spans to the next low.
- After a gap too long to bridge, a rep whose orientation is still suspect waits while more
  anchors may correct the tilt (``pipeline.motion_stages``): typically until the next low
  (~0.6 s later than usual); the correction uses the anchors known by then, the batch all
  of them up to the next static window.
- A top with no following backswing (bell put down from the top) is a live rep, not a batch one.
- The lockout window may still grow after the rep is sent (``top_static_s``, ``top_tilt_deg``
  and the path cover the part known then); the path's forward comes from the set's data so far.
- Set metrics use the live reps (no eccentric work or cycle time: those need the next low).
- A session that ends mid-set (no confirmed floor rest after the last rep) gets no live
  ``set_end``; ``finish`` still reports that set.
- A quiet overhead hold (no tremor, a rest relabelled as a lockout by ``refine_rests``) is not
  cut to ``MAX_WINDOW_S``: the window and the cost of each tick grow with its length. Nor is a
  static hold after a low whose top is not confirmed (below ``floor_max_height_m``).

Samples must arrive in order. Rows already received (a duplicate or overlapping packet, e.g. a
BLE retransmit) are dropped; a packet whose first index lies more than the firmware ring
(``rawlog.RING_TOLERANCE_SAMPLES``) before the previous packet's is a new run (the sensor
restarted) and raises ValueError: use a new ``LiveAnalyser`` per run. Stdlib + numpy only.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .anchors import Anchor, StretchMotion, _static_windows, motion_sums, sagittal_tilt
from .bell import BellProfile
from .calibration import (
    SMOOTH_SECONDS,
    Calibration,
    CalibrationError,
    calibrate,
    find_still_window,
    with_mounting,
)
from .classify import classify, rep_features
from .imu import GAP_GUARD_SAMPLES, G, ImuData, scale_run
from .integrate import power, segment_position, segment_velocity
from .metrics import set_metrics
from .orientation import GAP_INTERP_MAX_S, static_mask
from .paths import athlete_forward, rep_path
from .pipeline import (
    AnalysisConfig,
    Motion,
    SessionResult,
    _jsonable,
    analyse_run,
    motion_stages,
    pre_calibration_warning,
)
from .rawlog import RING_TOLERANCE_SAMPLES, Run
from .reps import Rep, SetResult, rep_metrics

PROCESS_EVERY_S = 0.25  # re-run the stages once this much new data has arrived
CONTEXT_S = 2.0  # the window starts this long before the pending low (prominence, filters)
MAX_WINDOW_S = 6.0  # a window inside a long hand-held hold is cut to this length
DESCENT_MIN_MS = 0.5  # m/s: falling speed that confirms a swing top
LOCKOUT_CONFIRM_S = 0.15  # a lockout still going on confirms a top once it lasts this long
_TOPS = ("top_swing", "top_lockout")
_INITIAL_CAPACITY = 4096


@dataclass
class _Carry:
    """State at a checkpoint (the window's first sample)."""
    q: np.ndarray  # (4,) world <- sensor
    bias: np.ndarray  # (3,) working gyro bias, rad/s
    forward: np.ndarray  # (3,) rest-based (fallback) forward in force
    motion: StretchMotion | None  # sums over the stretch before the checkpoint (None: it starts)
    suspect: bool  # orientation suspect there (after a gap it could not correct)


class LiveAnalyser:
    """Incremental analysis of one run (see the module docstring). `rate_hz`, `acc_range_g`
    and `gyro_range_dps` are those of the run; `rate_estimated` is passed to the ``Run`` that
    ``finish`` analyses (False adds the nominal-rate warning)."""

    def __init__(self, bell: BellProfile, cfg: AnalysisConfig, rate_hz: float, acc_range_g: int,
                 gyro_range_dps: int, rate_estimated: bool = False,
                 prior_calib: Calibration | None = None) -> None:
        self.bell, self.cfg = bell, cfg
        self.prior_calib = prior_calib
        self.rate_hz = float(rate_hz)
        self.acc_range_g, self.gyro_range_dps = acc_range_g, gyro_range_dps
        self.rate_estimated = rate_estimated
        self._first: int | None = None  # sensor sample_index of row 0
        self._last_first = 0  # highest first_index fed
        self._n = 0  # rows received (gaps included)
        self._cap = 0
        self._raw = np.zeros((0, 6), np.int16)
        self._present = np.zeros(0, bool)
        self._acc = np.zeros((0, 3))
        self._gyro = np.zeros((0, 3))
        self._valid = np.zeros(0, bool)
        self._sat = np.zeros(0, bool)
        self._gaps: list[tuple[int, int]] = []
        self._tick_every = max(1, math.ceil(PROCESS_EVERY_S * self.rate_hz))
        self._last_tick = 0  # rows at the last tick
        # calibration
        self._calib: Calibration | None = None
        self._acc_k = 1.0  # accelerometer scale correction g / g_measured, once calibrated
        self._failed = False
        self._cal_from = 0  # first row a still window not yet rejected may start at
        # window and event state (global row indices)
        self._w0 = 0
        self._carry: _Carry | None = None  # None: the window starts at row 0 (real calibration)
        self._resting = False
        self._after = -1  # anchors ending at or before this row are dealt with
        self._low: int | None = None  # low of the pending rep
        self._last_low: int | None = None  # low of the last reported rep
        self._rest_end: int | None = None  # last row of the latest floor rest passed
        self._set_start: int | None = None  # first row of the coming / current set
        self._in_set = False
        self._n_sets = 0
        self._set_reps: list[Rep] = []

    # ------------------------------------------------------------------------- input
    def feed(self, first_index: int, raw: np.ndarray) -> list[dict]:
        """Add consecutive samples (int16 (n, 6): acc xyz, gyro xyz counts) starting at sensor
        sample index `first_index`; returns the new events."""
        raw = np.asarray(raw)
        if raw.ndim != 2 or raw.shape[1] != 6:
            raise ValueError(f"raw must have shape (n, 6), got {raw.shape}")
        if len(raw) == 0:
            return []
        events: list[dict] = []
        first_index = int(first_index)
        if self._first is None:
            self._first = self._last_first = first_index
        if first_index < self._last_first - RING_TOLERANCE_SAMPLES:
            raise ValueError(f"sample index went backwards ({first_index} after "
                             f"{self._first + self._n - 1}): start a new LiveAnalyser per run")
        self._last_first = max(self._last_first, first_index)
        row0 = first_index - self._first
        if row0 < self._n:  # a duplicate or overlapping packet: keep the rows not seen yet
            raw = raw[self._n - row0:]
            if len(raw) == 0:
                return []
            first_index, row0 = self._first + self._n, self._n
        if row0 > self._n:
            self._gaps.append((self._first + self._n - 1, first_index))
            events.append({"type": "warning", "message": (
                f"data gap: {row0 - self._n} samples missing "
                f"at t = {self._n / self.rate_hz:.2f} s")})
        rows = row0 + len(raw)
        self._grow(rows)
        imu = scale_run(Run(np.arange(first_index, first_index + len(raw), dtype=np.int64),
                            raw.astype(np.int16), [], [], self.rate_hz, self.acc_range_g,
                            self.gyro_range_dps))
        sl = slice(row0, rows)
        self._raw[sl], self._present[sl] = raw, True
        self._acc[sl], self._gyro[sl] = imu.acc * self._acc_k, imu.gyro
        self._valid[sl], self._sat[sl] = True, imu.saturated
        self._n = rows
        self._apply_gap_guards(row0)
        if self._n - self._last_tick >= self._tick_every:
            events += self._tick()
            self._last_tick = self._n
        return events

    def finish(self) -> SessionResult:
        """``analyse_run`` on every sample fed (raises like it)."""
        if self._first is None:
            raise ValueError("no samples fed")
        n = self._n
        rows = np.flatnonzero(self._present[:n])
        run = Run(self._first + rows.astype(np.int64), self._raw[:n][rows], list(self._gaps), [],
                  self.rate_hz, self.acc_range_g, self.gyro_range_dps,
                  rate_estimated=self.rate_estimated)
        return analyse_run(run, self.bell, self.cfg, prior_calib=self.prior_calib)

    def _grow(self, rows: int) -> None:
        if rows <= self._cap:
            return
        cap = max(rows, 2 * self._cap, _INITIAL_CAPACITY)
        n = self._n

        def grown(a, fill):
            out = np.full((cap,) + a.shape[1:], fill, dtype=a.dtype)
            out[:n] = a[:n]
            return out
        self._raw, self._present = grown(self._raw, 0), grown(self._present, False)
        self._acc, self._gyro = grown(self._acc, np.nan), grown(self._gyro, np.nan)
        self._valid, self._sat = grown(self._valid, False), grown(self._sat, False)
        self._cap = cap

    def _apply_gap_guards(self, row0: int) -> None:
        """Invalidate the samples next to each gap, as ``scale_run`` does (also those of a gap
        whose guard reaches into the samples just added)."""
        for last_before, first_after in reversed(self._gaps):
            lo = max(0, last_before - self._first - (GAP_GUARD_SAMPLES - 1))
            hi = min(self._n, first_after - self._first + GAP_GUARD_SAMPLES)
            if hi <= row0 - GAP_GUARD_SAMPLES:
                break
            self._valid[lo:hi] = False

    def _imu(self, lo: int, hi: int) -> ImuData:
        """Rows [lo, hi) as ImuData (views; t on the session clock, as in ``scale_run``)."""
        return ImuData(np.arange(lo, hi, dtype=np.float64) / self.rate_hz, self._acc[lo:hi],
                       self._gyro[lo:hi], self._valid[lo:hi], self._sat[lo:hi], self.rate_hz,
                       self._first + lo)

    # ------------------------------------------------------------------------- processing
    def _tick(self) -> list[dict]:
        if self._failed:
            return []
        events: list[dict] = []
        if self._calib is None:
            events += self._try_calibrate()
            if self._calib is None:
                return events
        elif self._resting and np.all(static_mask(self._imu(self._last_tick, self._n))):
            return events  # still resting on the floor: nothing can change
        return events + self._process()

    def _try_calibrate(self) -> list[dict]:
        ccfg = self.cfg.calibration
        try:
            found = find_still_window(self._imu(self._cal_from, self._n), ccfg)
        except CalibrationError:
            # a window starting this early would have been complete and was rejected
            need = int(math.ceil(ccfg.still_seconds * self.rate_hz - 1e-9))
            k = max(1, int(round(SMOOTH_SECONDS * self.rate_hz)))
            self._cal_from = max(self._cal_from, self._n - need - k)
            return []
        win = slice(self._cal_from + found.start, self._cal_from + found.stop)
        try:
            self._calib = calibrate(self._imu(0, self._n), self.bell, ccfg, window=win)
        except CalibrationError as e:
            self._failed = True
            return [{"type": "calibration", "ok": False, "warnings": [str(e)]}]
        if self.prior_calib is not None:
            self._calib = with_mounting(self._calib, self.prior_calib)
        self._acc_k = G / self._calib.g_measured  # as pipeline.analyse_run (correct_acc_scale)
        self._acc[:self._n] *= self._acc_k
        warnings = self._calib.warnings + pre_calibration_warning(self._calib, self.rate_hz)
        return [{"type": "calibration", "ok": True, "warnings": warnings}]

    def _process(self) -> list[dict]:
        w0, n = self._w0, self._n
        imu = self._imu(w0, n)
        if self._carry is None:
            calib, fwd0, motion0, suspect0 = self._calib, None, None, False
        else:
            calib = dataclasses.replace(self._calib, q0=self._carry.q,
                                        gyro_bias=self._carry.bias, still=slice(0, 1))
            fwd0, motion0, suspect0 = self._carry.forward, self._carry.motion, self._carry.suspect
        m = motion_stages(imu, calib, self.bell, self.cfg, fwd0, motion0, suspect0)

        def stretch_at(i: int):  # local row -> its stretch (in a rest: the stretch after it)
            return next((st for st in m.stretches if i < st.s1), m.stretches[-1])

        win = _Window(w0, imu, calib, m, lambda i: stretch_at(i).forward, self.bell.mass_kg)
        anchors = [Anchor(a.i0 + w0, a.i1 + w0, a.kind) for a in m.anchors]
        events = self._advance(win, anchors)

        last = anchors[-1] if anchors else None
        self._resting = (self._low is None and last is not None and last.kind == "rest"
                         and last.i1 == n - 1)
        self._move_checkpoint(win)
        return events

    def _advance(self, win: "_Window", anchors: list[Anchor]) -> list[dict]:
        """Walk the anchors past those dealt with, emitting events (see the module docstring)."""
        events: list[dict] = []
        while True:
            if self._low is None:
                nxt = next((a for a in anchors if a.i1 > self._after), None)
                if nxt is None:
                    break
                self._after = nxt.i1
                if nxt.kind == "rest":
                    if self._in_set:
                        events.append(self._end_set(nxt))
                    self._rest_end, self._set_start = nxt.i1, nxt.i1 + 1
                elif nxt.kind == "low":
                    self._low = nxt.i0
                    if not self._in_set and self._set_start is None:
                        self._set_start = nxt.i0  # no rest before the set: its first anchor
                continue  # a top whose rep was already reported is skipped
            nxt = next((a for a in anchors if a.i0 > self._low), None)
            if nxt is None:
                found = win.live_top(self._low - win.w0, self.cfg)
                if found is None or win.correction_pending(self._low - win.w0):
                    break
                top, arrays = found
                events += self._emit_rep(win, Anchor(top.i0 + win.w0, top.i1 + win.w0, top.kind),
                                         arrays)
            elif nxt.kind in _TOPS:
                if win.correction_pending(self._low - win.w0):
                    break
                events += self._emit_rep(win, nxt, (win.m.v, win.m.pos, win.m.P))
            else:  # a rest (the final backswing's set-down) or a low: no rep from this low
                self._low = None
        return events

    def _emit_rep(self, win: "_Window", top: Anchor, arrays) -> list[dict]:
        events: list[dict] = []
        if not self._in_set:
            if self._set_start is None:
                self._set_start = self._low
            self._in_set, self._set_reps = True, []
            events.append({"type": "set_start", "set": self._n_sets,
                           "t": self._set_start / self.rate_hz})
        v, pos, P = arrays
        w0, imu = win.w0, win.imu
        seg = (self._low - w0, top.i0 - w0, top.i1 - w0, top.i1 - w0)
        met = rep_metrics(seg, imu.t, v, pos, P, imu.valid, imu.saturated, win.m.suspect)
        feats = rep_features(seg, win.m.q, win.calib, pos, imu.t)
        kind, hand, conf = classify(feats, self.cfg.classifier)
        flags = met.pop("flags")
        if conf < self.cfg.classifier.low_conf or flags & {"orientation", "implausible"}:
            flags.add("low_conf")
        met.update(work_ecc_j=math.nan, cycle_s=math.nan)  # need the next low
        rep = Rep(self._low, top.i0, top.i1, -1, kind=kind, hand=hand, confidence=conf,
                  features=feats, flags=flags, **met)
        fwd = athlete_forward(v, slice(max(0, self._set_start - w0), seg[2] + 1),
                              win.forward_at(seg[0]))
        path = rep_path(pos, seg, fwd, fs=self.rate_hz)[:, [0, 2]]
        events.append(_jsonable({
            "type": "rep", "set": self._n_sets, "rep": len(self._set_reps), "kind": kind,
            "hand": hand, "power_w": rep.power_w, "peak_power_w": rep.peak_power_w,
            "peak_velocity_ms": rep.peak_velocity_ms, "apex_m": rep.apex_m, "flags": flags,
            "path": np.round(path, 3)}))
        self._set_reps.append(rep)
        self._last_low, self._low, self._after = self._low, None, top.i1
        return events

    def _end_set(self, rest: Anchor) -> dict:
        i0, i1 = self._set_start, rest.i0 - 1
        s = SetResult(self._set_reps, i0, i1, i0 / self.rate_hz, i1 / self.rate_hz, 0.0)
        ev = _jsonable({"type": "set_end", "set": self._n_sets, "t": s.t_end,
                        "metrics": set_metrics(s, self.cfg.power_floor_w)})
        self._in_set, self._set_reps, self._n_sets = False, [], self._n_sets + 1
        return ev

    def _move_checkpoint(self, win: "_Window") -> None:
        """Move the window start forward to the latest final checkpoint (module docstring)."""
        w0, n, fs = win.w0, self._n, self.rate_hz
        static = static_mask(win.imu)
        clean = self._clean(w0)
        moving = np.flatnonzero(clean & ~static)
        target = 0  # local row; 0 keeps the window start
        ref = (self._low if self._low is not None else n - 1) - round(CONTEXT_S * fs) - w0
        k = int(np.searchsorted(moving, ref, side="right")) - 1
        if ref > 0 and k >= 0:
            target = int(moving[k])
        end = None if self._rest_end is None else self._rest_end - w0
        if end is not None and 0 < end < n - 1 - w0 and clean[end]:
            target = max(target, end)  # the last sample of a floor rest the bell has left
        c = n - 1 - w0 - round(MAX_WINDOW_S * fs)
        if (self._low is None and c > target and static[c] and clean[c]
                and not any(a.kind == "rest" and a.i0 <= c <= a.i1 for a in win.m.raw_anchors)):
            target = c  # inside a long hand-held static window
        if target > 0:
            st = next((x for x in win.m.stretches if target < x.s1), win.m.stretches[-1])
            motion = None  # the stretch starts at the checkpoint (it ends a rest)
            if st.s0 < target:  # sums over the stretch's part before the checkpoint
                sl = slice(st.s0, target)
                prior = self._carry.motion if st.s0 == 0 and self._carry is not None else None
                motion = motion_sums(win.m.q[sl], win.calib, win.m.kin.omega_w[sl],
                                     win.imu.valid[sl], prior)
                motion.sign_ref = st.motion.sign_ref  # decided on the window's data
            self._carry = _Carry(win.m.q[target].copy(), win.m.bias[target].copy(),
                                 np.array(st.hint, dtype=float), motion,
                                 bool(win.m.suspect[target]))
            self._w0 = w0 + target

    def _clean(self, w0: int) -> np.ndarray:
        """(n - w0,) bool per row of [w0, n): valid, and no invalid sample within ±r rows, r
        covering an orientation gap bridge and its gap guards (a window starting at or next to
        a gap would hold the orientation still across the gap); False where those rows are not
        all received yet."""
        n = self._n
        r = math.ceil(GAP_INTERP_MAX_S * self.rate_hz) + GAP_GUARD_SAMPLES + 2
        lo = max(0, w0 - r)
        bad = np.concatenate([[0], np.cumsum(~self._valid[lo:n])])
        rows = np.arange(w0, n)
        a, b = np.maximum(rows - r, 0) - lo, rows + r + 1 - lo
        return (b <= n - lo) & (bad[np.minimum(b, n - lo)] - bad[a] == 0)


@dataclass
class _Window:
    """One run of the stages on rows [w0, n) (local rows below are relative to w0)."""
    w0: int
    imu: ImuData
    calib: Calibration
    m: Motion
    forward_at: Callable[[int], np.ndarray]  # local row -> its stretch's forward (θ plane)
    mass_kg: float

    def correction_pending(self, low: int) -> bool:
        """The orientation from local row `low` on is suspect (after an unbridged gap) and the
        region is still open (no static window yet): more anchors may yet correct it
        (``pipeline.motion_stages``), so the rep waits."""
        return bool(self.m.suspect_open and np.any(self.m.suspect[low:]))

    def with_top(self, low: int, top: Anchor) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(v, pos, P) with `top` added after the low at local row `low` (also added, should
        the window's own detection have lost it)."""
        lows = [] if any(a.kind == "low" and a.i0 == low for a in self.m.anchors) else [
            Anchor(low, low, "low")]
        anchors = sorted(self.m.anchors + lows + [top])
        v = segment_velocity(self.m.kin.a_com, self.imu.t, anchors)
        pos = segment_position(v, self.imu.t, anchors)
        return v, pos, power(self.m.kin.a_com, v, self.mass_kg)

    def live_top(self, low: int, cfg: AnalysisConfig):
        """(top anchor, (v, pos, P) with it) confirmed after the low at local row `low` (the
        window's last anchor), or None (rules in the module docstring)."""
        acfg = cfg.anchors
        _, held = _static_windows(self.imu, acfg)
        n, grown = len(self.imu.t), math.ceil(LOCKOUT_CONFIRM_S * self.imu.fs - 1e-9)
        for a, b in held:
            if a <= low or (b == n and b - a < grown):  # still growing and short: wait
                continue
            top = Anchor(a, b - 1, "top_lockout")
            arrays = self.with_top(low, top)
            z = arrays[1][:, 2]
            if z[(a + b - 1) // 2] - z[low] >= acfg.floor_max_height_m:  # False for NaN
                return top, arrays
        th = sagittal_tilt(self.m.q[low:], self.calib, self.forward_at(low))
        im = int(np.argmin(th))
        if not np.max(th[im:]) - th[im] >= math.radians(acfg.prominence_deg):
            return None
        az, t = self.m.kin.a_com[low:, 2], self.imu.t[low:]
        vz = float(np.sum(0.5 * (az[1:] + az[:-1]) * np.diff(t)))  # from v = 0 at the low
        if not vz < -DESCENT_MIN_MS:  # False for NaN (a gap): wait for the next low
            return None
        top = Anchor(low + im, low + im, "top_swing")
        return top, self.with_top(low, top)
