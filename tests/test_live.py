import dataclasses
import json
import time

import numpy as np
import pytest

from powerkettle.bell import BellProfile
from powerkettle.calibration import CalibrationError
from powerkettle.live import LiveAnalyser
from powerkettle.pipeline import AnalysisConfig, analyse_run
from powerkettle.rawlog import Run
from powerkettle.simulate import Hold, PartialRep, Rest, SimConfig, Snatch, Swing, simulate

CHUNK = 19  # samples per BLE packet
BLOCK = [Snatch(10, "L"), Rest(25), Swing(15), Rest(25), Snatch(10, "R"), Hold(3), Rest(25)]
LONG = [Rest(3)] + BLOCK * 4 + [Swing(12, cadence_s=1.0), Rest(20), PartialRep(), Rest(10)]
SHORT = [Rest(3), Snatch(5, "L"), Rest(6), Swing(6), Rest(4)]
MAX_LATENCY_S = 0.6
MAX_RUNTIME_S = 30.0


def live_for(run: Run, **kw) -> LiveAnalyser:
    return LiveAnalyser(BellProfile(), AnalysisConfig(), run.rate_hz, run.acc_range_g,
                        run.gyro_range_dps, rate_estimated=run.rate_estimated, **kw)


def feed_all(live: LiveAnalyser, run: Run, chunk: int = CHUNK):
    """Feed `run` in packets of `chunk` consecutive samples (split at gaps); returns
    [(index of the last sample fed when the event came back, event)] and the time spent."""
    events, spent = [], 0.0
    idx = run.sample_index
    breaks = np.flatnonzero(np.diff(idx) != 1) + 1
    for lo_run, hi_run in zip(np.r_[0, breaks], np.r_[breaks, len(idx)]):
        for lo in range(lo_run, hi_run, chunk):
            hi = min(lo + chunk, hi_run)
            t0 = time.perf_counter()
            out = live.feed(int(idx[lo]), run.raw[lo:hi])
            spent += time.perf_counter() - t0
            events += [(int(idx[hi - 1]), e) for e in out]
    return events, spent


def batch_reps(res):
    return [r for s in res.sets for r in s.reps]


def of_type(events, kind):
    return [(i, e) for i, e in events if e["type"] == kind]


@pytest.fixture(scope="module")
def long_session():
    run, truth = simulate(LONG)
    live = live_for(run)
    events, spent = feed_all(live, run)
    return run, truth, live, events, spent, analyse_run(run)


# ----------------------------------------------------------------------------- 10-min session
def test_event_order(long_session):
    """calibration, then per set: set_start, its reps (numbered from 0), set_end."""
    *_, events, _, batch = long_session
    types = [e["type"] for _, e in events]
    assert types[0] == "calibration" and events[0][1]["ok"] is True
    assert "warning" not in types
    k, i, open_set = 0, 0, False
    for _, e in events[1:]:
        if e["type"] == "set_start":
            assert not open_set and e["set"] == k
            open_set, i = True, 0
        elif e["type"] == "rep":
            assert open_set and (e["set"], e["rep"]) == (k, i)
            i += 1
        else:
            assert e["type"] == "set_end" and open_set and e["set"] == k
            open_set, k = False, k + 1
    assert not open_set and k == len(batch.sets)


def test_reps_match_batch(long_session):
    *_, events, _, batch = long_session
    reps, ref = [e for _, e in of_type(events, "rep")], batch_reps(batch)
    assert len(reps) == len(ref) == 153
    assert [e["kind"] for e in reps] == [r.kind for r in ref]
    assert [e["hand"] for e in reps] == [r.hand for r in ref]
    for j, (e, r) in enumerate(zip(reps, ref)):
        assert e["power_w"] == pytest.approx(r.power_w, rel=0.01), j
        assert e["peak_power_w"] == pytest.approx(r.peak_power_w, rel=0.01), j
        assert e["peak_velocity_ms"] == pytest.approx(r.peak_velocity_ms, rel=0.01), j
        assert e["apex_m"] == pytest.approx(r.apex_m, rel=0.01), j


def test_set_starts_and_ends_match_batch(long_session):
    run, _, _, events, _, batch = long_session
    starts = [e for _, e in of_type(events, "set_start")]
    ends = [e for _, e in of_type(events, "set_end")]
    assert [e["t"] for e in starts] == pytest.approx([s.t_start for s in batch.sets], abs=1e-9)
    for e, s, m in zip(ends, batch.sets, batch.set_metrics):
        assert e["metrics"]["reps_by_kind_hand"] == m.reps_by_kind_hand
        assert e["metrics"]["qualifying"] == m.qualifying
        for kind, st in m.by_kind.items():
            assert e["metrics"]["by_kind"][kind]["n"] == st.n
            assert e["metrics"]["by_kind"][kind]["avg"] == pytest.approx(st.avg, rel=0.01)
        assert e["metrics"]["pace_rpm"] == pytest.approx(m.pace_rpm, rel=0.01)


def test_rep_latency(long_session):
    """Each rep is reported at most 0.6 s (simulated time) after its true top."""
    _, truth, _, events, _, _ = long_session
    reps = of_type(events, "rep")
    assert len(reps) == len(truth.reps)
    late = [truth.t[i] - truth.t[tr.i_top] for (i, _), tr in zip(reps, truth.reps)]
    assert max(late) <= MAX_LATENCY_S, max(late)
    assert min(late) > 0.0


def test_runtime_of_ten_minutes(long_session):
    run, truth, *_, spent, _ = long_session
    assert truth.t[-1] > 600.0
    assert spent < MAX_RUNTIME_S


def test_events_json_safe_with_side_view_path(long_session):
    *_, events, _, _ = long_session
    evs = [e for _, e in events]
    assert json.loads(json.dumps(evs, allow_nan=False)) == evs
    rep = next(e for e in evs if e["type"] == "rep")
    assert {"type", "set", "rep", "kind", "hand", "power_w", "peak_power_w", "peak_velocity_ms",
            "apex_m", "path"} <= rep.keys()
    path = np.array(rep["path"], dtype=float)
    assert path.ndim == 2 and path.shape[1] == 2 and len(path) > 10
    assert np.allclose(path[0], 0.0)
    assert np.allclose(path, np.round(path, 3))
    assert np.nanmax(path[:, 1]) == pytest.approx(rep["apex_m"], abs=0.01)  # z is up


def test_finish_is_the_batch_result(long_session):
    *_, live, _, _, batch = long_session
    assert live.finish().summary() == batch.summary()


# ----------------------------------------------------------------------------- edge cases
def test_gap_reported_and_reps_still_match():
    run, truth = simulate(SHORT, cfg=SimConfig(seed=2))
    tr = truth.reps[2]
    i0 = (tr.i_low + tr.i_top) // 2
    keep = np.ones(len(run.sample_index), dtype=bool)
    keep[i0:i0 + 40] = False
    run = dataclasses.replace(run, sample_index=run.sample_index[keep], raw=run.raw[keep],
                              gaps=[(i0 - 1, i0 + 40)])
    live = live_for(run)
    events, _ = feed_all(live, run)
    warns = [e for _, e in of_type(events, "warning")]
    assert len(warns) == 1 and "gap" in warns[0]["message"]
    batch = analyse_run(run)
    reps, ref = [e for _, e in of_type(events, "rep")], batch_reps(batch)
    assert len(reps) == len(truth.reps)
    assert [e["kind"] for e in reps] == [r.kind for r in ref]  # the gap rep: other (apex NaN)
    assert "gap" in reps[2]["flags"] and reps[2]["power_w"] is None
    for j, (e, r) in enumerate(zip(reps, ref)):
        if j != 2:
            assert e["power_w"] == pytest.approx(r.power_w, rel=0.01), j
    assert live.finish().summary() == batch.summary()


def test_no_still_window_waits_then_finish_raises():
    run, _ = simulate([Rest(0.5), Swing(5), Rest(1.0)], cfg=SimConfig(seed=3))
    live = live_for(run)
    events, _ = feed_all(live, run)
    assert events == []
    with pytest.raises(CalibrationError):
        live.finish()


def test_calibration_failure_reported_once():
    """A still window with |f| far off g (wrong accelerometer range): one calibration event
    with ok False and the reason, then nothing more."""
    run, _ = simulate(SHORT)
    live = LiveAnalyser(BellProfile(), AnalysisConfig(), run.rate_hz, 8, run.gyro_range_dps)
    events, _ = feed_all(live, run)
    assert [e["type"] for _, e in events] == ["calibration"]
    assert events[0][1]["ok"] is False and "off g" in events[0][1]["warnings"][0]


def test_calibration_event_carries_warnings():
    run, _ = simulate(SHORT, cfg=SimConfig(acc_bias=(0.2, 0.3, 0.0)))
    events, _ = feed_all(live_for(run), run)
    cal = events[0][1]
    assert cal["type"] == "calibration" and cal["ok"] is True
    assert any("accelerometer magnitude" in w for w in cal["warnings"])


def test_index_going_backwards_rejected():
    run, _ = simulate([Rest(3)])
    live = live_for(run)
    live.feed(0, run.raw[:CHUNK])
    with pytest.raises(ValueError, match="backwards"):
        live.feed(5, run.raw[5:5 + CHUNK])
