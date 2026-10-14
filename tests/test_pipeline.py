import dataclasses
import json
import struct

import numpy as np
import pytest

from powerkettle.__main__ import main
from powerkettle.bell import BellProfile
from powerkettle.calibration import CalibrationError
from powerkettle.pipeline import (
    ALGORITHM_VERSION,
    AnalysisConfig,
    SessionResult,
    analyse_run,
    map_markers,
)
from powerkettle.rawlog import RawRecord, Run, write_rawlog
from powerkettle.session_io import load_session, save_session
from powerkettle.simulate import Rest, SimConfig, Snatch, Swing, simulate

SESSION = [Rest(3), Snatch(8, "L"), Rest(6), Swing(10), Rest(6), Snatch(8, "R"), Rest(3)]
SAT_REP = 5
GAP_LEN = 40


def reps_of(res: SessionResult):
    return [r for s in res.sets for r in s.reps]


def with_gap(run: Run, i0: int, n: int) -> Run:
    """`run` with samples [i0, i0 + n) dropped (as a BLE gap)."""
    keep = np.ones(len(run.sample_index), dtype=bool)
    keep[i0:i0 + n] = False
    return dataclasses.replace(run, sample_index=run.sample_index[keep], raw=run.raw[keep],
                               gaps=[(i0 - 1, i0 + n)])


@pytest.fixture(scope="module")
def sim():
    return simulate(SESSION)


@pytest.fixture(scope="module")
def clean(sim):
    return analyse_run(sim[0])


# ----------------------------------------------------------------------------- end to end
def test_rep_count_and_kinds_match_truth(sim, clean):
    _, truth = sim
    reps = reps_of(clean)
    assert [r.kind for r in reps] == [t.kind for t in truth.reps]
    assert [r.hand for r in reps] == [t.hand for t in truth.reps]
    assert len(clean.sets) == 3
    assert clean.algorithm_version == ALGORITHM_VERSION == "1"
    assert all(r.flags == set() for r in reps)


def test_session_snatch_power_within_5_percent(sim, clean):
    _, truth = sim
    true_avg = np.mean([t.power_w for t in truth.reps if t.kind == "snatch"])
    st = clean.metrics.by_kind["snatch"]
    assert st.n == 16
    assert st.avg == pytest.approx(true_avg, rel=0.05)


def test_paths_filled_in_athlete_frame(clean):
    for r in reps_of(clean):
        assert r.path is not None and r.path.ndim == 2 and r.path.shape[1] == 3
        assert np.allclose(r.path[0], 0.0)
        assert np.nanmax(r.path[:, 2]) == pytest.approx(r.apex_m, abs=0.02)
    snatch = reps_of(clean)[0]  # snatch: forward then back up to the lockout above the low
    assert np.nanmax(snatch.path[:, 0]) > 0.2


def test_duration_and_set_metrics(sim, clean):
    run, _ = sim
    assert clean.duration_s == pytest.approx((len(run.sample_index) - 1) / run.rate_hz)
    assert len(clean.set_metrics) == len(clean.sets)
    assert [m.reps_by_kind_hand for m in clean.set_metrics] == [
        {"snatch:L": 8}, {"swing:-": 10}, {"snatch:R": 8}]


def test_power_floor_from_config(sim, clean):
    res = analyse_run(sim[0], cfg=AnalysisConfig(power_floor_w=1e6))
    assert res.metrics.qualifying == 0 and clean.metrics.qualifying == 26


# ----------------------------------------------------------------------------- flags and labels
@pytest.mark.parametrize("gap_rep", [4, 12])  # a snatch, a swing (mid-set)
def test_gap_inside_a_rep(sim, clean, gap_rep):
    """A 40-sample gap mid-concentric: that rep is flagged "gap" and left out of the power
    statistics; every other rep keeps its kind and hand (truth), no flag, and its power (within
    0.01 %: velocity is integrated per segment and orientation bridges the short gap)."""
    run, truth = sim
    tr = truth.reps[gap_rep]
    res = analyse_run(with_gap(run, (tr.i_low + tr.i_top) // 2, GAP_LEN))
    reps, ref = reps_of(res), reps_of(clean)
    assert len(reps) == len(ref) == len(truth.reps)
    assert "gap" in reps[gap_rep].flags
    for i, (r, c, t) in enumerate(zip(reps, ref, truth.reps)):
        if i != gap_rep:
            assert (r.kind, r.hand, r.flags) == (t.kind, t.hand, set()), i
            assert r.power_w == pytest.approx(c.power_w, rel=1e-4), i
    others = [c.power_w for i, c in enumerate(ref) if c.kind == tr.kind and i != gap_rep]
    assert res.metrics.by_kind[tr.kind].n == len(others)
    assert res.metrics.by_kind[tr.kind].avg == pytest.approx(np.mean(others), rel=1e-4)


def test_saturated_sample_flagged_and_counted(sim, clean):
    run, truth = sim
    tr = truth.reps[SAT_REP]
    raw = run.raw.copy()
    raw[(tr.i_low + tr.i_top) // 2, 0] = np.iinfo(np.int16).max
    res = analyse_run(dataclasses.replace(run, raw=raw))
    reps = reps_of(res)
    assert reps[SAT_REP].flags == {"saturated"}
    assert all(r.flags == set() for i, r in enumerate(reps) if i != SAT_REP)
    assert res.metrics.by_kind["snatch"].n == 16


def test_labels_override_classifier(sim, clean):
    res = analyse_run(sim[0], labels={3: {"kind": "other"}, 20: {"hand": "L"},
                                      9: {"kind": "snatch", "hand": "R"}})
    reps = reps_of(res)
    assert (reps[3].kind, reps[3].hand) == ("other", "-")
    assert "relabelled" in reps[3].flags
    assert (reps[20].kind, reps[20].hand) == ("snatch", "L") and "relabelled" in reps[20].flags
    assert (reps[9].kind, reps[9].hand) == ("snatch", "R")
    assert sum("relabelled" in r.flags for r in reps) == 3
    assert res.metrics.by_kind["snatch"].n == 16  # −1 (rep 3) +1 (rep 9)
    assert res.metrics.reps_by_kind_hand["other:-"] == 1
    assert res.metrics.reps_by_kind_hand["snatch:L"] == 8  # 7 left + rep 20
    with pytest.raises(ValueError, match="kind"):
        analyse_run(sim[0], labels={0: {"kind": "clean"}})


# ----------------------------------------------------------------------------- summary
def test_summary_is_json_safe(clean):
    s = clean.summary()
    assert json.loads(json.dumps(s, allow_nan=False)) == s
    assert s["algorithm_version"] == "1"
    assert len(s["reps"]) == 26 and len(s["sets"]) == 3
    rep = s["reps"][0]
    assert {"index", "set", "t_low", "kind", "hand", "power_w", "peak_power_w",
            "peak_velocity_ms", "apex_m", "flags", "features"} <= rep.keys()
    assert "path" not in rep
    assert s["metrics"]["by_kind"]["snatch"]["n"] == 16


def test_summary_nan_becomes_none(clean):
    res = dataclasses.replace(clean, sets=[dataclasses.replace(st) for st in clean.sets])
    rep = dataclasses.replace(res.sets[0].reps[0], peak_power_w=float("nan"),
                              flags={"gap", "low_conf"})
    res.sets[0].reps = [rep] + res.sets[0].reps[1:]
    s = res.summary()
    assert s["reps"][0]["peak_power_w"] is None
    assert s["reps"][0]["flags"] == ["gap", "low_conf"]
    json.dumps(s, allow_nan=False)


# ----------------------------------------------------------------------------- warnings / calibration
def test_nominal_rate_warning(sim, clean):
    assert not any("nominal" in w for w in clean.warnings)
    run = dataclasses.replace(sim[0], rate_estimated=False)
    res = analyse_run(run)
    assert "sample rate nominal (no status data)" in res.warnings


def test_prior_calibration_carried_over(clean):
    run, _ = simulate([Rest(0.5), Swing(5), Rest(1.0)], cfg=SimConfig(seed=3))
    with pytest.raises(CalibrationError):
        analyse_run(run)
    res = analyse_run(run, prior_calib=clean.calib)
    assert "calibration carried over from previous run" in res.warnings
    assert res.calib.gyro_bias == pytest.approx(clean.calib.gyro_bias)


# ----------------------------------------------------------------------------- markers
def test_map_markers(sim):
    run, _ = sim
    ms0, i0 = run.packet_times[3]
    ms1, i1 = run.packet_times[4]
    mid = ms0 + 0.25 * (ms1 - ms0)
    got = map_markers(run, [(ms0, "a"), (mid, "b"), (ms0 - 10_000.0, "early")])
    assert got[0] == (int(i0), "a")
    assert got[1] == (int(round(i0 + (mid - ms0) / 1000.0 * run.rate_hz)), "b")
    # before the first packet: extrapolated from the first packet
    first_ms, first_i = run.packet_times[0]
    assert got[2] == (int(round(first_i + (ms0 - 10_000.0 - first_ms) / 1000.0 * run.rate_hz)),
                      "early")
    assert map_markers(dataclasses.replace(run, packet_times=np.zeros((0, 2))), [(1.0, "x")]) == []


# ----------------------------------------------------------------------------- CLI
def run_records(run: Run, offset_ms: float = 0.0) -> list[RawRecord]:
    """A simulated run as .pkraw records: statuses and 19-sample IMU packets at its packet
    times (+ `offset_ms`)."""
    recs = []
    t0 = run.packet_times[0, 0] + offset_ms
    for st in run.statuses:  # status time on the phone clock: sample index / rate
        recs.append(RawRecord(2, t0 + 1000.0 * st.sample_index / run.rate_hz, struct.pack(
            "<BBHHBBHhIIQ", st.version, st.flags, st.battery_mv, st.odr_hz, st.acc_range_g, 0,
            st.gyro_range_dps, st.temp_raw, st.dropped_samples, st.sample_index, st.time_us)))
    for k, (ms, first) in enumerate(run.packet_times):
        first = int(first)
        samples = run.raw[first:first + 19]
        recs.append(RawRecord(1, float(ms) + offset_ms,
                              struct.pack("<HIBB", k & 0xFFFF, first, len(samples), 0)
                              + samples.astype("<i2").tobytes()))
    return recs


def to_pkraw(path, run: Run, markers=()):
    """Write a simulated run and phone-time markers as a .pkraw."""
    recs = run_records(run) + [RawRecord(3, ms, text.encode()) for ms, text in markers]
    recs.sort(key=lambda r: r.phone_ms)
    write_rawlog(path, {"app": "powerkettle-record", "v": 1}, recs)


def test_cli_analyse_pkraw(sim, clean, tmp_path, capsys):
    run, truth = sim
    t_marker = run.packet_times[0, 0] + 1000.0 * (truth.sets[1][0] - 400) / run.rate_hz
    to_pkraw(tmp_path / "s.pkraw", run, [(run.packet_times[0, 0], "snatch L"),
                                         (t_marker, "swings")])
    assert main(["analyse", str(tmp_path / "s.pkraw"), "--out", str(tmp_path / "s.npz")]) == 0
    out = capsys.readouterr().out
    set_lines = [ln for ln in out.splitlines() if ln.startswith("set ")]
    assert len(set_lines) == 3
    assert "snatch L" in set_lines[0] and "swings" in set_lines[1] and "swings" in set_lines[2]
    rep_lines = [ln for ln in out.splitlines() if ln.split()[:1] and ln.split()[0].isdigit()]
    assert len(rep_lines) == 26
    assert rep_lines[0].split()[2:4] == ["snatch", "L"]
    run2, bell, meta = load_session(tmp_path / "s.npz")
    assert np.array_equal(run2.raw, run.raw) and bell == BellProfile()
    assert [m["text"] for m in meta["markers"]] == ["snatch L", "swings"]
    assert meta["app_version"] == "powerkettle-record/1"
    assert meta["summary"]["metrics"]["by_kind"]["snatch"]["n"] == 16


def test_cli_analyse_npz_with_bell_and_labels(sim, tmp_path, capsys):
    run, _ = sim
    save_session(tmp_path / "s.npz", run, BellProfile(), {"labels": {3: {"kind": "other"}}})
    (tmp_path / "bell.json").write_text(json.dumps({"mass_kg": 24.0}))
    assert main(["analyse", str(tmp_path / "s.npz"), "--bell", str(tmp_path / "bell.json")]) == 0
    out = capsys.readouterr().out
    rep3 = [ln for ln in out.splitlines() if ln.split()[:1] == ["3"]][0]
    assert "other" in rep3 and "relabelled" in rep3
    assert main(["analyse", str(tmp_path / "missing.npz")]) == 1


def test_cli_multi_run_carries_calibration(sim, tmp_path, capsys):
    """A reconnect restarts sample indices (a second run); the second run has no still window
    and reuses the first run's calibration. Each run is saved to its own .npz."""
    run, _ = sim
    run2, _ = simulate([Rest(0.5), Swing(5), Rest(1.0)], cfg=SimConfig(seed=3))
    recs = run_records(run) + run_records(run2, offset_ms=120_000.0)
    recs.sort(key=lambda r: r.phone_ms)
    write_rawlog(tmp_path / "m.pkraw", {"app": "powerkettle-record", "v": 1}, recs)
    assert main(["analyse", str(tmp_path / "m.pkraw"), "--out", str(tmp_path / "m.npz")]) == 0
    out = capsys.readouterr().out
    assert "run 0:" in out and "run 1:" in out
    assert out.count("calibration carried over from previous run") == 1
    assert out.index("calibration carried over") > out.index("run 1:")
    r0, _, _ = load_session(tmp_path / "m.npz")
    r1, _, m1 = load_session(tmp_path / "m.run1.npz")
    assert np.array_equal(r0.raw, run.raw) and np.array_equal(r1.raw, run2.raw)
    assert "calibration carried over from previous run" in m1["summary"]["warnings"]
