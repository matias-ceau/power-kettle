import numpy as np
import pytest

from powerkettle.anchors import Anchor
from powerkettle.classify import ClassifierConfig
from powerkettle.reps import (
    Rep,
    SetResult,
    build_sets,
    moving_average,
    rep_metrics,
    segment_reps,
    segment_sets,
)
from powerkettle.simulate import PartialRep, Rest, Snatch, Swing

from sim_pipeline import run_stages

SESSION = [Rest(3), Snatch(8, "L"), Rest(6), Swing(12), Rest(6), Snatch(8, "R"), Rest(6),
           PartialRep(), Rest(3)]


@pytest.fixture(scope="module")
def session():
    s = run_stages(SESSION)
    s.sets = build_sets(s.anchors, s.imu, s.q, s.calib, s.v, s.pos, s.P)
    return s


# ----------------------------------------------------------------------------- segmentation
ANCHORS = [Anchor(0, 99, "rest"), Anchor(120, 120, "low"), Anchor(200, 239, "top_lockout"),
           Anchor(300, 300, "low"), Anchor(380, 380, "top_swing"), Anchor(450, 450, "low"),
           Anchor(500, 899, "rest"), Anchor(950, 950, "low"), Anchor(1000, 1000, "top_swing"),
           Anchor(1050, 1050, "low"), Anchor(1100, 1499, "rest")]


def test_segment_reps():
    assert segment_reps(ANCHORS, 1500) == [(120, 200, 239, 300), (300, 380, 380, 450),
                                           (950, 1000, 1000, 1050)]
    assert segment_reps(list(reversed(ANCHORS)), 1500)[0] == (120, 200, 239, 300)  # sorted
    assert segment_reps(ANCHORS, 1050) == segment_reps(ANCHORS, 1500)[:2]  # next low past end
    assert segment_reps([], 10) == []


def test_segment_sets():
    # from the sample after the rest before to the sample before the rest after
    assert segment_sets(ANCHORS) == [(100, 499), (900, 1099)]
    # no rest before / after: the first / last anchor of the run bounds the set
    assert segment_sets(ANCHORS[1:6]) == [(120, 450)]
    assert segment_sets([Anchor(0, 99, "rest"), Anchor(150, 299, "rest")]) == []
    assert segment_sets([]) == []


# ----------------------------------------------------------------------------- rep_metrics
FS = 400.0


def test_moving_average_centred_truncated_and_nan_local():
    x = np.arange(8.0)
    assert moving_average(x, 1) == pytest.approx(x)
    # k = 4: window [i − 2, i + 2), truncated at the ends
    assert moving_average(x, 4) == pytest.approx([0.5, 1.0, 1.5, 2.5, 3.5, 4.5, 5.5, 6.0])
    x[2] = np.nan
    m = moving_average(x, 4)
    assert np.all(np.isnan(m[1:5]))  # the windows holding sample 2
    assert m[[0, 5, 6, 7]] == pytest.approx([0.5, 4.5, 5.5, 6.0])


def synthetic_metrics_input():
    n = 1000
    t = np.arange(n) / FS
    seg = (100, 300, 340, 600)
    v = np.zeros((n, 3))
    k = np.arange(100, 301)
    v[k, 2] = 3.0 * np.sin(np.pi * (k - 100) / 200)  # peak 3 m/s at sample 200
    v[k, 0] = 4.0 * np.sin(np.pi * (k - 100) / 200)  # |v| peak 5 m/s
    pos = np.zeros((n, 3))
    pos[100:601, 2] = np.concatenate([np.linspace(0.0, 1.5, 201), np.full(40, 1.5),
                                      np.linspace(1.5, 0.0, 260)])
    P = np.zeros(n)
    P[100:301] = 1000.0
    P[200] = 6000.0  # one-sample spike: 25 ms (10 samples) average -> +500 W
    P[341:600] = -500.0
    valid = np.ones(n, dtype=bool)
    saturated = np.zeros(n, dtype=bool)
    return seg, t, v, pos, P, valid, saturated


def test_rep_metrics_values():
    seg, t, v, pos, P, valid, saturated = synthetic_metrics_input()
    m = rep_metrics(seg, t, v, pos, P, valid, saturated)
    work = 1000.0 * 0.5 + 5000.0 / FS  # trapezoid: the spike adds 5000 W over one sample step
    assert m["work_con_j"] == pytest.approx(work)
    assert m["con_duration_s"] == pytest.approx(0.5)
    assert m["power_w"] == pytest.approx(work / 0.5)
    assert m["peak_power_w"] == pytest.approx(1500.0)
    assert m["peak_velocity_ms"] == pytest.approx(5.0)
    # P = −500 W over samples 341..599; trapezoid half steps at both ends
    assert m["work_ecc_j"] == pytest.approx(-500.0 * 259 / FS)
    assert m["apex_m"] == pytest.approx(1.5)
    assert m["cycle_s"] == pytest.approx(1.25)
    assert m["flags"] == set()


def test_rep_metrics_flags_only_inside_the_rep():
    seg, t, v, pos, P, valid, saturated = synthetic_metrics_input()
    valid[50] = False
    saturated[700] = True
    v[650] = np.nan
    assert rep_metrics(seg, t, v, pos, P, valid, saturated)["flags"] == set()
    valid[450] = False
    saturated[150] = True
    assert rep_metrics(seg, t, v, pos, P, valid, saturated)["flags"] == {"gap", "saturated"}
    valid[450] = True
    v[320] = np.nan
    assert rep_metrics(seg, t, v, pos, P, valid, saturated)["flags"] == {"gap", "saturated"}


def test_rep_metrics_nan_velocity_gives_nan_power_without_warnings():
    seg, t, v, pos, P, valid, saturated = synthetic_metrics_input()
    v[150:301] = np.nan
    P[150:301] = np.nan
    with np.errstate(all="raise"):
        m = rep_metrics(seg, t, v, pos, P, valid, saturated)
    assert np.isnan(m["power_w"]) and np.isnan(m["peak_velocity_ms"])
    assert "gap" in m["flags"]


# ----------------------------------------------------------------------------- simulated session
def test_session_kinds_hands_and_sets(session):
    reps = [r for st in session.sets for r in st.reps]
    assert [r.kind for r in reps] == ["snatch"] * 8 + ["swing"] * 12 + ["snatch"] * 8 + ["other"]
    assert [r.hand for r in reps] == ["L"] * 8 + ["-"] * 12 + ["R"] * 8 + ["-"]
    assert len(session.sets) == 4
    assert [len(st.reps) for st in session.sets] == [8, 12, 8, 1]
    assert all(isinstance(st, SetResult) for st in session.sets)
    assert all(isinstance(r, Rep) and r.path is None for r in reps)


def test_session_reps_match_truth(session):
    reps = [r for st in session.sets for r in st.reps]
    truth = session.truth.reps
    assert len(reps) == len(truth)
    for r, tr in zip(reps, truth):
        assert abs(r.i_low - tr.i_low) <= 12 and abs(r.i_end - tr.i_end) <= 12
        # the partial rep (other, 228 W) measures +11.4 % power / +3.8 % peak velocity
        # (default seed; upstream anchor/velocity accuracy on a short low-energy rep)
        rel = 0.05 if tr.kind != "other" else 0.15
        assert r.power_w == pytest.approx(tr.power_w, rel=rel)
        assert r.peak_velocity_ms == pytest.approx(tr.peak_velocity, rel=rel)
        assert r.apex_m == pytest.approx(r.features["apex_m"])
        assert r.work_con_j == pytest.approx(r.power_w * r.con_duration_s)
        assert r.peak_power_w > r.power_w > 0.0 > r.work_ecc_j
        assert r.cycle_s > r.con_duration_s > 0.0
        assert r.flags == set(), (r.kind, r.confidence, r.features)
    snatch = [r for r in reps if r.kind == "snatch"]
    assert all(r.i_top_end > r.i_top for r in snatch)
    assert all(r.i_top_end == r.i_top for r in reps if r.kind != "snatch")


def test_session_set_bounds_and_rests(session):
    fs = session.imu.fs
    t = session.imu.t
    for st, (a, b) in zip(session.sets, session.truth.sets):
        assert abs(st.i_start - a) / fs < 0.3 and abs(st.i_end - b) / fs < 0.3
        assert (st.t_start, st.t_end) == (t[st.i_start], t[st.i_end])
        assert all(st.i_start <= r.i_low and r.i_end <= st.i_end for r in st.reps)
    rests = [st.rest_before_s for st in session.sets]
    assert rests == pytest.approx([3.0, 6.0, 6.0, 6.0], abs=0.3)


def test_low_conf_flag_from_config(session):
    strict = ClassifierConfig(low_conf=0.9)
    sets = build_sets(session.anchors, session.imu, session.q, session.calib, session.v,
                      session.pos, session.P, strict)
    reps = [r for st in sets for r in st.reps]
    assert all(("low_conf" in r.flags) == (r.confidence < 0.9) for r in reps)
    assert any("low_conf" in r.flags for r in reps)
