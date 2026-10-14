import math
import warnings

import pytest

from powerkettle.metrics import (PowerStats, SessionMetrics, SetMetrics, best_efforts,
                                 power_stats, session_metrics, set_metrics)
from powerkettle.reps import Rep, SetResult

REF = [300, 320, 310, 250, 240, 230]


def make_rep(power=300.0, work=400.0, kind="snatch", hand="L", flags=()):
    return Rep(i_low=0, i_top=1, i_top_end=1, i_end=2, kind=kind, hand=hand, confidence=1.0,
               features={}, flags=set(flags), power_w=power, peak_power_w=power * 1.5,
               peak_velocity_ms=2.0, work_con_j=work, work_ecc_j=-work / 2, apex_m=0.5,
               con_duration_s=0.4, cycle_s=1.0)


def make_set(reps, t_start=0.0, t_end=10.0, rest_before_s=0.0):
    return SetResult(reps, 0, 0, t_start, t_end, rest_before_s)


def set_of_snatches(powers, hand="L", duration_s=10.0):
    return make_set([make_rep(p, hand=hand) for p in powers], 0.0, duration_s)


def make_sets(rest_s):
    sets = [make_set([make_rep() for _ in range(5)], 0.0, 10.0),
            make_set([make_rep() for _ in range(5)], 10.0 + rest_s, 20.0 + rest_s, rest_s)]
    return sets, 20.0 + rest_s


def test_power_stats_reference():
    s = power_stats(REF)
    assert (s.avg, s.best) == (275, 320)
    assert s.n == 6
    assert s.top3 == pytest.approx(310)
    assert s.dropoff == pytest.approx(0.25)
    assert s.cv == pytest.approx(0.13068, abs=1e-4)


def test_best_efforts_reference():
    assert best_efforts(REF) == pytest.approx({1: 320, 3: 310, 5: 284})


def test_best_efforts_omits_long_windows_and_handles_empty():
    assert best_efforts([]) == {}
    assert best_efforts([200, 100], ns=(1, 2, 3)) == pytest.approx({1: 200, 2: 150})


def test_small_sessions_do_not_raise():
    e = power_stats([])
    assert (e.n, e.avg, e.best, e.top3, e.dropoff, e.cv) == (0, None, None, None, None, None)
    one = power_stats([200])
    assert (one.top3, one.dropoff, one.cv) == (200, 0.0, None)
    two = power_stats([200, 100])
    assert two.top3 == pytest.approx(150)
    assert two.dropoff == pytest.approx(0.25)  # (200 - mean(200, 100)) / 200


def test_power_stats_zero_power_has_no_ratio_metrics():
    s = power_stats([0.0, 0.0])
    assert (s.avg, s.dropoff, s.cv) == (0.0, None, None)


def test_rest_does_not_change_power_but_changes_density():
    a = session_metrics(*make_sets(rest_s=30))
    b = session_metrics(*make_sets(rest_s=180))
    assert a.by_kind["snatch"].avg == b.by_kind["snatch"].avg == 300
    assert (a.density_w, b.density_w) == pytest.approx((80.0, 20.0))
    assert (a.active_s, b.active_s) == (20.0, 20.0)
    assert (a.rest_s, b.rest_s) == pytest.approx((30.0, 180.0))
    assert (a.work_rest_ratio, b.work_rest_ratio) == pytest.approx((20 / 30, 20 / 180))
    assert a.work_j == b.work_j == 4000.0
    assert a.reps_by_kind_hand == {"snatch:L": 10}


def test_power_floor_counts_qualifying():
    s = set_of_snatches(REF)
    assert set_metrics(s, power_floor_w=260).qualifying == 3
    assert set_metrics(s, power_floor_w=250).qualifying == 4  # >= floor
    assert set_metrics(s).qualifying == 6


def test_set_metrics_values():
    m = set_metrics(set_of_snatches(REF, duration_s=30.0))
    assert m.by_kind["snatch"].avg == 275
    assert set(m.by_kind) == {"snatch"}
    assert m.reps_by_kind_hand == {"snatch:L": 6}
    assert m.pace_rpm == pytest.approx(12.0)  # 6 reps / 0.5 min
    assert m.work_j == pytest.approx(2400.0)


def test_set_metrics_excludes_gap_nonfinite_and_other_from_power_stats():
    reps = [make_rep(300), make_rep(100, flags={"gap"}), make_rep(float("nan"), work=50.0),
            make_rep(500, kind="other", hand="-"), make_rep(200, kind="swing", hand="-"),
            make_rep(260, flags={"saturated", "low_conf"})]
    m = set_metrics(make_set(reps, 0.0, 60.0), power_floor_w=250)
    assert m.by_kind["snatch"].n == 2 and m.by_kind["snatch"].avg == 280
    assert m.by_kind["swing"].n == 1
    assert set(m.by_kind) == {"snatch", "swing"}
    # counts include the gap rep and the non-finite one: they happened
    assert m.reps_by_kind_hand == {"snatch:L": 4, "other:-": 1, "swing:-": 1}
    assert m.pace_rpm == pytest.approx(6.0)
    # work over non-gap reps only: 400 (300) + 50 (nan power) + 400 (other) + 400 + 400
    assert m.work_j == pytest.approx(1650.0)
    assert m.qualifying == 3  # 300, 500 (other, non-gap), 260; nan never qualifies
    assert not math.isnan(m.work_j)


def test_set_metrics_hands_are_separate_keys():
    s = make_set([make_rep(hand="L"), make_rep(hand="R"), make_rep(hand="R"),
                  make_rep(hand="?")])
    assert set_metrics(s).reps_by_kind_hand == {"snatch:L": 1, "snatch:R": 2, "snatch:?": 1}
    assert set_metrics(s).by_kind["snatch"].n == 4  # hands mixed within a kind


def test_empty_inputs_do_not_raise_or_warn():
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        m = set_metrics(make_set([], 0.0, 0.0))
        assert m == SetMetrics({}, {}, 0, None, 0.0)
        assert set_metrics(make_set([make_rep()], 5.0, 5.0)).pace_rpm is None  # zero duration
        s = session_metrics([], 0.0)
        assert s.density_w is None and s.work_rest_ratio is None
        assert (s.work_j, s.active_s, s.rest_s) == (0.0, 0.0, 0.0)
        assert s.by_kind == {} and s.best_efforts == {} and s.reps_by_kind_hand == {}
        assert session_metrics([], 100.0).density_w == 0.0
        assert power_stats([]) == PowerStats(0, None, None, None, None, None)
        assert best_efforts([]) == {}


def test_session_pools_sets_and_best_efforts_per_kind():
    sets = [make_set([make_rep(300), make_rep(320), make_rep(100, flags={"gap"}),
                      make_rep(200, kind="swing", hand="-")], 0.0, 10.0),
            make_set([make_rep(310, hand="R"), make_rep(250, kind="swing", hand="-")],
                     40.0, 50.0, 30.0)]
    m = session_metrics(sets, 50.0)
    assert m.reps_by_kind_hand == {"snatch:L": 3, "snatch:R": 1, "swing:-": 2}
    assert m.by_kind["snatch"].n == 3 and m.by_kind["snatch"].best == 320
    assert m.by_kind["swing"].avg == 225
    # consecutive across sets, gap rep dropped: 300, 320, 310
    assert m.best_efforts["snatch"] == pytest.approx({1: 320, 3: 310})
    assert m.best_efforts["swing"] == pytest.approx({1: 250})
    assert m.work_j == pytest.approx(5 * 400.0)  # gap rep's work excluded (6 reps - 1)
    assert m.density_w == pytest.approx(40.0)
    assert (m.active_s, m.rest_s) == (20.0, 30.0)


def test_session_floor_only_affects_qualifying():
    s = session_metrics([set_of_snatches(REF), set_of_snatches(REF)], 20.0, power_floor_w=260)
    assert s.by_kind["snatch"].n == 12
    assert s.qualifying == 6
    assert session_metrics([set_of_snatches(REF)], 10.0).qualifying == 6
    assert isinstance(s, SessionMetrics)
