"""Physics sanity checks of the production chain (spec §9.1) and characterisation of the swing
assumption it relies on most (zero velocity at the backswing low)."""
import numpy as np
import pytest

from powerkettle.bell import BellProfile
from powerkettle.calibration import CalibrationConfig
from powerkettle.imu import G, scale_run
from powerkettle.pipeline import AnalysisConfig, analyse_run, calibrate_run, motion_stages
from powerkettle.simulate import Hold, Rest, SimConfig, Snatch, Swing, simulate

BELL = BellProfile()
MOVES = [Rest(3), Snatch(8, "L"), Hold(3), Rest(6), Swing(10), Rest(4)]


@pytest.fixture(scope="module")
def session():
    run, truth = simulate(MOVES, BELL)
    imu = scale_run(run)
    calib, imu, _ = calibrate_run(imu, BELL, CalibrationConfig())
    m = motion_stages(imu, calib, BELL, AnalysisConfig())
    res = analyse_run(run, BELL)
    return truth, m, [r for s in res.sets for r in s.reps]


def windows(truth, kind):
    return [(i0, i1) for i0, i1, k in truth.anchor_windows if k == kind]


def test_power_zero_at_rest(session):
    """P = m (a + g ẑ)·v: on the floor v ≈ 0, so |P| stays near 0 (measured max ≈ 1 W)."""
    truth, m, _ = session
    for i0, i1 in windows(truth, "rest"):
        assert np.nanmax(np.abs(m.P[i0:i1 + 1])) < 5.0, (i0, i1)


def test_velocity_zero_during_hold(session):
    """The 3 s overhead hold (lockout window) has v ≈ 0 throughout."""
    truth, m, _ = session
    holds = [(i0, i1) for i0, i1 in windows(truth, "top_lockout") if i1 - i0 > 2.0 * truth.fs_true]
    assert len(holds) == 1
    i0, i1 = holds[0]
    assert np.max(np.linalg.norm(m.v[i0:i1 + 1], axis=1)) < 0.05


def test_concentric_work_at_least_lifting_work(session):
    """Concentric work = m g Δz + kinetic energy at the top (≥ 0) ≥ m g apex, within the
    integration error (measured ratio 0.986–1.010)."""
    _, _, reps = session
    for i, r in enumerate(reps):
        assert r.work_con_j >= 0.98 * BELL.mass_kg * G * r.apex_m, i


def test_lockout_heights_consistent(session):
    """The snatches of a set reach the same lockout height (measured std 0.2 cm)."""
    _, _, reps = session
    apex = [r.apex_m for r in reps if r.kind == "snatch"]
    assert len(apex) == 8 and np.std(apex) < 0.05


@pytest.mark.parametrize("speed", [0.15, 0.3])
def test_residual_speed_at_the_low(speed):
    """Characterisation: the bell still moving along the arc at the backswing low (θ maximum,
    the low anchor, taken as v = 0). Measured session power error (swing / snatch): 0.15 m/s
    −0.5 % / +1.8 %, 0.3 m/s −0.2 % / +3.4 %; kinds unaffected. Bound: 10 %."""
    run, truth = simulate([Rest(3), Swing(12), Rest(5), Snatch(10, "R"), Rest(3)], BELL,
                          SimConfig(low_residual_speed=speed))
    res = analyse_run(run, BELL)
    assert [r.kind for s in res.sets for r in s.reps] == [t.kind for t in truth.reps]
    for kind, st in res.metrics.by_kind.items():
        true_avg = np.mean([t.power_w for t in truth.reps if t.kind == kind])
        assert st.avg == pytest.approx(true_avg, rel=0.10), kind
