import numpy as np
import pytest

from powerkettle import quat
from powerkettle.anchors import set_forwards
from powerkettle.paths import athlete_forward, rep_path
from powerkettle.reps import build_sets
from powerkettle.simulate import Rest, SimConfig, Snatch, Swing

from sim_pipeline import run_stages

SESSION = [Rest(3), Snatch(8, "L"), Rest(6), Swing(12), Rest(6)]
TRUTH_FORWARD = np.array([1.0, 0.0, 0.0])  # the estimate's world has athlete forward = +x


def to_estimate_frame(truth, vec):
    """Rotate truth vectors (simulator world) by −heading about z into the estimate's world."""
    heading = np.arctan2(truth.forward_w[1], truth.forward_w[0])
    return quat.rotate(quat.from_axis_angle([0.0, 0.0, 1.0], -heading), vec)


def angle_deg(a, b):
    return np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))


@pytest.fixture(scope="module")
def session():
    s = run_stages(SESSION, SimConfig(heading_deg=37.0))
    s.sets = build_sets(s.anchors, s.imu, s.q, s.calib, s.v, s.pos, s.P)
    s.hints = set_forwards(s.q, s.calib, s.anchors)
    s.truth_pos = to_estimate_frame(s.truth, s.truth.pos)
    return s


def set_forward(s, k):
    st = s.sets[k]
    return athlete_forward(s.v, slice(st.i_start, st.i_end + 1), s.hints[k])


def test_forward_within_3_degrees_of_truth(session):
    assert len(session.sets) == 2
    for k in range(2):
        f = set_forward(session, k)
        assert f.shape == (3,) and f[2] == 0.0 and np.linalg.norm(f) == pytest.approx(1.0)
        assert angle_deg(f, TRUTH_FORWARD) < 3.0


def test_snatch_path(session):
    s = session
    rep = s.sets[0].reps[0]
    assert rep.kind == "snatch"
    seg = (rep.i_low, rep.i_top, rep.i_top_end, rep.i_end)
    path = rep_path(s.pos, seg, set_forward(s, 0))
    assert path.shape == (len(range(rep.i_low, rep.i_end + 1, 8)), 3)
    assert np.allclose(path[0], 0.0, atol=1e-9)
    assert np.max(path[:, 2]) == pytest.approx(1.6, abs=0.05)
    tr = s.truth_pos[rep.i_low:rep.i_end + 1] - s.truth_pos[rep.i_low]
    tr_top = tr[rep.i_top - rep.i_low:rep.i_top_end - rep.i_low + 1]
    i_apex = np.argmax(path[:, 2])
    lockout_x = path[i_apex, 0]
    assert abs(lockout_x - tr_top[np.argmax(tr_top[:, 2]), 0]) < 0.05
    assert lockout_x == pytest.approx(0.35, abs=0.05)  # brief's constant, +0.35 m from the low


def test_swing_path(session):
    s = session
    rep = s.sets[1].reps[0]
    assert rep.kind == "swing"
    seg = (rep.i_low, rep.i_top, rep.i_top_end, rep.i_end)
    path = rep_path(s.pos, seg, set_forward(s, 1))
    tr = s.truth_pos[rep.i_low:rep.i_end + 1] - s.truth_pos[rep.i_low]
    assert np.max(path[:, 0]) == pytest.approx(0.90, abs=0.05)
    assert abs(np.max(path[:, 0]) - np.max(tr[:, 0])) < 0.05
    assert np.max(np.abs(path[:, 1])) < 0.05  # no lateral drift: swings are sagittal


def test_path_frame_and_decimation():
    t = np.arange(0, 1.0, 1 / 400)
    base = np.stack([np.sin(t), np.zeros_like(t), t], axis=1)  # world x, z
    pos = base + np.array([5.0, 5.0, 5.0])  # origin offset is removed
    fw = np.array([0.0, 1.0, 0.0])  # athlete forward = world +y, so left = world −x
    path = rep_path(pos, (0, 100, 100, 399), fw, decimate_to_hz=50, fs=400)
    assert path.shape == (50, 3)
    assert np.allclose(path[0], 0.0)
    assert np.allclose(path[:, 0], 0.0)  # nothing along world y
    assert np.allclose(path[:, 1], -np.sin(t[::8]) + 0.0)  # left = ẑ × fw = −x
    assert np.allclose(path[:, 2], t[::8])


def test_path_keeps_nan_rows():
    pos = np.zeros((100, 3))
    pos[40:60] = np.nan
    path = rep_path(pos, (0, 50, 50, 99), np.array([1.0, 0.0, 0.0]), decimate_to_hz=100, fs=400)
    assert path.shape == (25, 3)
    assert np.all(np.isnan(path[10:15])) and np.all(np.isfinite(path[:10]))


def test_forward_from_velocity_sign_follows_hint():
    ang = np.radians(30.0)
    d = np.array([np.cos(ang), np.sin(ang), 0.0])
    s = np.sin(np.linspace(0, 20, 400))
    v = np.outer(s, d) + np.array([0.0, 0.0, 1.0]) * s[:, None]
    f = athlete_forward(v, slice(0, 400), d)
    assert np.allclose(f, d, atol=1e-6)
    f = athlete_forward(v, slice(0, 400), -d)
    assert np.allclose(f, -d, atol=1e-6)


def test_forward_falls_back_to_hint():
    hint = np.array([0.0, 1.0, 0.0])
    v = np.full((400, 3), np.nan)
    v[:10, 0] = 1.0  # too few moving samples
    assert np.allclose(athlete_forward(v, slice(0, 400), hint), hint)
    assert np.allclose(athlete_forward(np.zeros((400, 3)), slice(0, 400), hint), hint)
