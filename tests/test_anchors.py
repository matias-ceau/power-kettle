import numpy as np
import pytest

from powerkettle.anchors import (
    Anchor,
    AnchorConfig,
    detect_anchors,
    rest_forward,
    sagittal_tilt,
    set_forwards,
    tremor_rms,
)
from powerkettle.bell import BellProfile
from powerkettle.calibration import Calibration, calibrate
from powerkettle.imu import G, ImuData, scale_run
from powerkettle.kinematics import com_acceleration
from powerkettle.orientation import estimate_orientation
from powerkettle.simulate import Hold, LieOnSide, Rest, SimConfig, Snatch, Swing, simulate

BELL = BellProfile()
POINT_TOL = 12  # samples
WINDOW_IOU = 0.8
MAIN = [Rest(3), Snatch(10, "L"), Rest(8), Swing(15), Rest(8), Snatch(10, "R"), Rest(3)]


def pipeline(moves, cfg=SimConfig()):
    run, truth = simulate(moves, BELL, cfg)
    imu = scale_run(run)
    calib = calibrate(imu, BELL)
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    anchors = detect_anchors(imu, q, kin, calib)
    return imu, truth, calib, q, anchors


def iou(a0, a1, b0, b1):
    """Overlap of two inclusive index windows as intersection / union."""
    inter = min(a1, b1) - max(a0, b0) + 1
    return max(inter, 0) / (max(a1, b1) - min(a0, b0) + 1)


def matches(truth_win, det: Anchor) -> bool:
    i0, i1, kind = truth_win
    if kind != det.kind:
        return False
    if kind in ("rest", "top_lockout"):
        return iou(i0, i1, det.i0, det.i1) >= WINDOW_IOU
    return abs(det.i0 - i0) <= POINT_TOL and det.i0 == det.i1


def assert_one_to_one(truth_windows, anchors):
    """Every truth anchor matched by exactly one detection and every detection by one truth."""
    unmatched_truth = [w for w in truth_windows if sum(matches(w, a) for a in anchors) != 1]
    unmatched_det = [a for a in anchors if sum(matches(w, a) for w in truth_windows) != 1]
    assert not unmatched_truth, f"truth anchors not matched once: {unmatched_truth[:5]}"
    assert not unmatched_det, f"detections not matched once: {unmatched_det[:5]}"
    assert len(anchors) == len(truth_windows)


@pytest.fixture(scope="module")
def main_session():
    return pipeline(MAIN)


def test_all_truth_anchors_matched_one_to_one(main_session):
    imu, truth, calib, q, anchors = main_session
    assert anchors == sorted(anchors, key=lambda a: (a.i0, a.i1))
    kinds = [a.kind for a in anchors]
    assert kinds.count("low") == 2 * 11 + 16  # n + 1 lows per set
    assert kinds.count("top_lockout") == 20 and kinds.count("top_swing") == 15
    assert kinds.count("rest") == 4
    assert_one_to_one(truth.anchor_windows, anchors)


def test_point_anchor_timing(main_session):
    """Characterise the timing error of the point anchors (tolerance 12 samples)."""
    imu, truth, calib, q, anchors = main_session
    for kind in ("low", "top_swing"):
        t_idx = np.array([i0 for i0, _, k in truth.anchor_windows if k == kind])
        d_idx = np.array([a.i0 for a in anchors if a.kind == kind])
        assert len(t_idx) == len(d_idx)
        assert np.max(np.abs(d_idx - t_idx)) <= POINT_TOL


def test_set_boundaries_in_anchor_terms(main_session):
    """A set is the run of anchors between two rests: it opens and closes with a low anchor
    (lift-off and the descent to the floor are not reps), and no top sits next to a rest."""
    imu, truth, calib, q, anchors = main_session
    for prev, nxt in zip(anchors[:-1], anchors[1:]):
        if prev.kind == "rest":
            assert nxt.kind in ("low", "rest")
        if nxt.kind == "rest":
            assert prev.kind in ("low", "rest")
        if prev.kind.startswith("top"):
            assert nxt.kind == "low"
        if prev.kind == "low" and nxt.kind == "low":
            pytest.fail(f"two lows with no top between them at {prev.i0}, {nxt.i0}")


def test_snatch_mid_pull_minimum_never_an_anchor(main_session):
    imu, truth, calib, q, anchors = main_session
    snatches = [r for r in truth.reps if r.kind == "snatch"]
    assert len(snatches) == 20
    for r in snatches:
        for a, b in ((r.i_low, r.i_top), (r.i_top_end, r.i_end)):  # pull and drop
            i_min = a + int(np.argmin(truth.theta[a:b + 1]))
            assert np.degrees(truth.theta[i_min]) < -25  # it is the mid-pull θ minimum
            near = [x for x in anchors if x.i0 - 20 <= i_min <= x.i1 + 20]
            assert not near, f"anchor at the mid-pull minimum {i_min}: {near}"


def test_hold_after_snatch_is_lockout_not_rest():
    imu, truth, calib, q, anchors = pipeline([Rest(3), Snatch(5, "R"), Hold(5), Rest(4)])
    last = truth.reps[-1]
    assert (last.i_top_end - last.i_top) / imu.fs > 5.0
    hold = [a for a in anchors if a.i0 <= last.i_top + 20 and a.i1 >= last.i_top_end - 20]
    assert len(hold) == 1 and hold[0].kind == "top_lockout"
    assert_one_to_one(truth.anchor_windows, anchors)


def test_lie_on_side_between_sets_is_rest():
    moves = [Rest(3), Swing(5), LieOnSide(10), Snatch(5, "R"), Rest(3)]
    imu, truth, calib, q, anchors = pipeline(moves)
    rests = [w for w in truth.anchor_windows if w[2] == "rest"]
    assert len(rests) == 3
    lie = rests[1]
    assert (lie[1] - lie[0]) / imu.fs > 9.0
    assert any(a.kind == "rest" and iou(*lie[:2], a.i0, a.i1) >= WINDOW_IOU for a in anchors)
    assert_one_to_one(truth.anchor_windows, anchors)


def test_tremor_free_hold_is_rest_and_lows_survive():
    """A hold without tremor is static and quiet, so it is taken as a rest here (Task 8's
    height check turns it into a lockout). The bell is twisted 180° at lockout, so the forward
    read from that 'rest' points backwards; the lows on both sides must still be found."""
    cfg = SimConfig(tremor_amp=0.0)
    imu, truth, calib, q, anchors = pipeline([Rest(3), Snatch(3, "R"), Hold(3), Rest(3)], cfg)
    last = truth.reps[-1]
    kinds = [a.kind for a in anchors]
    assert kinds == ["rest", "low", "top_lockout", "low", "top_lockout", "low", "rest",
                     "low", "rest"]
    hold = anchors[6]
    assert iou(last.i_top, last.i_top_end, hold.i0, hold.i1) >= WINDOW_IOU
    for got, want in zip([a for a in anchors if a.kind == "low"],
                         sorted({r.i_low for r in truth.reps} | {r.i_end for r in truth.reps})):
        assert abs(got.i0 - want) <= POINT_TOL


@pytest.mark.parametrize("mount_deg", [-30.0, 0.0, 30.0, 90.0])
def test_tremor_rms_floor_rest_vs_hold(mount_deg):
    """The tremor measure must not depend on how the box is mounted (at -30° the simulated
    tremor is almost perpendicular to gravity, invisible in |f|)."""
    run, truth = simulate([Rest(3), Snatch(2, "R"), Hold(5), Rest(4)], BELL,
                          SimConfig(mount_angle_deg=mount_deg))
    imu = scale_run(run)
    rests = [(i0, i1) for i0, i1, k in truth.anchor_windows if k == "rest"]
    assert len(rests) == 2
    for i0, i1 in rests:
        assert tremor_rms(imu.acc[i0:i1 + 1], imu.fs, (4.0, 12.0)) < 0.01  # half the threshold
    last = truth.reps[-1]
    hold = imu.acc[last.i_top:last.i_top_end + 1]
    assert tremor_rms(hold, imu.fs, (4.0, 12.0)) > 0.05


def test_tremor_rms_of_pure_tone_any_direction():
    fs = 400.0
    t = np.arange(800) / fs
    tone = 0.1 * np.sin(2 * np.pi * 8.0 * t)
    for axis in range(3):  # gravity on z; a tone across gravity counts as much as along it
        acc = np.zeros((800, 3))
        acc[:, 2] = G + 0.3 * t  # slow trend, removed by the detrend
        acc[:, axis] += tone
        assert tremor_rms(acc, fs, (4.0, 12.0)) == pytest.approx(0.1 / np.sqrt(2), rel=0.05)
        assert tremor_rms(acc, fs, (20.0, 40.0)) < 0.005


def test_sagittal_tilt_matches_simulator_theta(main_session):
    imu, truth, calib, q, anchors = main_session
    fwd = rest_forward(q, calib, Anchor(calib.still.start, calib.still.stop - 1, "rest"))
    theta = sagittal_tilt(q, calib, fwd)
    err = np.degrees(np.abs(theta - truth.theta))
    assert err.max() < 2.0, f"max θ error {err.max():.2f}°"


def test_set_forwards_point_forward(main_session):
    """The estimated world has athlete forward = +x (calibration); each set's forward, read
    from the rest before it, stays there up to heading drift."""
    imu, truth, calib, q, anchors = main_session
    fwds = set_forwards(q, calib, anchors)
    assert len(fwds) == 3
    for f in fwds:
        assert f[2] == 0.0 and np.linalg.norm(f) == pytest.approx(1.0)
        assert np.degrees(np.arccos(np.clip(f[0], -1, 1))) < 3.0


def _synthetic_calib():
    return Calibration(np.zeros(3), np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0]),
                       np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), slice(0, 10), G)


def _yaw(deg):
    a = np.radians(deg) / 2
    return np.array([np.cos(a), 0.0, 0.0, np.sin(a)])


def test_set_forwards_sign_continuity_and_degenerate_rest():
    """Sensor frame = world at calibration with outward = -x, so forward = +x. A later rest
    whose outward axis is flipped (bell parked turned round, or a quiet lockout) keeps the
    previous sign; one whose outward axis is vertical falls back to the previous forward."""
    calib = _synthetic_calib()
    n = 100
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    q[30:40] = _yaw(170.0)  # rest 2: outward turned round
    tip = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])  # 90° about y
    q[60:70] = tip  # rest 3: outward vertical
    anchors = [Anchor(0, 9, "rest"), Anchor(15, 15, "low"), Anchor(20, 20, "top_swing"),
               Anchor(25, 25, "low"), Anchor(30, 39, "rest"), Anchor(45, 45, "low"),
               Anchor(60, 69, "rest"), Anchor(75, 75, "low")]
    f1, f2, f3 = set_forwards(q, calib, anchors)
    assert np.allclose(f1, [1.0, 0.0, 0.0])
    assert np.degrees(np.arccos(f2[0])) == pytest.approx(10.0, abs=1e-6)  # axis kept, sign fixed
    assert np.allclose(f3, f2)
    assert np.allclose(rest_forward(q, calib, Anchor(30, 39, "rest"))[:2],
                       -f2[:2])  # the raw rest reading is the flipped one
    with pytest.raises(ValueError):
        rest_forward(q, calib, Anchor(60, 69, "rest"))


def test_set_forwards_without_leading_rest_uses_calibration_window():
    calib = _synthetic_calib()
    q = np.tile([1.0, 0.0, 0.0, 0.0], (50, 1))
    (f,) = set_forwards(q, calib, [Anchor(20, 20, "low"), Anchor(25, 25, "top_swing"),
                                   Anchor(30, 30, "low")])
    assert np.allclose(f, [1.0, 0.0, 0.0])


def test_static_signal_has_no_lows():
    n, fs = 2000, 400.0
    imu = ImuData(t=np.arange(n) / fs, acc=np.tile([0.0, G, 0.0], (n, 1)),
                  gyro=np.zeros((n, 3)), valid=np.ones(n, bool), saturated=np.zeros(n, bool),
                  fs=fs, first_index=0)
    calib = calibrate(imu, BELL)
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    anchors = detect_anchors(imu, q, kin, calib, AnchorConfig())
    assert [a.kind for a in anchors] == ["rest"]
    assert (anchors[0].i0, anchors[0].i1) == (0, n - 1)
