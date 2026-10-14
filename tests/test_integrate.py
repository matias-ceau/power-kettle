import warnings
from types import SimpleNamespace

import numpy as np
import pytest

from powerkettle import quat
from powerkettle.anchors import Anchor, AnchorConfig, detect_anchors, refine_rests
from powerkettle.bell import BellProfile
from powerkettle.calibration import calibrate
from powerkettle.imu import G, scale_run
from powerkettle.integrate import (
    concentric_power,
    power,
    rest_core,
    segment_position,
    segment_velocity,
)
from powerkettle.kinematics import com_acceleration
from powerkettle.orientation import estimate_orientation
from powerkettle.simulate import (
    Hold,
    LieOnSide,
    Rest,
    SimConfig,
    Snatch,
    Swing,
    TableLift,
    simulate,
)

BELL = BellProfile()
MAIN = [Rest(3), Snatch(10, "L"), Rest(8), Swing(15), Rest(8), Snatch(10, "R"), Rest(3)]
POINT_TOL = 12  # samples, anchor timing tolerance (as in test_anchors)


def pipeline(moves, cfg=SimConfig()):
    """Stages 2-8: scale, calibrate, orientation, COM acceleration, anchors, then velocity and
    position, refine_rests, and velocity / position / power once more on the refined anchors."""
    run, truth = simulate(moves, BELL, cfg)
    imu = scale_run(run)
    calib = calibrate(imu, BELL)
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    raw_anchors = detect_anchors(imu, q, kin, calib)
    v = segment_velocity(kin.a_com, imu.t, raw_anchors)
    pos = segment_position(v, imu.t, raw_anchors)
    anchors = refine_rests(raw_anchors, pos, AnchorConfig())
    v = segment_velocity(kin.a_com, imu.t, anchors)
    pos = segment_position(v, imu.t, anchors)
    P = power(kin.a_com, v, BELL.mass_kg)
    return SimpleNamespace(imu=imu, truth=truth, kin=kin, raw_anchors=raw_anchors,
                           anchors=anchors, v=v, pos=pos, P=P)


def truth_vel(truth):
    """Truth velocity in the estimate's world (athlete forward = +x): rotate about z by
    −heading (see test_kinematics)."""
    heading = np.arctan2(truth.forward_w[1], truth.forward_w[0])
    return quat.rotate(quat.from_axis_angle([0.0, 0.0, 1.0], -heading), truth.vel)


def detected_reps(anchors):
    """(low, top i0, next low) for every low -> top -> low run of consecutive anchors."""
    out = []
    for a, b, c in zip(anchors[:-2], anchors[1:-1], anchors[2:]):
        if a.kind == "low" and b.kind.startswith("top") and c.kind == "low":
            out.append((a.i0, b.i0, c.i0))
    return out


def matched_reps(s):
    """Pairs (truth rep, detected (low, top, next low)), one per truth rep."""
    det = detected_reps(s.anchors)
    pairs = []
    for r in s.truth.reps:
        hits = [d for d in det if abs(d[0] - r.i_low) <= POINT_TOL]
        assert len(hits) == 1, f"truth rep at {r.i_low}: {hits}"
        pairs.append((r, hits[0]))
    assert len(det) == len(s.truth.reps)
    return pairs


def rep_errors(s):
    """Relative power / peak-velocity errors and absolute apex errors, one per rep."""
    p_err, v_err, z_err = [], [], []
    for r, (lo, top, _) in matched_reps(s):
        p = concentric_power(s.P, s.imu.t, lo, top)
        p_err.append(p / r.power_w - 1.0)
        peak = np.max(np.linalg.norm(s.v[lo:top + 1], axis=1))
        v_err.append(peak / r.peak_velocity - 1.0)
        z_err.append(np.max(s.pos[lo:top + 1, 2] - s.pos[lo, 2]) - r.apex_m)
    return np.array(p_err), np.array(v_err), np.array(z_err)


@pytest.fixture(scope="module")
def main_session():
    return pipeline(MAIN)


# ----------------------------------------------------------------------------- simulated sessions
def test_velocity_rms_over_reps(main_session):
    s = main_session
    vt = truth_vel(s.truth)
    inside = np.zeros(len(s.imu.t), dtype=bool)
    for r in s.truth.reps:
        inside[r.i_low:r.i_end + 1] = True
    err = np.linalg.norm(s.v[inside] - vt[inside], axis=1)
    assert np.all(np.isfinite(err))
    rms = float(np.sqrt(np.mean(err**2)))
    assert rms < 0.08, f"velocity RMS error {rms:.4f} m/s"


def test_rep_power_peak_velocity_and_apex(main_session):
    p_err, v_err, z_err = rep_errors(main_session)
    assert len(p_err) == 35
    assert np.max(np.abs(p_err)) < 0.05, f"rep power errors {np.round(p_err, 4)}"
    assert np.max(np.abs(v_err)) < 0.05, f"peak velocity errors {np.round(v_err, 4)}"
    assert np.max(np.abs(z_err)) < 0.05, f"apex errors (m) {np.round(z_err, 4)}"


def test_float_residual_speed_characterisation():
    """Swing tops are not quite zero-velocity: with 0.15 m/s left at the float, forcing v = 0
    there biases rep power. Measured (default seed): max |rep power error| = 2.3 %
    (mean −0.95 %), against 1.3 % with no residual (main session)."""
    s = pipeline([Rest(3), Swing(15), Rest(3)], SimConfig(float_residual_speed=0.15))
    p_err, _, _ = rep_errors(s)
    assert len(p_err) == 15
    assert np.max(np.abs(p_err)) < 0.10, f"rep power errors {np.round(p_err, 4)}"


def test_table_lift_height():
    s = pipeline([Rest(3), TableLift(0.74), Rest(3)])
    z = s.pos[:, 2]
    assert np.nanmax(z) == pytest.approx(0.74, abs=0.03)


def test_tremor_free_hold_refined_to_lockout():
    s = pipeline([Rest(3), Snatch(3, "R"), Hold(3), Rest(3)], SimConfig(tremor_amp=0.0))
    last = s.truth.reps[-1]
    assert (last.i_top_end - last.i_top) / s.imu.fs > 3.0
    before = [a for a in s.raw_anchors if a.i0 <= last.i_top + 20 and a.i1 >= last.i_top_end - 20]
    assert [a.kind for a in before] == ["rest"]  # Task 7 alone takes the quiet hold as a rest
    after = [a for a in s.anchors if a.i0 <= last.i_top + 20 and a.i1 >= last.i_top_end - 20]
    assert len(after) == 1 and after[0].kind == "top_lockout"
    assert (after[0].i0, after[0].i1) == (before[0].i0, before[0].i1)
    assert [a.kind for a in s.anchors].count("rest") == 2  # the floor rests stay rests


def test_lie_on_side_stays_rest():
    s = pipeline([Rest(3), Swing(5), LieOnSide(10), Snatch(5, "R"), Rest(3)])
    assert s.anchors == s.raw_anchors
    assert [a.kind for a in s.anchors].count("rest") == 3


# ----------------------------------------------------------------------------- segment_velocity
FS = 100.0


def _t(n):
    return np.arange(n) / FS


def test_velocity_detrend_removes_constant_bias():
    """v_x = sin(π t / 1 s) is zero at the anchors 0, 100, 200; a constant acceleration bias
    integrates to a linear ramp, which the detrend removes."""
    n = 201
    t = _t(n)
    a = np.zeros((n, 3))
    a[:, 0] = np.pi * np.cos(np.pi * t) + 0.5
    a[:, 2] = -0.3
    anchors = [Anchor(0, 0, "low"), Anchor(100, 100, "top_swing"), Anchor(200, 200, "low")]
    v = segment_velocity(a, t, anchors)
    assert v.shape == (n, 3)
    assert np.max(np.abs(v[:, 0] - np.sin(np.pi * t))) < 1e-3
    assert np.max(np.abs(v[:, 1:])) < 1e-12
    assert np.all(v[[0, 100, 200]] == 0.0)


def test_velocity_zero_in_windows_nan_outside_anchors():
    n = 200
    t = _t(n)
    a = np.ones((n, 3))
    anchors = [Anchor(10, 79, "rest"), Anchor(100, 100, "low"), Anchor(120, 139, "top_lockout"),
               Anchor(160, 160, "low")]
    assert rest_core(anchors[0], FS) == (30, 59)  # REST_EDGE_S = 0.2 s at 100 Hz
    assert rest_core(anchors[2], FS) == (120, 139)  # a lockout is zero-velocity throughout
    v = segment_velocity(a, t, anchors)
    assert np.all(np.isnan(v[:10])) and np.all(np.isnan(v[161:]))
    assert np.all(v[10:60] == 0.0)  # leading rest edge faces no segment: stays 0
    assert np.all(v[120:140] == 0.0)
    assert np.all(np.isfinite(v[10:161]))
    # constant a: v_raw = a (t − t0), detrended to zero at both ends -> zero everywhere
    assert np.max(np.abs(v[10:161])) < 1e-12


def test_velocity_integrates_through_rest_edge():
    """Motion starting gently inside the detected rest window (as static_mask allows): the
    zero-velocity point is the rest core, so the onset speed at the window's end is kept."""
    n = 301
    t = _t(n)
    v_true = np.zeros(n)
    move = (t >= 0.9) & (t <= 2.9)  # starts 0.1 s before the window end (sample 99)
    v_true[move] = np.sin(np.pi * (t[move] - 0.9) / 2.0) ** 2
    a = np.zeros((n, 3))
    a[:, 2] = np.gradient(v_true, t)
    anchors = [Anchor(0, 99, "rest"), Anchor(290, 300, "rest")]
    v = segment_velocity(a, t, anchors)
    assert v_true[99] > 0.019  # forcing 0 there would offset the whole lift by this
    assert np.max(np.abs(v[:, 2] - v_true)) < 2e-3
    assert np.all(v[:80] == 0.0) and np.all(v[-1] == 0.0)


def test_velocity_nan_makes_whole_segment_nan():
    n = 201
    t = _t(n)
    a = np.zeros((n, 3))
    a[:, 0] = np.pi * np.cos(np.pi * t)
    a[150, 1] = np.nan
    anchors = [Anchor(0, 0, "low"), Anchor(100, 100, "top_swing"), Anchor(200, 200, "low")]
    v = segment_velocity(a, t, anchors)
    assert np.all(np.isnan(v[101:200]))
    assert np.all(np.isfinite(v[:101])) and np.all(v[200] == 0.0)
    assert np.max(np.abs(v[:101, 0] - np.sin(np.pi * t[:101]))) < 1e-3


def test_velocity_nan_segment_covers_its_rest_edges():
    """A NaN segment is NaN on the rest edges it owns too (they are integrated, not known);
    the rest core and lockouts stay 0."""
    n = 301
    t = _t(n)
    a = np.zeros((n, 3))
    a[150, 0] = np.nan
    anchors = [Anchor(0, 99, "rest"), Anchor(200, 200, "low"), Anchor(220, 239, "top_lockout"),
               Anchor(260, 260, "low")]
    v = segment_velocity(a, t, anchors)
    assert np.all(v[:80] == 0.0)  # leading edge (no segment) and core
    assert np.all(np.isnan(v[80:200]))  # trailing rest edge + segment
    assert np.all(v[200] == 0.0) and np.all(v[220:240] == 0.0)
    assert np.all(np.isfinite(v[200:261])) and np.all(np.isnan(v[261:]))


def test_velocity_without_anchors_is_nan():
    v = segment_velocity(np.zeros((10, 3)), _t(10), [])
    assert v.shape == (10, 3) and np.all(np.isnan(v))


# ----------------------------------------------------------------------------- segment_position
def test_position_restarts_at_lows_and_rest_core_end():
    n = 260
    t = _t(n)
    v = np.zeros((n, 3))
    v[:, 2] = 1.0  # constant 1 m/s, independent of the anchors (tests the integration only)
    anchors = [Anchor(10, 69, "rest"), Anchor(90, 90, "low"), Anchor(110, 110, "top_swing"),
               Anchor(130, 130, "low"), Anchor(150, 209, "rest")]
    v[10:70] = 0.0
    v[150:210] = 0.0
    pos = segment_position(v, t, anchors)
    z = pos[:, 2]
    assert np.all(np.isnan(z[:49]))  # no origin before the first restart (rest core end, 49)
    assert np.all(z[49:70] == 0.0) and z[71] == pytest.approx(0.015)
    assert z[89] == pytest.approx(0.195)
    assert z[90] == 0.0 and z[110] == pytest.approx(0.20)  # restart at the low, not the top
    assert z[130] == 0.0 and z[149] == pytest.approx(0.19)
    assert z[150:189] == pytest.approx(np.full(39, 0.195))  # entering height, up to the restart
    assert np.all(z[189:210] == 0.0) and z[259] == pytest.approx(0.495)


def test_position_nan_velocity_propagates_until_next_restart():
    n = 60
    t = _t(n)
    v = np.zeros((n, 3))
    v[:, 0] = 1.0
    v[25] = np.nan
    anchors = [Anchor(10, 10, "low"), Anchor(20, 20, "top_swing"), Anchor(40, 40, "low")]
    pos = segment_position(v, t, anchors)
    assert np.all(np.isfinite(pos[10:25])) and np.all(np.isnan(pos[25:40]))
    assert np.all(np.isfinite(pos[40:])) and pos[40, 0] == 0.0


# ----------------------------------------------------------------------------- power
def test_power_formula():
    a = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, -G], [np.nan, 0.0, 0.0]])
    v = np.array([[0.0, 0.0, 1.0], [3.0, 1.0, 5.0], [1.0, 0.0, 0.0]])
    p = power(a, v, 32.0)
    assert p[0] == pytest.approx(32.0 * G)  # lifting at 1 m/s against gravity
    assert p[1] == pytest.approx(32.0 * (3.0 + 2.0))
    assert np.isnan(p[2])


def test_concentric_power_is_work_over_duration():
    t = _t(101)
    P = 100.0 + 50.0 * t  # work over [0.2, 0.6] s = 100·0.4 + 25·(0.36 − 0.04) = 48 J
    assert concentric_power(P, t, 20, 60) == pytest.approx(48.0 / 0.4)


def test_concentric_power_degenerate_window_is_nan():
    t = _t(10)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert np.isnan(concentric_power(np.ones(10), t, 4, 4))
        assert np.isnan(concentric_power(np.ones(10), t, 5, 4))


# ----------------------------------------------------------------------------- refine_rests
def test_refine_rests_synthetic():
    n = 400
    pos = np.zeros((n, 3))
    pos[100:150, 2] = 1.5  # high "rest" after a low: a quiet lockout
    pos[200:250, 2] = -0.3  # back on the floor after a low
    pos[300:350, 2] = 2.0  # high, but no low before it in its set (rest, rest)
    anchors = [Anchor(0, 49, "rest"), Anchor(60, 60, "low"), Anchor(100, 149, "rest"),
               Anchor(170, 170, "low"), Anchor(200, 249, "rest"), Anchor(300, 349, "rest")]
    out = refine_rests(anchors, pos, AnchorConfig())
    assert [a.kind for a in out] == ["rest", "low", "top_lockout", "low", "rest", "rest"]
    assert [(a.i0, a.i1) for a in out] == [(a.i0, a.i1) for a in anchors]
    assert [a.kind for a in anchors][2] == "rest"  # input not modified
    pos[100:150, 2] = np.nan  # unknown height: not refined
    assert refine_rests(anchors, pos, AnchorConfig())[2].kind == "rest"


def _bump(t, t0, dur, dist):
    """Velocity sin² bump over [t0, t0 + dur] covering `dist` metres."""
    u = np.clip((t - t0) / dur, 0.0, 1.0)
    return 2.0 * dist / dur * np.sin(np.pi * u) ** 2


def test_refine_rests_second_quiet_rest_measured_from_the_low():
    """low -> quiet rest 1.5 m up -> +0.1 m -> quiet rest: once the first is refined, position
    restarts at its core end, so the second must add the first's height (1.6 m above the low)."""
    n = 260
    t = _t(n)
    v = np.zeros((n, 3))
    v[:, 2] = _bump(t, 0.10, 0.30, 1.5) + _bump(t, 1.20, 0.30, 0.1)
    anchors = [Anchor(10, 10, "low"), Anchor(40, 119, "rest"), Anchor(150, 229, "rest")]
    pos = segment_position(v, t, anchors)
    assert pos[79, 2] == pytest.approx(1.5, abs=1e-3)
    assert pos[189, 2] == pytest.approx(0.1, abs=1e-3)  # relative to the first rest's core end
    out = refine_rests(anchors, pos, AnchorConfig())
    assert [a.kind for a in out] == ["low", "top_lockout", "top_lockout"]
    lower = refine_rests(anchors, pos, AnchorConfig(floor_max_height_m=1.55))
    assert [a.kind for a in lower] == ["low", "rest", "rest"]  # 1.5 < 1.55, then set ended
