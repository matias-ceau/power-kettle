import dataclasses
import time

import numpy as np
import pytest

from powerkettle import quat
from powerkettle.bell import BellProfile
from powerkettle.calibration import calibrate
from powerkettle.imu import G, ImuData, scale_run
from powerkettle.orientation import estimate_orientation, static_mask
from powerkettle.simulate import Rest, Snatch, Swing, simulate

BELL = BellProfile()
Z = np.array([0.0, 0.0, 1.0])


def tilt_error(q_est, q_true):
    """Angle between estimated and true world-z expressed in the sensor frame."""
    return quat.angle_between(quat.rotate(quat.conj(q_est), Z), quat.rotate(quat.conj(q_true), Z))


@pytest.fixture(scope="module")
def session():
    run, truth = simulate([Rest(3), Snatch(10, "R"), Rest(10), Swing(10), Rest(3)], BELL)
    imu = scale_run(run)
    return imu, truth, calibrate(imu, BELL)


def synthetic(n, fs=400.0, acc=(0.0, 0.0, G), gyro=(0.0, 0.0, 0.0)):
    return ImuData(
        t=np.arange(n) / fs,
        acc=np.tile(np.asarray(acc, float), (n, 1)),
        gyro=np.tile(np.asarray(gyro, float), (n, 1)),
        valid=np.ones(n, bool), saturated=np.zeros(n, bool), fs=fs, first_index=0)


def test_tilt_error_under_2_degrees(session):
    imu, truth, cal = session
    q = estimate_orientation(imu, cal)
    assert q.shape == (len(imu.t), 4)
    assert len(imu.t) == len(truth.quat_ws)
    err = np.degrees(tilt_error(q, truth.quat_ws))
    assert err.max() < 2.0, f"max tilt error {err.max():.2f} deg"


def test_tilt_error_with_gyro_bias_error(session):
    imu, truth, cal = session
    bad = dataclasses.replace(cal, gyro_bias=cal.gyro_bias + 0.005)
    q = estimate_orientation(imu, bad)
    err = np.degrees(tilt_error(q, truth.quat_ws))
    assert err.max() < 3.0, f"max tilt error {err.max():.2f} deg"


def test_bias_reestimated_after_long_rest(session):
    imu, truth, cal = session
    bad = dataclasses.replace(cal, gyro_bias=cal.gyro_bias + 0.005)
    q, bias = estimate_orientation(imu, bad, return_bias=True)
    assert bias.shape == (len(imu.t), 3)
    fs = imu.fs
    # the first still window already re-estimates; after the Rest(10) it is within 0.002 of truth
    after_rest = bias[int(36 * fs)]  # Rest(10) ends ~24.3 s; the swing set starts ~34.3 s
    assert np.all(np.abs(after_rest - truth.gyro_bias_s) < 0.002)
    assert np.all(np.abs(bias[-1] - truth.gyro_bias_s) < 0.002)
    # no long static window -> the calibration bias is kept
    q2, bias2 = estimate_orientation(imu, bad, bias_update_min_s=1e6, return_bias=True)
    assert np.allclose(bias2, bad.gyro_bias)
    assert np.allclose(q2, estimate_orientation(imu, bad, bias_update_min_s=1e6))


def test_midsession_bias_change_is_absorbed(session):
    imu, truth, cal = session
    start = int(3.0 * imu.fs)  # the snatch set starts
    gyro = imu.gyro.copy()
    gyro[start:] += 0.005
    shifted = dataclasses.replace(imu, gyro=gyro)
    q, bias = estimate_orientation(shifted, cal, return_bias=True)
    err = np.degrees(tilt_error(q, truth.quat_ws))
    assert err.max() < 3.0, f"max tilt error {err.max():.2f} deg"
    new_true = truth.gyro_bias_s + 0.005
    assert np.all(np.abs(bias[int(36 * imu.fs)] - new_true) < 0.002)  # after Rest(10)
    assert np.all(np.abs(bias[1] - truth.gyro_bias_s) < 0.002)


def test_slow_rotation_window_not_absorbed_as_bias():
    fs, rate = 400.0, 0.1
    rng = np.random.default_rng(3)
    true_bias = np.array([0.01, -0.02, 0.015])
    segs = [(int(3 * fs), 0.0, G), (int(2 * fs), rate, G), (int(5 * fs), 0.0, 2 * G)]
    n = sum(s[0] for s in segs)
    imu = synthetic(n)
    w = np.concatenate([np.full(c, r) for c, r, _ in segs])
    imu.acc[:, 2] = np.concatenate([np.full(c, a) for c, _, a in segs])  # last set: not static
    imu.gyro[:, 2] = w
    imu.gyro += true_bias + 0.005 * rng.standard_normal((n, 3))
    cal = types_cal(true_bias, np.array([1.0, 0.0, 0.0, 0.0]))
    q, bias = estimate_orientation(imu, cal, return_bias=True)
    assert np.all(np.abs(bias[-1] - true_bias) < 0.002)
    # total rotation about z is the true 0.1 rad/s x 2 s, not inflated by an absorbed bias
    yaw = 2.0 * np.arctan2(q[-1, 3], q[-1, 0])
    assert abs(yaw - rate * 2.0) < np.radians(1.0)
    # control: the std/jump checks are what protect it
    _, loose = estimate_orientation(imu, cal, bias_max_std=1.0, bias_max_jump=1.0, return_bias=True)
    assert abs(loose[-1, 2] - true_bias[2]) > 0.03


def test_bias_update_ignores_invalid_samples():
    n = 1600
    imu = synthetic(n, gyro=(0.02, 0.0, 0.0))
    imu.gyro[100:200] = np.nan
    imu.valid[100:200] = False  # splits the run: [0,100) too short, [200,1600) is 3.5 s
    imu.gyro[1400:1420] = np.nan  # non-finite gyro also ends a static run
    cal = types_cal(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    q, bias = estimate_orientation(imu, cal, return_bias=True)
    assert np.allclose(bias[300], (0.02, 0.0, 0.0))
    assert np.allclose(bias[0], 0.0)  # window before the gap lasts < 1 s: no update
    assert np.isfinite(q).all()


def test_static_mask_rest_vs_motion(session):
    imu, truth, cal = session
    m = static_mask(imu)
    assert m.dtype == bool and m.shape == (len(imu.t),)
    fs = imu.fs
    assert m[int(0.5 * fs): int(2.9 * fs)].all()  # first rest
    assert m[-int(2.5 * fs): -int(0.2 * fs)].all()  # last rest
    # a sample flagged static is never one with large true angular rate
    w = np.linalg.norm(truth.omega_w, axis=1)
    assert w[m].max() < 0.4


def test_static_mask_needs_hold_time():
    imu = synthetic(400)
    imu.acc[100:120] = (0.0, 0.0, 2 * G)  # 50 ms excursion splits the run
    imu.acc[300:340] = (0.0, 0.0, 2 * G)
    m = static_mask(imu)
    assert not m[100:120].any() and not m[300:340].any()
    assert m[0:100].all() and m[120:300].all() and m[340:].all()
    # a short static island between motion is rejected (< 0.1 s = 40 samples)
    imu.acc[:] = (0.0, 0.0, 2 * G)
    imu.acc[200:230] = (0.0, 0.0, G)
    imu.acc[300:350] = (0.0, 0.0, G)
    m = static_mask(imu)
    assert not m[200:230].any()
    assert m[300:350].all()


def test_static_mask_excludes_invalid_and_nan():
    imu = synthetic(400)
    imu.valid[150:200] = False
    imu.gyro[300] = np.nan
    imu.acc[300] = np.nan
    m = static_mask(imu)
    assert not m[150:200].any() and not m[300]


def test_gravity_correction_levels_tilt_without_injecting_yaw():
    n = 2000
    tilt = np.radians(10.0)
    # sensor tilted: world z in sensor frame is (sin t, 0, cos t); truth orientation q_true
    q_true = quat.from_axis_angle([0.0, 1.0, 0.0], -tilt)
    f = quat.rotate(quat.conj(q_true), Z) * G
    imu = synthetic(n, acc=f)
    cal = types_cal(np.zeros(3), quat.from_axis_angle(Z, 0.7))  # level but heading 0.7
    q = estimate_orientation(imu, cal)
    assert np.degrees(tilt_error(q[0], q_true)) == pytest.approx(10.0, abs=0.01)
    assert np.degrees(tilt_error(q[-1], q_true)) < 0.1  # converged in 5 s at 2 rad/s gain
    # world-frame correction rotation q_end (x) conj(q_start) is about a horizontal axis
    d = quat.mul(q[-1], quat.conj(q[0]))
    assert abs(d[3]) < 1e-9
    # and it is monotone towards the measurement
    err = tilt_error(q, q_true)
    assert np.all(np.diff(err) < 1e-9)


def test_gravity_correction_only_where_static():
    n = 1600
    tilt = np.radians(10.0)
    q_true = quat.from_axis_angle([0.0, 1.0, 0.0], -tilt)
    f = quat.rotate(quat.conj(q_true), Z) * G
    imu = synthetic(n, acc=f)
    imu.acc[400:800] *= 2.0  # accelerating: not static
    cal = types_cal(np.zeros(3), quat.from_axis_angle(Z, 0.0))
    q = estimate_orientation(imu, cal)
    assert np.allclose(q[400], q[799], atol=1e-12)  # no gyro, no correction
    assert tilt_error(q[1199], q_true) < tilt_error(q[799], q_true)


def test_invalid_samples_hold_orientation():
    n = 1000
    imu = synthetic(n, gyro=(0.0, 0.0, 1.0), acc=(0.0, 0.0, 3 * G))  # rotating, never static
    imu.valid[300:400] = False
    imu.gyro[350] = np.nan
    cal = types_cal(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    q = estimate_orientation(imu, cal)
    assert np.isfinite(q).all()
    assert np.allclose(q[299:400], q[299])
    assert np.allclose(q[400], q[399])  # the step out of the gap touches an invalid sample
    assert not np.allclose(q[401], q[400])


def test_integration_exact_and_bias_removed():
    n = 4001
    imu = synthetic(n, gyro=(0.3, -0.2, 0.9), acc=(0.0, 0.0, 3 * G))
    bias = np.array([0.01, 0.02, -0.03])
    imu.gyro = imu.gyro + bias
    cal = types_cal(bias, np.array([1.0, 0.0, 0.0, 0.0]))
    q = estimate_orientation(imu, cal)
    w = np.array([0.3, -0.2, 0.9])
    t_end = (n - 1) / imu.fs
    expect = quat.from_axis_angle(w, np.linalg.norm(w) * t_end)
    assert quat.angle_between(quat.rotate(q[-1], [1, 0, 0]), quat.rotate(expect, [1, 0, 0])) < 1e-9
    assert np.allclose(np.linalg.norm(q, axis=1), 1.0)


def test_midpoint_rule_on_changing_rate():
    n = 801
    imu = synthetic(n, acc=(0.0, 0.0, 3 * G))
    imu.gyro[:, 0] = np.linspace(0.0, 4.0, n)  # linear ramp: trapezoid is exact
    cal = types_cal(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    q = estimate_orientation(imu, cal)
    total = 0.5 * 4.0 * (n - 1) / imu.fs
    assert quat.angle_between(quat.rotate(q[-1], [0, 1, 0]),
                              quat.rotate(quat.from_axis_angle([1, 0, 0], total), [0, 1, 0])) < 1e-9


def test_ten_minute_session_runtime():
    n = 240_000
    rng = np.random.default_rng(1)
    imu = synthetic(n, acc=(0.0, 0.0, G))
    imu.gyro = rng.standard_normal((n, 3)) * 0.05
    imu.gyro[::2000] += 5.0
    imu.acc[100_000:140_000] *= 2.0  # long non-static stretch
    cal = types_cal(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    t0 = time.perf_counter()
    q = estimate_orientation(imu, cal)
    assert time.perf_counter() - t0 < 10.0
    assert np.isfinite(q).all()


def types_cal(bias, q0):
    from powerkettle.calibration import Calibration
    return Calibration(gyro_bias=np.asarray(bias, float), up_s=Z.copy(), out_s=np.array([1.0, 0, 0]),
                       lever_arm_s=np.zeros(3), q0=np.asarray(q0, float), still=slice(0, 800),
                       g_measured=G)
