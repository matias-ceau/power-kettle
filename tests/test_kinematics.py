import dataclasses

import numpy as np
import pytest

from powerkettle import quat
from powerkettle.bell import BellProfile
from powerkettle.calibration import Calibration, calibrate
from powerkettle.imu import G, ImuData, scale_run
from powerkettle.kinematics import Kinematics, com_acceleration, savgol_derivative
from powerkettle.orientation import estimate_orientation
from powerkettle.simulate import Rest, Snatch, simulate

BELL = BellProfile()
FLIP_RATE = 4.0  # rad/s: |omega| above this marks the snatch flip phases


@pytest.fixture(scope="module")
def session():
    run, truth = simulate([Rest(3), Snatch(10, "R"), Rest(5)], BELL)
    imu = scale_run(run)
    calib = calibrate(imu, BELL)
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    return imu, truth, calib, q, bias


def truth_in_estimate_frame(truth, field):
    """Truth is in the simulator's world (athlete forward at `heading_deg`); the estimate's world
    has athlete forward = +x by construction of the calibration. Rotate truth about z."""
    heading = np.arctan2(truth.forward_w[1], truth.forward_w[0])
    q = quat.from_axis_angle([0.0, 0.0, 1.0], -heading)
    return quat.rotate(q, getattr(truth, field))


def test_savgol_derivative_of_sine_interior():
    dt = 0.0025
    t = np.arange(400) * dt
    d = savgol_derivative(np.sin(2 * np.pi * t), dt)
    assert d.shape == (400,)
    assert np.max(np.abs(d[10:-10] - 2 * np.pi * np.cos(2 * np.pi * t[10:-10]))) < 1e-3


def test_savgol_derivative_multicolumn_and_edges_exact_for_cubic():
    dt = 0.01
    t = np.arange(50) * dt
    x = np.stack([t**3, 2 * t**2 - t, np.full_like(t, 3.0)], axis=1)
    d = savgol_derivative(x, dt)
    expect = np.stack([3 * t**2, 4 * t - 1, np.zeros_like(t)], axis=1)
    assert d.shape == x.shape
    # a cubic is reproduced exactly by an order-3 fit; edges (shrunken window) are exact
    # wherever the shrunken window still supports order 3 (>= 4 points => half-width >= 2)
    assert np.allclose(d[2:-2], expect[2:-2], atol=1e-8)
    assert np.all(np.isfinite(d))


def test_savgol_derivative_nan_propagates_within_window_only():
    x = np.linspace(0.0, 1.0, 100)
    x[50] = np.nan
    d = savgol_derivative(x, 0.01)
    assert np.all(np.isnan(d[45:56]))
    assert np.all(np.isfinite(d[:45])) and np.all(np.isfinite(d[56:]))


def test_savgol_derivative_short_input():
    assert savgol_derivative(np.zeros((3, 3)), 0.01).shape == (3, 3)
    assert savgol_derivative(np.zeros((0, 3)), 0.01).shape == (0, 3)


def test_shapes_and_fnorm(session):
    imu, truth, calib, q, bias = session
    k = com_acceleration(imu, q, calib, bias)
    n = len(imu.t)
    assert isinstance(k, Kinematics)
    assert k.a_com.shape == k.omega_w.shape == k.alpha_w.shape == (n, 3)
    assert k.f_norm.shape == (n,)
    assert np.allclose(k.f_norm, np.linalg.norm(imu.acc, axis=1))


def test_com_acceleration_matches_truth(session):
    imu, truth, calib, q, bias = session
    k = com_acceleration(imu, q, calib, bias)
    a_true = truth_in_estimate_frame(truth, "acc")
    ok = imu.valid & np.all(np.isfinite(k.a_com), axis=1)
    assert ok.mean() > 0.99
    err = np.linalg.norm(k.a_com[ok] - a_true[ok], axis=1)
    rms = float(np.sqrt(np.mean(err**2)))
    assert rms < 0.3, f"a_com RMS error {rms:.3f} m/s^2"


def test_lever_arm_correction_matters_during_flips(session):
    imu, truth, calib, q, bias = session
    a_true = truth_in_estimate_frame(truth, "acc")
    flips = np.linalg.norm(truth.omega_w, axis=1) > FLIP_RATE
    assert flips.sum() > 20
    full = com_acceleration(imu, q, calib, bias)
    zero = com_acceleration(imu, q, dataclasses.replace(calib, lever_arm_s=np.zeros(3)), bias)
    ok = flips & imu.valid & np.all(np.isfinite(zero.a_com), axis=1)
    e_full = np.sqrt(np.mean(np.sum((full.a_com[ok] - a_true[ok]) ** 2, axis=1)))
    e_zero = np.sqrt(np.mean(np.sum((zero.a_com[ok] - a_true[ok]) ** 2, axis=1)))
    assert e_zero > 2.0, f"zero-lever error during flips {e_zero:.2f} m/s^2"
    assert e_full < e_zero / 3


def test_rest_gives_zero_acceleration(session):
    imu, truth, calib, q, bias = session
    k = com_acceleration(imu, q, calib, bias)
    seg = slice(int(0.5 * imu.fs), int(2.5 * imu.fs))
    assert np.max(np.abs(k.a_com[seg])) < 0.3


def test_default_bias_is_calibration_bias(session):
    imu, truth, calib, q, bias = session
    a = com_acceleration(imu, q, calib)
    b = com_acceleration(imu, q, calib, np.tile(calib.gyro_bias, (len(imu.t), 1)))
    assert np.allclose(a.a_com, b.a_com, equal_nan=True)


def test_invalid_samples_give_nan_not_invented_values():
    n, fs = 200, 400.0
    imu = ImuData(t=np.arange(n) / fs, acc=np.tile([0.0, 0.0, G], (n, 1)), gyro=np.zeros((n, 3)),
                  valid=np.ones(n, bool), saturated=np.zeros(n, bool), fs=fs, first_index=0)
    imu.valid[100] = False
    imu.acc[120] = np.nan
    imu.gyro[140] = np.nan
    calib = Calibration(np.zeros(3), np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]),
                        np.array([0.0, 0.1, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]),
                        slice(0, 10), G)
    q = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    k = com_acceleration(imu, q, calib, np.zeros((n, 3)))
    assert np.all(np.isnan(k.a_com[100])) and np.all(np.isnan(k.a_com[120]))
    assert np.all(np.isnan(k.a_com[140]))
    assert np.all(np.isfinite(k.a_com[:90])) and np.all(np.isfinite(k.a_com[150:]))
    assert np.allclose(k.a_com[:90], 0.0, atol=1e-9)

