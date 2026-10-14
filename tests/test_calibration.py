import numpy as np
import pytest

from powerkettle import quat
from powerkettle.bell import BellProfile, lever_arm_bell_m
from powerkettle.calibration import (
    Calibration,
    CalibrationConfig,
    CalibrationError,
    calibrate,
    find_still_window,
)
from powerkettle.imu import G, scale_run
from powerkettle.simulate import Rest, SimConfig, Swing, simulate

BELL = BellProfile()


@pytest.fixture(scope="module")
def session():
    run, truth = simulate([Rest(3), Swing(3), Rest(3)], BELL)
    return scale_run(run), truth


def test_still_window_is_first_rest(session):
    imu, truth = session
    win = find_still_window(imu, CalibrationConfig())
    assert win.start == 0
    assert (win.stop - win.start) / imu.fs >= 2.0
    assert win.stop <= int(3.05 * truth.fs_true)  # inside the first Rest


def test_calibration_matches_truth(session):
    imu, truth = session
    cal = calibrate(imu, BELL)
    assert isinstance(cal, Calibration)
    assert quat.angle_between(cal.up_s, truth.up_s) < np.radians(1.0)
    assert quat.angle_between(cal.out_s, truth.out_s) < np.radians(1.0)
    assert np.all(np.abs(cal.gyro_bias - truth.gyro_bias_s) < 0.003)
    assert np.linalg.norm(cal.up_s) == pytest.approx(1.0)
    assert np.linalg.norm(cal.out_s) == pytest.approx(1.0)
    assert abs(cal.up_s @ cal.out_s) < 1e-9
    assert cal.g_measured == pytest.approx(G, rel=0.01)
    assert cal.warnings == []
    assert cal.still == find_still_window(imu, CalibrationConfig())


def test_lever_arm_in_bell_frame(session):
    imu, truth = session
    cal = calibrate(imu, BELL)
    # bell axes (out, lateral = up x out, up) as columns give R_sb
    r_sb = np.stack([cal.out_s, np.cross(cal.up_s, cal.out_s), cal.up_s], axis=1)
    assert np.allclose(r_sb.T @ cal.lever_arm_s, lever_arm_bell_m(BELL), atol=0.003)
    assert np.allclose(cal.lever_arm_s, truth.lever_arm_s, atol=0.003)


def test_initial_orientation(session):
    imu, _ = session
    cal = calibrate(imu, BELL)
    assert np.linalg.norm(cal.q0) == pytest.approx(1.0)
    assert np.allclose(quat.rotate(cal.q0, cal.up_s), [0, 0, 1], atol=1e-9)
    assert np.allclose(quat.rotate(cal.q0, cal.out_s), [-1, 0, 0], atol=1e-9)
    # world frame is right-handed: sensor lateral (up x out) maps to world -y
    lateral = np.cross(cal.up_s, cal.out_s)
    assert np.allclose(quat.rotate(cal.q0, lateral), [0, -1, 0], atol=1e-9)


def test_other_normal_axis_and_sign():
    cfg = CalibrationConfig(normal_axis=0, normal_sign=-1)
    run, _ = simulate([Rest(3)], BELL, SimConfig(mount_angle_deg=0.0))
    imu = scale_run(run)
    # default sim: out = sensor +z, up = sensor y; asking for -x as outward gives out = -x
    # projected off up, i.e. -x (already perpendicular to y)
    cal = calibrate(imu, BELL, cfg)
    assert np.allclose(cal.out_s, [-1, 0, 0], atol=0.01)  # up_s carries ~0.3 deg of acc bias


def test_mid_swing_recording_has_no_still_window():
    run, _ = simulate([Swing(3)], BELL)
    imu = scale_run(run)
    with pytest.raises(CalibrationError, match="no still window of 2.0 s found"):
        find_still_window(imu, CalibrationConfig())
    with pytest.raises(CalibrationError):
        calibrate(imu, BELL)


def test_gaps_break_still_window(session):
    imu, _ = session
    n = int(2.5 * imu.fs)
    valid = imu.valid.copy()
    valid[n // 2] = False
    broken = type(imu)(imu.t, imu.acc, imu.gyro, valid, imu.saturated, imu.fs, imu.first_index)
    win = find_still_window(broken, CalibrationConfig())
    assert win.start > n // 2


def _scaled(imu, factor):
    return type(imu)(imu.t, imu.acc * factor, imu.gyro, imu.valid, imu.saturated, imu.fs,
                     imu.first_index)


def test_wrong_scale_warns_once(session):
    imu, _ = session
    cal = calibrate(_scaled(imu, 1.05), BELL)  # auto-detected window: stillness is scale-free
    assert len(cal.warnings) == 1
    assert cal.warnings[0].startswith("accelerometer magnitude off by")
    assert "5.0 %" in cal.warnings[0]
    assert cal.still == find_still_window(imu, CalibrationConfig())


def test_gross_scale_error_is_an_error(session):
    imu, _ = session
    with pytest.raises(CalibrationError, match="still but .* off g"):
        calibrate(_scaled(imu, 1.2), BELL)


def test_window_with_steady_but_changing_magnitude_is_not_still(session):
    imu, _ = session
    acc = imu.acc.copy()
    acc[: len(acc) // 2] *= np.linspace(1.0, 1.2, len(acc) // 2)[:, None]  # ramps 20 %
    ramped = type(imu)(imu.t, acc, imu.gyro, imu.valid, imu.saturated, imu.fs, imu.first_index)
    assert find_still_window(ramped, CalibrationConfig()).start > 0


def test_explicit_window_is_used(session):
    imu, _ = session
    cal = calibrate(imu, BELL, window=slice(100, 600))
    assert cal.still == slice(100, 600)
