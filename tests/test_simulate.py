import math

import numpy as np
import pytest

from powerkettle import quat
from powerkettle.bell import BellProfile, lever_arm_bell_m
from powerkettle.imu import G, scale_run
from powerkettle.rawlog import estimate_rate
from powerkettle.simulate import (
    Hold,
    LieOnSide,
    PartialRep,
    Rest,
    SimConfig,
    Snatch,
    Swing,
    TableLift,
    sensor_from_bell,
    simulate,
)

QUIET = {"acc_noise": 0.0, "gyro_noise": 0.0, "gyro_bias": (0, 0, 0), "acc_bias": (0, 0, 0),
         "tremor_amp": 0.0}
G_W = np.array([0.0, 0.0, -G])


def quiet(**kw):
    return SimConfig(**{**QUIET, **kw})


def windows(truth, kind):
    return [(i0, i1) for i0, i1, k in truth.anchor_windows if k == kind]


def test_static_reads_gravity():
    run, truth = simulate([Rest(3), Swing(2), Rest(3)], cfg=quiet())
    imu = scale_run(run)
    i0, i1 = windows(truth, "rest")[0]
    assert i0 == 0 and i1 >= int(2.9 * truth.fs_true)
    norm = np.linalg.norm(imu.acc[i0:i1 + 1], axis=1)
    assert np.all(np.abs(norm - 9.80665) < 0.01)
    assert np.all(np.linalg.norm(imu.gyro[i0:i1 + 1], axis=1) < 0.003)


def test_energy_conservation():
    moves = [Rest(3), Snatch(3, "R"), Hold(2), Rest(2), Swing(3), PartialRep(), Rest(2)]
    _, truth = simulate(moves, cfg=quiet())
    m = BellProfile().mass_kg
    t = truth.t
    work = np.concatenate([[0.0], np.cumsum(0.5 * (truth.power[1:] + truth.power[:-1]) * np.diff(t))])
    energy = 0.5 * m * np.sum(truth.vel**2, axis=1) + m * G * truth.pos[:, 2]
    d_energy = energy - energy[0]
    scale = np.max(np.abs(d_energy))
    assert scale > m * G * 1.5  # the snatch lifts the bell ~1.6 m
    assert np.max(np.abs(work - d_energy)) <= 0.01 * scale
    assert abs(work[-1] - d_energy[-1]) <= 0.01 * scale


def test_sensor_matches_differentiated_position():
    bell = BellProfile()
    run, truth = simulate([Rest(1), Snatch(2, "L"), Swing(2), Rest(1)], bell, quiet())
    imu = scale_run(run)
    p_s = truth.pos + quat.rotate(truth.quat, lever_arm_bell_m(bell))
    dt = 1.0 / truth.fs_true
    a_fd = (p_s[2:] - 2.0 * p_s[1:-1] + p_s[:-2]) / dt**2
    a_rendered = quat.rotate(truth.quat_ws, imu.acc) + G_W
    err = a_fd - a_rendered[1:-1]
    rms = math.sqrt(np.mean(np.sum(err**2, axis=1)))
    assert rms < 0.05
    # the lever-arm terms are not negligible, so the check above is meaningful
    lever = truth.a_sensor_w - truth.acc
    assert np.max(np.linalg.norm(lever, axis=1)) > 5.0


def test_rate_recoverable():
    run, _ = simulate([Rest(3), Swing(5), Rest(3)])
    assert estimate_rate(run.statuses, 400) == pytest.approx(402.0, rel=1e-4)
    assert run.rate_estimated and run.rate_hz == pytest.approx(402.0, rel=1e-4)
    assert all(s.flags & 1 for s in run.statuses)
    assert all(0 <= s.sample_index <= run.sample_index[-1] + 1 for s in run.statuses)
    pt = run.packet_times
    assert pt.shape[1] == 2 and np.all(np.diff(pt[:, 1]) == 19) and np.all(np.diff(pt[:, 0]) > 0)


def test_truth_reps():
    _, truth = simulate([Rest(3), Snatch(5, "L"), Rest(5), Swing(10), Rest(3)])
    assert len(truth.reps) == 15
    assert [r.kind for r in truth.reps] == ["snatch"] * 5 + ["swing"] * 10
    assert [r.hand for r in truth.reps] == ["L"] * 5 + ["-"] * 10
    assert len(truth.sets) == 2
    for r in truth.reps[:5]:
        assert r.apex_m == pytest.approx(1.6, abs=0.01)
        assert r.i_low < r.i_top < r.i_top_end < r.i_end
    for r in truth.reps[5:]:
        assert r.apex_m == pytest.approx(1.0, abs=0.01)
        assert r.i_top == r.i_top_end
    # concentric power: m g apex / concentric duration (zero velocity at both ends)
    m = BellProfile().mass_kg
    assert truth.reps[0].power_w == pytest.approx(m * G * 1.6 / 0.75, rel=0.01)
    assert truth.reps[5].power_w == pytest.approx(m * G * 1.0 / 0.55, rel=0.01)
    (s0, e0), (s1, _) = truth.sets
    assert s0 < truth.reps[0].i_low and truth.reps[4].i_end < e0 < s1 < truth.reps[5].i_low


def test_sensor_frame_conventions():
    cfg = SimConfig()
    r_sb = sensor_from_bell(cfg.mount_angle_deg)
    assert np.allclose(r_sb @ r_sb.T, np.eye(3)) and np.linalg.det(r_sb) == pytest.approx(1.0)
    assert np.allclose(r_sb @ [1, 0, 0], [0, 0, 1])  # sensor z = bell outward
    a = math.radians(cfg.mount_angle_deg)
    assert np.allclose(r_sb @ [0, 0, 1], [math.sin(a), math.cos(a), 0])  # bell-up in sensor
    run, truth = simulate([Rest(3), Snatch(1, "R"), Rest(1)], cfg=quiet())
    assert np.allclose(truth.R_sb, r_sb)
    assert np.allclose(truth.out_s, [0, 0, 1])
    assert np.allclose(truth.up_s, r_sb @ [0, 0, 1])
    assert np.allclose(truth.lever_arm_s, r_sb @ lever_arm_bell_m(BellProfile()))
    # quat_ws = quat (bell->world) composed with sensor->bell
    assert np.allclose(quat.to_matrix(truth.quat_ws), quat.to_matrix(truth.quat) @ r_sb.T, atol=1e-12)
    # floor rest: mean accel points along bell-up; outward faces the lifter (forward = -out)
    imu = scale_run(run)
    f = imu.acc[:1000].mean(axis=0)
    assert quat.angle_between(f, truth.up_s) < math.radians(0.05)
    out_w = quat.rotate(truth.quat_ws[0], truth.out_s)
    h = math.radians(cfg.heading_deg)
    assert np.allclose(truth.forward_w, [math.cos(h), math.sin(h), 0])
    assert np.allclose(-out_w, truth.forward_w, atol=1e-9)
    # biases are sensor-frame: the default-config rest gyro mean is the configured bias
    run_d, truth_d = simulate([Rest(5)])
    assert np.allclose(truth_d.gyro_bias_s, cfg.gyro_bias) and np.allclose(truth_d.acc_bias_s, cfg.acc_bias)
    assert np.allclose(scale_run(run_d).gyro.mean(axis=0), cfg.gyro_bias, atol=5e-4)


def test_pose_conventions():
    cfg = quiet()
    _, truth = simulate([Rest(1), Swing(1), Snatch(1, "L"), Rest(1)], cfg=cfg)
    fwd = truth.forward_w
    up_w = quat.rotate(truth.quat, [0.0, 0.0, 1.0])
    swing, snatch = truth.reps
    # low: handle leans forward (theta = +40 deg)
    assert up_w[swing.i_low] @ fwd == pytest.approx(math.sin(math.radians(40)), abs=1e-6)
    # swing top: bell-up horizontal, handle pointing back at the lifter
    assert np.allclose(up_w[swing.i_top], -fwd, atol=1e-6)
    # snatch lockout: handle up within 20 deg; twist -180 deg for the left hand
    assert up_w[snatch.i_top, 2] == pytest.approx(math.cos(math.radians(20)), abs=1e-6)
    assert truth.psi[snatch.i_top] == pytest.approx(-math.pi)
    mid = (snatch.i_low + snatch.i_top) // 2
    assert truth.psi[mid] < -math.radians(30)
    _, truth_r = simulate([Rest(1), Snatch(1, "R"), Rest(1)], cfg=cfg)
    assert truth_r.psi[truth_r.reps[0].i_top] == pytest.approx(math.pi)
    _, truth_flip = simulate([Rest(1), Snatch(1, "R"), Rest(1)], cfg=quiet(right_hand_twist_sign=-1))
    assert truth_flip.psi[truth_flip.reps[0].i_top] == pytest.approx(-math.pi)


def test_gyro_integrates_to_truth_orientation():
    """Gyro = instantaneous body rates in the sensor frame (quat.integrate convention).

    Integrating the mean of consecutive readings tracks quat_ws to < 0.05 deg (measured
    0.008 deg). Rectangular integration of single readings lags by |omega|·dt/2 (~0.7 deg
    peak at 10 rad/s) but returns to < 0.01 deg at every rep end (no drift).
    """
    run, truth = simulate([Rest(1), Snatch(3, "R"), Swing(3), Rest(1)], cfg=quiet())
    gyro = scale_run(run).gyro
    w = 0.5 * (gyro[:-1] + gyro[1:])
    q = np.empty_like(truth.quat_ws)
    q[0] = truth.quat_ws[0]
    dt = 1.0 / truth.fs_true
    for k in range(len(q) - 1):
        q[k + 1] = quat.integrate(q[k], w[k], dt)
    err = 2.0 * np.arccos(np.clip(np.abs(np.sum(q * truth.quat_ws, axis=1)), 0.0, 1.0))
    assert np.max(err) < math.radians(0.05)


def test_omega_and_alpha_truth():
    _, truth = simulate([Rest(1), Snatch(2, "R"), Rest(1)], cfg=quiet())
    dt = 1.0 / truth.fs_true
    # omega_w from the truth rotations: R(t+dt) R(t)^T ~ exp([omega dt])
    m = quat.to_matrix(truth.quat)
    d = m[2:] @ np.transpose(m[:-2], (0, 2, 1))
    w_fd = np.stack([d[:, 2, 1] - d[:, 1, 2], d[:, 0, 2] - d[:, 2, 0], d[:, 1, 0] - d[:, 0, 1]], axis=1)
    w_fd /= 2.0 * (2.0 * dt)
    assert np.max(np.abs(w_fd - truth.omega_w[1:-1])) < 0.05
    assert np.max(np.linalg.norm(truth.omega_w, axis=1)) > 5.0
    a_fd = (truth.omega_w[2:] - truth.omega_w[:-2]) / (2.0 * dt)
    assert np.sqrt(np.mean((a_fd - truth.alpha_w[1:-1]) ** 2)) < 0.5


def test_tremor_only_when_hand_held():
    cfg = SimConfig(acc_noise=0.0, gyro_noise=0.0)
    moves = [Rest(3), Snatch(1, "R"), Hold(3), LieOnSide(4), TableLift(0.74), Rest(3)]
    run, truth = simulate(moves, cfg=cfg)
    imu = scale_run(run)
    lockouts = windows(truth, "top_lockout")
    rests = windows(truth, "rest")
    assert len(lockouts) == 1 and len(rests) == 4  # floor, side, table, floor
    i0, i1 = lockouts[0]
    assert (i1 - i0) / truth.fs_true == pytest.approx(0.4 + 3.0, abs=0.01)
    assert truth.tremor[i0:i1].mean() > 0.9 and truth.tremor.sum() <= i1 - i0 + 1
    norm = np.linalg.norm(imu.acc, axis=1)
    mid = slice(i0 + 100, i1 - 100)
    assert np.std(norm[mid]) > 0.05
    for r0, r1 in rests:
        assert not truth.tremor[r0:r1 + 1].any()
        assert np.std(norm[r0:r1 + 1]) < 0.005
    # lying on its side: bell-up horizontal and static
    s0, s1 = rests[1]
    up_w = quat.rotate(truth.quat[s0:s1 + 1], [0.0, 0.0, 1.0])
    assert np.max(np.abs(up_w[:, 2])) < 1e-9
    assert np.max(np.linalg.norm(truth.vel[s0:s1 + 1], axis=1)) < 1e-12
    # table: 0.74 m above the floor rest
    t0, _ = rests[2]
    assert truth.pos[t0, 2] - truth.pos[0, 2] == pytest.approx(0.74)


def test_hold_must_follow_snatch():
    with pytest.raises(ValueError, match="Hold"):
        simulate([Rest(3), Swing(2), Hold(2)])
