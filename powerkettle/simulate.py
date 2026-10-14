"""Rigid-body kettlebell simulator with ground truth (spec §9.1).

``simulate(moves)`` renders scripted moves of the profiled bell into BMI160 readings (a
``rawlog.Run``) plus a ``Truth`` record that every analysis stage is validated against.
Stdlib + numpy only.

Frames
------
Athlete frame A: x forward, y left, z up, origin on the floor at the lifter.
World frame W = R_z(heading_deg) · A.
Bell frame B: x outward normal of the front flat (faces the lifter at the hike), z bell-up
(base -> handle), y = z × x.

Orientation (bell -> world)::

    R = R_z(heading) · R_y(θ) · R_z(ψ) · R_x(φ) · R_z(π)

ψ twists the bell about its own up axis (base -> handle), then θ tilts bell-up about +y, so θ
is the world sagittal tilt of bell-up whatever the twist (θ > 0 leans the handle forward;
at a snatch lockout, θ = +20°, the bell hangs behind the forearm with the handle 20° forward).
φ rolls about athlete x (non-zero only in ``LieOnSide``, where θ = ψ = 0). The constant R_z(π)
turns the front flat towards the lifter in the upright floor pose (bell x = −forward), so
forward = −horizontal(R·out).

Sensor frame S: ``sensor_from_bell(mount_angle_deg)`` gives R_S←B; sensor z = bell x
(outward normal of the box base) and, at mount angle 0, sensor x = bell y, sensor y = bell-up.

Kinematics are piecewise quintic between keyframes (COM position with its velocity and
acceleration; θ, ψ, φ with zero rate and acceleration at keyframes). Rotations are
rendered on a 4× oversampled grid; ω comes from consecutive rotations and α from
consecutive ω, then the grid is decimated to the sensor samples.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np

from . import quat
from .bell import BellProfile, lever_arm_bell_m
from .imu import G
from .protocol import STATUS_VERSION, Status
from .rawlog import MIN_RATE_SPAN_S, Run, estimate_rate
from .sim_motion import (
    Hold,
    LieOnSide,
    PartialRep,
    Rest,
    Script,
    Snatch,
    Swing,
    TableLift,
    evaluate,
    expand,
    orientation,
    quintic_coeffs,
)

__all__ = ["Hold", "LieOnSide", "PartialRep", "Rest", "SimConfig", "Snatch", "Swing",
           "TableLift", "Truth", "TruthRep", "crop", "sensor_from_bell", "simulate"]

ACC_RANGE_G = 16
GYRO_RANGE_DPS = 2000
OVERSAMPLE = 4
PACKET_SAMPLES = 19
STATUS_TIME0_US = 10_000_000  # device clock at the first sample (booted 10 s earlier)
PHONE_T0_MS = 1_760_400_000_000.0  # phone wall clock at the first sample
PHONE_LATENCY_MS = 30.0
TREMOR_RAMP_S = 0.1
G_W = np.array([0.0, 0.0, -G])


# ----------------------------------------------------------------------------- config / truth
@dataclass
class SimConfig:
    fs: float = 400.0  # nominal ODR, Hz
    clock_error: float = 0.005  # true rate = fs·(1 + clock_error)
    acc_noise: float = 0.02  # m/s², white, per axis
    gyro_noise: float = 0.005  # rad/s, white, per axis
    gyro_bias: tuple = (0.01, -0.02, 0.015)  # rad/s, sensor frame
    acc_bias: tuple = (0.05, -0.03, 0.04)  # m/s², sensor frame
    mount_angle_deg: float = 30.0  # in-plane rotation of the box on the front flat
    heading_deg: float = 37.0  # athlete forward in the world frame
    tremor_amp: float = 0.1  # m/s², hand-held static phases only
    tremor_hz: float = 8.0
    float_residual_speed: float = 0.0  # m/s, COM speed left at a swing top
    low_residual_speed: float = 0.0  # m/s, COM speed left at every backswing low (along the arc)
    acc_scale: float = 1.0  # accelerometer scale error: reads acc_scale × specific force (+ bias)
    right_hand_twist_sign: int = +1  # sign of ψ for the right hand's snatch twist
    seed: int = 0


@dataclass
class TruthRep:
    i_low: int
    i_top: int  # swing: top sample; snatch: first sample of the lockout window
    i_end: int  # next low
    kind: str  # "snatch" | "swing" | "other"
    hand: str  # "L" | "R" for snatches, "-" otherwise
    power_w: float  # mean concentric power over [i_low, i_top]
    peak_velocity: float  # max |v| over [i_low, i_top], m/s
    apex_m: float  # z(i_top) − z(i_low)
    i_top_end: int  # last sample of the lockout window (= i_top for swings and other)


@dataclass
class Truth:
    """Per-sample ground truth (index k at true time t = k / fs_true), world frame."""
    t: np.ndarray  # (N,) s
    pos: np.ndarray  # (N,3) COM position, m
    vel: np.ndarray  # (N,3) COM velocity
    acc: np.ndarray  # (N,3) COM acceleration (not specific force)
    quat: np.ndarray  # (N,4) world <- bell
    omega_w: np.ndarray  # (N,3) angular velocity, world frame
    power: np.ndarray  # (N,) m·(acc + g ẑ)·vel, W
    reps: list[TruthRep]
    sets: list[tuple[int, int]]  # (sample leaving the floor, sample back on the floor)
    anchors: list[tuple[int, str]]  # (index, kind): low | top_swing | top_lockout | rest (window start)
    anchor_windows: list[tuple[int, int, str]]  # (i0, i1 inclusive, kind); i0 == i1 for points
    fs_true: float  # true sample rate, Hz
    alpha_w: np.ndarray  # (N,3) angular acceleration, world
    a_sensor_w: np.ndarray  # (N,3) acceleration of the sensor point, world
    quat_ws: np.ndarray  # (N,4) world <- sensor
    theta: np.ndarray  # (N,) rad, world sagittal tilt of bell-up (+ = handle forward)
    psi: np.ndarray  # (N,) rad, twist about bell-up (snatch: ±π at lockout)
    phi: np.ndarray  # (N,) rad, roll about athlete x (LieOnSide only)
    tremor: np.ndarray  # (N,) bool, hand-held static (tremor applied)
    forward_w: np.ndarray  # (3,) athlete forward in the world frame
    R_sb: np.ndarray  # (3,3) sensor <- bell
    up_s: np.ndarray  # (3,) bell-up in the sensor frame
    out_s: np.ndarray  # (3,) bell outward in the sensor frame (= +z)
    lever_arm_b: np.ndarray  # (3,) COM -> sensor, bell frame
    lever_arm_s: np.ndarray  # (3,) COM -> sensor, sensor frame
    gyro_bias_s: np.ndarray  # (3,) rad/s
    acc_bias_s: np.ndarray  # (3,) m/s²


def sensor_from_bell(mount_angle_deg: float) -> np.ndarray:
    """R_S←B for a box rotated in-plane by mount angle α on the front flat.

    Rows are the sensor axes in bell coordinates::

        [[0,  cos α, sin α],    sensor x
         [0, −sin α, cos α],    sensor y
         [1,  0,     0    ]]    sensor z = bell outward

    so bell-up in the sensor frame is (sin α, cos α, 0).
    """
    a = math.radians(mount_angle_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[0.0, c, s], [0.0, -s, c], [1.0, 0.0, 0.0]])


def _q_bell_from_sensor(mount_angle_deg: float) -> np.ndarray:
    """Quaternion of R_B←S = (cyclic permutation: 120° about (1,1,1)) · R_z(α)."""
    perm = quat.from_axis_angle([1.0, 1.0, 1.0], 2.0 * math.pi / 3.0)
    return quat.mul(perm, quat.from_axis_angle([0.0, 0.0, 1.0], math.radians(mount_angle_deg)))


def _rates_from_rotations(q: np.ndarray, dt: float) -> np.ndarray:
    """World-frame rate between consecutive rotations: (M−1, 3), at interval midpoints."""
    dq = quat.mul(quat.conj(q[:-1]), q[1:])
    dq = np.where(dq[:, :1] < 0, -dq, dq)
    vn = np.linalg.norm(dq[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vn, dq[:, 0])
    k = np.where(vn > 1e-15, angle / np.where(vn > 1e-15, vn, 1.0), 2.0)
    return quat.rotate(q[:-1], dq[:, 1:] * (k / dt)[:, None])


# ----------------------------------------------------------------------------- rendering
def _window_mask(t: np.ndarray, windows: list[tuple[float, float]], ramp: float) -> np.ndarray:
    """Envelope in [0, 1]: 1 inside each window, raised-cosine ramps of `ramp` s at the edges."""
    env = np.zeros_like(t)
    for t0, t1 in windows:
        r = min(ramp, (t1 - t0) / 4.0)
        if r <= 0:
            continue
        u = np.clip(np.minimum(t - t0, t1 - t) / r, 0.0, 1.0)
        env = np.maximum(env, np.where((t >= t0) & (t <= t1), 0.5 - 0.5 * np.cos(np.pi * u), 0.0))
    return env


def _to_counts(x: np.ndarray, full_scale: float) -> np.ndarray:
    return np.clip(np.round(x * (32768.0 / full_scale)), -32768, 32767).astype(np.int16)


def _statuses(n: int, fs_true: float, fs_nominal: float) -> list[Status]:
    """1 Hz statuses on the true clock; sample_index = next sample the sensor will produce."""
    out = []
    t = 0.5
    while t * fs_true <= n - 1:
        out.append(Status(STATUS_VERSION, 1, 3900, round(fs_nominal), ACC_RANGE_G,
                          GYRO_RANGE_DPS, 0, 0, math.floor(t * fs_true + 1e-9) + 1,
                          STATUS_TIME0_US + round(t * 1e6)))
        t += 1.0
    return out


def simulate(moves: list, bell: BellProfile = BellProfile(),
             cfg: SimConfig = SimConfig()) -> tuple[Run, Truth]:
    """Render `moves` (starting from the bell upright on the floor) to a Run and its Truth."""
    sc = expand(moves, cfg.float_residual_speed, cfg.right_hand_twist_sign,
                cfg.low_residual_speed)
    if sc.t <= 0:
        raise ValueError("moves must have a positive total duration")
    times = np.array(sc.times)
    coeffs = quintic_coeffs(times, np.stack([p.state() for p in sc.poses]))
    fs_true = cfg.fs * (1.0 + cfg.clock_error)
    dt = 1.0 / fs_true
    n = math.floor(times[-1] * fs_true + 1e-9) + 1
    t = np.arange(n) * dt
    heading = math.radians(cfg.heading_deg)
    h = np.array([[math.cos(heading), -math.sin(heading), 0.0],
                  [math.sin(heading), math.cos(heading), 0.0], [0.0, 0.0, 1.0]])

    # COM kinematics (analytic) at the sample times, athlete -> world.
    x, dx, ddx = evaluate(times, coeffs, t)
    pos, vel, acc = x[:, :3] @ h.T, dx[:, :3] @ h.T, ddx[:, :3] @ h.T
    ang = x[:, 3:]

    # Rotations on the oversampled grid: fine node 1 + OVERSAMPLE·k is sample k.
    dt_f = dt / OVERSAMPLE
    t_f = (np.arange(OVERSAMPLE * (n - 1) + 3) - 1) * dt_f
    q_f = orientation(evaluate(times, coeffs, t_f, slice(3, 6), deriv=0), heading)
    w_mid = _rates_from_rotations(q_f, dt_f)
    node = 1 + OVERSAMPLE * np.arange(n)
    q = q_f[node]
    omega_w = 0.5 * (w_mid[node - 1] + w_mid[node])
    alpha_w = (w_mid[node] - w_mid[node - 1]) / dt_f

    # Sensor point acceleration and the readings.
    lever_b = lever_arm_bell_m(bell)
    r_w = quat.rotate(q, lever_b)
    a_s = acc + np.cross(alpha_w, r_w) + np.cross(omega_w, np.cross(omega_w, r_w))
    r_sb = sensor_from_bell(cfg.mount_angle_deg)
    q_conj = quat.conj(q)
    f_s = quat.rotate(q_conj, a_s - G_W) @ r_sb.T
    w_s = quat.rotate(q_conj, omega_w) @ r_sb.T

    rng = np.random.default_rng(cfg.seed)
    tremor_env = _window_mask(t, sc.tremor, TREMOR_RAMP_S)
    f_s = (cfg.acc_scale
           * (f_s + (cfg.tremor_amp * tremor_env * np.sin(2 * np.pi * cfg.tremor_hz * t))[:, None])
           + np.asarray(cfg.acc_bias, float) + cfg.acc_noise * rng.standard_normal((n, 3)))
    w_s = w_s + np.asarray(cfg.gyro_bias, float) + cfg.gyro_noise * rng.standard_normal((n, 3))
    raw = np.concatenate([_to_counts(f_s, ACC_RANGE_G * G),
                          _to_counts(np.degrees(w_s), float(GYRO_RANGE_DPS))], axis=1)

    statuses = _statuses(n, fs_true, cfg.fs)
    span = (statuses[-1].time_us - statuses[0].time_us) / 1e6 if statuses else 0.0
    rate_estimated = len(statuses) >= 2 and span >= MIN_RATE_SPAN_S
    first = np.arange(0, n, PACKET_SAMPLES)
    phone_ms = (PHONE_T0_MS + PHONE_LATENCY_MS
                + 1000.0 * (np.minimum(first + PACKET_SAMPLES, n) - 1) / fs_true)
    run = Run(np.arange(n, dtype=np.int64), raw, [], statuses, estimate_rate(statuses, cfg.fs),
              ACC_RANGE_G, GYRO_RANGE_DPS, rate_estimated=rate_estimated,
              packet_times=np.stack([phone_ms, first.astype(float)], axis=1))

    power = bell.mass_kg * np.sum((acc - G_W) * vel, axis=1)
    truth = Truth(
        t=t, pos=pos, vel=vel, acc=acc, quat=q, omega_w=omega_w, power=power,
        reps=[], sets=[], anchors=[], anchor_windows=[], fs_true=fs_true,
        alpha_w=alpha_w, a_sensor_w=a_s,
        quat_ws=quat.mul(q, _q_bell_from_sensor(cfg.mount_angle_deg)),
        theta=ang[:, 0], psi=ang[:, 1], phi=ang[:, 2], tremor=tremor_env > 0,
        forward_w=h[:, 0].copy(), R_sb=r_sb, up_s=r_sb[:, 2].copy(), out_s=r_sb[:, 0].copy(),
        lever_arm_b=lever_b, lever_arm_s=r_sb @ lever_b,
        gyro_bias_s=np.asarray(cfg.gyro_bias, float), acc_bias_s=np.asarray(cfg.acc_bias, float),
    )
    _fill_events(truth, sc, n)
    return run, truth


def crop(run: Run, truth: Truth, start_s: float) -> tuple[Run, Truth]:
    """`run` and `truth` from `start_s` (true time) on, as a recording that starts there (a
    reconnect mid-session): sample indices and times re-based to 0. Statuses and packet times
    before the cut are dropped (the rate is re-estimated from the rest); truth reps whose low
    precedes the cut and point anchors before it are dropped, a set or window the cut falls
    in starts at 0."""
    i0 = int(round(start_s * truth.fs_true))
    n = len(truth.t) - i0
    if not 0 <= i0 < len(truth.t):
        raise ValueError(f"start_s {start_s} outside the recording")
    statuses = [dataclasses.replace(s, sample_index=s.sample_index - i0)
                for s in run.statuses if s.sample_index > i0]
    span = (statuses[-1].time_us - statuses[0].time_us) / 1e6 if statuses else 0.0
    rate_estimated = len(statuses) >= 2 and span >= MIN_RATE_SPAN_S
    pt = run.packet_times[run.packet_times[:, 1] >= i0] - [0.0, i0]
    run2 = dataclasses.replace(
        run, sample_index=run.sample_index[i0:] - i0, raw=run.raw[i0:], gaps=[],
        statuses=statuses, rate_hz=estimate_rate(statuses, run.rate_hz) if rate_estimated
        else run.rate_hz, rate_estimated=rate_estimated, packet_times=pt)
    per_sample = {f.name: getattr(truth, f.name)[i0:] for f in dataclasses.fields(Truth)
                  if isinstance(getattr(truth, f.name), np.ndarray)
                  and getattr(truth, f.name).shape[:1] == (len(truth.t),)}
    per_sample["t"] = np.arange(n) / truth.fs_true
    reps = [dataclasses.replace(r, i_low=r.i_low - i0, i_top=r.i_top - i0, i_end=r.i_end - i0,
                                i_top_end=r.i_top_end - i0) for r in truth.reps if r.i_low >= i0]
    wins = [(max(0, a - i0), b - i0, k) for a, b, k in truth.anchor_windows if b >= i0]
    truth2 = dataclasses.replace(
        truth, **per_sample, reps=reps, anchor_windows=wins,
        anchors=[(a, k) for a, _, k in wins],
        sets=[(max(0, a - i0), b - i0) for a, b in truth.sets if b >= i0])
    return run2, truth2


def _fill_events(truth: Truth, sc: Script, n: int) -> None:
    fs = truth.fs_true

    def idx(t: float) -> int:
        return min(n - 1, round(t * fs))

    def window(t0: float, t1: float) -> tuple[int, int]:
        return min(n - 1, math.ceil(t0 * fs - 1e-9)), min(n - 1, math.floor(t1 * fs + 1e-9))

    wins: list[tuple[int, int, str]] = []
    rests: list[list[float]] = []
    for t0, t1 in sc.rests:  # merge back-to-back rests
        if rests and abs(rests[-1][1] - t0) < 1e-9:
            rests[-1][1] = t1
        else:
            rests.append([t0, t1])
    wins += [(*window(t0, t1), "rest") for t0, t1 in rests]

    lows: set[int] = set()
    for ev in sc.reps:
        i_low, i_end = idx(ev.t_low), idx(ev.t_end)
        lows.update((i_low, i_end))
        if ev.kind == "snatch":
            i_top, i_top_end = window(ev.t_top, ev.t_top_end)
            wins.append((i_top, i_top_end, "top_lockout"))
        else:
            i_top = i_top_end = idx(ev.t_top)
            wins.append((i_top, i_top, "top_swing"))
        con = slice(i_low, i_top + 1)
        p_con, t_con = truth.power[con], truth.t[con]
        work = np.sum(0.5 * (p_con[1:] + p_con[:-1]) * np.diff(t_con))
        truth.reps.append(TruthRep(
            i_low=i_low, i_top=i_top, i_end=i_end, kind=ev.kind, hand=ev.hand,
            power_w=float(work / (truth.t[i_top] - truth.t[i_low])),
            peak_velocity=float(np.max(np.linalg.norm(truth.vel[con], axis=1))),
            apex_m=float(truth.pos[i_top, 2] - truth.pos[i_low, 2]), i_top_end=i_top_end))
    wins += [(i, i, "low") for i in lows]
    wins.sort()
    truth.anchor_windows = wins
    truth.anchors = [(i0, kind) for i0, _, kind in wins]
    truth.sets = [(idx(t0), idx(t1)) for t0, t1 in sc.sets]
