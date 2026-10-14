"""Scripted kettlebell motion for the simulator: moves, keyframes, quintic segments.

Internal to ``powerkettle.simulate`` (which re-exports the move classes). Stdlib + numpy only.

Keyframes are in the athlete frame (x forward, y left, z up): COM position with velocity and
acceleration, plus pose angles θ, ψ, φ (degrees in ``Pose``, zero rate and acceleration at
keyframes). Between keyframes every coordinate follows a quintic. ``orientation`` builds
the bell -> world rotation

    R = R_z(heading) · R_y(θ) · R_z(ψ) · R_x(φ) · R_z(π)

(twist ψ about the bell's own up axis first, then the sagittal tilt θ: θ is always the world
sagittal tilt of bell-up and θ > 0 leans the handle forward; φ rolls about athlete x for
``LieOnSide`` only; R_z(π) makes the front flat face the lifter in the upright floor pose).
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np

from . import quat


# ----------------------------------------------------------------------------- moves
@dataclass(frozen=True)
class Rest:
    """Bell standing upright on the floor, untouched; parked turned by `yaw_deg` about its own
    up axis (ψ; 180 = front flat facing away from the lifter). The bell is turned while it is
    set down (or, after another rest, in a 1 s turn on the floor; for the session's first Rest,
    from the start) and turned back on the way to the next low."""
    seconds: float
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Swing:
    n: int
    apex_m: float = 1.0
    cadence_s: float = 1.2


MIN_LOCKOUT_S = 0.05


@dataclass(frozen=True)
class Snatch:
    n: int
    hand: str = "R"
    apex_m: float = 1.6
    lockout_s: float = 0.4  # at least MIN_LOCKOUT_S


@dataclass(frozen=True)
class Hold:
    """Overhead hold: extends the lockout of the last rep of the directly preceding Snatch."""
    seconds: float


@dataclass(frozen=True)
class TableLift:
    """Slow lift floor -> table (2 s), resting on the table (2 s), back down (2 s)."""
    height_m: float


@dataclass(frozen=True)
class PartialRep:
    """One swing-timed rep to a low apex with the handle only 40° back (classifies as other)."""
    apex_m: float = 0.4


@dataclass(frozen=True)
class LieOnSide:
    """Bell tipped 90° onto its side (bell-up horizontal), static; 1 s transitions each way."""
    seconds: float


# ----------------------------------------------------------------------------- motion script
@dataclass(frozen=True)
class Pose:
    """Keyframe in the athlete frame. Angles in degrees; their rates are zero at keyframes."""
    pos: tuple
    vel: tuple = (0.0, 0.0, 0.0)
    acc: tuple = (0.0, 0.0, 0.0)
    theta: float = 0.0
    psi: float = 0.0
    phi: float = 0.0

    def state(self) -> np.ndarray:
        """(3, 6): rows value / rate / acceleration; columns x, y, z, θ, ψ, φ (rad)."""
        ang = np.radians([self.theta, self.psi, self.phi])
        return np.array([[*self.pos, *ang], [*self.vel, 0, 0, 0], [*self.acc, 0, 0, 0]], float)


FLOOR = Pose((0.45, 0.0, 0.104))
LOW = Pose((-0.30, 0.0, 0.40), acc=(15.0, 0.0, 5.0), theta=40.0)
LOW_ARC = (0.96, 0.0, -0.28)  # unit tangent of the arc at the low, towards the next pull
LIE = Pose((0.45, -0.024, 0.100), phi=90.0)  # on its side: COM at ~ball radius, offset sideways


def _low(residual: float) -> Pose:
    """The backswing low, with `residual` speed (m/s) left along the arc (LOW_ARC)."""
    return dataclasses.replace(LOW, vel=tuple(residual * c for c in LOW_ARC))


def _swing_top(apex: float, residual: float, theta: float) -> Pose:
    return Pose((0.60, 0.0, 0.40 + apex), (-0.6 * residual, 0.0, -0.8 * residual),
                (0.0, 0.0, -8.8), theta=theta)


def _snatch_poses(apex: float, sgn: float) -> tuple[Pose, Pose, Pose]:
    z_mid = 0.40 + 0.6 * apex
    up = Pose((0.20, 0.0, z_mid), (0.3, 0.0, 2.5), (0.0, 0.0, -3.0), theta=-30.0, psi=90.0 * sgn)
    lock = Pose((0.05, 0.0, 0.40 + apex), theta=20.0, psi=180.0 * sgn)
    drop = Pose((0.20, 0.0, z_mid), (-0.3, 0.0, -3.0), (0.0, 0.0, -3.0), theta=-30.0, psi=90.0 * sgn)
    return up, lock, drop


@dataclass
class RepEvent:
    t_low: float
    t_top: float
    t_top_end: float
    t_end: float
    kind: str
    hand: str


class Script:
    """Accumulates keyframes and event times (seconds) while expanding the moves."""

    def __init__(self, low: Pose = LOW) -> None:
        self.low = low
        self.t = 0.0
        self.times = [0.0]
        self.poses = [FLOOR]
        self.rests: list[tuple[float, float]] = []
        self.tremor: list[tuple[float, float]] = []
        self.reps: list[RepEvent] = []
        self.sets: list[tuple[float, float]] = []

    def to(self, dur: float, pose: Pose) -> None:
        self.t += dur
        self.times.append(self.t)
        self.poses.append(pose)

    def stay(self, dur: float) -> None:
        if dur > 0:
            self.to(dur, self.poses[-1])

    def rest(self, dur: float) -> None:
        if dur > 0:
            self.rests.append((self.t, self.t + dur))
            self.stay(dur)

    def park(self, yaw_deg: float) -> None:
        """Make the current floor pose turned by `yaw_deg` (ψ) before a rest: the last keyframe
        (the set-down, or the start) is turned, unless it ends a rest, then a 1 s turn follows."""
        last = self.poses[-1]
        if last.psi == yaw_deg:
            return
        turned = dataclasses.replace(last, psi=float(yaw_deg))
        if self.rests and abs(self.rests[-1][1] - self.t) < 1e-12:
            self.to(1.0, turned)
        else:
            self.poses[-1] = turned

    def swing_set(self, n: int, apex: float, cadence_s: float, residual: float,
                  top_theta: float, kind: str) -> None:
        start, k = self.t, cadence_s / 1.2
        self.to(0.8, self.low)
        for _ in range(n):
            t_low = self.t
            self.to(0.55 * k, _swing_top(apex, residual, top_theta))
            t_top = self.t
            self.to(0.65 * k, self.low)
            self.reps.append(RepEvent(t_low, t_top, t_top, self.t, kind, "-"))
        self.to(1.0, FLOOR)
        self.sets.append((start, self.t))

    def snatch_set(self, mv: Snatch, hold_s: float, twist_sign: int) -> None:
        if mv.hand not in ("L", "R"):
            raise ValueError(f"Snatch hand must be 'L' or 'R', got {mv.hand!r}")
        if not mv.lockout_s >= MIN_LOCKOUT_S:
            raise ValueError(f"Snatch lockout_s must be >= {MIN_LOCKOUT_S} s, got {mv.lockout_s}")
        sgn = (1 if mv.hand == "R" else -1) * twist_sign
        up, lock, drop = _snatch_poses(mv.apex_m, sgn)
        start = self.t
        self.to(0.8, self.low)
        for i in range(mv.n):
            t_low = self.t
            self.to(0.40, up)
            self.to(0.35, lock)
            t_top = self.t
            self.stay(mv.lockout_s + (hold_s if i == mv.n - 1 else 0.0))
            self.tremor.append((t_top, self.t))
            t_top_end = self.t
            self.to(0.35, drop)
            self.to(0.45, self.low)
            self.reps.append(RepEvent(t_low, t_top, t_top_end, self.t, "snatch", mv.hand))
        self.to(1.0, FLOOR)
        self.sets.append((start, self.t))


def expand(moves: list, float_residual_speed: float = 0.0,
           right_hand_twist_sign: int = +1, low_residual_speed: float = 0.0) -> Script:
    """Expand moves into keyframes and event times, starting upright on the floor."""
    sc = Script(_low(low_residual_speed))
    i = 0
    while i < len(moves):
        mv = moves[i]
        if isinstance(mv, Rest):
            sc.park(mv.yaw_deg)
            sc.rest(mv.seconds)
        elif isinstance(mv, Swing):
            sc.swing_set(mv.n, mv.apex_m, mv.cadence_s, float_residual_speed, -90.0, "swing")
        elif isinstance(mv, PartialRep):
            sc.swing_set(1, mv.apex_m, 1.2, 0.0, -40.0, "other")
        elif isinstance(mv, Snatch):
            hold = 0.0
            while i + 1 < len(moves) and isinstance(moves[i + 1], Hold):
                i += 1
                hold += moves[i].seconds
            sc.snatch_set(mv, hold, right_hand_twist_sign)
        elif isinstance(mv, Hold):
            raise ValueError("Hold must directly follow a Snatch (it extends the last lockout)")
        elif isinstance(mv, TableLift):
            sc.to(2.0, Pose((FLOOR.pos[0], 0.0, FLOOR.pos[2] + mv.height_m)))
            sc.rest(2.0)
            sc.to(2.0, FLOOR)
        elif isinstance(mv, LieOnSide):
            sc.to(1.0, LIE)
            sc.rest(mv.seconds)
            sc.to(1.0, FLOOR)
        else:
            raise TypeError(f"unknown move {mv!r}")
        i += 1
    return sc


# ----------------------------------------------------------------------------- quintic segments
def quintic_coeffs(times: np.ndarray, states: np.ndarray) -> np.ndarray:
    """(K−1, 6, D) coefficients in normalised time s ∈ [0, 1] for each segment."""
    T = np.diff(times)[:, None]
    p0, v0, a0 = states[:-1, 0], states[:-1, 1], states[:-1, 2]
    p1, v1, a1 = states[1:, 0], states[1:, 1], states[1:, 2]
    dp = p1 - p0
    return np.stack([
        p0,
        v0 * T,
        0.5 * a0 * T**2,
        10 * dp - (6 * v0 + 4 * v1) * T - (1.5 * a0 - 0.5 * a1) * T**2,
        -15 * dp + (8 * v0 + 7 * v1) * T + (1.5 * a0 - a1) * T**2,
        6 * dp - 3 * (v0 + v1) * T - 0.5 * (a0 - a1) * T**2,
    ], axis=1)


def evaluate(times: np.ndarray, coeffs: np.ndarray, t: np.ndarray, cols: slice = slice(None),
             deriv: int = 2):
    """Columns `cols` (and their first/second time derivatives) at times t.

    Constant beyond both ends of the script.
    """
    seg = np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 2)
    T = (times[seg + 1] - times[seg])[:, None]
    s = np.clip((t - times[seg]) / T[:, 0], 0.0, 1.0)[:, None]
    c = [coeffs[:, i, cols][seg] for i in range(6)]
    x = c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5]))))
    if deriv == 0:
        return x
    inside = ((t >= times[0]) & (t <= times[-1]))[:, None]
    dx = c[1] + s * (2 * c[2] + s * (3 * c[3] + s * (4 * c[4] + s * 5 * c[5])))
    ddx = 2 * c[2] + s * (6 * c[3] + s * (12 * c[4] + s * 20 * c[5]))
    return x, np.where(inside, dx / T, 0.0), np.where(inside, ddx / T**2, 0.0)


def orientation(ang: np.ndarray, heading: float) -> np.ndarray:
    """World <- bell quaternions from (θ, ψ, φ) columns.

    R = R_z(heading) · R_y(θ) · R_z(ψ) · R_x(φ) · R_z(π).
    """
    ez, ey, ex = np.eye(3)[2], np.eye(3)[1], np.eye(3)[0]
    q = quat.mul(quat.from_axis_angle(ez, heading), quat.from_axis_angle(ey, ang[:, 0]))
    q = quat.mul(q, quat.from_axis_angle(ez, ang[:, 1]))
    q = quat.mul(q, quat.from_axis_angle(ex, ang[:, 2]))
    return quat.mul(q, quat.from_axis_angle(ez, math.pi))
