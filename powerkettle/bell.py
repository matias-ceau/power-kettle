"""Kettlebell geometry model (spec 3.4): ball + flats + inverted-U handle.

All public functions return SI (metres, kilograms). Profile fields are in cm.
Bell frame: x outward normal of the front flat, z bell-up (base -> handle).
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields

import numpy as np


@dataclass(frozen=True)
class BellProfile:
    mass_kg: float = 32.0
    circumference_cm: float = 63.0
    bottom_flat_diam_cm: float = 12.0
    side_flat_diam_cm: float = 10.0
    handle_clearance_cm: float = 9.0
    handle_tube_circumference_cm: float = 13.0
    handle_length_cm: float = 44.0
    handle_bar_cm: float = 16.0
    device_thickness_cm: float = 1.0

    def __post_init__(self) -> None:
        """ValueError for a profile no bell can have (every size and the mass positive, the
        thickness of the device not negative, the flats narrower than the ball)."""
        for f in fields(self):
            v = getattr(self, f.name)
            if not (math.isfinite(v) and (v >= 0 if f.name == "device_thickness_cm" else v > 0)):
                raise ValueError(f"bell profile: {f.name} must be a positive number, got {v}")
        ball = self.circumference_cm / math.pi
        for name, label in (("bottom_flat_diam_cm", "bottom flat"),
                            ("side_flat_diam_cm", "side flat")):
            if getattr(self, name) >= ball:
                raise ValueError(f"bell profile: {label} Ø {getattr(self, name):g} cm must be "
                                 f"smaller than the ball Ø {ball:.1f} cm (circumference "
                                 f"{self.circumference_cm:g} cm)")

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "BellProfile":
        """Profile from a dict of field values (cm, kg); ValueError for an unknown key or an
        invalid profile."""
        names = {f.name for f in fields(cls)}
        unknown = sorted(set(d) - names)
        if unknown:
            raise ValueError(f"bell profile: unknown field(s) {', '.join(unknown)} "
                             f"(known: {', '.join(sorted(names))})")
        return cls(**{k: float(v) for k, v in d.items()})


def ball_radius_m(b: BellProfile) -> float:
    return b.circumference_cm / (2.0 * math.pi) / 100.0


def _flat_distance_m(b: BellProfile, diam_cm: float) -> float:
    r = ball_radius_m(b)
    return math.sqrt(r * r - (diam_cm / 200.0) ** 2)


def side_flat_distance_m(b: BellProfile) -> float:
    """Distance from ball centre to each (front/back) side flat."""
    return _flat_distance_m(b, b.side_flat_diam_cm)


def _bottom_flat_distance_m(b: BellProfile) -> float:
    return _flat_distance_m(b, b.bottom_flat_diam_cm)


def _cap(r: float, d: float) -> tuple[float, float]:
    """Spherical cap cut at distance d from centre: (volume, centroid distance from centre)."""
    h = r - d
    vol = math.pi * h * h * (3.0 * r - h) / 3.0
    cen = 3.0 * (2.0 * r - h) ** 2 / (4.0 * (3.0 * r - h))
    return vol, cen


def _parts(b: BellProfile):
    """Solid parts as (volume_m3, centroid_z_m) relative to ball centre."""
    r = ball_radius_m(b)
    parts = [(4.0 / 3.0 * math.pi * r**3, 0.0)]
    vb, cb = _cap(r, _bottom_flat_distance_m(b))
    parts.append((-vb, -cb))
    vs, _ = _cap(r, side_flat_distance_m(b))
    parts.append((-2.0 * vs, 0.0))  # caps on +/-x: no z moment

    rt = b.handle_tube_circumference_cm / (2.0 * math.pi) / 100.0
    area = math.pi * rt * rt
    bar = b.handle_bar_cm / 100.0
    leg = (b.handle_length_cm / 100.0 - bar) / 2.0
    z_top = r + b.handle_clearance_cm / 100.0 + rt
    parts.append((area * bar, z_top))
    parts.append((2.0 * area * leg, z_top - leg / 2.0))
    return parts


def com_offset_m(b: BellProfile) -> float:
    """Height of the centre of mass above the ball centre (uniform density)."""
    parts = _parts(b)
    vol = sum(v for v, _ in parts)
    return sum(v * z for v, z in parts) / vol


def model_mass_kg(b: BellProfile, density: float = 7100.0) -> float:
    return density * sum(v for v, _ in _parts(b))


def lever_arm_bell_m(b: BellProfile) -> np.ndarray:
    """Vector from COM to sensor in the bell frame (outward x, bell-up z)."""
    return np.array(
        [side_flat_distance_m(b) + b.device_thickness_cm / 100.0, 0.0, -com_offset_m(b)]
    )
