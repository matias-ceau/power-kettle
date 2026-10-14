"""Per-rep features and the rule-based snatch / swing / other classifier (spec §6.2).

Features (``rep_features``) of a rep (i_low, i_top, i_top_end, i_end)
--------------------------------------------------------------------
- ``apex_m``: max COM height z over [i_low, i_top_end] above the low (``segment_position``
  restarts at 0 at every low, so this is the height gained low → top).
- ``top_tilt_deg``: unsigned angle between world bell-up (R·up_s) and vertical: averaged over
  the lockout window [i_top, i_top_end] for a ``top_lockout``, at i_top for a ``top_swing``.
  Swing ≈ 90°, snatch lockout ≈ 0–45°.
- ``twist_deg``: signed rotation about world z from the low to the top, ∫ ω_w·ẑ dt over
  [i_low, i_top], positive counter-clockwise seen from above. Summed sample to sample from the
  orientation quaternions (no wrap at ±180°). Snatch ≈ ±150–180° (punch-through), swing ≈ 0.
- ``top_static_s``: duration of the lockout window, t[i_top_end] − t[i_top] (0 for a swing top).

Rules (``classify``)
--------------------
- snatch: apex ≥ ``snatch_min_apex_m``, tilt ≤ ``snatch_max_top_tilt_deg``, static ≥
  ``snatch_min_static_s``;
- swing: ``swing_min_apex_m`` ≤ apex ≤ ``swing_max_apex_m`` and tilt within
  ``swing_top_tilt_deg`` (lo < tilt ≤ hi);
- otherwise other. Snatch and swing are disjoint (by tilt). Any NaN feature among apex, tilt
  and static gives other with confidence 0.

Normalised margins (≥ 0 when the rule holds): a lower bound lo gives (x − lo) / lo, an upper
bound hi gives (hi − x) / hi, a range [lo, hi] gives min(x − lo, hi − x) / ((hi − lo) / 2).
Confidence of a snatch or swing = its smallest margin; of other = its distance to the nearest
profile: for each of snatch and swing, the largest violation (−margin) among its failed rules,
then the smaller of the two. Clipped to [0, 1]; below ``low_conf`` callers flag the rep
``"low_conf"``.

Hand (snatches only): with |twist| ≥ ``hand_min_twist_deg``, sign(twist)·right_hand_twist_sign
> 0 → "R", < 0 → "L"; a smaller (or NaN) twist → "?". Swings and other → "-". The twist does not
enter the kind rules. Stdlib + numpy only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import quat


@dataclass
class ClassifierConfig:
    snatch_min_apex_m: float = 1.25
    snatch_max_top_tilt_deg: float = 60.0
    snatch_min_static_s: float = 0.1
    swing_min_apex_m: float = 0.5
    swing_max_apex_m: float = 1.3
    swing_top_tilt_deg: tuple = (60.0, 130.0)
    hand_min_twist_deg: float = 90.0
    right_hand_twist_sign: int = +1  # sign of the twist about world z of a right-hand snatch
    low_conf: float = 0.2


def _tilt_deg(up_w: np.ndarray) -> np.ndarray:
    return np.degrees(np.arctan2(np.hypot(up_w[..., 0], up_w[..., 1]), up_w[..., 2]))


def world_z_rotation(quats: np.ndarray) -> float:
    """Signed rotation (rad) about world z accumulated over consecutive world <- body
    quaternions: Σ z-component of the rotation vector of q[k+1] ⊗ q[k]* (= ∫ ω_w·ẑ dt)."""
    q = np.asarray(quats, dtype=float)
    if len(q) < 2:
        return 0.0
    dq = quat.mul(q[1:], quat.conj(q[:-1]))
    dq = dq * np.where(dq[:, :1] < 0.0, -1.0, 1.0)  # shortest rotation (w ≥ 0)
    s = np.linalg.norm(dq[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(s, dq[:, 0])
    scale = np.divide(angle, s, out=np.full_like(s, 2.0), where=s > 1e-12)  # → 2 as s → 0
    return float(np.sum(scale * dq[:, 3]))


def rep_features(seg: tuple[int, int, int, int], quats: np.ndarray, calib, pos: np.ndarray,
                 t: np.ndarray) -> dict:
    """``apex_m``, ``top_tilt_deg``, ``twist_deg``, ``top_static_s`` of the rep `seg` =
    (i_low, i_top, i_top_end, i_end) (see the module docstring). `quats` world <- sensor,
    `calib.up_s` bell-up in the sensor frame, `pos` from ``segment_position``."""
    i_low, i_top, i_top_end, _ = (int(i) for i in seg)
    z = np.asarray(pos, dtype=float)[:, 2]
    up = quat.rotate(np.asarray(quats, float)[i_top:i_top_end + 1], calib.up_s)
    return {
        "apex_m": float(np.max(z[i_low:i_top_end + 1]) - z[i_low]),
        "top_tilt_deg": float(np.mean(_tilt_deg(up))),
        "twist_deg": math.degrees(world_z_rotation(np.asarray(quats)[i_low:i_top + 1])),
        "top_static_s": float(t[i_top_end] - t[i_top]),
    }


def _lower(x: float, lo: float) -> float:
    return (x - lo) / lo


def _upper(x: float, hi: float) -> float:
    return (hi - x) / hi


def _range(x: float, lo: float, hi: float) -> float:
    return min(x - lo, hi - x) / ((hi - lo) / 2.0)


def _margins(f: dict, cfg: ClassifierConfig) -> dict[str, list[float]]:
    apex, tilt, static = f["apex_m"], f["top_tilt_deg"], f["top_static_s"]
    lo, hi = cfg.swing_top_tilt_deg
    return {
        "snatch": [_lower(apex, cfg.snatch_min_apex_m),
                   _upper(tilt, cfg.snatch_max_top_tilt_deg),
                   _lower(static, cfg.snatch_min_static_s)],
        "swing": [_range(apex, cfg.swing_min_apex_m, cfg.swing_max_apex_m),
                  _range(tilt, lo, hi)],
    }


def _hand(twist_deg: float, cfg: ClassifierConfig) -> str:
    if not abs(twist_deg) >= cfg.hand_min_twist_deg:  # False for NaN
        return "?"
    return "R" if twist_deg * cfg.right_hand_twist_sign > 0.0 else "L"


def classify(features: dict, cfg: ClassifierConfig = ClassifierConfig()) -> tuple[str, str, float]:
    """(kind, hand, confidence) of a rep from its ``rep_features`` (rules, margins and hand in
    the module docstring); kind ∈ snatch | swing | other, hand ∈ L | R | ? | -."""
    if not all(math.isfinite(features[k]) for k in ("apex_m", "top_tilt_deg", "top_static_s")):
        return "other", "-", 0.0
    margins = _margins(features, cfg)
    snatch_ok = all(m >= 0.0 for m in margins["snatch"])
    lo, _ = cfg.swing_top_tilt_deg
    # tilt == lo satisfies the swing margin but is a snatch tilt: swing needs lo < tilt
    swing_ok = all(m >= 0.0 for m in margins["swing"]) and features["top_tilt_deg"] > lo
    if snatch_ok:
        kind, hand, conf = "snatch", _hand(features["twist_deg"], cfg), min(margins["snatch"])
    elif swing_ok:
        kind, hand, conf = "swing", "-", min(margins["swing"])
    else:
        kind, hand = "other", "-"
        conf = min(max(-m for m in ms) for ms in margins.values())
    return kind, hand, float(np.clip(conf, 0.0, 1.0))
