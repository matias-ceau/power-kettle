import math
from types import SimpleNamespace

import numpy as np
import pytest

from powerkettle import quat
from powerkettle.classify import ClassifierConfig, classify, rep_features
from powerkettle.reps import segment_reps
from powerkettle.simulate import Rest, SimConfig, Snatch, Swing

from sim_pipeline import run_stages

CFG = ClassifierConfig()


def feats(apex, tilt, twist, static):
    return {"apex_m": apex, "top_tilt_deg": tilt, "twist_deg": twist, "top_static_s": static}


# ----------------------------------------------------------------------------- classify
def test_snatch_hands_from_twist_sign():
    kind, hand, conf = classify(feats(1.6, 25, 175, 0.4), CFG)
    assert (kind, hand) == ("snatch", "R")
    # smallest margin: apex (1.6 − 1.25) / 1.25; tilt (60 − 25) / 60; static (0.4 − 0.1) / 0.1
    assert conf == pytest.approx(0.28)
    assert classify(feats(1.6, 25, -175, 0.4), CFG)[:2] == ("snatch", "L")


def test_swing():
    kind, hand, conf = classify(feats(1.0, 92, 5, 0), CFG)
    assert (kind, hand) == ("swing", "-")
    # apex in [0.5, 1.3]: min(0.5, 0.3) / 0.4; tilt in (60, 130): min(32, 38) / 35
    assert conf == pytest.approx(0.75)


def test_other():
    kind, hand, conf = classify(feats(0.4, 40, 0, 0), CFG)
    assert (kind, hand) == ("other", "-")
    # snatch fails by max(0.68 apex, 1.0 static) = 1.0; swing by max(0.25 apex, 20/35 tilt);
    # confidence = the smaller (distance to the nearest profile)
    assert conf == pytest.approx(20 / 35)


def test_small_twist_snatch_hand_unknown():
    assert classify(feats(1.6, 25, 40, 0.4), CFG)[:2] == ("snatch", "?")
    assert classify(feats(1.6, 25, -89, 0.4), CFG)[:2] == ("snatch", "?")
    assert classify(feats(1.6, 25, -90, 0.4), CFG)[:2] == ("snatch", "L")


def test_hand_sign_mapping_configurable():
    left = ClassifierConfig(right_hand_twist_sign=-1)
    assert classify(feats(1.6, 25, 175, 0.4), left)[:2] == ("snatch", "L")
    assert classify(feats(1.6, 25, -175, 0.4), left)[:2] == ("snatch", "R")


def test_snatch_needs_static_top_and_upright_bell():
    assert classify(feats(1.6, 25, 175, 0.05), CFG)[0] == "other"  # no lockout
    assert classify(feats(1.6, 75, 175, 0.4), CFG)[0] == "other"  # too high for a swing
    assert classify(feats(1.2, 25, 175, 0.4), CFG)[0] == "other"  # too low for a snatch


def test_confidence_near_threshold_is_low_and_clipped():
    _, _, conf = classify(feats(1.3, 25, 175, 0.4), CFG)
    assert conf == pytest.approx(0.04)
    assert conf < CFG.low_conf
    assert classify(feats(1.0, 95, 0, 0), CFG)[2] <= 1.0
    assert classify(feats(5.0, 175, 0, 9.0), CFG)[2] == 1.0  # far from both profiles


def test_nan_features_are_other_with_zero_confidence():
    assert classify(feats(float("nan"), 25, 175, 0.4), CFG) == ("other", "-", 0.0)
    kind, hand, _ = classify(feats(1.6, 25, float("nan"), 0.4), CFG)
    assert (kind, hand) == ("snatch", "?")


# ----------------------------------------------------------------------------- rep_features
FS = 400.0


def synthetic_rep(top_end_extra=40, sign=1.0):
    """Bell-up from sensor z; R = R_z(ψ)·R_y(θ): world z-rate = ψ', tilt from vertical = |θ|.
    ψ: 0 at the low, sign·200° at the top (no wrap), back to 0 at the next low."""
    i_low, i_top = 40, 240
    i_top_end = i_top + top_end_extra
    i_end = i_top_end + 200
    n = i_end + 40
    psi = np.zeros(n)
    theta = np.full(n, math.radians(40.0))
    up = np.linspace(0.0, 1.0, i_top - i_low + 1)
    psi[i_low:i_top + 1] = sign * math.radians(200.0) * up
    theta[i_low:i_top + 1] = math.radians(40.0) + math.radians(-60.0) * up  # 40° -> −20°
    psi[i_top:i_top_end + 1] = sign * math.radians(200.0)
    hold = np.linspace(-1.0, 1.0, i_top_end - i_top + 1) if top_end_extra else np.zeros(1)
    theta[i_top:i_top_end + 1] = math.radians(-20.0) + math.radians(5.0) * hold  # −25..−15°
    down = np.linspace(1.0, 0.0, i_end - i_top_end + 1)
    psi[i_top_end:i_end + 1] = sign * math.radians(200.0) * down
    q = quat.mul(quat.from_axis_angle([0, 0, 1.0], psi), quat.from_axis_angle([0, 1.0, 0], theta))
    pos = np.zeros((n, 3))
    pos[:, 2] = 2.0  # outside the rep: ignored
    k = np.arange(i_low, i_end + 1)
    pos[k, 2] = 1.55 * np.sin(np.pi * (k - i_low) / (i_end - i_low))
    t = np.arange(n) / FS
    calib = SimpleNamespace(up_s=np.array([0.0, 0.0, 1.0]))
    return (i_low, i_top, i_top_end, i_end), q, calib, pos, t


def test_rep_features_lockout():
    seg, q, calib, pos, t = synthetic_rep()
    f = rep_features(seg, q, calib, pos, t)
    assert f["twist_deg"] == pytest.approx(200.0, abs=0.01)  # no wrap at ±180°
    assert f["top_tilt_deg"] == pytest.approx(20.0, abs=0.05)  # mean over the lockout
    assert f["top_static_s"] == pytest.approx(40 / FS)
    assert f["apex_m"] == pytest.approx(1.55, abs=1e-3)


def test_rep_features_swing_top_and_sign():
    seg, q, calib, pos, t = synthetic_rep(top_end_extra=0, sign=-1.0)
    f = rep_features(seg, q, calib, pos, t)
    assert f["twist_deg"] == pytest.approx(-200.0, abs=0.01)
    assert f["top_tilt_deg"] == pytest.approx(20.0, abs=0.05)  # θ(i_top) = −20°
    assert f["top_static_s"] == 0.0


def test_rep_features_relative_to_low():
    seg, q, calib, pos, t = synthetic_rep()
    pos[:, 2] += 0.3
    assert rep_features(seg, q, calib, pos, t)["apex_m"] == pytest.approx(1.55, abs=1e-3)


# ----------------------------------------------------------------------------- simulated hands
@pytest.mark.parametrize("sign", [+1, -1])
def test_simulated_snatch_hands(sign):
    """The simulator twists ψ by +180°·(+1 R, −1 L)·right_hand_twist_sign; the features see the
    same sign about world z, so a matching config recovers the hand."""
    s = run_stages([Rest(3), Snatch(4, "R"), Rest(4), Snatch(4, "L"), Rest(4),
                    Swing(3), Rest(3)],
                   SimConfig(right_hand_twist_sign=sign))
    segs = segment_reps(s.anchors, len(s.imu.t))
    assert len(segs) == len(s.truth.reps) == 11
    cfg = ClassifierConfig(right_hand_twist_sign=sign)
    swapped = ClassifierConfig(right_hand_twist_sign=-sign)
    for seg, r in zip(segs, s.truth.reps):
        f = rep_features(seg, s.q, s.calib, s.pos, s.imu.t)
        kind, hand, _ = classify(f, cfg)
        assert (kind, hand) == (r.kind, r.hand), f
        if kind == "snatch":
            assert abs(f["twist_deg"]) > 120.0
            assert classify(f, swapped)[1] == {"R": "L", "L": "R"}[r.hand]
