import pytest

from powerkettle.bell import (
    BellProfile,
    ball_radius_m,
    com_offset_m,
    lever_arm_bell_m,
    model_mass_kg,
    side_flat_distance_m,
)


def test_bell_matches_hand_calculation():
    b = BellProfile()
    assert ball_radius_m(b) == pytest.approx(0.10027, abs=1e-4)
    assert side_flat_distance_m(b) == pytest.approx(0.0869, abs=2e-4)
    assert com_offset_m(b) == pytest.approx(0.0237, abs=1.5e-3)
    assert model_mass_kg(b) == pytest.approx(32.6, abs=0.4)
    assert lever_arm_bell_m(b) == pytest.approx([0.0969, 0.0, -0.0237], abs=2e-3)


def test_bell_dict_roundtrip():
    b = BellProfile(device_thickness_cm=1.5)
    assert BellProfile.from_dict(b.to_dict()) == b


def test_bell_from_dict_rejects_unknown_keys():
    with pytest.raises(ValueError, match="mass"):
        BellProfile.from_dict({"mass": 24.0})


@pytest.mark.parametrize("kw, match", [
    ({"bottom_flat_diam_cm": 21.0}, "bottom flat"),  # ball Ø = 63 / π ≈ 20.05 cm
    ({"side_flat_diam_cm": 20.1}, "side flat"),
    ({"circumference_cm": 0.0}, "circumference"),
    ({"mass_kg": -1.0}, "mass"),
])
def test_bell_invalid_geometry(kw, match):
    with pytest.raises(ValueError, match=match):
        BellProfile(**kw)
    with pytest.raises(ValueError, match=match):
        BellProfile.from_dict(kw)
