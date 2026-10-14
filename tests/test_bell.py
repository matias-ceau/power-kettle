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
