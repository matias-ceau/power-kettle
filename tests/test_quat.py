import numpy as np
import pytest

from powerkettle.quat import (
    angle_between,
    from_axis_angle,
    from_two_vectors,
    integrate,
    rotate,
)


def test_quat_rotate_and_integrate():
    q = from_axis_angle([0, 0, 1], np.pi / 2)
    assert rotate(q, [1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)
    q2 = np.array([1.0, 0, 0, 0])
    for _ in range(400):
        q2 = integrate(q2, np.array([0, 0, np.pi / 2]), 1 / 400)
    assert angle_between(rotate(q2, [1, 0, 0]), [0, 1, 0]) < 1e-9
    assert angle_between(rotate(from_two_vectors([0, 0, 1], [1, 0, 0]), [0, 0, 1]), [1, 0, 0]) < 1e-12


def test_quat_vectorised_matrix_and_antiparallel():
    from powerkettle.quat import conj, mul, normalize, to_matrix

    rng = np.random.default_rng(0)
    q = normalize(rng.normal(size=(5, 4)))
    v = rng.normal(size=(5, 3))
    assert rotate(q, v) == pytest.approx(np.einsum("nij,nj->ni", to_matrix(q), v), abs=1e-12)
    assert rotate(mul(q, conj(q)), v) == pytest.approx(v, abs=1e-12)
    for a in ([0, 0, 1], [1, 0, 0]):
        qa = from_two_vectors(a, [-x for x in a])
        assert angle_between(rotate(qa, a), [-x for x in a]) < 1e-12
    # batched integrate: per-sample rates, scalar dt
    out = integrate(np.tile([1.0, 0, 0, 0], (3, 1)), np.zeros((3, 3)), 0.01)
    assert out == pytest.approx(np.tile([1.0, 0, 0, 0], (3, 1)))
