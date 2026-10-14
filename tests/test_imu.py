import math

import numpy as np

from powerkettle.imu import ImuData, scale_run
from powerkettle.rawlog import Run

G = 9.80665


def make_run(indices, raw=None, gaps=None, rate_hz=400.0, acc_g=16, gyro_dps=2000):
    idx = np.asarray(indices, dtype=np.int64)
    if raw is None:
        raw = np.zeros((len(idx), 6), dtype=np.int16)
    return Run(idx, np.asarray(raw, dtype=np.int16), gaps or [], [], rate_hz, acc_g, gyro_dps)


def test_acc_scale_16g():
    run = make_run([0, 1], raw=[[2048, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    d = scale_run(run)
    assert isinstance(d, ImuData)
    assert math.isclose(d.acc[0, 0], G, rel_tol=1e-12)
    assert d.acc.shape == (2, 3) and d.gyro.shape == (2, 3)


def test_gyro_scale_2000dps():
    run = make_run([0], raw=[[0, 0, 0, 16384, 0, 0]])
    d = scale_run(run)
    assert math.isclose(d.gyro[0, 0], math.radians(1000.0), rel_tol=1e-12)


def test_gap_fills_nan_and_invalidates_neighbours():
    idx = list(range(0, 19)) + list(range(50, 69))
    run = make_run(idx, raw=np.ones((len(idx), 6)), gaps=[(18, 50)])
    d = scale_run(run)
    assert len(d.t) == 69
    assert d.first_index == 0
    assert np.all(np.isnan(d.acc[19:50])) and np.all(np.isnan(d.gyro[19:50]))
    assert not d.valid[17:52].any()
    assert d.valid[:17].all()
    assert d.valid[52:].all()
    assert not d.saturated[19:50].any()


def test_time_axis_uses_rate_and_first_index():
    run = make_run([10, 11, 12], rate_hz=402.0)
    d = scale_run(run)
    assert d.first_index == 10
    assert d.t[0] == 0.0
    assert d.t[1] == 1 / 402
    assert d.fs == 402.0


def test_saturation_flag():
    raw = np.zeros((4, 6), dtype=np.int16)
    raw[1, 1] = -32768
    raw[2, 2] = 32767
    raw[3, 3] = -32768  # gyro only: not acc saturation
    d = scale_run(make_run([0, 1, 2, 3], raw=raw))
    assert d.saturated.tolist() == [False, True, True, False]


def test_other_ranges():
    run = make_run([0], raw=[[16384, 0, 0, 0, 0, 16384]], acc_g=8, gyro_dps=1000)
    d = scale_run(run)
    assert math.isclose(d.acc[0, 0], 4 * G, rel_tol=1e-12)
    assert math.isclose(d.gyro[0, 2], math.radians(500.0), rel_tol=1e-12)
