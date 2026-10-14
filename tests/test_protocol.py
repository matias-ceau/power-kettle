import struct

import numpy as np
import pytest

from powerkettle.protocol import (
    decode_imu_packet,
    decode_status,
    encode_command,
)

IMU_GOLDEN = bytes.fromhex("3412e80300000201010002000300040005000600fffffefffdfffcfffbfffaff")
STATUS_GOLDEN = bytes.fromhex("0101930f90011000d00700fe0700000040e20100ea16b04c02000000")


def test_decode_imu_golden():
    p = decode_imu_packet(IMU_GOLDEN)
    assert (p.seq, p.first_index, p.flags) == (0x1234, 1000, 1)
    assert p.samples.dtype == np.int16
    assert p.samples.tolist() == [[1, 2, 3, 4, 5, 6], [-1, -2, -3, -4, -5, -6]]


def test_decode_imu_single_sample():  # small-MTU case
    b = struct.pack("<HIBB6h", 1, 5, 1, 0, 7, 8, 9, 10, 11, 12)
    assert decode_imu_packet(b).samples.shape == (1, 6)


def test_decode_imu_rejects_bad_length():
    with pytest.raises(ValueError):
        decode_imu_packet(IMU_GOLDEN[:-1])


def test_decode_imu_rejects_more_than_19_samples():
    b = struct.pack("<HIBB", 1, 0, 20, 0) + bytes(12 * 20)
    with pytest.raises(ValueError):
        decode_imu_packet(b)


def test_decode_status_golden():
    s = decode_status(STATUS_GOLDEN)
    assert (s.battery_mv, s.odr_hz, s.acc_range_g, s.gyro_range_dps) == (3987, 400, 16, 2000)
    assert (s.dropped_samples, s.sample_index, s.time_us) == (7, 123456, 9876543210)
    assert s.temperature_c == pytest.approx(22.0)


def test_decode_status_rejects_bad_length_and_version():
    with pytest.raises(ValueError):
        decode_status(STATUS_GOLDEN[:-1])
    with pytest.raises(ValueError):
        decode_status(b"\x02" + STATUS_GOLDEN[1:])


def test_encode_commands():
    assert encode_command("start") == b"\x01"
    assert encode_command("stop") == b"\x02"
    assert encode_command("ping") == b"\x04"
    assert encode_command("set_config", odr_hz=400, acc_range_g=16, gyro_range_dps=2000).hex() == "03900110d007"
    with pytest.raises(ValueError):
        encode_command("set_config", odr_hz=300, acc_range_g=16, gyro_range_dps=2000)


def test_encode_unknown_op_rejected():
    with pytest.raises(ValueError):
        encode_command("reboot")
