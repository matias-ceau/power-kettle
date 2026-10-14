import json
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from powerkettle.protocol import Status
from powerkettle.rawlog import (
    RawLog,
    RawRecord,
    estimate_rate,
    read_rawlog,
    to_recording,
    write_rawlog,
)
from test_protocol import IMU_GOLDEN, STATUS_GOLDEN

FIXTURES = Path(__file__).parent / "fixtures"


def make_log(packets, statuses=(), markers=()):
    recs = [RawRecord(1, float(i), p) for i, p in enumerate(packets)]
    recs += [RawRecord(2, 0.0, s) for s in statuses] + [RawRecord(3, 1.0, m.encode()) for m in markers]
    return RawLog({"app": "test"}, sorted(recs, key=lambda r: r.phone_ms))


def pkt(seq, first, n, flags=0):
    return struct.pack("<HIBB", seq & 0xFFFF, first, n, flags) + np.arange(n*6, dtype="<i2").tobytes()


def status_bytes(acc=16, gyro=2000, odr=400, sample_index=0, time_us=0):
    return struct.pack("<BBHHBBHhIIQ", 1, 1, 4000, odr, acc, 0, gyro, 0, 0, sample_index, time_us)


def test_roundtrip_bytes(tmp_path):
    log = make_log([pkt(0, 0, 19)], markers=["snatch L"])
    write_rawlog(tmp_path/"a.pkraw", log.meta, log.records)
    assert read_rawlog(tmp_path/"a.pkraw") == log


def test_read_from_bytes(tmp_path):
    log = make_log([pkt(0, 0, 19)])
    write_rawlog(tmp_path/"a.pkraw", log.meta, log.records)
    assert read_rawlog((tmp_path/"a.pkraw").read_bytes()) == log


def test_written_layout(tmp_path):
    write_rawlog(tmp_path/"a.pkraw", {"a": 1}, [RawRecord(3, 2.0, b"hi")])
    assert (tmp_path/"a.pkraw").read_bytes() == (
        b"PKRAW1\n" + struct.pack("<I", 7) + b'{"a":1}' + struct.pack("<BdH", 3, 2.0, 2) + b"hi"
    )


def test_bad_magic_rejected():
    with pytest.raises(ValueError, match="magic"):
        read_rawlog(b"NOTPKRAW" + bytes(20))


@pytest.mark.parametrize("cut", [3, 9, 20, -1, -5])
def test_truncated_rejected(tmp_path, cut):
    log = make_log([pkt(0, 0, 19)])
    write_rawlog(tmp_path/"a.pkraw", log.meta, log.records)
    data = (tmp_path/"a.pkraw").read_bytes()
    data = data[:cut]
    with pytest.raises(ValueError):
        read_rawlog(data)


def test_contiguous_run():
    rec = to_recording(make_log([pkt(i, i*19, 19) for i in range(10)]))
    assert len(rec.runs) == 1 and rec.runs[0].raw.shape == (190, 6) and rec.runs[0].gaps == []
    assert rec.runs[0].raw.dtype == np.int16
    assert rec.runs[0].sample_index.dtype == np.int64
    assert rec.runs[0].sample_index.tolist() == list(range(190))


def test_gap_detected():
    rec = to_recording(make_log([pkt(0, 0, 19), pkt(1, 50, 19)]))
    assert rec.runs[0].gaps == [(18, 50)]
    assert rec.runs[0].sample_index[18:20].tolist() == [18, 50]


def test_seq_wrap_is_not_a_gap():
    pk = [pkt(65534 + i, i*19, 19) for i in range(4)]   # seq 65534, 65535, 0, 1
    assert to_recording(make_log(pk)).runs[0].gaps == []


def test_index_reset_starts_new_run():
    rec = to_recording(make_log([pkt(0, 0, 19), pkt(1, 19, 19), pkt(0, 0, 19)]))
    assert [r.raw.shape[0] for r in rec.runs] == [38, 19]


def test_runs_take_ranges_from_latest_status_before_run():
    st_a = status_bytes(acc=8, gyro=1000)
    st_b = status_bytes(acc=4, gyro=500)
    log = RawLog({}, [
        RawRecord(2, 0.0, st_a), RawRecord(1, 1.0, pkt(0, 0, 19)),
        RawRecord(1, 1.5, pkt(1, 19, 19)), RawRecord(2, 2.0, st_b), RawRecord(1, 3.0, pkt(0, 0, 19)),
    ])
    rec = to_recording(log)
    assert [(r.acc_range_g, r.gyro_range_dps) for r in rec.runs] == [(8, 1000), (4, 500)]
    assert [len(r.statuses) for r in rec.runs] == [1, 0]  # a status arriving mid-run belongs to that run


def test_default_ranges_without_status():
    run = to_recording(make_log([pkt(0, 0, 19)])).runs[0]
    assert (run.acc_range_g, run.gyro_range_dps) == (16, 2000)
    assert run.rate_hz == 400.0 and run.statuses == []


def test_run_rate_estimated_from_its_statuses():
    recs = [RawRecord(2, 0.0, status_bytes(odr=400)), RawRecord(1, 1.0, pkt(0, 0, 19))]
    recs += [RawRecord(2, 2.0 + i, status_bytes(sample_index=int(i*402), time_us=i*1_000_000)) for i in range(10)]
    run = to_recording(RawLog({}, recs)).runs[0]
    assert run.rate_hz == pytest.approx(402.0, rel=1e-6)
    assert len(run.statuses) == 10


def test_markers_events_meta():
    log = RawLog({"app": "x"}, [
        RawRecord(3, 5.0, "snatch L".encode()), RawRecord(4, 6.0, b'{"e":"disconnect"}'),
    ])
    rec = to_recording(log)
    assert rec.runs == [] and rec.meta == {"app": "x"}
    assert rec.markers == [(5.0, "snatch L")]
    assert rec.events == [(6.0, {"e": "disconnect"})]


def test_unknown_record_type_ignored():
    assert to_recording(RawLog({}, [RawRecord(99, 0.0, b"?")])).runs == []


def test_estimate_rate():
    st = [Status(1,1,4000,400,16,2000,0,0, int(i*402.0), i*1_000_000) for i in range(30)]
    assert estimate_rate(st, 400.0) == pytest.approx(402.0, rel=1e-6)
    assert estimate_rate(st[:1], 400.0) == 400.0


def test_estimate_rate_short_span_returns_nominal():
    st = [Status(1,1,4000,400,16,2000,0,0, i*402, i*1_000_000) for i in range(5)]  # 4 s span
    assert estimate_rate(st, 400.0) == 400.0


def test_golden_file_reads():
    log = read_rawlog(FIXTURES/"golden.pkraw")
    assert log.meta == {"app": "golden", "v": 1}
    assert [r.type for r in log.records] == [2, 1, 3, 4]


def test_golden_file_matches_generator(tmp_path):
    subprocess.run([sys.executable, str(FIXTURES/"make_golden.py"), str(tmp_path/"g.pkraw")], check=True)
    assert (tmp_path/"g.pkraw").read_bytes() == (FIXTURES/"golden.pkraw").read_bytes()


def test_golden_recording():
    rec = to_recording(read_rawlog(FIXTURES/"golden.pkraw"))
    assert len(rec.runs) == 1 and rec.runs[0].raw.shape == (2, 6)
    assert rec.runs[0].sample_index.tolist() == [1000, 1001]
    assert rec.markers == [(1002.0, "snatch L")]
    assert rec.events == [(1003.0, {"e": "disconnect"})]


def test_info_cli():
    out = subprocess.run([sys.executable, "-m", "powerkettle", "info", str(FIXTURES/"golden.pkraw")],
                         check=True, capture_output=True, text=True).stdout
    assert "runs: 1" in out and "samples: 2" in out and "markers: 1" in out
    assert "snatch L" in out
