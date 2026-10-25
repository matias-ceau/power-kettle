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
    Run,
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


def status_bytes(acc=16, gyro=2000, odr=400, sample_index=0, time_us=0, flags=1):
    return struct.pack("<BBHHBBHhIIQ", 1, flags, 4000, odr, acc, 0, gyro, 0, 0, sample_index, time_us)


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
    assert run.rate_estimated is False


def test_packet_times_map_phone_time_to_first_index():
    log = RawLog({}, [
        RawRecord(1, 10.5, pkt(0, 0, 19)), RawRecord(3, 11.0, b"m"), RawRecord(1, 12.25, pkt(1, 19, 19)),
        RawRecord(1, 20.0, pkt(2, 50, 7)), RawRecord(1, 30.0, pkt(0, 0, 19)),
    ])
    runs = to_recording(log).runs
    assert runs[0].packet_times.dtype == np.float64 and runs[0].packet_times.shape == (3, 2)
    assert runs[0].packet_times.tolist() == [[10.5, 0.0], [12.25, 19.0], [20.0, 50.0]]
    assert runs[1].packet_times.tolist() == [[30.0, 0.0]]


def stream_records(seconds, rate=402, t0_ms=0.0, ahead=0):
    """One second of packets (19 samples each, 402/s -> last packet short) followed by a status."""
    recs, idx = [], 0
    for sec in range(seconds):
        target = (sec + 1) * rate
        while idx < target:
            n = min(19, target - idx)
            recs.append(RawRecord(1, t0_ms + sec * 1000.0, pkt(idx // 19, idx, n)))
            idx += n
        recs.append(RawRecord(2, t0_ms + sec * 1000.0 + 500, status_bytes(sample_index=idx + ahead, time_us=(sec + 1) * 1_000_000)))
    return recs, idx


def test_run_rate_estimated_from_its_statuses():
    recs, _ = stream_records(10)
    run = to_recording(RawLog({}, recs)).runs[0]
    assert run.rate_hz == pytest.approx(402.0, rel=1e-6)
    assert len(run.statuses) == 10
    assert run.rate_estimated is True


def test_short_run_rate_not_estimated():
    recs, _ = stream_records(3)  # statuses span 2 s < MIN_RATE_SPAN_S
    run = to_recording(RawLog({}, recs)).runs[0]
    assert run.rate_hz == 400.0 and run.rate_estimated is False


def test_status_ahead_of_received_packets_still_gives_rate():
    # real statuses are latched at the FIFO read, so sample_index leads the last packet received
    recs, _ = stream_records(10, ahead=25)
    run = to_recording(RawLog({}, recs)).runs[0]
    assert run.rate_hz == pytest.approx(402.0, rel=1e-6)
    assert len(run.statuses) == 10


def test_status_far_beyond_run_extent_rejected():
    recs, last = stream_records(10)
    recs.append(RawRecord(2, 30000.0, status_bytes(sample_index=last + 5000, time_us=11_000_000)))
    assert len(to_recording(RawLog({}, recs)).runs[0].statuses) == 10


def test_idle_trailing_statuses_do_not_affect_rate():
    recs, last = stream_records(10)
    # device stopped: status keeps notifying at 1 Hz, streaming flag clear, constant sample_index
    recs += [RawRecord(2, 20000.0 + i * 1000.0, status_bytes(sample_index=last, time_us=(11 + i) * 1_000_000, flags=0))
             for i in range(30)]
    run = to_recording(RawLog({}, recs)).runs[0]
    assert run.rate_hz == pytest.approx(402.0, rel=1e-6)
    assert len(run.statuses) == 10


def test_backwards_status_not_accepted():
    recs, last = stream_records(10)
    recs.append(RawRecord(2, 30000.0, status_bytes(sample_index=last - 5, time_us=11_000_000)))
    assert len(to_recording(RawLog({}, recs)).runs[0].statuses) == 10


def test_post_reset_status_before_new_run_does_not_join_old_run():
    recs, last = stream_records(10)
    # device restarted: streaming status with sample_index back at 0 arrives before the first new packet
    recs.append(RawRecord(2, 30000.0, status_bytes(sample_index=0, time_us=500_000)))
    recs.append(RawRecord(1, 30001.0, pkt(0, 0, 19)))
    rec = to_recording(RawLog({}, recs))
    assert len(rec.runs) == 2
    assert len(rec.runs[0].statuses) == 10
    assert rec.runs[0].rate_hz == pytest.approx(402.0, rel=1e-6)


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
    assert "rate 400.00 Hz (nominal: no status data)" in out


def test_info_estimated_rate_not_marked_nominal(tmp_path):
    recs, _ = stream_records(10)
    write_rawlog(tmp_path/"a.pkraw", {}, recs)
    out = subprocess.run([sys.executable, "-m", "powerkettle", "info", str(tmp_path/"a.pkraw")],
                         check=True, capture_output=True, text=True).stdout
    assert "rate 402.00 Hz," in out and "nominal" not in out


def _still_run(n, rate=400.0, noise_counts=(4.0, 1.5), seed=0):
    rng = np.random.default_rng(seed)
    raw = np.zeros((n, 6))
    raw[:, 2] = 2048.0  # +1 g on z at +-16 g (2048 LSB/g)
    raw[:, :3] += rng.normal(0.0, noise_counts[0], (n, 3))
    raw[:, 3:] += rng.normal(0.0, noise_counts[1], (n, 3))
    return Run(sample_index=np.arange(n, dtype=np.int64), raw=np.round(raw).astype(np.int16),
               gaps=[], statuses=[], rate_hz=rate, acc_range_g=16, gyro_range_dps=2000)


def test_still_noise_picks_quietest_second():
    from powerkettle.__main__ import still_noise
    run = _still_run(4000)
    run.raw[:2000, :] += np.int16(300)  # wobble: the first 5 s are not still
    run.raw[:2000:2, :] -= np.int16(600)
    acc_mg, gyro_dps, g = still_noise(run)
    assert acc_mg == pytest.approx(4.0 * 16000 / 32768, rel=0.15)
    assert gyro_dps == pytest.approx(1.5 * 2000 / 32768, rel=0.15)
    assert g == pytest.approx(1.0, abs=0.01)


def test_still_noise_short_run_is_none():
    from powerkettle.__main__ import still_noise
    assert still_noise(_still_run(100)) is None
