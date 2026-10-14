import io
import json

import numpy as np
import pytest

from powerkettle.bell import BellProfile
from powerkettle.pipeline import ALGORITHM_VERSION, analyse_run
from powerkettle.session_io import (
    calibration_from_dict,
    calibration_to_dict,
    load_session,
    reanalyse,
    save_session,
)
from powerkettle.simulate import Rest, Snatch, Swing, simulate

BELL = BellProfile(mass_kg=24.0, circumference_cm=58.5)
REP_LABELS = {3: {"kind": "other"}, 7: {"kind": "snatch", "hand": "L"}}  # by rep index


@pytest.fixture(scope="module")
def run():
    r, _ = simulate([Rest(3), Snatch(5, "R"), Rest(4), Swing(5), Rest(3)])
    return r


@pytest.fixture(scope="module")
def LABELS(run):
    """REP_LABELS keyed by the sample index of the rep's low."""
    reps = [r for s in analyse_run(run, BELL).sets for r in s.reps]
    return {int(run.sample_index[0]) + reps[i].i_low: lab for i, lab in REP_LABELS.items()}


def test_roundtrip_exact(run, LABELS, tmp_path):
    meta = {"labels": LABELS, "markers": [{"phone_ms": 1.5, "text": "go", "sample_index": 12}],
            "app_version": "powerkettle-record/1"}
    save_session(tmp_path / "s.npz", run, BELL, meta)
    run2, bell2, meta2 = load_session(tmp_path / "s.npz")
    assert run2.sample_index.dtype == np.int64 and run2.raw.dtype == np.int16
    assert np.array_equal(run2.sample_index, run.sample_index)
    assert np.array_equal(run2.raw, run.raw)
    assert run2.gaps == run.gaps
    assert run2.rate_hz == run.rate_hz and run2.rate_estimated == run.rate_estimated
    assert (run2.acc_range_g, run2.gyro_range_dps) == (run.acc_range_g, run.gyro_range_dps)
    assert bell2 == BELL
    assert meta2["labels"] == LABELS  # int keys restored
    assert meta2["markers"] == meta["markers"]
    assert meta2["app_version"] == "powerkettle-record/1"
    assert meta2["algorithm_version"] == ALGORITHM_VERSION


def test_npz_layout(run):
    buf = io.BytesIO()
    save_session(buf, run, BELL, {"labels": {}})
    buf.seek(0)
    with np.load(buf, allow_pickle=False) as z:
        assert set(z.files) == {"sample_index", "raw", "gaps", "rate_hz", "ranges", "meta_json"}
        assert z["gaps"].shape == (0, 2)
        assert list(z["ranges"]) == [run.acc_range_g, run.gyro_range_dps]
        meta = json.loads(str(z["meta_json"]))
    assert meta["bell"] == BELL.to_dict()


def test_gaps_roundtrip(run):
    gaps = [(99, 140), (500, 501)]
    keep = np.ones(len(run.sample_index), dtype=bool)
    keep[100:140] = False
    keep[500] = False
    from dataclasses import replace
    gapped = replace(run, sample_index=run.sample_index[keep], raw=run.raw[keep], gaps=gaps)
    buf = io.BytesIO()
    save_session(buf, gapped, BELL, {})
    buf.seek(0)
    run2, _, _ = load_session(buf)
    assert run2.gaps == gaps and all(isinstance(a, int) for g in run2.gaps for a in g)


def test_reanalysis_of_loaded_run_identical(run, LABELS, tmp_path):
    res = analyse_run(run, BELL, labels=LABELS)
    assert sum("relabelled" in r.flags for s in res.sets for r in s.reps) == 2
    save_session(tmp_path / "s.npz", run, BELL, {"labels": LABELS, "summary": res.summary()})
    run2, bell2, meta2 = load_session(tmp_path / "s.npz")
    res2 = analyse_run(run2, bell2, labels=meta2["labels"])
    assert res2.summary() == res.summary()
    assert meta2["summary"] == res.summary()


def test_load_tolerates_missing_meta_fields(run):
    buf = io.BytesIO()
    np.savez_compressed(buf, sample_index=run.sample_index, raw=run.raw,
                        gaps=np.zeros((0, 2), dtype=np.int64), rate_hz=np.float64(run.rate_hz),
                        ranges=np.array([16, 2000]), meta_json=np.array("{}"))
    buf.seek(0)
    run2, bell, meta = load_session(buf)
    assert bell == BellProfile()
    assert meta["labels"] == {} and meta["markers"] == []
    assert meta["app_version"] is None and meta["algorithm_version"] is None
    assert meta["summary"] is None and meta["calibration"] is None
    assert run2.rate_estimated is False and run2.statuses == []
    assert run2.packet_times.shape == (0, 2)


def test_save_rejects_nan_in_meta(run):
    with pytest.raises(ValueError):
        save_session(io.BytesIO(), run, BELL, {"summary": {"x": float("nan")}})


def test_load_rejects_foreign_files(tmp_path):
    (tmp_path / "x.npz").write_bytes(b"not a zip")
    with pytest.raises(ValueError, match="not a powerkettle .npz session"):
        load_session(tmp_path / "x.npz")
    np.savez_compressed(tmp_path / "y.npz", a=np.zeros(3))
    with pytest.raises(ValueError, match="not a powerkettle .npz session"):
        load_session(tmp_path / "y.npz")


def test_calibration_roundtrip_exact(run):
    res = analyse_run(run, BELL)
    d = calibration_to_dict(res.calib, carried=False)
    buf = io.BytesIO()
    save_session(buf, run, BELL, {"calibration": d})
    buf.seek(0)
    _, _, meta = load_session(buf)
    assert meta["calibration"] == d and meta["calibration"]["carried"] is False
    c = calibration_from_dict(meta["calibration"])
    for name in ("gyro_bias", "up_s", "out_s", "lever_arm_s", "q0"):
        assert np.array_equal(getattr(c, name), getattr(res.calib, name)), name
    assert (c.still, c.g_measured, c.warnings) == (res.calib.still, res.calib.g_measured,
                                                   res.calib.warnings)


def test_reanalyse_uses_labels_and_stored_calibration(run, LABELS):
    res = analyse_run(run, BELL, labels=LABELS)
    meta = {"labels": LABELS, "calibration": calibration_to_dict(res.calib, carried=False)}
    buf = io.BytesIO()
    save_session(buf, run, BELL, meta)
    buf.seek(0)
    run2, bell2, meta2 = load_session(buf)
    assert reanalyse(run2, bell2, meta2).summary() == res.summary()


def _old_file(run, meta: dict) -> io.BytesIO:
    """A session file as written before labels were keyed by sample index."""
    buf = io.BytesIO()
    np.savez_compressed(buf, sample_index=run.sample_index, raw=run.raw,
                        gaps=np.zeros((0, 2), dtype=np.int64), rate_hz=np.float64(run.rate_hz),
                        ranges=np.array([16, 2000]), meta_json=np.array(json.dumps(meta)))
    buf.seek(0)
    return buf


def test_old_rep_index_labels_migrated_with_summary(run, LABELS):
    summary = analyse_run(run, BELL).summary()
    old = {"labels": {str(k): v for k, v in REP_LABELS.items()}, "summary": summary}
    _, _, meta = load_session(_old_file(run, old))
    assert meta["labels"] == LABELS and meta["load_warnings"] == []
    assert meta["labels_keyed_by"] == "low_sample_index"


def test_old_rep_index_labels_dropped_without_summary(run, tmp_path):
    _, _, meta = load_session(_old_file(run, {"labels": {"3": {"kind": "other"}}}))
    assert meta["labels"] == {}
    assert len(meta["load_warnings"]) == 1 and "dropped" in meta["load_warnings"][0]
    save_session(tmp_path / "s.npz", run, BELL, meta)  # the warning is not stored
    _, _, meta2 = load_session(tmp_path / "s.npz")
    assert meta2["load_warnings"] == []
