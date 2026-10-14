"""Session export / import as NumPy ``.npz`` (spec §8.2), written with ``np.savez_compressed``.

Keys
----
- ``sample_index`` int64 (N,), ``raw`` int16 (N, 6): the run's samples as received;
- ``gaps`` int64 (k, 2): (last index before the gap, first index after);
- ``rate_hz`` float64 scalar; ``ranges`` int64 [acc_range_g, gyro_range_dps];
- ``meta_json``: a JSON string holding the caller's meta (labels, markers, app version,
  summary, …) plus ``bell`` (``BellProfile.to_dict``), ``algorithm_version`` (always the
  current one) and ``rate_estimated`` (whether ``rate_hz`` was fitted from status data: kept
  so that re-analysing a loaded run reproduces the same warnings and summary).

``calibration`` (``calibration_to_dict``) is the calibration the run was analysed with, with a
``carried`` flag: a run without a still window (after a reconnect) is analysed with the previous
run's calibration, which cannot be rebuilt from its own samples, and a later run keeps the first
run's mounting, so ``reanalyse`` passes it back to ``analyse_run`` as ``prior_calib`` (its
mounting is kept; gyro bias and q0 are re-derived from the run's own still window if any).

Conventions for the meta written by the CLI: ``labels`` maps the sample index of the rep's low
(JSON string key; ``pipeline`` module docstring) to ``{"kind", "hand"}``, and
``labels_keyed_by`` = "low_sample_index" says so; ``markers`` is a list of
``{"phone_ms", "text", "sample_index"}``, the phone-clock markers already placed on the run's
sample index (``pipeline.map_markers``), since the packet times needed to place them are not
stored.

``load_session`` fills meta fields missing from older or foreign files with defaults (labels {},
markers [], app_version / algorithm_version / summary None, the default bell, rate_estimated
False) and restores label keys to int. Labels of older files (no ``labels_keyed_by``) are keyed
by the rep's index in the session: they are converted with the stored summary's rep list
(index → its low's sample index) when present, otherwise dropped with a message in
``meta["load_warnings"]`` (a list, empty when nothing happened; not saved). Statuses and packet
times are not stored (the run's rate is). Stdlib + numpy only.
"""
from __future__ import annotations

import json
import os
import zipfile
from typing import BinaryIO

import numpy as np

from .bell import BellProfile
from .calibration import Calibration
from .pipeline import ALGORITHM_VERSION, AnalysisConfig, SessionResult, analyse_run
from .rawlog import Run

META_DEFAULTS = {"labels": {}, "markers": [], "app_version": None, "algorithm_version": None,
                 "summary": None, "calibration": None}
LABELS_KEYED_BY = "low_sample_index"
_CALIB_VECTORS = ("gyro_bias", "up_s", "out_s", "lever_arm_s", "q0")


def calibration_to_dict(calib: Calibration, carried: bool) -> dict:
    """JSON-safe dict of `calib` (vectors as lists, ``still`` as [start, stop]) plus the flag
    `carried` (taken over from a previous run). Floats round-trip exactly through JSON."""
    d = {name: [float(x) for x in getattr(calib, name)] for name in _CALIB_VECTORS}
    d.update(still=[int(calib.still.start or 0), int(calib.still.stop)],
             g_measured=float(calib.g_measured), warnings=list(calib.warnings),
             carried=bool(carried))
    return d


def calibration_from_dict(d: dict) -> Calibration:
    """Inverse of ``calibration_to_dict`` (the ``carried`` flag is dropped)."""
    return Calibration(*(np.asarray(d[name], dtype=float) for name in _CALIB_VECTORS),
                       still=slice(*d["still"]), g_measured=float(d["g_measured"]),
                       warnings=list(d.get("warnings", [])))


def _migrate_labels(labels: dict[int, dict], meta: dict, first_index: int) -> dict[int, dict]:
    """Rep-index labels of an older file re-keyed by their rep's low sample index, from the
    stored summary's reps ("index", "i_low" relative to the run's first sample); without it,
    none (a message goes to ``meta["load_warnings"]``)."""
    reps = (meta.get("summary") or {}).get("reps") or []
    lows = {int(r["index"]): first_index + int(r["i_low"]) for r in reps
            if r.get("index") is not None and r.get("i_low") is not None}
    out = {lows[i]: lab for i, lab in labels.items() if i in lows}
    lost = len(labels) - len(out)
    if lost:
        why = "no stored summary" if not lows else "rep index not in the stored summary"
        meta["load_warnings"].append(
            f"{lost} label(s) keyed by rep index dropped ({why}): re-label those reps")
    return out


def reanalyse(run: Run, bell: BellProfile, meta: dict,
              cfg: AnalysisConfig = AnalysisConfig()) -> SessionResult:
    """``analyse_run`` on a loaded session with its stored labels and, as ``prior_calib``, its
    stored calibration (if any)."""
    calib = meta.get("calibration")
    return analyse_run(run, bell, cfg, labels=meta.get("labels") or None,
                       prior_calib=calibration_from_dict(calib) if calib else None)


def save_session(path_or_buffer: str | os.PathLike | BinaryIO, run: Run, bell: BellProfile,
                 meta: dict) -> None:
    """Write `run`, `bell` and `meta` (JSON-safe; NaN is rejected with ValueError)."""
    full = {**meta, "algorithm_version": ALGORITHM_VERSION, "bell": bell.to_dict(),
            "rate_estimated": bool(run.rate_estimated), "labels_keyed_by": LABELS_KEYED_BY}
    full.pop("load_warnings", None)
    full["labels"] = {str(k): v for k, v in full.get("labels", {}).items()}
    meta_json = json.dumps(full, allow_nan=False, separators=(",", ":"))
    np.savez_compressed(
        path_or_buffer,
        sample_index=np.asarray(run.sample_index, dtype=np.int64),
        raw=np.asarray(run.raw, dtype=np.int16),
        gaps=np.asarray(run.gaps, dtype=np.int64).reshape(-1, 2),
        rate_hz=np.float64(run.rate_hz),
        ranges=np.array([run.acc_range_g, run.gyro_range_dps], dtype=np.int64),
        meta_json=np.array(meta_json),
    )


def load_session(path_or_buffer: str | os.PathLike | BinaryIO) -> tuple[Run, BellProfile, dict]:
    """(run, bell, meta) from a file written by ``save_session`` (see the module docstring)."""
    try:
        with np.load(path_or_buffer, allow_pickle=False) as z:
            sample_index = z["sample_index"].astype(np.int64)
            raw = z["raw"].astype(np.int16)
            gaps = [(int(a), int(b)) for a, b in z["gaps"].reshape(-1, 2)]
            rate_hz = float(z["rate_hz"])
            acc_g, gyro_dps = (int(x) for x in z["ranges"])
            meta = json.loads(str(z["meta_json"]))
    except (KeyError, ValueError, EOFError, zipfile.BadZipFile) as e:
        raise ValueError(f"not a powerkettle .npz session: {e}") from None
    meta = {**META_DEFAULTS, **meta, "load_warnings": []}
    labels = {int(k): v for k, v in (meta["labels"] or {}).items()}
    if labels and meta.get("labels_keyed_by") != LABELS_KEYED_BY:
        labels = _migrate_labels(labels, meta, int(sample_index[0]) if len(sample_index) else 0)
    meta["labels"] = labels
    meta["labels_keyed_by"] = LABELS_KEYED_BY
    bell = BellProfile.from_dict(meta["bell"]) if meta.get("bell") else BellProfile()
    run = Run(sample_index, raw, gaps, [], rate_hz, acc_g, gyro_dps,
              rate_estimated=bool(meta.get("rate_estimated", False)))
    return run, bell, meta
