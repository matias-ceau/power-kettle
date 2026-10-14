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

Conventions for the meta written by the CLI: ``labels`` maps the rep index (JSON string key) to
``{"kind", "hand"}``; ``markers`` is a list of ``{"phone_ms", "text", "sample_index"}``, the
phone-clock markers already placed on the run's sample index (``pipeline.map_markers``), since
the packet times needed to place them are not stored.

``load_session`` fills meta fields missing from older or foreign files with defaults (labels {},
markers [], app_version / algorithm_version / summary None, the default bell, rate_estimated
False) and restores label keys to int. Statuses and packet times are not stored (the run's rate
is). Stdlib + numpy only.
"""
from __future__ import annotations

import json
import os
import zipfile
from typing import BinaryIO

import numpy as np

from .bell import BellProfile
from .pipeline import ALGORITHM_VERSION
from .rawlog import Run

META_DEFAULTS = {"labels": {}, "markers": [], "app_version": None, "algorithm_version": None,
                 "summary": None}


def save_session(path_or_buffer: str | os.PathLike | BinaryIO, run: Run, bell: BellProfile,
                 meta: dict) -> None:
    """Write `run`, `bell` and `meta` (JSON-safe; NaN is rejected with ValueError)."""
    full = {**meta, "algorithm_version": ALGORITHM_VERSION, "bell": bell.to_dict(),
            "rate_estimated": bool(run.rate_estimated)}
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
    meta = {**META_DEFAULTS, **meta}
    meta["labels"] = {int(k): v for k, v in (meta["labels"] or {}).items()}
    bell = BellProfile.from_dict(meta["bell"]) if meta.get("bell") else BellProfile()
    run = Run(sample_index, raw, gaps, [], rate_hz, acc_g, gyro_dps,
              rate_estimated=bool(meta.get("rate_estimated", False)))
    return run, bell, meta
