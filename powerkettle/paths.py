"""Per-rep paths of the bell's centre of mass in the athlete frame (x forward, y left, z up).

The estimate's world frame has an arbitrary heading, so a rep's path is expressed in the frame
of the lifter: ``athlete_forward`` finds the horizontal direction the bell moves along in a set
(first principal axis of the horizontal velocity, its sign taken from the rest-based forward
``anchors.set_forwards``), and ``rep_path`` rotates the relative position into that frame and
decimates it for display. Stdlib + numpy only.
"""
from __future__ import annotations

import numpy as np

MIN_MOVING_SAMPLES = 50  # fewer finite moving samples in a set: the hint is used as is
MOVING_SPEED_MS = 0.1  # horizontal speed above which a sample counts as moving


def _horizontal_unit(w: np.ndarray) -> np.ndarray:
    h = np.array([w[0], w[1], 0.0], dtype=float)
    n = np.linalg.norm(h)
    if not n > 0.0:
        raise ValueError("forward hint has no horizontal component")
    return h / n


def athlete_forward(v: np.ndarray, set_slice: slice, hint_w: np.ndarray) -> np.ndarray:
    """Unit horizontal forward (z = 0) of a set: first principal axis (uncentred, so a back and
    forth swing is one axis) of the horizontal velocity (x, y) over the finite moving samples of
    `v[set_slice]`, signed to have a positive dot with `hint_w`. With fewer than
    ``MIN_MOVING_SAMPLES`` such samples, the horizontal part of `hint_w`."""
    hint = _horizontal_unit(np.asarray(hint_w, dtype=float))
    vh = np.asarray(v, dtype=float)[set_slice, :2]
    vh = vh[np.all(np.isfinite(vh), axis=1)]
    vh = vh[np.linalg.norm(vh, axis=1) > MOVING_SPEED_MS]
    if len(vh) < MIN_MOVING_SAMPLES:
        return hint
    _, vecs = np.linalg.eigh(vh.T @ vh)  # ascending: the last is the principal axis
    axis = vecs[:, -1]
    f = np.array([axis[0], axis[1], 0.0])
    return f if f @ hint >= 0.0 else -f


def rep_path(pos: np.ndarray, seg: tuple[int, int, int, int], forward_w: np.ndarray,
             decimate_to_hz: float = 50, fs: float = 400) -> np.ndarray:
    """(k, 3) path of the rep `seg` = (i_low, i_top, i_top_end, i_end) in the athlete frame: the
    position relative to its value at ``i_low``, over [i_low, i_end], rotated so x = forward,
    y = left (ẑ × forward), z = up, and decimated from `fs` to `decimate_to_hz` by stride
    ``round(fs / decimate_to_hz)``. NaN rows stay NaN; a NaN at ``pos[i_low]`` (the origin) makes
    the whole path NaN, and the decimated path may omit the sample ``i_end`` itself."""
    i_low, i_end = int(seg[0]), int(seg[3])
    pos = np.asarray(pos, dtype=float)
    rel = pos[i_low:i_end + 1] - pos[i_low]
    fwd = _horizontal_unit(np.asarray(forward_w, dtype=float))
    left = np.array([-fwd[1], fwd[0], 0.0])
    basis = np.stack([fwd, left, np.array([0.0, 0.0, 1.0])], axis=1)  # athlete <- world columns
    stride = max(1, round(fs / decimate_to_hz))
    return (rel @ basis)[::stride]
