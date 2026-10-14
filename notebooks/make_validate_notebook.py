"""Generate ``validate_recording.ipynb`` (run from anywhere):

    python notebooks/make_validate_notebook.py

The notebook is written without outputs (small in git). To check that it runs top to bottom:

    jupyter nbconvert --to notebook --execute notebooks/validate_recording.ipynb --output-dir /tmp/x

The cell sources live in this file so the notebook can be regenerated and diffed as text.
"""
from __future__ import annotations

from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell, new_notebook

OUT = Path(__file__).with_name("validate_recording.ipynb")

cells = []


def md(text: str) -> None:
    cells.append(new_markdown_cell(text.strip("\n")))


def code(text: str) -> None:
    cells.append(new_code_cell(text.strip("\n")))


# ============================================================================ intro
md(r"""
# Power Kettle: validate a recording

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/matias-ceau/power-kettle/blob/main/notebooks/validate_recording.ipynb)

This notebook takes one recorded `.pkraw` file, runs the same analysis the Power Kettle app will run,
and shows you what it found so that you can check it against what you actually did.

**How to use it**

1. Run the first cell (installs the analysis code; about 20 seconds in Colab).
2. In the *settings* cell, set `PATH` to your recording, for example
   `/content/drive/MyDrive/PowerKettle/pk-20261020-101500.pkraw`.
   Leave it empty (`""`) to try the notebook on a **simulated demo recording** first.
3. Use *Runtime > Run all*, then read the sections below top to bottom.

Each section says in one or two sentences what to look for. Nothing here changes your file.
""")

# ============================================================================ setup
md(r"""
## 1. Set-up

Installs the `powerkettle` package when running in Google Colab (outside Colab it is imported from
this repository) and loads the libraries.
""")
code(r'''
import sys
import os
import subprocess

IN_COLAB = "google.colab" in sys.modules

if IN_COLAB:
    subprocess.run([sys.executable, "-m", "pip", "install", "-q",
                    "git+https://github.com/matias-ceau/power-kettle@main"], check=True)
else:
    # Running locally: use the installed package, else the one in this repository.
    try:
        import powerkettle  # noqa: F401
    except ImportError:
        here = os.path.abspath(os.getcwd())
        for d in (here, os.path.dirname(here)):
            if os.path.isdir(os.path.join(d, "powerkettle")):
                sys.path.insert(0, d)
                break

import dataclasses
import math
import re
import struct
import tempfile
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

from powerkettle.__main__ import info
from powerkettle.anchors import (LOW_GATE_HALF_S, MOTION_MIN_SAMPLES, _prominence,
                                 sagittal_tilt, set_forwards)
from powerkettle.bell import BellProfile
from powerkettle.calibration import CalibrationError, correct_acc_scale
from powerkettle.imu import G, scale_run
from powerkettle.integrate import rest_core
from powerkettle.pipeline import (WARN_CARRIED_CALIBRATION, AnalysisConfig, analyse_run,
                                  map_markers, motion_stages)
from powerkettle.rawlog import RawRecord, read_rawlog, to_recording, write_rawlog
from powerkettle.reps import segment_sets
from powerkettle.session_io import (calibration_to_dict, load_session, reanalyse,
                                    save_session)
from powerkettle.simulate import (Hold, Rest, SimConfig, Snatch, Swing, TableLift, simulate)

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
    "axes.grid": True, "grid.alpha": 0.3, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "figure.dpi": 100,
})
KIND_COLOUR = {"snatch": "#D55E00", "swing": "#0072B2", "other": "#6b6b6b"}  # colour-blind safe
MARKER_COLOUR = "#7b3294"
print("powerkettle ready", "(Colab)" if IN_COLAB else "(local)")
''')

# ============================================================================ settings
md(r"""
## 2. Settings

Only `PATH` is needed. The rest are optional.
""")
code(r'''
# 1) Your recording. Empty string = analyse a simulated demo recording.
PATH = ""   # e.g. "/content/drive/MyDrive/PowerKettle/pk-20261020-101500.pkraw"

# 2) Your bell, only the values that differ from the defaults (32 kg, 63 cm ball, ...).
#    Use the numbers from your bell-profile.txt. Names: mass_kg, circumference_cm,
#    bottom_flat_diam_cm, side_flat_diam_cm, handle_clearance_cm, handle_tube_circumference_cm,
#    handle_length_cm, handle_bar_cm, device_thickness_cm.
BELL_OVERRIDES = {}      # e.g. {"mass_kg": 32, "device_thickness_cm": 1.2}

# 3) Optional.
T_RANGE = None           # zoom the time plots to a window, in seconds from the start of each
                         # run, e.g. (30, 75). None = show everything.
EXPECTED_REPS = []       # reps you really did, set by set in order, e.g. [10, 10, 10, 8].
                         # When given, the notebook compares them with the reps it found.
SAVE_NPZ = ""            # e.g. "/content/drive/MyDrive/PowerKettle/clipA.npz": also save the
                         # analysed run(s) (run 0 to this name, run K to name.runK.npz)

LOOKAHEAD_S = 2.0        # a marker tapped up to this long AFTER the bell left the floor still
                         # counts for that set
MIN_LIFT_M = 0.15        # a rest-to-rest height change smaller than this is not a table lift
''')

# ============================================================================ demo helper
md(r"""
## 3. Demo recording (only used when `PATH` is empty)

The simulator in the package renders a made-up session: a table lift, ten swings, eight snatches
left, eight snatches right with a 5 s hold. It is written as a normal `.pkraw` file, with markers
typed the way you will tap them (`table lift 74cm`, `swing 2H`, `snatch L`, `snatch R`).
Because it comes from a simulator we know the true answer: 74 cm, 10 swings, 8 + 8 snatches.
""")
code(r'''
def make_demo(path):
    """Write a simulated session to `path` as .pkraw (with 1 Hz status records, so the sample rate
    is estimated as for a real recording) and return the path."""
    run, truth = simulate([Rest(6), TableLift(0.74), Rest(6), Swing(10), Rest(10),
                           Snatch(8, "L"), Rest(10), Snatch(8, "R"), Hold(5), Rest(6)])
    t0 = run.packet_times[0, 0]

    def phone_ms(i):                     # phone-clock time of sample i
        return t0 + 1000.0 * i / run.rate_hz

    recs = []
    for st in run.statuses:              # statuses keep flags=1 (streaming): needed for the rate
        recs.append(RawRecord(2, phone_ms(st.sample_index), struct.pack(
            "<BBHHBBHhIIQ", st.version, st.flags, st.battery_mv, st.odr_hz, st.acc_range_g, 0,
            st.gyro_range_dps, st.temp_raw, st.dropped_samples, st.sample_index, st.time_us)))
    for k, (ms, first) in enumerate(run.packet_times):
        first = int(first)
        samples = run.raw[first:first + 19]
        recs.append(RawRecord(1, float(ms), struct.pack("<HIBB", k & 0xFFFF, first, len(samples), 0)
                              + samples.astype("<i2").tobytes()))
    first_move = next(i1 for _, i1, kind in truth.anchor_windows if kind == "rest")  # table lift
    marks = [(0, "rest"), (first_move - 400, "table lift 74cm")]
    for (i_start, _), text in zip(truth.sets, ["swing 2H", "snatch L", "snatch R"]):
        marks.append((i_start - 400, text))      # tapped 1 s before the bell leaves the floor
    recs += [RawRecord(3, phone_ms(i), text.encode("utf-8")) for i, text in marks]
    recs.sort(key=lambda r: r.phone_ms)
    write_rawlog(path, {"app": "powerkettle-record", "v": 1, "demo": True}, recs)
    return path
''')

# ============================================================================ load
md(r"""
## 4. Load the recording

`info` is the same summary as `python -m powerkettle info`. For a good clip you want: `runs: 1`, a rate
close to 400 Hz, `gaps: 0`, and a list of markers that matches what you tapped, in order.
""")
code(r'''
DEMO = PATH.strip() == ""
if DEMO:
    PATH = make_demo(os.path.join(tempfile.gettempdir(), "powerkettle-demo.pkraw"))
    print("PATH is empty: using the simulated demo recording\n")
elif IN_COLAB and PATH.startswith("/content/drive"):
    from google.colab import drive
    drive.mount("/content/drive")

path = Path(PATH).expanduser()
if path.is_dir():
    found = sorted(p.name for p in path.glob("*.pkraw"))
    raise ValueError(f"PATH is a folder. Set it to one file, e.g. one of: {found or 'no .pkraw here'}")
if not path.exists():
    raise FileNotFoundError(f"{path} not found. In Colab, run the Drive mount first and check the "
                            f"spelling (Files panel on the left > drive > MyDrive).")

log = read_rawlog(path)
rec = to_recording(log)
print(info(str(path)))

bell = dataclasses.replace(BellProfile(), **BELL_OVERRIDES)   # a misspelt name raises an error
cfg = AnalysisConfig()
print(f"\nbell: {bell.mass_kg:g} kg, ball circumference {bell.circumference_cm:g} cm, "
      f"device thickness {bell.device_thickness_cm:g} cm")
''')

# ============================================================================ analysis
md(r"""
## 5. Analyse every run

A *run* is one continuous stretch of data (a Bluetooth reconnect starts a new run). Each run is
analysed on its own; a later run keeps the mounting found in the run before it, and a run with no
still period at all re-uses its calibration (a warning says so). Data before the first still
period is analysed too (a warning says how many seconds).

The notebook also keeps the intermediate signals (orientation, velocity, position, anchors) so
that the next sections can plot them.
""")
code(r'''
MARKER_SLACK_MS = 2000.0     # markers tapped this close outside a run's packets still belong to it


def run_markers(run, markers):
    """[(seconds from the run start, text)] of the markers tapped while this run was recording."""
    pt = np.asarray(run.packet_times, dtype=float).reshape(-1, 2)
    if len(pt) == 0:
        return []
    lo, hi = pt[0, 0] - MARKER_SLACK_MS, pt[-1, 0] + MARKER_SLACK_MS
    inside = [(ms, text) for ms, text in markers if lo <= ms <= hi]
    first = int(run.sample_index[0])
    return [((i - first) / run.rate_hz, text) for i, text in map_markers(run, inside)]


def theta_series(imu, motion, calib, cfg):
    """Sagittal tilt (degrees) of the bell-up axis, + = handle leaning towards the front, using
    the forward direction of each set (found from the motion, as the analysis does)."""
    q, anchors = motion.q, motion.anchors
    n = len(q)
    bounds = segment_sets(anchors)
    fwds = set_forwards(q, calib, anchors, cfg.anchors, imu, motion.kin)
    theta = np.empty(n)
    starts = [0] + [b[0] for b in bounds[1:]] + [n]
    for k in range(len(starts) - 1):
        f = fwds[k] if fwds else np.array([1.0, 0.0, 0.0])
        theta[starts[k]:starts[k + 1]] = np.degrees(
            sagittal_tilt(q[starts[k]:starts[k + 1]], calib, f))
    return theta


results = []      # one dict per analysed run
prior = None
for k, run in enumerate(rec.runs):
    if len(run.sample_index) == 0:
        continue
    try:
        res = analyse_run(run, bell, cfg, prior_calib=prior)
    except CalibrationError as e:
        print(f"run {k}: NOT analysed: {e}")
        continue
    prior = res.calib
    imu = correct_acc_scale(scale_run(run), res.calib)   # as analyse_run
    motion = motion_stages(imu, res.calib, bell, cfg)
    results.append({
        "k": k, "run": run, "res": res, "motion": motion, "t": res.t, "imu": imu,
        "markers": run_markers(run, rec.markers),
        "theta": theta_series(imu, motion, res.calib, cfg),
        "speed": np.linalg.norm(motion.v, axis=1),
        "omega": np.linalg.norm(motion.kin.omega_w, axis=1),
    })

# number sets and reps continuously over the whole file
set_no = rep_no = 0
for r in results:
    r["set0"], r["rep0"] = set_no, rep_no
    set_no += len(r["res"].sets)
    rep_no += sum(len(s.reps) for s in r["res"].sets)

for r in results:
    res, run = r["res"], r["run"]
    print(f"run {r['k']}: {res.duration_s:.1f} s at {run.rate_hz:.2f} Hz, "
          f"{sum(len(s.reps) for s in res.sets)} reps in {len(res.sets)} sets, "
          f"gravity read as {res.calib.g_measured:.3f} m/s2 (should be 9.81)")
    for w in res.warnings:
        print(f"   warning: {w}")
if not results:
    raise RuntimeError("no run could be analysed: each needs about 5 s with the bell standing "
                       "still at the start (see the recording protocol)")
''')

# ============================================================================ plots
md(r"""
## 6. Rotation: |ω| and tilt θ

**Top:** how fast the bell is rotating (|ω|, the length of the gyroscope's rotation vector, in rad/s).
**Bottom:** the bell's tilt θ in the forward/backward plane: 0° = standing upright, positive = handle
leaning towards your front, about -90° at the top of a swing.

Shaded grey = still periods where the bell is parked on the floor (rests), shaded orange = still
periods overhead (snatch lockouts). Black triangles are the bottom of each backswing ("lows"); blue
diamonds the top of each swing. The velocity is forced to zero at all of these, so **the picture
should show one low per rep and one top between consecutive lows**. Purple dotted lines are your
markers, with their text.

*Look for:* a missed or extra low/top (a rep the analysis would skip or split), and rests where you
did put the bell down.
""")
code(r'''
def draw_markers(ax, markers, text=False):
    for t, label in markers:
        ax.axvline(t, color=MARKER_COLOUR, ls=":", lw=1.2)
        if text:
            ax.text(t, 0.98, " " + (label if len(label) <= 26 else label[:25] + "…"),
                    transform=ax.get_xaxis_transform(), rotation=90, va="top", ha="right",
                    fontsize=7.5, color=MARKER_COLOUR,
                    bbox=dict(facecolor="white", alpha=0.6, edgecolor="none", pad=0.5))


def shade_anchors(ax, t, anchors):
    for a in anchors:
        if a.kind == "rest":
            ax.axvspan(t[a.i0], t[a.i1], color="#999999", alpha=0.25, lw=0)
        elif a.kind == "top_lockout":
            ax.axvspan(t[a.i0], t[a.i1], color="#E69F00", alpha=0.35, lw=0)


def finish(ax, r):
    if T_RANGE is not None:
        ax.set_xlim(*T_RANGE)


for r in results:
    t, m = r["t"], r["motion"]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True, constrained_layout=True)
    a1.plot(t, r["omega"], color="#009E73", lw=0.9)
    a1.set_ylabel("rotation speed |ω| (rad/s)")
    a1.set_title(f"Run {r['k']}: rotation of the bell")
    a2.plot(t, r["theta"], color="#0072B2", lw=0.9)
    a2.set_ylabel("tilt θ (degrees)")
    a2.set_xlabel("time since the run started (s)")
    for ax in (a1, a2):
        shade_anchors(ax, t, m.anchors)
    lows = [a.i0 for a in m.anchors if a.kind == "low"]
    tops = [a.i0 for a in m.anchors if a.kind == "top_swing"]
    a2.plot(t[lows], r["theta"][lows], "v", color="black", ms=6, label="low (bottom of backswing)")
    a2.plot(t[tops], r["theta"][tops], "D", color="#0072B2", mfc="white", ms=6,
            label="top of a swing")
    a2.legend(handles=a2.get_legend_handles_labels()[0] + [
        Patch(color="#999999", alpha=0.4, label="rest (bell parked)"),
        Patch(color="#E69F00", alpha=0.5, label="lockout (overhead, still)"),
        plt.Line2D([], [], color=MARKER_COLOUR, ls=":", label="your marker")],
        loc="lower right", fontsize=8, ncol=3)
    draw_markers(a1, r["markers"], text=True)
    draw_markers(a2, r["markers"])
    finish(a1, r)
    plt.show()
''')

md(r"""
## 7. Speed of the bell

The speed of the bell's centre (|v|, m/s) from integrating the acceleration between the zero-velocity
points above. It should return to zero at every low, swing top, lockout and rest (black dots).

*Look for:* a smooth hump per rep that touches zero between reps, and similar peak speeds from rep
to rep within a set (the simulated demo peaks at about 4 m/s; your own reps will differ). A hump
that does not return to zero, or gaps in the line (the analysis found invalid data there), mean that
part is unreliable.
""")
code(r'''
for r in results:
    t, m = r["t"], r["motion"]
    fig, ax = plt.subplots(figsize=(12, 3.6), constrained_layout=True)
    ax.plot(t, r["speed"], color="#CC79A7", lw=1.0)
    zero = [a.i0 for a in m.anchors if a.kind in ("low", "top_swing")]
    ax.plot(t[zero], r["speed"][zero], "o", color="black", ms=3.5, label="forced to zero (low / top)")
    shade_anchors(ax, t, m.anchors)
    draw_markers(ax, r["markers"])
    ax.set_ylabel("speed of bell centre |v| (m/s)")
    ax.set_xlabel("time since the run started (s)")
    ax.set_title(f"Run {r['k']}: speed")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_ylim(bottom=0)
    finish(ax, r)
    plt.show()
''')

md(r"""
## 8. Power

Power delivered to the bell (watts) = mass × (acceleration + gravity) · velocity. The **shaded
bands are the concentric phases** (from the low until the top of the swing or the lockout, the part
that counts as the rep's power): orange = snatch, blue = swing, grey = other (partial rep, bell
set down, anything the classifier does not call a rep). Negative power is the bell being lowered.

*Look for:* one band per rep you did, in the right colour, with a clear positive peak inside it.
""")
code(r'''
for r in results:
    t, m, res = r["t"], r["motion"], r["res"]
    fig, ax = plt.subplots(figsize=(12, 4), constrained_layout=True)
    for s in res.sets:
        for rep in s.reps:
            ax.axvspan(t[rep.i_low], t[rep.i_top], color=KIND_COLOUR[rep.kind], alpha=0.25, lw=0)
    ax.plot(t, m.P, color="black", lw=0.9)
    ax.axhline(0, color="black", lw=0.6)
    draw_markers(ax, r["markers"])
    ax.set_ylabel("power to the bell (W)")
    ax.set_xlabel("time since the run started (s)")
    ax.set_title(f"Run {r['k']}: power, concentric phases shaded")
    ax.legend(handles=[Patch(color=KIND_COLOUR[k], alpha=0.4, label=k) for k in KIND_COLOUR]
              + [plt.Line2D([], [], color=MARKER_COLOUR, ls=":", label="your marker")],
              loc="upper right", fontsize=8, ncol=4)
    finish(ax, r)
    plt.show()
''')

# ============================================================================ rep table
md(r"""
## 9. Set and rep table

One block per set, one line per rep (numbered through the whole file). `power` is the average power
over the concentric phase, `peak` the highest instantaneous power, `peak v` the top speed, `height`
how far the bell's centre rose from the low to the top. `flags` can be `relabelled`, `gap` (data
missing inside the rep), `saturated` (the sensor hit its limit), `orientation` (after a data gap
the tilt could not be re-established: kind and height unreliable), `implausible` (an impossible
height or speed: left out of the power statistics) or `low_conf` (the classifier was unsure).

*Look for:* the number of reps per set equals what you counted, and the set's reps all have the same
kind and hand.
""")
code(r'''
def fmt(x, spec):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else format(x, spec)


def set_label(s):
    """Classifier's label of a set, e.g. "snatch L x8" (most common first)."""
    c = Counter(f"{rep.kind} {rep.hand}".replace(" -", "") for rep in s.reps)
    return ", ".join(f"{lab} x{n}" for lab, n in c.most_common()) or "no reps"


for r in results:
    res, t = r["res"], r["t"]
    print(f"===== run {r['k']} =====")
    n = r["rep0"]
    for j, (s, sm) in enumerate(zip(res.sets, res.set_metrics)):
        pace = f", pace {sm.pace_rpm:.1f} reps/min" if sm.pace_rpm else ""
        print(f"\nSet {r['set0'] + j + 1}: {s.t_start:.1f} to {s.t_end:.1f} s, rest before it "
              f"{s.rest_before_s:.1f} s, {len(s.reps)} reps ({set_label(s)}){pace}, "
              f"work {sm.work_j / 1000:.2f} kJ")
        print(f"  {'rep':>3} {'at (s)':>7} {'kind':<6} {'hand':<4} {'power W':>8} {'peak W':>7} "
              f"{'peak v':>6} {'height m':>8}  flags")
        for rep in s.reps:
            n += 1
            print(f"  {n:>3} {t[rep.i_low]:>7.2f} {rep.kind:<6} {rep.hand:<4} "
                  f"{fmt(rep.power_w, '.0f'):>8} {fmt(rep.peak_power_w, '.0f'):>7} "
                  f"{fmt(rep.peak_velocity_ms, '.2f'):>6} {fmt(rep.apex_m, '.2f'):>8}  "
                  f"{','.join(sorted(rep.flags)) or '-'}")
    mt = res.metrics
    print()
    for kind, st in sorted(mt.by_kind.items()):
        print(f"session {kind}: {st.n} reps, average {fmt(st.avg, '.0f')} W, "
              f"best {fmt(st.best, '.0f')} W, top-3 average {fmt(st.top3, '.0f')} W")
    print(f"session: work {mt.work_j / 1000:.2f} kJ, active {mt.active_s:.0f} s, rest {mt.rest_s:.0f} s")
''')

# ============================================================================ paths
md(r"""
## 10. Side view of every rep

Path of the bell's centre during each rep, seen from the side: horizontal axis = distance forward
(away from you is positive), vertical axis = height gained since the low. All reps of a set are
overlaid, **light = first rep, dark = last rep**. The athlete's forward direction is worked out per
set, so the picture does not depend on which way you faced in the room.

*Look for:* snatch paths that rise to a lockout overhead, swing paths in an arc up to about chest
height (compare the heights with the tape or your video); a consistent shape from rep to rep; the
odd rep that is clearly different (your partial rep, a re-grip).
""")
code(r'''
for r in results:
    res = r["res"]
    sets = [(j, s) for j, s in enumerate(res.sets) if s.reps]
    if not sets:
        continue
    ncol = min(4, len(sets))
    nrow = math.ceil(len(sets) / ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 3.4 * nrow), squeeze=False,
                             constrained_layout=True)
    for ax in axes.ravel():
        ax.set_visible(False)
    for ax, (j, s) in zip(axes.ravel(), sets):
        ax.set_visible(True)
        cols = plt.cm.viridis_r(np.linspace(0.05, 0.85, max(len(s.reps), 2)))
        for c, rep in zip(cols, s.reps):
            if rep.path is not None and np.isfinite(rep.path).any():
                ax.plot(rep.path[:, 0], rep.path[:, 2], color=c, lw=1.2)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"Set {r['set0'] + j + 1}: {set_label(s)}", fontsize=9)
        ax.set_xlabel("forward (m)")
        ax.set_ylabel("height gained (m)")
    fig.suptitle(f"Run {r['k']}: side view, light = first rep, dark = last", fontweight="bold")
    plt.show()
''')

# ============================================================================ markers
md(r"""
## 11. Your markers against the classifier

For every set the notebook looks for the **movement marker you tapped for it** (`snatch L`, `snatch R`,
`swing 2H`, `swing L`, `swing R`; the last one tapped after the previous set ended and up to 2 s
after the bell left the floor), and compares it rep by rep with what the classifier decided.

- A snatch marker with a hand (`snatch L`) expects snatch + that hand; `snatch` alone accepts either
  hand. A swing marker expects swing (the classifier does not tell the hand of a swing).
- A marker that contains `partial rep N` (for example `snatch R partial rep 6`) expects the Nth rep to
  be classified `other`, as the protocol asks.
- Other markers (`rest`, `table lift 74cm`, `overhead hold`, notes) are not compared.

If you filled in `EXPECTED_REPS` in the settings cell, the rep count is checked too.

*Look for:* agreement near 100 %. A disagreement tells you which rep to look at in the plots above
(it may be the analysis, or a marker you tapped late or for the wrong set).
""")
code(r'''
MOVE_RE = re.compile(r"^\s*(snatch|swing)s?\s*(L|R|2H)?(?![A-Za-z])", re.I)
PARTIAL_RE = re.compile(r"partial\s*rep\s*(\d+)", re.I)


def parse_marker(text):
    """(kind, hand or None, partial rep number or None) for a movement marker, else None."""
    m = MOVE_RE.match(text)
    if not m:
        return None
    hand = (m.group(2) or "").upper()
    partial = PARTIAL_RE.search(text)
    return (m.group(1).lower(), None if hand in ("", "2H") else hand,
            int(partial.group(1)) if partial else None)


rows = []        # one dict per set, in file order
agree_total = reps_total = 0
for r in results:
    res = r["res"]
    prev_end = -math.inf
    for j, s in enumerate(res.sets):
        n_set = r["set0"] + j + 1
        cands = [(t, txt) for t, txt in r["markers"]
                 if prev_end <= t <= s.t_start + LOOKAHEAD_S and parse_marker(txt)]
        prev_end = s.t_end
        line = f"Set {n_set} ({s.t_start:.1f} to {s.t_end:.1f} s): classifier says {set_label(s)}"
        if not cands:
            rows.append((line + "\n    no new movement marker before this set: not compared", None))
            continue
        _, text = cands[-1]
        kind, hand, partial = parse_marker(text)
        bad = []
        for i, rep in enumerate(s.reps, start=1):
            if partial == i:
                want_kind, want_hand = "other", None
            else:
                want_kind, want_hand = kind, hand if kind == "snatch" else None
            ok = rep.kind == want_kind and (want_hand is None or rep.hand == want_hand)
            if not ok:
                bad.append(f"rep {i}: marker expects {want_kind}{' ' + want_hand if want_hand else ''}, "
                           f"classifier says {rep.kind}{'' if rep.hand == '-' else ' ' + rep.hand}")
        n_ok = len(s.reps) - len(bad)
        agree_total += n_ok
        reps_total += len(s.reps)
        pct = 100.0 * n_ok / len(s.reps) if s.reps else float("nan")
        line += f"\n    marker \"{text}\": {n_ok}/{len(s.reps)} reps agree ({fmt(pct, '.0f')} %)"
        line += "".join(f"\n      - {b}" for b in bad)
        rows.append((line, (n_ok, len(s.reps))))

# optional rep-count check
all_sets = [s for r in results for s in r["res"].sets]
for i, (line, st) in enumerate(rows):
    if i < len(EXPECTED_REPS):
        found, want = len(all_sets[i].reps), EXPECTED_REPS[i]
        line += f"\n    rep count: found {found}, you said {want}" + ("" if found == want else "   <-- differs")
        rows[i] = (line, st)

for line, _ in rows:
    print(line)
if reps_total:
    print(f"\nOverall: {agree_total}/{reps_total} reps match their marker "
          f"({100.0 * agree_total / reps_total:.0f} %)")
else:
    print("\nNo set had a movement marker to compare with.")
if len(EXPECTED_REPS) > len(all_sets):
    print(f"note: EXPECTED_REPS lists {len(EXPECTED_REPS)} sets, {len(all_sets)} found")
''')

# ============================================================================ table lift
md(r"""
## 12. Table lift: measured against the tape

For the *table lift* exercise (protocol step 1) the bell is lifted from the floor onto a table and back.
The analysis never sees the table, only the movement, so this is an independent test of the
position estimate: the height the bell **moved up** between two rests should match the table height
you measured with the tape (the bell moves as high as the table top).

The height comes from the marker (`table lift 74cm`; `cm`, `mm` and `m` are understood). Every
rest-to-rest height change above 15 cm is listed: lifts up are compared with the marker, the way
back down is shown as a check (it should be the same size with a minus sign).

*Look for:* an error of a couple of centimetres or less. Much more usually means the slow lift was
not recognised as still-to-still (see the rotation plot) or the bell was not set flat on the table.
""")
code(r'''
LIFT_RE = re.compile(r"table\s*lift", re.I)
HEIGHT_RE = re.compile(r"(\d+(?:[.,]\d+)?)\s*(mm|cm|m)\b", re.I)     # a number with its unit
BARE_RE = re.compile(r"\s*(\d+(?:[.,]\d+)?)(?![\d.,])")                # right after "table lift"


def marker_height_m(text):
    """Height in m from a marker like "table lift 74cm" or "table lift #2: 0.74 m": the first
    number with a unit (mm, cm, m) after "table lift"; else a bare number right after it
    ("table lift 74": cm if >= 3, else m); else None."""
    lift = LIFT_RE.search(text)
    if not lift:
        return None
    rest = text[lift.end():]
    m = HEIGHT_RE.search(rest)
    if m:
        v, unit = float(m.group(1).replace(",", ".")), m.group(2).lower()
    else:
        m = BARE_RE.match(rest)
        if not m:
            return None
        v = float(m.group(1).replace(",", "."))
        unit = "cm" if v >= 3 else "m"
    return v * {"mm": 1e-3, "cm": 1e-2, "m": 1.0}[unit]


any_lift = False
errors = []
for r in results:
    t, m, run = r["t"], r["motion"], r["run"]
    fs = run.rate_hz
    anchors = sorted(m.anchors)
    heights = [(tt, marker_height_m(txt)) for tt, txt in r["markers"] if marker_height_m(txt)]
    for a, b in zip(anchors[:-1], anchors[1:]):
        if a.kind != "rest" or b.kind != "rest":
            continue
        c0, c1 = rest_core(b, fs)
        dz = float(np.nanmedian(m.pos[c0:c1 + 1, 2])) if np.isfinite(m.pos[c0:c1 + 1, 2]).any() else float("nan")
        if not abs(dz) >= MIN_LIFT_M:
            continue
        any_lift = True
        when = f"run {r['k']}, rest at {t[a.i1]:.1f} s -> rest at {t[b.i0]:.1f} s"
        if dz > 0:
            marked = [h for tt, h in heights if tt <= t[a.i1] + LOOKAHEAD_S]
            ref = marked[-1] if marked else (heights[0][1] if heights else None)
            if ref is None:
                print(f"{when}: moved up {100 * dz:.1f} cm   (no 'table lift <height>' marker to compare)")
            else:
                errors.append(100 * (dz - ref))
                print(f"{when}: measured up {100 * dz:.1f} cm, marker {100 * ref:.1f} cm, "
                      f"difference {100 * (dz - ref):+.1f} cm ({100 * (dz - ref) / ref:+.1f} %)")
        else:
            print(f"{when}: moved down {-100 * dz:.1f} cm")
if not any_lift:
    print("No rest-to-rest height change above 15 cm found: no table lift in this recording "
          "(or the lift was not recognised; check the rotation plot).")
    if any(marker_height_m(txt) for r in results for _, txt in r["markers"]):
        print("A 'table lift <height>' marker exists, so one was expected.")
elif errors:
    print(f"\n{len(errors)} lift(s): mean difference {np.mean(errors):+.1f} cm, "
          f"worst {max(errors, key=abs):+.1f} cm")
''')

# ============================================================================ margins
md(r"""
## 13. Threshold margins

The analysis decides with a few thresholds (``AnalysisConfig``). This section shows **how far your
data sits from each one**, so the first real recordings tell whether a threshold is about to bite:

- **|f| at each low** (in g; the largest within ±25 ms of the low) against `low_min_f_g`: a low needs
  the arm pulling the bell out of the backswing.
- **θ prominence of each low** (degrees) against `prominence_deg`: how far the tilt falls on both
  sides before rising again.
- **Static time of each lockout** (s) against `static_min_s` (the shortest still window seen) and the
  classifier's `snatch_min_static_s`.
- **Rest-based forward vs motion axis** per stretch between rests (degrees): the analysis takes the
  forward/backward plane from the motion; the bell's parked pose would give a forward this far off
  (0° when you park the bell facing you; large is fine, it shows the motion is needed).

*Look for:* values well clear of the dashed threshold lines. Values close to a line are the reps the
analysis may miss or misclassify on another day.
""")
code(r'''
acfg, ccfg = cfg.anchors, cfg.classifier
low_f, low_prom, lock_s, fwd_angle = [], [], [], []
for r in results:
    m, imu = r["motion"], r["imu"]
    half = max(0, round(LOW_GATE_HALF_S * imu.fs))
    f_norm = np.where(imu.valid, m.kin.f_norm, np.nan)
    for st in m.stretches:
        if st.s1 - st.s0 < 3:
            continue
        th = sagittal_tilt(m.q[st.s0:st.s1], r["res"].calib, st.forward)
        for a in m.raw_anchors:
            if a.kind == "low" and st.s0 <= a.i0 < st.s1:
                win = f_norm[max(0, a.i0 - half):a.i0 + half + 1]
                low_f.append(np.nanmax(win) / G if np.isfinite(win).any() else np.nan)
                low_prom.append(math.degrees(_prominence(th, a.i0 - st.s0)))
        if st.motion.moving >= MOTION_MIN_SAMPLES:
            fwd_angle.append(math.degrees(math.acos(np.clip(st.hint @ st.forward, -1, 1))))
    lock_s += [(a.i1 - a.i0 + 1) / imu.fs for a in m.anchors if a.kind == "top_lockout"]


def margin_line(name, values, threshold, unit, above=True):
    v = np.asarray([x for x in values if np.isfinite(x)])
    if len(v) == 0:
        print(f"{name}: none")
        return
    worst = v.min() if above else v.max()
    print(f"{name}: n = {len(v)}, min {v.min():.2f}, median {np.median(v):.2f}, max {v.max():.2f} "
          f"{unit}; threshold {threshold:g} {unit}, closest margin {worst - threshold:+.2f} {unit}")


margin_line("|f| at lows", low_f, acfg.low_min_f_g, "g")
margin_line("θ prominence of lows", low_prom, acfg.prominence_deg, "deg")
margin_line("lockout static time", lock_s, ccfg.snatch_min_static_s, "s")
print(f"  (still windows shorter than static_min_s = {acfg.static_min_s:g} s are not seen at all)")
if fwd_angle:
    print("rest-based forward vs motion axis per stretch: "
          + ", ".join(f"{x:.0f}°" for x in fwd_angle))

fig, axes = plt.subplots(1, 4, figsize=(14, 3.2), constrained_layout=True)
panels = [(low_f, acfg.low_min_f_g, "|f| at lows (g)"),
          (low_prom, acfg.prominence_deg, "θ prominence of lows (deg)"),
          (lock_s, ccfg.snatch_min_static_s, "lockout static time (s)"),
          (fwd_angle, None, "rest forward vs motion axis (deg)")]
for ax, (vals, thr, title) in zip(axes, panels):
    vals = [x for x in vals if np.isfinite(x)]
    ax.plot(np.zeros(len(vals)) + np.random.default_rng(0).uniform(-0.2, 0.2, len(vals)), vals,
            "o", ms=4, alpha=0.6, color="#0072B2")
    if thr is not None:
        ax.axhline(thr, color="#D55E00", ls="--", lw=1.2, label="threshold")
        ax.legend(loc="best", fontsize=8)
    ax.set_xticks([])
    ax.set_xlim(-1, 1)
    ax.set_title(title, fontsize=9)
plt.show()
''')

# ============================================================================ save
md(r"""
## 14. Optional: save the analysed session

Set `SAVE_NPZ` in the settings cell to keep the analysed run(s) as `.npz` files (raw data, bell,
markers, calibration, summary). The notebook then re-loads each file and re-runs the analysis to prove
that the saved session reproduces the same summary.
""")
code(r'''
if not SAVE_NPZ:
    print("SAVE_NPZ is empty: nothing saved.")
else:
    out = Path(SAVE_NPZ).expanduser()
    if out.suffix.lower() != ".npz":
        out = out.with_name(out.name + ".npz")
    for r in results:
        res, run = r["res"], r["run"]
        dest = out if r["k"] == 0 else out.with_name(f"{out.stem}.run{r['k']}{out.suffix}")
        markers = [{"phone_ms": ms, "text": text, "sample_index": i}
                   for (ms, text), (i, _) in zip(rec.markers, map_markers(run, rec.markers))]
        meta = {"labels": {}, "markers": markers, "app_version": None,
                "calibration": calibration_to_dict(res.calib, WARN_CARRIED_CALIBRATION in res.warnings),
                "summary": res.summary()}
        save_session(dest, run, bell, meta)
        run2, bell2, meta2 = load_session(dest)
        same = reanalyse(run2, bell2, meta2).summary() == meta2["summary"]
        print(f"saved {dest}; re-analysis of the saved file reproduces the summary: {same}")
''')

# ============================================================================ checklist
md(r"""
## What to look for: checklist

| Section | Good sign | Worth a closer look |
|---|---|---|
| 4 Load | `runs: 1`, about 400 Hz, `gaps: 0`, markers as tapped | several runs, big gaps, rate far from 400 |
| 5 Analyse | gravity close to 9.81, no warnings | calibration warnings, "carried over" |
| 6 Rotation | one low per rep, a top between lows, rests where the bell was parked | missing or extra lows/tops |
| 7 Speed | humps returning to zero between reps, similar peaks within a set | humps that stay above zero, long gaps |
| 8 Power | one correctly coloured band per rep with a clear peak | missing bands, wrong colour |
| 9 Table | rep counts equal your counts | extra/missing reps, mixed kinds in one set |
| 10 Side view | the same shape rep after rep | a very different path (partial rep?) |
| 11 Markers | agreement near 100 %, partial rep detected as `other` | a rep that disagrees: which one, and why |
| 12 Table lift | within about 2 cm of the tape | several cm out |
| 13 Margins | values well clear of the dashed lines | values close to a threshold |

Anything unexpected: note the file name and the section, so it can be investigated against the
raw data.
""")

# ============================================================================ write
for i, c in enumerate(cells):  # deterministic ids: regenerating gives an identical file
    c["id"] = f"cell-{i:02d}"
nb = new_notebook(cells=cells, metadata={
    "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
    "language_info": {"name": "python"},
    "colab": {"provenance": [], "name": "validate_recording.ipynb"},
})
nbformat.validate(nb)
nbformat.write(nb, OUT)
print(f"wrote {OUT}")
