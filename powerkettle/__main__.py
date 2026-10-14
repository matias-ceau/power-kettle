"""Command line:

    python -m powerkettle info FILE.pkraw
    python -m powerkettle analyse FILE(.pkraw|.npz) [--bell bell.json] [--out out.npz]

``analyse`` runs ``pipeline.analyse_run`` on every run of the recording (a Bluetooth reconnect
starts a new run; a run without a still window reuses the previous run's calibration) and prints
one line per set (with the nearest marker at or before the set's start) and a rep table. A
``.npz`` session brings its own bell and labels; ``--bell`` overrides the bell. ``--out`` saves
each run as a ``.npz`` session (``out.npz``; ``out.run1.npz``, … for later runs).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from .bell import BellProfile
from .calibration import CalibrationError
from .pipeline import SessionResult, analyse_run, map_markers
from .rawlog import Run, read_rawlog, to_recording
from .session_io import load_session, save_session


def info(path: str) -> str:
    log = read_rawlog(path)
    rec = to_recording(log)
    times = [r.phone_ms for r in log.records]
    duration_s = (max(times) - min(times)) / 1000.0 if times else 0.0  # wall-clock span of the log
    lines = [
        f"file: {path}",
        f"duration: {duration_s:.3f} s",
        f"runs: {len(rec.runs)}",
        f"samples: {sum(r.raw.shape[0] for r in rec.runs)}",
    ]
    for i, r in enumerate(rec.runs):
        dropped = sum(b - a - 1 for a, b in r.gaps)
        rate_note = "" if r.rate_estimated else " (nominal: no status data)"
        lines.append(
            f"  run {i}: {r.raw.shape[0]} samples, rate {r.rate_hz:.2f} Hz{rate_note}, "
            f"{r.acc_range_g} g / {r.gyro_range_dps} dps, "
            f"gaps {len(r.gaps)} ({dropped} samples dropped)"
        )
    total_gaps = sum(len(r.gaps) for r in rec.runs)
    total_dropped = sum(b - a - 1 for r in rec.runs for a, b in r.gaps)
    lines.append(f"gaps: {total_gaps} ({total_dropped} samples dropped)")
    lines.append(f"markers: {len(rec.markers)}")
    for ms, text in rec.markers:
        lines.append(f"  {ms:.1f} ms  {text}")
    return "\n".join(lines)


def _f(x, spec: str) -> str:
    """`x` formatted with `spec`, "-" for None / NaN."""
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) else format(x, spec)


def _app_version(meta: dict) -> str | None:
    return f"{meta['app']}/{meta['v']}" if "app" in meta and "v" in meta else None


def _load(path: str) -> tuple[list[tuple[Run, list[dict]]], BellProfile, dict, str | None]:
    """([(run, markers)], bell, labels, app version) of a .pkraw or .npz file; markers as
    {"phone_ms", "text", "sample_index"} dicts."""
    if Path(path).suffix.lower() == ".npz":
        run, bell, meta = load_session(path)
        return [(run, list(meta["markers"]))], bell, meta["labels"], meta["app_version"]
    rec = to_recording(read_rawlog(path))
    runs = []
    for run in rec.runs:
        mapped = map_markers(run, rec.markers)
        runs.append((run, [{"phone_ms": ms, "text": text, "sample_index": i}
                           for (ms, text), (i, _) in zip(rec.markers, mapped)]))
    return runs, BellProfile(), {}, _app_version(rec.meta)


def _marker_before(markers: list[dict], sample_index: int) -> str | None:
    before = [m for m in markers if m.get("sample_index") is not None
              and m["sample_index"] <= sample_index]
    return max(before, key=lambda m: m["sample_index"])["text"] if before else None


def _report(k: int, run: Run, res: SessionResult, markers: list[dict]) -> list[str]:
    first = int(run.sample_index[0])
    lines = [f"run {k}: {res.duration_s:.1f} s at {run.rate_hz:.2f} Hz, "
             f"{sum(len(s.reps) for s in res.sets)} reps in {len(res.sets)} sets, "
             f"|g| {res.calib.g_measured:.3f} m/s²"]
    lines += [f"  warning: {w}" for w in res.warnings]
    index = 0
    for j, (s, sm) in enumerate(zip(res.sets, res.set_metrics)):
        counts = ", ".join(f"{key} {n}" for key, n in sorted(sm.reps_by_kind_hand.items()))
        power = "; ".join(
            f"{kind} avg {_f(st.avg, '.0f')} W best {_f(st.best, '.0f')} W drop-off "
            f"{_f(None if st.dropoff is None else 100 * st.dropoff, '.1f')} %"
            for kind, st in sorted(sm.by_kind.items()))
        marker = _marker_before(markers, first + s.i_start)
        lines.append(f"set {j + 1}: {s.t_start:.1f}-{s.t_end:.1f} s, rest before "
                     f"{s.rest_before_s:.1f} s, {len(s.reps)} reps ({counts})"
                     + (f", {power}" if power else "")
                     + f", pace {_f(sm.pace_rpm, '.1f')} /min, work {sm.work_j / 1000:.2f} kJ"
                     + (f', marker "{marker}"' if marker is not None else ""))
        lines.append(f"  {'#':>3} {'time_s':>7} {'kind':<6} {'hand':<4} {'power_W':>7} "
                     f"{'peak_W':>7} {'peak_v':>6} {'apex_m':>6} flags")
        for r in s.reps:
            lines.append(f"  {index:>3} {res.t[r.i_low]:>7.2f} {r.kind:<6} {r.hand:<4} "
                         f"{_f(r.power_w, '.0f'):>7} {_f(r.peak_power_w, '.0f'):>7} "
                         f"{_f(r.peak_velocity_ms, '.2f'):>6} {_f(r.apex_m, '.2f'):>6} "
                         f"{','.join(sorted(r.flags)) or '-'}")
            index += 1
    m = res.metrics
    for kind, st in sorted(m.by_kind.items()):
        lines.append(f"session {kind}: {st.n} reps, avg {_f(st.avg, '.0f')} W, "
                     f"best {_f(st.best, '.0f')} W, top-3 {_f(st.top3, '.0f')} W")
    lines.append(f"session: work {m.work_j / 1000:.2f} kJ, density {_f(m.density_w, '.0f')} W, "
                 f"active {m.active_s:.0f} s, rest {m.rest_s:.0f} s")
    return lines


def _out_path(out: str, k: int) -> Path:
    p = Path(out)
    return p if k == 0 else p.with_name(f"{p.stem}.run{k}{p.suffix or '.npz'}")


def analyse(path: str, bell_path: str | None = None, out: str | None = None) -> int:
    runs, bell, labels, app_version = _load(path)
    if bell_path is not None:
        with open(bell_path, encoding="utf-8") as f:
            bell = BellProfile.from_dict(json.load(f))
    print(f"file: {path}")
    print(f"bell: {bell.mass_kg:g} kg")
    analysed, prior = 0, None
    for k, (run, markers) in enumerate(runs):
        if len(run.sample_index) == 0:
            continue
        try:
            res = analyse_run(run, bell, labels=labels, prior_calib=prior)
        except CalibrationError as e:
            print(f"run {k}: not analysed: {e}")
            continue
        prior = res.calib
        analysed += 1
        print("\n".join(_report(k, run, res, markers)))
        if out is not None:
            meta = {"labels": labels, "markers": markers, "app_version": app_version,
                    "summary": res.summary()}
            save_session(_out_path(out, k), run, bell, meta)
            print(f"saved: {_out_path(out, k)}")
    return 0 if analysed else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="powerkettle")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_info = sub.add_parser("info", help="summarise a .pkraw recording")
    p_info.add_argument("file")
    p_an = sub.add_parser("analyse", help="analyse a .pkraw recording or .npz session")
    p_an.add_argument("file")
    p_an.add_argument("--bell", help="bell profile JSON (BellProfile fields)")
    p_an.add_argument("--out", help="save the session(s) as .npz")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "analyse":
            return analyse(args.file, args.bell, args.out)
        print(info(args.file))
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
