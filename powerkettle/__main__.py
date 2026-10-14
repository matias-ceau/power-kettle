"""Command line: python -m powerkettle info FILE.pkraw"""
from __future__ import annotations

import argparse
import sys

from .rawlog import read_rawlog, to_recording


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
        lines.append(
            f"  run {i}: {r.raw.shape[0]} samples, rate {r.rate_hz:.2f} Hz, "
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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="powerkettle")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_info = sub.add_parser("info", help="summarise a .pkraw recording")
    p_info.add_argument("file")
    args = ap.parse_args(argv)
    try:
        print(info(args.file))
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
