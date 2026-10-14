"""Rep-derived power statistics, set metrics and session metrics (spec §7.2, §7.3).

Principle: every power metric derives from *rep power*, so rest never dilutes it. Only
``density_w`` and ``work_rest_ratio`` (session) involve rest, deliberately.

Conventions
-----------
- Power statistics (``PowerStats``, best efforts) use the reps of one kind (snatch / swing
  never mixed; "other" is never in the statistics), excluding reps flagged "gap" and reps with
  a non-finite ``power_w``. Hands are pooled within a kind.
- ``PowerStats``: ``top3`` = mean of the min(3, n) best; ``dropoff`` = (best − mean of the last
  min(3, n) reps) / best (None if best ≤ 0); ``cv`` = population std (ddof = 0) / mean, None
  for n < 2 or mean ≤ 0.
- ``reps_by_kind_hand`` counts every rep, gap reps included (they happened), keyed
  "kind:hand" (e.g. "snatch:L", "swing:-", "other:-").
- ``work_j`` = Σ ``work_con_j`` over non-gap reps with a finite value (any kind).
- ``qualifying`` = non-gap snatch / swing reps (never "other": failed or partial reps do not
  count toward a rep target) whose finite ``power_w`` ≥ ``power_floor_w``; with no floor, all
  non-gap snatch / swing reps. In a session it is summed over all sets.
- Set ``pace_rpm`` = number of reps (all, incl. gap) / set duration in minutes; None for 0 reps
  or a non-positive duration.
- Session: ``active_s`` = Σ (t_end − t_start) over sets; ``rest_s`` = duration_s − active_s;
  ``density_w`` = work_j / duration_s (None if duration_s ≤ 0); ``work_rest_ratio`` =
  active_s / rest_s — time-based (spec "work:rest ratio"), None if rest_s ≤ 0.
- Session ``best_efforts[kind]``: best mean rep power over n consecutive reps of that kind, on
  the session's rep sequence of that kind (sets concatenated in order, rest between sets
  ignored; gap / non-finite reps dropped from the sequence, so the neighbours become adjacent).
Stdlib + numpy only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .reps import Rep, SetResult

BEST_EFFORT_NS = (1, 3, 5, 10, 20, 50)
POWER_KINDS = ("snatch", "swing")


@dataclass
class PowerStats:
    n: int
    avg: float | None
    best: float | None
    top3: float | None
    dropoff: float | None
    cv: float | None


@dataclass
class SetMetrics:
    by_kind: dict[str, PowerStats]
    reps_by_kind_hand: dict[str, int]  # e.g. "snatch:L"
    qualifying: int
    pace_rpm: float | None
    work_j: float


@dataclass
class SessionMetrics:
    reps_by_kind_hand: dict[str, int]
    work_j: float
    by_kind: dict[str, PowerStats]
    best_efforts: dict[str, dict[int, float]]
    density_w: float | None
    active_s: float
    rest_s: float
    work_rest_ratio: float | None
    qualifying: int = 0


def power_stats(powers: Sequence[float]) -> PowerStats:
    """Statistics of a sequence of rep powers in time order (see the module docstring)."""
    p = np.asarray(powers, dtype=float)
    n = len(p)
    if n == 0:
        return PowerStats(0, None, None, None, None, None)
    avg, best = float(np.mean(p)), float(np.max(p))
    top3 = float(np.mean(np.sort(p)[::-1][:3]))
    dropoff = float((best - np.mean(p[-3:])) / best) if best > 0 else None
    cv = float(np.std(p) / avg) if n >= 2 and avg > 0 else None
    return PowerStats(n, avg, best, top3, dropoff, cv)


def best_efforts(powers: Sequence[float], ns: Sequence[int] = BEST_EFFORT_NS) -> dict[int, float]:
    """{n: max rolling mean of n consecutive reps}; windows longer than the sequence omitted."""
    p = np.asarray(powers, dtype=float)
    c = np.concatenate(([0.0], np.cumsum(p)))
    return {int(n): float(np.max(c[n:] - c[:-n]) / n) for n in ns if 0 < n <= len(p)}


def _counts(reps: Sequence[Rep]) -> dict[str, int]:
    out: dict[str, int] = {}
    for r in reps:
        key = f"{r.kind}:{r.hand}"
        out[key] = out.get(key, 0) + 1
    return out


def _usable(r: Rep) -> bool:
    return "gap" not in r.flags and math.isfinite(r.power_w)


def _powers_by_kind(reps: Sequence[Rep]) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for r in reps:
        if r.kind in POWER_KINDS and _usable(r):
            out.setdefault(r.kind, []).append(float(r.power_w))
    return out


def _work(reps: Sequence[Rep]) -> float:
    return float(sum(r.work_con_j for r in reps
                     if "gap" not in r.flags and math.isfinite(r.work_con_j)))


def _qualifying(reps: Sequence[Rep], power_floor_w: float | None) -> int:
    ok = [r for r in reps if r.kind in POWER_KINDS and "gap" not in r.flags]
    if power_floor_w is None:
        return len(ok)
    return sum(1 for r in ok if math.isfinite(r.power_w) and r.power_w >= power_floor_w)


def set_metrics(s: SetResult, power_floor_w: float | None = None) -> SetMetrics:
    """Metrics of one set (spec §7.2)."""
    by_kind = {k: power_stats(v) for k, v in _powers_by_kind(s.reps).items()}
    duration = s.t_end - s.t_start
    pace = len(s.reps) / (duration / 60.0) if s.reps and duration > 0 else None
    return SetMetrics(by_kind, _counts(s.reps), _qualifying(s.reps, power_floor_w), pace,
                      _work(s.reps))


def session_metrics(sets: list[SetResult], duration_s: float,
                    power_floor_w: float | None = None) -> SessionMetrics:
    """Metrics of a session of `sets` lasting `duration_s` (spec §7.3)."""
    reps = [r for s in sets for r in s.reps]
    powers = _powers_by_kind(reps)
    work = _work(reps)
    active = float(sum(s.t_end - s.t_start for s in sets))
    rest = float(duration_s) - active
    return SessionMetrics(
        reps_by_kind_hand=_counts(reps),
        work_j=work,
        by_kind={k: power_stats(v) for k, v in powers.items()},
        best_efforts={k: best_efforts(v) for k, v in powers.items()},
        density_w=work / duration_s if duration_s > 0 else None,
        active_s=active,
        rest_s=rest,
        work_rest_ratio=active / rest if rest > 0 else None,
        qualifying=_qualifying(reps, power_floor_w),
    )
