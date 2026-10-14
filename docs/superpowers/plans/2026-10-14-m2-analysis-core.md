# Power Kettle M2 — Analysis Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The `powerkettle` Python analysis core: from a raw IMU run to calibrated orientation, centre-of-mass velocity and power, reps and sets, snatch/swing/other classification with hand, rep paths and all spec §7 metrics — batch and incremental (live) — validated against a physics simulator of the actual bell.

**Architecture:** Small numpy-only modules, one stage each (bell → imu → calibration → orientation → kinematics → anchors → integrate → reps/classify → metrics/paths → pipeline). A rigid-body simulator renders known motions of the profiled bell into BMI160 readings so every stage is tested against ground truth. `pipeline.analyse_run` composes the stages; `LiveAnalyser` wraps it incrementally.

**Tech Stack:** Python ≥ 3.11, numpy, pytest; matplotlib only in the notebook.

**Spec:** `docs/superpowers/specs/2026-10-14-power-kettle-design.md` (§3.4, §5, §6, §7, §8.2 export format, §9.1). Builds on M1 plan `docs/superpowers/plans/2026-10-14-m1-data-pipe.md` (Task 1–2: `protocol`, `rawlog.Run`, `rawlog.Recording`).

## Global Constraints

- `powerkettle/` imports only stdlib + numpy (Pyodide). No scipy.
- Units: SI internally (m, s, m/s, m/s², rad, rad/s, W, J); degrees only in config/feature fields named `*_deg`.
- g = 9.80665 m/s². World frame: z up. Athlete frame: x forward, y left, z up. Bell frame: x outward normal of the front flat (towards lifter at hike), z bell-up (base → handle), y = z × x.
- Sensor axis normal to the box base = `CalibrationConfig.normal_axis` (default 2) with sign `normal_sign` (default +1) = bell outward.
- Accelerometer measures specific force f = a − g_w; so a = R·f + g_w with g_w = (0, 0, −g).
- Power P = m · (a_com + g ẑ) · v_com. Rep power = concentric work / concentric duration.
- Default bell: 32 kg; circumference 63 cm; bottom flat Ø 12 cm; front/back flat Ø 10 cm; handle clearance 9 cm; handle tube circumference 13 cm, length 44 cm; device thickness 1.0 cm.
- Metric definitions exactly as spec §7 (drop-off = (best − mean of last 3)/best; CV population (ddof = 0); best efforts over 1, 3, 5, 10, 20, 50 consecutive reps).
- If a test tolerance below cannot be met, stop and report the measured error — never loosen a tolerance silently.

## Review Focus

1. Recording with no still period at the start → `CalibrationError("no still window …")`, not silent garbage. (Task 4 test.)
2. Data gap inside a rep → rep flagged `"gap"` and excluded from metrics; analysis completes. (Task 12 test.)
3. Sessions with 0, 1 or 2 reps → metrics return `None` fields where undefined, never raise. (Task 10 test.)
4. Bell laid on its side between sets (not upright) → still detected as floor rest, so the set ends. (Task 7 test.)
5. Accelerometer saturation (±16 g) on a hard catch → rep flagged `"saturated"`, still counted. (Task 12 test.)

---

### Task 1: Bell geometry and quaternion utilities

**Files:** Create `powerkettle/bell.py`, `powerkettle/quat.py`. Test `tests/test_bell.py`, `tests/test_quat.py`.

**Interfaces:**
- `@dataclass(frozen=True) class BellProfile: mass_kg=32.0; circumference_cm=63.0; bottom_flat_diam_cm=12.0; side_flat_diam_cm=10.0; handle_clearance_cm=9.0; handle_tube_circumference_cm=13.0; handle_length_cm=44.0; handle_bar_cm=16.0; device_thickness_cm=1.0` with `to_dict()` / `from_dict()`.
- `ball_radius_m(b) -> float`, `side_flat_distance_m(b) -> float`, `com_offset_m(b) -> float` (COM height above ball centre), `lever_arm_bell_m(b) -> np.ndarray` (3,) in bell frame, `model_mass_kg(b, density=7100.0) -> float`.
  Model: sphere minus bottom cap and two side caps; handle = inverted U (top bar `handle_bar_cm` at centreline height R + clearance + tube radius, two legs of length (L − bar)/2 descending from it) as a tube of the given circumference.
- `quat.py` (scalar-first `[w, x, y, z]`, vectorised over leading axes): `mul(a, b)`, `conj(q)`, `normalize(q)`, `rotate(q, v)` (body → world), `from_axis_angle(axis, angle)`, `from_two_vectors(a, b)`, `to_matrix(q)`, `angle_between(a, b) -> rad`, `integrate(q, omega_body, dt)` (exact exponential map step).

- [ ] **Step 1: Failing tests**
```python
def test_bell_matches_hand_calculation():
    b = BellProfile()
    assert ball_radius_m(b) == pytest.approx(0.10027, abs=1e-4)
    assert side_flat_distance_m(b) == pytest.approx(0.0869, abs=2e-4)
    assert com_offset_m(b) == pytest.approx(0.0237, abs=1.5e-3)
    assert model_mass_kg(b) == pytest.approx(32.6, abs=0.4)
    assert lever_arm_bell_m(b) == pytest.approx([0.0969, 0.0, -0.0237], abs=2e-3)

def test_bell_dict_roundtrip(): b = BellProfile(device_thickness_cm=1.5); assert BellProfile.from_dict(b.to_dict()) == b

def test_quat_rotate_and_integrate():
    q = from_axis_angle([0, 0, 1], np.pi / 2)
    assert rotate(q, [1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)
    q2 = np.array([1.0, 0, 0, 0])
    for _ in range(400): q2 = integrate(q2, np.array([0, 0, np.pi / 2]), 1 / 400)
    assert angle_between(rotate(q2, [1, 0, 0]), [0, 1, 0]) < 1e-9
    assert angle_between(rotate(from_two_vectors([0, 0, 1], [1, 0, 0]), [0, 0, 1]), [1, 0, 0]) < 1e-12
```
- [ ] **Step 2: Run** `pytest tests/test_bell.py tests/test_quat.py -v` → FAIL (import).
- [ ] **Step 3: Implement** per Interfaces.
- [ ] **Step 4: Run** → PASS.
- [ ] **Step 5: Commit** `feat(py): bell geometry model and quaternion utils`.

---

### Task 2: IMU scaling (Run → ImuData)

**Files:** Create `powerkettle/imu.py`. Test `tests/test_imu.py`.

**Interfaces:**
- Consumes: `rawlog.Run`.
- `@dataclass class ImuData: t: np.ndarray (N,) s; acc: (N,3) m/s²; gyro: (N,3) rad/s; valid: (N,) bool; saturated: (N,) bool; fs: float; first_index: int`
- `scale_run(run: Run) -> ImuData` — uniform grid from first to last `sample_index`; missing samples are NaN with `valid=False`, and `valid=False` also for 2 samples on each side of a gap; `t = (index − first)/rate_hz`; acc scale `range_g·g/32768`, gyro scale `range_dps/32768·π/180`; `saturated` where any acc axis is −32768 or 32767.

- [ ] **Step 1: Failing tests**: counts `[2048,0,0,0,0,0]` at 16 g → acc x = 9.80665; gyro count 16384 at 2000 dps → 1000°/s in rad/s; a run with samples at indices 0..18 and 50..68 (gap `(18, 50)`) → `len(t) == 69`, rows 19..49 NaN, `valid[17:52]` all False and `valid[:17]`, `valid[52:]` all True; rate 402 Hz → `t[1] == 1/402`; a −32768 sample → `saturated` True.
- [ ] **Step 2–4:** run → FAIL, implement, run → PASS.
- [ ] **Step 5: Commit** `feat(py): scale raw runs to SI ImuData`.

---

### Task 3: Rigid-body simulator

**Files:** Create `powerkettle/simulate.py`. Test `tests/test_simulate.py`.

**Interfaces:**
- Consumes: Task 1, `rawlog.Run`, `protocol.Status`.
- Moves (dataclasses): `Rest(seconds)`, `Swing(n, apex_m=1.0, cadence_s=1.2)`, `Snatch(n, hand="R", apex_m=1.6, lockout_s=0.4)`, `Hold(seconds)` (overhead, from a snatch lockout), `TableLift(height_m)`, `PartialRep(apex_m=0.4)`, `LieOnSide(seconds)` (bell rests tipped 90° onto its side).
- `@dataclass class SimConfig: fs=400.0; clock_error=0.005; acc_noise=0.02; gyro_noise=0.005; gyro_bias=(0.01,-0.02,0.015); acc_bias=(0.05,-0.03,0.04); mount_angle_deg=30.0; heading_deg=37.0; tremor_amp=0.1; tremor_hz=8.0; float_residual_speed=0.0; right_hand_twist_sign=+1; seed=0`
- `@dataclass class TruthRep: i_low; i_top; i_end; kind; hand; power_w; peak_velocity; apex_m`
- `@dataclass class Truth: t; pos; vel; acc; quat; omega_w; power  # per sample; reps: list[TruthRep]; sets: list[tuple[int,int]]; anchors: list[tuple[int,str]]`
- `simulate(moves: list, bell=BellProfile(), cfg=SimConfig()) -> tuple[Run, Truth]` — the Run carries synthetic 1 Hz statuses (true `time_us`) so `estimate_rate` recovers `fs·(1+clock_error)`. A session always starts with the given moves; callers put `Rest(3)` first.

Kinematics: piecewise **quintic** segments between keyframes (position, velocity, acceleration; tilt θ and twist ψ with zero rate and acceleration at keyframes). Keyframes in the athlete frame (x forward, z up; θ = rotation of bell-up about +y, so θ > 0 leans the handle forward; ψ = twist about world z):

| Keyframe | pos (m) | vel (m/s) | acc (m/s²) | θ | ψ |
|---|---|---|---|---|---|
| floor rest | (0.45, 0, 0.104) | 0 | 0 | 0° | 0 |
| low (backswing) | (−0.30, 0, 0.40) | 0 | (15, 0, 5) | +40° | 0 |
| swing top | (0.60, 0, 0.40 + apex) | residual · (−0.6, 0, −0.8) | (0, 0, −8.8) | −90° | 0 |
| snatch mid-pull (up) | (0.20, 0, 0.40 + 0.6·apex) | (0.3, 0, 2.5) | (0, 0, −3) | −30° | ±90° |
| lockout | (0.05, 0, 0.40 + apex) | 0 | 0 | +20° | ±180° (sign = hand × right_hand_twist_sign) |
| snatch mid-drop | (0.20, 0, 0.40 + 0.6·apex) | (−0.3, 0, −3.0) | (0, 0, −3) | −30° | ±90° |

Timings: floor→low 0.8 s; swing low→top 0.55 s, top→low 0.65 s; snatch low→mid 0.40, mid→lockout 0.35, lockout hold, lockout→mid 0.35, mid→low 0.45; last low→floor 1.0 s; TableLift 2 s up, 2 s still, 2 s down. Partial rep: swing timing, apex as given, top θ −40°.
Sensor rendering on a 4× oversampled grid, then decimated: R = R_heading · R_z(ψ) · R_y(θ) (bell → world); ω from consecutive rotations; r_w = R·lever_arm; a_s = a + α×r_w + ω×(ω×r_w); f_bell = Rᵀ(a_s − g_w); sensor frame = bell frame with sensor z along bell x, rotated in-plane by `mount_angle_deg`; add bias, white noise, tremor (sinusoid on all axes, amplitude `tremor_amp`) during hand-held static phases only; quantise to counts (16 g / 2000 dps) with clipping.
Truth power = m·(a + g ẑ)·v; truth rep power = mean concentric power over [i_low, i_top].

- [ ] **Step 1: Failing tests** (noise off via `SimConfig(acc_noise=0, gyro_noise=0, gyro_bias=(0,0,0), acc_bias=(0,0,0), tremor_amp=0)` where stated):
  - `test_static_reads_gravity`: during `Rest(3)` the scaled accel norm = 9.80665 ± 0.01 and gyro ≈ 0.
  - `test_energy_conservation`: ∫ truth power dt over the whole run = Δ(½mv² + mgz) ± 1 %.
  - `test_sensor_matches_differentiated_position`: second difference of (pos + R·lever_arm) equals rendered a_s within 0.05 m/s² RMS (noise off).
  - `test_rate_recoverable`: `estimate_rate(run.statuses, 400) == pytest.approx(402.0, rel=1e-4)`.
  - `test_truth_reps`: `[Rest(3), Snatch(5, "L"), Rest(5), Swing(10), Rest(3)]` → 15 truth reps, kinds/hands as scripted, 2 sets; snatch apex truth 1.6 ± 0.01.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): rigid-body kettlebell simulator with ground truth`.

---

### Task 4: Session calibration

**Files:** Create `powerkettle/calibration.py`. Test `tests/test_calibration.py`.

**Interfaces:**
- `class CalibrationError(ValueError)`
- `@dataclass class CalibrationConfig: still_seconds=2.0; gyro_still_max=0.05 (rad/s); acc_still_tol=0.03 (fraction of g); g_tolerance=0.02; normal_axis=2; normal_sign=+1`
- `@dataclass class Calibration: gyro_bias: (3,); up_s: (3,) unit; out_s: (3,) unit; lever_arm_s: (3,) m; q0: (4,) world←sensor (heading: forward = −horizontal(out)); still: slice; g_measured: float; warnings: list[str]`
- `find_still_window(imu, cfg) -> slice` — first window of ≥ `still_seconds` where |ω| < gyro_still_max and ||f| − g| < acc_still_tol·g; raises `CalibrationError("no still window of 2.0 s found …")`.
- `calibrate(imu, bell, cfg=CalibrationConfig(), window: slice | None = None) -> Calibration` — warning `"accelerometer magnitude off by X %"` when outside `g_tolerance`.

- [ ] **Step 1: Failing tests**: simulated `Rest(3)+Swing(3)+Rest(3)` (default noise/bias) → `angle_between(up_s, truth up in sensor) < 1°`, `|gyro_bias − truth| < 0.003`, `lever_arm_s` rotated to bell frame = `lever_arm_bell_m` ± 3 mm; recording that starts mid-swing (`[Swing(3)]` only) → `CalibrationError`; scaling acc by 1.05 → one warning.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): still-window calibration with lever arm`.

---

### Task 5: Orientation

**Files:** Create `powerkettle/orientation.py`. Test `tests/test_orientation.py`.

**Interfaces:**
- `static_mask(imu, gyro_max=0.3, acc_tol=0.05) -> np.ndarray` bool (|ω| < gyro_max and ||f| − g| < acc_tol·g, held ≥ 0.1 s).
- `estimate_orientation(imu, calib, gain=2.0) -> np.ndarray` (N,4) world←sensor — exact gyro integration of bias-corrected ω from `calib.q0`; gravity correction (rotation towards measured f direction, rate `gain` rad/s) only where `static_mask`; invalid samples hold the last orientation.

- [ ] **Step 1: Failing tests**: simulated `[Rest(3), Snatch(10,"R"), Rest(10), Swing(10), Rest(3)]` with default noise → tilt error (angle between estimated and true world-z in sensor frame) < 2° at all samples; with gyro bias error injected (+0.005 rad/s after calibration) still < 3°.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): gyro orientation with static gravity correction`.

---

### Task 6: Centre-of-mass acceleration

**Files:** Create `powerkettle/kinematics.py`. Test `tests/test_kinematics.py`.

**Interfaces:**
- `savgol_derivative(x: np.ndarray, dt: float, window: int = 11, order: int = 3) -> np.ndarray` (numpy least-squares Savitzky–Golay, axis 0, edge rows by shrinking the window).
- `@dataclass class Kinematics: a_com: (N,3) world; omega_w: (N,3); alpha_w: (N,3); f_norm: (N,) specific-force magnitude`
- `com_acceleration(imu, quats, calib) -> Kinematics` — a_s = R·f + g_w; r_w = R·lever_arm_s; a_com = a_s − α×r_w − ω×(ω×r_w).

- [ ] **Step 1: Failing tests**: `savgol_derivative(sin(2πt))` matches `2π cos` within 1e-3 (interior); simulated snatch session → a_com RMS error vs truth < 0.3 m/s² over valid samples; **with lever arm zeroed** the error during flips exceeds 2 m/s² (proves the correction matters).
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): lever-arm-corrected COM acceleration`.

---

### Task 7: Anchors, rests and tremor

**Files:** Create `powerkettle/anchors.py`. Test `tests/test_anchors.py`.

**Interfaces:**
- `@dataclass class AnchorConfig: prominence_deg=30.0; low_min_f_g=1.3; static_min_s=0.15; rest_min_s=2.0; tremor_band=(4.0, 12.0); tremor_rms_max=0.02 (m/s², floor threshold); floor_max_height_m=0.8`
- `@dataclass class Anchor: i0: int; i1: int; kind: Literal["low","top_swing","top_lockout","rest"]` (i0 == i1 for point anchors; windows for lockout/rest).
- `tremor_rms(acc: np.ndarray, fs: float, band) -> float` (FFT band RMS of detrended |f|).
- `sagittal_tilt(quats, calib, forward_w: np.ndarray) -> np.ndarray` rad (θ convention as simulator).
- `detect_anchors(imu, quats, kin, calib, cfg=AnchorConfig()) -> list[Anchor]`, sorted, algorithm:
  1. Static windows ≥ `static_min_s` from `static_mask`. A window is **rest** if tremor ≤ `tremor_rms_max` and length ≥ `rest_min_s`; otherwise a hand-held static window (candidate lockout). (The height check is added in Task 8 by `refine_rests`.)
  2. Forward direction per set = −horizontal(R·out_s) averaged over the preceding rest.
  3. **Low anchors** = local maxima of θ with prominence ≥ `prominence_deg`, outside static windows, with f_norm ≥ `low_min_f_g`·g.
  4. **Top anchor** between consecutive lows = the hand-held static window inside that interval if any (`top_lockout`), else argmin θ (`top_swing`).

- [ ] **Step 1: Failing tests** (simulated, default noise):
  - every truth anchor matched within 12 samples (lockout/rest windows: overlap ≥ 80 %), and no unmatched detections, on `[Rest(3), Snatch(10,"L"), Rest(8), Swing(15), Rest(8), Snatch(10,"R"), Rest(3)]`;
  - `Hold(5)` after a snatch is a lockout, not a rest (tremor present);
  - `LieOnSide(10)` between sets is a rest;
  - the snatch mid-pull θ minimum never becomes an anchor;
  - `tremor_rms` of the simulated floor rest < 0.01, of the hold > 0.05.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): turnaround, lockout and rest detection`.

---

### Task 8: Velocity, position and power

**Files:** Create `powerkettle/integrate.py`. Test `tests/test_integrate.py`.

**Interfaces:**
- `segment_velocity(a_com, t, anchors) -> np.ndarray` (N,3): trapezoidal integration between consecutive anchors with linear detrend so v = 0 at both ends; v = 0 inside rest and lockout windows; NaN across invalid samples (the containing segment becomes NaN).
- `segment_position(v, t, anchors) -> np.ndarray` (N,3): integral of v restarting at 0 at every low anchor (and at rest windows).
- `power(a_com, v, mass_kg) -> np.ndarray` (N,) = m·(a_com + g ẑ)·v.
- In `anchors.py`: `refine_rests(anchors, pos, cfg) -> list[Anchor]` — a rest window whose height above the preceding low anchor (same set) is ≥ `floor_max_height_m` becomes `top_lockout`; the pipeline then re-runs `segment_velocity`/`segment_position` once.

- [ ] **Step 1: Failing tests** (pipeline stages 2–7 on simulated sessions):
  - velocity RMS error < 0.08 m/s over reps; per-rep mean concentric power within 5 % of truth; peak velocity within 5 %; apex (max z of position from low) within 5 cm;
  - characterisation: `SimConfig(float_residual_speed=0.15)` swings → rep power error < 10 % (record the measured error in the test's docstring);
  - `TableLift(0.74)` → max position z gain 0.74 ± 0.03 m.
  - `refine_rests`: a 3 s `Hold` simulated with `tremor_amp=0` (initially classified rest) becomes `top_lockout`; a `LieOnSide(10)` rest stays rest.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): drift-free segment integration and power`.

---

### Task 9: Reps, sets and classification

**Files:** Create `powerkettle/reps.py`, `powerkettle/classify.py`. Test `tests/test_reps.py`, `tests/test_classify.py`.

**Interfaces:**
- `@dataclass class Rep: i_low: int; i_top: int; i_top_end: int; i_end: int; kind: str; hand: str; confidence: float; features: dict; flags: set[str]; power_w: float; peak_power_w: float; peak_velocity_ms: float; work_con_j: float; work_ecc_j: float; apex_m: float; con_duration_s: float; cycle_s: float; path: np.ndarray | None`
  (`kind` ∈ {snatch, swing, other}; `hand` ∈ {L, R, ?, -} — "-" for swings/other; `i_top_end` = end of lockout window, = i_top for swings.)
- `@dataclass class SetResult: reps: list[Rep]; i_start: int; i_end: int; t_start: float; t_end: float; rest_before_s: float`
- `segment_reps(anchors, n_samples) -> list[tuple[int,int,int,int]]` (low, top, top_end, next low) and `segment_sets(anchors) -> list[tuple[int,int]]` (between rests).
- `rep_metrics(seg, t, v, p, pos, P, valid, saturated) -> dict` filling the numeric Rep fields: peak power from P smoothed with a 25 ms moving average; `flags` gets `"gap"` if any invalid sample, `"saturated"` if any saturated sample.
- `@dataclass class ClassifierConfig: snatch_min_apex_m=1.25; snatch_max_top_tilt_deg=60; snatch_min_static_s=0.1; swing_min_apex_m=0.5; swing_max_apex_m=1.3; swing_top_tilt_deg=(60,130); hand_min_twist_deg=90; right_hand_twist_sign=+1; low_conf=0.2`
- `rep_features(seg, quats, calib, pos, t) -> dict` with `apex_m`, `top_tilt_deg` (unsigned angle of bell-up from vertical at top), `twist_deg` (signed rotation about world z, low → top), `top_static_s`.
- `classify(features, cfg) -> tuple[str, str, float]` (kind, hand, confidence = smallest normalised margin to the rule thresholds, clipped to [0, 1]); `"low_conf"` flag when < `low_conf`.

- [ ] **Step 1: Failing tests**:
  - Simulated `[Rest(3), Snatch(8,"L"), Rest(6), Swing(12), Rest(6), Snatch(8,"R"), Rest(6), PartialRep(), Rest(3)]` → kinds exactly `8×snatch, 12×swing, 8×snatch, 1×other`; snatch hands L then R; swing hands "-"; 4 sets.
  - `classify` unit cases: `{apex 1.6, tilt 25, twist 175, static 0.4}` → (snatch, R); same with twist −175 → L; `{1.0, 92, 5, 0}` → (swing, -); `{0.4, 40, 0, 0}` → other; twist 40 on a snatch → hand "?".
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): rep/set segmentation and snatch/swing classifier`.

---

### Task 10: Metrics

**Files:** Create `powerkettle/metrics.py`. Test `tests/test_metrics.py`.

**Interfaces:**
- `@dataclass class PowerStats: n: int; avg: float | None; best: float | None; top3: float | None; dropoff: float | None; cv: float | None`
- `power_stats(powers: Sequence[float]) -> PowerStats` (top3 = mean of the min(3, n) best; drop-off uses the last min(3, n) reps; all None when n = 0; cv None when n < 2).
- `best_efforts(powers, ns=(1,3,5,10,20,50)) -> dict[int, float]` (max rolling mean of consecutive reps; keys with n > len omitted).
- `@dataclass class SetMetrics: by_kind: dict[str, PowerStats]; reps_by_kind_hand: dict[str, int]  # e.g. "snatch:L"; qualifying: int; pace_rpm: float | None; work_j: float`
- `set_metrics(s: SetResult, power_floor_w: float | None = None) -> SetMetrics` (reps flagged `"gap"` excluded from power stats; `"other"` never in snatch/swing stats).
- `@dataclass class SessionMetrics: reps_by_kind_hand; work_j; by_kind: dict[str, PowerStats]; best_efforts: dict[str, dict[int, float]]; density_w: float | None; active_s: float; rest_s: float; work_rest_ratio: float | None`
- `session_metrics(sets: list[SetResult], duration_s: float, power_floor_w=None) -> SessionMetrics` — active time = Σ set durations; density = work / duration.

- [ ] **Step 1: Failing tests**
```python
def test_power_stats_reference():
    s = power_stats([300, 320, 310, 250, 240, 230])
    assert (s.avg, s.best) == (275, 320)
    assert s.top3 == pytest.approx(310)
    assert s.dropoff == pytest.approx(0.25)
    assert s.cv == pytest.approx(0.13068, abs=1e-4)

def test_best_efforts_reference():
    assert best_efforts([300, 320, 310, 250, 240, 230]) == pytest.approx({1: 320, 3: 310, 5: 284})

def test_small_sessions_do_not_raise():
    assert power_stats([]).avg is None
    one = power_stats([200]); assert (one.top3, one.dropoff, one.cv) == (200, 0.0, None)

# make_sets(rest_s): two SetResults, each 5 snatch reps of power 300 W and work_con 400 J,
# each set lasting 10 s, separated by rest_s; returns (sets, duration_s = 20 + rest_s)
def test_rest_does_not_change_power_but_changes_density():
    a = session_metrics(*make_sets(rest_s=30)); b = session_metrics(*make_sets(rest_s=180))
    assert a.by_kind["snatch"].avg == b.by_kind["snatch"].avg == 300
    assert (a.density_w, b.density_w) == pytest.approx((80.0, 20.0))

def test_power_floor_counts_qualifying():
    s = set_of_snatches([300, 320, 310, 250, 240, 230])   # helper: one SetResult
    assert set_metrics(s, power_floor_w=260).qualifying == 3
```
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): rep, set and session metrics`.

---

### Task 11: Rep paths in the athlete frame

**Files:** Create `powerkettle/paths.py`. Test `tests/test_paths.py`.

**Interfaces:**
- `athlete_forward(v: np.ndarray, set_slice: slice, hint_w: np.ndarray) -> np.ndarray` — first principal axis of horizontal velocity in the set, sign chosen to align with `hint_w` (the rest-based forward from Task 7).
- `rep_path(pos, seg, forward_w, decimate_to_hz=50, fs=400) -> np.ndarray` (k,3) in athlete frame (x forward, y left, z up), origin at the low anchor.

- [ ] **Step 1: Failing tests**: simulated with `heading_deg=37` → recovered forward within 3° of truth; snatch path apex z 1.6 ± 0.05, lockout x within 5 cm of truth (+0.35 m relative to the low anchor); swing path max x ≈ 0.90 ± 0.05.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): per-rep paths in athlete frame`.

---

### Task 12: Pipeline, session export, CLI, Pyodide import guard

**Files:** Create `powerkettle/pipeline.py`, `powerkettle/session_io.py`; Modify `powerkettle/__main__.py` (add `analyse`). Test `tests/test_pipeline.py`, `tests/test_session_io.py`, `tests/test_imports.py`.

**Interfaces:**
- `ALGORITHM_VERSION = "1"`
- `@dataclass class AnalysisConfig: calibration=CalibrationConfig(); anchors=AnchorConfig(); classifier=ClassifierConfig(); power_floor_w: float | None = None`
- `@dataclass class SessionResult: calib: Calibration; sets: list[SetResult]; metrics: SessionMetrics; duration_s: float; warnings: list[str]; algorithm_version: str` with `summary() -> dict` (JSON-safe: metrics + per-rep fields without arrays).
- `analyse_run(run: Run, bell: BellProfile = BellProfile(), cfg: AnalysisConfig = AnalysisConfig(), labels: dict | None = None) -> SessionResult` — stages Task 2 → 11; `labels` maps rep index → `{"kind":…, "hand":…}` and overrides classifier output (flag `"relabelled"`).
- `session_io.save_session(path_or_buffer, run, bell, meta: dict) -> None` and `load_session(path_or_buffer) -> tuple[Run, BellProfile, dict]` — `.npz` (`np.savez_compressed`) with keys `sample_index`, `raw`, `gaps` (k,2), `rate_hz`, `ranges` [acc_g, gyro_dps], `meta_json` (labels, markers, app/algorithm version, bell, summary).
- CLI: `python -m powerkettle analyse FILE(.pkraw|.npz) [--bell bell.json] [--out out.npz]` prints one line per set and a rep table (index, time, kind, hand, power W, peak W, peak v, apex, flags) and, for `.pkraw`, the nearest preceding marker text per set.
- `tests/test_imports.py`: every `import`/`from` in `powerkettle/*.py` resolves to `sys.stdlib_module_names`, `numpy`, or `powerkettle`.

- [ ] **Step 1: Failing tests**:
  - end-to-end simulated session → rep count and kinds match truth; session snatch avg rep power within 5 % of truth;
  - a 40-sample gap injected inside one rep → that rep flagged `"gap"`, excluded from `metrics.by_kind`, other reps unaffected;
  - a rep with a forced saturated sample → flagged `"saturated"` and still counted;
  - labels `{3: {"kind": "other"}}` → rep 3 kind "other" with `"relabelled"`;
  - save → load roundtrip reproduces `raw`, `sample_index`, bell and labels exactly; re-analysis of the loaded run gives an identical `summary()`;
  - the import guard passes.
- [ ] **Step 2–4:** FAIL → implement → PASS (`pytest -q` whole suite).
- [ ] **Step 5: Commit** `feat(py): end-to-end pipeline, npz sessions, analyse CLI`.

---

### Task 13: Incremental live analyser

**Files:** Create `powerkettle/live.py`. Test `tests/test_live.py`.

**Interfaces:**
- `class LiveAnalyser: __init__(self, bell, cfg, rate_hz, acc_range_g, gyro_range_dps)`; `feed(self, first_index: int, raw: np.ndarray) -> list[dict]`; `finish(self) -> SessionResult`.
- Events (JSON-safe dicts): `{"type":"calibration","ok":bool,"warnings":[…]}`, `{"type":"set_start","set":k,"t":…}`, `{"type":"rep","set":k,"rep":i,"kind","hand","power_w","peak_power_w","peak_velocity_ms","apex_m","path":[[x,z],…] (side view)}`, `{"type":"set_end","set":k,"metrics":{…}}`, `{"type":"warning","message":…}`.
- Approach: keep all samples; whenever ≥ 0.25 s of new data has arrived, re-run stages 4–9 on the window from the last finalised rest/low anchor minus 2 s to now; emit a `rep` once its top anchor is confirmed (lockout window ≥ 0.15 s, or θ risen ≥ `prominence_deg` above the swing minimum); never re-emit a rep already emitted.

- [ ] **Step 1: Failing tests**: simulated session fed in 19-sample chunks → same number of `rep` events as batch reps, powers within 1 % of batch, kinds identical, events in order `calibration, set_start, rep…, set_end, …`; each rep event emitted ≤ 0.6 s (simulated time) after its truth top; total processing time for a 10-min session < 30 s on CI.
- [ ] **Step 2–4:** FAIL → implement → PASS.
- [ ] **Step 5: Commit** `feat(py): incremental live analyser`.

---

### Task 14: Validation notebook

**Files:** Create `notebooks/validate_recording.ipynb`, `notebooks/README.md`.

Content: Colab-ready first cell (`!pip install -q git+https://github.com/matias-ceau/power-kettle@main`) guarded by `"google.colab" in sys.modules`; optional `drive.mount("/content/drive")`; path input; then `read_rawlog` → `to_recording` → `analyse_run` per run; plots (matplotlib): |ω| and θ with anchors and markers, velocity norm, power with concentric phases shaded, rep table, side-view rep paths per set, marker text vs classifier label per set (agreement %); a "table lift" section that prints measured vs marker height.

- [ ] **Step 1:** Write the notebook; run it top-to-bottom locally on a simulated `.pkraw` (write one with `write_rawlog` from `simulate` output in the first cell when no path is given) — Expected: executes without errors (`jupyter nbconvert --to notebook --execute`).
- [ ] **Step 2: Commit and push** `docs: validation notebook`; `git push -u origin ccr-cc05eceb-786d04`.
