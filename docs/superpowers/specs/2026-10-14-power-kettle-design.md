# Power Kettle — Design Spec

Date: 2026-10-14
Status: Draft, awaiting review

## 1. Purpose

A small sensor stuck on a kettlebell that measures **power** — the way a cycling
power meter does for a bike — plus velocity, work, rep counts and rep paths, with
a phone companion app for live feedback and history.

### Goals

- Measure per-rep power (headline metric), velocity, work and 3D rep path for a
  single 32 kg cast-iron kettlebell.
- Automatically count reps and classify **snatch** (main use) vs **swing**, and
  the hand used for snatches.
- Power metrics that are **not diluted by rest** between sets, and that keep
  **peak reps visible** within a set.
- Workouts driven by rep targets, optionally counting only reps above a power
  floor, with audio feedback.
- All data stored on the phone; no computer needed for normal use. Optional deeper
  analysis on any computer via Google Colab.

### Non-goals (v1)

- Other movements (cleans, presses, squats, get-ups).
- Multiple kettlebells / bell switching (the bell profile is editable, but only one
  is active).
- Absolute position in the room (only per-rep relative paths).
- Calorie estimation, training-load score, automatic Drive upload, auto sleep /
  wake-on-motion (deferred to "Later", §11).

## 2. System overview

```
┌──────────── device on bell ────────────┐        ┌──────────── Android phone (Chrome PWA) ─────────────┐
│ BMI160 ──I²C──► ESP32-C3 SuperMini ────┼─BLE──► │ JS: BLE, storage, UI, audio                         │
│ (400 Hz FIFO)    (raw data pipe)       │        │ Web Worker: Pyodide + `powerkettle` Python package  │
└────────────────────────────────────────┘        │ IndexedDB: raw data + metrics + labels              │
                                                  └──────────────┬──────────────────────────────────────┘
                                                                 │ share sheet export (.npz)
                                                                 ▼
                                                  Google Drive /PowerKettle ──► Colab notebooks
                                                                               (same `powerkettle` package)
```

Principle: **firmware is a dumb, reliable data pipe; all analysis is one Python
package** (`powerkettle`) that runs both in the browser (Pyodide) and in normal
CPython (Colab / local / tests). Raw data is always kept, so every past session
can be re-analysed when the algorithms improve.

## 3. Hardware

### 3.1 Parts

| Part | Notes |
|---|---|
| ESP32-C3 SuperMini | Owned. BLE 5, USB-C (flashing). Onboard 3.3 V LDO. |
| BMI160 breakout (purple GY-BMI160) | Owned. ±16 g accel, ±2000 °/s gyro, 1 KB FIFO. Chosen over the MPU-9250 board: smaller, current part (MPU-9250 discontinued, boards often relabelled MPU-6500 clones), and a magnetometer is useless on an iron bell. |
| 1S LiPo ~300 mAh (e.g. 402535) | ~5 h streaming. |
| TP4056 USB-C charger **with protection** (DW01A + 8205A, 6-pad) | Charging + over-discharge protection. Separate USB-C port used only for charging. **Charge-current fix (mandatory):** modules ship with R_PROG = 1.2 kΩ (I ≈ 1200/R = 1000 mA ≈ 3.3C for a 300 mAh cell); replace it (next to TP4056 pin 2, marked "122") with **4.7 kΩ** (≈ 255 mA, below 1C). Charge with the slide switch OFF so charge termination works. |
| 4.7 kΩ resistor (SMD) | Replaces R_PROG on the TP4056 (see above). |
| Schottky diode (SS14 or 1N5817) | In series between the slide switch and the C3 5V pad (anode switch side, cathode/band C3 side). The SuperMini 5V pad is USB VBUS and on a protected TP4056 OUT+ is the B+ net, so without the diode a C3 USB cable with the switch ON would force 5 V into the LiPo, bypassing the charger. |
| Slide switch | Battery on/off. |
| 2 × 100 kΩ | Battery voltage divider. |
| Pot magnet 20–25 mm, countersunk | Shear ≈ 30 N vs ≈ 7 N for a ~30 g device at 20 g impact. |
| Small ABS project box (~50 × 35 × 15 mm), epoxy | No 3D printer available. |

No buck/boost converter is needed: LiPo → switch → diode → SuperMini 5V pin →
onboard LDO. The diode costs ~0.3 V, so the 3.3 V rail starts to sag below
~3.7 V battery but stays ≥ 3.0 V down to ~3.4 V (~90 % of capacity); the
3500 mV low-battery threshold remains appropriate.

Rule: slide switch OFF before plugging anything into the C3's USB-C; charge ONLY
through the TP4056's USB-C port.

### 3.2 Wiring

| From | To |
|---|---|
| LiPo | TP4056 B+ / B− |
| TP4056 OUT+ | slide switch → Schottky diode (anode switch side, band C3 side) → C3 **5V** |
| TP4056 OUT− | C3 **GND** (this is circuit ground; B− is the unprotected battery side and is not ground) |
| C3 **3V3** | BMI160 **3V3** (bypass its LDO) |
| C3 GND | BMI160 GND |
| C3 **GPIO6** | BMI160 **SDA** |
| C3 **GPIO7** | BMI160 **SCL** (I²C 400 kHz) |
| C3 **GPIO3** | BMI160 **INT1** (FIFO watermark; deep-sleep-wake capable for later) |
| TP4056 OUT+ (before the switch; same net as B+ on protected boards) → 100 kΩ → C3 **GPIO4** → 100 kΩ → GND | Battery sense (ADC1) |
| BMI160 CS | high (I²C mode) |
| BMI160 SAO/SDO | low (address 0x68) |

GPIO2, GPIO8, GPIO9 are avoided (boot strapping pins; GPIO8 is the onboard LED,
which is used for status only). Exact BMI160 board pin labels are verified
against the board silkscreen in the build guide.

### 3.3 Mounting

- The device is centred on the **front flat face** (the circular "32KG" plate,
  Ø 10 cm) on the side that faces the lifter at the hike. This face is opposite
  the forearm contact for either hand and away from the handle window. Before
  gluing, the lifter tests clearance with a dummy box taped in place.
- Pot magnet epoxied to the box base; a thin rubber pad or steel washer if the
  raised lettering prevents flat contact.
- The BMI160 board is **rigidly** fixed (epoxy/screw, no foam) and **parallel to
  the box base**, so one sensor axis is the outward normal of the flat face.
- In-plane rotation of the box on the face does not need to be precise; it is
  calibrated at the start of every session (§5.2).

### 3.4 Bell profile (measured)

| Quantity | Value |
|---|---|
| Mass | 32 kg |
| Ball circumference | 63 cm → R = 10.03 cm |
| Bottom flat diameter | 12 cm → flat at 8.03 cm below ball centre |
| Front/back flat diameter | 10 cm → flat at 8.69 cm from ball centre |
| Clearance top of ball → handle | 9 cm |
| Handle tube circumference / length | 13 cm / 44 cm |
| Total height | ~30 cm |

Solid cast-iron model (7.1 g/cm³) from these numbers gives 32.6 kg, confirming the
geometry. Handle ≈ 13 % of mass with centroid ≈ 16.6 cm above ball centre →
**centre of mass (COM) ≈ 2.4 cm above ball centre**.

Lever arm (COM → sensor), with device thickness t (sensor height above the flat,
default 1.0 cm):

- outward component: 8.69 + t ≈ **9.7 cm**
- bell-up component: **−2.4 cm**
- |r| ≈ 10 cm → ~1 m/s apparent velocity at 10 rad/s if uncorrected.

The app's bell profile stores these measurements and computes COM and r with this
model (`powerkettle.bell`).

## 4. Firmware

- C++ / Arduino framework, PlatformIO, NimBLE-Arduino. ~300 lines; rarely changes.
- Built by GitHub Actions; flashed once via a web flasher page (ESP Web Tools,
  desktop Chrome, no toolchain install).

### 4.1 Sensor configuration

- Accel ±16 g, gyro ±2000 °/s, both 400 Hz, normal filter mode.
- FIFO headerless mode, accel + gyro (12 B/frame), watermark ≈ 20 frames → INT1.
  FIFO holds ~200 ms, so C3/BLE latency never loses samples or adds timing jitter.

### 4.2 BLE GATT

One custom service, device name `PowerKettle`.

| Characteristic | Props | Content |
|---|---|---|
| `imu_stream` | notify | sample packets (below) |
| `control` | write | `start`, `stop`, `set_config`, `ping` |
| `status` | read, notify (1 Hz) | battery mV, firmware version, active ranges/ODR, sensor temperature, dropped-sample count, C3 µs timestamp + latest sample index |

`imu_stream` packet (little-endian):

```
u16 seq           packet counter (gap detection)
u32 first_index   index of first sample since `start`
u8  n             samples in packet (≤ 19)
u8  flags         bit0: ring-buffer overflow occurred
n × i16[6]        ax, ay, az, gx, gy, gz  (raw counts)
```

~21 packets/s, ~5 KB/s. Raw counts are sent; all scaling happens in Python.

### 4.3 Behaviour

- Advertises on power-up; streams only after `start`.
- ~4 s ring buffer (~19 KB RAM) absorbs BLE stalls; overflow is flagged and the
  gap is explicit via `first_index`.
- Status (1 Hz) pairs the C3 microsecond clock with the sample index so the app
  estimates the true sensor rate (nominal 400 Hz, up to ~±1 % off).
- LED: slow blink = advertising, solid = connected, fast blink = low battery.
- Power off via slide switch.

## 5. Signal processing (`powerkettle` package)

numpy only (no scipy) so it stays light in Pyodide. Identical code path live and
offline; live processing is incremental, finalising each segment when its closing
anchor is detected.

### 5.1 Scaling and timebase

Counts → m/s², rad/s. Sample timestamps from index and the estimated true rate.
Gaps (overflow / disconnect) are marked; segments spanning a gap are flagged and
excluded from metrics.

### 5.2 Session calibration

Bell standing upright on the floor, untouched, for 2 s:

- gyro bias = mean gyro;
- bell-up axis **u** (sensor frame) = normalised mean accel;
- outward axis **n** = sensor axis normal to the box base, orthogonalised against **u**;
- lever arm **r** (sensor frame) from the bell profile (§3.4) along **n** and **u**;
- check |g| = 9.81 ± 2 %, else warn.

### 5.3 Orientation

Quaternion integration of bias-corrected gyro at 400 Hz. Accelerometer tilt
correction only when quasi-static (|a| ≈ g and |ω| small). Full re-anchoring
at every floor rest and lockout.

### 5.4 COM acceleration

a_sensor (world) = R · f + g_world, then

a_com = a_sensor − α × r − ω × (ω × r)

with ω from the gyro and α its smoothed derivative (Savitzky–Golay implemented in
numpy), r rotated to world.

### 5.5 Anchors (zero-velocity points)

Velocity is integrated between anchors with linear drift removal so v = 0 at both
ends of every segment.

- **Low anchor**: the extreme backward tilt of the bell in each cycle (deepest
  backswing, where the bell reverses). Extremes must stand out by ≥ ~30° from
  neighbouring extremes (hysteresis), so mid-pull rotation wobbles are ignored.
- **High anchor (swing)**: extreme forward tilt (float).
- **High anchor (snatch)**: the static lockout window.
- **Rest** windows are anchors too.

Risk: the bell may not be exactly at zero velocity at a swing float. Validated
with recordings (§9.2).

### 5.6 Velocity, position, power

- v_com = ∫ a_com dt per segment, detrended to zero at both anchors.
- Position = ∫ v_com dt per segment, chained within a rep from the low anchor.
- Power P(t) = m · (a_com + g ẑ) · v_com — force applied by the lifter on the
  bell, including lifting against gravity. Positive = driving, negative =
  absorbing.
- Rotational kinetic energy is ignored (~6 J vs ~700 J per snatch, < 1 %).
- This is power delivered **to the bell**, not the lifter's total metabolic cost.

### 5.7 Athlete frame

Per set, the dominant horizontal direction of motion (principal axis of horizontal
velocity) defines "forward". Paths are shown in this frame: side view
(forward/up) and front view (lateral/up). Absolute heading drift (a few °/min) is
irrelevant to per-rep paths.

## 6. Reps, sets, classification

### 6.1 Segmentation

- A **rep** = low anchor → high anchor → next low anchor.
- **Concentric phase** = low → high.
- **Rest** = static (|ω| ≈ 0, |a| ≈ g, accel variance at sensor-noise level, no
  tremor) for ≥ 2 s, near the height of the preceding low anchor.
- **Lockout** is distinguished from floor rest by **physiological tremor** (4–12 Hz
  micro-oscillation present when hand-held, absent on the floor) and by height
  relative to the preceding low anchor (~1.5 m above it, vs ~0–0.3 m for a bell
  parked on the floor). Height is always measured from the last low anchor, never
  as absolute height over a whole set (which would drift). A long lockout hold
  never ends a set.
- A **set** starts when the bell leaves rest and ends at the next floor rest.

### 6.2 Snatch vs swing

The snatch, for reference: backswing (bell behind, base pointing back, palm facing
the lifter) → hip drive on a swing-like arc → near-vertical pull close to the body
→ punch-through, where the hand rotates ~180° around the forearm/vertical axis
(palm turns from facing the lifter to facing forward) → lockout with the arm
vertical, **handle up**, bell hanging upright behind the hand resting on the back
of the forearm → drop.

Rule-based classifier (v1) on per-rep features:

| Feature | Swing | Snatch |
|---|---|---|
| Apex height above low point | ~0.8–1.1 m | ~1.4–1.8 m |
| Bell tilt at top (base→handle axis vs vertical) | ~90° | ~0–45° |
| Twist about vertical during drive | small | ~150–180° |
| Static time at top | ~0 | ≥ 0.2 s (lockout) |

- Reps fitting neither profile → **other** (e.g. failed snatch caught at the
  shoulder, partial reps): shown, excluded from snatch/swing stats.
- **Hand (L/R)** for snatches from the twist direction; sign mapping learned from
  labelled reps. Shipped in v1 only if it proves reliable on recordings.
- Thresholds are configuration, tuned on the lifter's labelled recordings.
- Users can correct a rep's or set's label in the app; corrections are stored
  with the session and used for tuning.

## 7. Metrics

All power metrics derive from **rep power**, so rest cannot dilute them.

### 7.1 Rep

| Metric | Definition |
|---|---|
| **Rep power** (headline) | Mean concentric power = concentric work / concentric duration |
| Peak power | max of P(t) smoothed over 25 ms |
| Peak velocity | max ‖v_com‖ in the concentric phase |
| Concentric / eccentric work | ∫P dt, positive / negative parts |
| Apex height | height gained low → top |
| Timing | concentric duration, cycle time |
| Path | 3D COM path from the low anchor (athlete frame) |
| Flags | data gap, low classification confidence |

### 7.2 Set

| Metric | Definition |
|---|---|
| Reps | by type and hand; plus "qualifying reps" above the power floor |
| **Avg rep power** | mean of rep powers |
| Best rep / Top-3 | max rep power; mean of 3 best |
| Drop-off % | (best rep − mean of last 3) / best |
| Consistency | coefficient of variation of rep power |
| Pace | reps/min within the set |
| Work | Σ concentric work (kJ) |

### 7.3 Session

- Totals: reps by type/hand, work (kJ).
- Session rep power per type (snatch and swing never mixed).
- Rest-aware, deliberately separate: **density** = total work / total session
  time (W), work:rest ratio, active time.
- **Best-efforts curve**: best mean rep power over 1, 3, 5, 10, 20, 50
  consecutive reps, per type.

Deliberately excluded: Normalized Power (rep-based averaging already solves the
problem it addresses); training-load score (needs a personal reference test — Later).

## 8. Companion app

### 8.1 Platform

- Static Progressive Web App on **GitHub Pages** (HTTPS required for Web
  Bluetooth), installed to the Android home screen, fully offline after first load
  (service worker caches the app, Pyodide and numpy, ~15 MB).
- **JS** (thin): Web Bluetooth, IndexedDB, UI, charts (uPlot), audio (Web Audio +
  Web Speech API).
- **Python** in a Web Worker via Pyodide: the `powerkettle` package.
- Worker interface: raw packets in; events out — `calibration`, `rep`,
  `set_start`, `set_end`, `status`, `warning`.

### 8.2 Storage

- IndexedDB with `navigator.storage.persist()`.
- Per session: compressed raw samples, config/calibration, metrics, label
  corrections, workout settings, app + algorithm version.
- ~5–8 MB per 30-min session (~1 GB/year at 3 sessions/week).
- **Re-analyse all sessions** on the phone when the algorithm version changes.
- Export format: NumPy `.npz` (raw int16 samples, timebase, calibration, bell
  profile, JSON metadata incl. labels), written with numpy inside Pyodide.

### 8.3 Export and analysis on a computer

- **Export new sessions** → Android share sheet → Google Drive folder
  `PowerKettle` (one tap; doubles as backup). Reminder if last export > 14 days.
- Notebooks in the repo open in **Google Colab** on any computer: cell 1 installs
  `powerkettle` from GitHub, cell 2 `drive.mount()` (one-click Google sign-in, no
  OAuth client setup). Notebooks: session explorer, 3D rep paths (Plotly),
  trends, classifier tuning from labels.
- Local use also supported: `pip install` the package and point it at `.npz`
  files.

### 8.4 Screens

1. **Home** — device connection + battery, Start session (workout picker), recent
   sessions.
2. **Calibrate** — "Stand the bell upright, don't touch it" → 2 s → ✓.
3. **Live** — large: rep count (with target), last rep power. Smaller: set avg
   power, drop-off %, last rep side-view path (previous reps of the set faded),
   rest timer between sets.
4. **Session summary** — sets table; rep-power bar chart coloured by type with
   threshold line; overlaid rep paths; best-efforts curve; tap a rep to relabel.
5. **History** — trends per type: rep power, best-efforts curve over time, weekly
   work and reps.
6. **Settings** — bell profile (measurements → COM/r computed), audio, export,
   re-analyse.

### 8.5 Workouts and audio

- **Free** — record; optional beep per rep.
- **Rep target** — N reps of {snatch, swing} × {L, R, any}, optional **power
  floor** (X W or X % of today's best rep). Qualifying rep: high beep, counts.
  Below floor: low buzz, does not count. Voice milestones ("50", "set done").
- **Timed test** — e.g. 10-min snatch test: timer, total reps, hand switches,
  voice pace updates.
- **Drop-off alert** (any mode) — voice cue when the set's drop-off exceeds a
  limit (e.g. 20 %).

### 8.6 Replay mode

The app can play a recorded session as if it were live. Used for development,
automated browser tests (Playwright) and demos without the bell.

## 9. Testing and validation

### 9.1 Automated

- **Synthetic motion simulator** (`powerkettle.simulate`): rigid-body motion of
  the profiled bell — pendulum swings, snatch trajectories with twist and lockout,
  floor rests, overhead holds — rendered to BMI160 readings with gravity, lever
  arm, noise, gyro bias, clock-rate error. Tests assert recovered velocity, power,
  rep counts, labels and paths match ground truth within tolerances.
- **Physics sanity tests**: P ≈ 0 at rest; concentric work ≥ m·g·apex height;
  v ≈ 0 during holds; consistent lockout heights.
- **Regression fixtures**: short labelled real recordings.
- **Firmware**: 30-min streaming soak — dropped samples, battery drain.
- **App**: Playwright tests in replay mode with a mocked BLE device.
- CI (GitHub Actions): pytest, firmware build, app tests, deploy to Pages.

### 9.2 Real-world validation (with the lifter)

1. **Known-height lift**: slow lift floor → table of measured height; integrated
   displacement vs tape.
2. **Labelled sets**: snatches L and R, two-hand and one-hand swings, floor rests,
   long overhead hold. Used to validate anchors (especially the swing float) and
   tune the classifier.
3. Optional side-view slow-motion phone video for qualitative path comparison.

## 10. Repository layout

```
firmware/           PlatformIO project (C++)
powerkettle/        Python package: io, bell, calibration, orientation, kinematics,
                    anchors, reps, classify, metrics, paths, simulate
tests/              pytest (synthetic + fixtures)
app/                PWA (HTML/JS, service worker, Pyodide worker)
notebooks/          Colab notebooks
docs/               specs, plans, hardware build guide (wiring diagram)
.github/workflows/  CI + Pages deploy + firmware build
```

## 11. Milestones

1. **Data pipe** — build guide, firmware, web flasher, minimal "record raw" page
   in the app. Lifter builds the device and records validation sets.
2. **Analysis core** — simulator, pipeline, anchors, reps, classifier, metrics,
   paths, tests; tuned and validated on milestone-1 recordings.
3. **Live app** — calibration, live screen, audio, storage, session summary.
4. **Training features** — workouts, history/trends, export, re-analysis, Colab
   notebooks.
5. **Later** (not v1) — automatic Drive upload (`drive.file` scope, one-time
   OAuth client setup), wake-on-motion sleep, training-load score, calorie
   estimate, learned classifier, other movements.

## 12. Open risks

| Risk | Mitigation |
|---|---|
| Bell not at zero velocity at swing float → velocity/power error | Validate with recordings (§9.2); fall back to position-based constraints (equal low-point heights, smoother) in offline analysis |
| Anchor detection confused by mid-pull rotation | Hysteresis on tilt extremes; regression fixtures from real snatches |
| L/R detection unreliable | Ship only if validated; otherwise "any" |
| Magnet slips under impact | Shear margin ~4×; VHB tape as backup; check after first sessions |
| Pyodide too slow on phone for live processing | Segment-wise processing (~0.5 s latency budget); numpy-only; profile in milestone 3 |
| Browser storage wiped | Persistent storage request + export reminder |
