# Power Kettle

A small sensor that sticks to a 32 kg kettlebell and measures **power**, the
way a cycling power meter does for a bike, plus velocity, work, rep counts and
rep paths. A BMI160 motion sensor and an ESP32-C3 stream raw data over
Bluetooth to a phone; all analysis is one Python package, `powerkettle`, that
runs both in the phone's browser and on a normal computer. This repository is
currently at **milestone 1 (data pipe)**: the build guide, firmware, web
flasher and a "record raw data" page, plus a command line tool to check
recordings. The analysis comes in milestone 2.

## Links

- Design spec: [docs/superpowers/specs/2026-10-14-power-kettle-design.md](docs/superpowers/specs/2026-10-14-power-kettle-design.md)
- Plans: [milestone 1, data pipe](docs/superpowers/plans/2026-10-14-m1-data-pipe.md),
  [milestone 2, analysis core](docs/superpowers/plans/2026-10-14-m2-analysis-core.md)
- Hardware build guide: [docs/hardware/build-guide.md](docs/hardware/build-guide.md)
  (with the [wiring diagram](docs/hardware/wiring.svg))
- Validation recording protocol: [docs/hardware/recording-protocol.md](docs/hardware/recording-protocol.md)
- App (served by GitHub Pages): <https://matias-ceau.github.io/power-kettle/>
  - Flash the firmware (desktop Chrome): `.../flash.html`
  - Record a session (Chrome on Android): `.../record.html`

  The site is deployed from the `main` branch by GitHub Actions. In the
  repository settings, Pages, Source must be set to **GitHub Actions** (see the
  build guide).

## Safety rule for the device

**Slide switch OFF whenever a USB cable is in the C3. Charge and flash through
the C3's USB-C only** (the TP4056's own port stays unused). See the build guide
for why, and for the mandatory charge-current resistor change on the TP4056
board.

## Checking a recording

Recordings are `.pkraw` files saved by the record page. Install the analysis
package and summarise a file:

```bash
pip install git+https://github.com/matias-ceau/power-kettle
python -m powerkettle info FILE.pkraw
```

The output shows the duration, number of runs, samples, estimated sample rate,
gaps and dropped samples, the sensor noise in the quietest second (a bench
check), and the markers you tapped during recording.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/` | PlatformIO project (C++) for the ESP32-C3. Build and flash: `cd firmware && pio run -e c3 -t upload` |
| `powerkettle/` | Python package (stdlib + numpy only) |
| `tests/` | pytest tests for the Python package |
| `app/` | Static web app: record page and web flasher (no build step) |
| `docs/` | Spec, plans, hardware build guide and wiring diagram |
| `.github/workflows/` | CI and GitHub Pages deploy |

## Development

```bash
pip install -e '.[dev]' && pytest          # Python tests
cd app && npm test                         # JavaScript tests
cd firmware && pio test -e native          # firmware logic tests (no hardware)
```
