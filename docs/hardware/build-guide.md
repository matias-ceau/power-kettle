# Power Kettle: hardware build guide

This guide takes you from a bag of parts to a working sensor stuck on the
kettlebell and streaming data to your phone. You do not need to know embedded
programming. If you can solder a few wires onto small modules and use a
multimeter, you can do all of it.

Plan about one evening for the soldering and one more day for the epoxy to
cure.

Source of truth for pins and parts: spec
[section 3](../superpowers/specs/2026-10-14-power-kettle-design.md). If this
guide and the spec ever disagree, tell me; the spec wins for pins.

Contents

1. [What you are building](#1-what-you-are-building)
2. [Parts and tools](#2-parts-and-tools)
3. [Wiring](#3-wiring)
4. [Before you solder: check the BMI160 pin labels](#4-before-you-solder-check-the-bmi160-pin-labels)
5. [Build order](#5-build-order)
6. [Flash the firmware](#6-flash-the-firmware)
7. [First power-up checks](#7-first-power-up-checks)
8. [Soak test (30 minutes)](#8-soak-test-30-minutes)
9. [Mounting on the bell](#9-mounting-on-the-bell)
10. [Enabling GitHub Pages (one-time)](#10-enabling-github-pages-one-time)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. What you are building

A small box that sticks to the flat front face of the bell with a magnet.
Inside:

- a **BMI160** motion sensor (accelerometer plus gyroscope) that measures how
  the bell moves 400 times per second;
- an **ESP32-C3 SuperMini**, a thumb-sized computer with Bluetooth that reads
  the sensor and sends the numbers to your phone;
- a **LiPo battery** with a **TP4056** charger board and a **slide switch**.

The box does no analysis. It just sends raw numbers; the phone saves them and
a Python program analyses them later.

Power path in one line: LiPo, TP4056 (charges the battery and cuts it off if it
gets too empty), slide switch, the C3's **5V** pin. The C3 has a built-in
regulator that turns that into a steady 3.3 V, which also powers the sensor.
No extra voltage converter is needed.

## 2. Parts and tools

| Part | Notes | Search terms |
|---|---|---|
| ESP32-C3 SuperMini | Owned. Bluetooth 5, USB-C (used for flashing). Has an onboard 3.3 V regulator. | "ESP32-C3 SuperMini" |
| BMI160 breakout (purple GY-BMI160) | Owned. The motion sensor. | "GY-BMI160" |
| 1S LiPo battery, about 300 mAh, e.g. size 402535 | About 5 h of streaming. Check it has a protection circuit or at least that the TP4056 board does. Mind the plug type. | "402535 3.7V 300mAh lipo" |
| TP4056 USB-C charger **with protection** (DW01 chip, 6 solder pads) | The version with 6 pads (B+, B-, OUT+, OUT-, plus two for the USB) has over-discharge protection. The plain 4-pad one does **not**; do not buy that. Its USB-C port is for charging only. | "TP4056 type-c 6 pin protection DW01" |
| Slide switch | Battery on/off. Small SPDT or SPST slide switch. | "SS12D00 slide switch" or "mini slide switch 3 pin" |
| 2 x 100 kΩ resistors | Battery voltage divider (quarter-watt, any 1 % or 5 %). | "100k ohm resistor 1/4W" |
| Pot magnet, 20 to 25 mm, countersunk | Holds the box on the bell. Pull strength matters: about 30 N shear against about 7 N needed. | "pot magnet 25mm countersunk neodymium" |
| Small ABS project box, about 50 x 35 x 15 mm | No 3D printer needed. | "ABS enclosure 50x35x15" |
| Two-part epoxy (5 to 30 min type) | For magnet and sensor board. | "epoxy AB 5 minute" |

Also: thin hookup wire (silicone, 26 to 30 AWG, red, black and a few other
colours), heat-shrink tubing or Kapton tape, a thin rubber pad or a steel
washer (see section 9).

Tools: soldering iron with a fine tip, solder, flux (helps a lot on small
pads), wire cutters and strippers, **multimeter** (continuity beep plus
voltage), tweezers, a USB-C **data** cable (not a charge-only cable), desktop
or laptop with Chrome.

Safety with the LiPo: never short the two wires together, never puncture or
bend the cell, never solder while it is connected, and keep the
protected TP4056 in the circuit. If it ever gets hot, swells or smells,
disconnect it and stop using it.

## 3. Wiring

![Wiring diagram](wiring.svg)

Plain-text version of the same wiring (if the picture does not display, open
[wiring.svg](wiring.svg) directly):

| From | To | Why |
|---|---|---|
| LiPo | TP4056 **B+** / **B−** | Battery connects to the charger board's battery pads. |
| TP4056 **OUT+** | slide switch, then C3 **5V** | The switch cuts power to everything. |
| TP4056 **OUT−** | C3 **GND** | Common ground. |
| C3 **3V3** | BMI160 **3V3** | Powers the sensor straight from the C3's regulator. Use the board's **3V3** pin, not its VIN pin (VIN goes through the breakout's own regulator, which we skip). |
| C3 **GND** | BMI160 **GND** | Common ground. |
| C3 **GPIO6** | BMI160 **SDA** | I²C data line (the two-wire bus the sensor talks over). |
| C3 **GPIO7** | BMI160 **SCL** | I²C clock line. Runs at 400 kHz. |
| C3 **GPIO3** | BMI160 **INT1** | Sensor taps this pin when 20 new samples are ready. |
| Battery **+** (B+ pad) → 100 kΩ → C3 **GPIO4** → 100 kΩ → GND | divider, see diagram | Battery voltage sensing. Two equal resistors halve the voltage so the C3's input (max 3.3 V) can read a battery that is up to 4.2 V. The firmware doubles the reading. |
| BMI160 **CS** | 3V3 (high) | Tells the sensor to use I²C rather than SPI. |
| BMI160 **SAO** (sometimes labelled SDO) | GND (low) | Selects I²C address 0x68, which is what the firmware expects. |

Pins to leave alone: **GPIO2, GPIO8, GPIO9**. These are "strapping" pins that
the C3 reads at power-up to decide how to boot, so loads on them can stop it
starting. GPIO8 is the onboard LED, which the firmware uses for status only.
GPIO9 is the BOOT button.

Notes on the battery sense: tap the battery positive on the **battery side of
the switch** (the B+ pad or the red LiPo wire), not on the C3's 5V pin. This
is the "BAT+" of the spec's wiring table. It also means the battery reading is
about 0 mV when the C3 is running from USB with no battery attached, because
the divider then sees no battery. The divider draws
only about 18 µA from the battery, even with the switch off. That is
negligible, but it is why you should not leave the battery connected for months
without charging.

## 4. Before you solder: check the BMI160 pin labels

BMI160 breakout boards from different sellers label pins slightly differently,
and some already have CS or SAO tied to something by tiny resistors on the
board. Check yours first. Two small settings matter:

- **CS** (chip select). The BMI160 decides at power-up whether to talk I²C or
  SPI based on CS. For I²C, **CS must be high** (3.3 V).
- **SAO / SDO**. This sets the I²C address: **low = 0x68** (what the firmware
  uses), high = 0x69.

Steps:

1. Look at the board silkscreen (the printed text). Write down the order of
   the pins. Typical labels: VIN, 3V3, GND, SCL, SDA, CS, SAO (some boards
   print SDO instead of SAO, or VCC instead of VIN).
2. With nothing connected, set the multimeter to continuity (the beep mode).
   Touch one probe to the **3V3** pad and the other to the **CS** pad. A beep
   means the board already ties CS to 3V3. Repeat for **SAO to GND** and
   **SAO to 3V3**.
3. Write down what you found:
   - CS already tied to 3V3: nothing to do for CS.
   - Not tied: you will add a short wire from CS to 3V3 in the build.
   - SAO already tied to GND: nothing to do.
   - SAO floating (no beep to anything): add a short wire from SAO to GND.
   - SAO tied to 3V3: that gives address 0x69. Cut that tie or choose a board
     that lets you ground SAO. The firmware only looks at 0x68 and will report
     "chip not found".
4. Check the labels against the wiring table in section 3. The diagram uses
   the names SDA, SCL, INT1, CS, SAO. If your board prints something different
   for any of them, trust the silkscreen and look the board up.

## 5. Build order

Doing it in this order lets you test each stage before the next one hides it.

### Step 1: charger and switch (no battery connected yet)

1. Solder a short red wire to **OUT+** and a short black wire to **OUT−** on
   the TP4056 board.
2. Cut the red wire in the middle and solder the slide switch into it (use the
   middle pin and one outer pin of a 3-pin switch). Cover the joints with
   heat-shrink.
3. Check with the multimeter (continuity): with the switch in "on", the two
   ends of the red wire beep; in "off" they do not.

### Step 2: check battery polarity, then connect the battery

LiPo plugs and TP4056 boards from different sellers are **not always wired the
same way round**. Plugging in the wrong way round can destroy the charger and
is a fire risk.

1. Find the battery's red (+) and black (−) wires. If it has a plug, the plug
   will not necessarily match any board, so I recommend cutting the plug off
   and soldering the wires directly (keep the battery wires covered and
   separate while you do this; do one wire at a time, so the two bare ends
   never touch).
2. With a multimeter on DC volts, check the battery voltage on the bare wires:
   red probe on red wire. You should read between about 3.3 and 4.2 V and a
   positive sign. A negative sign means your wire colours are swapped.
3. Solder battery **+** to **B+** and battery **−** to **B−** on the TP4056.
4. Measure between **OUT+** and **OUT−** with the switch on: you should read
   the battery voltage (3.3 to 4.2 V).
5. Plug a phone charger into the TP4056's USB-C port: one LED should show
   charging (usually red), and it turns off or changes to a different colour
   (usually green or blue) when full.

### Step 3: the C3 and the battery sense divider

1. Solder a wire from TP4056 **OUT−** to a **GND** pad on the C3.
2. Solder the switched red wire to the C3 **5V** pad.
3. Build the divider: twist or solder the two 100 kΩ resistors end to end.
   Solder one free end to the battery **+** (the B+ pad, so on the battery
   side of the switch). Solder the other free end to a C3 **GND** pad. Solder
   a short wire from the **junction between the two resistors** to **GPIO4**.
   Cover each bare joint with heat-shrink or Kapton tape so nothing shorts.
   Use the multimeter to confirm the junction reads half the battery voltage
   with the switch off or on (for example, about 1.9 V for a 3.8 V battery).
4. **Do not** connect any other peripheral to GPIO2, GPIO8 or GPIO9.

### Step 4: the sensor

1. Decide on the CS / SAO wires from your section 4 notes.
2. Solder 3V3 (C3) to BMI160 **3V3**, GND to GND, **GPIO6 to SDA**,
   **GPIO7 to SCL**, **GPIO3 to INT1**.
3. Add the CS-to-3V3 and SAO-to-GND jumpers if your board needed them.
4. Check the I²C wires have not swapped: SDA is GPIO6, SCL is GPIO7.
5. Double-check there are no solder bridges between neighbouring pads with the
   multimeter in continuity mode (SDA to SCL should not beep, 3V3 to GND should
   not beep).

### Step 5: bench test before putting it in the box

Do the flashing and first power-up checks in sections 6 and 7 with everything
wired but loose on the desk. It is far easier to fix a wire now than inside
the box.

## 6. Flash the firmware

"Flashing" means copying the program onto the C3. You only do it once, or
again when the firmware is updated.

**Before you plug the C3 into the computer, switch the slide switch off** (or
leave the battery disconnected) so the C3 is powered only from USB. Two power
sources feeding the 5V pin at once is not good for either of them.

1. Use **desktop Chrome or Edge** (Safari, Firefox and Android Chrome do not
   work for this). The page needs the firmware to be published first; see
   [section 10](#10-enabling-github-pages-one-time).
2. Plug the C3 into the computer with a USB-C **data** cable.
3. Open <https://matias-ceau.github.io/power-kettle/flash.html>.
4. Click **Install**, pick the C3 in the list that pops up (it often appears
   as "USB JTAG/serial debug unit" or "ESP32-C3"), and confirm. It takes about
   30 s. Wait for "Installation complete".
5. If the C3 does not show up in the list: unplug it, **hold the BOOT button**
   (tiny button on the board), plug it back in while holding it, release, then
   try again.

For developers (needs PlatformIO and the repo checked out):

```bash
cd firmware && pio run -e c3 -t upload
```

## 7. First power-up checks

Do checks 1 to 3 with the C3 plugged into USB (switch off) and checks 4 to 6
on battery power (USB unplugged).

1. **LED.** After flashing, the onboard blue LED should blink slowly, about
   once a second. That means it is advertising: waiting for a phone.
   - Slow blink 1 Hz: waiting for a connection.
   - Solid on: connected to a phone.
   - Fast blink 5 Hz: low battery (below 3500 mV).
   - **On USB power with no battery, the battery reading is about 0 mV, so a
     fast blink and the "Low battery" banner on the record page are normal on
     the bench.** Ignore them until you test on battery.
2. **Record page connects.** On an Android phone with Bluetooth and location
   switched on, open Chrome and go to
   <https://matias-ceau.github.io/power-kettle/record.html>. Tap **Connect**,
   pick **PowerKettle**. The LED goes solid. The page shows the battery
   reading.
3. **Start recording** and watch the readouts for 10 to 20 seconds with the
   sensor lying still:
   - **Samples** keeps climbing.
   - **Rate (Hz)** settles at about **400** (anything within about 396 to 404
     is fine; the chip clock is up to about 1 % off by design and the analysis
     corrects for it).
   - **Gaps** stays at 0 and **Dropped** stays at 0.
4. **Battery mV is plausible**: with a charged LiPo, 3700 to 4200 mV. Compare
   with your multimeter reading of the battery (they should agree within about
   50 to 100 mV).
5. **Slide switch.** With the USB cable out, switch on: the LED should start
   blinking within a couple of seconds. Switch off: the LED goes out.
6. **Charging.** Plug a charger into the TP4056's USB-C port with the switch
   on or off: the charging LED lights up.
7. Tap **Stop & save** and check the file with
   `python -m powerkettle info FILE.pkraw` (see
   [recording-protocol.md](recording-protocol.md) for how to get it onto a
   computer). You want 1 run, about 400 Hz, 0 gaps.

If the sensor is not found, the serial log shows the problem. Open the serial
monitor (`pio device monitor -b 115200` or any serial terminal at **115200
baud** on the C3's USB port). If the BMI160 is not detected it prints the chip
ID failure and retries every second. See troubleshooting.

## 8. Soak test (30 minutes)

A soak test checks that nothing falls over during a long recording. It
catches loose wires, a bad Bluetooth setup or a flaky battery.

1. Fully charge the LiPo. Put the sensor on the desk, **not moving**.
2. Put the phone next to it (within 1 m) with the screen on and Chrome in the
   foreground. Turn off battery saver. The record page keeps the screen awake
   by itself.
3. On the record page, note the **Battery** reading (mV). Tap **Start
   recording**.
4. Leave for **30 minutes** without touching the sensor or the phone.
5. Note the **Battery** reading again, then tap **Stop & save**.
6. Get the file onto a computer (see
   [recording-protocol.md](recording-protocol.md), "Getting files to your
   computer") and run:

   ```bash
   pip install git+https://github.com/matias-ceau/power-kettle
   python -m powerkettle info pk-YYYYMMDD-HHMMSS.pkraw
   ```

**Pass criteria**

| Line in the output | Expected |
|---|---|
| `runs:` | **1** |
| `rate` on the run line | **400 ± 4 Hz** (396 to 404) |
| `gaps:` | **0** (and 0 samples dropped) |
| `duration:` | about 1800 s (30 min) |
| `samples:` | about 720,000 |

Also write down the **battery drop** (mV at start minus mV at end). A healthy
cell should lose only a small amount in 30 minutes (my expectation is a few
tens of mV, but this is a first measurement, so your number becomes the
baseline). A drop of 200 mV or more suggests an old cell or extra current
draw. Keep the numbers; they tell you the real runtime.

If it does not pass: more than 1 run means the phone and sensor disconnected;
gaps mean data was lost (phone too far away, Chrome in the background, battery
saver). Try again closer; if it keeps happening, tell me and bring the file.

## 9. Mounting on the bell

Do this only after the bench tests and the soak test pass.

### Where it goes

On the **front flat face** of the bell, the circular "32KG" plate (about 10 cm
across), on the side that faces you at the hike (the backswing start). Why
there: it is the one place that is away from the handle window and from the
forearm contact for either hand.

### Dummy-box clearance test (do not skip)

Gluing a magnet to the wrong spot is annoying. Test first:

1. Make a dummy: a cardboard box about 50 x 35 x 15 mm, or just use the real
   box lid with nothing in it.
2. Tape it centred on the front face with painter's tape (removes cleanly).
3. Do a few easy swings and snatches with each hand, slowly at first, and
   watch for:
   - the forearm brushing the box;
   - the bell hitting your thighs or hips on the way back;
   - the box touching anything at the top of the snatch.
4. Adjust the position until nothing touches. Mark the final spot with a
   pencil dot.

### Build the box

1. **Rigid sensor board.** Fix the BMI160 board **rigidly** to the box,
   **parallel to the box base** (the side that touches the bell), components
   facing up. Use epoxy or a small screw with spacers. **No foam, no tape
   cushion**: any flex between sensor and bell shows up as noise. The reason
   it must be parallel is that one of the sensor's axes then points straight
   out of the bell's face, which the analysis relies on. It does not matter
   how the box is rotated on the face; the app handles that at the start of
   every session.
2. Put the C3, charger board and battery in the box; keep wires short and
   tidy, and make sure the **slide switch** and the charger's USB-C port are
   reachable through cutouts (small holes you make with a hobby knife or
   rotary tool). Cover bare solder joints and the underside of the boards
   with Kapton tape so nothing shorts against anything else inside the box.
3. Make sure the C3's USB-C is reachable if you want to reflash later, or
   plan to open the box.

### Magnet

1. Roughen the magnet's back and the box base with sandpaper and wipe with
   alcohol.
2. **Epoxy the pot magnet to the box base**, flat side outward (the countersunk
   hole filled by epoxy or left empty, whichever you prefer). Let it cure
   fully (24 h for most epoxies even if the label says 5 minutes).
3. If the raised lettering of the "32KG" plate stops the magnet sitting flat,
   put a **thin rubber pad or a steel washer** between magnet and bell so it
   grips without rocking.
4. Before the first session, pull on the box with your hand sideways and
   upwards: it should not slide. If it slips, add a strip of VHB double-sided
   tape between magnet and bell as a backup.

Check the box after the first few sessions, and tape the cable cutouts if you
see any wobble.

## 10. Enabling GitHub Pages (one-time)

The flasher page and the record page are served from GitHub Pages. This is a
one-time setup in the repository settings:

1. On GitHub, open the repository, **Settings**, **Pages**.
2. Under **Build and deployment**, set **Source** to **GitHub Actions**.
3. The site is built and published from the **`main` branch**. If this
   milestone's work is still on another branch, merge it into `main` first;
   the publishing workflow ("Deploy Pages") runs automatically on every push to
   `main`. You can watch it under the **Actions** tab. A green tick means the
   site is live.
4. After the first successful run the pages are at
   - <https://matias-ceau.github.io/power-kettle/flash.html>
   - <https://matias-ceau.github.io/power-kettle/record.html>

   It can take a minute or so after the workflow finishes before they load.

## 11. Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Nothing happens when switched on, no LED | Battery not connected or flat. Check the TP4056 OUT+/OUT− voltage with the switch on. Check the switch is wired into the red wire correctly. |
| C3 not listed in the flash dialog | Charge-only USB cable. Use another. Or hold BOOT while plugging in. |
| LED blinks fast on the bench | Normal with USB and no battery (reads 0 mV). With the battery connected, it means below 3500 mV: charge it. |
| LED blinks slowly, phone cannot find "PowerKettle" | Bluetooth and location off on the phone; another phone is already connected; or you are not using Chrome. |
| Record page says no Web Bluetooth | Use Chrome on Android (not Firefox, not an in-app browser). |
| Rate far from 400 Hz, or no samples | Sensor not talking. See next row. |
| Serial log says chip ID failure (retries every second) | BMI160 not found. Check SDA/SCL are not swapped, 3V3 and GND are connected, **SAO is low** (address 0x68), **CS is high**, and no solder bridges. |
| Battery mV reads 0 on battery power | Divider not connected to GPIO4 or battery +, or a broken resistor joint. |
| Battery mV reads about double or half | Check both resistors are really 100 kΩ and the midpoint goes to GPIO4. |
| Gaps in a recording | Phone too far, Chrome in the background, battery saver on. |
| Device resets or disconnects when moved | Loose wire or solder joint. Strain-relieve the wires. |
