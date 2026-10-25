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
- a **LiPo battery** with a **TP4056** charger board, a **slide switch**, a
  small **5 V boost** board and a **3.3 V buck** board.

The box does no analysis. It just sends raw numbers; the phone saves them and
a Python program analyses them later.

Power in two lines:

- **Running:** LiPo, TP4056 (it also cuts the battery off if it gets too
  empty), slide switch, boost (battery 3.0 to 4.2 V up to 5 V), buck (5 V
  down to a steady 3.3 V), the C3's **3V3** pin. The sensor shares that 3.3 V.
- **Charging:** a USB cable in the **C3's** USB-C. The C3's **5V** pin
  carries the USB 5 V, and it is wired to the TP4056's **IN+**, so the C3's
  port charges the battery. One port for charging, flashing and the serial
  log; the TP4056's own USB-C port stays unused.

## 2. Parts and tools

| Part | Notes | Search terms |
|---|---|---|
| ESP32-C3 SuperMini | Owned. Bluetooth 5, USB-C (charging, flashing, serial log). | "ESP32-C3 SuperMini" |
| BMI160 breakout (purple GY-BMI160) | Owned. The motion sensor. | "GY-BMI160" |
| 1S LiPo battery, about 300 mAh, e.g. size 402535 | About 4 h of streaming with the boost and buck (the soak test measures yours). Mind the plug type (section 5, step 3). | "402535 3.7V 300mAh lipo" |
| TP4056 USB-C charger **with protection** | The protected version has **two extra small chips next to the TP4056 chip, marked DW01A and 8205A**, and 6 solder pads: **IN+, IN−** (the USB input, next to the USB-C connector), **B+, B−** (battery), **OUT+, OUT−** (load). The plain 4-pad one has no over-discharge protection; do not use it. | "TP4056 type-c 6 pin protection DW01" |
| **4.7 kΩ resistor** (SMD, same size as the one you are replacing, probably 0805 or 0603) | **Mandatory.** Replaces the TP4056's charge-current resistor so the battery is charged gently and the USB port is not overloaded (section 5, step 1). | "4.7k ohm resistor SMD 0805" (or 0603) |
| Small **5 V boost** board | Owned. Battery voltage up to 5 V for the buck. Any small board that starts from 3 V or less and gives at least 300 mA. Later you can swap in an adjustable one (XL6009 type) set to 6 to 7 V; set and measure its output **before** connecting the buck, because those can output far more than the buck accepts. | "mini boost 5V 3.7V" |
| **3.3 V buck** board (Mini560 type) | Owned (marked EY9, chip GMH3J). Makes the steady 3.3 V. It wants about 5 V or more at its input, which is why the boost sits in front of it. | "Mini560 3.3V" |
| Slide switch | Battery on/off. Small SPDT or SPST slide switch. | "SS12D00 slide switch" or "mini slide switch 3 pin" |
| 2 x 100 kΩ resistors | Battery voltage divider (quarter-watt, any 1 % or 5 %). | "100k ohm resistor 1/4W" |
| 47 Ω resistor, quarter-watt (test only) | A dummy load to check the buck before it meets the C3 (section 5, step 4). Gets warm; that is normal. | "47 ohm resistor 1/4W" |
| Pot magnet, 20 to 25 mm, countersunk | Holds the box on the bell. Pull strength matters: about 30 N shear against about 7 N needed. | "pot magnet 25mm countersunk neodymium" |
| Small ABS project box, about 60 x 40 x 20 mm | The boost and buck boards need a little more room than before. No 3D printer needed. | "ABS enclosure 60x40x20" |
| Two-part epoxy (5 to 30 min type) | For magnet and sensor board. | "epoxy AB 5 minute" |

Also: thin hookup wire (silicone, 26 to 30 AWG, red, black and a few other
colours), heat-shrink tubing or Kapton tape, a thin rubber pad or a steel
washer (see section 9), a **2-pin connector pigtail that matches your
battery's plug** (strongly recommended, section 5, step 3), and masking tape
and a marker for labelling the box.

Tools: soldering iron with a fine tip, solder, flux (helps a lot on small
pads), solder wick, wire cutters and strippers, **multimeter** (resistance,
continuity beep, DC voltage), tweezers, a USB **data** cable (not a
charge-only cable), and a desktop or laptop with Chrome.

Safety with the LiPo: never short the two wires together, never puncture or
bend the cell, never solder on it while it is connected to anything powered,
and keep the protected TP4056 in the circuit. If it ever gets hot, swells or
smells, disconnect it and stop using it.

## 3. Wiring

> **RULE: Slide switch OFF whenever a USB cable is in the C3. Charge and
> flash through the C3's USB-C only; never plug anything into the TP4056's
> own USB-C port.**
>
> Why switch OFF: with a cable in, the C3 runs from USB and the TP4056
> charges the battery. If the switch is also ON, the boost and buck feed the
> same 3.3 V line as the C3's own regulator (two regulators fighting), and the
> charger never sees the battery as full, so it never stops. A moment of
> forgetting will not break anything; just do not leave it like that.
>
> Why not the TP4056's port: it is wired straight to the C3's USB 5 V. A
> cable in each port at once would connect two power supplies together.
> Put tape over the TP4056's port.

![Wiring diagram](wiring.svg)

Plain-text version of the same wiring (if the picture does not display, open
[wiring.svg](wiring.svg) directly):

| From | To | Why |
|---|---|---|
| LiPo | TP4056 **B+** / **B−** | Battery connects to the charger board's battery pads. **B− is NOT ground.** Only the battery's negative wire goes to B−; the ground of the circuit is **OUT−**. B− is the unprotected battery side: anything else connected to it bypasses the protection chip. |
| TP4056 **OUT+** | slide switch → boost **IN+** | The switch cuts power to everything after it. |
| TP4056 **OUT−** | boost **IN−** | Ground. |
| boost **OUT+** (5 V) | buck **IN+** | |
| boost **OUT−** | buck **IN−** | Ground. |
| buck **OUT+** (3.3 V) | C3 **3V3** | Powers the C3 directly on its 3.3 V pin. **Measure the buck output before making this connection** (section 5, step 4). |
| buck **OUT−** | C3 **GND** | Ground. |
| C3 **5V** | TP4056 **IN+** | The C3's 5V pin is its USB 5 V. This wire is how the C3's USB-C charges the battery. Nothing else connects to the C3's 5V pin. |
| C3 **GND** (another GND pad) | TP4056 **IN−** | USB ground for the charger. |
| C3 **3V3** | BMI160 **3V3** | Powers the sensor from the same 3.3 V line. Use the board's **3V3** pin, not its VIN pin (VIN goes through the breakout's own regulator, which we skip). |
| C3 **GND** | BMI160 **GND** | Common ground. |
| C3 **GPIO6** | BMI160 **SDA** | I²C data line (the two-wire bus the sensor talks over). |
| C3 **GPIO7** | BMI160 **SCL** | I²C clock line. Runs at 400 kHz. |
| C3 **GPIO3** | BMI160 **INT1** | Sensor taps this pin when 20 new samples are ready. |
| TP4056 **OUT+** (before the switch) → 100 kΩ → C3 **GPIO4** → 100 kΩ → GND | divider, see diagram | Battery voltage sensing. The C3's ADC (analogue-to-digital converter, the part that measures voltage) can only measure about 0 to 2.5 V in the mode the firmware uses, so two equal resistors halve the battery's 3.0 to 4.2 V. The firmware doubles the reading. |
| BMI160 **CS** | 3V3 (high) | Tells the sensor to use I²C rather than SPI. |
| BMI160 **SAO** (sometimes labelled SDO) | GND (low) | Selects I²C address 0x68, which is what the firmware expects. |

Pins to leave alone: **GPIO2, GPIO8, GPIO9**. These are "strapping" pins that
the C3 reads at power-up to decide how to boot, so loads on them can stop it
starting. GPIO8 is the onboard LED, which the firmware uses for status only.
GPIO9 is the BOOT button.

Notes:

- All the minus pads (TP4056 OUT− and IN−, boost IN− and OUT−, buck IN− and
  OUT−, every C3 GND, BMI160 GND) are one ground. You will confirm with a
  continuity beep that the TP4056's IN− and OUT− are the same wire on your
  board (section 5, step 1).
- The divider taps **OUT+ before the switch**. On a protected TP4056, OUT+ is
  electrically the same wire as B+ (the battery positive), so the reading is
  the true battery voltage, and it works with the switch OFF while charging
  from USB.
- The divider draws about 18 µA from the battery all the time, even with the
  switch off. That is negligible, but it is why you should not leave the
  battery for months without charging.
- **Low battery.** The boost keeps the 3.3 V steady almost until the battery
  is empty, so the device keeps working well below the point where a LiPo
  should be recharged; the TP4056's protection only cuts it off at about
  2.5 V, which is hard on the cell. So: when the LED blinks fast or the app
  says "Low battery" (below 3.5 V), stop and charge. Do not leave the device
  switched on overnight.
- The boost and buck switch at high frequency, which can add electrical noise.
  Keep them at the far end of the box from the sensor if you can. The soak
  test (section 8) measures the sensor's noise at rest, so you will see if it
  matters.

## 4. Before you solder: check the BMI160 pin labels

BMI160 breakout boards from different sellers label pins slightly differently,
and some already have CS or SAO tied to something on the board, either by a
tiny resistor or by a hard connection. Check yours first. Two small settings
matter:

- **CS** (chip select). The BMI160 decides at power-up whether to talk I²C or
  SPI based on CS. For I²C, **CS must be high** (3.3 V).
- **SAO / SDO**. This sets the I²C address: **low = 0x68** (what the firmware
  uses), high = 0x69.

Steps (board unpowered, nothing connected to it):

1. Look at the board silkscreen (the printed text). Write down the order of
   the pins. Typical labels: VIN (or VCC), 3V3, GND, SCL, SDA, CS, SAO (some
   boards print SDO instead of SAO), **INT1**, **INT2**. We use INT1 only;
   leave INT2 unconnected.
2. Set the multimeter to **resistance (Ω)**, not the continuity beep. A beep
   only tells you the resistance is below a few tens of ohms; it cannot tell
   a hard connection from a small resistor, and that difference matters here.
   First measure **3V3 to GND**: it should read high (kΩ or more, possibly
   climbing slowly). If it reads near 0 Ω, the board is faulty; stop.
3. Measure these four pairs and write the four numbers down:
   **CS to 3V3**, **CS to GND**, **SAO to 3V3**, **SAO to GND**.
   Read each as: near 0 Ω (under about 10 Ω) = hard connection; a few kΩ to a
   few hundred kΩ = a resistor to that rail; "OL" or very high = no
   connection.
4. Decide, checking **both** rails each time:
   - **CS**: if it is hard-connected to **GND** (near 0 Ω to GND), **stop**: a
     jumper from CS to 3V3 would short 3V3 to GND through it. Cut the board's
     tie, or use a different board. If CS already reads a resistor or hard
     connection to 3V3, nothing to do. If it reads a resistor to GND (a pull-
     down), the jumper to 3V3 still works but wastes a little current. If it
     reads open to both, add a short jumper from CS to 3V3 in the build.
   - **SAO**: same logic mirrored. Hard-connected to 3V3 gives address 0x69,
     which the firmware will not find ("chip not found"); cut that tie. If it
     is already tied (hard or by a resistor) to GND, nothing to do. If it is
     open to both, add a jumper from SAO to GND. Never add a jumper to a rail
     the pin is hard-tied to the other rail of.
5. Check the labels against the wiring table in section 3. The diagram uses
   the names SDA, SCL, INT1, CS, SAO. If your board prints something different
   for any of them, trust the silkscreen and look the board up.

## 5. Build order

Three principles:

- **Solder the wires onto the TP4056 before the battery goes on.** Once the
  battery is attached, OUT+ and B+ are always live.
- **Switch OFF while soldering anything after the switch**, every time, and
  unplug the battery's connector if you have one.
- Test each stage before the next one hides it. In particular, **the buck's
  output is measured before it ever touches the C3.**

### Step 1: the TP4056 board (mandatory charge-current fix, no battery)

*No battery: nothing is connected to this board yet.*

**Why this matters.** The TP4056 charges at a current set by one small
resistor ("R_PROG", on the chip's pin 2). The rule of thumb is
**charge current in mA = 1200 ÷ R in kΩ**. These modules are almost always
sold with a **1.2 kΩ** resistor, which means **1000 mA**. Your battery holds
only 300 mAh, so 1000 mA is about 3.3 times its capacity per hour (3.3 C).
Normal LiPo cells are meant to be charged at 1 C or less (300 mA here). Charging
at 3.3 C makes the cell hot, wears it out fast and can make it swell or catch
fire. It would also pull about 1 A through the C3's USB port, more than a
computer port gives. So before it ever sees the battery, change the resistor
to **4.7 kΩ**: 1200 ÷ 4.7 ≈ **255 mA**, under 1 C.

1. Find the **TP4056 chip**: an 8-pin chip with "TP4056" printed on it.
   Pin 1 is at the dot or the corner marked on the chip; pin 2 is the next pin
   along the same side (its name is PROG).
2. Find the small rectangular resistor connected to **pin 2**, usually
   right next to the chip. It is typically marked **122** (that code means
   12 followed by two zeros: 1200 Ω = 1.2 kΩ). Other markings for the same
   value: 1201, 1.2k. Confirm with the multimeter: measured from pin 2 to
   the board's ground (the OUT− pad), it should read about 1.2 kΩ.
   A different value is possible (marked 102 = 1 kΩ, 152 = 1.5 kΩ, 222 =
   2.2 kΩ...): in that case divide 1200 by what you find to see what current
   you have, and still replace it with 4.7 kΩ (marked **472**).
3. Remove that resistor: add fresh solder to both ends, then heat both ends
   alternately and nudge the resistor off with tweezers (or use solder wick
   and a hot iron, or a hot-air tool). Clean the pads with flux and wick.
4. Solder the **4.7 kΩ** resistor (marked 472) in its place. Cool; wipe off
   flux with alcohol. Check there is no solder bridge to the neighbouring chip
   pins.
5. Measure pin 2 to ground again: about **4.7 kΩ**. If you read 1.2 kΩ, you
   fitted nothing or fitted the wrong part. Fix before going on.
6. Confirm the board is the **protected** kind: two extra small chips near
   the TP4056 marked **DW01A** and **8205A**. If you cannot find them, do not
   use the board.
7. Three continuity checks (multimeter on the beep setting):
   - **B+ to OUT+: beeps.** Same wire.
   - **IN− to OUT−: beeps.** Same wire (the circuit's ground). This matters
     because the C3's ground goes to IN−. **If IN− beeps to B− instead, stop
     and tell me:** on that board the wiring would bypass the protection.
   - **B− to OUT−: no beep.** The protection chip sits between them; they must
     never be joined.

### Step 2: pre-wire the TP4056 pads (still no battery)

*No battery.* Soldering is easier while the board is dead. Use wires long
enough to reach where each part will sit in the box.

1. **OUT+**: a red wire (to the switch).
2. **OUT−**: a black wire (to the boost's IN−).
3. **OUT+** again: one leg of the first **100 kΩ** divider resistor (or a
   short wire to it). Cover it with heat-shrink, leaving the other end free.
4. **IN+**: a wire to go to the C3's **5V** pin later (use a colour you will
   not mix up with the 3.3 V wire; it carries USB 5 V).
5. **IN−**: a black wire to go to a C3 **GND** pad later.
6. Put tape over the TP4056's own USB-C port so nobody plugs into it.

### Step 3: attach the battery

1. LiPo plugs and TP4056 boards from different sellers are **not always wired
   the same way round**. A wrong connection can destroy the charger and is a
   fire risk. Find the battery's red (+) and black (−) wires. With a
   multimeter on DC volts, red probe on the red wire: you should read about
   3.3 to 4.2 V with a **positive** sign. A negative sign means the wire
   colours are swapped (go by the meter, not the colour).
2. **Preferred, no soldering on a live cell:** a 2-pin connector pigtail that
   mates with your battery's plug (usually JST-PH 2.0 or JST-SH 1.25; check
   the plug). With the battery unplugged, solder the pigtail's wires to
   **B+** and **B−** on the TP4056. Check the pigtail's polarity with the
   meter against the battery plug: the battery's red pin must meet the
   pigtail wire going to **B+**. Only then plug the battery in. With a
   connector, unplug the battery whenever you solder in the later steps.
3. **If the battery has bare wires** (or its plug does not match and you cut
   it off): cover the bare end of the **black** wire with Kapton tape, then
   solder the **red** wire to **B+**. Only then uncover the black wire and
   solder it to **B−**. Keep the iron tip away from the neighbouring pad: B+
   and B− are close together, and a short across the cell is dangerous. At no
   point may the two bare ends touch each other or the board's other pads.
   From now on the red OUT+ wire and the divider resistor are live: keep their
   free ends taped until you solder them.
4. Measure between **OUT+** and **OUT−**: the battery voltage (3.3 to
   4.2 V). 0 V here is normal on some protected boards until a charger has
   been connected once; it will come alive when you first plug USB into the
   C3 in section 7.

### Step 4: switch, boost and buck, then measure (no C3 yet)

*Switch OFF while soldering; unplug the battery if you have a connector.*

1. Solder the slide switch into the red OUT+ wire (use the middle pin and one
   outer pin of a 3-pin switch). Cover the joints with heat-shrink.
2. Switch side of the red wire to the **boost IN+**; the black OUT− wire to
   the **boost IN−**.
3. If your boost board has voltage jumpers, set it to **5 V**.
4. **Boost OUT+** to **buck IN+**, **boost OUT−** to **buck IN−** (short
   wires).
5. Solder a red-ish wire to **buck OUT+** and a black wire to **buck OUT−**,
   long enough to reach the C3. Leave their ends **free and apart** for now.
6. **Measure.** Battery plugged in, switch **ON**:
   - boost OUT+ to OUT−: **4.9 to 5.3 V**;
   - buck OUT+ to OUT−: **3.25 to 3.35 V**.
7. **Load test.** Hold the **47 Ω** resistor across the buck's OUT+ and OUT−
   wires (it draws about 70 mA, a bit more than the device uses) and measure
   again: still **3.2 V or more**. It gets warm within a few seconds; that is
   normal. Remove it.
8. Switch **OFF**.

**If the buck output is above 3.45 V or reads near 5 V, stop: do not connect
it to the C3.** 5 V on the 3V3 pin destroys the C3 and the sensor. If it is
below 3.2 V under load, the boost is probably not giving 5 V: check the boost
output under the same load. Tell me the numbers either way.

### Step 5: the C3 connections

*Switch OFF, battery unplugged if you have a connector.*

1. **Buck OUT+** to the C3 **3V3** pad. **Buck OUT−** to a C3 **GND** pad.
2. The **IN+** wire from the TP4056 to the C3 **5V** pad.
3. The **IN−** wire from the TP4056 to another C3 **GND** pad (two wires on
   one GND pad is fine too).
4. The divider: solder the free end of the first 100 kΩ (the one on OUT+) to
   one end of the second 100 kΩ. Solder the other end of the second resistor
   to a C3 **GND** pad (or any ground wire). Solder a short wire from the
   **junction between the two resistors** to **GPIO4**. Cover each bare joint
   with heat-shrink or Kapton tape so nothing shorts.
5. **Do not** connect anything to GPIO2, GPIO8 or GPIO9.

### Step 6: the sensor

*Switch OFF, battery unplugged if you have a connector.*

1. Decide on the CS / SAO wires from your section 4 notes.
2. Solder C3 **3V3** to BMI160 **3V3**, GND to GND, **GPIO6 to SDA**,
   **GPIO7 to SCL**, **GPIO3 to INT1**.
3. Add the CS-to-3V3 and SAO-to-GND jumpers only if your section 4 notes say
   so.
4. Check the I²C wires have not swapped: SDA is GPIO6, SCL is GPIO7.

### Step 7: checks before the first power-up

*Switch OFF, battery unplugged if you have a connector.*

1. Multimeter in continuity mode: no solder bridges between neighbouring pads
   (SDA to SCL should not beep). C3 **3V3** to **GND** should not beep (the
   meter may chirp for an instant as capacitors charge; a steady beep is a
   short). C3 **5V** to **GND** should not beep.
2. Visual check: the only wire on the C3's **5V** pad is the one from the
   TP4056's **IN+**; the buck's output goes to **3V3**, not 5V.

Then flash the firmware (section 6) and do the power-up checks (section 7)
with everything still loose on the desk. It is far easier to fix a wire now
than inside the box.

## 6. Flash the firmware

"Flashing" means copying the program onto the C3. You only do it once, or
again when the firmware is updated.

> **RULE: Slide switch OFF whenever a USB cable is in the C3.**

With the switch off, the C3 runs from the USB cable (and the battery
charges at the same time). It makes no difference whether the battery is
attached.

1. Use **desktop Chrome or Edge** (Safari, Firefox and Android Chrome do not
   work for this). The page needs the firmware to be published first; see
   [section 10](#10-enabling-github-pages-one-time).
2. **Check the slide switch is OFF.** Then plug the C3 into the computer with
   a USB **data** cable.
3. Open <https://matias-ceau.github.io/power-kettle/flash.html>.
4. Click **Install**, pick the C3 in the list that pops up (it often appears
   as "USB JTAG/serial debug unit" or "ESP32-C3"), and confirm. It takes about
   30 s. Wait for "Installation complete".
5. If the C3 does not show up in the list: unplug it, **hold the BOOT button**
   (tiny button on the board), plug it back in while holding it, release, then
   try again.

**Reflashing later, with the device in its box:** same thing. Switch OFF,
plug into the C3's port, flash.

For developers (needs PlatformIO and the repo checked out; the same rule
applies, switch OFF first):

```bash
cd firmware && pio run -e c3 -t upload
```

## 7. First power-up checks

The LED has three modes: slow blink 1 Hz = advertising (waiting for a phone),
solid on = connected, fast blink 5 Hz = **low battery (below 3500 mV)**. Low
battery **overrides** the other two: if the battery reads below 3500 mV, the
LED fast-blinks even while connected.

**Part A: on USB (switch OFF).** Right after flashing, cable still in.

1. **Charging LED.** The TP4056's charge LED lights (usually red), even
   though the cable is in the C3: that is the C3's 5V pin feeding the
   charger's IN+. If the battery is already full it shows the "full" colour
   (usually green or blue) instead. Feel the TP4056 chip and the cell after a
   few minutes: warm is fine, hot is not (unplug, recheck the 4.7 kΩ). At
   about 255 mA a flat 300 mAh cell takes roughly 1.5 to 2 hours.
2. **USB voltage at the charger.** Measure TP4056 **IN+** to **IN−**: about
   4.6 to 5.1 V.
3. **Record page connects.** On an Android phone with Bluetooth and location
   switched on, open Chrome and go to
   <https://matias-ceau.github.io/power-kettle/record.html>. Tap **Connect**,
   pick **PowerKettle**.
4. **Battery mV is plausible**: within about 50 to 100 mV of your multimeter
   reading at OUT+ to OUT−. While charging it reads higher than the battery's
   resting voltage and climbs towards 4200 mV; that is normal. If it is below
   3500 mV the LED fast-blinks and the page shows "Low battery": leave it
   charging.
5. **Start recording** and watch the readouts for 10 to 20 seconds with the
   sensor lying still:
   - **Samples** keeps climbing.
   - **Rate (Hz)** settles at about **400** (anything within about 396 to 404
     is fine; the chip clock is up to about 1 % off by design and the analysis
     corrects for it).
   - **Gaps** stays at 0, **Runs** stays at 1, and **Dropped** stays at 0.
     (A Runs count above 1 means the Bluetooth link dropped and the page
     reconnected; data around each drop is lost.)
   - No "Bluetooth MTU too small" or "Device is not streaming" banner.
6. **Bluetooth packet size (serial log).** Open the serial monitor (see the
   end of this section) and connect the record page again. The log must show
   `[ble] mtu 247 -> 19 samples/packet` shortly after `[ble] connected`. A
   smaller MTU (for example `mtu 23 -> 1 samples/packet`) means the phone did
   not accept a larger packet size: the recording still works but sends far
   more, smaller packets; try another phone or update Chrome.

**Part B: on battery (USB unplugged).**

7. Unplug the cable, then **switch ON**: within a couple of seconds the LED
   blinks slowly (or fast, if the battery is below 3500 mV: charge it).
8. Measure C3 **3V3** to **GND**: **3.25 to 3.35 V**.
9. Connect with the record page: LED solid, battery reading close to the
   multimeter at OUT+. Do one short recording (Start, wait 20 s, Stop & save),
   then check the file with `python -m powerkettle info FILE.pkraw` (see
   [recording-protocol.md](recording-protocol.md) for how to get it onto a
   computer). You want 1 run, about 400 Hz, 0 gaps.
10. Switch OFF: the LED goes out.

If the sensor is not found, the serial log shows the problem. Open the serial
monitor (`pio device monitor -b 115200` or any serial terminal at **115200
baud** on the C3's USB port, switch OFF). If the BMI160 is not detected it
prints the chip ID failure and retries every second. See troubleshooting.

## 8. Soak test (30 minutes)

A soak test checks that nothing falls over during a long recording. It
catches loose wires, a bad Bluetooth setup, a flaky battery, or a noisy power
supply.

1. Fully charge the LiPo (cable in the C3, **switch OFF**, until the
   TP4056's LED shows "full"). Unplug. Put the sensor on the desk, **not
   moving**.
2. Switch ON. Put the phone next to it (within 1 m) with the screen on and
   Chrome in the foreground. Turn off battery saver. The record page keeps the
   screen awake by itself.
3. On the record page, note the **Battery** reading (mV). Tap **Start
   recording**.
4. Leave for **30 minutes** without touching the sensor, the desk or the
   phone.
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
| `rate` on the run line | **400 ± 4 Hz** (396 to 404), and the line must **not** say `nominal` (that means the rate was not measured from the sensor's status data) |
| `gaps:` | **0** (and 0 samples dropped) |
| `duration:` | about 1800 s (30 min) |
| `samples:` | about 720,000 |
| `quietest 1 s:` `acc noise` | about **2 to 4 mg**; above about **8 mg** means extra noise |
| `quietest 1 s:` `gyro noise` | about **0.1 °/s**; above about **0.3 °/s** means extra noise |
| `quietest 1 s:` `\|acc\|` | **0.95 to 1.05 g** (gravity; the sensor's own offset accounts for the spread) |

The noise figures are my estimates from the BMI160 datasheet, so your first
numbers become the baseline. If the noise is clearly above them, the boost or
buck is probably leaking switching noise into the sensor: move them away from
the BMI160, or solder a 10 µF capacitor between the BMI160's 3V3 and GND
pins, and repeat.

Also write down the **battery drop** (mV at start minus mV at end). A healthy
cell should lose only a small amount in 30 minutes (a few tens of mV is my
expectation; the boost and buck cost a bit more than a plain regulator would),
so your number becomes the baseline. A drop of 200 mV or more suggests an old
cell or extra current draw. Keep the numbers; they tell you the real runtime.

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
2. Put the C3, charger board, boost, buck and battery in the box, with the
   boost and buck at the **far end from the sensor** if you can. Keep wires
   short and tidy, and make sure the **slide switch** and the **C3's USB-C**
   are reachable through cutouts (small holes you make with a hobby knife or
   rotary tool). The C3's port is the only one you need: it charges and
   flashes. Cover bare solder joints and the underside of the boards with
   Kapton tape so nothing shorts against anything else inside the box.
3. **Label the box** next to the C3's port: "**USB: switch OFF first**".
   Leave the TP4056's own port covered (tape), not reachable from outside.

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

> **RULE: Slide switch OFF whenever a USB cable is in the C3. Charge and
> flash through the C3's USB-C only.**

| Symptom | Likely cause and fix |
|---|---|
| Nothing happens when switched on (USB unplugged), no LED | Battery flat or not connected: check OUT+ to OUT− (0 V on a new board until USB has been plugged in once). Then follow the chain with the switch ON: boost IN+ (battery voltage), boost OUT+ (about 5 V), buck OUT+ and C3 3V3 (about 3.3 V). The first place the voltage is missing is the fault. |
| C3 3V3 reads well below 3.2 V on battery | Boost not giving 5 V under load (check its output with the device running), or a weak or nearly empty battery. |
| No charging LED with the cable in the C3 | IN+ wire not on the C3's 5V pad, or IN− not on GND. Check TP4056 IN+ to IN− reads about 5 V with the cable in. A charge-only cable still charges, so if this fails with every cable, it is the wiring. |
| C3 not listed in the flash dialog | Charge-only USB cable. Use another. Or hold BOOT while plugging in. Also check the slide switch is OFF. |
| LED blinks fast | Low battery: the reading is below 3500 mV. Low battery overrides "solid when connected", so it blinks fast even while connected. Charge (cable in the C3, switch OFF). |
| LED blinks slowly, phone cannot find "PowerKettle" | Bluetooth and location off on the phone; another phone is already connected; or you are not using Chrome. |
| Record page says no Web Bluetooth | Use Chrome on Android (not Firefox, not an in-app browser). |
| Rate far from 400 Hz, or no samples | Sensor not talking. See next row. |
| Serial log says chip ID failure (retries every second) | BMI160 not found. Check SDA/SCL are not swapped, 3V3 and GND are connected, **SAO is low** (address 0x68), **CS is high**, and no solder bridges. |
| Battery mV reads 0 | Divider not connected to OUT+ or GPIO4, or a broken resistor joint. |
| Battery mV reads about double or half | Check both resistors are really 100 kΩ and the midpoint goes to GPIO4. |
| Charging LED never goes out, or the cell or TP4056 chip gets hot | Unplug. Check the 4.7 kΩ charge-current resistor was really fitted (pin 2 to ground reads about 4.7 kΩ), and that the switch was OFF while charging (with it ON, the charger never sees the battery as full). |
| Soak test noise well above the expected values | Switching noise from the boost or buck: move them away from the sensor, add a 10 µF capacitor across the BMI160's 3V3 and GND, or check for a loose ground wire. |
| Gaps in a recording | Phone too far, Chrome in the background, battery saver on. |
| Device resets or disconnects when moved | Loose wire or solder joint. Strain-relieve the wires. |
