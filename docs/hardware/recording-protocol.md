# Validation recording protocol

Milestone 1 ends with you building the device and recording a set of labelled
movements. Those recordings are the ground truth used to tune and validate the
analysis in milestone 2 (spec
[section 9.2](../superpowers/specs/2026-10-14-power-kettle-design.md)). The
more carefully they are made and labelled, the better the analysis will be, so
it is worth doing slowly and exactly once.

You need: the finished device on the bell (see
[build-guide.md](build-guide.md)), an Android phone with Chrome, a tape
measure, a table or sturdy surface at a known height, and a flat floor space.
Allow about 45 minutes in total.

## 1. Measure the bell profile (once)

The analysis needs the bell's real dimensions to work out where its centre of
mass is relative to the sensor. Measure each value twice and write down the
average. Use a soft tape measure (a sewing tape is ideal).

| Quantity | How to measure | Spec's value | Your value |
|---|---|---|---|
| Ball circumference | Round the widest part of the ball | 63 cm | |
| Bottom flat diameter | Across the flat base | 12 cm | |
| Front/back flat diameter | Across the circular "32KG" plate | 10 cm | |
| Clearance, top of ball to handle | Gap under the handle's inner top | 9 cm | |
| Handle tube circumference | Round the handle tube | 13 cm | |
| Handle length | Along the handle | 44 cm | |
| Total height | Floor to top of handle, bell upright | about 30 cm | |
| **Device thickness** | Height of the sensor board above the bell face when the box is mounted (face to the sensor board, not the box top) | 1.0 cm (default) | |
| Mass | From the bell's marking, or a scale | 32 kg | |

Save your values in a text file named `bell-profile.txt` next to the
recordings in Drive. Also write down the date and the table height you use
below.

If your numbers differ from the spec's by more than about 5 mm, keep yours;
the spec's are only my own earlier measurements.

## 2. Phone and device setup

1. Charge the device fully (USB cable in the C3, slide switch OFF), unplug, switch on, and note the battery reading.
2. On the phone: Bluetooth and location on, battery saver off, Chrome open,
   screen timeout long. Keep the phone **within 2 to 3 m** of the bell, on a
   chair or table where it is safe from the bell (not on the floor in the
   swing path).
3. Open <https://matias-ceau.github.io/power-kettle/record.html>.
4. Tap **Connect** and choose **PowerKettle** (LED goes solid).

## 3. The rules for every clip

A "clip" is one recording: from **Start recording** to **Stop & save**. Make
several shorter clips (suggested plan below) rather than one huge one: if
something goes wrong, you lose less.

For **every clip**:

1. Tap **Start recording**.
2. Put the bell **upright on the floor, untouched, for 5 seconds**. (Tap the
   `rest` marker just before, so the 5 s rest is labelled.) The analysis uses
   this still period to calibrate the sensor: it works out which way is up and
   how much the gyroscope reads when nothing is moving.
3. Then do the sets below. **Before each set, tap its marker button** on the
   phone (it is stamped with the exact time). Then pick up the bell and lift.
   After the set, put the bell **down on the floor** and tap `rest` (or just
   wait; see rule 5).
4. **Do exactly the number of reps stated.** The analysis will be checked
   against your count. If you lose count, tap a free-text marker such as
   `lost count` and carry on.
5. **Rest at least 10 seconds** (bell on the floor, not touched) between
   sets.
6. Finish with the bell upright on the floor for 5 s, then tap **Stop & save**.
7. If you make a mistake or something odd happens (dropped bell, wrong hand),
   tap a free-text marker describing it right away, e.g. `oops: dropped bell
   rep 4`. Do not delete anything. A labelled mistake is still useful.

Marker buttons on the page: `snatch L`, `snatch R`, `swing 2H`, `swing L`,
`swing R`, `rest`, `overhead hold`, `table lift`, plus a free-text box with an
**Add** button.

The `L` and `R` in `snatch L` / `swing L` mean the hand holding the bell.
`swing 2H` means a two-handed swing.

## 4. The movements

Do these in this order. Suggested split into clips is in section 5.

| # | Marker to tap | What to do | Notes |
|---|---|---|---|
| 1 | Free text: `table lift 74cm` (put your measured height) | **Table lift x5**: lift the bell slowly from the floor onto the table, pause 3 s on the table, lower slowly back to the floor, pause 3 s. Repeat 5 times. | Measure the table height first, from the floor to the table top (the surface the bell base sits on) with a tape. This tests that the measured upward movement matches the tape. Take about 3 s each way, no jerks. You may tap `table lift` too. |
| 2 | `swing 2H` | 10 two-handed swings | Normal swing to chest height, bell parked on the floor at the end. |
| 3 | `swing L` | 10 swings, left hand | |
| 4 | `swing R` | 10 swings, right hand | |
| 5 | `snatch L` | 10 snatches, left hand | Your normal snatch technique, lockout overhead for about 1 s on each rep, then drop and carry on. |
| 6 | `snatch R` | 10 snatches, right hand | |
| 7 | `overhead hold` | **Overhead hold x3**: snatch (or press) the bell overhead and hold it still for 5 s, then lower. Repeat 3 times with a short pause between. | Use the same hand every time and note it, e.g. a free-text marker `overhead hold R`. These help the analysis tell a lockout apart from resting on the floor. |
| 8 | Free text: `snatch R partial rep 6` (set the real rep and hand) | A normal set of 8 snatches with **one deliberate partial rep** (for example, stop the pull at the shoulder or let the bell drop back before lockout) | Write down which rep was partial (here the 6th). The analysis must not count it as a full snatch. |

## 5. Suggested clips

Each clip starts and ends with the 5 s upright rest from section 3.

| Clip | Contents |
|---|---|
| A | Table lifts (step 1) |
| B | Swings: 2H, L, R (steps 2 to 4) |
| C | Snatches L and R (steps 5, 6) |
| D | Overhead holds and the partial-rep set (steps 7, 8) |

Optional: a slow-motion side-view video of a few reps on your phone (propped
on a stack of books, side-on) for comparing paths later. Say the clip name and
tap a free-text marker such as `video start` while filming so you can line the
two up roughly.

## 6. Saving and getting the files off the phone

At the end of each clip tap **Stop & save**. The file is named
`pk-YYYYMMDD-HHMMSS.pkraw` (phone's local time). Check the message under the
buttons: it says "Saved pk-... (N kB)".

What happens next depends on the phone. Android Chrome will most likely just
**download** the file (the share sheet rarely accepts a file type it does not
know). If a share sheet does appear, pick **Drive** and you are done. If the
file downloaded:

1. Open the **Files** app (or Downloads) on the phone.
2. Go to **Downloads** and find the `.pkraw` files.
3. Long-press a file (or select several), tap **Share** or the three-dot menu,
   then **Send to / Share to Drive**, choose the Drive folder `PowerKettle`
   (create it if it is not there) and tap **Save / Upload**.
   Alternative: open the **Google Drive** app, tap **+**, **Upload**, and
   choose the files from Downloads.
4. Check they appear in Drive. Do not delete the phone copies until you have
   checked the files on your computer.

If you never see a download, look in Chrome's three-dot menu, **Downloads**.
If **Stop & save** said "Not saved yet", tap it again; the recording is kept in
the page's memory until it is saved. Do not close the tab until it says
"Saved".

## 7. Getting files to your computer and checking them

1. Download the `.pkraw` files and `bell-profile.txt` from Drive into one
   folder on your computer.
2. Install the analysis tool once:

   ```bash
   pip install git+https://github.com/matias-ceau/power-kettle
   ```

3. For each file run:

   ```bash
   python -m powerkettle info pk-20261020-101500.pkraw
   ```

   It prints the duration, number of runs, number of samples, the estimated
   sample rate, gaps and dropped samples, and every marker with its time.

**What a good clip looks like**

- `runs: 1`. A second run means Bluetooth dropped and the page reconnected
  automatically: every reconnect re-sends start, which begins a new run, and
  the data between the drop and the reconnect is lost.
- Rate close to 400 Hz (about 396 to 404).
- `gaps: 0 (0 samples dropped)`. A few gaps are acceptable but note them; they
  usually mean the phone was too far or Chrome went to the background.
- The markers list matches what you did, in order, with the free-text markers
  showing your table height and the partial-rep note.

Keep a short notes file (`session-notes.txt`) with, for each clip: the file
name, the date, what you did, anything that went wrong, and the starting and
ending battery readings. That is the information that makes the recordings
usable months from now.

## 8. Checklist

- [ ] Bell profile measured, `bell-profile.txt` saved
- [ ] Table height measured
- [ ] Clip A: table lifts x5 (marker with height)
- [ ] Clip B: swing 2H x10, swing L x10, swing R x10
- [ ] Clip C: snatch L x10, snatch R x10
- [ ] Clip D: overhead hold 5 s x3, set with a partial rep (noted)
- [ ] Every clip starts and ends with the bell upright on the floor for 5 s
- [ ] At least 10 s rest between sets
- [ ] All files uploaded to Drive, checked with `python -m powerkettle info`
