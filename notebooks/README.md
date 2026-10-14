# Notebooks

## `validate_recording.ipynb`: check a recording against what you did

Runs the Power Kettle analysis on one `.pkraw` file and plots the intermediate signals so you can
check them by eye: rotation and tilt with the detected anchors and your markers, bell speed, power
with the concentric phases shaded, a rep table, side-view rep paths, your markers against the
classifier's labels (with agreement in %), and the table-lift height against your tape measure.

### Open it in Google Colab

Click the badge (or paste the URL into a browser):

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/matias-ceau/power-kettle/blob/main/notebooks/validate_recording.ipynb)

```
https://colab.research.google.com/github/matias-ceau/power-kettle/blob/main/notebooks/validate_recording.ipynb
```

The same URL pattern opens any notebook of the repository:
`https://colab.research.google.com/github/matias-ceau/power-kettle/blob/main/notebooks/<name>.ipynb`.
(The notebook must be on the `main` branch of the GitHub repository for this link to work.)

### Use it

1. **Run the first cell.** In Colab it installs the package from GitHub
   (`pip install git+https://github.com/matias-ceau/power-kettle@main`, about 20 s).
2. **Mount Google Drive.** If `PATH` starts with `/content/drive`, the notebook mounts Drive for you:
   Colab asks you to allow access once. You can also click the folder icon on the left, then the
   Drive icon ("Mount Drive"). Your files are then under `/content/drive/MyDrive/`. In the Files
   panel, right-click a `.pkraw` and choose *Copy path* to get its path.
3. **Set `PATH`** in the *Settings* cell, for example
   `PATH = "/content/drive/MyDrive/PowerKettle/pk-20261020-101500.pkraw"`.
   Leave it as `""` to run on a simulated demo recording (true answer: 74 cm table lift, 10 swings,
   8 + 8 snatches), which is a good way to see what a "good" result looks like first.
4. Optional settings: `BELL_OVERRIDES` (your measured bell values from `bell-profile.txt`),
   `T_RANGE` (zoom the plots), `EXPECTED_REPS` (the rep counts you really did, to compare),
   `SAVE_NPZ` (keep the analysed session).
5. **Runtime > Run all**, and read the sections in order. Each starts with a short "what to look for".

Local use (no Colab): `pip install -e .`, `pip install matplotlib jupyter`, then open the notebook
in `notebooks/`. When `powerkettle` is not installed the notebook imports it from the repository.

### What to look for

- **Load:** `runs: 1`, rate about 400 Hz, `gaps: 0`, and a marker list that matches what you tapped.
- **Rotation plot:** one black triangle (low) per rep, a blue diamond (top) between consecutive
  lows for swings, orange lockout bands for snatches, grey bands where the bell stood on the floor.
- **Speed plot:** a hump per rep that returns to zero between reps.
- **Power plot:** one band per rep you did, coloured by the classifier's kind (orange snatch, blue
  swing, grey other).
- **Rep table:** the number of reps per set equals what you counted.
- **Side view:** the reps of a set overlap; the odd one out is usually your partial rep.
- **Markers vs classifier:** agreement near 100 %; the partial rep (`snatch R partial rep 6`) is
  expected to be classified `other`. Rep indices of any disagreement are listed.
- **Table lift:** measured height within about 2 cm of the tape measure (the marker, for example
  `table lift 74cm`, carries your measured height).

### Regenerating the notebook

The notebook is built by a script (cell sources live there, so changes show up as text diffs):

```bash
pip install matplotlib nbformat nbconvert ipykernel
python notebooks/make_validate_notebook.py        # writes notebooks/validate_recording.ipynb
jupyter nbconvert --to notebook --execute notebooks/validate_recording.ipynb \
    --output-dir /tmp/nb-check                    # check it runs top to bottom (demo recording)
```

The committed notebook has no stored outputs; the check above writes its executed copy elsewhere.
