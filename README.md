# Black Archive / Navigation continuity

A distinct SIH26168 frontend and reproducible, limited GNSS/INS experiment for The Black Archive. This project does not replace or modify the team's earlier TBA site.

**Public demonstration:** `web/index.html` reads `web/benchmark.json`, an offline result from a real KITTI OXTS sensor stream. This is not a live receiver feed or hosted backend.

## Method

KITTI raw drive `2011_09_26_drive_0005_sync` includes OXTS GPS/IMU-integrated packets sampled around 10 Hz. `backend/fetch_kitti.py` fetches only the OXTS files from the [official KITTI archive](https://www.cvlibs.net/datasets/kitti/raw_data.php) with ZIP HTTP byte ranges (rather than the full 646 MB camera archive). `backend/fusion.py` converts reference lat/lon to a local metric frame. A six-state 2D Kalman filter propagates East/North position, forward speed and yaw with actual OXTS body-frame forward acceleration (`af`, field 14) and yaw rate (`wz`, field 19); it estimates scalar accelerometer and yaw-rate biases and covariance. At 1 Hz it corrects with OXTS position and forward speed. A deterministic test **withholds position and speed updates from t=5 through t<11 seconds**. It reports Euclidean error against OXTS positions and RMSE within that window. The plot is **precomputed measured trace** from these inputs, not a synthesized user track.

### Reproduce

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r backend/requirements.txt
python backend/fetch_kitti.py
python backend/fusion.py
python -m http.server --directory web 8000
```

Output is `web/benchmark.json`. Copy both web/index.html and web/benchmark.json to the repository root before committing a new public version. GitHub Pages hosts the root mirror (index.html and benchmark.json) from main; the canonical frontend source remains in web/. Each push to main automatically rebuilds the public page. No Python service runs on Pages. Every data export is reproducible locally.

### Limitations

This is a proof-of-concept on **one 15.8-second sequence**, not an ML model, 3D ESKF, tested vehicle navigation stack, calibrated GNSS/IMU rig, or receiver outage in the field. The OXTS 'reference' is GPS/INS-derived and is not an independent surveyed ground truth; the same integrated stream supplies both filter inputs and evaluation positions. The short artificial denial test must not be read as 99% accuracy, independent accuracy, or performance under real multipath. A rigorous next stage requires raw independent IMU and GNSS, an independently surveyed reference trajectory, longer withheld drives, alignment/calibration, a full 3D attitude/error-state estimator, test folds, and baselines. The reported error values are specific to this run and this methodology.

## Design & provenance

See [`DESIGN-DIRECTION.md`](DESIGN-DIRECTION.md) for reference deconstruction, composition, typography, interaction and accessibility decisions. The visual route artwork is original SVG; no SpaceX, Mapbox or Linear images, code or assets were copied. The code is newly implemented, not copied from GPL navigation repositories. The OXTS sample is redistributed to reproduce this benchmark; data rights remain with [KITTI](https://www.cvlibs.net/datasets/kitti/). Team: The Black Archive. Source, method and measured-data caveats are shown on the public page.

## Live phone mode (v7)

`live.html` is a browser-local, opt-in location experiment. It asks for GPS and motion/orientation access on tap, maps fresh device GPS fixes, and shows the phone's reported horizontal accuracy. During a gap it extrapolates for at most 20 seconds **only when GPS has supplied a valid speed and travel course**, with rapidly expanding illustrative uncertainty. It cannot derive a bus trajectory from handheld inertial acceleration because phone attitude, gravity, bias and actual vehicle heading are unknown. If speed/course are absent, it freezes the last fix. A 15-second "Test GPS gap" deliberately ignores incoming fixes to make its behavior inspectable; it is not a real outage. Sensor readings are received but do not feed the positional extrapolation. There is no destination search, turn guidance, or offline map tiles. The page depends on internet to load map tiles and runs only in foreground reliably. No coordinates or session history are uploaded or saved by the app; map tile requests to OpenStreetMap reveal the viewed area to the tile provider. Attribution is visible. See `LIVE-MODE-DIRECTION.md`.
