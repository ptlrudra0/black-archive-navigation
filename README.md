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

The earlier filter benchmark is a proof-of-concept on **one 15.8-second sequence**, not a test of the new ML correction, 3D ESKF, tested vehicle navigation stack, calibrated GNSS/IMU rig, or receiver outage in the field. The OXTS 'reference' is GPS/INS-derived and is not an independent surveyed ground truth; the same integrated stream supplies both filter inputs and evaluation positions. The short artificial denial test must not be read as 99% accuracy, independent accuracy, or performance under real multipath. A rigorous next stage requires raw independent IMU and GNSS, an independently surveyed reference trajectory, longer withheld drives, alignment/calibration, a full 3D attitude/error-state estimator, test folds, and baselines. The reported error values are specific to this run and this methodology.

## Design & provenance

See [`DESIGN-DIRECTION.md`](DESIGN-DIRECTION.md) for reference deconstruction, composition, typography, interaction and accessibility decisions. The visual route artwork is original SVG; no SpaceX, Mapbox or Linear images, code or assets were copied. The code is newly implemented, not copied from GPL navigation repositories. The OXTS sample is redistributed to reproduce this benchmark; data rights remain with [KITTI](https://www.cvlibs.net/datasets/kitti/). Team: The Black Archive. Source, method and measured-data caveats are shown on the public page.

## Live phone sensor lab (v16)
`live.html` runs a local two-axis position/velocity Kalman filter in `phone-fusion.js`. GNSS fixes correct position. Sensor-reported GNSS speed/course, or clearly separated recent accurate fixes, initialize velocity. Gravity-free `devicemotion.acceleration` is rotated into East/North with the W3C Z-X-Y orientation convention. Absolute Earth orientation is required. Safari compass alignment is accepted only near level with reported compass accuracy <=25 degrees. Relative orientation, missing gravity-free acceleration, stale samples, acceleration >4 m/s² or rotation >80 deg/s gate inertial use.

A regularized constant-bias learner fits integrated acceleration minus observed GNSS velocity change over past valid intervals. It activates after 12 qualifying windows, is clamped and resets with every session. This is online calibration, not a pretrained phone-location network, and its performance is unmeasured on real phones. No KITTI weights enter the phone code.

During a GNSS gap, usable inertial data supports a short estimate; otherwise the last known velocity coasts. All estimates stop by 20 seconds. Without reliable velocity, the last position is held. The uncertainty circle is model-based and not certified accuracy. Backgrounding stops integration. Browser sensor behavior, compass bias, handheld movement and GNSS noise remain important limitations. A simulation button withholds callbacks for 25 seconds and demonstrates the hold; it is not satellite loss.

Session data and learned parameters stay in memory. No route is uploaded or saved. OpenStreetMap tile requests reveal the viewed map area; tiles need internet. Open HTTPS in Safari/Chrome, tap Start and allow location plus motion/orientation prompts. Keep the phone stable, near level on Safari, outdoors and foreground. This is not turn guidance or a safety-critical navigation tool.

`node tests/phone-fusion.test.cjs` checks the math and failure cases using synthetic inputs. It does not establish real-device accuracy. Field evaluation against an independent reference is still required.

## Learned correction experiment (new)

`backend/ml_benchmark.py` runs an offline, drive-held-out ridge regression experiment on 16 real synchronized KITTI OXTS drives (2011-09-26). It predicts local position error from initial forward speed and the inertial acceleration/yaw-rate stream through each instant of an artificially hidden six-second position window. An open-loop propagation on the same windows is the paired baseline. We use nested leave-one-entire-drive-out validation to choose regularization without peeking at the held-out drive. Output: `web/ml_benchmark.json` (mirrored to root for Pages).

To reproduce, download OXTS-only files for the drive IDs in `ml_benchmark.json` using the official KITTI synchronized ZIP archives, into `backend/data/ml/<drive-id>/data/*.txt` and `timestamps.txt`. Then `pip install -r backend/requirements.txt && python backend/ml_benchmark.py`. Local training data are deliberately untracked. KITTI license and data source: https://www.cvlibs.net/datasets/kitti/raw_data.php . This is not a live ML phone feature or real GNSS outage. The earlier 1.371 m Kalman-filter figure is a different one-clip setup and must not be presented as the paired baseline.

## v17 field-test fixes
Fresh GNSS follows directly when velocity is unavailable, rather than covariance collapsing around the first fix. Missing speed/course walking velocity uses a separated anchor across up to 20 seconds, not adjacent one-second displacements. Coarse location is shown with its reported uncertainty but never propagated. Error codes, denied permissions and callback counts are visible in Details; errors open Details automatically. Cached samples older than 15 seconds are rejected with a reason. Asset query versions force a fresh v17 script load. Physical device failure still needs a user screenshot/phone-browser report.

V19, October 1: user requested longer gaps. Research/demo limit is now 20 seconds with LOW CONFIDENCE past 8 seconds, visible growing illustrative radius, and eventual hold. The gap test runs 25 seconds. Neither limit nor radius is a field-validated accuracy guarantee. Previously delivered pitch video describes the old 8-second build.

V20: level-compass anchor aligns Safari relative yaw for up to 60s, full Z-X-Y rotation handles subsequent tilt. This assumes browser yaw-frame stability and needs device validation. Bias regression regularization reduced from 10 to 2 after 12 qualifying windows. Controlled synthetic benchmark is phone-bias-benchmark.cjs; exact GNSS, 60s calibration, 15s gap. Not real-phone accuracy.

V21: live screen shows phone-reported GPS radius vs illustrative model radius, not true error. First-order causal 120ms horizontal acceleration smoothing and 250ms settling after acceleration/rotation rejection address constructed vibration/shock cases. phone-road-benchmark.cjs is a deterministic synthetic vehicle disturbance test, not measured India-road performance.
