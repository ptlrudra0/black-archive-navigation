# Phone positioning: ARENA decision, October 1, 2026
Objective: a real phone-local sensor product, not a vehicle benchmark dressed as a mobile model. Preserve the approved satellite, black canvas, warm metal palette, map-first active mode.

Technical families examined: GPS smoothing; constant velocity; compass/step walking; tilt-aligned acceleration; raw double integration; learned vehicle correction; online phone calibration; camera visual odometry; native error-state fusion; map matching; BLE anchors; Wi-Fi fingerprinting; barometer/floor tracking; inertial-only mode; user-aligned fixed mount; GNSS Doppler velocity; zero-velocity constraints; route constraints; uncertainty-first tracker; local replay lab. Rejected sensor-only and vehicle-model transfer as unsafe. Camera/anchors/native routes need resources beyond this browser product. Step models fail on buses. Road snapping can conceal a wrong estimate.

Finalists: A GPS-only conservative filter (reliable fallback); B gated GNSS/inertial fusion with session bias learning (balanced); C native camera/IMU/GNSS odometry (ambitious, cannot deploy via this page). Winner: B with A's safe fallback. Accuracy/safety carry greater weight than novelty. Calibration is an online regularized constant-bias regression against observed GNSS velocity changes, never a pretrained KITTI phone model. It resets each session and cannot claim tested phone accuracy.

UI candidates: automotive HUD; aviation instrument; hiking compass; editorial map; OS sheet; terminal; cinematic satellite; sensor dashboard; full-screen camera; route timeline. Winner: editorial map + OS sheet, retaining cinematic satellite only on overview. Borrow sensor readiness from dashboard into Details; reject instrument clutter and new hero replacement. Refine labels, type rhythm, buttons and failure messages, not decorative cards.

Failure simulations: denied sensors, relative-only heading, tilted compass, stale motion, stale GPS timestamp, impossible speed, accuracy loss, phone jerk, hidden tab, long outage, no initial velocity, reset, no internet. Every prediction remains labeled estimated. No 99% claim or vehicle RMSE on the phone view. Field validation remains necessary.

Sources: W3C orientation-event specification; MDN acceleration, absolute orientation, coordinate frame and geolocation accuracy pages. Full replay tests are in tests/phone-fusion.test.cjs.

V19, October 1: user requested longer gaps. Research/demo limit is now 20 seconds with LOW CONFIDENCE past 8 seconds, visible growing illustrative radius, and eventual hold. The gap test runs 25 seconds. Neither limit nor radius is a field-validated accuracy guarantee. Previously delivered pitch video describes the old 8-second build.
