# worklog.md

## 2026-09-19 — DELTAR_ROTATION_SHADOW_V1

Branch: `test/worked5-deltar-rotation-shadow`

Base:
- `frozen/worked_5pct_acc_cam_IMU`
- base commit: `4484b031c2d27db3ccffd8484117ac5f42986c81`
- frozen branch is not modified by this experiment.

Goal:
- isolate horizontal false translation caused by body rotation;
- test full three-axis inter-frame attitude compensation from FC ATTITUDE;
- keep compass contribution indirect through ArduPilot EKF yaw;
- separately expose camera lever-arm motion and FC/IMU-center translation.

Implementation:
- production WORKED5 and MAVLink OPTICAL_FLOW paths remain unchanged;
- existing Metric Shadow full R0/R1 ray geometry is reused;
- `metric_odometry_shadow.hpp` now exposes diagnostic-only:
  - camera-center translation after full attitude compensation;
  - lever-arm translation caused by rigid camera offset;
  - final IMU-center translation = camera translation - lever arm;
- new `deltar_rotation_shadow.csv` records every processed interval:
  - frame timestamps and dt;
  - interpolated roll/pitch/yaw at t0 and t1;
  - dRoll/dPitch/dYaw;
  - frozen WORKED5 dN/dE;
  - full-delta-R camera dN/dE;
  - lever-arm dN/dE;
  - final delta-R+lever IMU dN/dE;
  - attitude bracket gaps, pair counts and residual.

Safety / isolation:
- shadow only;
- no values from this diagnostic are sent to ArduPilot;
- no WORKED5 estimator math, focal scale, KLT/RANSAC thresholds, FPS,
  anchor policy, FUSED-V1/V2, or production optical-flow output is changed.

Known timing limitation to measure, not hide:
- ATTITUDE samples are currently keyed by Raspberry Pi MAVLink receive time
  (`sample_ns=recv_ns`), not FC measurement time mapped to camera clock.
- therefore a rotation test must inspect attitude bracket gaps and residual
  timing before any production promotion.

Validation plan:
1. static baseline;
2. yaw in place;
3. roll in place;
4. pitch in place;
5. mixed rotation;
6. compare accumulated endpoint drift for WORKED5 vs delta-R camera vs
   delta-R+lever IMU-center result.
7. do not promote anything to frozen without explicit user approval.


## 2026-09-19 — DELTAR_GYRO_SHADOW_V1

Reason:
- first yaw-in-place run (~89.3 deg total yaw) showed:
  - WORKED5 endpoint drift: 91.239 mm;
  - ATTITUDE full-delta-R camera: 89.136 mm;
  - lever component: 57.021 mm;
  - ATTITUDE delta-R + lever IMU-center: 32.434 mm.
- geometry/lever correction therefore moves strongly in the correct direction,
  but ~32 mm residual remains too large for production.
- ATTITUDE bracket median was 11.983 ms, max 23.862 ms, so receive-time phase
  error is a plausible remaining cause.

Change:
- production remains untouched;
- add diagnostic body-rate integration over each exact camera interval;
- rates are interpolated at t0/t1, integrated piecewise with trapezoidal omega
  and SO(3) composition;
- absolute ATTITUDE at t0 is used only to anchor the local NED orientation;
- R1 for the new arm is R0 * integrated_delta_R;
- the same ray/ground-plane geometry and same lever-arm subtraction are used;
- new gyro fields are appended to `deltar_rotation_shadow.csv`.

Important limitation:
- gyro history is still keyed by RPi MAVLink receive time because no FC->RPi
  clock mapping is promoted yet. This test isolates ATTITUDE endpoint phase
  sensitivity; it does not claim final timestamp correctness.

Frozen branch remains unchanged. Promotion requires explicit user approval.


## 2026-09-19 — build fix after DELTAR_GYRO_SHADOW_V1

Observed on Raspberry Pi smoke build at commit `40000001215e331047b2a43c9f39ab3ee72ac06e`:
- compiler reported stray backslashes in `metric_odometry_shadow.hpp:102`;
- cause: the generated function signature contained literal `\\n` text instead of real source newlines;
- this was a source-generation/commit formatting error, not a delta-R algorithm result.

Fix:
- replace only the malformed `estimateWithRotations(...)` declaration formatting with real newlines;
- no estimator math, gyro integration, thresholds, production output, or frozen branch is changed.

Validation required:
- rerun `scripts/smoke_build.sh` on Raspberry Pi before any rotation test.


## 2026-09-19 — HIGHRES_GYRO_SHADOW_V1

Ground truth correction from yaw test:
- physical motion was rotation about the IMU centre: +90 deg then -90 deg;
- final physical yaw and IMU-centre XY therefore returned to the start;
- previous one-way-yaw interpretation was invalid.
- ATTITUDE/ATTITUDE-rate shadow did not represent the full ~180 deg absolute
  angular travel, so its ~30 mm endpoint must not be treated as validated
  rotation compensation accuracy.

Repository audit before change:
- runtime already receives MAVLink HIGHRES_IMU and uses q.xgyro/ygyro/zgyro for
  the existing IMU DR path;
- HIGHRES_IMU was requested at 50 Hz;
- the previous DELTAR_GYRO shadow did NOT use this stream: it integrated
  ATTITUDE.rollspeed/pitchspeed/yawspeed.

Change:
- keep all production paths unchanged;
- request HIGHRES_IMU at 100 Hz for this test branch;
- capture every HIGHRES_IMU gyro sample into an independent shadow history;
- write raw diagnostic file
  `/home/vio/Desktop/monkeysStab/highres_gyro_shadow_latest.csv`;
- each row records FC q.time_usec, RPi monotonic receive time, and raw
  xgyro/ygyro/zgyro in rad/s.

Purpose of next test:
- repeat +90 deg then -90 deg about IMU centre;
- verify raw signed gyro-z integral returns near zero;
- verify integral of |gyro| is near the expected ~180 deg physical angular
  travel before coupling raw gyro to camera-frame delta-R.

Isolation:
- frozen branch untouched;
- WORKED5, OPTICAL_FLOW, focal scale, KLT/RANSAC, lever geometry and all
  production estimator outputs are unchanged.


## 2026-09-19 — HIGHRES_DELTAR_SHADOW_V1

Validated source test:
- physical maneuver: one-way yaw, approximately +90 deg about the IMU centre;
- HIGHRES_IMU: 3096 samples / 30.950 s;
- median dt 9.996 ms = 100.04 Hz, max accepted dt 11.190 ms;
- static bias [gx,gy,gz] = [+0.000296,-0.000082,+0.000381] rad/s;
- active-window signed integrals: X -0.881 deg, Y -0.153 deg, Z +86.512 deg;
- |Z| path 97.558 deg, 3D path 97.943 deg.
This confirms HIGHRES_IMU sees the intended yaw maneuver independently of
ATTITUDE rate fields. No scale tuning is derived from the nominal 90 deg.

Change:
- add a third delta-R diagnostic arm to deltar_rotation_shadow.csv;
- integrate HIGHRES_IMU xgyro/ygyro/zgyro over each camera interval;
- use the same absolute ATTITUDE R0 only to define the local frame;
- use HIGHRES_IMU delta-R for R1;
- run the same ray/ground geometry and the same camera lever-arm correction;
- log highres camera, lever and IMU-centre increments plus timing diagnostics.
- first A/B deliberately keys HIGHRES samples by RPi receive_ns, while preserving
  FC time_usec in the raw gyro log; no unverified FC->RPi time mapping is added.

Isolation:
- diagnostic only;
- frozen branch untouched;
- production WORKED5 / OPTICAL_FLOW / ArduPilot feed unchanged.


## 2026-09-19 — interactive ΔR / lever-arm visualizer

Added tools/visualize_rotation_shadow.py. It reads the newest
deltar_rotation_shadow.csv (or an explicit file), accumulates WORKED5,
ATTITUDE, ATT-rate and HIGHRES IMU-centre trajectories, and generates a
self-contained interactive HTML canvas. The scene includes a schematic
3D vehicle centered on the IMU, the camera lever, trajectory endpoints,
and explicit HIGHRES CAMERA / LEVER / IMU = CAMERA - LEVER vectors.
No production estimator or frozen branch changes.


## 2026-09-19 — realtime ΔR 3D visualizer

Extended tools/visualize_rotation_shadow.py with --live mode. It serves the
same interactive 3D diagnostic scene over HTTP and follows the newest
deltar_rotation_shadow.csv while runtime is writing it. Browser polls the
diagnostic snapshot at 5 Hz, showing live WORKED5 / ATT / ATT-rate / HIGHRES
trajectories plus HIGHRES CAMERA, LEVER and IMU residual vectors. Diagnostic
only; no production estimator, runtime feed, or frozen branch changes.


## 2026-09-19 — fix realtime visualizer JavaScript

Fixed tools/visualize_rotation_shadow.py live-page generation: the Python
triple-quoted HTML contained escaped JavaScript template-literal delimiters
(\` and \${), which were emitted literally into the browser and caused a
JavaScript parse error. The page shell loaded but the canvas/legend stayed
blank. Live polling logic and estimator data are otherwise unchanged.
No production estimator or frozen branch changes.


## 2026-09-20 — rewrite flight UI movement comparison

Reworked the current Web UI instead of restoring historical commits. Added a
reliable Canvas2D realtime top-down comparison fed by the existing WebSocket:
CAM/WORKED5 (raw_of N/E), IMU DR, FUSED V1 and FC EKF ("ФАКТ"). Each source
gets its own trail, endpoint and live N/E/|XY| readout with automatic mm scale.
IMU and FUSED are locally zeroed at UI start/HOME so all four trajectories share
a visual origin. Existing WebGL scene, controls, camera preview and runtime are
kept. No estimator, MAVLink publisher, ΔR logic, or frozen branch changes.


## 2026-09-20 — explicit XYZ source rows in flight HUD

Added three explicit XYZ rows below the existing FC EKF row: IMU DR, CAM/WORKED5
and FUSED V1. Existing top XYZ labels now explicitly say FC EKF. IMU DR exposes
N/E/D as X/Y/Z for the diagnostic HUD. CAM uses WORKED5 raw N/E; Z is shown as
unavailable because the current WORKED5 optical-flow path does not independently
estimate vertical displacement. FUSED V1 currently exposes N/E only, so its Z is
also shown as unavailable rather than fabricating a value. No estimator/runtime
publisher behavior changed; UI only. Frozen branch untouched.


## 2026-09-20 — HOME resets IMU DR and FUSED display origins

Fixed the Web HOME/zero semantics. Previously /api/zero only advanced FC EKF and
RAW OF baselines; IMU DR and FUSED V1 kept their runtime cumulative coordinates,
so the new explicit rows did not reset. Added server-side IMU N/E/D and FUSED
N/E display baselines. HOME now zeroes FC EKF, CAM/WORKED5, IMU DR and FUSED V1
together, and RC HOME applies the same baselines. The comparison canvas now uses
the server-relative IMU/FUSED coordinates directly instead of adding a second
browser-local zero. This changes presentation/reference origins only; estimator
states and MAVLink output are untouched. Frozen branch untouched.


## 2026-09-20 — live IMU DR stationary-gate diagnostics

Added a read-only IMU DR diagnostic panel to the current flight Web UI after
observing metre-scale IMU-only drift while the airframe was physically static.
The existing runtime already publishes residual local acceleration, integrated
velocity, acceleration magnitude, gyro magnitude, gate booleans, reject counters,
stationary sample count and dt; web_service previously discarded part of these
fields. It now forwards and displays them together with camera-stationary state.
No thresholds, estimator state, ZUPT logic, MAVLink output or frozen code changed.


## 2026-09-20 — expose existing camera-gated IMU ZUPT state

After yaw tests showed the pure IMU DR retaining false velocity after physical
stop, added a separate read-only "IMU+CAM ZUPT" row to the flight UI. This uses
the already-existing imu_camvc_state diagnostics from the runtime: N/E/D,
N/E/D velocity, active flag, stop samples and activation count. It is deliberately
kept separate from pure IMU DR so a yaw test can distinguish "raw inertial DR
keeps false velocity" from "camera-gated ZUPT fails to arrest it". No estimator,
threshold, ZUPT, FUSED, MAVLink or frozen behavior changed.


## 2026-09-20 — IMU DR body-frame accelerometer bias shadow fix

Yaw testing exposed a persistent horizontal acceleration residual after the
airframe stopped at a new yaw. Auditing src/imu_dead_reckoning.hpp found that
startup accelerometer bias was accumulated in NED and then subtracted as a fixed
NED vector forever. That makes a body-fixed sensor bias incorrect after yaw.

Changed the test branch IMU DR calibration to estimate accelerometer bias in
BODY coordinates: during stationary startup, subtract the ideal body-frame
specific-force vector derived from roll/pitch, average the remaining body-fixed
residual, then subtract that body bias from every HIGHRES_IMU acceleration sample
before bodyToNed(). Gravity removal remains in bodyToNed() and the existing
thresholds/ZUPT/integration are unchanged.

This intentionally changes only the experimental IMU DR/FUSED diagnostic path.
WORKED5 production optical flow, MAVLink optical-flow output and frozen branch
remain untouched. Next validation is stationary -> HOME -> ~90 deg yaw around
the IMU centre -> stationary, checking whether post-yaw ACC N/E returns near
zero and whether IMU+CAM ZUPT endpoint error shrinks.
