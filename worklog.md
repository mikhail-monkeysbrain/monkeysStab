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
