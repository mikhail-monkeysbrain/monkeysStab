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
