# WORKLOG — WORKED5 input cascade fix

Branch: `test/worked5-input-cascade-fix`  
Base: frozen WORKED5 commit `ed43ecce73904fef37bdb73095d7fab6a76983ed`

## Rules

- This branch is based directly on the frozen validated WORKED5 snapshot.
- `frozen/worked_5pct_acc_cam_IMU` is not modified here.
- Promotion into frozen happens only after explicit user approval of blind-test results.
- Every relevant code commit on this branch must update this file in the same commit.

## 2026-09-19 — failure being fixed

Frozen blind run: `20260919_202320_OPTICAL_FLOW`, GT disclosed after the blind estimate: 475 mm.

The run itself shows the main cascade around frames 1163-1207:
- nominal selected-frame spacing is about 8 ms;
- processing loses CPU headroom and V4L2 queue drops begin;
- selected-frame intervals jump to about 16-28 ms;
- larger inter-frame displacement raises LK/RANSAC cost and reduces geometric coherence;
- tracked points remain high while homography-RANSAC inliers collapse;
- repeated `invalid_reason=5` intervals are rejected;
- frozen anchor policy still advances to the newest decoded frame, therefore rejected displacement is not recovered by WORKED5.

The external 40 mm/s movement detector also cut off a slow tail. That is a separate analysis-boundary error and must not be used to retune WORKED5.

The existing FUSED-V2 realtime shadow already bridged the collapse causally in this run. It remains shadow-only for the first fix so that the input-headroom hypothesis can be tested independently.

## WORKED5_INPUT_GUARD_V1

Change under test:
- camera request: 120 FPS -> 100 FPS;
- read back negotiated FPS via `VIDIOC_G_PARM` and print it at startup;
- WORKED5 estimator source and math remain untouched;
- no focal-scale, KLT, RANSAC, anchor-policy, or FUSED-V2 policy change.

Why 100 FPS:
- 120 FPS gives about 8.33 ms wall-clock budget per frame;
- 100 FPS gives 10 ms;
- this adds CPU headroom while increasing nominal inter-frame displacement only about 20%;
- the intended causal test is whether avoiding the first queue drop prevents the positive-feedback 16-28 ms cascade.

## Validation protocol

Use only the existing console guided runner. For the next blind A->B:
- GT stays hidden until the estimate is fixed;
- capture requested/actual FPS marker;
- inspect queue drops, dt distribution, LK/RANSAC timings, reason5 intervals, WORKED5 and FUSED-V2 shadow;
- do not tune anything after seeing GT.

If reason5 loss still occurs, the next separate commit may promote the already-frozen FUSED-V2 bridge outside the WORKED5 estimator. Do not combine that recovery change with this first FPS-headroom experiment.


## 2026-09-19 — blind validation WORKED5_INPUT_GUARD_V1

Tested branch/commit:
- branch: `test/worked5-input-cascade-fix`
- commit: `c90fae492d39879badd1a12126af955aa9e8c208`
- startup readback: `requested_fps=100 actual_fps=100`

Blind estimate was fixed before GT:
- selected interval: frame 502 -> 841
- duration: 3.380 s
- dN = -369.384 mm
- dE = +94.141 mm
- WORKED5 distance = 381.192 mm

GT disclosed afterward:
- physical distance = 393 mm
- motion description: jerky
- signed error = -11.808 mm
- relative error = -3.005%

Input-path diagnostics from the same run:
- rows: 4090
- mean camera interval = 9.991 ms (~100.09 FPS long-term)
- median dt = 8.044 ms
- p95 dt = 11.996 ms
- max dt = 88.027 ms
- queue drops = 26 total across 26 frames
- invalid reasons: 4087 reason0, 2 reason5, 1 reason6

Interpretation:
- the long reason5/RANSAC-collapse cascade seen in BAD475 was not reproduced;
- metric accuracy returned inside the frozen <=5% target on this blind jerky motion;
- this supports the input-headroom hypothesis: 100 FPS removed enough pressure to prevent the positive-feedback queue-drop/dt/RANSAC cascade in this run;
- one blind run is evidence, not final promotion proof;
- frozen branch remains untouched pending explicit user approval.

Next:
- repeat at least one more blind A->B on the unchanged 100 FPS branch, preferably similarly jerky, before proposing promotion;
- do not retune WORKED5 or enable FUSED-V2 in the same step.


## 2026-09-19 — blind validation #2, WORKED5_INPUT_GUARD_V1

Unchanged test branch:
- branch: `test/worked5-input-cascade-fix`
- HEAD before run: `1cac99c515db296830fb1cdedda751b726e6fe6f`
- code behavior unchanged from `c90fae492d39879badd1a12126af955aa9e8c208`
- startup readback: `requested_fps=100 actual_fps=100`

Input health:
- rows: 2935
- mean dt = 10.0373 ms
- median dt = 8.05 ms
- p95 dt = 11.996 ms
- max dt = 88.03 ms
- queue drops = 30 across 30 frames
- invalid reasons = {0:2935}; no reason5 and no reason6

Blind estimate fixed before GT:
- interval: frame 1122 -> 1497
- duration: 3.964 s
- dN = +443.526 mm
- dE = -101.838 mm
- WORKED5 endpoint distance = 455.067 mm
- accumulated path = 542.925 mm

GT disclosed afterward:
- physical A->B = 465 mm
- motion description: jerky
- signed endpoint error = -9.933 mm
- relative endpoint error = -2.136%

Interpretation:
- second consecutive blind jerky run is inside the <=5% target;
- the 100 FPS input-headroom guard again prevented the reason5 cascade despite some queue drops;
- endpoint accuracy remains good without changing WORKED5 estimator math;
- large path-vs-endpoint difference reflects non-straight motion during the jerky transfer and is not used as the A->B metric;
- evidence now consists of two consecutive blind validations at 100 FPS: 393 mm -> 381.192 mm (-3.005%) and 465 mm -> 455.067 mm (-2.136%).

Frozen branch remains untouched. Promotion still requires explicit user approval.
