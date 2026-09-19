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
