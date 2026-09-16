#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

cat <<'EOF'
======================================================================
monkeysStab — ASTRA FAST CANDIDATE CONTROL (C++)
======================================================================
Без Python / pymavlink.
Используется существующий C++ MAVLink/TF-Luna/OV9281 контур monkeysStab.

ВАЖНО:
Это realtime-проверка документированного fast candidate Astra:
  ROI + grid GFTT + LK + FB 0.8 px + robust homography.
Это НЕ выдаётся за готовый flight-estimator Astra: в forensic-отчёте такой
estimator не был зафиксирован как готовый.

OPTICAL_FLOW publisher остаётся штатным. Результат ASTRA/FB — shadow only.
======================================================================
EOF

export MONKEYS_LOCAL_GUI=0
export MONKEYS_RETURN_CLI=1
export MONKEYS_FB_SHADOW_MAX_PX=0.8
export MONKEYS_NO_OBS_SHADOW=1

# Documented Astra surface ROI / nominal calibration.
export MONKEYS_FEATURE_ROI="0.10 0.44 0.90 0.96"
export MONKEYS_MAX_FEATURES=500
export MONKEYS_FOCAL_SCALE=1.0

exec bash "$ROOT/scripts/run.sh"
