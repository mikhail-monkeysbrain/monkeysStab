#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "======================================================================"
echo "monkeysStab — ASTRA REALTIME CONTROL"
echo "======================================================================"
echo "Shadow-only: OPTICAL_FLOW в FC этим тестом НЕ отправляется."
echo "OV9281 + FC ATTITUDE + TF-Luna DISTANCE_SENSOR."
echo "Алгоритм: Astra fast 320x240 LK+FB + homography gates + plane/range metric."
echo "======================================================================"

python3 - <<'PY'
try:
    import cv2, numpy, pymavlink
except Exception as e:
    raise SystemExit("ОШИБКА Python dependency: "+repr(e))
print("DEPENDENCIES: OK")
PY

exec python3 "$ROOT/tools/astra_realtime_control.py"
