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

PYTHON=""

# Prefer an existing project/venv Python that already has pymavlink.
CANDIDATES=(
  "$ROOT/.venv/bin/python3"
  "$ROOT/venv/bin/python3"
  "$HOME/venv-jtzero-mav/bin/python3"
  "$HOME/.venv-jtzero-mav/bin/python3"
  "/home/vio/venv-jtzero-mav/bin/python3"
  "python3"
)

for p in "${CANDIDATES[@]}"; do
  if [[ "$p" == "python3" ]]; then
    command -v python3 >/dev/null 2>&1 || continue
    P="$(command -v python3)"
  else
    [[ -x "$p" ]] || continue
    P="$p"
  fi

  if "$P" - <<'PY' >/dev/null 2>&1
import cv2
import numpy
import pymavlink
PY
  then
    PYTHON="$P"
    break
  fi
done

# Last-resort discovery: only inspect existing virtualenvs; do not install anything.
if [[ -z "$PYTHON" ]]; then
  while IFS= read -r p; do
    if "$p" - <<'PY' >/dev/null 2>&1
import cv2
import numpy
import pymavlink
PY
    then
      PYTHON="$p"
      break
    fi
  done < <(find "$HOME" -maxdepth 4 -type f \( -path '*/bin/python3' -o -path '*/bin/python' \) 2>/dev/null | sort)
fi

if [[ -z "$PYTHON" ]]; then
  echo "ОШИБКА: не найден существующий Python с cv2 + numpy + pymavlink." >&2
  echo "Ничего через pip/apt автоматически не устанавливалось." >&2
  echo "Покажи вывод:" >&2
  echo "  find ~ -maxdepth 4 -type f \( -path '*/bin/python3' -o -path '*/bin/python' \) -print 2>/dev/null" >&2
  exit 2
fi

echo "PYTHON: $PYTHON"
"$PYTHON" - <<'PY'
import cv2, numpy, pymavlink
print("DEPENDENCIES: OK")
print("OpenCV:", cv2.__version__)
PY

exec "$PYTHON" "$ROOT/tools/astra_realtime_control.py"
