#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export MONKEYS_FB_SHADOW_MAX_PX="${MONKEYS_FB_SHADOW_MAX_PX:-0.5}"
export MONKEYS_DATASET_ROOT="${MONKEYS_DATASET_ROOT:-$HOME/monkeysStab_ab_datasets}"

echo "======================================================================"
echo "monkeysStab — SAME-FRAME A/B/C: production vs FB vs FB+robust"
echo "======================================================================"
echo "A: текущий production optical flow; именно A отправляется в FC."
echo "B: те же кадры/точки/forward-KLT + backward consistency <= ${MONKEYS_FB_SHADOW_MAX_PX} px + обычный LS."
echo "C: те же B-inliers + adaptive Huber IRLS для translation/scale/yaw."
echo "B/C только записываются в CSV и НИКОГДА не отправляются в FC."
echo
echo "Все три версии считаются на одних и тех же кадрах, поэтому это paired A/B/C."
echo "Дополнительный backward KLT увеличит нагрузку только в этом диагностическом тесте."
echo "======================================================================"
echo

bash "$ROOT/scripts/capture_surface_dataset.sh"

LATEST="$(find "$MONKEYS_DATASET_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
if [[ -z "$LATEST" || ! -f "$LATEST/optical_flow_mavlink.csv" ]]; then
  echo "ОШИБКА: не найден итоговый A/B dataset" >&2
  exit 2
fi

echo
echo "===== A/B/C SUMMARY ====="
python3 "$ROOT/tools/analyze_fb_ab_dataset.py" "$LATEST/optical_flow_mavlink.csv"
echo
echo "A/B dataset: $LATEST"
