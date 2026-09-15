#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "======================================================================"
echo "monkeysStab — DYNAMIC 175 mm PAIRED A/B/C"
echo "======================================================================"
echo "A = production"
echo "B = FB consistency <= 0.5 px + ordinary LS"
echo "C = FB consistency <= 0.5 px + adaptive Huber IRLS"
echo "D отключён, чтобы не тратить CPU в динамическом тесте."
echo
echo "Физический протокол:"
echo "  LEG 1: A -> B ровно 175 мм"
echo "  LEG 2: B -> A ровно 175 мм"
echo "  roll/pitch стараться сохранять; yaw естественный."
echo "  После ПОЛНОЙ остановки на каждом конце нажать Enter."
echo "======================================================================"
echo
echo "Выберите поверхность:"
echo "  1) Дерево"
echo "  2) Кафель"
read -r -p "Номер [1-2]: " sel
case "$sel" in
  1) SURFACE=wood; LABEL=WOOD ;;
  2) SURFACE=tile; LABEL=TILE ;;
  *) echo "ОШИБКА: выберите 1 или 2" >&2; exit 2 ;;
esac

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="${MONKEYS_DYNAMIC_ROOT:-$HOME/monkeysStab_dynamic_ab}"
RUN_DIR="$OUT_ROOT/${STAMP}_${LABEL}_175MM"
mkdir -p "$RUN_DIR"

cp -f "$ROOT/config/runtime.json" "$RUN_DIR/runtime.json"
cp -f "$ROOT/config/mount_geometry.json" "$RUN_DIR/mount_geometry.json"
cp -f "$ROOT/config/ov9281_current_mount.yaml" "$RUN_DIR/ov9281_current_mount.yaml"
cp -f "$ROOT/config/fc_profile.json" "$RUN_DIR/fc_profile.json"
git -C "$ROOT" rev-parse HEAD > "$RUN_DIR/git_head.txt"

cat > "$RUN_DIR/metadata.json" <<EOF
{
  "kind": "dynamic_175mm_paired_abc",
  "surface": "$SURFACE",
  "target_mm": 175.0,
  "legs": 2,
  "leg_1": "A_to_B",
  "leg_2": "B_to_A",
  "fb_max_px": 0.5,
  "D_disabled": true,
  "created_local": "$(date --iso-8601=seconds)"
}
EOF

export MONKEYS_RUN_DIR="$RUN_DIR"
export MONKEYS_DATASET_DIR="$RUN_DIR"
export MONKEYS_DATASET_SURFACE="$SURFACE"
unset MONKEYS_DATASET_DURATION_SEC || true
export MONKEYS_FB_SHADOW_MAX_PX=0.5
export MONKEYS_NO_OBS_SHADOW=1
export MONKEYS_GUIDED_MM=175
export MONKEYS_CONTINUOUS_LEGS=2
export MONKEYS_PRE_STATIC_SEC=3
export MONKEYS_POST_STATIC_SEC=3
export MONKEYS_LOCAL_GUI=0

echo
echo "ДАТАСЕТ: $RUN_DIR"
echo
echo "ВАЖНО:"
echo "  1) Во время СТАТИКА не двигать."
echo "  2) Когда появится 'LEG 1 ДВИГАЙТЕ' — сдвинуть аппарат A -> B РОВНО 175 мм."
echo "  3) Полностью остановить и нажать Enter."
echo "  4) После результата LEG 1 программа попросит Enter ещё раз для LEG 2."
echo "  5) LEG 2: вернуть B -> A РОВНО 175 мм, остановить, Enter."
echo

set +e
bash "$ROOT/scripts/run.sh"
rc=$?
set -e

CSV="$RUN_DIR/optical_flow_mavlink.csv"
if [[ -f "$CSV" ]]; then
  echo
  echo "===== DYNAMIC A/B/C SUMMARY ====="
  python3 "$ROOT/tools/analyze_dynamic_abc.py" "$CSV" 175
  echo
  echo "Dataset: $RUN_DIR"
fi
exit "$rc"
