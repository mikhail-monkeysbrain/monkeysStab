#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
DATASET_ROOT="${MONKEYS_DATASET_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="${1:-$DATASET_ROOT/${STAMP}_ASTRA_BLIND4_RAW}"
mkdir -p "$DATASET_DIR"

cat <<EOF
======================================================================
ASTRA BLIND4 RAW DATASET
======================================================================
Dataset: $DATASET_DIR

Цель: четыре независимых blind A->B прохода без ввода GT программе.
GT измеряй физически и сохраняй отдельно; НЕ вводи его в этот capture.

Режимы:
  1) обычный медленный;
  2) быстрее;
  3) ещё быстрее, без намеренного рывка;
  4) естественный, допускаются небольшие roll/pitch/yaw и уход по Y.

БПЛА всё время на столе, не отрывать.
Стол специально не выравнивать: реальная поверхность может иметь наклон
и локальную неплоскостность.
======================================================================
EOF

{
  echo "dataset=$DATASET_DIR"
  echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "git_head=$(git rev-parse HEAD)"
  echo "git_branch=$(git rev-parse --abbrev-ref HEAD)"
  echo "protocol=ASTRA_BLIND4_RAW_V1"
  echo "gt_disclosed_to_capture=false"
  echo "surface=table_not_leveled_not_assumed_planar"
  echo "camera_mount_x_m=0.0625"
  echo "camera_mount_y_m=0"
  echo "camera_mount_z_m=0.05"
  echo "rangefinder_mount_x_m=0.0855"
  echo "rangefinder_mount_y_m=0"
  echo "rangefinder_mount_z_m=0.055"
} > "$DATASET_DIR/capture_manifest.txt"

echo
echo "ВАЖНО:"
echo "Текущий production estimator не является blind-оценкой этого теста."
echo "Этот launcher предназначен для RAW capture; GT в программу не вводить."
echo
echo "На текущем этапе штатный run.sh имеет только один A/B/A CLI-протокол."
echo "Поэтому этот файл пока НЕ запускает capture автоматически, чтобы не записать"
echo "методологически неверный dataset под видом BLIND4."
echo
echo "Нужна поддержка событий A1/B1..A4/B4 в optical_flow_mavlink.cpp."
echo "RESULT: PROTOCOL GUARD — capture не запущен."
exit 4
