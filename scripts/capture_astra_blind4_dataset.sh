#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
STAMP="$(date +%Y%m%d_%H%M%S)"
DATASET_ROOT="${MONKEYS_DATASET_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="${1:-$DATASET_ROOT/${STAMP}_ASTRA_BLIND4_RAW}"
RUN_DIR="$HOME/monkeysStab_runs/${STAMP}_ASTRA_BLIND4"
mkdir -p "$DATASET_DIR" "$RUN_DIR"
cat <<EOF
======================================================================
ASTRA BLIND4 RAW DATASET
======================================================================
Dataset: $DATASET_DIR
SPACE: A1, B1, A2, B2, A3, B3, A4, B4.
GT в программу НЕ вводить. Измеряй GT1..GT4 и записывай отдельно.
БПЛА не отрывать от стола. Стол не считается горизонтальным/идеально плоским.
После B4 запись завершится автоматически.
======================================================================
EOF
{
 echo "dataset=$DATASET_DIR"
 echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
 echo "git_head=$(git rev-parse HEAD)"
 echo "git_branch=$(git rev-parse --abbrev-ref HEAD)"
 echo "protocol=ASTRA_BLIND4_RAW_V1"
 echo "event_codes=11:A1,12:B1,13:A2,14:B2,15:A3,16:B3,17:A4,18:B4"
 echo "gt_disclosed_to_capture=false"
 echo "surface=table_not_leveled_not_assumed_planar"
 echo "camera_mount_x_m=0.0625"
 echo "camera_mount_y_m=0"
 echo "camera_mount_z_m=0.05"
 echo "rangefinder_mount_x_m=0.0855"
 echo "rangefinder_mount_y_m=0"
 echo "rangefinder_mount_z_m=0.055"
} > "$DATASET_DIR/capture_manifest.txt"
export MONKEYS_LOCAL_GUI=0
export MONKEYS_BLIND4_CLI=1
export MONKEYS_RETURN_CLI=0
export MONKEYS_RUN_DIR="$RUN_DIR"
export MONKEYS_DATASET_DIR="$DATASET_DIR"
export MONKEYS_DATASET_SURFACE="table_unleveled_nonideal"
export MONKEYS_DATASET_DURATION_SEC=0
export MONKEYS_REMOTE_LOG="$DATASET_DIR/fc_dataflash.bin"
set +e
bash "$ROOT/scripts/run.sh"
RC=$?
set -e
CSV="$RUN_DIR/optical_flow_mavlink.csv"
[[ -f "$CSV" ]] && cp -f "$CSV" "$DATASET_DIR/optical_flow_mavlink.csv"
echo
echo "===== POST-CAPTURE BLIND4 INTEGRITY ====="
for f in frames.csv frames.mjpgbin optical_flow_mavlink.csv capture_manifest.txt; do
 [[ -s "$DATASET_DIR/$f" ]] && echo "$f: OK ($(stat -c%s "$DATASET_DIR/$f") bytes)" || echo "$f: MISSING/EMPTY"
done
[[ -s "$DATASET_DIR/fc_dataflash.bin" ]] && echo "fc_dataflash.bin: OK ($(stat -c%s "$DATASET_DIR/fc_dataflash.bin") bytes)" || echo "fc_dataflash.bin: MISSING/EMPTY"
if [[ -s "$DATASET_DIR/optical_flow_mavlink.csv" ]]; then
python3 - "$DATASET_DIR/optical_flow_mavlink.csv" <<'PY'
import csv,sys
with open(sys.argv[1],newline="") as f: r=list(csv.DictReader(f))
ev=[(x.get("frame"),x.get("return_event")) for x in r if x.get("return_event") not in ("","0",None)]
print("events:",ev)
got=[e for _,e in ev]
want=["11","12","13","14","15","16","17","18"]
print("event_sequence:","PASS" if got==want else "FAIL",got)
PY
fi
echo "Dataset: $DATASET_DIR"
exit "$RC"
