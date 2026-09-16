#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
DATA_ROOT="${MONKEYS_DATA_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="${1:-$DATA_ROOT/${STAMP}_ASTRA_AB_RAW}"
mkdir -p "$DATASET_DIR"

export MONKEYS_LOCAL_GUI=0
export MONKEYS_RETURN_CLI=1
export MONKEYS_DATASET_DIR="$DATASET_DIR"
export MONKEYS_DATASET_SURFACE="${MONKEYS_DATASET_SURFACE:-table}"
export MONKEYS_REMOTE_LOG="$DATASET_DIR/fc_dataflash.bin"

cat >"$DATASET_DIR/capture_manifest.txt" <<EOF
capture_type=ASTRA_AB_RAW
created_local=$(date --iso-8601=seconds)
git_head=$(git rev-parse HEAD)
dataset_dir=$DATASET_DIR
protocol=A(SPACE) -> B(SPACE) -> GT_mm(ENTER) -> SPACE -> A(SPACE) -> automatic stop
camera=OV9281 MJPEG; original JPEG payloads stored in frames.mjpgbin
fc_remote_dataflash=$MONKEYS_REMOTE_LOG
surface=$MONKEYS_DATASET_SURFACE
NOTE=Do not tune estimator parameters from this capture launcher. This run is evidence collection.
EOF

echo "======================================================================"
echo "ASTRA RAW A/B/A DATASET"
echo "======================================================================"
echo "Dataset: $DATASET_DIR"
echo
echo "Протокол:"
echo "  1. После СИСТЕМА ГОТОВА — аппарат неподвижен в A -> SPACE."
echo "  2. Проведи аппарат руками по столу в B -> остановись -> SPACE."
echo "  3. Измерь физический A->B, введи миллиметры -> ENTER."
echo "  4. По приглашению -> SPACE, затем верни аппарат в физическую A."
echo "  5. Остановись в A -> SPACE. Программа завершится."
echo
echo "Будут записаны:"
echo "  - frames.mjpgbin: каждый исходный JPEG OV9281;"
echo "  - frames.csv: frame + camera/monotonic timestamps + JPEG size;"
echo "  - optical_flow_mavlink.csv: production + датчики/FC диагностика;"
echo "  - fc_dataflash.bin: remote DataFlash FC (если FC backend разрешает);"
echo "  - capture_manifest.txt: версия кода и протокол."
echo "======================================================================"

set +e
bash "$ROOT/scripts/run.sh"
RC=$?
set -e

echo
echo "======================================================================"
echo "POST-CAPTURE INTEGRITY"
echo "======================================================================"
python3 - "$DATASET_DIR" <<'PY'
from pathlib import Path
import csv,sys
d=Path(sys.argv[1])
required=["frames.csv","frames.mjpgbin","fc_dataflash.bin","capture_manifest.txt"]
ok=True
for n in required:
    p=d/n
    sz=p.stat().st_size if p.exists() else 0
    print(f"{n}: {'OK' if sz>0 else 'MISSING/EMPTY'} ({sz} bytes)")
    if sz<=0: ok=False

fcsv=d/"frames.csv"
if fcsv.exists():
    with fcsv.open(newline="") as f:
        n=sum(1 for _ in f)-1
    print("saved camera frames:",max(n,0))
    if n<=0: ok=False

runs=sorted(Path.home().joinpath("monkeysStab_runs").glob("*_OPTICAL_FLOW/optical_flow_mavlink.csv"),
            key=lambda p:p.stat().st_mtime, reverse=True)
if runs:
    src=runs[0]
    dst=d/"optical_flow_mavlink.csv"
    dst.write_bytes(src.read_bytes())
    print("optical_flow_mavlink.csv: COPIED",dst.stat().st_size,"bytes")
    with dst.open(newline="") as f:
        rows=list(csv.DictReader(f))
    ev=[(r.get("frame"),r.get("return_event")) for r in rows if r.get("return_event") not in (None,"","0")]
    print("return events:",ev)
    if len(ev)<2:
        print("WARNING: fewer than 2 persisted return_event markers")
else:
    print("optical_flow_mavlink.csv: NOT FOUND")
    ok=False

print("RESULT:", "CAPTURE FILES PRESENT" if ok else "CAPTURE INCOMPLETE")
sys.exit(0 if ok else 4)
PY
CHECK_RC=$?

echo "Dataset: $DATASET_DIR"
if (( RC != 0 )); then
  echo "run.sh exit code: $RC"
fi
exit $(( RC != 0 ? RC : CHECK_RC ))
