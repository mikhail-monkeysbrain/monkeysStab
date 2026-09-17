#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
DATA_ROOT="${MONKEYS_DATA_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="${1:-$DATA_ROOT/${STAMP}_ASTRA_BLIND_AB_RAW}"
mkdir -p "$DATASET_DIR"

export MONKEYS_LOCAL_GUI=0
export MONKEYS_RETURN_CLI=1
export MONKEYS_DATASET_DIR="$DATASET_DIR"
export MONKEYS_DATASET_SURFACE="${MONKEYS_DATASET_SURFACE:-table}"
export MONKEYS_REMOTE_LOG="$DATASET_DIR/fc_dataflash.bin"

cat >"$DATASET_DIR/capture_manifest.txt" <<EOF
capture_type=ASTRA_BLIND_AB_RAW
created_local=$(date --iso-8601=seconds)
git_head=$(git rev-parse HEAD)
dataset_dir=$DATASET_DIR
protocol=A(SPACE) -> B(SPACE) -> stop; GT deliberately not entered
camera=OV9281 MJPEG; original JPEG payloads stored in frames.mjpgbin
fc_remote_dataflash=$MONKEYS_REMOTE_LOG
surface=$MONKEYS_DATASET_SURFACE
NOTE=Blind evidence collection. Do not enter or encode physical ground truth before Astra replay result is fixed.
EOF

echo "======================================================================"
echo "ASTRA BLIND RAW A->B DATASET"
echo "======================================================================"
echo "Dataset: $DATASET_DIR"
echo
echo "Протокол:"
echo "  1. После СИСТЕМА ГОТОВА — аппарат неподвижен в A -> SPACE."
echo "  2. Проведи аппарат руками по столу в B -> остановись -> SPACE."
echo "  3. НЕ вводи физическое расстояние в программу."
echo "  4. После сохранения B останови процесс Ctrl+C."
echo "  5. Физический GT сообщается только после фиксации результата Astra replay."
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
required=["frames.csv","frames.mjpgbin","capture_manifest.txt"]
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
        ok=False
else:
    print("optical_flow_mavlink.csv: NOT FOUND")
    ok=False

p=d/"fc_dataflash.bin"
print("fc_dataflash.bin:", "OK" if p.exists() and p.stat().st_size>0 else "MISSING/EMPTY (non-fatal for Astra replay)")
print("RESULT:", "CAPTURE FILES PRESENT" if ok else "CAPTURE INCOMPLETE")
sys.exit(0 if ok else 4)
PY
CHECK_RC=$?

echo "Dataset: $DATASET_DIR"
# Ctrl+C after B is expected for this blind two-marker protocol. Integrity decides success.
if (( CHECK_RC != 0 )); then
  exit "$CHECK_RC"
fi
exit 0
