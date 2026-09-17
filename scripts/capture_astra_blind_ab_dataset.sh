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
export MONKEYS_FC="tcp://127.0.0.1:5760"

ROUTER_PID=""
ROUTER_LOG="$DATASET_DIR/mavlink_router.log"
cleanup(){
  if [[ -n "$ROUTER_PID" ]] && kill -0 "$ROUTER_PID" 2>/dev/null; then
    kill -TERM "$ROUTER_PID" 2>/dev/null || true
    wait "$ROUTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

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

# run.sh expects the same local MAVLink TCP endpoint as the web service.
if ! python3 - <<'PY'
import socket,sys
try:
    s=socket.create_connection(("127.0.0.1",5760),timeout=.35); s.close(); sys.exit(0)
except OSError:
    sys.exit(1)
PY
then
  echo "Запускаю MAVLink router /dev/ttyAMA0 -> tcp://127.0.0.1:5760 ..."
  bash "$ROOT/scripts/run_mavlink_wifi.sh" >"$ROUTER_LOG" 2>&1 &
  ROUTER_PID=$!
  ready=0
  for _ in $(seq 1 60); do
    if python3 - <<'PY'
import socket,sys
try:
    s=socket.create_connection(("127.0.0.1",5760),timeout=.2); s.close(); sys.exit(0)
except OSError:
    sys.exit(1)
PY
    then ready=1; break; fi
    if ! kill -0 "$ROUTER_PID" 2>/dev/null; then break; fi
    sleep .1
  done
  if [[ "$ready" != 1 ]]; then
    echo "ОШИБКА: MAVLink router не открыл tcp://127.0.0.1:5760" >&2
    tail -80 "$ROUTER_LOG" >&2 || true
    exit 3
  fi
fi

# Snapshot existing run CSVs. Post-capture may only use a CSV created after this point.
RUN_SNAPSHOT="$DATASET_DIR/.runs_before.txt"
find "$HOME/monkeysStab_runs" -maxdepth 2 -type f -name optical_flow_mavlink.csv -print 2>/dev/null | sort >"$RUN_SNAPSHOT" || true
CAPTURE_START_NS="$(date +%s%N)"

set +e
bash "$ROOT/scripts/run.sh"
RC=$?
set -e

echo
echo "======================================================================"
echo "POST-CAPTURE INTEGRITY"
echo "======================================================================"
python3 - "$DATASET_DIR" "$CAPTURE_START_NS" <<'PY'
from pathlib import Path
import csv,sys
d=Path(sys.argv[1]); start_ns=int(sys.argv[2])
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

runs=[]
for p in Path.home().joinpath("monkeysStab_runs").glob("*_OPTICAL_FLOW/optical_flow_mavlink.csv"):
    try:
        if p.stat().st_mtime_ns >= start_ns:
            runs.append(p)
    except OSError:
        pass
runs.sort(key=lambda p:p.stat().st_mtime_ns,reverse=True)
if runs:
    src=runs[0]
    dst=d/"optical_flow_mavlink.csv"
    dst.write_bytes(src.read_bytes())
    print("optical_flow_mavlink.csv: COPIED FROM",src)
    print("optical_flow_mavlink.csv:",dst.stat().st_size,"bytes")
    with dst.open(newline="") as f:
        rows=list(csv.DictReader(f))
    ev=[(r.get("frame"),r.get("return_event")) for r in rows if r.get("return_event") not in (None,"","0")]
    print("return events:",ev)
    if len(ev)<2:
        print("WARNING: fewer than 2 persisted return_event markers")
        ok=False
else:
    print("optical_flow_mavlink.csv: NO NEW RUN CREATED BY THIS CAPTURE")
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
