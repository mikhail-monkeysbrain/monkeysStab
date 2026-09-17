#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
DATA_ROOT="${MONKEYS_DATA_ROOT:-$HOME/monkeysStab_datasets}"
DATASET_DIR="${1:-$DATA_ROOT/${STAMP}_ASTRA_BLIND_AB_RAW}"
mkdir -p "$DATASET_DIR"

export MONKEYS_LOCAL_GUI=0
export MONKEYS_RETURN_CLI=0
export MONKEYS_BLIND4_CLI=1
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
protocol=A(SPACE) -> B(SPACE) -> automatic stop -> literal Astra RAW replay; GT deliberately not entered
camera=OV9281 MJPEG; original JPEG payloads stored in frames.mjpgbin
fc_remote_dataflash=$MONKEYS_REMOTE_LOG
surface=$MONKEYS_DATASET_SURFACE
NOTE=Blind evidence collection. Do not enter or encode physical ground truth before Astra replay result is fixed.
EOF

echo "======================================================================"
echo "ASTRA — ЧИСТЫЙ СЛЕПОЙ A -> B"
echo "======================================================================"
echo "GT НЕ ВВОДИТЬ. После B система сама остановит запись и посчитает Astra."
echo "Dataset: $DATASET_DIR"
echo "======================================================================"

# run.sh uses the same local MAVLink endpoint as the web service. Start it here
# so this dedicated capture does not depend on the web UI.
if ! python3 - <<'PY'
import socket,sys
try:
    s=socket.create_connection(("127.0.0.1",5760),timeout=.35); s.close(); sys.exit(0)
except OSError:
    sys.exit(1)
PY
then
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
    tail -40 "$ROUTER_LOG" >&2 || true
    exit 3
  fi
fi

CAPTURE_START_NS="$(date +%s%N)"
set +e
python3 "$ROOT/tools/astra_blind_ab_console.py"
CAPTURE_RC=$?
set -e

# Only a run created by this capture may be attached to this dataset.
python3 - "$DATASET_DIR" "$CAPTURE_START_NS" <<'PY'
from pathlib import Path
import csv,sys
d=Path(sys.argv[1]); start_ns=int(sys.argv[2])
required=["frames.csv","frames.mjpgbin","capture_manifest.txt"]
ok=True
for n in required:
    p=d/n
    if not (p.exists() and p.stat().st_size>0): ok=False

runs=[]
for p in Path.home().joinpath("monkeysStab_runs").glob("*_OPTICAL_FLOW/optical_flow_mavlink.csv"):
    try:
        if p.stat().st_mtime_ns >= start_ns: runs.append(p)
    except OSError:
        pass
runs.sort(key=lambda p:p.stat().st_mtime_ns,reverse=True)
if not runs:
    print("ОШИБКА: текущий optical_flow_mavlink.csv не найден")
    sys.exit(4)

dst=d/"optical_flow_mavlink.csv"
dst.write_bytes(runs[0].read_bytes())
with dst.open(newline="") as f:
    rows=list(csv.DictReader(f))
ev=[(r.get("frame"),r.get("return_event")) for r in rows if r.get("return_event") not in (None,"","0")]
if len(ev)<2 or ev[0][1]!="11" or ev[1][1]!="12":
    print("ОШИБКА: не зафиксированы обе границы A/B; events=",ev)
    sys.exit(4)
if not ok:
    print("ОШИБКА: RAW dataset неполный")
    sys.exit(4)
print(f"Границы RAW подтверждены: A frame={ev[0][0]}, B frame={ev[1][0]}")
PY

echo
echo "ASTRA СЧИТАЕТ A -> B ПО RAW..."

BUILD_LOG="$DATASET_DIR/astra_replay_build.log"
REPLAY_LOG="$DATASET_DIR/astra_replay.log"
if ! g++ -std=c++17 -O2 \
  $(pkg-config --cflags opencv4) -I"$ROOT/src" \
  "$ROOT/tools/astra_reference_raw_replay.cpp" \
  -o /tmp/astra_reference_raw_replay \
  $(pkg-config --libs opencv4) >"$BUILD_LOG" 2>&1; then
  echo "ОШИБКА: Astra replay не собрался. Лог: $BUILD_LOG" >&2
  exit 5
fi

if ! /tmp/astra_reference_raw_replay "$DATASET_DIR" >"$REPLAY_LOG" 2>&1; then
  echo "ОШИБКА: Astra replay завершился с ошибкой. Лог: $REPLAY_LOG" >&2
  tail -20 "$REPLAY_LOG" >&2
  exit 6
fi

RESULT_LINE="$(grep -E '^LEG 11->12 ' "$REPLAY_LOG" | tail -1 || true)"
if [[ -z "$RESULT_LINE" ]]; then
  echo "ОШИБКА: Astra не выдала LEG 11->12. Лог: $REPLAY_LOG" >&2
  exit 7
fi

python3 - "$RESULT_LINE" <<'PY'
import re,sys
s=sys.argv[1]
m=re.search(r'local X/Y=\(([-+0-9.eE]+),\s*([-+0-9.eE]+)\) mm magnitude=([-+0-9.eE]+) mm coverage=([-+0-9.eE]+)%',s)
if not m:
    print("ASTRA RAW RESULT:",s)
    raise SystemExit(0)
x,y,mag,cov=map(float,m.groups())
print("\n======================================================================")
print("ASTRA BLIND RESULT — ЗАФИКСИРОВАН ДО GT")
print("======================================================================")
print(f"A -> B = {mag:.3f} mm")
print(f"local X/Y = ({x:+.3f}, {y:+.3f}) mm")
print(f"coverage = {cov:.3f}%")
print("======================================================================")
print("Теперь можно раскрыть физически измеренный GT A -> B.")
PY

echo "Полный диагностический лог сохранён без вывода на экран:"
echo "  $DATASET_DIR/capture_process.log"
echo "  $REPLAY_LOG"
