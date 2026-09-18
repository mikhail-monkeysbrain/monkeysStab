#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
WEB_RUNTIME_LOG="$RUN_ROOT/web_runtime.log"
WEB_TEST_LOG="/tmp/jtzero_lk_stress_web.log"
SESSION_LOG="/tmp/jtzero_lk_stress_session.log"
CPU_CSV="/tmp/jtzero_lk_cpu_freq.csv"
CPU_PID=""
mkdir -p "$RUN_ROOT"

cleanup(){
  [[ -n "${CPU_PID:-}" ]] && kill "$CPU_PID" 2>/dev/null || true
  pkill -f 'monkeysstab_optical_flow|tools/web_service.py|scripts/run_system.sh' 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "============================================================"
echo " LK RUNTIME STRESS TEST V4 + CPU FREQ"
echo " REST 5s -> STRESS 7s -> REST 5s"
echo "============================================================"

echo "[0/8] Убираю старые процессы..."
cleanup
sleep 1
OLD_PIDS="$(pgrep -f 'monkeysstab_optical_flow|tools/web_service.py|scripts/run_system.sh' 2>/dev/null || true)"
if [[ -n "$OLD_PIDS" ]]; then echo "✗ Старые процессы: $OLD_PIDS"; exit 10; fi
[[ -f "$ROOT/tools/sample_cpu_freq.py" ]] || { echo "✗ Нет tools/sample_cpu_freq.py"; exit 15; }

BEFORE_DIRS="$(mktemp)"
find "$RUN_ROOT" -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%f\n' | sort > "$BEFORE_DIRS"
LOG_OFFSET=0
[[ -f "$WEB_RUNTIME_LOG" ]] && LOG_OFFSET="$(stat -c %s "$WEB_RUNTIME_LOG" 2>/dev/null || echo 0)"

echo "[1/8] Запуск Web..."
: > "$WEB_TEST_LOG"
bash "$ROOT/scripts/run_web.sh" >"$WEB_TEST_LOG" 2>&1 &
WEBPID=$!

echo "[2/8] Жду новый runtime..."
RUN_DIR=""; BIN=""; RTPID=""
for n in $(seq 1 60); do
  while IFS= read -r d; do
    [[ -z "$d" ]] && continue
    if ! grep -Fxq "$d" "$BEFORE_DIRS"; then RUN_DIR="$RUN_ROOT/$d"; fi
  done < <(find "$RUN_ROOT" -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%f\n' | sort)
  if [[ -n "$RUN_DIR" ]]; then
    BIN="$RUN_DIR/monkeysstab_optical_flow"
    if [[ -x "$BIN" ]]; then
      RTPID="$(pgrep -n -f "$BIN" 2>/dev/null || true)"
      if [[ -n "$RTPID" && -r "/proc/$RTPID/exe" && "$(readlink -f "/proc/$RTPID/exe" 2>/dev/null)" == "$BIN" ]]; then
        echo "✓ run-dir: $RUN_DIR"; echo "✓ runtime PID: $RTPID"; break
      fi
    fi
  fi
  kill -0 "$WEBPID" 2>/dev/null || { echo "✗ Web завершился"; exit 11; }
  printf "сборка/запуск %d/60\r" "$n"; sleep 1
done
rm -f "$BEFORE_DIRS"; echo
[[ -x "$BIN" && -n "$RTPID" ]] || { echo "✗ Runtime не подтвержден"; exit 12; }
strings "$BIN" | grep -q 'LK_RUNTIME wall_ms=' || { echo "✗ Нет LK_RUNTIME"; exit 13; }

echo "[3/8] Запускаю независимый CPU sampler 50 Hz..."
rm -f "$CPU_CSV"
python3 "$ROOT/tools/sample_cpu_freq.py" --out "$CPU_CSV" --hz 50 &
CPU_PID=$!
sleep 0.2
kill -0 "$CPU_PID" 2>/dev/null || { echo "✗ CPU sampler не запустился"; exit 16; }
echo "✓ CPU sampler PID=$CPU_PID"

phase(){
  local title="$1" sec="$2" instruction="$3"
  echo; echo "===== $title — $instruction ====="
  for ((i=1;i<=sec;i++)); do printf "[%d/%d]\n" "$i" "$sec"; sleep 1; kill -0 "$RTPID" 2>/dev/null || exit 14; done
}

echo "[4/8]"; phase "ПОКОЙ" 5 "НЕ ДВИГАТЬ"
echo "[5/8]"; phase "ДВИЖЕНИЕ" 7 "РЕЗКИЕ ГОРИЗОНТАЛЬНЫЕ ДВИЖЕНИЯ"
echo "[6/8]"; phase "ПОКОЙ" 5 "СТОП"

echo "[7/8] Сохраняю логи..."
kill "$CPU_PID" 2>/dev/null || true; wait "$CPU_PID" 2>/dev/null || true; CPU_PID=""
sleep 0.2
if [[ -f "$WEB_RUNTIME_LOG" ]]; then tail -c +$((LOG_OFFSET+1)) "$WEB_RUNTIME_LOG" > "$SESSION_LOG"; else : > "$SESSION_LOG"; fi
cleanup
trap - INT TERM EXIT
sleep 1

echo "[8/8] Анализ"
echo "===== BUILD ====="; wc -c "$RUN_DIR/build.log" 2>/dev/null || true
echo "binary: $BIN"
echo
echo "===== LK_RUNTIME >20ms ====="
grep 'LK_RUNTIME wall_ms=' "$SESSION_LOG" 2>/dev/null || echo "NO_LK_RUNTIME_GT20"
echo
echo "===== CPU FREQUENCY ====="
python3 - "$CPU_CSV" <<'PY'
import csv,sys,statistics
rows=list(csv.DictReader(open(sys.argv[1])))
print("samples=",len(rows))
for k in rows[0].keys() if rows else []:
    if k=="mono_ns": continue
    v=[int(r[k])/1000 for r in rows if r.get(k)]
    if v: print(f"{k}: min={min(v):.0f} MHz median={statistics.median(v):.0f} MHz max={max(v):.0f} MHz below2000={sum(x<2000 for x in v)}/{len(v)}")
PY
echo
echo "===== LK VERDICT INPUT ====="
python3 - "$SESSION_LOG" "$CPU_CSV" <<'PY'
import re,sys,csv,statistics
text=open(sys.argv[1],errors="replace").read()
lk=[float(x) for x in re.findall(r'LK_RUNTIME wall_ms=([0-9.]+)',text)]
print("lk_gt20_count=",len(lk))
if lk: print(f"lk_gt20_max_ms={max(lk):.3f} median_ms={statistics.median(lk):.3f}")
rows=list(csv.DictReader(open(sys.argv[2])))
vals=[]
for r in rows:
    for k,v in r.items():
        if k!="mono_ns" and v: vals.append(int(v)/1000)
if vals: print(f"cpu_all_samples_min_mhz={min(vals):.0f} median_mhz={statistics.median(vals):.0f} max_mhz={max(vals):.0f}")
print("NOTE: frequency is sampled independently over the whole test; this version does not claim per-LK-event correlation.")
PY
echo
echo "============================================================"
echo "ГОТОВО. Пришли вывод от ===== BUILD ===== и ниже."
echo "============================================================"
