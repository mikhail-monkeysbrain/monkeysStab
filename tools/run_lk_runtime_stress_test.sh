#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
WEB_RUNTIME_LOG="$RUN_ROOT/web_runtime.log"
WEB_TEST_LOG="/tmp/jtzero_lk_stress_web.log"
SESSION_LOG="/tmp/jtzero_lk_stress_session.log"
mkdir -p "$RUN_ROOT"

cleanup(){
  echo
  echo "[cleanup] Останавливаю тестовый Web/runtime..."
  pkill -f 'monkeysstab_optical_flow|tools/web_service.py|scripts/run_system.sh' 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "============================================================"
echo " LK RUNTIME STRESS TEST V3"
echo " REST 5s -> STRESS 7s -> REST 5s"
echo "============================================================"

echo "[0/7] Убираю старые процессы..."
cleanup
sleep 1
OLD_PIDS="$(pgrep -f 'monkeysstab_optical_flow|tools/web_service.py|scripts/run_system.sh' 2>/dev/null || true)"
if [[ -n "$OLD_PIDS" ]]; then
  echo "      ✗ Не удалось убрать старые процессы: $OLD_PIDS"
  exit 10
fi

BEFORE_DIRS="$(mktemp)"
find "$RUN_ROOT" -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%f\n' | sort > "$BEFORE_DIRS"
LOG_OFFSET=0
[[ -f "$WEB_RUNTIME_LOG" ]] && LOG_OFFSET="$(stat -c %s "$WEB_RUNTIME_LOG" 2>/dev/null || echo 0)"
echo "      web_runtime.log offset=$LOG_OFFSET"

echo "[1/7] Запуск Web в фоне..."
: > "$WEB_TEST_LOG"
bash "$ROOT/scripts/run_web.sh" >"$WEB_TEST_LOG" 2>&1 &
WEBPID=$!
echo "      Web PID=$WEBPID"

echo "[2/7] Жду НОВЫЙ run-dir, binary и runtime PID..."
RUN_DIR=""
BIN=""
RTPID=""
for n in $(seq 1 60); do
  while IFS= read -r d; do
    [[ -z "$d" ]] && continue
    if ! grep -Fxq "$d" "$BEFORE_DIRS"; then RUN_DIR="$RUN_ROOT/$d"; fi
  done < <(find "$RUN_ROOT" -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%f\n' | sort)

  if [[ -n "$RUN_DIR" ]]; then
    BIN="$RUN_DIR/monkeysstab_optical_flow"
    if [[ -x "$BIN" ]]; then
      RTPID="$(pgrep -n -f "$BIN" 2>/dev/null || true)"
      if [[ -n "$RTPID" && -r "/proc/$RTPID/exe" ]]; then
        LIVE_EXE="$(readlink -f "/proc/$RTPID/exe" 2>/dev/null || true)"
        if [[ "$LIVE_EXE" == "$BIN" ]]; then
          echo
          echo "      ✓ новый run-dir: $RUN_DIR"
          echo "      ✓ новый binary:  $BIN"
          echo "      ✓ runtime PID:   $RTPID"
          echo "      ✓ /proc exe:     $LIVE_EXE"
          break
        fi
      fi
    fi
  fi

  if ! kill -0 "$WEBPID" 2>/dev/null; then
    echo
    echo "      ✗ Web завершился до готовности нового runtime"
    [[ -n "$RUN_DIR" && -f "$RUN_DIR/build.log" ]] && { echo "===== BUILD LOG ====="; cat "$RUN_DIR/build.log"; }
    echo "===== WEB LOG ====="; tail -80 "$WEB_TEST_LOG"
    exit 11
  fi
  printf "      сборка/запуск: %2d/60\r" "$n"
  sleep 1
done
rm -f "$BEFORE_DIRS"
echo

if [[ -z "$RUN_DIR" || ! -x "$BIN" || -z "$RTPID" ]]; then
  echo "      ✗ Новый runtime не подтверждён. Движение ЗАПРЕЩЕНО."
  exit 12
fi
if ! strings "$BIN" | grep -q 'LK_RUNTIME wall_ms='; then
  echo "      ✗ Новый binary не содержит LK_RUNTIME wall_ms=. Движение ЗАПРЕЩЕНО."
  exit 13
fi
echo "      ✓ LK_RUNTIME wall_ms= найден в новом binary"
echo "      binary sha256: $(sha256sum "$BIN" | awk '{print $1}')"

phase(){
  local title="$1" sec="$2" instruction="$3"
  echo
  echo "============================================================"
  echo " $title — $instruction"
  echo "============================================================"
  for ((i=1;i<=sec;i++)); do
    printf "      ["
    for ((j=1;j<=sec;j++)); do if (( j<=i )); then printf "#"; else printf "."; fi; done
    printf "] %d/%d\n" "$i" "$sec"
    sleep 1
    if ! kill -0 "$RTPID" 2>/dev/null; then
      echo "      ✗ Runtime PID=$RTPID завершился. Тест остановлен."
      exit 14
    fi
  done
}

echo "[3/7] Фаза покоя"; phase "ПОКОЙ" 5 "АППАРАТ НЕ ДВИГАТЬ"
echo "[4/7] Стресс-фаза"; phase "ДВИЖЕНИЕ" 7 "РЕЗКИЕ ГОРИЗОНТАЛЬНЫЕ ДВИЖЕНИЯ"
echo "[5/7] Финальный покой"; phase "ПОКОЙ" 5 "СТОП, АППАРАТ НЕ ДВИГАТЬ"

echo
echo "[6/7] Снимаю хвост лога и останавливаю runtime..."
sleep 1
if [[ -f "$WEB_RUNTIME_LOG" ]]; then
  tail -c +$((LOG_OFFSET+1)) "$WEB_RUNTIME_LOG" > "$SESSION_LOG"
else
  : > "$SESSION_LOG"
fi
cleanup
trap - INT TERM EXIT
sleep 1

echo "[7/7] Анализ только текущей сессии"
echo "      Run: $RUN_DIR"
echo "      Session log: $SESSION_LOG ($(wc -c < "$SESSION_LOG") bytes)"
echo
echo "===== BUILD ====="
wc -c "$RUN_DIR/build.log" 2>/dev/null || true
echo "binary: $BIN"
echo
echo "===== LK_RUNTIME >20ms ====="
LK_LINES="$(grep 'LK_RUNTIME wall_ms=' "$SESSION_LOG" 2>/dev/null || true)"
if [[ -n "$LK_LINES" ]]; then printf '%s\n' "$LK_LINES"; else echo "NO_LK_RUNTIME_GT20"; fi

echo
echo "===== MAX PRODUCTION LK ====="
python3 - "$SESSION_LOG" <<'PY'
import re,sys
p=sys.argv[1]
best=None
rx=re.compile(r'OF frame=(\d+).*?inliers=(\d+)/(\d+).*?latency=([0-9.]+)ms.*?stage_ms\[F/L/R/P\]=([0-9.]+)/([0-9.]+)/([0-9.]+)/([0-9.]+)')
with open(p,'r',errors='replace') as f:
    for line in f:
        m=rx.search(line)
        if not m: continue
        lk=float(m.group(6))
        if best is None or lk>best[0]: best=(lk,line.rstrip())
if best:
    print(f"MAX_LK_MS={best[0]:.3f}")
    print(best[1])
else:
    print("NO_OF_STAGE_LINES_IN_SESSION")
PY

echo
echo "===== TOP 20 OF BY LK ====="
python3 - "$SESSION_LOG" <<'PY'
import re,sys
rows=[]
rx=re.compile(r'stage_ms\[F/L/R/P\]=[0-9.]+/([0-9.]+)/')
with open(sys.argv[1],'r',errors='replace') as f:
    for line in f:
        m=rx.search(line)
        if m: rows.append((float(m.group(1)),line.rstrip()))
for _,line in sorted(rows,reverse=True)[:20]: print(line)
PY

echo
echo "===== FORENSIC CURRENT SESSION ====="
grep -E 'W5_WINDOW|FPS_FORENSIC|LK_FORENSIC|LK_RUNTIME wall_ms=' "$SESSION_LOG" 2>/dev/null | tail -100 || true
echo
echo "============================================================"
echo " ГОТОВО. Пришли вывод от ===== BUILD ===== и ниже."
echo "============================================================"
