#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
WEB_TEST_LOG="/tmp/jtzero_lk_stress_web.log"
mkdir -p "$RUN_ROOT"
cleanup(){
  echo
  echo "[cleanup] Останавливаю тестовый Web/runtime..."
  pkill -f 'monkeysstab_optical_flow|tools/web_service.py|scripts/run_system.sh' 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "============================================================"
echo " LK RUNTIME STRESS TEST"
echo " REST 5s -> STRESS 7s -> REST 5s"
echo "============================================================"
echo "[1/6] Запуск Web в фоне..."
: > "$WEB_TEST_LOG"
bash "$ROOT/scripts/run_web.sh" >"$WEB_TEST_LOG" 2>&1 &
WEBPID=$!
echo "      Web PID=$WEBPID"

echo "[2/6] Ожидание реального monkeysstab_optical_flow..."
RTPID=""
for n in $(seq 1 30); do
  RTPID="$(pgrep -n -f 'monkeysstab_optical_flow' 2>/dev/null || true)"
  if [[ -n "$RTPID" ]]; then
    echo "      ✓ runtime PID=$RTPID найден"
    break
  fi
  if ! kill -0 "$WEBPID" 2>/dev/null; then
    echo "      ✗ Web завершился до запуска runtime"
    tail -60 "$WEB_TEST_LOG"
    exit 2
  fi
  printf "      ожидание runtime: %2d/30\r" "$n"
  sleep 1
done
echo
if [[ -z "$RTPID" ]]; then
  echo "      ✗ runtime не появился за 30 секунд"
  tail -80 "$WEB_TEST_LOG"
  exit 3
fi

phase(){
  local title="$1" sec="$2" instruction="$3"
  echo
  echo "============================================================"
  echo " $title — $instruction"
  echo "============================================================"
  for ((i=1;i<=sec;i++)); do
    printf "      ["
    for ((j=1;j<=sec;j++)); do
      if (( j<=i )); then printf "#"; else printf "."; fi
    done
    printf "] %d/%d\n" "$i" "$sec"
    sleep 1
    if ! pgrep -f 'monkeysstab_optical_flow' >/dev/null 2>&1; then
      echo "      ✗ runtime завершился во время теста"
      tail -80 "$WEB_TEST_LOG"
      exit 4
    fi
  done
}

echo "[3/6] Фаза покоя"
phase "ПОКОЙ" 5 "АППАРАТ НЕ ДВИГАТЬ"
echo "[4/6] Стресс-фаза"
phase "ДВИЖЕНИЕ" 7 "РЕЗКИЕ ГОРИЗОНТАЛЬНЫЕ ДВИЖЕНИЯ"
echo "[5/6] Финальный покой"
phase "ПОКОЙ" 5 "СТОП, АППАРАТ НЕ ДВИГАТЬ"

echo
echo "[6/6] Остановка и анализ..."
cleanup
trap - INT TERM EXIT
sleep 2
LATEST="$(find "$RUN_ROOT" -maxdepth 1 -type d -name '*_OPTICAL_FLOW' | sort | tail -1)"
echo "      Latest run: $LATEST"
echo
echo "===== LK_RUNTIME ====="
grep 'LK_RUNTIME' "$RUN_ROOT/web_runtime.log" 2>/dev/null | tail -100 || true
echo
echo "===== FORENSIC CONTEXT ====="
if [[ -n "$LATEST" ]]; then
  grep -hE 'LK_RUNTIME|W5_WINDOW|FPS_FORENSIC|OF frame=' "$LATEST"/* 2>/dev/null | tail -150 || true
fi
echo
echo "============================================================"
echo " ГОТОВО. Пришли вывод от ===== LK_RUNTIME ===== и ниже."
echo "============================================================"
