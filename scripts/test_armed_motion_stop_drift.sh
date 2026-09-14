#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "======================================================================"
echo "monkeysStab — ARMED MOTION → STOP DRIFT TEST"
echo "======================================================================"
echo "Цель: определить, где живёт послемоушн-дрейф:"
echo "  1) RAW Optical Flow продолжает движение после физической остановки"
echo "  2) RAW уже остановился, но EKF продолжает ползти"
echo
echo "ТРЕБОВАНИЯ:"
echo "  - ПРОПЕЛЛЕРЫ СНЯТЫ"
echo "  - FC должен быть ARMED до запуска теста"
echo "  - поверхность и высота произвольные, но не меняются после STOP"
echo
echo "ПРОТОКОЛ:"
echo "  PRE: 5 с неподвижно"
echo "  MOVE: активно двигать аппарат 20–30 с; затем ПОЛНОСТЬЮ остановить"
echo "        и сразу нажать Enter"
echo "  STOP: 15 с вообще не трогать аппарат"
echo
echo "Тест завершится сам и напечатает RAW/EKF decay summary."
echo "======================================================================"
echo

read -r -p "FC ARMED, винты сняты, готовы? [y/N]: " ans
case "${ans,,}" in
  y|yes|д|да) ;;
  *) echo "Отмена."; exit 1 ;;
esac

export MONKEYS_REQUIRE_ARMED=1
export MONKEYS_GUIDED_MM=175
export MONKEYS_PRE_STATIC_SEC=5
export MONKEYS_POST_STATIC_SEC=15
export MONKEYS_LOCAL_GUI=0

before="$(find "${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}" -mindepth 1 -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"

set +e
bash "$ROOT/scripts/run.sh"
rc=$?
set -e

after="$(find "${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}" -mindepth 1 -maxdepth 1 -type d -name '*_OPTICAL_FLOW' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"

if [[ -z "$after" || "$after" == "$before" || ! -f "$after/optical_flow_mavlink.csv" ]]; then
  echo "ОШИБКА: новый CSV теста не найден." >&2
  exit "${rc:-2}"
fi

echo
echo "===== MOTION → STOP FORENSIC ====="
python3 "$ROOT/tools/analyze_motion_stop_drift.py" "$after/optical_flow_mavlink.csv"
echo
echo "Dataset: $after"

exit "$rc"
