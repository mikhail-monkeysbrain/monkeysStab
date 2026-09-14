#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"

if [[ $# -ge 1 ]]; then
  CSV="$1"
else
  CSV="$(find "$RUN_ROOT" -mindepth 2 -maxdepth 2 -type f -name optical_flow_mavlink.csv -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[[ -n "${CSV:-}" && -f "$CSV" ]] || { echo "ОШИБКА: CSV не найден" >&2; exit 2; }

echo "Анализирую: $CSV"
exec python3 "$ROOT/tools/analyze_runaway_transition.py" "$CSV"
