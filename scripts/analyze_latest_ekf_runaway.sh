#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
CSV="${1:-}"
if [[ -z "$CSV" ]]; then
  CSV="$(find "$RUN_ROOT" -mindepth 2 -maxdepth 2 -type f -name optical_flow_mavlink.csv -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[[ -f "$CSV" ]] || { echo "ОШИБКА: CSV не найден: $CSV" >&2; exit 2; }
exec python3 "$ROOT/tools/analyze_ekf_runaway_forensic.py" "$CSV" "${2:-64}" "${3:-81}"
