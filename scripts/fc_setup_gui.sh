#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_ROOT="$d"; break; fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi
mkdir -p "$ROOT/build"
g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member \
  -I"$MAVLINK_ROOT" "$ROOT/src/fc_param_batch_checked.cpp" -o "$ROOT/build/fc_param_tool"

if ! python3 - <<'PY' >/dev/null 2>&1
import tkinter
PY
then
  echo "ОШИБКА: не установлен python3-tk. Установите: sudo apt install python3-tk" >&2
  exit 2
fi
exec python3 "$ROOT/tools/fc_setup_gui.py"
