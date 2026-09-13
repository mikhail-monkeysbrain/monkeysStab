#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/third_party/mavlink"

if [[ -f "$DEST/ardupilotmega/mavlink.h" ]]; then
  echo "MAVLink headers: $DEST"
  exit 0
fi

command -v git >/dev/null 2>&1 || {
  echo "ОШИБКА: git не найден." >&2
  exit 2
}

mkdir -p "$ROOT/third_party"
TMP="$ROOT/third_party/.mavlink_tmp"
rm -rf "$TMP"

echo "Загружаю MAVLink C headers..."
git clone --filter=blob:none --no-checkout https://github.com/mavlink/c_library_v2.git "$TMP"
git -C "$TMP" checkout --detach 04fffaab116486ffdf7501c37a3f7393eb7beffc
rm -rf "$DEST"
mv "$TMP" "$DEST"
rm -rf "$DEST/.git"

[[ -f "$DEST/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink ardupilotmega headers не появились после загрузки." >&2
  exit 2
}

echo "MAVLink headers готовы: $DEST"
echo "Версия: 04fffaab116486ffdf7501c37a3f7393eb7beffc"
