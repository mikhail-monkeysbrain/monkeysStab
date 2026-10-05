#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SRC="$ROOT/systemd/jtzero-flight.service"
SERVICE_DST="/etc/systemd/system/jtzero-flight.service"

if [[ ! -f "$SERVICE_SRC" ]]; then
  echo "ОШИБКА: не найден $SERVICE_SRC" >&2
  exit 1
fi

echo "Устанавливаю JT-Zero systemd service..."
sudo install -m 0644 "$SERVICE_SRC" "$SERVICE_DST"
sudo systemctl daemon-reload
sudo systemctl enable jtzero-flight.service

echo
echo "ГОТОВО."
echo "Автозапуск включён со следующей загрузки Raspberry Pi."
echo "Текущий runtime этим скриптом НЕ перезапускается."
echo
echo "Проверка:"
echo "  systemctl is-enabled jtzero-flight.service"
echo "  systemctl status jtzero-flight.service --no-pager"
echo
echo "После перезагрузки:"
echo "  journalctl -u jtzero-flight.service -b --no-pager -n 100"
