#!/usr/bin/env bash
set -Eeuo pipefail

IFACE="${MONKEYS_AP_IFACE:-wlan0}"
AP_NAME="${MONKEYS_AP_CONNECTION:-JTZero-Flight}"
STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/jtzero-flight-ap"
STATE_FILE="$STATE_DIR/previous_connection"

die() {
    echo "ОШИБКА: $*" >&2
    read -r -p "Enter — закрыть окно..." _ || true
    exit 1
}

[[ -f "$STATE_FILE" ]] || die "не сохранено имя обычной Wi-Fi сети"
OLD_CONNECTION="$(cat "$STATE_FILE")"
[[ -n "$OLD_CONNECTION" ]] || die "сохранённое имя обычной Wi-Fi сети пустое"

sudo -v

echo "============================================================"
echo " JT-ZERO — ВОЗВРАТ В ОБЫЧНУЮ Wi-Fi СЕТЬ"
echo "============================================================"
echo "Закрываю: $AP_NAME"
echo "Возвращаю: $OLD_CONNECTION"
echo "============================================================"

sudo nmcli connection down "$AP_NAME" >/dev/null 2>&1 || true

if sudo nmcli connection up "$OLD_CONNECTION" ifname "$IFACE"; then
    rm -f "$STATE_FILE"
    echo
    echo "ГОТОВО. RPi снова подключён к: $OLD_CONNECTION"
    echo "После переключения ноутбука в ту же сеть используйте hostname: vio"
    sleep 4
else
    die "не удалось поднять обычную сеть '$OLD_CONNECTION'"
fi
