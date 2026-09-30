#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

IFACE="${MONKEYS_AP_IFACE:-wlan0}"
AP_NAME="${MONKEYS_AP_CONNECTION:-JTZero-Flight}"
SSID="${MONKEYS_AP_SSID:-JTZero-Flight}"
PASSWORD="${MONKEYS_AP_PASSWORD:-jtzero2026}"
AP_IP="${MONKEYS_AP_IP:-192.168.2.1}"
PREFIX="${MONKEYS_AP_PREFIX:-24}"

STATE_DIR="${XDG_RUNTIME_DIR:-/tmp}/jtzero-flight-ap"
STATE_FILE="$STATE_DIR/previous_connection"
mkdir -p "$STATE_DIR"

die() {
    echo "ОШИБКА: $*" >&2
    read -r -p "Enter — закрыть окно..." _ || true
    exit 1
}

command -v nmcli >/dev/null 2>&1 || die "nmcli не найден; нужен NetworkManager"
ip link show "$IFACE" >/dev/null 2>&1 || die "Wi-Fi интерфейс $IFACE не найден"

CURRENT="$(nmcli -g GENERAL.CONNECTION device show "$IFACE" 2>/dev/null | head -n1 || true)"
if [[ -z "$CURRENT" || "$CURRENT" == "--" ]]; then
    die "не удалось определить текущее Wi-Fi подключение на $IFACE"
fi

if [[ "$CURRENT" != "$AP_NAME" ]]; then
    printf '%s\n' "$CURRENT" > "$STATE_FILE"
fi

sudo -v

if ! nmcli -t -f NAME connection show | grep -Fxq "$AP_NAME"; then
    sudo nmcli connection add type wifi ifname "$IFACE" con-name "$AP_NAME" ssid "$SSID" >/dev/null
fi

sudo nmcli connection modify "$AP_NAME" \
    connection.autoconnect no \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv4.addresses "$AP_IP/$PREFIX" \
    ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD"

echo "============================================================"
echo " JT-ZERO — ЗАПУСК ПОЛЁТНОЙ Wi-Fi СЕТИ"
echo "============================================================"
echo "SSID   : $SSID"
echo "Пароль : $PASSWORD"
echo "RPi IP : $AP_IP"
echo
echo "Runtime НЕ запускается этим файлом."
echo "После переключения подключите ноутбук к $SSID."
echo "============================================================"

sudo nmcli connection up "$AP_NAME" ifname "$IFACE" >/dev/null

deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
    if ip -4 addr show dev "$IFACE" | grep -Fq "$AP_IP/$PREFIX"; then
        echo
        echo "ГОТОВО: $SSID, RPi=$AP_IP"
        sleep 3
        exit 0
    fi
    sleep 1
done

die "$IFACE не получил $AP_IP/$PREFIX"
