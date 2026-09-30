#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

IFACE="${MONKEYS_AP_IFACE:-wlan0}"
AP_NAME="${MONKEYS_AP_CONNECTION:-JTZero-Flight}"
SSID="${MONKEYS_AP_SSID:-JTZero-Flight}"
PASSWORD="${MONKEYS_AP_PASSWORD:-jtzero2026}"
AP_IP="${MONKEYS_AP_IP:-192.168.50.1}"
PREFIX="${MONKEYS_AP_PREFIX:-24}"
WEB_PORT="${MONKEYS_WEB_PORT:-8080}"
UDP_PORT="${MONKEYS_GCS_UDP_PORT:-14550}"

WEB_PID=""
RESTORED=0

die() {
    echo "ОШИБКА: $*" >&2
    exit 1
}

command -v nmcli >/dev/null 2>&1 || die "nmcli не найден; нужен NetworkManager"
ip link show "$IFACE" >/dev/null 2>&1 || die "Wi-Fi интерфейс $IFACE не найден"

# Remember exactly the Wi-Fi profile that was active before flight mode.
OLD_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$IFACE" 2>/dev/null | head -n1 || true)"
if [[ -z "$OLD_CONNECTION" || "$OLD_CONNECTION" == "--" || "$OLD_CONNECTION" == "$AP_NAME" ]]; then
    die "не удалось определить обычное Wi-Fi подключение на $IFACE"
fi

cleanup() {
    local rc=$?
    trap - INT TERM EXIT

    echo
    echo "============================================================"
    echo " JT-ZERO — ВЫХОД ИЗ ПОЛЁТНОЙ СЕТИ"
    echo "============================================================"

    if [[ -n "$WEB_PID" ]] && kill -0 "$WEB_PID" 2>/dev/null; then
        echo "Останавливаю Web/runtime/router..."
        kill -INT "$WEB_PID" 2>/dev/null || true
        for _ in {1..50}; do
            kill -0 "$WEB_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$WEB_PID" 2>/dev/null; then
            kill -TERM "$WEB_PID" 2>/dev/null || true
        fi
        wait "$WEB_PID" 2>/dev/null || true
    fi

    echo "Закрываю точку доступа $SSID..."
    sudo nmcli connection down "$AP_NAME" >/dev/null 2>&1 || true

    echo "Возвращаю обычную Wi-Fi сеть: $OLD_CONNECTION"
    if sudo nmcli connection up "$OLD_CONNECTION" ifname "$IFACE"; then
        RESTORED=1
        echo
        echo "Обычная сеть восстановлена."
        echo "После переключения ноутбука обратно подключайтесь к RPi как раньше: vio"
    else
        echo
        echo "ВНИМАНИЕ: NetworkManager не смог автоматически поднять $OLD_CONNECTION."
        echo "Профиль не удалён; NetworkManager сможет подключиться к нему обычным способом."
    fi

    exit "$rc"
}
trap cleanup INT TERM EXIT

# Ask for sudo while the operator is still connected through the normal LAN.
sudo -v

echo "============================================================"
echo " JT-ZERO — FLIGHT NETWORK"
echo "============================================================"
echo "Обычная сеть : $OLD_CONNECTION"
echo "Полётная сеть: $SSID"
echo "RPi          : $AP_IP"
echo "Web UI       : http://$AP_IP:$WEB_PORT"
echo "MissionPlan. : UDPCl -> $AP_IP:$UDP_PORT"
echo "============================================================"

# Dedicated AP profile. Nothing is enabled automatically at boot.
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

echo
echo "Сейчас Wi-Fi RPi переключится с '$OLD_CONNECTION' на '$SSID'."
echo "SSH/VNC через старую сеть временно оборвутся."
echo

sudo nmcli connection up "$AP_NAME" ifname "$IFACE" >/dev/null

deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
    if ip -4 addr show dev "$IFACE" | grep -Fq "$AP_IP/$PREFIX"; then
        break
    fi
    sleep 1
done
ip -4 addr show dev "$IFACE" | grep -Fq "$AP_IP/$PREFIX" || die "$IFACE не получил $AP_IP/$PREFIX"

echo
echo "============================================================"
echo " ПОЛЁТНАЯ СЕТЬ ГОТОВА"
echo "============================================================"
echo "1. На ноутбуке: Wi-Fi '$SSID'"
echo "   пароль: $PASSWORD"
echo "2. Mission Planner: UDPCl -> $AP_IP:$UDP_PORT"
echo "3. Web UI: http://$AP_IP:$WEB_PORT"
echo
echo "Для завершения уличного режима: Ctrl+C в этом процессе."
echo "RPi закроет AP и вернётся в '$OLD_CONNECTION'."
echo "============================================================"
echo

export MONKEYS_RAW_UNIFIED_PUBLISH="${MONKEYS_RAW_UNIFIED_PUBLISH:-1}"
export MONKEYS_WEB_PORT="$WEB_PORT"
export MONKEYS_GCS_UDP_PORT="$UDP_PORT"

bash "$ROOT/scripts/run_web.sh" &
WEB_PID=$!
wait "$WEB_PID"
