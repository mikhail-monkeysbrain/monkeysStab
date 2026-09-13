#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 -u "$ROOT/tools/mavlink_wifi_router.py"   --serial "${MONKEYS_FC_UART:-/dev/ttyAMA0}"   --baud "${MONKEYS_FC_BAUD:-460800}"   --tcp-port "${MONKEYS_FC_TCP_PORT:-5760}"   --udp-port "${MONKEYS_GCS_UDP_PORT:-14550}"   ${MONKEYS_GCS_IP:+--gcs-ip "$MONKEYS_GCS_IP"}
