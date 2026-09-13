#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PORT="${MONKEYS_WEB_PORT:-8080}"
bash "$ROOT/scripts/fetch_web_assets.sh" || true
exec python3 -u "$ROOT/tools/web_service.py" --host 0.0.0.0 --port "$PORT"
