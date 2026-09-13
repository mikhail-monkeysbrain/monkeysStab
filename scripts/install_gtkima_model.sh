#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DST="$ROOT/web_assets/GTKimaQuadcopter.glb"

validate_glb() {
  local f="$1"
  [[ -s "$f" ]] || return 1
  python3 - "$f" <<'PY'
from pathlib import Path
import sys,struct
p=Path(sys.argv[1])
b=p.read_bytes()[:12]
if len(b)<12 or b[:4]!=b'glTF':
    raise SystemExit(1)
version=struct.unpack('<I',b[4:8])[0]
if version!=2:
    raise SystemExit(1)
PY
}

if [[ $# -ge 1 ]]; then
  SRC="$1"
  validate_glb "$SRC" || { echo "ОШИБКА: это не GLB 2.0: $SRC" >&2; exit 1; }
  cp -f "$SRC" "$DST"
  echo "GTKima QuadCopter установлен: $DST"
  exit 0
fi

if validate_glb "$DST"; then
  echo "GTKima QuadCopter уже установлен: $DST"
  exit 0
fi

shopt -s nullglob
candidates=(
  "$HOME/Downloads/"*Quad*Copter*.glb
  "$HOME/Downloads/"*quadcopter*.glb
  "$HOME/Downloads/"*Quadcopter*.glb
  "$HOME/Downloads/"*drone*.glb
  "$HOME/Загрузки/"*Quad*Copter*.glb
  "$HOME/Загрузки/"*quadcopter*.glb
  "$HOME/Загрузки/"*drone*.glb
)

best=""
best_mtime=0
for f in "${candidates[@]}"; do
  [[ -f "$f" ]] || continue
  if validate_glb "$f"; then
    mt=$(stat -c %Y "$f" 2>/dev/null || echo 0)
    if (( mt > best_mtime )); then
      best="$f"; best_mtime=$mt
    fi
  fi
done

if [[ -n "$best" ]]; then
  cp -f "$best" "$DST"
  echo "GTKima QuadCopter найден и установлен:"
  echo "  $best"
  echo "  -> $DST"
  exit 0
fi

cat <<'EOF'
GTKima QuadCopter пока не найден.

Скачайте GLB модели:
https://www.fab.com/listings/7e862e56-5134-4f46-8519-a7cb79692074
или:
https://sketchfab.com/3d-models/low-poly-quadcopter-drone-fa0261d9db004dda9d4d3ff9bc985717

Сохраните .glb в ~/Downloads или ~/Загрузки.
После этого снова запустите:
  bash scripts/run_web.sh

Либо установите вручную:
  bash scripts/install_gtkima_model.sh /путь/к/модели.glb
EOF
exit 0
