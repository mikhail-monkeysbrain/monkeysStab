#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS="$ROOT/web_assets"
mkdir -p "$ASSETS"

# Prefer the selected GTKima low-poly quadcopter if the user has downloaded it.
bash "$ROOT/scripts/install_gtkima_model.sh" || true

fetch() {
  local url="$1" dst="$2"
  [[ -s "$dst" ]] && return 0
  echo "Загружаю web asset: $(basename "$dst")"
  python3 - "$url" "$dst" <<'PY'
import sys,urllib.request,pathlib
url,dst=sys.argv[1],pathlib.Path(sys.argv[2])
tmp=dst.with_suffix(dst.suffix+".tmp")
with urllib.request.urlopen(url,timeout=30) as r, tmp.open("wb") as f:
    while True:
        b=r.read(1024*256)
        if not b: break
        f.write(b)
tmp.replace(dst)
PY
}

set +e
fetch "https://raw.githubusercontent.com/CesiumGS/cesium/main/Apps/SampleData/models/CesiumDrone/CesiumDrone.glb" "$ASSETS/CesiumDrone.glb"
MODEL_RC=$?
fetch "https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js" "$ASSETS/model-viewer.min.js"
VIEWER_RC=$?
fetch "https://unpkg.com/three@0.169.0/build/three.module.js" "$ASSETS/three.module.js"
THREE_RC=$?
fetch "https://unpkg.com/three@0.169.0/examples/jsm/loaders/GLTFLoader.js" "$ASSETS/GLTFLoader.js"
GLTF_RC=$?
fetch "https://unpkg.com/three@0.169.0/examples/jsm/utils/BufferGeometryUtils.js" "$ASSETS/BufferGeometryUtils.js"
BGU_RC=$?

# GLTFLoader and BufferGeometryUtils are published for an npm-style module
# layout.  monkeysStab serves all modules from one /assets directory, so
# rewrite every import to the local flat asset layout.  This runs on every
# launch and also repairs already-cached files on the Raspberry Pi.
python3 - "$ASSETS/GLTFLoader.js" "$ASSETS/BufferGeometryUtils.js" <<'PY'
from pathlib import Path
import sys

gltf=Path(sys.argv[1])
bgu=Path(sys.argv[2])

def localize_three(text: str) -> str:
    text=text.replace("from 'three';", "from './three.module.js';")
    text=text.replace('from "three";', 'from "./three.module.js";')
    return text

if gltf.exists():
    text=localize_three(gltf.read_text(encoding="utf-8"))
    text=text.replace(
        "from '../utils/BufferGeometryUtils.js';",
        "from './BufferGeometryUtils.js';"
    )
    text=text.replace(
        'from "../utils/BufferGeometryUtils.js";',
        'from "./BufferGeometryUtils.js";'
    )
    gltf.write_text(text,encoding="utf-8")

if bgu.exists():
    text=localize_three(bgu.read_text(encoding="utf-8"))
    bgu.write_text(text,encoding="utf-8")

# Fail loudly if a published import remains unresolved.
bad=[]
if gltf.exists():
    text=gltf.read_text(encoding="utf-8")
    for token in ("from 'three'", 'from "three"', "../utils/BufferGeometryUtils.js"):
        if token in text:
            bad.append("GLTFLoader.js: "+token)
if bgu.exists():
    text=bgu.read_text(encoding="utf-8")
    for token in ("from 'three'", 'from "three"'):
        if token in text:
            bad.append("BufferGeometryUtils.js: "+token)
if bad:
    raise SystemExit("Неразрешённые ES imports: "+", ".join(bad))
PY
set -e

if [[ "$MODEL_RC" != 0 || "$VIEWER_RC" != 0 || "$THREE_RC" != 0 || "$GLTF_RC" != 0 || "$BGU_RC" != 0 ]]; then
  echo "ПРЕДУПРЕЖДЕНИЕ: расширенные 3D assets не загружены. Simple/Light режимы продолжат работать." >&2
  exit 0
fi

cat >"$ASSETS/NOTICE.txt" <<'EOF'
GTKimaQuadcopter.glb
Model: Low poly QuadCopter Drone
Author: GTKima
Source: https://sketchfab.com/3d-models/low-poly-quadcopter-drone-fa0261d9db004dda9d4d3ff9bc985717
Fab: https://www.fab.com/listings/7e862e56-5134-4f46-8519-a7cb79692074
License on Sketchfab: CC Attribution
Used as the preferred flight visualizer model when installed locally.

CesiumDrone.glb
Source: CesiumGS/cesium, Apps/SampleData/models/CesiumDrone/CesiumDrone.glb
Repository: https://github.com/CesiumGS/cesium
The Cesium repository is distributed under the Apache License 2.0.

model-viewer
Source: https://github.com/google/model-viewer
Used only for the small model preview thumbnails.

three.js / GLTFLoader
Source: https://github.com/mrdoob/three.js
Pinned web modules: r169
Used for the unified advanced 3D flight scene (grid + trajectory + GLB aircraft).
EOF
echo "Web assets: ГОТОВО"
