#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS="$ROOT/web_assets"
mkdir -p "$ASSETS"

# Advanced 3D mode was removed. Clean all legacy Three.js / GLB assets that
# may have been downloaded by older versions of monkeysStab.
rm -f   "$ASSETS/GTKimaQuadcopter.glb"   "$ASSETS/CesiumDrone.glb"   "$ASSETS/model-viewer.min.js"   "$ASSETS/three.module.js"   "$ASSETS/GLTFLoader.js"   "$ASSETS/BufferGeometryUtils.js"   "$ASSETS/NOTICE.txt"

echo "Web assets: ГОТОВО (расширенный 3D удалён)"
