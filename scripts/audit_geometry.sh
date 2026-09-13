#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${MONKEYS_FC:-/dev/ttyAMA0}"
BAUD="${MONKEYS_FC_BAUD:-460800}"
SYSID="${MONKEYS_FC_SYSID:-1}"
COMPID="${MONKEYS_FC_COMPID:-1}"
GEOMETRY_JSON="${MONKEYS_GEOMETRY_JSON:-$ROOT/config/mount_geometry.json}"
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
[[ -f "${MAVLINK_ROOT:-}/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены. Сначала запустите scripts/bootstrap_dependencies.sh или задайте MAVLINK_ROOT." >&2
  exit 2
}

BIN="/tmp/monkeysstab_fc_param_reader"
g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member \
  -I"$MAVLINK_ROOT" "$ROOT/src/fc_param_batch_checked.cpp" -o "$BIN"

OUT="$("$BIN" "$DEVICE" "$BAUD" "$SYSID" "$COMPID" read \
  INS_POS1_X INS_POS1_Y INS_POS1_Z \
  FLOW_POS_X FLOW_POS_Y FLOW_POS_Z \
  RNGFND1_POS_X RNGFND1_POS_Y RNGFND1_POS_Z)"

echo "======================================================================"
echo "monkeysStab — CURRENT MOUNT GEOMETRY"
echo "======================================================================"
echo "$OUT"
echo

python3 - "$OUT" "$GEOMETRY_JSON" <<'PY'
import json,math,sys
vals={}
for line in sys.argv[1].splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    try: vals[k.strip()]=float(v.strip())
    except ValueError: pass
with open(sys.argv[2], "r", encoding="utf-8") as f:
    g=json.load(f)
c=g["camera"]; r=g["rangefinder"]
expected={
 "INS_POS1_X":0.0,"INS_POS1_Y":0.0,"INS_POS1_Z":0.0,
 "FLOW_POS_X":float(c["x"]),"FLOW_POS_Y":float(c["y"]),"FLOW_POS_Z":float(c["z"]),
 "RNGFND1_POS_X":float(r["x"]),"RNGFND1_POS_Y":float(r["y"]),"RNGFND1_POS_Z":float(r["z"]),
}
failed=[]
for k,e in expected.items():
    if k not in vals: failed.append(f"{k}: MISSING"); continue
    if not math.isclose(vals[k],e,rel_tol=0,abs_tol=max(1e-6,abs(e)*1e-5)):
        failed.append(f"{k}: {vals[k]} expected {e}")
if failed:
    print("RESULT: FAIL")
    for s in failed: print("  "+s)
    raise SystemExit(1)
print("RESULT: PASS")
print("Геометрия FC соответствует текущему монтажу OV9281 + TF-Luna.")
PY
