#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${MONKEYS_FC:-/dev/ttyAMA0}"
BAUD="${MONKEYS_FC_BAUD:-460800}"
SYSID="${MONKEYS_FC_SYSID:-1}"
COMPID="${MONKEYS_FC_COMPID:-1}"
MAVLINK_ROOT="${MAVLINK_ROOT:-/usr/local/include/mavlink/v2.0}"

[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены в $MAVLINK_ROOT" >&2
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

python3 - "$OUT" <<'PY'
import math,sys
vals={}
for line in sys.argv[1].splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    try: vals[k.strip()]=float(v.strip())
    except ValueError: pass
expected={
 "INS_POS1_X":0.0,"INS_POS1_Y":0.0,"INS_POS1_Z":0.0,
 "FLOW_POS_X":0.0625,"FLOW_POS_Y":0.0,"FLOW_POS_Z":0.0500,
 "RNGFND1_POS_X":0.0855,"RNGFND1_POS_Y":0.0,"RNGFND1_POS_Z":0.0550,
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
