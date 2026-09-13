#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${MONKEYS_FC:-/dev/ttyAMA0}"
BAUD="${MONKEYS_FC_BAUD:-460800}"
SYSID="${MONKEYS_FC_SYSID:-1}"
COMPID="${MONKEYS_FC_COMPID:-1}"
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

PARAMS=(FLOW_TYPE FLOW_OPTIONS FLOW_ORIENT_YAW FLOW_FXSCALER FLOW_FYSCALER \
        EK3_FLOW_DELAY EK3_FLOW_MAX EK3_SRC1_POSXY EK3_SRC1_VELXY \
        EK3_SRC1_POSZ EK3_SRC1_VELZ EK3_SRC1_YAW)
OUT="$("$BIN" "$DEVICE" "$BAUD" "$SYSID" "$COMPID" read "${PARAMS[@]}")"

echo "======================================================================"
echo "monkeysStab — OPTICAL FLOW PREFLIGHT"
echo "======================================================================"
echo "FC: $DEVICE @ $BAUD"
echo "Параметры только читаются. Ничего не изменяется."
echo
echo "$OUT"
echo

python3 - "$OUT" <<'PY'
import math,sys
expected={
 "FLOW_TYPE":5,"FLOW_OPTIONS":0,"FLOW_ORIENT_YAW":0,
 "FLOW_FXSCALER":0,"FLOW_FYSCALER":0,"EK3_FLOW_DELAY":0,
 "EK3_SRC1_POSXY":0,"EK3_SRC1_VELXY":5,"EK3_SRC1_VELZ":0,"EK3_SRC1_YAW":0,
}
vals={}
for line in sys.argv[1].splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    try: vals[k.strip()]=float(v.strip())
    except ValueError: pass
fail=False
for k,e in expected.items():
    if k not in vals:
        print(f"FAIL  {k:<18}: не прочитан"); fail=True; continue
    if math.isclose(vals[k],float(e),rel_tol=0,abs_tol=max(1e-6,abs(float(e))*1e-5)):
        print(f"PASS  {k:<18}= {vals[k]:g}")
    else:
        print(f"FAIL  {k:<18}= {vals[k]:g} expected={e:g}"); fail=True
if "EK3_SRC1_POSZ" in vals:
    v=vals["EK3_SRC1_POSZ"]
    if v in (1.0,2.0):
        print(f"INFO  EK3_SRC1_POSZ      = {v:g} ({'Baro' if v==1 else 'RangeFinder'})")
    else:
        print(f"FAIL  EK3_SRC1_POSZ      = {v:g} expected 1 or 2"); fail=True
else:
    print("FAIL  EK3_SRC1_POSZ      : не прочитан"); fail=True
if "EK3_FLOW_MAX" in vals:
    print(f"INFO  EK3_FLOW_MAX       = {vals['EK3_FLOW_MAX']:g} rad/s")
print()
if fail:
    print("RESULT: FAIL")
    raise SystemExit(1)
print("RESULT: PASS")
PY
