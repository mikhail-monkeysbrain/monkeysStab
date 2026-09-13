#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${MONKEYS_FC:-/dev/ttyAMA0}"
BAUD="${MONKEYS_FC_BAUD:-460800}"
SYSID="${MONKEYS_FC_SYSID:-1}"
COMPID="${MONKEYS_FC_COMPID:-1}"
PROFILE="${MONKEYS_FC_PROFILE:-$ROOT/config/fc_profile.json}"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_ROOT="$d"; break; fi
  done
fi
[[ -f "${MAVLINK_ROOT:-}/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены." >&2; exit 2;
}
[[ -f "$PROFILE" ]] || { echo "ОШИБКА: профиль FC не найден: $PROFILE" >&2; exit 2; }

BIN="/tmp/monkeysstab_fc_param_reader"
g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member   -I"$MAVLINK_ROOT" "$ROOT/src/fc_param_batch_checked.cpp" -o "$BIN"

mapfile -t PARAMS < <(python3 - "$PROFILE" <<'PY'
import json,sys
with open(sys.argv[1],"r",encoding="utf-8") as f:
    p=json.load(f)["params"]
for k in p.keys(): print(k)
PY
)

OUT="$("$BIN" "$DEVICE" "$BAUD" "$SYSID" "$COMPID" read "${PARAMS[@]}")"

echo "======================================================================"
echo "monkeysStab — CRITICAL FC PREFLIGHT"
echo "======================================================================"
echo "FC: $DEVICE @ $BAUD"
echo "Параметры только читаются. Ничего не изменяется."
echo
echo "$OUT"
echo

python3 - "$OUT" "$PROFILE" <<'PY'
import json,math,sys
vals={}
for line in sys.argv[1].splitlines():
    if "=" not in line: continue
    k,v=line.split("=",1)
    try: vals[k.strip()]=float(v.strip().split()[0])
    except ValueError: pass
with open(sys.argv[2],"r",encoding="utf-8") as f:
    expected=json.load(f)["params"]

fail=False
for k,e in expected.items():
    if k not in vals:
        print(f"FAIL  {k:<20}: не прочитан")
        fail=True
        continue
    v=vals[k]
    ok=math.isclose(v,float(e),rel_tol=0,abs_tol=max(1e-6,abs(float(e))*1e-5))
    print(f"{'PASS' if ok else 'FAIL'}  {k:<20}= {v:g}" + ("" if ok else f" expected={e:g}"))
    if not ok: fail=True

print()
yaw=int(round(expected.get("EK3_SRC1_YAW",0)))
if yaw==6:
    print("FAIL  ExternalNav yaw запрещён для текущего monkeysStab контура.")
    fail=True
elif yaw==1:
    print("INFO  Yaw source: Compass")
else:
    print("INFO  Yaw source: None")

print("INFO  Horizontal velocity source: OpticalFlow" if int(round(expected.get("EK3_SRC1_VELXY",-1)))==5
      else "INFO  Horizontal velocity source: другое значение")
print("INFO  Z source: RangeFinder" if int(round(expected.get("EK3_SRC1_POSZ",-1)))==2
      else "INFO  Z source: Baro")

print()
if fail:
    print("RESULT: FAIL")
    raise SystemExit(1)
print("RESULT: PASS")
PY
