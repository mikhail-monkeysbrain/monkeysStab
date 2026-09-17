#!/usr/bin/env python3
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "tools" / "web_service.py"
s = p.read_text(encoding="utf-8")

old = '''        if rn is not None and re is not None:\n            # Current cumulative raw values = previous zero + current relative values.\n            _raw_zero["n"]=(_raw_zero["n"] or 0.0)+float(rn)/1000.0\n            _raw_zero["e"]=(_raw_zero["e"] or 0.0)+float(re)/1000.0\n        cur["x_mm"]=cur["y_mm"]=cur["z_mm"]=0.0\n'''

new = '''        if rn is not None and re is not None:\n            # Current cumulative raw values = previous zero + current relative values.\n            _raw_zero["n"]=(_raw_zero["n"] or 0.0)+float(rn)/1000.0\n            _raw_zero["e"]=(_raw_zero["e"] or 0.0)+float(re)/1000.0\n\n        # Durable forensic marker for Web HOME.  The C++ accumulator is deliberately\n        # NOT reset here: Web HOME remains a presentation/reference zero only.\n        # The frame number lets offline analysis cut the production CSV at the exact\n        # sample used by the Web UI, while raw_abs_n/e preserve the WORKED5 baseline.\n        marker_path=RUN_ROOT/"web_home_events.csv"\n        marker_new=not marker_path.exists() or marker_path.stat().st_size==0\n        with open(marker_path,"a",encoding="utf-8",newline="",buffering=1) as mh:\n            mw=csv.writer(mh)\n            if marker_new:\n                mw.writerow(["wall_time","frame","raw_abs_n_m","raw_abs_e_m","runtime_csv"])\n            mw.writerow([\n                f"{time.time():.6f}",\n                int(cur.get("frame",0) or 0),\n                f"{(_raw_zero['n'] if _raw_zero['n'] is not None else float('nan')):.9f}",\n                f"{(_raw_zero['e'] if _raw_zero['e'] is not None else float('nan')):.9f}",\n                str(_active_csv or ""),\n            ])\n        log_event("INFO",f"HOME/0 Web forensic marker: frame={int(cur.get('frame',0) or 0)} rawN={_raw_zero['n']} rawE={_raw_zero['e']}")\n\n        cur["x_mm"]=cur["y_mm"]=cur["z_mm"]=0.0\n'''

if new in s:
    print("OK: Web HOME forensic marker already present")
    raise SystemExit(0)
if old not in s:
    raise SystemExit("ERROR: expected set_zero() block not found; source was not modified")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")
print("OK: Web HOME forensic marker added to tools/web_service.py")
print("Output: ~/monkeysStab_runs/web_home_events.csv")
print("WORKED5 estimator, C++ accumulator and ArduPilot publisher were not changed.")
