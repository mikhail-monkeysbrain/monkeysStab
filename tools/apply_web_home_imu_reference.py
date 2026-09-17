#!/usr/bin/env python3
from pathlib import Path
p=Path('tools/web_service.py'); s=p.read_text()

# Add an IMU display reference. This fixes Web HOME immediately without pretending
# that the browser can reset the C++ estimator through the one-way telemetry UDP path.
s=s.replace('''    raw_rel_n=(raw_n-rzn) if raw_n is not None and rzn is not None else None
    raw_rel_e=(raw_e-rze) if raw_e is not None and rze is not None else None
    ekf_rel_x=x-zx''','''    raw_rel_n=(raw_n-rzn) if raw_n is not None and rzn is not None else None
    raw_rel_e=(raw_e-rze) if raw_e is not None and rze is not None else None
    imu_n=raw.get("imu_dr_n"); imu_e=raw.get("imu_dr_e"); imu_d=raw.get("imu_dr_d")
    try:
        imu_n=float(imu_n) if imu_n is not None else None
        imu_e=float(imu_e) if imu_e is not None else None
        imu_d=float(imu_d) if imu_d is not None else None
    except Exception:
        imu_n=imu_e=imu_d=None
    if not hasattr(live_payload,"imu_zero"):
        live_payload.imu_zero={"n":None,"e":None,"d":None}
    iz=live_payload.imu_zero
    if imu_n is not None and iz["n"] is None:
        iz["n"],iz["e"],iz["d"]=imu_n,imu_e,imu_d
    imu_rel_n=(imu_n-iz["n"]) if imu_n is not None and iz["n"] is not None else None
    imu_rel_e=(imu_e-iz["e"]) if imu_e is not None and iz["e"] is not None else None
    imu_rel_d=(imu_d-iz["d"]) if imu_d is not None and iz["d"] is not None else None
    ekf_rel_x=x-zx''',1)

s=s.replace('''        "imu_dr_n_mm":float(raw.get("imu_dr_n",0.0))*1000.0 if raw.get("imu_dr_n") is not None else None,
        "imu_dr_e_mm":float(raw.get("imu_dr_e",0.0))*1000.0 if raw.get("imu_dr_e") is not None else None,
        "imu_dr_d_mm":float(raw.get("imu_dr_d",0.0))*1000.0 if raw.get("imu_dr_d") is not None else None,''','''        "imu_dr_n_mm":imu_rel_n*1000.0 if imu_rel_n is not None else None,
        "imu_dr_e_mm":imu_rel_e*1000.0 if imu_rel_e is not None else None,
        "imu_dr_d_mm":imu_rel_d*1000.0 if imu_rel_d is not None else None,''',1)

anchor='''        if rn is not None and re is not None:
            cur["raw_of_n_mm"]=0.0;cur["raw_of_e_mm"]=0.0;cur["raw_of_drift_mm"]=0.0
        _live_latest=cur'''
repl='''        if rn is not None and re is not None:
            cur["raw_of_n_mm"]=0.0;cur["raw_of_e_mm"]=0.0;cur["raw_of_drift_mm"]=0.0
        # Web HOME uses the current independent IMU position as its display/reference origin.
        # The C++ IMU estimator itself is not reset here; telemetry is currently one-way runtime -> web.
        if not hasattr(live_payload,"imu_zero"):
            live_payload.imu_zero={"n":None,"e":None,"d":None}
        inz=cur.get("imu_dr_n_mm"); iez=cur.get("imu_dr_e_mm"); idz=cur.get("imu_dr_d_mm")
        raw_imu_n=cur.get("imu_dr_raw_n")
        # Recover absolute runtime IMU state from current reference + displayed relative value.
        iz=live_payload.imu_zero
        if inz is not None and iz["n"] is not None: iz["n"]+=float(inz)/1000.0
        if iez is not None and iz["e"] is not None: iz["e"]+=float(iez)/1000.0
        if idz is not None and iz["d"] is not None: iz["d"]+=float(idz)/1000.0
        if inz is not None: cur["imu_dr_n_mm"]=0.0
        if iez is not None: cur["imu_dr_e_mm"]=0.0
        if idz is not None: cur["imu_dr_d_mm"]=0.0
        _live_latest=cur'''
assert anchor in s, 'set_zero anchor not found'
s=s.replace(anchor,repl,1)

# Physical RC HOME resets the C++ IMU state, so its Web reference must reset too.
anchor2='''            if raw_n is not None and raw_e is not None:
                _raw_zero["n"],_raw_zero["e"]=raw_n,raw_e
            rc_zero_event=True'''
repl2='''            if raw_n is not None and raw_e is not None:
                _raw_zero["n"],_raw_zero["e"]=raw_n,raw_e
            if hasattr(live_payload,"imu_zero"):
                live_payload.imu_zero={"n":None,"e":None,"d":None}
            rc_zero_event=True'''
assert anchor2 in s, 'RC HOME anchor not found'
s=s.replace(anchor2,repl2,1)

p.write_text(s)
print('OK: Web HOME now zeros IMU XYZ reference; physical RC HOME clears the Web IMU reference.')
print('NOTE: this is reference synchronization only; C++ IMU bias recalibration remains tied to physical RC HOME.')
