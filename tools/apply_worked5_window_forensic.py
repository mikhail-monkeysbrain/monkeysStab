#!/usr/bin/env python3
# Добавляет короткое 250-ms окно forensic поверх уже установленного
# FPS_FORENSIC. Алгоритм WORKED5/FUSED не меняется.
from pathlib import Path
import sys

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
orig=s

if "fps_dqbuf" not in s:
    sys.exit("ERROR: сначала примените tools/apply_worked5_fps_forensic.py")

anchor='''      static uint64_t fps_dt_n=0;
      std::vector<uint8_t> latest_jpeg;'''
insert='''      static uint64_t fps_dt_n=0;

      // W5 WINDOW FORENSIC: локальное окно 250 ms, чтобы короткий провал
      // не растворялся в cumulative FPS_FORENSIC.
      static uint64_t w5w_dqbuf=0, w5w_selected=0, w5w_drop=0;
      static uint64_t w5w_decoded=0, w5w_attempt=0, w5w_valid=0;
      static uint64_t w5w_dt_n=0;
      static double w5w_dt_sum_ms=0.0, w5w_dt_max_ms=0.0;
      static int64_t w5w_t0_ns=monoNs();
      std::vector<uint8_t> latest_jpeg;'''
if "static uint64_t w5w_dqbuf=" not in s:
    if anchor not in s: sys.exit("ERROR: window state anchor not found")
    s=s.replace(anchor,insert,1)

repls=[
("++fps_dqbuf;", "++fps_dqbuf; ++w5w_dqbuf;"),
("++fps_selected;\n      fps_queue_drop += camera_queue_dropped;",
 "++fps_selected; ++w5w_selected;\n      fps_queue_drop += camera_queue_dropped;\n      w5w_drop += camera_queue_dropped;"),
("fps_dt_max_ms=std::max(fps_dt_max_ms,dms);\n        ++fps_dt_n;",
 "fps_dt_max_ms=std::max(fps_dt_max_ms,dms);\n        ++fps_dt_n;\n        w5w_dt_sum_ms+=dms; w5w_dt_max_ms=std::max(w5w_dt_max_ms,dms); ++w5w_dt_n;"),
("++fps_decoded;", "++fps_decoded; ++w5w_decoded;"),
("++fps_w5_attempt;", "++fps_w5_attempt; ++w5w_attempt;"),
("++fps_w5_valid;", "++fps_w5_valid; ++w5w_valid;"),
]
for old,new in repls:
    if new in s: continue
    if old not in s: sys.exit("ERROR: counter anchor not found: "+old)
    s=s.replace(old,new,1)

anchor='''        if(now-fps_t0_ns>=2000000000LL){'''
report='''        if(now-w5w_t0_ns>=250000000LL){
          const double wsec=(now-w5w_t0_ns)*1e-9;
          std::cerr<<"W5_WINDOW"
                   <<" dqbuf_hz="<<(w5w_dqbuf/wsec)
                   <<" selected_hz="<<(w5w_selected/wsec)
                   <<" dropped_hz="<<(w5w_drop/wsec)
                   <<" decoded_hz="<<(w5w_decoded/wsec)
                   <<" attempt_hz="<<(w5w_attempt/wsec)
                   <<" valid_hz="<<(w5w_valid/wsec)
                   <<" valid_ratio="<<(w5w_attempt?double(w5w_valid)/double(w5w_attempt):0.0)
                   <<" dt_mean_ms="<<(w5w_dt_n?w5w_dt_sum_ms/w5w_dt_n:0.0)
                   <<" dt_max_ms="<<w5w_dt_max_ms
                   <<"\\n";
          w5w_dqbuf=w5w_selected=w5w_drop=w5w_decoded=w5w_attempt=w5w_valid=0;
          w5w_dt_n=0; w5w_dt_sum_ms=0.0; w5w_dt_max_ms=0.0;
          w5w_t0_ns=now;
        }

        if(now-fps_t0_ns>=2000000000LL){'''
if 'std::cerr<<"W5_WINDOW"' not in s:
    if anchor not in s: sys.exit("ERROR: report anchor not found")
    s=s.replace(anchor,report,1)

if s==orig:
    print("OK: W5 window forensic already present")
else:
    p.write_text(s)
    print("OK: W5 250-ms window forensic added")
