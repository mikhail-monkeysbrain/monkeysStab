#!/usr/bin/env python3
# Локальный патчер: добавляет диагностические счетчики FPS в текущий
# src/optical_flow_mavlink.cpp, не меняя алгоритм WORKED5/FUSED.
from pathlib import Path
import sys

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
orig=s

anchor='''      std::vector<uint8_t> latest_jpeg;
      int64_t ts=0;'''
insert='''      // FPS FORENSIC: считаем все DQBUF, выбранные newest кадры, decode и WORKED5.
      static uint64_t fps_dqbuf=0, fps_selected=0, fps_queue_drop=0;
      static uint64_t fps_decoded=0, fps_w5_attempt=0, fps_w5_valid=0;
      static int64_t fps_t0_ns=monoNs();
      static int64_t fps_prev_selected_ts=0;
      static double fps_dt_sum_ms=0.0, fps_dt_max_ms=0.0;
      static uint64_t fps_dt_n=0;
      std::vector<uint8_t> latest_jpeg;
      int64_t ts=0;'''
if 'static uint64_t fps_dqbuf=' not in s:
    if anchor not in s: sys.exit("ERROR: capture anchor not found")
    s=s.replace(anchor,insert,1)

anchor='''        const int64_t bts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;'''
if '++fps_dqbuf;' not in s:
    if anchor not in s: sys.exit("ERROR: DQBUF anchor not found")
    s=s.replace(anchor,'''        ++fps_dqbuf;
'''+anchor,1)

anchor='''      camera_queue_dropped_total += camera_queue_dropped;

      const int64_t now=monoNs();'''
insert='''      camera_queue_dropped_total += camera_queue_dropped;
      ++fps_selected;
      fps_queue_drop += camera_queue_dropped;
      if(fps_prev_selected_ts>0 && ts>fps_prev_selected_ts){
        const double dms=(ts-fps_prev_selected_ts)*1e-6;
        fps_dt_sum_ms+=dms;
        fps_dt_max_ms=std::max(fps_dt_max_ms,dms);
        ++fps_dt_n;
      }
      fps_prev_selected_ts=ts;

      const int64_t now=monoNs();'''
if '++fps_selected;' not in s:
    if anchor not in s: sys.exit("ERROR: selected anchor not found")
    s=s.replace(anchor,insert,1)

anchor='''      if(gray.empty()) continue;
      ++frame;'''
insert='''      if(gray.empty()) continue;
      ++fps_decoded;
      ++frame;'''
if '++fps_decoded;' not in s:
    if anchor not in s: sys.exit("ERROR: decode anchor not found")
    s=s.replace(anchor,insert,1)

anchor='''            const auto w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,hcam,dt);'''
insert='''            ++fps_w5_attempt;
            const auto w5=worked5::estimate(
              s.metric_prev_points,s.metric_curr_points,
              calib.K,focal_scale,calib.D,hcam,dt);'''
if '++fps_w5_attempt;' not in s:
    if anchor not in s: sys.exit("ERROR: W5 attempt anchor not found")
    s=s.replace(anchor,insert,1)

anchor='''            if(w5.valid){
              // Frozen blind convention'''
insert='''            if(w5.valid){
              ++fps_w5_valid;
              // Frozen blind convention'''
if '++fps_w5_valid;' not in s:
    if anchor not in s: sys.exit("ERROR: W5 valid anchor not found")
    s=s.replace(anchor,insert,1)

# Печатаем раз в ~2 секунды. Счетчики cumulative; Hz считаются от старта,
# поэтому не вмешиваются в рабочие данные и не требуют дополнительного state reset.
anchor='''        double lm=0; int strength=0; int64_t lns=0;'''
insert='''        if(now-fps_t0_ns>=2000000000LL){
          const double sec=(now-fps_t0_ns)*1e-9;
          std::cerr<<"FPS_FORENSIC"
                   <<" dqbuf_hz="<<(fps_dqbuf/sec)
                   <<" selected_hz="<<(fps_selected/sec)
                   <<" dropped_hz="<<(fps_queue_drop/sec)
                   <<" decoded_hz="<<(fps_decoded/sec)
                   <<" w5_attempt_hz="<<(fps_w5_attempt/sec)
                   <<" w5_valid_hz="<<(fps_w5_valid/sec)
                   <<" selected_dt_mean_ms="<<(fps_dt_n?fps_dt_sum_ms/fps_dt_n:0.0)
                   <<" selected_dt_max_ms="<<fps_dt_max_ms
                   <<"\\n";
        }

        double lm=0; int strength=0; int64_t lns=0;'''
if '<<"FPS_FORENSIC"' not in s:
    if anchor not in s: sys.exit("ERROR: report anchor not found")
    s=s.replace(anchor,insert,1)

if s==orig:
    print("OK: FPS forensic already present")
else:
    p.write_text(s)
    print("OK: FPS forensic instrumentation added")
