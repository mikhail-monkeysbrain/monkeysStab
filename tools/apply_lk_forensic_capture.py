#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Добавляет forensic MJPEG ring capture, не меняя WORKED5/LK."""
from pathlib import Path
p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
if "LK_FORENSIC_CAPTURE_V1" in s:
    print("already applied"); raise SystemExit(0)

old='#include <deque>'
if old not in s:
    # insert near vector include
    old='#include <vector>'
    if old not in s: raise SystemExit("include anchor not found")
    s=s.replace(old,old+'\n#include <deque>\n#include <filesystem>',1)
else:
    s=s.replace(old,old+'\n#include <filesystem>',1)

anchor='''      static int64_t w5w_t0_ns=monoNs();
      std::vector<uint8_t> latest_jpeg;'''
insert='''      static int64_t w5w_t0_ns=monoNs();

      // LK_FORENSIC_CAPTURE_V1: compressed MJPEG ring in RAM.
      // Diagnostic only: no WORKED5/LK parameters or measurements are changed.
      struct LkForensicFrame {
        uint64_t frame=0;
        int64_t mono_ns=0, v4l2_ns=0, dq_ns=0;
        uint32_t v4l2_flags=0;
        std::vector<uint8_t> jpeg;
      };
      static std::deque<LkForensicFrame> lkfc_ring;
      static bool lkfc_triggered=false;
      static int64_t lkfc_trigger_ns=0;
      static uint64_t lkfc_trigger_frame=0;
      static constexpr int64_t kLkfcPreNs=2000000000LL;
      static constexpr int64_t kLkfcPostNs=2000000000LL;
      static constexpr double kLkfcTriggerMs=20.0;
      static uint64_t lkfc_seq=0;

      std::vector<uint8_t> latest_jpeg;'''
if anchor not in s: raise SystemExit("state anchor not found")
s=s.replace(anchor,insert,1)

anchor2='''      cv::Mat gray=cv::imdecode(latest_jpeg,cv::IMREAD_GRAYSCALE);
      if(gray.empty())continue;
      ++fps_decoded; ++w5w_decoded;'''
insert2='''      // Preserve the selected compressed frame before decode.  The ring is
      // bounded by camera timestamp, so normal operation uses only a few tens
      // of MiB and performs no disk I/O.
      lkfc_ring.push_back(LkForensicFrame{frame,ts,selected_v4l2_ts_ns,
                                          selected_dq_mono_ns,selected_v4l2_flags,
                                          latest_jpeg});
      while(lkfc_ring.size()>1 && ts-lkfc_ring.front().mono_ns>kLkfcPreNs+kLkfcPostNs)
        lkfc_ring.pop_front();

      cv::Mat gray=cv::imdecode(latest_jpeg,cv::IMREAD_GRAYSCALE);
      if(gray.empty())continue;
      ++fps_decoded; ++w5w_decoded;'''
if anchor2 not in s: raise SystemExit("decode anchor not found")
s=s.replace(anchor2,insert2,1)

anchor3='''        if(!prev.empty())s=estimateRawFlow(prev,gray,dt,calib,
                                            prev_camera_height_m,current_camera_height_m,
                                            C1_R_C0_ptr,dr_interp_gap_ms);
        web_live.sendPreview(gray,s.inlier_points,ts);'''
insert3='''        if(!prev.empty())s=estimateRawFlow(prev,gray,dt,calib,
                                            prev_camera_height_m,current_camera_height_m,
                                            C1_R_C0_ptr,dr_interp_gap_ms);

        // Trigger on production forward-LK wall time.  Do not use valid=0:
        // the observed collapse begins before the final validity gate fails.
        if(!lkfc_triggered && s.t_lk_ms>kLkfcTriggerMs){
          lkfc_triggered=true;
          lkfc_trigger_ns=ts;
          lkfc_trigger_frame=frame;
          std::cerr<<"LK_FORENSIC TRIGGER frame="<<frame
                   <<" lk_ms="<<s.t_lk_ms<<" dt_ms="<<(dt*1000.0)<<"\\n";
        }
        if(lkfc_triggered && ts-lkfc_trigger_ns>=kLkfcPostNs){
          const auto dir=std::filesystem::path("/tmp")/
            ("monkeysstab_lk_forensic_"+std::to_string(++lkfc_seq));
          std::filesystem::create_directories(dir);
          std::ofstream meta(dir/"frames.csv");
          meta<<"seq,frame,mono_ns,v4l2_ns,dq_ns,v4l2_flags,jpeg\\n";
          size_t n=0;
          for(const auto& q:lkfc_ring){
            if(q.mono_ns < lkfc_trigger_ns-kLkfcPreNs ||
               q.mono_ns > lkfc_trigger_ns+kLkfcPostNs) continue;
            const std::string name="frame_"+std::to_string(q.frame)+".jpg";
            std::ofstream jf(dir/name,std::ios::binary);
            jf.write(reinterpret_cast<const char*>(q.jpeg.data()),
                     static_cast<std::streamsize>(q.jpeg.size()));
            meta<<n++<<','<<q.frame<<','<<q.mono_ns<<','<<q.v4l2_ns<<','
                <<q.dq_ns<<','<<q.v4l2_flags<<','<<name<<"\\n";
          }
          meta.flush();
          std::ofstream trig(dir/"trigger.txt");
          trig<<"trigger_frame="<<lkfc_trigger_frame<<"\\n"
              <<"trigger_mono_ns="<<lkfc_trigger_ns<<"\\n"
              <<"threshold_lk_ms="<<kLkfcTriggerMs<<"\\n";
          trig.flush();
          std::cerr<<"LK_FORENSIC SAVED dir="<<dir.string()
                   <<" frames="<<n<<" trigger_frame="<<lkfc_trigger_frame<<"\\n";
          lkfc_triggered=false;
          lkfc_ring.clear();
        }

        web_live.sendPreview(gray,s.inlier_points,ts);'''
if anchor3 not in s: raise SystemExit("flow anchor not found")
s=s.replace(anchor3,insert3,1)
p.write_text(s)
print("patched",p)
