#!/usr/bin/env python3
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "src" / "optical_flow_mavlink.cpp"
s = p.read_text(encoding="utf-8")

anchor1 = """    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    double prev_camera_height_m=0.0;
"""
replace1 = """    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    // WORKED5_CADENCE_AB_V1: test-only frame cadence divisor.
    // div=1 preserves the frozen ed43ecc behavior. div=2 processes every
    // second decoded camera frame, producing ~16 ms pairs from a ~120 Hz stream.
    // No WORKED5 scale/KLT/RANSAC parameter is changed.
    int worked5_cadence_div=1;
    if(const char* e=std::getenv("MONKEYS_WORKED5_CADENCE_DIV"); e && *e)
      worked5_cadence_div=std::clamp(std::atoi(e),1,8);
    uint64_t worked5_cadence_seen=0;
    std::cerr<<"WORKED5_CADENCE_AB div="<<worked5_cadence_div<<"\\n";
    double prev_camera_height_m=0.0;
"""
if anchor1 not in s:
    raise SystemExit("ERROR: init anchor not found; source is not expected ed43ecc layout")
s = s.replace(anchor1, replace1, 1)

anchor2 = """      ++fps_decoded; ++w5w_decoded;
      ++frame;

      // Preserve only successfully decoded selected MJPEG frames. frame now
"""
replace2 = """      ++fps_decoded; ++w5w_decoded;
      ++worked5_cadence_seen;
      if(worked5_cadence_div>1 &&
         ((worked5_cadence_seen-1)%static_cast<uint64_t>(worked5_cadence_div))!=0){
        continue;
      }
      ++frame;

      // Preserve only successfully decoded selected MJPEG frames. frame now
"""
if anchor2 not in s:
    raise SystemExit("ERROR: decode anchor not found; source is not expected ed43ecc layout")
s = s.replace(anchor2, replace2, 1)

p.write_text(s, encoding="utf-8")
print("PATCHED:", p)
print("MONKEYS_WORKED5_CADENCE_DIV=1 -> frozen cadence")
print("MONKEYS_WORKED5_CADENCE_DIV=2 -> every second decoded frame")
