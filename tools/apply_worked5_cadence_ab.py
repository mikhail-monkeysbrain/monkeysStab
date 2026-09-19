#!/usr/bin/env python3
from pathlib import Path

p = Path(__file__).resolve().parents[1] / "src" / "optical_flow_mavlink.cpp"
s = p.read_text(encoding="utf-8")

# Remove the superseded V1 patch if it was already applied locally.
old_init = """    // WORKED5_CADENCE_AB_V1: test-only frame cadence divisor.
    // div=1 preserves the frozen ed43ecc behavior. div=2 processes every
    // second decoded camera frame, producing ~16 ms pairs from a ~120 Hz stream.
    // No WORKED5 scale/KLT/RANSAC parameter is changed.
    int worked5_cadence_div=1;
    if(const char* e=std::getenv("MONKEYS_WORKED5_CADENCE_DIV"); e && *e)
      worked5_cadence_div=std::clamp(std::atoi(e),1,8);
    uint64_t worked5_cadence_seen=0;
    std::cerr<<"WORKED5_CADENCE_AB div="<<worked5_cadence_div<<"\\n";
"""
old_skip = """      ++worked5_cadence_seen;
      if(worked5_cadence_div>1 &&
         ((worked5_cadence_seen-1)%static_cast<uint64_t>(worked5_cadence_div))!=0){
        continue;
      }
"""
s = s.replace(old_init, "", 1)
s = s.replace(old_skip, "", 1)

# V2 deliberately changes ONLY the production optical-flow anchor cadence.
# The camera dequeue/decode loop, FC/range handling and the rest of the runtime
# still execute on every selected frame. On skipped frames we do not call
# estimateRawFlow and do not publish/integrate a visual step; the anchor remains
# the previous cadence frame. This isolates the image-pair baseline without the
# V1 whole-runtime continue().
anchor1 = """    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    double prev_camera_height_m=0.0;
"""
replace1 = """    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    // WORKED5_CADENCE_AB_V2: test-only optical-flow anchor cadence.
    int worked5_cadence_div=1;
    if(const char* e=std::getenv("MONKEYS_WORKED5_CADENCE_DIV"); e && *e)
      worked5_cadence_div=std::clamp(std::atoi(e),1,8);
    uint64_t worked5_cadence_seen=0;
    std::cerr<<"WORKED5_CADENCE_AB_V2 div="<<worked5_cadence_div<<"\\n";
    double prev_camera_height_m=0.0;
"""
if anchor1 not in s:
    raise SystemExit("ERROR: init anchor not found")
s = s.replace(anchor1, replace1, 1)

anchor2 = """        const double dt=prev_ts?(ts-prev_ts)*1e-9:0.0;
        FlowStep s;
        if(!prev.empty())s=estimateRawFlow(
          prev,gray,dt,calib,
          prev_camera_height_valid?prev_camera_height_m:0.0,
          current_camera_height_valid?current_camera_height_m:0.0);
"""
replace2 = """        ++worked5_cadence_seen;
        const bool worked5_cadence_process =
            worked5_cadence_div<=1 ||
            ((worked5_cadence_seen-1)%static_cast<uint64_t>(worked5_cadence_div))==0;
        const double dt=prev_ts?(ts-prev_ts)*1e-9:0.0;
        FlowStep s;
        if(worked5_cadence_process && !prev.empty())s=estimateRawFlow(
          prev,gray,dt,calib,
          prev_camera_height_valid?prev_camera_height_m:0.0,
          current_camera_height_valid?current_camera_height_m:0.0);
        else if(!worked5_cadence_process)
          s.invalid_reason=90; // test-only skipped visual frame
"""
if anchor2 not in s:
    raise SystemExit("ERROR: flow anchor not found")
s = s.replace(anchor2, replace2, 1)

anchor3 = """        prev=gray.clone();
        prev_ts=ts;
        prev_camera_height_m=current_camera_height_m;
        prev_camera_height_valid=current_camera_height_valid;
        bridge_pending=false;
"""
replace3 = """        if(worked5_cadence_process){
          prev=gray.clone();
          prev_ts=ts;
          prev_camera_height_m=current_camera_height_m;
          prev_camera_height_valid=current_camera_height_valid;
        }
        bridge_pending=false;
"""
if anchor3 not in s:
    raise SystemExit("ERROR: final anchor not found")
s = s.replace(anchor3, replace3, 1)

p.write_text(s, encoding="utf-8")
print("PATCHED V2:", p)
print("DIV=1: exact frozen optical-flow cadence")
print("DIV=2: every second decoded frame forms the next optical-flow pair")
print("No whole-runtime continue() is used.")
