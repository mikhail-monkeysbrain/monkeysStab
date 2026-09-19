#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="// FUSED_V2_FRAME_CAPTURE_V1"
if marker in s:
    print("already applied")
    raise SystemExit(0)

# Add a diagnostic method to FlowFc immediately before its destructor.
anchor='''  ~FlowFc(){ stop(); }
'''
if anchor not in s:
    raise SystemExit("FlowFc destructor anchor not found; source not modified")

method=r'''  // FUSED_V2_FRAME_CAPTURE_V1
  // Snapshot nearest buffered IMU DR state for every camera frame, including
  // invalid reason5/reason6 frames. Diagnostic only.
  void writeFusedV2FrameCapture(const std::string& csvpath,
                                uint64_t frame,
                                int64_t cam_ns,
                                bool production_valid,
                                int invalid_reason,
                                int tracked,
                                int inliers,
                                double inlier_ratio,
                                double dt_s){
    std::lock_guard<std::mutex> l(mu);
    if(fused_v2_imu_history.empty()) return;
    auto best=fused_v2_imu_history.begin();
    int64_t best_abs=std::llabs(best->recv_ns-cam_ns);
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      const int64_t d=std::llabs(it->recv_ns-cam_ns);
      if(d<best_abs){best=it;best_abs=d;}
    }
    static std::ofstream out;
    static bool header=false;
    if(!out.is_open()){
      const std::filesystem::path production_csv_path(csvpath);
      out.open(production_csv_path.parent_path()/"fused_v2_frame_capture.csv",
               std::ios::out|std::ios::trunc);
    }
    if(!out.is_open()) return;
    if(!header){
      out<<"frame,cam_ns,imu_recv_ns,age_ms,production_valid,invalid_reason,"
           "tracked,inliers,inlier_ratio,dt_s,imu_n_m,imu_e_m,imu_vn,imu_ve\n";
      header=true;
    }
    out<<frame<<','<<cam_ns<<','<<best->recv_ns<<','
       <<(cam_ns-best->recv_ns)*1e-6<<','
       <<(production_valid?1:0)<<','<<invalid_reason<<','
       <<tracked<<','<<inliers<<','<<inlier_ratio<<','<<dt_s<<','
       <<best->pos_n<<','<<best->pos_e<<','<<best->vel_n<<','<<best->vel_e<<'\n';
    out.flush();
  }

'''
s=s.replace(anchor,method+anchor,1)

# The production CSV is emitted once per processed frame. Insert immediately
# before its existing write statement; local source is intentionally matched
# by a stable prefix rather than line number.
needle='''        csv<<ts<<','<<camera_ts_ns'''
if needle not in s:
    raise SystemExit("production CSV write anchor not found; source not modified")

call=r'''        // FUSED_V2_FRAME_CAPTURE_V1
        // mono timestamp 'ts' is the same frame clock used by production CSV.
        fc.writeFusedV2FrameCapture(
          csvpath,frame,ts,s.valid,s.invalid_reason,
          s.tracked,s.inliers,s.inlier_ratio,dt);

'''
s=s.replace(needle,call+needle,1)

p.write_text(s)
print("patched",p)
print("added fused_v2_frame_capture.csv for every processed frame; production unchanged")
