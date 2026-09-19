#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="// FUSED_V2_REALTIME_SHADOW_V1"
if marker in s:
    print("already applied")
    raise SystemExit(0)

# This patch intentionally depends on the already-validated causal frame-capture
# instrumentation. It adds a parallel realtime shadow only; production outputs
# and WORKED5/FUSED-V1 are not modified.
destructor='''  ~FlowFc(){ stop(); }
'''
if destructor not in s:
    raise SystemExit("FlowFc destructor anchor not found; source not modified")

method=r'''  // FUSED_V2_REALTIME_SHADOW_V1
  // Frozen policy, identical to tools/analyze_fused_v2_full_shadow.py:
  // BAD      = eligible && ratio < 0.50 && inliers < 100
  // RECOVER  = two consecutive eligible frames with ratio > 0.70 && inliers >= 100
  // pre-roll = 150 ms
  // IMU      = latest causal sample (recv_ns <= camera monotonic timestamp)
  //
  // Diagnostic shadow only. It never changes WORKED5, FUSED-V1 or MAVLink output.
  struct FusedV2RtSnap {
    uint64_t frame=0;
    int64_t cam_ns=0;
    double shadow_n=0.0,shadow_e=0.0;
    double imu_n=0.0,imu_e=0.0;
  };
  std::deque<FusedV2RtSnap> fused_v2_rt_history;
  double fused_v2_rt_n=0.0,fused_v2_rt_e=0.0;
  uint64_t fused_v2_rt_last_cam_seq=0;
  bool fused_v2_rt_bridge=false;
  int fused_v2_rt_good_streak=0;
  uint64_t fused_v2_rt_events=0;
  uint64_t fused_v2_rt_anchor_frame=0,fused_v2_rt_bad_frame=0;
  double fused_v2_rt_base_n=0.0,fused_v2_rt_base_e=0.0;
  double fused_v2_rt_base_imu_n=0.0,fused_v2_rt_base_imu_e=0.0;

  void updateFusedV2RealtimeShadow(const std::string& csvpath,
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

    // Strictly causal IMU lookup: never use a sample from the future.
    auto imu_it=fused_v2_imu_history.end();
    for(auto it=fused_v2_imu_history.begin();it!=fused_v2_imu_history.end();++it){
      if(it->recv_ns<=cam_ns &&
         (imu_it==fused_v2_imu_history.end() || it->recv_ns>imu_it->recv_ns))
        imu_it=it;
    }
    if(imu_it==fused_v2_imu_history.end()) return;

    // Consume every unique WORKED5 event exactly once. While bridging, the event
    // is deliberately consumed but not added: offline shadow removes the same
    // WORKED5 increments from (anchor,recovery].
    bool new_w5=false;
    double w5_dn=0.0,w5_de=0.0;
    if(imu_cam_seq!=fused_v2_rt_last_cam_seq){
      fused_v2_rt_last_cam_seq=imu_cam_seq;
      if(imu_cam_valid){
        new_w5=true;
        w5_dn=imu_cam_dN;
        w5_de=imu_cam_dE;
      }
    }

    const bool eligible=(dt_s>0.0 && tracked>=20);
    const bool is_bad=(eligible && inlier_ratio<0.50 && inliers<100);
    const bool is_good=(eligible && inlier_ratio>0.70 && inliers>=100);

    // Healthy mode follows frozen WORKED5 exactly.
    if(!fused_v2_rt_bridge && new_w5){
      fused_v2_rt_n+=w5_dn;
      fused_v2_rt_e+=w5_de;
    }

    if(!fused_v2_rt_bridge && is_bad){
      const int64_t target=cam_ns-150000000LL;
      auto a=fused_v2_rt_history.end();
      for(auto it=fused_v2_rt_history.begin();it!=fused_v2_rt_history.end();++it)
        if(it->cam_ns<=target) a=it;

      if(a!=fused_v2_rt_history.end()){
        fused_v2_rt_bridge=true;
        fused_v2_rt_good_streak=0;
        ++fused_v2_rt_events;
        fused_v2_rt_anchor_frame=a->frame;
        fused_v2_rt_bad_frame=frame;
        fused_v2_rt_base_n=a->shadow_n;
        fused_v2_rt_base_e=a->shadow_e;
        fused_v2_rt_base_imu_n=a->imu_n;
        fused_v2_rt_base_imu_e=a->imu_e;
        // Rewind the already accumulated visual trajectory to the frozen
        // 150-ms anchor, then replace it with causal IMU displacement.
        fused_v2_rt_n=fused_v2_rt_base_n+(imu_it->pos_n-fused_v2_rt_base_imu_n);
        fused_v2_rt_e=fused_v2_rt_base_e+(imu_it->pos_e-fused_v2_rt_base_imu_e);
      }
    }else if(fused_v2_rt_bridge){
      fused_v2_rt_n=fused_v2_rt_base_n+(imu_it->pos_n-fused_v2_rt_base_imu_n);
      fused_v2_rt_e=fused_v2_rt_base_e+(imu_it->pos_e-fused_v2_rt_base_imu_e);
      if(is_good) ++fused_v2_rt_good_streak;
      else fused_v2_rt_good_streak=0;
      if(fused_v2_rt_good_streak>=2){
        // Recovery confirmation frame is still covered by IMU. Future WORKED5
        // events resume from the next processed frame.
        fused_v2_rt_bridge=false;
        fused_v2_rt_good_streak=0;
      }
    }

    fused_v2_rt_history.push_back({
      frame,cam_ns,fused_v2_rt_n,fused_v2_rt_e,imu_it->pos_n,imu_it->pos_e});
    while(!fused_v2_rt_history.empty() &&
          cam_ns-fused_v2_rt_history.front().cam_ns>500000000LL)
      fused_v2_rt_history.pop_front();

    static std::ofstream out;
    static bool header=false;
    if(!out.is_open()){
      const std::filesystem::path production_csv_path(csvpath);
      out.open(production_csv_path.parent_path()/"fused_v2_realtime_shadow.csv",
               std::ios::out|std::ios::trunc);
    }
    if(!out.is_open()) return;
    if(!header){
      out<<"frame,cam_ns,production_valid,invalid_reason,tracked,inliers,inlier_ratio,dt_s,"
           "new_w5,w5_dN_m,w5_dE_m,bridge,event_count,anchor_frame,bad_frame,"
           "imu_age_ms,shadow_n_m,shadow_e_m,shadow_endpoint_m\n";
      header=true;
    }
    out<<frame<<','<<cam_ns<<','<<(production_valid?1:0)<<','<<invalid_reason<<','
       <<tracked<<','<<inliers<<','<<inlier_ratio<<','<<dt_s<<','
       <<(new_w5?1:0)<<','<<w5_dn<<','<<w5_de<<','
       <<(fused_v2_rt_bridge?1:0)<<','<<fused_v2_rt_events<<','
       <<fused_v2_rt_anchor_frame<<','<<fused_v2_rt_bad_frame<<','
       <<(cam_ns-imu_it->recv_ns)*1e-6<<','
       <<fused_v2_rt_n<<','<<fused_v2_rt_e<<','
       <<std::hypot(fused_v2_rt_n,fused_v2_rt_e)<<'\n';
    out.flush();
  }

'''
s=s.replace(destructor,method+destructor,1)

# Insert next to the already-installed per-frame causal capture. This anchor is
# local-dirty-tree friendly and avoids replacing the stale remote production file.
call_anchor='''        fc.writeFusedV2FrameCapture(
          csvpath,frame,now,s.valid,s.invalid_reason,
          s.tracked,s.inliers,s.inlier_ratio,dt);
'''
if call_anchor not in s:
    raise SystemExit("FUSED-V2 frame-capture call anchor not found; source not modified")

call=call_anchor+r'''        // FUSED_V2_REALTIME_SHADOW_V1
        fc.updateFusedV2RealtimeShadow(
          csvpath,frame,now,s.valid,s.invalid_reason,
          s.tracked,s.inliers,s.inlier_ratio,dt);
'''
s=s.replace(call_anchor,call,1)

p.write_text(s)
print("patched",p)
print("added realtime FUSED-V2 shadow; production WORKED5/FUSED-V1/MAVLink unchanged")
