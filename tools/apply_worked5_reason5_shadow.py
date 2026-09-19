#!/usr/bin/env python3
from pathlib import Path

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="// WORKED5_REASON5_SHADOW_V1"
if marker in s:
    print("already applied")
    raise SystemExit(0)

anchor='''        // WORKED5_MAG_SHADOW_V1
'''
if anchor not in s:
    raise SystemExit("anchor not found: WORKED5_MAG_SHADOW_V1")

block=r'''        // WORKED5_REASON5_SHADOW_V1
        // Diagnostic C arms only. Production validity and frozen WORKED5 are untouched.
        // Re-fit the exact production RANSAC inliers on reason5 frames with lower
        // diagnostic-only point-count floors: 15, 10, 7.
        {
          static std::ofstream r5_csv;
          static bool r5_header=false;
          static double c15_n=0.0,c15_e=0.0,c10_n=0.0,c10_e=0.0,c7_n=0.0,c7_e=0.0;
          if(!r5_csv.is_open()){
            const std::filesystem::path production_csv_path(csvpath);
            r5_csv.open(production_csv_path.parent_path()/"worked5_reason5_shadow.csv",
                        std::ios::out|std::ios::trunc);
          }
          if(r5_csv.is_open() && !r5_header){
            r5_csv<<"frame,mono_ns,production_valid,invalid_reason,pairs,dt_s,hcam_m,"
                     "c15_valid,c15_dN_m,c15_dE_m,c15_pN_m,c15_pE_m,"
                     "c10_valid,c10_dN_m,c10_dE_m,c10_pN_m,c10_pE_m,"
                     "c7_valid,c7_dN_m,c7_dE_m,c7_pN_m,c7_pE_m\n";
            r5_header=true;
          }

          double h=0.0;
          if(bench_true_camera_height>0.0) h=bench_true_camera_height;
          else if(current_camera_height_valid) h=current_camera_height_m;
          const int np=(int)std::min(s.metric_prev_points.size(),s.metric_curr_points.size());

          auto fit=[&](int minpts, double& dN, double& dE)->bool{
            dN=dE=0.0;
            if(s.invalid_reason!=5 || !fg_ok || np<minpts ||
               !(dt>0.0 && dt<0.2) || !(h>0.02) || !std::isfinite(h)) return false;

            cv::Mat K=calib.K.clone();
            const double k=worked5::kFocalScale/focal_scale;
            K.at<double>(0,0)*=k; K.at<double>(1,1)*=k;
            std::vector<cv::Point2f> a,b;
            cv::undistortPoints(s.metric_prev_points,a,K,calib.D);
            cv::undistortPoints(s.metric_curr_points,b,K,calib.D);
            if(a.size()!=b.size() || (int)a.size()<minpts) return false;

            cv::Mat A((int)a.size()*2,4,CV_64F), rhs((int)a.size()*2,1,CV_64F);
            for(size_t i=0;i<a.size();++i){
              const double x=a[i].x,y=a[i].y;
              const double du=b[i].x-a[i].x,dv=b[i].y-a[i].y;
              const int r=(int)(2*i);
              A.at<double>(r,0)=1.0; A.at<double>(r,1)=0.0;
              A.at<double>(r,2)=x;   A.at<double>(r,3)=-y;
              rhs.at<double>(r,0)=du;
              A.at<double>(r+1,0)=0.0; A.at<double>(r+1,1)=1.0;
              A.at<double>(r+1,2)=y;   A.at<double>(r+1,3)=x;
              rhs.at<double>(r+1,0)=dv;
            }
            cv::Mat sol;
            if(!cv::solve(A,rhs,sol,cv::DECOMP_SVD) || sol.rows!=4) return false;
            const double du=sol.at<double>(0,0), dv=sol.at<double>(1,0);
            const double dx=dv*h, dy=-du*h;
            if(!std::isfinite(dx)||!std::isfinite(dy)) return false;

            const double cr=std::cos(fg.roll),sr=std::sin(fg.roll);
            const double cp=std::cos(fg.pitch),sp=std::sin(fg.pitch);
            const double cy=std::cos(fg.yaw),sy=std::sin(fg.yaw);
            const double r00=cy*cp, r01=cy*sp*sr-sy*cr;
            const double r10=sy*cp, r11=sy*sp*sr+cy*cr;
            dN=r00*dx+r01*dy; dE=r10*dx+r11*dy;
            return std::isfinite(dN)&&std::isfinite(dE);
          };

          double n15=0,e15=0,n10=0,e10=0,n7=0,e7=0;
          const bool v15=fit(15,n15,e15);
          const bool v10=fit(10,n10,e10);
          const bool v7 =fit(7,n7,e7);
          if(v15){c15_n+=n15;c15_e+=e15;}
          if(v10){c10_n+=n10;c10_e+=e10;}
          if(v7 ){c7_n +=n7; c7_e +=e7;}

          if(r5_csv.is_open()){
            r5_csv<<frame<<','<<ts<<','<<(s.valid?1:0)<<','<<s.invalid_reason<<','
                  <<np<<','<<dt<<','<<h<<','
                  <<(v15?1:0)<<','<<n15<<','<<e15<<','<<c15_n<<','<<c15_e<<','
                  <<(v10?1:0)<<','<<n10<<','<<e10<<','<<c10_n<<','<<c10_e<<','
                  <<(v7?1:0)<<','<<n7<<','<<e7<<','<<c7_n<<','<<c7_e<<'\n';
            r5_csv.flush();
          }
        }

'''
s=s.replace(anchor,block+anchor,1)
p.write_text(s)
print("patched",p)
