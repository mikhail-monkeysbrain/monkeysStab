#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace imu_dr {

struct State {
  bool calibrated=false;
  bool calibrating=true;
  int bias_samples=0;
  // Legacy telemetry fields retained for Web/API compatibility. In V1 they
  // expose the startup mean HIGHRES_IMU acceleration and are not subtracted
  // as a fixed accelerometer bias.
  double bias_bx=0,bias_by=0,bias_bz=0;
  // IMU_DR_GRAVITY_OBSERVER_V1: gravity/specific-force reference is estimated
  // in the native HIGHRES_IMU frame from the same accel+gyro stream. This
  // avoids subtracting a gravity vector synthesized from independently moving
  // FC ATTITUDE roll/pitch.
  double gravity_x=0,gravity_y=0,gravity_z=0;
  double gyro_bias_x=0,gyro_bias_y=0,gyro_bias_z=0;
  double gravity_mag=0;
  double acc_n=0,acc_e=0,acc_d=0;
  double vel_n=0,vel_e=0,vel_d=0;
  double pos_n=0,pos_e=0,pos_d=0;
  int stationary_samples=0;
  double diag_amag=0,diag_gmag=0,diag_dt=0;
  bool diag_acc_ok=false,diag_gyro_ok=false,diag_stationary=false;
  // V2 transition diagnostics: observational only; no estimator decisions use these fields.
  double diag_external_motion_speed_mps=0.0,diag_gravity_correction_weight=0.0;
  bool diag_external_zupt_allow=true,diag_zupt_active=false;
  uint64_t diag_acc_rejects=0,diag_gyro_rejects=0;
  // Startup calibration diagnostics. These are observational only.
  double startup_res_mean_x=0,startup_res_mean_y=0,startup_res_mean_z=0;
  double startup_res_m2_x=0,startup_res_m2_y=0,startup_res_m2_z=0;
  double startup_gmag_mean=0,startup_gmag_m2=0;
  double startup_res_norm_max=0,startup_gmag_max=0;
  uint64_t last_time_usec=0;
};

inline void reset(State& s){ s=State{}; }

// FC AHRS trim measured read-only from this airframe.  HIGHRES_IMU is in the
// autopilot/sensor body frame while MAVLink ATTITUDE is vehicle attitude.
// Convert acceleration with exact inverse AHRS_TRIM before using ATTITUDE.
constexpr double kAhrsTrimX=-0.03428453207;
constexpr double kAhrsTrimY=-0.0220823437;

inline void inverseAhrsTrim(double ax,double ay,double az,double& x,double& y,double& z){
  // R_trim = R321(trim_x,trim_y,0); inverse is its exact transpose.
  const double cr=std::cos(kAhrsTrimX),sr=std::sin(kAhrsTrimX);
  const double cp=std::cos(kAhrsTrimY),sp=std::sin(kAhrsTrimY);
  // R321 with yaw=0:
  // [ cp, sp*sr, sp*cr ; 0, cr, -sr ; -sp, cp*sr, cp*cr ]
  // Apply transpose directly.
  x= cp*ax - sp*az;
  y= sp*sr*ax + cr*ay + cp*sr*az;
  z= sp*cr*ax - sr*ay + cp*cr*az;
}

inline void bodyToNed(double ax,double ay,double az,double roll,double pitch,double yaw,
                      double& n,double& e,double& d){
  const double cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch),cy=std::cos(yaw),sy=std::sin(yaw);
  n=cy*cp*ax+(cy*sp*sr-sy*cr)*ay+(cy*sp*cr+sy*sr)*az;
  e=sy*cp*ax+(sy*sp*sr+cy*cr)*ay+(sy*sp*cr-cy*sr)*az;
  d=-sp*ax+cp*sr*ay+cp*cr*az+9.80665; // experimentally verified for FC HIGHRES_IMU convention
}

inline void update(State& s,double ax,double ay,double az,double gx,double gy,double gz,
                   double roll,double pitch,double yaw,uint64_t time_usec,
                   bool external_zupt_allow=true,
                   double external_motion_speed_mps=0.0){
  // V1 startup calibration uses 200 HIGHRES_IMU samples (~2 s at 100 Hz).
  // It deliberately does not synthesize gravity from FC ATTITUDE.
  if(s.calibrating){
    // Calibrate in the native HIGHRES_IMU frame. The mean accelerometer vector
    // is the initial gravity/specific-force reference; the mean gyro is the
    // propagation bias.  Keep the legacy bias_* telemetry fields observational
    // only so the Web/API schema remains compatible.
    const double rx=ax, ry=ay, rz=az;
    const double gm=std::sqrt(gx*gx+gy*gy+gz*gz);
    s.gravity_x+=ax; s.gravity_y+=ay; s.gravity_z+=az;
    s.gyro_bias_x+=gx; s.gyro_bias_y+=gy; s.gyro_bias_z+=gz;
    ++s.bias_samples;
    const double k=static_cast<double>(s.bias_samples);
    auto welford=[k](double v,double& mean,double& m2){
      const double delta=v-mean;
      mean+=delta/k;
      m2+=delta*(v-mean);
    };
    welford(rx,s.startup_res_mean_x,s.startup_res_m2_x);
    welford(ry,s.startup_res_mean_y,s.startup_res_m2_y);
    welford(rz,s.startup_res_mean_z,s.startup_res_m2_z);
    welford(gm,s.startup_gmag_mean,s.startup_gmag_m2);
    s.startup_res_norm_max=std::max(s.startup_res_norm_max,std::sqrt(rx*rx+ry*ry+rz*rz));
    s.startup_gmag_max=std::max(s.startup_gmag_max,gm);
    if(s.bias_samples>=200){
      const double inv=1.0/static_cast<double>(s.bias_samples);
      s.gravity_x*=inv; s.gravity_y*=inv; s.gravity_z*=inv;
      s.gyro_bias_x*=inv; s.gyro_bias_y*=inv; s.gyro_bias_z*=inv;
      s.gravity_mag=std::sqrt(s.gravity_x*s.gravity_x+
                              s.gravity_y*s.gravity_y+
                              s.gravity_z*s.gravity_z);
      // Preserve legacy fields for telemetry; they no longer participate in
      // the estimator.
      s.bias_bx=s.gravity_x; s.bias_by=s.gravity_y; s.bias_bz=s.gravity_z;
      s.calibrating=false; s.calibrated=true; s.last_time_usec=time_usec;
    }
    return;
  }
  if(!s.calibrated||!s.last_time_usec||time_usec<=s.last_time_usec){s.last_time_usec=time_usec;return;}
  const double dt=(time_usec-s.last_time_usec)*1e-6; s.last_time_usec=time_usec;
  if(!(dt>0&&dt<0.1))return;
  // Propagate the gravity reference with HIGHRES_IMU gyro in that same native
  // frame, then weakly pull it toward the measured accelerometer direction.
  // tau=1 s is deliberately the already-tested static MVP; dynamic validation
  // is required before treating this as a final inertial-navigation model.
  const double wx=gx-s.gyro_bias_x, wy=gy-s.gyro_bias_y, wz=gz-s.gyro_bias_z;
  const double cx=wy*s.gravity_z-wz*s.gravity_y;
  const double cy=wz*s.gravity_x-wx*s.gravity_z;
  const double cz=wx*s.gravity_y-wy*s.gravity_x;
  s.gravity_x-=cx*dt; s.gravity_y-=cy*dt; s.gravity_z-=cz*dt;
  auto renorm_gravity=[&](){
    const double q=std::sqrt(s.gravity_x*s.gravity_x+s.gravity_y*s.gravity_y+
                             s.gravity_z*s.gravity_z);
    if(q>1e-9 && s.gravity_mag>1e-9){
      const double k=s.gravity_mag/q;
      s.gravity_x*=k; s.gravity_y*=k; s.gravity_z*=k;
    }
  };
  renorm_gravity();
  const double amag_raw=std::sqrt(ax*ax+ay*ay+az*az);
  if(amag_raw>1e-9 && s.gravity_mag>1e-9){
    // IMU_DR_GRAVITY_OBSERVER_V2: accelerometer correction is strong only
    // while the independent visual velocity says the camera is nearly still.
    // During translation, gyro propagation carries gravity so real specific
    // force is not immediately absorbed as a tilt/gravity change.
    constexpr double kGravityTauS=1.0;
    constexpr double kVisualStillMps=0.01;
    constexpr double kVisualMovingMps=0.05;
    double correction_weight=0.0;
    if(std::isfinite(external_motion_speed_mps)){
      if(external_motion_speed_mps<=kVisualStillMps) correction_weight=1.0;
      else if(external_motion_speed_mps<kVisualMovingMps)
        correction_weight=(kVisualMovingMps-external_motion_speed_mps)/
                          (kVisualMovingMps-kVisualStillMps);
    }
    s.diag_external_motion_speed_mps=external_motion_speed_mps;
    s.diag_gravity_correction_weight=correction_weight;
    s.diag_external_zupt_allow=external_zupt_allow;
    const double alpha=correction_weight*(1.0-std::exp(-dt/kGravityTauS));
    if(alpha>0.0){
      const double k=s.gravity_mag/amag_raw;
      s.gravity_x=(1.0-alpha)*s.gravity_x+alpha*(ax*k);
      s.gravity_y=(1.0-alpha)*s.gravity_y+alpha*(ay*k);
      s.gravity_z=(1.0-alpha)*s.gravity_z+alpha*(az*k);
      renorm_gravity();
    }
  }

  // Linear specific force is formed BEFORE ATTITUDE, in the same native IMU
  // frame. Only the residual is trim-converted and rotated to NED. Therefore
  // slow FC roll/pitch evolution can no longer manufacture acceleration from
  // an otherwise unchanged accelerometer vector.
  const double rax=ax-s.gravity_x, ray=ay-s.gravity_y, raz=az-s.gravity_z;
  double rbx,rby,rbz;
  inverseAhrsTrim(rax,ray,raz,rbx,rby,rbz);
  double n,e,d_with_g;
  bodyToNed(rbx,rby,rbz,roll,pitch,yaw,n,e,d_with_g);
  s.acc_n=n; s.acc_e=e; s.acc_d=d_with_g-9.80665;
  const double amag=std::sqrt(s.acc_n*s.acc_n+s.acc_e*s.acc_e+s.acc_d*s.acc_d);
  const double gmag=std::sqrt(gx*gx+gy*gy+gz*gz);
  const bool acc_ok=amag<0.12;
  const bool gyro_ok=gmag<0.02;
  const bool imu_stationary=acc_ok&&gyro_ok;
  const bool stationary=imu_stationary&&external_zupt_allow;
  s.diag_external_motion_speed_mps=external_motion_speed_mps;
  s.diag_external_zupt_allow=external_zupt_allow;
  s.diag_zupt_active=stationary;
  s.diag_amag=amag; s.diag_gmag=gmag; s.diag_dt=dt;
  s.diag_acc_ok=acc_ok; s.diag_gyro_ok=gyro_ok; s.diag_stationary=imu_stationary;
  if(!acc_ok) ++s.diag_acc_rejects;
  if(!gyro_ok) ++s.diag_gyro_rejects;
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;
  if(s.stationary_samples>=10){s.vel_n=s.vel_e=s.vel_d=0;return;}
  s.pos_n+=s.vel_n*dt+0.5*s.acc_n*dt*dt; s.pos_e+=s.vel_e*dt+0.5*s.acc_e*dt*dt; s.pos_d+=s.vel_d*dt+0.5*s.acc_d*dt*dt;
  s.vel_n+=s.acc_n*dt; s.vel_e+=s.acc_e*dt; s.vel_d+=s.acc_d*dt;
}
}
