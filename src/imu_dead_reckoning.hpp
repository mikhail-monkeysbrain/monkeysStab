#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace imu_dr {

struct State {
  bool calibrated=false;
  bool calibrating=true;
  int bias_samples=0;
  // Accelerometer bias must stay in the sensor/body frame.  A bias stored
  // in NED becomes yaw-dependent after the airframe rotates.
  double bias_bx=0,bias_by=0,bias_bz=0;
  double acc_n=0,acc_e=0,acc_d=0;
  double vel_n=0,vel_e=0,vel_d=0;
  double pos_n=0,pos_e=0,pos_d=0;
  int stationary_samples=0;
  double diag_amag=0,diag_gmag=0,diag_dt=0;
  bool diag_acc_ok=false,diag_gyro_ok=false,diag_stationary=false;
  uint64_t diag_acc_rejects=0,diag_gyro_rejects=0;
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
                   bool external_zupt_allow=true){
  // Calibrate the accelerometer residual in BODY coordinates.  During the
  // stationary startup calibration the ideal specific-force vector in body is
  // R_ned_to_body * [0,0,-g] for the HIGHRES_IMU convention used below.
  // Subtracting that ideal vector leaves a body-fixed sensor bias, which then
  // rotates correctly with the airframe on every subsequent sample.
  // First put HIGHRES_IMU into the same vehicle/body frame as ATTITUDE.
  double tax,tay,taz;
  inverseAhrsTrim(ax,ay,az,tax,tay,taz);

  if(s.calibrating){
    double ideal_ax,ideal_ay,ideal_az;
    const double cr=std::cos(roll),sr=std::sin(roll);
    const double cp=std::cos(pitch),sp=std::sin(pitch);
    ideal_ax= 9.80665*sp;
    ideal_ay=-9.80665*cp*sr;
    ideal_az=-9.80665*cp*cr;
    s.bias_bx+=tax-ideal_ax;
    s.bias_by+=tay-ideal_ay;
    s.bias_bz+=taz-ideal_az;
    ++s.bias_samples;
    if(s.bias_samples>=50){
      s.bias_bx/=s.bias_samples; s.bias_by/=s.bias_samples; s.bias_bz/=s.bias_samples;
      s.calibrating=false; s.calibrated=true; s.last_time_usec=time_usec;
    }
    return;
  }
  if(!s.calibrated||!s.last_time_usec||time_usec<=s.last_time_usec){s.last_time_usec=time_usec;return;}
  const double dt=(time_usec-s.last_time_usec)*1e-6; s.last_time_usec=time_usec;
  if(!(dt>0&&dt<0.1))return;
  double n,e,d;
  bodyToNed(tax-s.bias_bx,tay-s.bias_by,taz-s.bias_bz,roll,pitch,yaw,n,e,d);
  s.acc_n=n; s.acc_e=e; s.acc_d=d;
  const double amag=std::sqrt(s.acc_n*s.acc_n+s.acc_e*s.acc_e+s.acc_d*s.acc_d);
  const double gmag=std::sqrt(gx*gx+gy*gy+gz*gz);
  const bool acc_ok=amag<0.12;
  const bool gyro_ok=gmag<0.02;
  const bool imu_stationary=acc_ok&&gyro_ok;
  const bool stationary=imu_stationary&&external_zupt_allow;
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
