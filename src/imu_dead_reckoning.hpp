#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace imu_dr {

struct State {
  bool calibrated=false;
  bool calibrating=true;
  int bias_samples=0;
  double bias_n=0,bias_e=0,bias_d=0;
  double acc_n=0,acc_e=0,acc_d=0;
  double vel_n=0,vel_e=0,vel_d=0;
  double pos_n=0,pos_e=0,pos_d=0;
  int stationary_samples=0;
  uint64_t last_time_usec=0;
};

inline void reset(State& s){ s=State{}; }

inline void bodyToNed(double ax,double ay,double az,double roll,double pitch,double yaw,
                      double& n,double& e,double& d){
  const double cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch),cy=std::cos(yaw),sy=std::sin(yaw);
  n=cy*cp*ax+(cy*sp*sr-sy*cr)*ay+(cy*sp*cr+sy*sr)*az;
  e=sy*cp*ax+(sy*sp*sr+cy*cr)*ay+(sy*sp*cr-cy*sr)*az;
  d=-sp*ax+cp*sr*ay+cp*cr*az+9.80665; // experimentally verified for FC HIGHRES_IMU convention
}

inline void update(State& s,double ax,double ay,double az,double gx,double gy,double gz,
                   double roll,double pitch,double yaw,uint64_t time_usec){
  double n,e,d; bodyToNed(ax,ay,az,roll,pitch,yaw,n,e,d);
  if(s.calibrating){
    s.bias_n+=n; s.bias_e+=e; s.bias_d+=d; ++s.bias_samples;
    if(s.bias_samples>=50){s.bias_n/=s.bias_samples;s.bias_e/=s.bias_samples;s.bias_d/=s.bias_samples;s.calibrating=false;s.calibrated=true;s.last_time_usec=time_usec;}
    return;
  }
  if(!s.calibrated||!s.last_time_usec||time_usec<=s.last_time_usec){s.last_time_usec=time_usec;return;}
  const double dt=(time_usec-s.last_time_usec)*1e-6; s.last_time_usec=time_usec;
  if(!(dt>0&&dt<0.1))return;
  s.acc_n=n-s.bias_n; s.acc_e=e-s.bias_e; s.acc_d=d-s.bias_d;
  const double amag=std::sqrt(s.acc_n*s.acc_n+s.acc_e*s.acc_e+s.acc_d*s.acc_d);
  const double gmag=std::sqrt(gx*gx+gy*gy+gz*gz);
  const bool stationary=(amag<0.12&&gmag<0.02);
  if(stationary) ++s.stationary_samples; else s.stationary_samples=0;
  if(s.stationary_samples>=10){s.vel_n=s.vel_e=s.vel_d=0;return;}
  s.pos_n+=s.vel_n*dt+0.5*s.acc_n*dt*dt; s.pos_e+=s.vel_e*dt+0.5*s.acc_e*dt*dt; s.pos_d+=s.vel_d*dt+0.5*s.acc_d*dt*dt;
  s.vel_n+=s.acc_n*dt; s.vel_e+=s.acc_e*dt; s.vel_d+=s.acc_d*dt;
}
}
