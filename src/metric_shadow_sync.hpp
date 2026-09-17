#pragma once

#include "metric_odometry_shadow.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace metric_shadow {

struct TimedAttitude {
  double roll=0.0,pitch=0.0,yaw=0.0;
  int64_t sample_ns=0;
  bool valid=false;
};

struct AttitudeLookup {
  Attitude attitude{};
  bool valid=false;
  double bracket_gap_ms=-1.0;
  double nearest_age_ms=-1.0;
};

inline double wrapPi(double x){
  constexpr double pi=3.14159265358979323846;
  while(x>pi) x-=2.0*pi;
  while(x<-pi) x+=2.0*pi;
  return x;
}

inline double interpAngle(double a,double b,double u){
  return wrapPi(a+u*wrapPi(b-a));
}

// Interpolate only when the requested camera timestamp is actually bracketed
// by two attitude samples.  No nearest-sample extrapolation is allowed: a
// missing temporal link is a measurement gap, not zero angular motion.
inline AttitudeLookup interpolateAttitude(const std::deque<TimedAttitude>& h,
                                          int64_t t_ns,
                                          double max_bracket_gap_ms=30.0){
  AttitudeLookup out;
  if(h.size()<2 || t_ns<=0) return out;

  auto hi=std::lower_bound(h.begin(),h.end(),t_ns,
    [](const TimedAttitude& a,int64_t t){ return a.sample_ns<t; });
  if(hi==h.end()) return out;
  if(hi->sample_ns==t_ns && hi->valid){
    out.attitude={hi->roll,hi->pitch,hi->yaw,hi->sample_ns,true};
    out.valid=true;
    out.bracket_gap_ms=0.0;
    out.nearest_age_ms=0.0;
    return out;
  }
  if(hi==h.begin()) return out;
  const auto lo=std::prev(hi);
  if(!lo->valid || !hi->valid || hi->sample_ns<=lo->sample_ns) return out;

  const double gap_ms=(hi->sample_ns-lo->sample_ns)*1e-6;
  const double nearest_ms=std::min(t_ns-lo->sample_ns,hi->sample_ns-t_ns)*1e-6;
  out.bracket_gap_ms=gap_ms;
  out.nearest_age_ms=nearest_ms;
  if(!(gap_ms>=0.0 && gap_ms<=max_bracket_gap_ms)) return out;

  const double u=(double)(t_ns-lo->sample_ns)/(double)(hi->sample_ns-lo->sample_ns);
  if(!(u>=0.0 && u<=1.0)) return out;
  out.attitude.roll=interpAngle(lo->roll,hi->roll,u);
  out.attitude.pitch=interpAngle(lo->pitch,hi->pitch,u);
  out.attitude.yaw=interpAngle(lo->yaw,hi->yaw,u);
  out.attitude.sample_ns=t_ns;
  out.attitude.valid=true;
  out.valid=true;
  return out;
}

} // namespace metric_shadow
