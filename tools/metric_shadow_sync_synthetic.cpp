#include "../src/metric_shadow_sync.hpp"

#include <cmath>
#include <deque>
#include <iostream>

int main(){
  using metric_shadow::TimedAttitude;
  std::deque<TimedAttitude> h;
  const int64_t t0=1000000000LL;
  h.push_back({0.10,-0.20, 3.13,t0,true});
  h.push_back({0.30, 0.20,-3.13,t0+10000000LL,true});
  h.push_back({0.50, 0.40,-3.00,t0+20000000LL,true});

  bool pass=true;
  const auto mid=metric_shadow::interpolateAttitude(h,t0+5000000LL,30.0);
  std::cout<<"mid valid="<<mid.valid
           <<" roll="<<mid.attitude.roll
           <<" pitch="<<mid.attitude.pitch
           <<" yaw="<<mid.attitude.yaw
           <<" gap_ms="<<mid.bracket_gap_ms
           <<" nearest_ms="<<mid.nearest_age_ms<<"\n";
  pass &= mid.valid;
  pass &= std::abs(mid.attitude.roll-0.20)<1e-12;
  pass &= std::abs(mid.attitude.pitch-0.0)<1e-12;
  // Shortest-path yaw interpolation across +pi/-pi must stay near pi, not 0.
  pass &= std::abs(std::abs(mid.attitude.yaw)-3.14159265358979323846)<0.02;

  const auto before=metric_shadow::interpolateAttitude(h,t0-1,30.0);
  const auto after=metric_shadow::interpolateAttitude(h,t0+20000001LL,30.0);
  std::cout<<"outside before="<<before.valid<<" after="<<after.valid<<"\n";
  pass &= !before.valid && !after.valid;

  std::deque<TimedAttitude> sparse;
  sparse.push_back({0,0,0,t0,true});
  sparse.push_back({0,0,0,t0+50000000LL,true});
  const auto gap=metric_shadow::interpolateAttitude(sparse,t0+25000000LL,30.0);
  std::cout<<"wide-gap valid="<<gap.valid<<" gap_ms="<<gap.bracket_gap_ms<<"\n";
  pass &= !gap.valid;

  std::cout<<(pass?"PASS":"FAIL")<<"\n";
  return pass?0:1;
}
