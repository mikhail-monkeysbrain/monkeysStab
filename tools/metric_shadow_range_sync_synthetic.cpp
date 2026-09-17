#include "metric_shadow_range_sync.hpp"

#include <cmath>
#include <deque>
#include <iostream>

int main(){
  using metric_shadow_sync::interpolateRange;
  constexpr int64_t ms=1000000LL;
  std::deque<LunaSample> h{
    {0.190,100,1000*ms},
    {0.200,100,1010*ms},
    {0.220,100,1020*ms}
  };

  bool pass=true;
  auto mid=interpolateRange(h,1015*ms);
  std::cout<<"mid valid="<<mid.valid
           <<" range="<<mid.distance_m
           <<" gap_ms="<<mid.bracket_gap_ms
           <<" nearest_ms="<<mid.nearest_sample_ms<<"\n";
  pass &= mid.valid && std::abs(mid.distance_m-0.210)<1e-12;

  auto before=interpolateRange(h,999*ms);
  auto after=interpolateRange(h,1021*ms);
  std::cout<<"outside before="<<before.valid<<" after="<<after.valid<<"\n";
  pass &= !before.valid && !after.valid;

  std::deque<LunaSample> wide{
    {0.190,100,1000*ms},
    {0.200,100,1060*ms}
  };
  auto bad_gap=interpolateRange(wide,1030*ms);
  std::cout<<"wide-gap valid="<<bad_gap.valid
           <<" gap_ms="<<bad_gap.bracket_gap_ms<<"\n";
  pass &= !bad_gap.valid;

  std::deque<LunaSample> far{
    {0.190,100,1000*ms},
    {0.200,100,1030*ms}
  };
  auto bad_nearest=interpolateRange(far,1029*ms,40.0,0.5);
  std::cout<<"far-nearest valid="<<bad_nearest.valid
           <<" nearest_ms="<<bad_nearest.nearest_sample_ms<<"\n";
  pass &= !bad_nearest.valid;

  std::cout<<(pass?"PASS":"FAIL")<<"\n";
  return pass?0:1;
}
