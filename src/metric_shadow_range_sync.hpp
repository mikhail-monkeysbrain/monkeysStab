#pragma once

#include "runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace metric_shadow_sync {

struct RangeAtTime {
  bool valid=false;
  double distance_m=0.0;
  double bracket_gap_ms=-1.0;
  double nearest_sample_ms=-1.0;
};

// Interpolate only inside a real pair of TF-Luna receive timestamps.  We do
// not extrapolate the latest sample across a camera timestamp because that
// would hide an unknown temporal relation between range and image capture.
// recv_ns is RPi CLOCK_MONOTONIC receive time, not a hardware measurement
// timestamp; transport latency therefore remains an explicit limitation.
inline RangeAtTime interpolateRange(const std::deque<LunaSample>& history,
                                    int64_t target_ns,
                                    double max_bracket_gap_ms=40.0,
                                    double max_nearest_sample_ms=25.0){
  RangeAtTime out;
  if(history.size()<2 || target_ns<=0) return out;

  auto hi=std::lower_bound(
    history.begin(),history.end(),target_ns,
    [](const LunaSample& s,int64_t t){ return s.recv_ns<t; });
  if(hi==history.begin() || hi==history.end()) return out;
  const auto lo=std::prev(hi);
  if(lo->recv_ns<=0 || hi->recv_ns<=lo->recv_ns) return out;
  if(!(lo->distance_m>0.05) || !(hi->distance_m>0.05) ||
     !std::isfinite(lo->distance_m) || !std::isfinite(hi->distance_m)) return out;

  const double gap_ms=(hi->recv_ns-lo->recv_ns)*1e-6;
  const double lo_ms=(target_ns-lo->recv_ns)*1e-6;
  const double hi_ms=(hi->recv_ns-target_ns)*1e-6;
  const double nearest_ms=std::min(lo_ms,hi_ms);
  out.bracket_gap_ms=gap_ms;
  out.nearest_sample_ms=nearest_ms;
  if(gap_ms>max_bracket_gap_ms || nearest_ms>max_nearest_sample_ms) return out;

  const double alpha=(double)(target_ns-lo->recv_ns)/
                     (double)(hi->recv_ns-lo->recv_ns);
  out.distance_m=lo->distance_m + alpha*(hi->distance_m-lo->distance_m);
  out.valid=std::isfinite(out.distance_m) && out.distance_m>0.05;
  return out;
}

} // namespace metric_shadow_sync
