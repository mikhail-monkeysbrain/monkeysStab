#!/usr/bin/env python3
"""Add a one-shot OpenCV parallel_for_ probe in the production flow thread.
Diagnostic only: does not change LK/WORKED5/FUSED parameters or results.
"""
from pathlib import Path
import sys

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="OPENCV_PARALLEL_PROBE_V1"
if marker in s:
    print("already patched",p); raise SystemExit(0)

# Need standard headers for Linux TID set.
inc_anchor="#include <opencv2/opencv.hpp>"
if inc_anchor not in s:
    print("opencv include anchor not found; source not modified",file=sys.stderr); raise SystemExit(2)
extra='''\n// OPENCV_PARALLEL_PROBE_V1 headers\n#include <mutex>\n#include <set>\n#include <sys/syscall.h>\n#include <unistd.h>\n'''
s=s.replace(inc_anchor,inc_anchor+extra,1)

# Insert at entry to the actual flow estimator so the probe runs from the same caller context.
candidates=[
 "FlowStep estimateRawFlow(",
 "static FlowStep estimateRawFlow(",
]
pos=-1
for a in candidates:
    i=s.find(a)
    if i>=0:
        brace=s.find("{",i)
        if brace>=0: pos=brace+1; break
if pos<0:
    print("estimateRawFlow() anchor not found; source not modified",file=sys.stderr); raise SystemExit(3)

probe=r'''
  // OPENCV_PARALLEL_PROBE_V1 -- one-shot diagnostic from the production flow thread.
  {
    static std::once_flag jtzero_parallel_probe_once;
    std::call_once(jtzero_parallel_probe_once, [] {
      std::mutex mu;
      std::set<long> tids;
      cv::parallel_for_(cv::Range(0, 4096), [&](const cv::Range& r) {
        const long tid = static_cast<long>(::syscall(SYS_gettid));
        {
          std::lock_guard<std::mutex> lk(mu);
          tids.insert(tid);
        }
        volatile double sink = 0.0;
        for (int i = r.start; i < r.end; ++i)
          for (int k = 0; k < 4000; ++k)
            sink += (i + 1) * 1e-12 + k * 1e-15;
        (void)sink;
      }, 64.0);
      std::cerr << "OPENCV_PARALLEL_PROBE workers=" << tids.size()
                << " configured_threads=" << cv::getNumThreads()
                << " cpus=" << cv::getNumberOfCPUs()
                << " tids=";
      for (long tid : tids) std::cerr << tid << ",";
      std::cerr << "\n";
    });
  }
'''
s=s[:pos]+probe+s[pos:]
p.write_text(s)
print("patched",p)
