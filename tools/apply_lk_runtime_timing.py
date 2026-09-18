#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Добавляет wall/thread/process CPU timing вокруг production PyrLK. Только диагностика."""
from pathlib import Path

p = Path("src/optical_flow_mavlink.cpp")
s = p.read_text()

marker = "LK_RUNTIME_TIMING_V1"
if marker in s:
    print("already applied")
    raise SystemExit(0)

# CLOCK_THREAD_CPUTIME_ID / CLOCK_PROCESS_CPUTIME_ID.
if "#include <time.h>" not in s:
    # Do not depend on the local include layout: this source has diverged from
    # remote main. A preprocessing directive is valid at the start of the TU.
    s = "#include <time.h>\n" + s

old = """  const auto lk0=std::chrono::steady_clock::now();
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,
                           cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),
                           0,1e-4);
  const auto lk1=std::chrono::steady_clock::now();
  s.t_lk_ms=std::chrono::duration<double,std::milli>(lk1-lk0).count();"""

new = """  // LK_RUNTIME_TIMING_V1
  // Diagnostic only. LK inputs/parameters/results are unchanged.
  timespec lk_thr0{}, lk_thr1{}, lk_proc0{}, lk_proc1{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID,&lk_thr0);
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&lk_proc0);
  const auto lk0=std::chrono::steady_clock::now();
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,
                           cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),
                           0,1e-4);
  const auto lk1=std::chrono::steady_clock::now();
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&lk_proc1);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID,&lk_thr1);
  s.t_lk_ms=std::chrono::duration<double,std::milli>(lk1-lk0).count();
  const auto lk_ts_ms=[](const timespec& a,const timespec& b){
    return double(b.tv_sec-a.tv_sec)*1000.0+
           double(b.tv_nsec-a.tv_nsec)/1000000.0;
  };
  const double lk_thread_cpu_ms=lk_ts_ms(lk_thr0,lk_thr1);
  const double lk_process_cpu_ms=lk_ts_ms(lk_proc0,lk_proc1);
  if(s.t_lk_ms>20.0){
    std::cerr<<"LK_RUNTIME wall_ms="<<s.t_lk_ms
             <<" thread_cpu_ms="<<lk_thread_cpu_ms
             <<" process_cpu_ms="<<lk_process_cpu_ms
             <<" features="<<p0.size()<<"\\n";
  }"""

if old not in s:
    raise SystemExit("production LK anchor not found; source not modified")

s = s.replace(old, new, 1)
p.write_text(s)
print("patched", p)
