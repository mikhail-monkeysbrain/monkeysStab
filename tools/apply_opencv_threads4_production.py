#!/usr/bin/env python3
"""Switch production OpenCV from forced 1 thread to 4 threads.
Changes only cv::setNumThreads(1) -> cv::setNumThreads(4).
"""
from pathlib import Path
import sys
p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
old="cv::setNumThreads(1);"
new="cv::setNumThreads(4);"
n=s.count(old)
if n!=1:
    print(f"expected exactly one production anchor, found {n}; source not modified",file=sys.stderr)
    raise SystemExit(2)
s=s.replace(old,new,1)
p.write_text(s)
print("patched",p,": cv::setNumThreads(1) -> cv::setNumThreads(4)")
