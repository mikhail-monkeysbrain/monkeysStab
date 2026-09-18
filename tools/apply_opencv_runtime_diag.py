#!/usr/bin/env python3
"""Patch local production source with startup-only OpenCV runtime diagnostics.

No optical-flow algorithm, LK parameters, WORKED5 or FUSED behavior is changed.
"""
from pathlib import Path
import sys

p=Path("src/optical_flow_mavlink.cpp")
s=p.read_text()
marker="OPENCV_RUNTIME_DIAG_V1"
if marker in s:
    print("already patched", p)
    raise SystemExit(0)

# Insert immediately after main() opening. Handle common local signatures.
candidates=[
    "int main(int argc, char** argv) {",
    "int main(int argc,char** argv){",
    "int main(int argc, char **argv) {",
]
anchor=next((x for x in candidates if x in s),None)
if not anchor:
    print("main() anchor not found; source not modified", file=sys.stderr)
    raise SystemExit(2)

diag=r'''
  // OPENCV_RUNTIME_DIAG_V1 -- startup diagnostics only.
  std::cerr << "OPENCV_RUNTIME threads=" << cv::getNumThreads()
            << " cpus=" << cv::getNumberOfCPUs()
            << " optimized=" << (cv::useOptimized() ? 1 : 0)
            << "\n";
'''
s=s.replace(anchor,anchor+diag,1)
p.write_text(s)
print("patched", p)
