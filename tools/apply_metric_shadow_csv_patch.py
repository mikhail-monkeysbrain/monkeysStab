#!/usr/bin/env python3
from pathlib import Path

p = Path("src/optical_flow_mavlink.cpp")
s = p.read_text()

# This patch is deliberately anchored to the current diagnostic runtime block.
# It does not modify the production OPTICAL_FLOW publisher or metric estimator.

old = """          metric_shadow_integrator.consume(ms);\n          ++metric_shadow_interval;\n"""
new = """          // Do not mark startup warm-up as a trajectory gap. The metric trajectory\n          // starts on the first valid synchronized interval. After that, every rejected\n          // interval remains visible through Integrator::complete/gap_time_s.\n          static bool metric_shadow_started=false;\n          const bool metric_shadow_startup_wait =\n              !metric_shadow_started && !ms.valid &&\n              (ms.reason==metric_shadow::RejectReason::BAD_ATTITUDE ||\n               ms.reason==metric_shadow::RejectReason::BAD_RANGE);\n          if(ms.valid) metric_shadow_started=true;\n          if(!metric_shadow_startup_wait) metric_shadow_integrator.consume(ms);\n          ++metric_shadow_interval;\n\n          // Per-interval shadow log. This is intentionally independent of the\n          // production optical_flow_mavlink.csv and never affects MAVLink output.\n          static std::ofstream metric_shadow_csv;\n          static bool metric_shadow_csv_header=false;\n          if(!metric_shadow_csv.is_open()){\n            const std::filesystem::path prod_csv_path(csv_path);\n            const auto shadow_path=prod_csv_path.parent_path()/\"metric_shadow.csv\";\n            metric_shadow_csv.open(shadow_path, std::ios::out|std::ios::trunc);\n          }\n          if(metric_shadow_csv.is_open()){\n            if(!metric_shadow_csv_header){\n              metric_shadow_csv\n                <<\"interval_id,t0_ns,t1_ns,started,startup_wait,valid,reason,pairs,used,\"\n                <<\"dN_m,dE_m,dD_m,pN_m,pE_m,pD_m,accepted,rejected,complete,\"\n                <<\"accepted_time_s,gap_time_s,att0_valid,att1_valid,att0_gap_ms,att1_gap_ms,\"\n                <<\"range0_valid,range1_valid,range0_m,range1_m,range0_gap_ms,range1_gap_ms,\"\n                <<\"residual_median_m,residual_mad_m\\n\";\n              metric_shadow_csv_header=true;\n            }\n            metric_shadow_csv\n              <<metric_shadow_interval<<','<<prev_ts<<','<<ts<<','\n              <<(metric_shadow_started?1:0)<<','<<(metric_shadow_startup_wait?1:0)<<','\n              <<(ms.valid?1:0)<<','<<metric_shadow::rejectReasonName(ms.reason)<<','\n              <<flow.metric_prev_points.size()<<','<<ms.points<<','\n              <<ms.delta_local_m[0]<<','<<ms.delta_local_m[1]<<','<<ms.delta_local_m[2]<<','\n              <<metric_shadow_integrator.position_m[0]<<','\n              <<metric_shadow_integrator.position_m[1]<<','\n              <<metric_shadow_integrator.position_m[2]<<','\n              <<metric_shadow_integrator.accepted<<','<<metric_shadow_integrator.rejected<<','\n              <<(metric_shadow_integrator.complete?1:0)<<','\n              <<metric_shadow_integrator.accepted_time_s<<','<<metric_shadow_integrator.gap_time_s<<','\n              <<(a0.valid?1:0)<<','<<(a1.valid?1:0)<<','<<a0.bracket_gap_ms<<','<<a1.bracket_gap_ms<<','\n              <<(r0.valid?1:0)<<','<<(r1.valid?1:0)<<','<<r0.distance_m<<','<<r1.distance_m<<','\n              <<r0.bracket_gap_ms<<','<<r1.bracket_gap_ms<<','\n              <<ms.residual_median_m<<','<<ms.residual_mad_m<<'\\n';\n            metric_shadow_csv.flush();\n          }\n"""
if s.count(old) != 1:
    raise SystemExit(f"runtime consume anchor count={s.count(old)}; expected 1")
s = s.replace(old, new)

# std::filesystem is needed only for deriving metric_shadow.csv next to the
# existing production CSV. Add it once near the standard includes.
inc_anchor = "#include <fstream>\n"
if "#include <filesystem>" not in s:
    if s.count(inc_anchor) != 1:
        raise SystemExit(f"fstream include anchor count={s.count(inc_anchor)}; expected 1")
    s = s.replace(inc_anchor, inc_anchor + "#include <filesystem>\n")

p.write_text(s)
print("PATCH OK: per-interval metric_shadow.csv + startup warm-up gate")
