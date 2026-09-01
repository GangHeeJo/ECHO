# synth_v3_route.tcl — particle_scorer(파티클 1개)를 place&route까지 돌려서
# 진짜 구현 타이밍(post-route)을 확인한다. 지금까지의 WNS=-2.594ns(≈79MHz)는
# synth_design 직후(post-synth) 추정치였음 — 배치·배선을 실제로 안 거친 값이라
# 과도하게 낙관적이거나 비관적일 수 있음. RTL은 전혀 안 건드림.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer.v}
read_xdc constraints.xdc
synth_design -top particle_scorer -part xc7a35ticsg324-1L
opt_design
place_design
route_design
report_utilization -file util_v3_postroute.rpt
report_timing_summary -file timing_v3_postroute.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_v3_postroute_worstpath.rpt
puts "V3 POST-ROUTE OK"
