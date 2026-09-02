# synth_beamrom_route.tcl — beam ROM 버전을 place&route까지 돌려서 post-synth
# 추정치(WNS -2.418ns)를 실제 배치·배선 결과로 확인. particle_scorer(기존
# 배럴시프터)의 post-route(synth_v3_route.tcl, WNS -2.370ns)와 같은 조건 비교.
read_verilog {rtl/ray_march_edt_beamrom.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer_beamrom.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_beamrom -part xc7a35ticsg324-1L
opt_design
place_design
route_design
report_utilization -file util_beamrom_postroute.rpt
report_timing_summary -file timing_beamrom_postroute.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_beamrom_postroute_worstpath.rpt
puts "BEAMROM POST-ROUTE OK"
