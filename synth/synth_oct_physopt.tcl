# synth_oct_physopt.tcl — synth_oct_route.tcl과 완전히 동일한 흐름 + route_design
# 뒤에 post-route phys_opt_design 한 번 추가(RTL 변경 없음, tcl 한 줄 A/B).
# 사용자가 GUI 타이밍 리포트에서 직접 확인: Path 1(u_scorer2/u_ray_march)의
# 지연 74%가 net delay(fanout 310인 y_reg가 넓게 퍼져서) — phys_opt_design은
# 배치 이후 실제 배선 지연 정보를 갖고 high-fanout net 복제/재배치 등을
# 시도하는 단계라, 이 문제에 정확히 맞는 최적화 시도.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter8.v rtl/particle_scorer_arb.v rtl/particle_scorer_oct_arb.v}
read_xdc constraints.xdc
synth_design -mode out_of_context -top particle_scorer_oct_arb -part xc7a35ticsg324-1L
opt_design
place_design
route_design
phys_opt_design -directive AggressiveExplore
report_utilization -file util_oct_physopt.rpt
report_timing_summary -file timing_oct_physopt.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_oct_physopt_worstpath.rpt
puts "OCT PHYS_OPT OK"
