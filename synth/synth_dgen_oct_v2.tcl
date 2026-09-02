# synth_dgen_oct_v2.tcl — theta_wrap.v가 통합된 particle_scorer_dgen_oct_arb
# post-route. theta_wrap은 작은 반복형 비교+덧셈이라 크리티컬 패스나 자원에
# 큰 영향은 없을 것으로 예상하지만, 추정 대신 실측으로 확정한다.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/cordic_sincos.v rtl/cordic_sincos_full.v rtl/angle_wrap.v rtl/theta_wrap.v rtl/direction_gen.v rtl/arbiter8.v rtl/particle_scorer_dgen_arb.v rtl/particle_scorer_dgen_oct_arb.v}
read_xdc constraints.xdc
synth_design -mode out_of_context -top particle_scorer_dgen_oct_arb -part xc7a35ticsg324-1L
opt_design
place_design
route_design
report_utilization -file util_dgen_oct_v2.rpt
report_timing_summary -file timing_dgen_oct_v2.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_dgen_oct_v2_worstpath.rpt
puts "DGEN OCT V2 POST-ROUTE OK"
