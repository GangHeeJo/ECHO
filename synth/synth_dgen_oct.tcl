# synth_dgen_oct.tcl — direction_gen 내장 8-way(particle_scorer_dgen_oct_arb)
# post-route. 지금까지의 1.238ms 추정은 기존 oct_arb(외부 dx,dy)의 클럭
# (79.0MHz)을 빌려온 값 — 이 설계 자체의 진짜 클럭과 DSP=0을 확인한다.
# oct_arb와 동일 이유(포트 수 초과)로 out_of_context 필요.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/cordic_sincos.v rtl/cordic_sincos_full.v rtl/angle_wrap.v rtl/direction_gen.v rtl/arbiter8.v rtl/particle_scorer_dgen_arb.v rtl/particle_scorer_dgen_oct_arb.v}
read_xdc constraints.xdc
synth_design -mode out_of_context -top particle_scorer_dgen_oct_arb -part xc7a35ticsg324-1L
opt_design
place_design
route_design
report_utilization -file util_dgen_oct.rpt
report_timing_summary -file timing_dgen_oct.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_dgen_oct_worstpath.rpt
puts "DGEN OCT POST-ROUTE OK"
