# synth_quad_arb.tcl — particle_scorer_quad_arb(arbiter4로 single-port table_mem
# 하나를 파티클 4개가 시분할 공유) 합성. 목표: BRAM이 여전히 39/50(78%) 근처로
# 유지되는지(파티클 1개/2개와 동일) 확인.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter4.v rtl/particle_scorer_arb.v rtl/particle_scorer_quad_arb.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_quad_arb -part xc7a35ticsg324-1L
report_utilization -file util_quad_arb.rpt
report_utilization -hierarchical -file util_quad_arb_hier.rpt
report_timing_summary -file timing_quad_arb.rpt -max_paths 5
puts "QUAD ARB SYNTH OK"
