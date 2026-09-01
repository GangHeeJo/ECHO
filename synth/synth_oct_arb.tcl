# synth_oct_arb.tcl — particle_scorer_oct_arb(arbiter8로 single-port table_mem
# 하나를 파티클 8개가 시분할 공유) 합성. 목표: BRAM이 여전히 39/50(78%) 근처로
# 유지되는지, LUT/타이밍이 이 규모에서도 버티는지 확인.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter8.v rtl/particle_scorer_arb.v rtl/particle_scorer_oct_arb.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_oct_arb -part xc7a35ticsg324-1L
report_utilization -file util_oct_arb.rpt
report_utilization -hierarchical -file util_oct_arb_hier.rpt
report_timing_summary -file timing_oct_arb.rpt -max_paths 5
puts "OCT ARB SYNTH OK"
