# synth_pair_arb.tcl — particle_scorer_pair_arb(arbiter2로 single-port table_mem
# 하나를 시분할 공유) 합성. 목표: BRAM이 particle_scorer 1개(39/50)와 비슷한
# 수준으로 유지되는지(진짜 듀얼포트 공유 시도는 156/100으로 실패했었음) 확인.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter2.v rtl/particle_scorer_arb.v rtl/particle_scorer_pair_arb.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_pair_arb -part xc7a35ticsg324-1L
report_utilization -file util_pair_arb.rpt
report_utilization -hierarchical -file util_pair_arb_hier.rpt
report_timing_summary -file timing_pair_arb.rpt -max_paths 5
puts "PAIR ARB SYNTH OK"
