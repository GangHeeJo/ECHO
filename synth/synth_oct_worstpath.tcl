# synth_oct_worstpath.tcl — particle_scorer_oct_arb 재합성 + 최악 경로 상세.
# 목적: 8파티클에서 WNS가 -2.438ns로 1~4개(-2.594ns)와 달랐던 이유 확인 —
# 크리티컬 패스가 여전히 ray_march_edt의 배럴 시프터인지, 아니면 8-way
# 중재/mux 쪽으로 옮겨갔는지.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter8.v rtl/particle_scorer_arb.v rtl/particle_scorer_oct_arb.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_oct_arb -part xc7a35ticsg324-1L
report_timing -delay_type max -max_paths 5 -file timing_oct_worstpath.rpt
puts "OCT WORSTPATH OK"
