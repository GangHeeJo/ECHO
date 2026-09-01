# synth_v3_timed.tcl — v3(particle_scorer) 합성 + 100MHz 클럭 제약을 걸고
# 실제 타이밍 리포트(최대 동작 속도)까지 뽑는다.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer.v}
read_xdc constraints.xdc
synth_design -top particle_scorer -part xc7a35ticsg324-1L
report_utilization -file util_v3_timed.rpt
report_timing_summary -file timing_v3_timed.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_v3_worstpath.rpt
puts "V3 TIMED SYNTH OK"
