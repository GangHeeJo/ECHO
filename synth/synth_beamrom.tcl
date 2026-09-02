# synth_beamrom.tcl — particle_scorer_beamrom(범용 배럴시프터 대신 beam ROM)의
# post-synth 타이밍/자원을 particle_scorer(기존 배럴시프터)와 비교하기 위한 A/B.
# RTL v3(particle_scorer.v)는 전혀 안 건드림 — 별도 모듈로 나란히 둠.
read_verilog {rtl/ray_march_edt_beamrom.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer_beamrom.v}
read_xdc constraints.xdc
synth_design -top particle_scorer_beamrom -part xc7a35ticsg324-1L
report_utilization -file util_beamrom.rpt
report_timing_summary -file timing_beamrom.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_beamrom_worstpath.rpt
puts "BEAMROM POST-SYNTH OK"
