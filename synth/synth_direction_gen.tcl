# synth_direction_gen.tcl — direction_gen(CORDIC 기반 방향생성기) post-synth
# 확인. 핵심 질문: 이 설계 철학(곱셈기 없이 시프트-덧셈)이 실제 합성에서도
# DSP=0으로 나오는가?
read_verilog {rtl/cordic_sincos.v rtl/cordic_sincos_full.v rtl/angle_wrap.v rtl/direction_gen.v}
read_xdc constraints.xdc
synth_design -top direction_gen -part xc7a35ticsg324-1L
report_utilization -file util_direction_gen.rpt
report_timing_summary -file timing_direction_gen.rpt -max_paths 10
puts "DIRECTION_GEN POST-SYNTH OK"
