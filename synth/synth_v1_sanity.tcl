# synth_v1_sanity.tcl — 가장 작은 독립 모듈(sensor_pe.v, 메모리 파일 없음)로
# Vivado 배치모드 파이프라인 자체가 도는지부터 확인하는 sanity 테스트.
read_verilog rtl/sensor_pe.v
synth_design -top sensor_pe -part xc7a35ticsg324-1L
report_utilization -file util_v1_sensor_pe.rpt
report_timing_summary -file timing_v1_sensor_pe.rpt -max_paths 5
puts "SANITY SYNTH OK"
