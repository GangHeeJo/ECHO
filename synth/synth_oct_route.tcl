# synth_oct_route.tcl — 최종 후보인 8-way(particle_scorer_oct_arb)만 post-route까지
# 실행. 2/4-way는 이미 "새 병목 아님"으로 확인됐으니 재검증 생략(사용자 지시).
# ⚠️ 1차 시도(모드 없이)는 place_design에서 실패함: oct_arb는 파티클 8개분
# 포트(dx/dy/r/x0y0/각종 start·done)를 전부 top-level 포트로 노출해 850개 —
# 이 칩의 실제 사용가능 핀(210개)보다 훨씬 많아서 "칩의 최종 톱"으로는 배치가
# 물리적으로 불가능(정상 — oct_arb는 원래 더 큰 설계 안에 들어갈 서브모듈이지
# 칩 경계가 아님). -mode out_of_context로 IO 버퍼 삽입/핀 제약을 끄고 서브모듈
# 취급하면 배치·배선은 되고, 내부 로직의 진짜 post-route 타이밍은 그대로 나옴.
read_verilog {rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter8.v rtl/particle_scorer_arb.v rtl/particle_scorer_oct_arb.v}
read_xdc constraints.xdc
synth_design -mode out_of_context -top particle_scorer_oct_arb -part xc7a35ticsg324-1L
opt_design
place_design
route_design
report_utilization -file util_oct_postroute.rpt
report_timing_summary -file timing_oct_postroute.rpt -max_paths 10
report_timing -delay_type max -max_paths 5 -file timing_oct_postroute_worstpath.rpt
puts "OCT POST-ROUTE OK"
