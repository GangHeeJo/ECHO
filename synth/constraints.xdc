# 100MHz(10ns) 목표 클럭 — 지금까지 시뮬레이션 테스트벤치가 써온 클럭 주기(#5 clk=~clk)와 동일
create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]
