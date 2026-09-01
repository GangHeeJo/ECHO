# ECHO

ZERO(F1TENTH)의 `particle_filter` 위치추정 연산을 위한 FPGA 가속기 프로젝트.
이번 학기 디지털 회로설계 수업과 연계해서 진행하는 개인 프로젝트 — ZERO 팀
레포(`~/zero/src`)와는 완전히 별개, git 미포함.

## 왜 이걸 만드는가

ZERO의 particle_filter는 매 라이다 스캔(40Hz)마다 파티클 하나당 라이다 빔 60개를
비교해서 위치 후보의 점수를 매긴다(레이마칭 + 센서모델 평가). 이 반복연산이 실제로
얼마나 비싼지, 어디가 진짜 병목인지 실측까지 마쳤다 — 자세한 근거·수치는
[`docs/problem_statement.md`](docs/problem_statement.md) 참고.

**v1 타깃: 센서모델 평가.** 레이마칭이 실제로는 더 큰 비중(90%+)을 차지하지만,
첫 RTL 프로젝트로 난이도가 적당하고 기존 문헌(Bernardi et al., DATE 2022)이 안
건드린 부분이라 여기서 시작한다. 레이마칭은 v2.

## 핵심 설계 아이디어

원본 소프트웨어는 파티클 하나의 점수를 "확률 60개를 곱해서" 계산한다. 고정소수점
하드웨어에서 곱셈을 반복하면 언더플로우가 심해지므로, **룩업테이블 자체를
log(확률)로 저장**해서 하드웨어는 매 빔마다 **룩업 + 덧셈만** 하면 되게 설계했다
(곱셈기 불필요). squash_factor 거듭제곱도 log 영역에서는 상수곱이라 그대로 살아남는다.

## 구조

```
docs/problem_statement.md   실측 기반 문제정의·목표(반드시 먼저 읽을 것)
progress.md                 세션별 진행 기록
rtl/
  sensor_pe.v                센서모델 PE 1개 — 룩업+누적, 곱셈기 없음
  table_mem.v                log(확률) 테이블 BRAM, 싱글포트 (.hex 로드)
  table_mem_dp.v              위와 같은 테이블의 듀얼포트 버전 — PE 2개가 동시에 읽음
tb/
  tb_sensor_pe.v              PE 1개, 파이썬 정답지와 대조하는 자가검증 테스트벤치 (60빔)
  tb_sensor_pe_parallel.v     PE 2개가 듀얼포트 메모리를 공유하며 파티클 2개를 동시 처리하는지 검증
sim/
  sensor_model_log_q5_8.hex   룩업테이블 데이터
  testvec_addrs.hex           테스트용 주소 목록 (파티클 5개 x 빔 60개)
  testvec_expected.hex        파티클별 정답(log-weight)
  (echo-ref/gen_sensor_model.py 산출물 — 전부 여기로 복사해서 씀)
```

파이썬 쪽 정답지 생성기(`gen_sensor_model.py`)는 Windows
`C:\SNU\포트폴리오\ZERO\echo-ref\`에 따로 있음 — ZERO의 실제 센서모델 수식을
그대로 이식해서, 테이블(.hex)과 RTL 테스트벡터(.hex)를 만들어 여기 `sim/`으로 가져온다.

## 빌드 & 실행

```bash
cd ~/echo

# PE 1개, 60빔짜리 정식 스펙 검증
iverilog -o sim/tb_sensor_pe.vvp rtl/table_mem.v rtl/sensor_pe.v tb/tb_sensor_pe.v
cd sim && vvp tb_sensor_pe.vvp && cd ..

# PE 2개 병렬 처리 검증 (듀얼포트 메모리 공유)
iverilog -o sim/tb_sensor_pe_parallel.vvp rtl/table_mem_dp.v rtl/sensor_pe.v tb/tb_sensor_pe_parallel.v
cd sim && vvp tb_sensor_pe_parallel.vvp && cd ..

gtkwave sim/tb_sensor_pe.vcd            # 단일 PE 파형
gtkwave sim/tb_sensor_pe_parallel.vcd   # 병렬 PE 파형
```

## 툴체인

- **지금(v1, 보드 없이 시뮬레이션만)**: Icarus Verilog(`iverilog`/`vvp`) + GTKWave — 둘 다 설치됨
- **나중(보드 확정되면)**: Xilinx 보드면 Vivado ML Standard, Intel/Altera 보드면
  Quartus Prime Lite — 용량이 커서 보드 정해지기 전엔 설치 안 함
- VS Code + "Verilog-HDL/SystemVerilog" 확장(Remote-WSL 창에서 별도 설치 필요)

## 현재 상태 (2026-09-01)

센서모델 PE — **실제 스펙(빔 60개)으로 검증 완료**(5/5 PASS), 그리고 **PE 2개가
듀얼포트 메모리 하나를 공유하며 파티클 2개를 동시에 처리하는 것도 검증 완료**
(두 PE 다 정답 일치 + 총 소요시간이 "파티클 1개 처리 시간"과 거의 같음 = 진짜
병렬 처리 확인). 다음은 PE를 더 늘리는 것(자원 공유 구조 재설계 필요)과
레이마칭(v2). 상세 진행 기록은 [`progress.md`](progress.md).
