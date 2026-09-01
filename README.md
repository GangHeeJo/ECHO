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
  table_mem.v                log(확률) 테이블 BRAM (.hex 로드)
tb/
  tb_sensor_pe.v              파이썬 정답지와 대조하는 자가검증 테스트벤치
sim/
  sensor_model_log_q5_8.hex   룩업테이블 데이터 (echo-ref/gen_sensor_model.py 산출물)
  tb_sensor_pe.vcd            시뮬레이션 파형 (GTKWave용, 실행할 때마다 갱신됨)
```

파이썬 쪽 정답지 생성기(`gen_sensor_model.py`)는 Windows
`C:\SNU\포트폴리오\ZERO\echo-ref\`에 따로 있음 — ZERO의 실제 센서모델 수식을
그대로 이식해서, 테이블(.hex)과 RTL 테스트벡터를 만들어 여기 `sim/`으로 가져온다.

## 빌드 & 실행

```bash
cd ~/echo
iverilog -o sim/tb_sensor_pe.vvp rtl/table_mem.v rtl/sensor_pe.v tb/tb_sensor_pe.v
cd sim
vvp tb_sensor_pe.vvp          # PASS/FAIL 결과 출력
gtkwave tb_sensor_pe.vcd      # 파형 직접 보기 (선택)
```

## 툴체인

- **지금(v1, 보드 없이 시뮬레이션만)**: Icarus Verilog(`iverilog`/`vvp`) + GTKWave — 둘 다 설치됨
- **나중(보드 확정되면)**: Xilinx 보드면 Vivado ML Standard, Intel/Altera 보드면
  Quartus Prime Lite — 용량이 커서 보드 정해지기 전엔 설치 안 함
- VS Code + "Verilog-HDL/SystemVerilog" 확장(Remote-WSL 창에서 별도 설치 필요)

## 현재 상태 (2026-09-01)

센서모델 PE 1개, 5개 파티클(8빔 장난감 시나리오)로 시뮬레이션 검증 완료 — 파이썬
정답지와 비트 단위로 정확히 일치(5/5 PASS). 다음은 실제 스펙(60빔)으로 확장,
그다음 PE 여러 개 병렬화. 상세 진행 기록은 [`progress.md`](progress.md).
