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
건드린 부분이라 여기서 시작했다. **v2: 레이마칭** — v1 완주 후 착수, 진행 중.

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
  addr_gen.v                  r*301+d 주소를 시프트+덧셈만으로 계산(곱셈기 없음)
  ray_march.v                 [v2.0] 방향벡터 따라 격자 지도를 걸어 벽까지 거리 재기 (20x20 테스트지도, 2차원 배열 인덱싱)
  ray_march_bram.v             [v2.1] 위와 같은 알고리즘, 실제 changwon 트랙(400x160칸) — 평면(flat) BRAM 주소 계산으로 교체
  particle_scorer.v            [v3] v1+v2 결합 — 빔마다 레이마칭으로 기대거리 구하고 센서모델로 채점, 파티클 전체 점수까지
  ray_march_edt.v               [v2.2] 거리장(EDT) 기반 마칭 — 빈 공간에서 2^k칸씩 성큼성큼(배럴 시프터, 곱셈기 없음)
tb/
  tb_sensor_pe.v              PE 1개, 파이썬 정답지와 대조하는 자가검증 테스트벤치 (60빔)
  tb_sensor_pe_parallel.v     PE 2개가 듀얼포트 메모리를 공유하며 파티클 2개를 동시 처리하는지 검증
  tb_sensor_pe_x4.v           PE 4개(테이블 복사본 2개, generate로 인스턴스화), 파티클 4개 동시 처리 검증
  tb_sensor_pe_seq4.v         PE 1개로 파티클 4개를 순서대로(대조군) — 병렬 대비 속도 비교용
  tb_addr_gen.v                addr_gen 단독 검증(손으로 계산한 값과 대조)
  tb_sensor_pe_addrgen.v      addr_gen을 실제로 PE 앞단에 연결 — 주소를 하드웨어가 직접 계산하는 전체 파이프라인 검증
  tb_ray_march.v               레이마칭 v2.0 — 벽/경계/대각선/최대거리 4가지 케이스 검증
  tb_ray_march_bram.v          레이마칭 v2.1 — 실제 changwon 트랙에서 6가지 방향 검증
  tb_particle_scorer.v         v3 — 파티클 1개, 빔 8개, 레이마칭+센서모델 전체 파이프라인 검증
  tb_particle_scorer_parallel.v v3 병렬 — particle_scorer 2개 동시 처리, 소요시간이 단일 처리와 동일함을 확인
  tb_ray_march_edt.v            거리장 기반 마칭 검증 — 같은 6케이스에서 기존 대비 클럭 수 직접 대조(최대 6배 이상 감소)
tools/
  gen_track_map.py             changwon map.pgm -> 지도 .hex + 파이썬으로 미리 계산한 테스트 시나리오 정답
  gen_particle_scorer_test.py  particle_scorer 통합 테스트용 정답지(위 두 스크립트를 그대로 import해서 재사용)
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

## 현재 상태 (2026-09-02)

- **v1(센서모델) 완료**: 실제 스펙(빔 60개) 검증, PE 2개/4개 병렬화(4배 속도향상을
  순차 대조군과 직접 비교해 실측 확정: 2560ns → 640ns), 주소생성(`r*301+d`)까지
  곱셈기 없이 하드웨어가 직접 계산.
- **v2(레이마칭) 완료**: 20x20 테스트지도 → 실제 changwon 트랙(400x160칸) 확장
  (비트폭 오버플로 버그 실측으로 수정) → **거리장(EDT) 기반 최적화**(`ray_march_edt.v`,
  가장 가까운 벽까지 안전거리를 2의 거듭제곱으로 반올림해 배럴 시프터로 한 번에
  전진 — 곱셈기 없이 큰 보폭, 같은 6케이스에서 클럭 수 최대 6배 이상 감소).
- **v3(v1+v2 결합) 완료**: `particle_scorer.v` — 빔마다 레이마칭으로 기대거리를
  구하고 센서모델로 채점, 파티클 전체 점수까지. 파티클 1개 검증(`weight_o=-7441`)
  → **2개 동시 병렬 처리**(전체 파이프라인 단위 병렬화 확인) → **내부 레이마칭을
  ray_march_edt(거리장 기반)로 교체 — 단일 처리 4305ns→1315ns(약 3.3배), 병렬
  처리도 1310ns로 여전히 단일 처리와 동일(최적화+병렬화 효과가 서로 방해 없이
  공존).** 빔별 처리시간 로그에서 "먼 거리 빔이 압도적으로 오래 걸린다"(레이
  마칭이 전체 비용의 90%라던 최초 실측 결과)가 시뮬레이션 타이밍으로도 그대로
  확인됨.

상세 진행 기록은 [`progress.md`](progress.md).
