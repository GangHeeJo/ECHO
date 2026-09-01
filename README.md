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
  particle_scorer_shared.v      [v3.1, 폐기] 테이블을 외부 포트로 뺀 버전 — table_mem_dp로 진짜 듀얼포트 공유 시도, 합성기가 통째 복사해 BRAM 2배로 실패
  particle_scorer_pair.v        [v3.1, 폐기] particle_scorer_shared 2개 + table_mem_dp — 합성 실패(BRAM 156/100)로 폐기, 코드는 실패 사례로 보존
  particle_scorer_arb.v         [v3.2] particle_scorer_shared를 중재(arbiter) 방식으로 재설계 — 공유 테이블 포트에 req/gnt 인터페이스 추가
  particle_scorer_pair_arb.v    [v3.2] particle_scorer_arb 2개가 arbiter2(전국AI반도체경진대회 프로젝트 재사용)로 single-port table_mem 하나를 시분할 — BRAM 추가비용 0으로 합성 성공
  arbiter2.v                    전국AI반도체경진대회 프로젝트에서 그대로 재사용 — 2-input round-robin 중재기(안 건드림)
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
  tb_particle_scorer_x4.v       v3 4배 병렬 — particle_scorer 4개 동시 처리(1520ns, 단일 처리와 비슷)
  tb_ray_march_edt.v            거리장 기반 마칭 검증 — 같은 6케이스에서 기존 대비 클럭 수 직접 대조(최대 6배 이상 감소)
  tb_particle_scorer_pair.v     [폐기] particle_scorer_pair(진짜 듀얼포트 공유) 기능검증 — 시뮬은 통과하나 합성이 실패해 폐기된 경로
  tb_particle_scorer_pair_arb.v v3.2 — particle_scorer_pair_arb(중재 공유) 기능검증, 파티클 2개 다 PASS
tools/
  gen_track_map.py             changwon map.pgm -> 지도 .hex + 파이썬으로 미리 계산한 테스트 시나리오 정답
  gen_particle_scorer_test.py  particle_scorer 통합 테스트용 정답지(위 두 스크립트를 그대로 import해서 재사용)
synth/
  constraints.xdc              100MHz 클럭 제약(합성 타이밍 분석용)
  synth_v1_sanity.tcl          가장 작은 모듈(sensor_pe)로 Vivado 배치모드 파이프라인 자체를 검증
  synth_v3_timed.tcl           particle_scorer 실제 합성(Arty A7-35T 타깃) + 타이밍 리포트
  util_v3_timed.rpt            자원 사용량 실측(LUT 6.4%/FF 0.61%/BRAM 78%/DSP 0%)
  timing_v3_timed.rpt          타이밍 서머리(실제 최대 클럭 ≈ 79MHz)
  timing_v3_worstpath.rpt      최악 경로 상세(크리티컬 패스 = ray_march_edt 배럴 시프터)
  synth_pair_arb.tcl            particle_scorer_pair_arb 합성(파티클 2개, 테이블 공유)
  util_pair_arb.rpt             자원 사용량 실측(BRAM 39/50 — 파티클 1개였을 때와 동일)
  util_pair_arb_hier.rpt        모듈별 BRAM 내역(테이블 하나가 BRAM 100% 차지 확인)
  timing_pair_arb.rpt           타이밍(WNS=-2.594ns, 단일 버전과 동일 — 중재 로직이 크리티컬 패스 안 건드림)
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
  → 2개 동시 병렬 처리 → 내부 레이마칭을 ray_march_edt(거리장 기반)로 교체(단일
  처리 4305ns→1315ns, 약 3.3배) → **파티클 4개 동시 병렬 처리로 확장(1520ns,
  단일 처리와 비슷한 수준 — 4배 병렬 확인)**. 이 과정에서 정답지(오라클) 버그를
  하나 실제로 찾아 고침: 파이썬 정답지가 방향벡터(cos/sin)를 정밀 float로 마칭해서
  구했는데, RTL은 양자화된(Q9.8) 방향벡터를 쓰므로 벽 여유가 딱 1칸 안쪽인 코너
  케이스에서 파이썬과 RTL이 다른 칸에서 충돌을 감지해 d가 1 어긋남(파티클 2번의
  빔 1개에서 실측) — 오라클도 RTL과 똑같이 방향벡터를 먼저 양자화한 뒤 마칭하도록
  수정해서 해결. "소프트웨어 정답지는 하드웨어와 정확히 같은 연산 순서로 계산해야
  한다"는 이 프로젝트의 원칙이 고정소수점 반올림뿐 아니라 기하학적 양자화에도 그대로
  적용된 사례.
- **첫 합성(무료 Vivado ML Standard, 보드 없이) 완료** — `synth/`. 타깃: Arty A7-35T의
  `xc7a35ticsg324-1L`(무료 라이선스 대상 디바이스). `particle_scorer`(파티클 1개분)
  실측 자원: **LUT 6.4%(1332/20800), FF 0.61%, BRAM 78%(39/50 RAMB36) — DSP
  0%(0/90, 곱셈기 없는 설계가 합성 결과로 실증됨)**. **BRAM이 병목** — 이 칩엔
  particle_scorer 1개도 빠듯하고 2개 이상 병렬은 물리적으로 불가능(78%×2>100%).
  시뮬레이션에서 검증한 4개 병렬은 이 칩 기준으로는 더 큰 칩이나 BRAM 절약(맵
  32비트 워드 패킹, 이미 남은 과제였음)이 있어야 실제로 넣을 수 있음 — 시뮬레이션
  만으론 알 수 없었던 정보. 100MHz(10ns) 클럭 제약 기준 **WNS=-2.594ns 위반 →
  실제 최대 클럭 ≈ 79MHz**, 크리티컬 패스는 `ray_march_edt`의 가변 배럴 시프터
  (`cur_shift`만큼 dx/dy를 시프트하는 조합논리 체인, 12단 로직레벨) — 다음 최적화
  타깃으로 확정.
- **BRAM 병목 해결 — 파티클 2개가 테이블 하나를 공유, BRAM 추가비용 0** —
  `particle_scorer_arb.v` + `particle_scorer_pair_arb.v`. 계층별 리포트로
  BRAM 78% 전량이 `table_mem`(센서모델 테이블) 하나에서만 나온다는 걸 먼저
  확정(레이마칭/주소생성/PE 자체는 BRAM 0 사용). 처음엔 `table_mem_dp.v`(진짜
  듀얼포트)로 공유를 시도했으나 **합성기가 90601깊이 테이블의 듀얼포트
  크로스바를 못 만들고 통째로 복사해버려 BRAM이 오히려 156/100(초과)로 실패**
  — 시뮬레이션에서만 검증됐던 패턴이 실제 합성에선 안 통한 첫 사례. 대신
  **다른 프로젝트(전국AI반도체경진대회, AER 이벤트 중재기 설계)의
  `arbiter2.v`를 그대로 재사용**해서 진짜 single-port 테이블 하나를 매 클럭
  시분할로 나눠 쓰게 재설계 — `sensor_pe`가 `valid_i` 이후 1클럭 뒤 데이터를
  쓰는데 내부 `pend_valid` 릴레이 때문에 실제로는 승인 후 2클럭 동안 주소가
  안정적이어야 한다는 걸 실측(첫 시도는 기능 검증 자체가 실패, `-7459`/
  `-10357` vs 기대 `-7441`/`-6687`)으로 발견하고, 승인 후 1클럭 더 주소를
  붙잡아두는 홀드 로직을 얹어 해결(`arbiter2.v` 원본은 안 건드림). **합성
  실측: BRAM 39/50(78%) — 파티클 1개였을 때와 완전히 동일**, LUT
  13.07%(2719/20800, 로직 2배+중재기), FF 0.91%, DSP 여전히 0%. 타이밍도
  WNS=-2.594ns로 단일 버전과 동일(중재 로직이 크리티컬 패스를 악화시키지
  않음). 중재기 자체 비용은 LUT 2개, FF 1개(거의 무료).

상세 진행 기록은 [`progress.md`](progress.md).
