# ECHO

ZERO(F1TENTH)의 `particle_filter` 위치추정 연산을 위한 FPGA 가속기 프로젝트.
이번 학기 디지털 회로설계 수업과 연계해서 진행하는 개인 프로젝트 — ZERO 팀
레포(`~/zero/src`)와는 완전히 별개, git 미포함.

**현재 상태 (2026-09-02): v1(센서모델)·v2(레이마칭, EDT 최적화)·v3(결합) 완료,
Vivado 합성으로 실측까지 마쳤고, arbiter 기반 BRAM 공유로 파티클을 1→8개까지
BRAM 추가비용 0으로 늘리는 데 성공했다(합성 3회 전부 BRAM 39/50, 동일 칩).**

## 핵심 아이디어

원본 소프트웨어는 파티클 하나의 점수를 "라이다 빔 60개의 확률을 곱해서"
계산한다. 이 프로젝트는 그 계산 전체(레이마칭 + 센서모델 평가)를 **곱셈기
없이** RTL로 옮긴다 — 확률을 log 영역에 저장해 곱을 덧셈으로 바꾸고
(`sensor_pe`), 주소 계산은 시프트-덧셈으로(`addr_gen`), 레이마칭의 가변
스텝은 배럴 시프터로(`ray_march_edt`) 대체한다.

```
파티클 위치 + LiDAR 관측(r_obs) × 60빔
        │
        ▼
  ray_march_edt   거리장(EDT) 기반, 곱셈기 없음 → 기대거리 d
        │
        ▼
  addr_gen        (r_obs, d) → 테이블 주소, 시프트-덧셈만
        │
        ▼
  arbiter2/4/8    전국AI반도체경진대회 프로젝트 재사용 → 공유 테이블 포트 승인
        │
        ▼
  table_mem       센서모델 log-확률 BRAM — 파티클이 몇 개든 물리적으로 1개
        │
        ▼
  sensor_pe       룩업값 누적(곱셈기 없음) → 파티클 log-weight
```

## 수치 결과 (Vivado ML Standard, Arty A7-35T `xc7a35ticsg324-1L`, 무료 라이선스 대상 — 보드 없이 합성만)

| 구성 | BRAM(50개 중) | LUT(20800개 중) | DSP | WNS(100MHz 제약, 합성 기준 추정치) | 병렬 지연시간(빔8개 기준) |
|---|---|---|---|---|---|
| 파티클 1개(테이블 전용 소유) | 39 (78%) | 6.40% | 0 | -2.594ns(합성 추정 ≈79MHz) → **post-route 실측 -2.370ns(≈80.8MHz, 오히려 더 좋음)** | — |
| 독립 테이블로 2개 이상 병렬 | **156(2개분) — 예산초과, 합성 실패** | — | — | — | 1520 ns(시뮬만, 이 칩엔 못 들어감) |
| `arbiter2`로 2개가 테이블 공유 | 39 (78%, 동일) | 13.07% | 0 | -2.594ns(동일) | 1310 ns |
| `arbiter4`로 4개가 테이블 공유 | 39 (78%, 동일) | 25.88% | 0 | -2.594ns(동일) | 1540 ns(+1.3%) |
| `arbiter8`로 8개가 테이블 공유 | **39 (78%, 동일)** | 51.77% | 0 | -2.438ns(거의 동일) | **1660 ns** |

DSP(하드 곱셈기)는 다섯 구성 전부 0개(post-route에서도 재확인) — "곱셈기
없는 설계" 원칙이 실증됨. **BRAM은 1→8개 전부 39/50으로 동일** — 병목이
처음부터 끝까지 테이블 하나였고, 중재로 완전히 해소됨. 타이밍은 1~4개는
WNS=-2.594ns(post-synth 추정)로 완전히 같고, 8개는 -2.438ns로 소폭 다름 —
최악경로 상세로 재확인한 결과 **새 병목이 아니라 같은 크리티컬 패스
(`ray_march_edt`의 배럴 시프터, `y_reg`→`x_reg`)가 설계 규모에 따라 미세하게
다른 게이트 조합으로 매핑된 것**(로직레벨 12~13단, 데이터경로 지연 12.0~12.2ns
근처로 사실상 동일). 파티클 1개는 **post-route까지 확인**했고 post-synth
추정치보다 오히려 살짝 나은 결과(-2.370ns)가 나왔음 — 아직 2~8개 구성은
post-route 미실시. LUT는 8개에서 51.77%까지 올라와 더
늘리면 LUT가 먼저 병목이 될 가능성이 있음. 자세한 실패 원인(왜 진짜
듀얼포트 공유는 안 통했는지)과 전체 로그는 [`progress.md`](progress.md)의
2026-09-02 항목 참고.

## 빠른 시작

시뮬레이션만이면 지도 파일 없이 바로 됨 — `sim/`에 필요한 `.hex`가 전부
커밋돼 있음:

```bash
cd ~/echo
iverilog -o sim/tb_particle_scorer_quad_arb.vvp \
  rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v \
  rtl/arbiter4.v rtl/particle_scorer_arb.v rtl/particle_scorer_quad_arb.v \
  tb/tb_particle_scorer_quad_arb.v
cd sim && vvp tb_particle_scorer_quad_arb.vvp   # 파티클 4개 다 PASS 나와야 정상
```

**테스트 벡터를 처음부터 재현**하려면(선택, 이미 만들어진 `.hex`를 그냥 쓸
거면 필요 없음) `tools/gen_particle_scorer_test.py`를 돌리면 되는데, 이
스크립트는 ZERO 팀 레포의 지도 파일(`~/zero/src/track_assets/maps/changwon/map.pgm`)
을 하드코딩된 절대경로로 읽는다 — 이 레포만 단독으로 clone해서는 못 돌리고,
`~/zero/src`가 옆에 있어야 함(이 프로젝트 자체는 `~/zero/src`와 git 분리돼
있지만, 지도 데이터는 그쪽이 원본이라 재사용). scipy 필요(`compute_edt_shift`).

```bash
cd ~/echo/sim
python3 ../tools/gen_particle_scorer_test.py
```

**합성**은 Vivado가 필요(무료 ML Standard Edition, Artix-7/Zynq-7000 계열은
라이선스 없이 지원). ⚠️ 작업 폴더 경로에 한글이 있으면 Vivado가 크래시하니
(`progress.md` 참고) 순수 영문 경로에서 실행할 것 — `rtl/*.v` + `sim/*.hex` +
`synth/*.tcl,*.xdc`를 한 폴더에 모은 뒤:

```
vivado -mode batch -source synth_quad_arb.tcl -log vivado_quad_arb.log
```

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
  ray_march_edt.v               [v2.2] 거리장(EDT) 기반 마칭 — 빈 공간에서 2^k칸씩 성큼성큼(배럴 시프터, 곱셈기 없음)
  particle_scorer.v            [v3] v1+v2 결합 — 빔마다 레이마칭으로 기대거리 구하고 센서모델로 채점, 파티클 전체 점수까지
  particle_scorer_shared.v      [v3.1, 폐기] 테이블을 외부 포트로 뺀 버전 — table_mem_dp로 진짜 듀얼포트 공유 시도, 합성기가 통째 복사해 BRAM 2배로 실패
  particle_scorer_pair.v        [v3.1, 폐기] particle_scorer_shared 2개 + table_mem_dp — 합성 실패(BRAM 156/100)로 폐기, 코드는 실패 사례로 보존
  particle_scorer_arb.v         [v3.2] particle_scorer_shared를 중재(arbiter) 방식으로 재설계 — 공유 테이블 포트에 req/gnt 인터페이스 추가
  particle_scorer_pair_arb.v    [v3.2] particle_scorer_arb 2개가 arbiter2(전국AI반도체경진대회 프로젝트 재사용)로 single-port table_mem 하나를 시분할 — BRAM 추가비용 0으로 합성 성공
  particle_scorer_quad_arb.v    [v3.3] particle_scorer_arb 4개가 arbiter4(같은 프로젝트 재사용)로 single-port table_mem 하나를 시분할 — BRAM 39/50 그대로 유지
  particle_scorer_oct_arb.v     [v3.4] particle_scorer_arb 8개 + arbiter8 — 기능검증 8/8 PASS(1660ns), 합성도 완료(BRAM 39/50 동일, LUT 51.77%)
  arbiter2.v / arbiter4.v / arbiter8.v   전국AI반도체경진대회 프로젝트에서 그대로 재사용 — round-robin 중재기(안 건드림)
tb/
  tb_sensor_pe.v              PE 1개, 파이썬 정답지와 대조하는 자가검증 테스트벤치 (60빔)
  tb_sensor_pe_parallel.v     PE 2개가 듀얼포트 메모리를 공유하며 파티클 2개를 동시 처리하는지 검증
  tb_sensor_pe_x4.v           PE 4개(테이블 복사본 2개, generate로 인스턴스화), 파티클 4개 동시 처리 검증
  tb_sensor_pe_seq4.v         PE 1개로 파티클 4개를 순서대로(대조군) — 병렬 대비 속도 비교용
  tb_addr_gen.v                addr_gen 단독 검증(손으로 계산한 값과 대조)
  tb_sensor_pe_addrgen.v      addr_gen을 실제로 PE 앞단에 연결 — 주소를 하드웨어가 직접 계산하는 전체 파이프라인 검증
  tb_ray_march.v               레이마칭 v2.0 — 벽/경계/대각선/최대거리 4가지 케이스 검증
  tb_ray_march_bram.v          레이마칭 v2.1 — 실제 changwon 트랙에서 6가지 방향 검증
  tb_ray_march_edt.v            거리장 기반 마칭 검증 — 같은 6케이스에서 기존 대비 클럭 수 직접 대조(최대 6배 이상 감소)
  tb_particle_scorer.v         v3 — 파티클 1개, 빔 8개, 레이마칭+센서모델 전체 파이프라인 검증
  tb_particle_scorer_parallel.v v3 병렬 — particle_scorer 2개 동시 처리, 소요시간이 단일 처리와 동일함을 확인
  tb_particle_scorer_x4.v       v3 4배 병렬 — particle_scorer 4개 동시 처리(1520ns, 단일 처리와 비슷)
  tb_particle_scorer_pair.v     [폐기] particle_scorer_pair(진짜 듀얼포트 공유) 기능검증 — 시뮬은 통과하나 합성이 실패해 폐기된 경로
  tb_particle_scorer_pair_arb.v v3.2 — particle_scorer_pair_arb(중재 공유) 기능검증, 파티클 2개 다 PASS
  tb_particle_scorer_quad_arb.v v3.3 — particle_scorer_quad_arb(4-way 중재 공유) 기능검증, 파티클 4개 다 PASS(1540ns)
  tb_particle_scorer_oct_arb.v  v3.4 — particle_scorer_oct_arb(8-way 중재 공유) 기능검증, 파티클 8개 다 PASS(1660ns)
tools/
  gen_track_map.py             changwon map.pgm -> 지도 .hex + 파이썬으로 미리 계산한 테스트 시나리오 정답
  gen_sensor_model.py          ZERO 실제 센서모델 수식을 이식 — 룩업테이블(.hex)과 RTL 테스트벡터 생성
  gen_particle_scorer_test.py  particle_scorer 통합 테스트용 정답지(위 두 스크립트를 그대로 import해서 재사용) — 재현 방법은 위 "빠른 시작" 참고
synth/
  constraints.xdc              100MHz 클럭 제약(합성 타이밍 분석용)
  synth_v1_sanity.tcl          가장 작은 모듈(sensor_pe)로 Vivado 배치모드 파이프라인 자체를 검증
  synth_v3_timed.tcl / util_v3_timed.rpt / timing_v3_timed.rpt / timing_v3_worstpath.rpt
                                particle_scorer(파티클 1개) 합성+타이밍 — LUT 6.4%/FF 0.61%/BRAM 78%/DSP 0%, 합성 기준 추정 ≈79MHz(실물 보드 측정 아님, place&route 전 수치)
  synth_v3_route.tcl / util_v3_postroute.rpt / timing_v3_postroute*.rpt
                                particle_scorer place&route까지 실행 — BRAM/DSP 동일, 타이밍 post-route 실측 -2.370ns(≈80.8MHz, 합성 추정치보다 오히려 좋음), 크리티컬 패스 동일(ray_march_edt 배럴 시프터) 재확인
  synth_pair_arb.tcl / util_pair_arb*.rpt / timing_pair_arb.rpt
                                particle_scorer_pair_arb(파티클 2개, 테이블 공유) 합성 — BRAM 39/50 그대로
  synth_quad_arb.tcl / util_quad_arb*.rpt / timing_quad_arb.rpt
                                particle_scorer_quad_arb(파티클 4개, 테이블 공유) 합성 — BRAM 39/50 그대로
  synth_oct_arb.tcl / util_oct_arb*.rpt / timing_oct_arb.rpt
                                particle_scorer_oct_arb(파티클 8개, 테이블 공유) 합성 — BRAM 39/50 그대로, LUT 51.77%
sim/
  sensor_model_log_q5_8.hex   룩업테이블 데이터
  testvec_addrs.hex           테스트용 주소 목록 (파티클 5개 x 빔 60개)
  testvec_expected.hex        파티클별 정답(log-weight)
  particle{0..7}_*.hex        particle_scorer 계열 통합 테스트용 정답지(파티클 8개분)
  changwon_occ.hex / changwon_edt_shift.hex  changwon 트랙 지도(점유격자 + EDT 배럴시프트값)
  (전부 tools/의 세 스크립트 산출물 — 이미 커밋돼 있어 재생성 없이 바로 시뮬 가능)
```

## 툴체인

- **시뮬레이션**: Icarus Verilog(`iverilog`/`vvp`) + GTKWave — 둘 다 설치됨
- **합성**: Vivado ML Standard Edition(무료, Artix-7/Zynq-7000 계열은 라이선스
  불필요) — `C:\Xilinx\Vivado\2024.1`에 설치돼 있음. 아직 실물 FPGA 보드는 없음
  (구매 계획 없음) — 합성/타이밍/자원 리포트까지가 지금 목표.
- VS Code + "Verilog-HDL/SystemVerilog" 확장(Remote-WSL 창에서 별도 설치 필요)

## 전체 빌드 명령 (버전별)

```bash
cd ~/echo

# v1 — PE 1개, 60빔짜리 정식 스펙 검증
iverilog -o sim/tb_sensor_pe.vvp rtl/table_mem.v rtl/sensor_pe.v tb/tb_sensor_pe.v
cd sim && vvp tb_sensor_pe.vvp && cd ..

# v1 — PE 2개 병렬 처리 검증 (듀얼포트 메모리 공유)
iverilog -o sim/tb_sensor_pe_parallel.vvp rtl/table_mem_dp.v rtl/sensor_pe.v tb/tb_sensor_pe_parallel.v
cd sim && vvp tb_sensor_pe_parallel.vvp && cd ..

gtkwave sim/tb_sensor_pe.vcd            # 단일 PE 파형
gtkwave sim/tb_sensor_pe_parallel.vcd   # 병렬 PE 파형
```

v3 계열(particle_scorer/pair_arb/quad_arb/oct_arb)의 정확한 `iverilog` 명령은
각 `tb/*.v` 파일 맨 위 주석에 그대로 있음 — "빠른 시작"의 quad_arb 명령과
같은 패턴으로 파일명만 바꾸면 됨.

상세 진행 기록(실패 사례·디버깅 과정 포함)은 [`progress.md`](progress.md).
