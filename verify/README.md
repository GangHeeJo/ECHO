# ECHO 검증 환경 (cocotb)

이 세션까지 이 프로젝트의 검증은 전부 "파이썬으로 정답지 hex 파일을
미리 만들고 → Verilog 테스트벤치가 `$readmemh`로 읽어서 비교"하는 방식
이었다(`tools/`, `tb/`). 정확하지만 케이스가 전부 **미리 정해진 고정
값**이라, 랜덤화·경계조건 자동 탐색·타이밍 레이스 재현 같은 건 손으로
케이스를 늘리는 수밖에 없었다.

이 디렉터리는 [cocotb](https://www.cocotb.org/)(파이썬으로 직접 DUT를
구동·검사하는 업계 표준 검증 프레임워크, Icarus Verilog 백엔드 지원)로
그 빈틈을 메운다 — **매 실행마다 새로운 무작위 입력을 그 자리에서
생성**해서 DUT에 넣고, 같은 파이썬 프로세스 안에서 골든 모델과 즉시
비교한다.

## 설치

```bash
~/zero/.venv/bin/pip install cocotb   # 이미 설치됨(2.0.1)
```

## 실행

```bash
cd ~/echo/verify
~/zero/.venv/bin/python3 run_angle_wrap.py
~/zero/.venv/bin/python3 run_particle_scorer_dgen_arb.py
```

## 파일

- `test_angle_wrap.py` / `run_angle_wrap.py` — `angle_wrap.v`(순수 정수
  연산) 무작위 3000케이스 + 경계 9케이스, 전부 비트 단위 완전일치 기대.
- `test_particle_scorer_dgen_arb.py` / `run_particle_scorer_dgen_arb.py` —
  **이 세션에서 실제로 찾은 데드락 버그 계열의 회귀 테스트**. `beam_start`
  (외부 r_obs 공급)를 매 빔마다 0~5클럭 무작위 지연으로 주면서 실제 ZERO
  파티클 8개를 처리 — 절대 안 멈춰야 하고(타임아웃으로 확인), weight도
  `bench/cordic_recompute_001.npz`(RTL의 CORDIC을 비트단위 재현한 파이썬
  모델) 기준값과 정확히 일치해야 함. 1차 구현(`S_PREP_DIR0`/
  `S_WAIT_NEXT_DIR` 분리)이 정확히 이 타이밍 패턴에서 데드락났었음(실제
  이력은 `progress.md`/`docs/problem_statement.md` 참고) — 이 테스트는
  그 수정이 일반적으로 안전한지 반복 확인한다.
- `tb_wrapper_dgen_arb.v` — `particle_scorer_dgen_arb`는 원래
  오케스트레이터(`particle_scorer_dgen_oct_arb`)만 직접 다루는
  `table_addr`/`table_data`/`req`/`gnt` 포트가 있어서, 단독 테스트하려면
  `table_mem`을 붙이고 `gnt`를 항상 1로 고정해주는 작은 하네스가 필요함.
- `*.hex` — `ray_march_edt`/`table_mem`/`cordic_sincos`/`direction_gen`
  이 기본 파라미터로 찾는 룩업 테이블들의 복사본(원본은 `sim/`) — cocotb
  가 `$readmemh`를 실행할 때의 작업 디렉터리가 `verify/`라서 필요.
- `rf_b0_p{0..7}_*.hex` — 실제 ZERO 파티클 0~7의 실측 데이터(`sim/`의
  복사본, `gen_real_frame_test.py`/`gen_dgen_batch_theta.py`가 만든 것).

## 다음에 추가할 만한 것 (미착수)

- direction_gen/CORDIC 자체의 랜덤화 회귀(현재는 angle_wrap만 커버)
- arbiter8의 fairness/starvation 무작위 스트레스 테스트
- ray_march_edt의 경계조건(맵 가장자리, max-range 빔) 전용 테스트
- SystemVerilog assertion 또는 cocotb 쪽 monitor로 req/gnt·beam_start/
  beam_done 핸드셰이크 프로토콜 규칙을 매 클럭 자동 검증
