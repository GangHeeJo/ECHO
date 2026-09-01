# ECHO 진행 기록

## 2026-09-01 — 프로젝트 시작, 문제정의, 센서모델 PE 1개 검증 완료

**시작**: ZERO에서 겪은 실제 병목을 이번 학기 회로설계 수업 프로젝트로 가속해보자는
아이디어로 착수.

**시장조사**: 같은 F1TENTH particle_filter를 FPGA로 가속한 선행논문 실제로 확인함
— Bernardi et al., *"An FPGA Overlay for Efficient Real-Time Localization in
1/10th Scale Autonomous Vehicles"* (DATE 2022). 놀랍게도 ZERO가 쓰는 것과 **완전히
같은 원본 소프트웨어**(Walsh & Karaman, ICRA 2018 reference implementation)를
베이스로 함. 이 논문은:
- Xilinx Zynq UltraScale+ (Ultra96/ZCU102, 산업용 대형 보드)
- Vivado **HLS**로 C++→RTL 자동합성 (손으로 짠 RTL 아님)
- 레이마칭만 하드웨어 가속(전체 지연의 90%), 센서모델 평가는 소프트코어에서
  그대로 실행(데이터 이동만 줄임, 병렬 하드웨어화 안 함)

→ **차별화 지점 확정**: 손으로 짠 RTL, 소형 교육용 보드, 그리고 논문이 안 건드린
센서모델 평가를 v1 타깃으로.

**실측 (가장 중요한 작업)**: `particle_filter.py`의 `fine_timing` 파라미터를 켜고
ZERO 시뮬레이션(`sim:=true planner:=none`, opponent_perception 없이 PF 단독)을
실제로 돌려서 진짜 숫자를 뽑음.
- 레이마칭이 센서모델 계산의 **89.6%(500파티클) / 90~95%(4000파티클)** — 논문
  수치(90%)와 3중 일치
- **500파티클: possible ≈ 620Hz(요구 40Hz 대비 15배 여유) / 4000파티클(설계 목표):
  possible ≈ 88Hz(2.2배 여유)** — 이 개발PC에서는 PF가 병목이었던 적이 한 번도 없음
- 결론: `max_particles`가 4000→500으로 다운그레이드된 채 방치된(2026-08-07~) 진짜
  원인은 PF 자체가 아니라 그때 같이 돌던 `opponent_perception`(O(n²) 버그)일 가능성이
  높음. Jetson 실측은 여전히 미실측(가장 큰 남은 갭) — 상세는
  [`docs/problem_statement.md`](docs/problem_statement.md).

**진단 중 발견한 버그(재발 방지용 기록)**: `ros_clean.sh`와 `ros2 launch ...`를 같은
`bash -lc "..."` 문자열 안에서 연달아 실행하면, `ros_clean.sh`의
`pkill -f 'ros2 launc[h]'`가 **그 문자열 자체(나중에 나올 "ros2 launch" 텍스트를
이미 포함)를 실행 중인 셸 프로세스 자신**에 매칭돼서 자기 자신을 죽여버림(exit 15).
반드시 별개의 명령으로 분리해서 실행할 것. 또한 `bash -lc`(비대화형)는 `.bashrc`의
인터랙티브 체크 때문에 ROS 환경이 자동 로드 안 됨 — 매번 4줄(foxy/venv/install/
PYTHONPATH) 명시적으로 source 필요.

**타깃 확정**: 레이마칭이 임팩트는 훨씬 크지만(3중 확인), **v1은 센서모델 평가로
확정, 레이마칭은 v2.** 이미 설계가 끝나있고 난이도가 첫 프로젝트에 적당함.

**설계 & 구현**:
- `echo-ref/gen_sensor_model.py`(Windows) — ZERO의 `precompute_sensor_model()`
  수식을 그대로 이식. 표를 log(확률)로 저장해서 RTL이 곱셈기 없이 룩업+덧셈만
  하면 되게 설계(squash_factor도 log영역에선 상수곱). 실측 log(prob) 범위
  [-9.10, -2.12]에서 역산한 **Q5.8(13비트)** 고정소수점, 테이블 301×301(15m/5cm),
  전체 143.8KB(추후 보드 BRAM 예산과 대조 필요).
- `rtl/sensor_pe.v` — PE 1개. BRAM 1클럭 읽기지연에 맞춘 파이프라인(주소 제시 →
  다음클럭 데이터 도착 → 누적). `start`/`valid_i`/`last_i`/`done` 인터페이스.
- `rtl/table_mem.v` — `.hex` 로드하는 동기식 BRAM.
- `tb/tb_sensor_pe.v` — 파이썬이 만든 파티클 5개(8빔 장난감 시나리오)로 검증.

**검증 — 첫 실행 2/5 실패, 원인 규명 후 5/5 통과**: 처음 돌렸을 때 5개 중 2개가
±1 LSB(=1/256) 차이로 실패. 원인은 RTL 버그가 아니라 **정답지 생성 방식의
반올림 순서 실수**였음 — 파이썬이 "float 8개를 다 더한 뒤 한 번에 반올림"했는데,
RTL(진짜 하드웨어)은 "이미 반올림된 표 값 8개를 정수로 더함" — 이 둘은 수학적으로
다른 결과가 나올 수 있음. 실제로 문제였던 두 파티클의 표 항목을 직접 더해보니
RTL 결과(-12229)가 맞고 파이썬 정답지(-12230)가 틀렸음을 확인. `gen_sensor_model.py`
를 "양자화된 표 값을 정수로 더하는" 방식으로 수정 → 재생성 → **5/5 완전 일치.**

**남은 것**:
1. 8빔 장난감 → 실제 스펙 60빔으로 테스트 확장
2. PE 여러 개로 복제해서 파티클 병렬 처리 (v1의 다음 단계)
3. Jetson에서 `fine_timing` 실측 — problem_statement.md의 가장 중요한 미해결 갭
4. FPGA 보드 확정되면 합성(Vivado/Quartus) — 비트폭·PE개수·BRAM예산 최종 확정
5. v2: 레이마칭 PE (더 어려움 — 가변 길이 격자 순회, 훨씬 큰 온칩 메모리 필요)
