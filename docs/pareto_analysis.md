# ECHO — PE 수(파티클 병렬도) × 자원 × 타이밍 Pareto 분석

이 세션(2026-09-02) 동안 만든 모든 합성 데이터를 한 표로 모음. 개별
수치는 전부 `synth/*.rpt`에 실측 리포트로 남아있음 — 이 문서는 그걸
종합한 것.

## 요약 표 1 — PE 수(병렬 파티클 수)에 따른 자원/타이밍

외부 dx,dy 인터페이스(direction_gen 없음, Jetson이 방향 미리 계산):

| PE 수 | LUT | LUT% | BRAM | BRAM% | DSP | post-synth WNS | post-route WNS | fMax(post-route) |
|---|---|---|---|---|---|---|---|---|
| 1 | 1332 | 6.40% | 39/50 | 78% | 0 | -2.594ns | **-2.370ns** | **80.8MHz** |
| 2 | 2719 | 13.07% | 39/50 | 78% | 0 | -2.594ns | (미실시) | — |
| 4 | 5384 | 25.88% | 39/50 | 78% | 0 | -2.594ns | (미실시) | — |
| 8 | 10769 | 51.77% | 39/50 | 78% | 0 | -2.438ns | **-2.653ns** | **79.0MHz** |

온칩 direction generator 포함(θ만 보내면 됨, CORDIC 8개 내장):

| PE 수 | LUT | LUT% | BRAM | BRAM% | DSP | post-route WNS | fMax |
|---|---|---|---|---|---|---|---|
| 8(θ wrap 전) | 14429 | 69.37% | 39/50 | 78% | 0 | -2.992ns | 76.97MHz |
| 8(θ wrap 포함, 최종) | 14804 | 71.17% | 39/50 | 78% | 0 | **-2.716ns** | **78.64MHz** |
| direction_gen 단독(참고) | 341 | 1.64% | 0/50 | 0% | 0 | **+4.119ns**(양수!) | 100MHz 여유 통과 |

**읽는 법**:
- **BRAM은 PE 수와 완전히 무관하게 항상 39/50(78%)** — arbiter로 단일
  `table_mem`을 공유하는 구조 덕분(이 세션 초반의 핵심 성과: 진짜
  듀얼포트를 시도했을 땐 BRAM이 156/100까지 터졌었음). PE를 8개로 늘려도
  메모리는 안 늘어남 — "메모리 포트가 병목이 되는 지점"이 이 설계에선
  구조적으로 없음(공유 아키텍처가 정확히 이 문제를 없앰).
- **LUT는 PE 수에 거의 선형 비례**(1→2→4→8배로 얼추 2배씩 증가:
  1332→2719→5384→10769, 각 단계 비율 2.04/1.98/2.00) — `ray_march_edt`가
  통째로 복제되기 때문(맵/EDT 룩업 테이블이 BRAM이 아니라 LUT 기반
  콤비네이셔널 ROM이라 PE마다 그대로 복제됨 — 이게 LUT 사용량의 정체,
  이번 세션 초반에 크리티컬 패스 분석에서도 확인됨).
- **direction_gen 추가 비용은 작음**: 8개 PE에 CORDIC까지 내장해도
  LUT는 51.77%→71.17%(+19.4%p)뿐, BRAM/DSP는 전혀 안 늘어남 —
  direction_gen 자체가 341LUT(1.64%)로 매우 저렴하고 자체 크리티컬
  패스도 훨씬 얕아서(WNS +4.119ns, 다른 모든 구성이 마이너스인 것과
  대조적) 전체 WNS를 깎아먹지 않음.
- **WNS는 PE 수와 무관하게 거의 일정**(-2.4~-3.0ns 범위) — 크리티컬
  패스가 항상 `ray_march_edt`(같은 EDT 배럴시프터/맵 ROM 디코드 경로)
  라서, PE를 늘려도 "가장 느린 PE 하나"의 내부 구조는 안 바뀜. 8-way에서
  post-route가 오히려 살짝 나빠지는 건(1개 -2.370→8개 -2.653/-2.992)
  배선 혼잡 증가 때문(LUT 51%→69~71%로 칩이 꽉 차면서).

## 요약 표 2 — 500파티클(실제 ZERO 프레임) 전체 처리 시간

| 구성 | 측정/추정 | 사이클 | 클럭 | 시간 | ZERO(1.428ms) 대비 |
|---|---|---|---|---|---|
| 외부 dx,dy, 8-way (Jetson 전처리 0.782ms 포함) | 실측+실측 | 72,289 | 79.0MHz | 0.915ms+0.782ms=**1.697ms** | 19% 느림 |
| 온칩 dgen, 8-way(θ wrap 전) | 실측 | 97,865 | 76.97MHz | **1.271ms** | 11.2% 빠름 |
| 온칩 dgen, 8-way(θ wrap 포함, 최종) | 실측 | 98,116 | 78.64MHz | **1.248ms** | 12.6% 빠름 |
| 〃, Jetson 기준(Geekbench 비율 추정) | 추정 | 98,116 | 78.64MHz | **1.248ms** vs ZERO 2.606ms | **약 52% 빠름(2.09배)** |

## 무엇을 위해 PE를 늘렸고, 어디서 한계가 왔는가

이 세션이 겪은 "처리량을 높이려고 PE를 늘렸지만 어느 지점부터 효율이
줄어드는" 지점은 **메모리(BRAM)가 아니라 배선 혼잡**이었다 — 처음
가설(BRAM 포트 경합)은 이미 arbiter로 완전히 해소됐고(BRAM은 PE 수와
무관하게 78% 고정), 대신 LUT가 커지면서(8-way에서 51~71%) 배선 혼잡이
늘어 WNS가 post-route에서 post-synth 추정보다 나빠지는 정도로 나타남
(1개 PE는 post-route가 오히려 더 좋았는데, 8개부터는 반대). **8-way가
이 칩(xc7a35t, LUT 20800개)에서 실질적으로 편안한 상한에 가깝다** — LUT
71%면 다음 단계(16-way 등)를 시도하면 배선 혼잡이 더 심해져 WNS가
더 나빠질 가능성이 높음(직접 확인은 안 했음, 이 표의 추세로 추론).

## 데이터 출처

전부 `synth/` 아래 실제 파일:
`util_v3_particle_scorer.rpt`/`util_v3_postroute.rpt`(1개),
`util_pair_arb.rpt`/`timing_pair_arb.rpt`(2개),
`util_quad_arb.rpt`/`timing_quad_arb.rpt`(4개),
`util_oct_arb.rpt`/`timing_oct_arb.rpt`(8개 post-synth, 외부 dx,dy),
`util_oct_postroute.rpt`/`timing_oct_postroute.rpt`(8개 post-route, 외부
dx,dy), `util_direction_gen.rpt`/`timing_direction_gen.rpt`(direction_gen
단독), `util_dgen_oct.rpt`/`timing_dgen_oct.rpt`(8개+온칩dgen, θ wrap 전),
`util_dgen_oct_v2.rpt`/`timing_dgen_oct_v2.rpt`(8개+온칩dgen+θ wrap, 최종).
