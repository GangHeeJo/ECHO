#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cocotb 회귀 테스트 — particle_scorer_dgen_arb.v. 이 세션에서 실제로
찾은 데드락 버그 계열을 정확히 겨냥한다: "방향(direction_gen) 준비 대기"
와 "beam_start(외부 r_obs 공급) 대기"가 서로 다른 타이밍으로 오는데, 그 중
하나를 놓치면 영원히 멈춘다 — 처음 구현은 beam_start를 특정 state에서만
받아서 데드락났었고(S_PREP_DIR0/S_WAIT_NEXT_DIR 분리), state 무관 캡처로
고쳤다. 이 테스트는 "beam_start를 언제 줘도(빠르게/느리게/무작위로) 절대
안 걸려야 한다"를 다양한 타이밍으로 무작위 반복해서 그 수정이 실제로
일반적으로 안전한지 회귀 확인한다.

정확성도 같이 봄 — 실제 ZERO 파티클 0~7(rf_b0_p0..7_*)의 기대 weight는
bench/cordic_recompute_001.npz(RTL의 CORDIC을 파이썬으로 비트단위 재현한
값)에서 가져온 상수.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, with_timeout, SimTimeoutError

NUM_RAYS = 60

# rf_b0_p0..7의 실제 ZERO 파티클 — 기대 weight는 gen_dgen_accuracy_test.py가
# RTL의 CORDIC을 파이썬으로 비트 단위 재현해서 낸 값(bench/cordic_recompute_001.npz)
EXPECTED_WEIGHTS = [-46170, -50615, -50388, -46390, -50044, -49927, -48818, -50209]

# 데드락 재현 위해 넉넉하게 잡은 상한(정상이면 훨씬 일찍 끝남 — 60빔 기준
# 겹침 없이 최악이어도 수천 클럭 안쪽) — 이 상한을 넘으면 사실상 데드락.
MAX_CYCLES = 20000


def read_hex_lines(path):
    with open(path) as f:
        return [int(line.strip(), 16) for line in f if line.strip()]


def unsign(v, w):
    return v - (1 << w) if v >= (1 << (w - 1)) else v


async def run_particle(dut, particle_idx, beam_delay_fn):
    """particle_idx(0~7)를 처리, beam_delay_fn(beam_no)로 매 빔마다 몇 클럭
    기다렸다 beam_start를 줄지 무작위로 결정 — 빠르게/느리게 섞어서
    S_WAIT_THETA/S_WAIT_BEAM의 타이밍 레이스를 다양하게 찌른다."""
    prefix = "rf_b0_p%d" % particle_idx
    rs = read_hex_lines("%s_r.hex" % prefix)
    x0y0 = read_hex_lines("%s_x0y0.hex" % prefix)
    theta_raw = read_hex_lines("%s_theta_raw.hex" % prefix)[0]

    dut.x0.value = x0y0[0]
    dut.y0.value = x0y0[1]
    dut.theta_raw.value = theta_raw

    await RisingEdge(dut.clk)
    dut.particle_start.value = 1
    await RisingEdge(dut.clk)
    dut.particle_start.value = 0

    for i in range(NUM_RAYS):
        delay = beam_delay_fn(i)
        for _ in range(delay):
            await RisingEdge(dut.clk)
        dut.r_obs.value = rs[i]
        dut.beam_start.value = 1
        await RisingEdge(dut.clk)
        dut.beam_start.value = 0

        # beam_done을 기다리되, 타임아웃을 걸어 데드락이면 즉시 실패 처리
        # (무한 대기 대신 진단 가능한 에러로 만듦)
        while not dut.beam_done.value:
            await RisingEdge(dut.clk)

    while not dut.particle_done.value:
        await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)  # 기존 Verilog 테스트벤치들과 동일하게 한 클럭 더(안정화)

    await ReadOnly()
    got = unsign(int(dut.weight_o.value), 20)
    await RisingEdge(dut.clk)  # ReadOnly 단계에서 바로 리턴하면 호출자가 값을
                                # 못 쓰는 상태라 다음 쓰기 가능한 시점까지 진행
    return got


@cocotb.test()
async def test_dgen_arb_beam_timing_fuzz(dut):
    """beam_start 타이밍을 무작위로 섞어서(0~5클럭 지연, 매 빔마다 다르게)
    8개 실제 파티클을 반복 처리 — 절대 안 멈춰야 하고(타임아웃으로 확인),
    weight도 정확히 맞아야 함."""
    random.seed(20260902)
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.rst_n.value = 0
    dut.particle_start.value = 0
    dut.beam_start.value = 0
    dut.x0.value = 0
    dut.y0.value = 0
    dut.theta_raw.value = 0
    dut.r_obs.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    fails = 0
    for particle_idx in range(8):
        # 매 빔마다 0~5클럭 무작위 지연 — 어떤 빔은 direction_gen이 먼저
        # 끝나있고(지연 큼) 어떤 빔은 레이마칭이 먼저 끝나는(지연 0) 상황을
        # 둘 다 무작위로 만들어냄.
        delays = [random.randint(0, 5) for _ in range(NUM_RAYS)]

        def delay_fn(i, delays=delays):
            return delays[i]

        try:
            got = await with_timeout(
                run_particle(dut, particle_idx, delay_fn),
                MAX_CYCLES * 10, "ns",
            )
        except SimTimeoutError:
            fails += 1
            cocotb.log.error("particle %d: TIMEOUT(데드락 의심, %d 사이클 안에 안 끝남)"
                              % (particle_idx, MAX_CYCLES))
            continue

        exp = EXPECTED_WEIGHTS[particle_idx]
        if got != exp:
            fails += 1
            cocotb.log.error("particle %d: weight=%d (expected %d)" % (particle_idx, got, exp))
        else:
            cocotb.log.info("particle %d: weight=%d OK (beam_start 타이밍 무작위)" % (particle_idx, got))

    assert fails == 0, "%d/8 particles failed(데드락 또는 오답)" % fails
