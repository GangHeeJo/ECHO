#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cocotb 랜덤화 테스트 — angle_wrap.v. 지금까지 이 프로젝트가 써온 방식
(파이썬으로 정답지 hex 파일을 미리 만들고 $readmemh로 읽는 방식)과 다르게,
매 실행마다 새로운 무작위 입력을 그 자리에서 만들어 DUT에 넣고 즉시
비교한다 — 커버리지가 실행마다 달라지고, 케이스 수를 늘리는 데 별도
파일 생성이 필요 없다.

골든 모델은 angle_wrap.v와 정확히 같은 정수 연산(부호확장 + 최대 1회
+-2pi 보정)을 그대로 재현 — 이 모듈은 근사가 없는 순수 정수 연산이라
비트 단위 완전일치를 기대한다(오차 허용치 없음).
"""

import random

import cocotb
from cocotb.triggers import Timer

W_EXT = 19
PI_FIX = 205887
TWO_PI_FIX = 411775
N_RANDOM = 3000


def to_unsigned(v, w):
    return v & ((1 << w) - 1)


def to_signed(v, w):
    v = to_unsigned(v, w)
    return v - (1 << w) if v >= (1 << (w - 1)) else v


def golden_wrap(a_signed, b_signed):
    s = a_signed + b_signed
    if s > PI_FIX:
        s -= TWO_PI_FIX
    elif s < -PI_FIX:
        s += TWO_PI_FIX
    return to_unsigned(s, W_EXT)


@cocotb.test()
async def test_angle_wrap_random(dut):
    """|theta_a|,|theta_b| <= pi 범위에서 균일 무작위 N_RANDOM개."""
    random.seed(20260902)
    fails = 0
    for i in range(N_RANDOM):
        a = random.randint(-PI_FIX, PI_FIX)
        b = random.randint(-PI_FIX, PI_FIX)
        dut.theta_a.value = to_unsigned(a, W_EXT)
        dut.theta_b.value = to_unsigned(b, W_EXT)
        await Timer(1, unit="ns")

        exp = golden_wrap(a, b)
        got = int(dut.theta_o.value)
        if got != exp:
            fails += 1
            if fails <= 10:
                cocotb.log.error(
                    "MISMATCH #%d: a=%d b=%d got=0x%05x exp=0x%05x" % (i, a, b, got, exp)
                )

    cocotb.log.info("angle_wrap random: %d/%d passed" % (N_RANDOM - fails, N_RANDOM))
    assert fails == 0, "%d/%d random cases failed" % (fails, N_RANDOM)


@cocotb.test()
async def test_angle_wrap_boundary(dut):
    """경계 케이스: 정확히 +-pi, +-pi/2, 0, 그리고 wrap이 정확히 걸리는
    극단값(sum이 PI_FIX보다 딱 1 큰/작은 경우)."""
    boundary_pairs = [
        (0, 0), (PI_FIX, 0), (-PI_FIX, 0), (PI_FIX, PI_FIX), (-PI_FIX, -PI_FIX),
        (PI_FIX, 1), (-PI_FIX, -1),          # wrap 경계를 딱 넘김
        (PI_FIX // 2, PI_FIX // 2 + 1),      # 반씩 더해 경계 살짝 넘김
        (PI_FIX, -PI_FIX),                    # 합이 정확히 0
    ]
    fails = 0
    for a, b in boundary_pairs:
        dut.theta_a.value = to_unsigned(a, W_EXT)
        dut.theta_b.value = to_unsigned(b, W_EXT)
        await Timer(1, unit="ns")
        exp = golden_wrap(a, b)
        got = int(dut.theta_o.value)
        if got != exp:
            fails += 1
            cocotb.log.error("BOUNDARY MISMATCH: a=%d b=%d got=0x%05x exp=0x%05x" % (a, b, got, exp))

    cocotb.log.info("angle_wrap boundary: %d/%d passed" % (len(boundary_pairs) - fails, len(boundary_pairs)))
    assert fails == 0
