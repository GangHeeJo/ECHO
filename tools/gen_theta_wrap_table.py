#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""theta_wrap.v 검증용 — 실제 ZERO의 파티클 theta는 wrap 없이 누적되므로
(motion_model이 매 스텝 더하기만 함, mod 2pi 안 함) 임의 크기가 될 수 있다.
angle_wrap.v(두 개의 이미 [-pi,pi] 안인 값의 합, 최대 1회 보정)와 달리
theta_wrap은 "얼마나 벗어나 있는지 모르는" 값을 받아 반복 보정한다.

실측(bench/zero_frame_001.npz): 500개 중 1회 보정으로 충분한 게 440개,
0회(이미 범위 안)가 60개, 2회 이상 필요한 경우 0개 — 그래도 안전하게
MAX_ITER=8(최대 ±8바퀴, 약 ±50rad)까지 대응하도록 여유있게 설계.
"""

import math

FRAC_W = 16
W_RAW = 24   # 부호1 + 정수부7(±64 커버) + 소수부16
W_EXT = 19   # 출력(Q3.16, angle_wrap.v/cordic_sincos_full.v와 동일 포맷)
SCALE = 1 << FRAC_W

PI_FIX = 205887      # 기존 스크립트들과 동일 상수(Q_.16)
TWO_PI_FIX = 411775


def to_fixed(v, w):
    iv = int(round(v * SCALE))
    assert -(1 << (w - 1)) <= iv < (1 << (w - 1)), 'overflow: %r' % v
    return iv & ((1 << w) - 1)


def unsign(iv_masked, w):
    return iv_masked - (1 << w) if iv_masked >= (1 << (w - 1)) else iv_masked


def wrap_fixed(raw_signed, max_iter=8):
    acc = raw_signed
    for _ in range(max_iter):
        if acc > PI_FIX:
            acc -= TWO_PI_FIX
        elif acc < -PI_FIX:
            acc += TWO_PI_FIX
        else:
            break
    return acc & ((1 << W_EXT) - 1)


def main():
    # 실측 범위(-0.35~7.41rad) + 여유있게 더 큰 값(다중 바퀴)도 같이 검증
    test_rad = [-0.350, 2.974, 4.573, 4.965, 5.712, 4.801, 7.409,
                -7.0, 10.5, -12.0, 15.0, math.pi, -math.pi, 0.0]
    with open('theta_wrap_test_raw.hex', 'w') as fr, \
         open('theta_wrap_test_exp.hex', 'w') as fe, \
         open('theta_wrap_test_deg.txt', 'w') as fd:
        for rad in test_rad:
            raw_fixed = to_fixed(rad, W_RAW)
            raw_signed = unsign(raw_fixed, W_RAW)
            exp = wrap_fixed(raw_signed)
            fr.write('%06x\n' % raw_fixed)
            fe.write('%05x\n' % exp)
            fd.write('%.4f -> exp raw=%d\n' % (rad, exp))
    print('작성: theta_wrap_test_{raw,exp}.hex, theta_wrap_test_deg.txt (%d개)' % len(test_rad))


if __name__ == '__main__':
    main()
