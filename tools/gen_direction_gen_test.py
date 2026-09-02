#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""direction_gen.v(CORDIC 기반 방향생성기) 검증 — bench/zero_frame_001.npz의
실제 ZERO 파티클 theta·실제 60빔 각도를 그대로 써서, "진짜 이 프로젝트가
풀어야 했던 그 데이터"로 dx,dy를 재현할 수 있는지 확인한다(합성 테스트
케이스가 아니라 실측 프레임으로 닫는 검증 루프).
"""

import math

import numpy as np

FRAC_W = 16
W_EXT = 19
SCALE_EXT = 1 << FRAC_W

OUT_FRAC_W = 8
W_OUT = 18
SCALE_OUT = 1 << OUT_FRAC_W

NUM_PARTICLES_TEST = 5


def to_fixed_ext(v):
    iv = int(round(v * SCALE_EXT))
    assert -(1 << (W_EXT - 1)) <= iv < (1 << (W_EXT - 1)), 'overflow: %r' % v
    return iv & ((1 << W_EXT) - 1)


def to_fixed_out(v):
    iv = int(round(v * SCALE_OUT))
    assert -(1 << (W_OUT - 1)) <= iv < (1 << (W_OUT - 1)), 'overflow: %r' % v
    return iv & ((1 << W_OUT) - 1)


def wrap_pi(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def main():
    d = np.load('/home/ganghee/echo/bench/zero_frame_001.npz')
    angles = d['angles']          # (60,) 실제 빔 상대각(라디안)
    particles = d['particles']    # (500,3) 실제 파티클 x,y,theta

    assert len(angles) == 60, 'NUM_RAYS=60 가정과 다름: %d' % len(angles)

    with open('beam_angles.hex', 'w') as f:
        for a in angles:
            f.write('%05x\n' % to_fixed_ext(float(a)))
    print('작성: beam_angles.hex (실제 ZERO 60빔 각도, %.4f~%.4frad)'
          % (angles.min(), angles.max()))

    thetas = particles[:NUM_PARTICLES_TEST, 2]
    with open('direction_gen_test_theta.hex', 'w') as ft:
        for th in thetas:
            th_wrapped = wrap_pi(float(th))
            ft.write('%05x\n' % to_fixed_ext(th_wrapped))

    # 파티클 x NUM_RAYS 전체 조합 기대값(dx,dy, Q9.8) — 실제 RTL이 계산해야
    # 할 그 값. CORDIC 근사 오차(Q3.16 기준 최대 4LSB)가 있어 완전 비트일치는
    # 기대 안 함(테스트벤치에서 LSB 허용치로 비교).
    with open('direction_gen_test_dx.hex', 'w') as fdx, \
         open('direction_gen_test_dy.hex', 'w') as fdy:
        for th in thetas:
            th_wrapped = wrap_pi(float(th))
            for ang in angles:
                world = wrap_pi(th_wrapped + float(ang))
                fdx.write('%05x\n' % to_fixed_out(math.cos(world)))
                fdy.write('%05x\n' % to_fixed_out(math.sin(world)))

    print('작성: direction_gen_test_theta.hex(%d개 파티클), '
          'direction_gen_test_{dx,dy}.hex(%d x %d = %d개 케이스)'
          % (NUM_PARTICLES_TEST, NUM_PARTICLES_TEST, len(angles),
             NUM_PARTICLES_TEST * len(angles)))
    print('실제 파티클 theta(rad): %s' % np.round(thetas, 4).tolist())


if __name__ == '__main__':
    main()
