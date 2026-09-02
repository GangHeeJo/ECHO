#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tb_particle_scorer_dgen_oct_arb_realbatch.v용 — 실제 ZERO 파티클 500개의
theta를 gen_real_frame_test.py와 완전히 같은 63배치x8 나눔(rf_bB_pS_*)으로
rf_bB_pS_theta.hex에 추가 저장. x0,y0/r_obs는 이미 있는 rf_bB_pS_x0y0.hex/
r.hex를 그대로 재사용(direction_gen이 dx,dy를 대신 만들어주는 것만 다름).
"""

import math

import numpy as np

FRAC_W = 16
W_EXT = 19
SCALE = 1 << FRAC_W
NUM_PARTICLES = 500
BATCH = 8


def to_fixed_ext(v):
    iv = int(round(v * SCALE))
    assert -(1 << (W_EXT - 1)) <= iv < (1 << (W_EXT - 1)), 'overflow: %r' % v
    return iv & ((1 << W_EXT) - 1)


def wrap_pi(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def main():
    d = np.load('/home/ganghee/echo/bench/zero_frame_001.npz')
    particles = d['particles']
    thetas_wrapped = [wrap_pi(float(t)) for t in particles[:, 2]]

    n_batches = (NUM_PARTICLES + BATCH - 1) // BATCH  # 63
    for b in range(n_batches):
        for slot in range(BATCH):
            pidx = min(b * BATCH + slot, NUM_PARTICLES - 1)
            prefix = 'rf_b%d_p%d' % (b, slot)
            with open('%s_theta.hex' % prefix, 'w') as f:
                f.write('%05x\n' % to_fixed_ext(thetas_wrapped[pidx]))

    print('작성: rf_b{0..%d}_p{0..7}_theta.hex (%d배치 x 8)' % (n_batches - 1, n_batches))


if __name__ == '__main__':
    main()
