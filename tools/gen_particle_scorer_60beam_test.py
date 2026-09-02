#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""particle_scorer.v를 실제 ZERO와 같은 빔 개수(60, angle_step=18 다운샘플 실측치)
로 돌려 진짜 latency를 재기 위한 정답지 1개. gen_particle_scorer_test.py의
particle0과 완전히 같은 자유공간 시작점(SEARCH_CENTERS[0]=(80,200))을 쓰되
빔 개수만 8 -> 60으로 늘림 — "동일 조건 벤치마크"의 ECHO쪽 절반.

실제 ZERO(particle_filter.py)의 downsampled_angles는 라이다 원본 FOV를 그대로
등분한 게 아니라 angle_step=18로 서브샘플한 것이라 정확한 각도폭은 라이다
장착값에 달렸지만, "빔 60개, 부채꼴로 고르게" 정도의 근사면 이 벤치마크
목적(레이마칭 1회당 비용 측정)엔 충분 — 정확한 FOV는 정확도 비교 단계(이번
스코프 밖)에서나 중요해짐.
"""

import math

import numpy as np

from gen_sensor_model import (MAX_RANGE_M, MAP_RESOLUTION_M,
                              precompute_sensor_model_table, to_fixed_q)
from gen_track_map import (GRID_W, GRID_H, FRAC_W as RM_FRAC_W, load_occupancy,
                           compute_edt_shift, march_edt)
from gen_particle_scorer_test import find_free_point, w_signed, quantize, SEARCH_CENTERS

NUM_RAYS = 60


def gen_one(occ, shift, q, scale, idx, y_c, x_c):
    x0, y0 = find_free_point(occ, y_c, x_c)
    print('p60_%d 시작점: x0=%d y0=%d' % (idx, x0, y0))

    rng = np.random.default_rng(idx + 1)  # gen_particle_scorer_test.py와 동일 시드 규칙
    angles_deg = np.linspace(-60, 60, NUM_RAYS)
    total_q = 0
    dxs, dys, rs = [], [], []
    for ang in angles_deg:
        rad = math.radians(ang)
        dx, dy = quantize(math.cos(rad), scale), quantize(math.sin(rad), scale)
        d, hit, _iters = march_edt(occ, shift, float(x0), float(y0), dx, dy, 300)
        noise = int(rng.integers(-3, 4))
        r = int(np.clip(d + noise, 0, 300))
        total_q += int(q[r, d])
        dxs.append(dx); dys.append(dy); rs.append(r)
    total_q &= (1 << 20) - 1

    p = 'p60_%d' % idx
    with open('%s_dx.hex' % p, 'w') as fdx, \
         open('%s_dy.hex' % p, 'w') as fdy, \
         open('%s_r.hex' % p, 'w') as fr:
        for dx, dy, r in zip(dxs, dys, rs):
            fdx.write('%05x\n' % w_signed(dx, 18, scale))
            fdy.write('%05x\n' % w_signed(dy, 18, scale))
            fr.write('%03x\n' % r)
    with open('%s_expected.hex' % p, 'w') as fe:
        fe.write('%05x\n' % total_q)
    with open('%s_x0y0.hex' % p, 'w') as fp:
        fp.write('%05x\n' % w_signed(float(x0), 18, scale))
        fp.write('%05x\n' % w_signed(float(y0), 18, scale))

    print('  최종 log-weight: %d' % (total_q - (1 << 20 if total_q >= (1 << 19) else 0)))


def main():
    max_range_px = MAX_RANGE_M / MAP_RESOLUTION_M
    table = precompute_sensor_model_table(max_range_px)
    log_table = np.log(table)
    lo, hi = log_table.min(), log_table.max()
    int_bits = max(1, math.ceil(math.log2(max(abs(lo), abs(hi)) + 1))) + 1
    frac_bits = 8
    q = to_fixed_q(log_table, int_bits, frac_bits)
    scale = 1 << RM_FRAC_W

    occ = load_occupancy('/home/ganghee/zero/src/track_assets/maps/changwon/map.pgm')
    shift = compute_edt_shift(occ)

    for idx, (y_c, x_c) in enumerate(SEARCH_CENTERS):
        gen_one(occ, shift, q, scale, idx, y_c, x_c)

    print('작성: p60_{0..7}_{dx,dy,r,expected,x0y0}.hex (%d빔 x 8파티클, '
          'gen_particle_scorer_test.py와 같은 SEARCH_CENTERS 재사용)' % NUM_RAYS)


if __name__ == '__main__':
    main()
