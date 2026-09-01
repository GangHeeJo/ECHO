#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""particle_scorer.v(레이마칭+센서모델 결합) 테스트용 정답지.

파티클 하나, 빔 8개짜리 시나리오를 만든다: changwon 지도 위 자유공간 한 점에서
부채꼴로 8방향을 골라, 그 각각을 파이썬으로 레이마칭(gen_track_map.march와 동일
알고리즘)해서 "기대거리(d)"를 얻고, 거기에 노이즈를 살짝 얹어 "관측거리(r)"를
만든 다음, 센서모델 양자화 테이블(gen_sensor_model과 동일 수식)로 최종 파티클
점수(log-weight)까지 계산한다 — RTL(particle_scorer.v)이 맞춰야 할 정답.

gen_sensor_model.py/gen_track_map.py를 그대로 import해서 재사용 — 물리/수식을
다시 베끼지 않는다(재구현이 아니라 재사용).
"""

import math

import numpy as np

from gen_sensor_model import (MAX_RANGE_M, MAP_RESOLUTION_M,
                              precompute_sensor_model_table, to_fixed_q)
from gen_track_map import GRID_W, GRID_H, FRAC_W as RM_FRAC_W, load_occupancy, march


def find_free_point(occ, y_start=80, x_start=200):
    """(y_start,x_start) 근처에서 자유공간 한 점을 찾는다."""
    if not occ[y_start, x_start]:
        return x_start, y_start
    for radius in range(1, 50):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                y, x = y_start + dy, x_start + dx
                if 0 <= y < GRID_H and 0 <= x < GRID_W and not occ[y, x]:
                    return x, y
    raise RuntimeError('자유공간을 못 찾음')


def main():
    rng = np.random.default_rng(1)

    # 1) 센서모델 양자화 테이블 (gen_sensor_model.py와 완전히 같은 절차)
    max_range_px = MAX_RANGE_M / MAP_RESOLUTION_M
    table = precompute_sensor_model_table(max_range_px)
    log_table = np.log(table)
    lo, hi = log_table.min(), log_table.max()
    int_bits = max(1, math.ceil(math.log2(max(abs(lo), abs(hi)) + 1))) + 1
    frac_bits = 8
    q = to_fixed_q(log_table, int_bits, frac_bits)

    # 2) changwon 지도 + 자유공간 시작점
    occ = load_occupancy('/home/ganghee/zero/src/track_assets/maps/changwon/map.pgm')
    x0, y0 = find_free_point(occ)
    print('시작점(자유공간 확인됨): x0=%d y0=%d' % (x0, y0))

    # 3) 부채꼴 8방향(-60~+60도)으로 레이마칭 -> 기대거리(d) -> 관측거리(r)=d+노이즈
    scale = 1 << RM_FRAC_W
    angles_deg = np.linspace(-60, 60, 8)
    beams = []
    total_q = 0
    for ang in angles_deg:
        rad = math.radians(ang)
        dx, dy = math.cos(rad), math.sin(rad)
        d, hit = march(occ, float(x0), float(y0), dx, dy, 300)
        noise = int(rng.integers(-3, 4))
        r = int(np.clip(d + noise, 0, 300))
        total_q += int(q[r, d])
        beams.append((dx, dy, r, d))
        print('  angle=%+6.1f dx=%.4f dy=%.4f -> d=%3d r=%3d (hit=%d)' % (ang, dx, dy, d, r, hit))

    acc_mask = (1 << 20) - 1
    total_q &= acc_mask
    print('최종 파티클 log-weight (Q%d.%d 정수, 20비트 acc): %d' % (int_bits, frac_bits, total_q))

    # 4) RTL 테스트벤치용 .hex — dx,dy는 Q9.8(RM_FRAC_W=8) signed, r은 9비트
    def w_signed(v, bits):
        iv = int(round(v * scale))
        return iv & ((1 << bits) - 1)

    with open('particle_dx.hex', 'w') as fdx, \
         open('particle_dy.hex', 'w') as fdy, \
         open('particle_r.hex', 'w') as fr:
        for dx, dy, r, d in beams:
            fdx.write('%05x\n' % w_signed(dx, 18))
            fdy.write('%05x\n' % w_signed(dy, 18))
            fr.write('%03x\n' % r)
    with open('particle_expected.hex', 'w') as fe:
        fe.write('%05x\n' % total_q)
    with open('particle_x0y0.hex', 'w') as fp:
        fp.write('%05x\n' % w_signed(float(x0), 18))
        fp.write('%05x\n' % w_signed(float(y0), 18))

    print('작성: particle_dx/dy/r.hex, particle_expected.hex, particle_x0y0.hex')


if __name__ == '__main__':
    main()
