#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""particle_scorer.v(레이마칭+센서모델 결합) 테스트용 정답지 — 파티클 여러 개.

파티클마다 changwon 지도 위 자유공간 한 점에서 부채꼴로 8방향을 골라, 그 각각을
파이썬으로 레이마칭(gen_track_map.march와 동일 알고리즘)해서 "기대거리(d)"를
얻고, 거기에 노이즈를 살짝 얹어 "관측거리(r)"를 만든 다음, 센서모델 양자화
테이블(gen_sensor_model과 동일 수식)로 최종 파티클 점수(log-weight)까지 계산한다
— RTL(particle_scorer.v 여러 개를 병렬로)이 맞춰야 할 정답.

gen_sensor_model.py/gen_track_map.py를 그대로 import해서 재사용 — 물리/수식을
다시 베끼지 않는다(재구현이 아니라 재사용).
"""

import math

import numpy as np

from gen_sensor_model import (MAX_RANGE_M, MAP_RESOLUTION_M,
                              precompute_sensor_model_table, to_fixed_q)
from gen_track_map import (GRID_W, GRID_H, FRAC_W as RM_FRAC_W, load_occupancy,
                           compute_edt_shift, march_edt)

# 파티클마다 다른 자유공간을 찾을 검색 시작점 — 서로 확실히 떨어진 지점으로 골라서
# "같은 자리 두 번"이 아니라 진짜 다른 파티클임을 보여준다.
SEARCH_CENTERS = [(80, 200), (40, 80), (120, 320), (140, 150)]  # (y,x) 순서 — 아래 루프의 언패킹과 맞춤


def find_free_point(occ, y_start, x_start):
    if not occ[y_start, x_start]:
        return x_start, y_start
    for radius in range(1, 50):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                y, x = y_start + dy, x_start + dx
                if 0 <= y < GRID_H and 0 <= x < GRID_W and not occ[y, x]:
                    return x, y
    raise RuntimeError('자유공간을 못 찾음')


def w_signed(v, bits, scale):
    iv = int(round(v * scale))
    return iv & ((1 << bits) - 1)


def quantize(v, scale):
    """RTL이 실제로 받는 고정소수점 값으로 반올림 — 파이썬 오라클도 이 값을 써야
    한다. march_edt를 정밀 float 방향벡터로 돌리면, RTL은 양자화된(Q9.8) 방향을
    쓰기 때문에 코너 근처(예: 벽까지 거리가 벽 여유 1칸 이내) 케이스에서 파이썬과
    RTL이 다른 칸에서 충돌을 감지해 d가 ±1 어긋날 수 있다(실측으로 발견)."""
    return int(round(v * scale)) / scale


def score_one_particle(occ, shift, q, x0, y0, rng):
    """부채꼴 8방향 레이마칭 + 채점. (beams, total_q_masked) 반환.

    particle_scorer.v가 내부에서 ray_march_edt(거리장 기반 근사 마칭)를 쓰므로,
    정답지도 정확한 march()가 아니라 같은 근사 알고리즘인 march_edt()로 d를
    구해야 한다 — RTL과 다른 알고리즘으로 낸 "더 정확한" 값은 여기선 오답이다."""
    angles_deg = np.linspace(-60, 60, 8)
    scale = 1 << RM_FRAC_W
    beams = []
    total_q = 0
    for ang in angles_deg:
        rad = math.radians(ang)
        dx, dy = quantize(math.cos(rad), scale), quantize(math.sin(rad), scale)
        d, hit, _iters = march_edt(occ, shift, float(x0), float(y0), dx, dy, 300)
        noise = int(rng.integers(-3, 4))
        r = int(np.clip(d + noise, 0, 300))
        total_q += int(q[r, d])
        beams.append((dx, dy, r, d))
    return beams, total_q & ((1 << 20) - 1)


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
        rng = np.random.default_rng(idx + 1)  # 파티클마다 다른 노이즈 시드
        x0, y0 = find_free_point(occ, y_c, x_c)
        print('파티클 %d 시작점: x0=%d y0=%d' % (idx, x0, y0))
        beams, total_q = score_one_particle(occ, shift, q, x0, y0, rng)
        for ang, (dx, dy, r, d) in zip(np.linspace(-60, 60, 8), beams):
            print('  angle=%+6.1f -> d=%3d r=%3d' % (ang, d, r))
        print('  최종 log-weight: %d' % (total_q - (1 << 20 if total_q >= (1 << 19) else 0)))

        p = 'particle%d' % idx
        with open('%s_dx.hex' % p, 'w') as fdx, \
             open('%s_dy.hex' % p, 'w') as fdy, \
             open('%s_r.hex' % p, 'w') as fr:
            for dx, dy, r, d in beams:
                fdx.write('%05x\n' % w_signed(dx, 18, scale))
                fdy.write('%05x\n' % w_signed(dy, 18, scale))
                fr.write('%03x\n' % r)
        with open('%s_expected.hex' % p, 'w') as fe:
            fe.write('%05x\n' % total_q)
        with open('%s_x0y0.hex' % p, 'w') as fp:
            fp.write('%05x\n' % w_signed(float(x0), 18, scale))
            fp.write('%05x\n' % w_signed(float(y0), 18, scale))

    print('작성: particle{0,1,...}_{dx,dy,r,expected,x0y0}.hex')


if __name__ == '__main__':
    main()
