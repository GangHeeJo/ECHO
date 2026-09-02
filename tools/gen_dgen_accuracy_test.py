#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4단계(로드맵) — direction_gen(CORDIC)이 실제로 만드는 dx,dy로 500파티클
전체를 재계산해서, "정확한 삼각함수" 기준으로 이미 봤던 정확도(Spearman
0.998, ESS 23.4%)와 비교. RTL을 30000번(500파티클x60빔) 시뮬레이션 돌리는
대신, RTL의 CORDIC 알고리즘(N=16 반복, 시프트-덧셈, 사분면 접기)을 파이썬
정수 연산으로 그대로 재현 — cordic_sincos.v/cordic_sincos_full.v와 비트
단위로 같은 로직(다르면 이 스크립트가 잘못된 것).
"""

import math

import numpy as np

from gen_sensor_model import MAX_RANGE_M, MAP_RESOLUTION_M, precompute_sensor_model_table, to_fixed_q
from gen_track_map import GRID_W, GRID_H, FRAC_W as RM_FRAC_W, load_occupancy, compute_edt_shift, march_edt

FRAME = '/home/ganghee/echo/bench/zero_frame_001.npz'
NUM_RAYS = 60
NUM_PARTICLES = 500

# ── cordic_sincos.v와 동일 상수(Q1.16, W=18) ────────────────────────────
N_ITER = 16
W_CORE = 18
FRAC_CORE = 16
X0 = 39797  # gen_cordic_table.py의 compute_x0(16) -> to_fixed
ATANS = [int(round(math.atan(2.0 ** (-i)) * (1 << FRAC_CORE))) for i in range(N_ITER)]

# ── cordic_sincos_full.v와 동일 상수(Q3.16, W=19) ───────────────────────
W_EXT = 19
PI_CONST = 205887
PI_HALF_CONST = 102944

# ── angle_wrap.v와 동일 상수(W_SUM=20) ──────────────────────────────────
TWO_PI = 411775


def sign_extend(v, width):
    if v >= (1 << (width - 1)):
        v -= (1 << width)
    return v


def trunc(v, width):
    return v & ((1 << width) - 1)


def cordic_core(theta_i_raw):
    """cordic_sincos.v — theta_i(Q1.16, W_CORE비트, [-90,90]도 안)를 그대로
    N_ITER회 반복. 비트폭 W_CORE로 매 스텝 마스킹(RTL의 레지스터 폭과 동일)."""
    x = X0
    y = 0
    z = sign_extend(theta_i_raw, W_CORE)
    for i in range(N_ITER):
        # 산술 시프트(부호 유지) — 파이썬 >>는 음수에서도 산술시프트라 그대로 씀
        xs = sign_extend(x, W_CORE)
        ys = sign_extend(y, W_CORE)
        x_shift = xs >> i
        y_shift = ys >> i
        d_neg = z < 0
        if d_neg:
            x_next = xs + y_shift
            y_next = ys - x_shift
            z_next = z + ATANS[i]
        else:
            x_next = xs - y_shift
            y_next = ys + x_shift
            z_next = z - ATANS[i]
        x = trunc(x_next, W_CORE)
        y = trunc(y_next, W_CORE)
        z = sign_extend(trunc(z_next, W_CORE), W_CORE)
    return sign_extend(x, W_CORE), sign_extend(y, W_CORE)


def cordic_full(theta_raw_ext):
    """cordic_sincos_full.v — theta(Q3.16, W_EXT비트, |theta|<=pi)를 사분면
    접어서 코어에 넣고 부호 복원."""
    theta = sign_extend(theta_raw_ext, W_EXT)
    abs_theta = -theta if theta < 0 else theta
    fold = abs_theta > PI_HALF_CONST
    reduced = (PI_CONST - abs_theta) if fold else abs_theta
    reduced_core = trunc(reduced, W_CORE)  # 하위 W_CORE비트만 코어에
    core_cos, core_sin = cordic_core(reduced_core)
    cos_ext = -core_cos if fold else core_cos
    sin_ext = -core_sin if (theta < 0) else core_sin
    return cos_ext, sin_ext


def to_fixed19(v):
    iv = int(round(v * (1 << 16)))
    assert -(1 << 18) <= iv < (1 << 18)
    return iv


def to_fixed_q98(v):
    return int(round(v * 256.0))


def wrap_pi(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def rankdata(a):
    order = np.argsort(a)
    ranks = np.empty_like(order, dtype=float)
    ranks[order] = np.arange(len(a))
    return ranks


def main():
    d = np.load(FRAME)
    angles = d['angles']
    particles = d['particles']
    weights_zero = d['weights']
    origin = d['map_origin']
    res = float(d['map_resolution'])

    # beam_angles.hex와 동일 값(direction_gen이 실제로 읽는 ROM)
    beam_angle_fixed = [to_fixed19(float(a)) for a in angles]

    occ = load_occupancy('/home/ganghee/zero/src/track_assets/maps/changwon/map.pgm')
    shift = compute_edt_shift(occ)
    max_range_px = MAX_RANGE_M / MAP_RESOLUTION_M
    table = precompute_sensor_model_table(max_range_px)
    log_table = np.log(table)
    lo, hi = log_table.min(), log_table.max()
    int_bits = max(1, math.ceil(math.log2(max(abs(lo), abs(hi)) + 1))) + 1
    q = to_fixed_q(log_table, int_bits, 8)

    cordic_logw = np.zeros(NUM_PARTICLES)
    n_sanity = 0
    for pidx in range(NUM_PARTICLES):
        x0, y0, theta = particles[pidx]
        theta_w = wrap_pi(float(theta))
        theta_fixed = to_fixed19(theta_w)
        gx, gy = (x0 - origin[0]) / res, (y0 - origin[1]) / res
        gx = min(max(gx, 1), GRID_W - 2)
        gy = min(max(gy, 1), GRID_H - 2)
        obs_px = d['obs'] / res

        total_q = 0
        for bidx in range(NUM_RAYS):
            # angle_wrap.v와 동일: 부호확장된 정수합에 최대 1회 +-2pi 보정
            # (TWO_PI=411775 — gen_angle_wrap_table.py의 to_fixed(2pi, 20)과
            # 정확히 같은 상수, RTL의 localparam과 일치)
            world_signed = sign_extend(theta_fixed, W_EXT) + sign_extend(beam_angle_fixed[bidx], W_EXT)
            if world_signed > PI_CONST:
                world_signed -= TWO_PI
            elif world_signed < -PI_CONST:
                world_signed += TWO_PI
            world_fixed = trunc(world_signed, W_EXT)

            cos_c, sin_c = cordic_full(world_fixed)
            dx_q98 = trunc(cos_c >> 8, 18)  # Q1.16 -> Q9.8 (direction_gen.v와 동일 시프트)
            dy_q98 = trunc(sin_c >> 8, 18)
            dx = sign_extend(dx_q98, 18) / 256.0
            dy = sign_extend(dy_q98, 18) / 256.0

            dist, hit, _it = march_edt(occ, shift, float(gx), float(gy), dx, dy, 300)
            r = int(np.clip(round(obs_px[bidx]), 0, 300))
            total_q += int(q[r, dist])
        total_q &= (1 << 20) - 1
        cordic_logw[pidx] = total_q - (1 << 20 if total_q >= (1 << 19) else 0)
        if pidx % 100 == 0:
            print('진행 %d/%d' % (pidx, NUM_PARTICLES))

    # sanity: particle 0의 CORDIC weight가 RTL 실측(-46170)과 일치해야 함
    print('particle0 CORDIC weight(파이썬 재현): %d (RTL 실측 -46170과 비교)' % cordic_logw[0])

    r_cordic = rankdata(cordic_logw)
    r_zero = rankdata(weights_zero)
    n = NUM_PARTICLES
    d2 = np.sum((r_cordic - r_zero) ** 2)
    spearman = 1 - 6 * d2 / (n * (n ** 3 - 1))
    top10_cordic = set(np.argsort(-cordic_logw)[:10])
    top10_zero = set(np.argsort(-weights_zero)[:10])
    overlap = len(top10_cordic & top10_zero)

    w_zero_n = weights_zero / weights_zero.sum()
    ess_zero = 1.0 / np.sum(w_zero_n ** 2)

    real_log = cordic_logw / 256.0
    lw = real_log - real_log.max()
    w_raw = np.exp(lw)
    w_squashed = np.power(w_raw, 1 / 2.2)
    w_n = w_squashed / w_squashed.sum()
    ess_cordic = 1.0 / np.sum(w_n ** 2)

    print('\n=== CORDIC 버전 정확도 (vs 정확한 삼각함수 버전) ===')
    print('Spearman: %.4f (정확한 버전: 0.9979)' % spearman)
    print('top10 겹침: %d/10 (정확한 버전: 1/10)' % overlap)
    print('ESS: CORDIC %.1f%% vs ZERO %.1f%% (정확한 버전 ESS: 23.4%%)' %
          (100 * ess_cordic / n, 100 * ess_zero / n))

    np.savez('/home/ganghee/echo/bench/cordic_recompute_001.npz',
             cordic_logw=cordic_logw, weights_zero=weights_zero,
             spearman=spearman, top10_overlap=overlap,
             ess_cordic=ess_cordic, ess_zero=ess_zero)


if __name__ == '__main__':
    main()
