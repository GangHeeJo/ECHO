#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench/zero_frame_001.npz(실제 ZERO 프레임 덤프: 파티클 500개, 실제 스캔,
range_libc 예측거리, weight, 타이밍)를 ECHO 파이프라인에 그대로 흘려서:
  1) 좌표계가 실제로 맞는지 검증(파티클이 occ 그리드에서 대부분 자유공간에
     있어야 정상 — 아니면 축/원점/스케일 어긋남)
  2) 500파티클×60빔을 ECHO의 march_edt+양자화 테이블로 직접 재계산해
     RTL이 받을 실제 테스트벡터(dx,dy는 파티클 theta까지 반영 — beam_id만으론
     안 된다는 지적이 바로 이 부분, 여기서는 Jetson 쪽에서 이미 계산해서
     보내주는 시나리오를 그대로 재현)
  3) ECHO 자체 계산 weight(로그 도메인 합산)와 ZERO 실측 weight(squash 후
     선형 도메인)의 순위 상관관계 비교 — squash는 단조변환이라 순위는
     보존되어야 정상, 어긋나면 근사오차가 순위에 영향을 준다는 뜻
  4) 500파티클을 8개씩 63배치로 나눠 particle_scorer_oct_arb 테스트벤치용
     hex 파일 생성(마지막 배치는 4개 부족해 앞 4개를 재사용해 채움 — 사이클
     측정엔 실제 파티클 자원을 재사용하는 거라 왜곡 없음, 근거는 progress.md)
"""

import math

import numpy as np

from gen_sensor_model import (MAX_RANGE_M, MAP_RESOLUTION_M,
                              precompute_sensor_model_table, to_fixed_q)
from gen_track_map import GRID_W, GRID_H, FRAC_W as RM_FRAC_W, load_occupancy, compute_edt_shift, march_edt
from gen_particle_scorer_test import w_signed

FRAME = '/home/ganghee/echo/bench/zero_frame_001.npz'
NUM_RAYS = 60
NUM_PARTICLES = 500
BATCH = 8


def main():
    d = np.load(FRAME)
    angles = d['angles']              # (60,) rad, 라이다 프레임 상대각
    particles = d['particles']        # (500,3) x_m,y_m,theta_rad, world/map frame
    weights_zero = d['weights']       # (500,) squash 후 선형(미정규화)
    origin = d['map_origin']          # [ox, oy] m
    res = float(d['map_resolution'])  # m/px

    occ = load_occupancy('/home/ganghee/zero/src/track_assets/maps/changwon/map.pgm')
    shift = compute_edt_shift(occ)

    # ── 1) 좌표계 검증 ──────────────────────────────────────────────
    px = (particles[:, 0] - origin[0]) / res
    py = (particles[:, 1] - origin[1]) / res
    in_bounds = (px >= 0) & (px < GRID_W) & (py >= 0) & (py < GRID_H)
    print('좌표 변환: px range [%.1f, %.1f], py range [%.1f, %.1f]'
          % (px.min(), px.max(), py.min(), py.max()))
    print('격자 범위(0~%d, 0~%d) 안: %d / %d' % (GRID_W, GRID_H, in_bounds.sum(), NUM_PARTICLES))
    ib_idx = np.where(in_bounds)[0]
    free_count = sum(1 for i in ib_idx if not occ[int(py[i]), int(px[i])])
    print('격자 안 파티클 중 자유공간(비occupied): %d / %d'
          % (free_count, len(ib_idx)))
    if len(ib_idx) < NUM_PARTICLES * 0.9 or free_count < len(ib_idx) * 0.5:
        print('⚠️ 좌표계가 안 맞을 가능성 높음(격자 이탈 또는 대부분 벽) — '
              '축 반전(py 부호/원점)을 의심할 것. 계속 진행은 하되 결과 신뢰 낮음.')

    # ── 2) 500x60 실제 재계산 (ECHO 알고리즘, 실제 theta 반영) ──────
    max_range_px = MAX_RANGE_M / MAP_RESOLUTION_M
    table = precompute_sensor_model_table(max_range_px)
    log_table = np.log(table)
    lo, hi = log_table.min(), log_table.max()
    int_bits = max(1, math.ceil(math.log2(max(abs(lo), abs(hi)) + 1))) + 1
    q = to_fixed_q(log_table, int_bits, 8)
    scale = 1 << RM_FRAC_W

    echo_logw = np.zeros(NUM_PARTICLES)
    all_dx, all_dy, all_r = [], [], []   # RTL 테스트벡터용
    all_px0, all_py0 = [], []
    n_oob = 0
    for pidx in range(NUM_PARTICLES):
        x0, y0, theta = particles[pidx]
        gx, gy = (x0 - origin[0]) / res, (y0 - origin[1]) / res
        if not (0 <= gx < GRID_W and 0 <= gy < GRID_H):
            n_oob += 1
            gx = min(max(gx, 1), GRID_W - 2)
            gy = min(max(gy, 1), GRID_H - 2)
        obs_px = d['obs'] / res  # obs는 실제 관측 range(m) -> px
        total_q = 0
        dxs, dys, rs = [], [], []
        for bidx in range(NUM_RAYS):
            world_ang = theta + angles[bidx]
            dx = round(math.cos(world_ang) * scale) / scale
            dy = round(math.sin(world_ang) * scale) / scale
            dist, hit, _it = march_edt(occ, shift, float(gx), float(gy), dx, dy, 300)
            r = int(np.clip(round(obs_px[bidx]), 0, 300))
            total_q += int(q[r, dist])
            dxs.append(dx); dys.append(dy); rs.append(r)
        total_q &= (1 << 20) - 1
        echo_logw[pidx] = total_q - (1 << 20 if total_q >= (1 << 19) else 0)
        all_dx.append(dxs); all_dy.append(dys); all_r.append(rs)
        all_px0.append(gx); all_py0.append(gy)
        if pidx % 50 == 0:
            print('  진행 %d/%d' % (pidx, NUM_PARTICLES))
    print('격자 밖이라 클램프한 파티클: %d / %d' % (n_oob, NUM_PARTICLES))

    # ── 3) 순위 비교 ────────────────────────────────────────────────
    # squash(x^(1/2.2))는 단조증가라 ranking은 보존돼야 정상. Spearman 대용으로
    # scipy 없이 순위 상관(스피어만) 직접 계산.
    def rankdata(a):
        order = np.argsort(a)
        ranks = np.empty_like(order, dtype=float)
        ranks[order] = np.arange(len(a))
        return ranks
    r_echo = rankdata(echo_logw)
    r_zero = rankdata(weights_zero)
    n = NUM_PARTICLES
    d2 = np.sum((r_echo - r_zero) ** 2)
    spearman = 1 - 6 * d2 / (n * (n ** 3 - 1))
    top10_echo = set(np.argsort(-echo_logw)[:10])
    top10_zero = set(np.argsort(-weights_zero)[:10])
    overlap = len(top10_echo & top10_zero)
    print('Spearman rank correlation(ECHO logw vs ZERO squashed w): %.4f' % spearman)
    print('상위 10개 파티클 겹침: %d / 10' % overlap)

    np.savez('/home/ganghee/echo/bench/echo_recompute_001.npz',
             echo_logw=echo_logw, weights_zero=weights_zero,
             spearman=spearman, top10_overlap=overlap)

    # ── 4) RTL 63배치 hex 생성 ──────────────────────────────────────
    all_dx = np.array(all_dx); all_dy = np.array(all_dy); all_r = np.array(all_r)
    n_batches = (NUM_PARTICLES + BATCH - 1) // BATCH  # 63
    for b in range(n_batches):
        idxs = [min(b * BATCH + k, NUM_PARTICLES - 1) for k in range(BATCH)]
        # 마지막 배치(62번, 0-indexed)는 496..503 중 500..503이 범위밖 -> 499로 클램프(재사용)
        for slot, pidx in enumerate(idxs):
            prefix = 'rf_b%d_p%d' % (b, slot)
            with open('%s_dx.hex' % prefix, 'w') as fdx, \
                 open('%s_dy.hex' % prefix, 'w') as fdy, \
                 open('%s_r.hex' % prefix, 'w') as fr:
                for k in range(NUM_RAYS):
                    fdx.write('%05x\n' % w_signed(all_dx[pidx][k], 18, scale))
                    fdy.write('%05x\n' % w_signed(all_dy[pidx][k], 18, scale))
                    fr.write('%03x\n' % all_r[pidx][k])
            with open('%s_x0y0.hex' % prefix, 'w') as fp:
                fp.write('%05x\n' % w_signed(all_px0[pidx], 18, scale))
                fp.write('%05x\n' % w_signed(all_py0[pidx], 18, scale))
    print('작성: rf_b{0..%d}_p{0..7}_{dx,dy,r,x0y0}.hex (%d배치 x 8, 실제 파티클 500개 재사용)'
          % (n_batches - 1, n_batches))


if __name__ == '__main__':
    main()
