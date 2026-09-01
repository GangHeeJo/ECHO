#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""changwon 트랙의 실제 map.pgm을 ray_march.v가 읽을 occupancy grid(.hex/.bin)로
변환하고, 같은 고정스텝 알고리즘을 파이썬으로 재현해 RTL 테스트벤치의 "정답"을
같이 만든다. 지도가 복잡해서(진짜 트랙 모양) 손으로 계산할 수 없으니, 여기서
소프트웨어로 먼저 걸어보고 그 결과를 RTL과 대조하는 것 — 이번에도 "소프트웨어
정답지 먼저" 원칙 그대로.
"""

import argparse
import math

import numpy as np
from PIL import Image

GRID_W = 400            # changwon map.pgm 실제 폭(칸)
GRID_H = 160             # 실제 높이(칸)
PADDED_W = 512            # 2의 거듭제곱으로 패딩 — 주소 = {y,9'b0}|x, 곱셈/시프트-덧셈 불필요
FRAC_W = 8                # Q9.8 고정소수점(정수부 9비트: 0~511 커버, 소수부 8비트)
MAX_STEPS = 300           # 15m / 0.05m = 300칸 (실제 라이다 최대거리)


def load_occupancy(pgm_path):
    """map.pgm -> (GRID_H, GRID_W) bool 배열. True=벽(occupied)."""
    im = Image.open(pgm_path)
    arr = np.array(im)
    assert arr.shape == (GRID_H, GRID_W), '예상 크기(%d,%d)와 다름: %s' % (GRID_H, GRID_W, arr.shape)
    # ROS map_server 관례: 0(검정)=occupied, 255(흰색)=free. changwon은 0/255만 있어 깔끔함.
    occ = (arr < 128)
    return occ


def write_readmemb(occ, out_path):
    """진짜 평면(flat) BRAM처럼 셀 하나당 한 줄(0/1) — 주소 = y*PADDED_W+x로 그대로
    인덱싱한다(ray_march.v의 2차원 배열 인덱싱과 다르게, 실제 합성 가능한 방식).
    실제 폭(400) 밖(400~511)은 전부 1(벽)로 채움 — out_of_bounds 체크와 이중 안전장치.
    ponytail: 1비트짜리 항목을 그대로 BRAM 한 칸씩 쓰는 거라 실리콘 낭비가 있음 —
    32비트씩 묶어서 워드 단위로 저장하면 더 효율적인데, 이번 단계 스코프 밖으로 미룸."""
    with open(out_path, 'w') as fh:
        for y in range(GRID_H):
            for x in range(PADDED_W):
                bit = occ[y, x] if x < GRID_W else True
                fh.write('1\n' if bit else '0\n')
    print('작성: %s (%d x %d = %d줄, 셀당 1비트)' % (out_path, GRID_H, PADDED_W, GRID_H * PADDED_W))


def march(occ, x0, y0, dx, dy, max_steps):
    """RTL(ray_march.v)과 정확히 같은 알고리즘: 고정스텝 1.0, 정수부만 보고 충돌판정."""
    x, y = x0, y0
    for steps in range(max_steps + 1):
        xi, yi = int(math.floor(x)), int(math.floor(y))
        out = xi < 0 or xi >= GRID_W or yi < 0 or yi >= GRID_H
        if out or occ[yi, xi]:
            return steps, True
        if steps >= max_steps:
            return steps, False
        x += dx
        y += dy
    return max_steps, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='/home/ganghee/zero/src/track_assets/maps/changwon/map.pgm')
    ap.add_argument('--out-dir', default='.')
    args = ap.parse_args()

    occ = load_occupancy(args.map)
    write_readmemb(occ, '%s/changwon_occ.hex' % args.out_dir)

    # 트랙 중앙 부근(대략 자유공간일 가능성이 높은 지점)에서 몇 가지 방향으로 쏴봄.
    # raceline 근처가 아니라 그냥 지도 중앙이라 실제로 몇 칸만에 벽에 부딪힐 수 있음
    # — 그래도 "소프트웨어가 먼저 계산한 값과 RTL이 맞는지" 검증에는 문제 없음.
    scale = 1 << FRAC_W
    scenarios = [
        ('center_+x',  200.0, 80.0, 1.0, 0.0),
        ('center_-x',  200.0, 80.0, -1.0, 0.0),
        ('center_+y',  200.0, 80.0, 0.0, 1.0),
        ('center_-y',  200.0, 80.0, 0.0, -1.0),
        ('diag',       200.0, 80.0, 0.70703125, 0.70703125),  # 181/256
        ('near_wall',   10.0, 80.0, -1.0, 0.0),
    ]

    print('\n%-12s %8s %8s %8s %8s -> %6s %4s' % ('name', 'x0', 'y0', 'dx', 'dy', 'dist', 'hit'))
    with open('%s/track_scenarios.txt' % args.out_dir, 'w') as fh:
        for name, x0, y0, dx, dy in scenarios:
            dist, hit = march(occ, x0, y0, dx, dy, MAX_STEPS)
            print('%-12s %8.2f %8.2f %8.4f %8.4f -> %6d %4d' % (name, x0, y0, dx, dy, dist, int(hit)))
            fh.write('%s %d %d %d %d %d %d\n' % (
                name,
                int(round(x0 * scale)), int(round(y0 * scale)),
                int(round(dx * scale)), int(round(dy * scale)),
                dist, int(hit)))
    print('\n작성: %s/track_scenarios.txt (RTL 테스트벤치용, Q9.8 고정소수점 정수값)' % args.out_dir)


if __name__ == '__main__':
    main()
