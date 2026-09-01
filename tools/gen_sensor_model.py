#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ZERO particle_filter.py의 precompute_sensor_model()을 그대로 포팅해서,
ECHO(센서모델 평가 가속기) RTL이 맞춰야 할 "정답지"를 만든다.

- ROS/rclpy 없이 순수 numpy만 씀 (원본은 Node 클래스 메서드라 그대로 import 불가 —
  math 자체는 particle_filter.py:523-578과 라인 단위로 동일하게 옮김, 재구현이 아니라
  이식. 상수(z_hit 등)는 ~/zero/src/particle_filter/config/localize.yaml에서 그대로 가져옴).
- 하드웨어 설계 핵심 아이디어: 원본은 확률을 곱해서 파티클 가중치를 만드는데(레이마다
  곱셈 60번), 고정소수점 하드웨어에서 확률곱은 언더플로우가 심하다. 그래서 테이블 자체를
  **log(prob)**로 저장해두면, RTL은 매 레이마다 룩업 + 덧셈만 하면 된다(곱셈기 불필요).
  squash_factor 거듭제곱도 log 영역에서는 나눗셈(=상수곱)이라 그대로 살아남는다.
  최종 선형 가중치가 필요한 순간(리샘플링)만 소프트웨어에서 한 번 exp()하면 됨 —
  그건 파티클 수(500~4000)만큼만 하는 거라 병목이 아님.
"""

import argparse
import math

import numpy as np

# ~/zero/src/particle_filter/config/localize.yaml 그대로
Z_SHORT = 0.01
Z_MAX = 0.07
Z_RAND = 0.12
Z_HIT = 0.75
SIGMA_HIT = 8.0
MAP_RESOLUTION_M = 0.05   # track_assets 전 트랙 공통 확인됨
MAX_RANGE_M = 15.0        # localize.yaml max_range


def precompute_sensor_model_table(max_range_px):
    """particle_filter.py:523-578과 동일한 수식. (table_width, table_width) 확률표."""
    table_width = int(max_range_px) + 1
    table = np.zeros((table_width, table_width))
    for d in range(table_width):          # d: RangeLibc가 계산한 기대거리
        norm = 0.0
        for r in range(table_width):      # r: 라이다가 실제 관측한 거리
            prob = 0.0
            z = float(r - d)
            prob += (Z_HIT * np.exp(-(z * z) / (2.0 * SIGMA_HIT ** 2))
                     / (SIGMA_HIT * np.sqrt(2.0 * np.pi)))
            if r < d:
                prob += 2.0 * Z_SHORT * (d - r) / float(d) if d else 0.0
            if int(r) == int(max_range_px):
                prob += Z_MAX
            if r < int(max_range_px):
                prob += Z_RAND / float(max_range_px)
            norm += prob
            table[int(r), int(d)] = prob
        table[:, int(d)] /= norm
    return table


def to_fixed_q(values, int_bits, frac_bits):
    """실수 배열을 Q(int_bits).(frac_bits) 부호있는 고정소수점 정수로 양자화.
    (log(prob)는 항상 음수라 signed 고정 — unsigned 분기는 쓸 일이 없어서 뺌)"""
    scale = 1 << frac_bits
    q = np.round(values * scale).astype(np.int64)
    lo = -(1 << (int_bits + frac_bits - 1))
    hi = (1 << (int_bits + frac_bits - 1)) - 1
    clipped = np.clip(q, lo, hi)
    if (clipped != q).any():
        n = int((clipped != q).sum())
        print('  경고: %d개 값이 Q%d.%d 범위를 벗어나 클리핑됨' % (n, int_bits, frac_bits))
    return clipped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default='.')
    ap.add_argument('--frac-bits', type=int, default=8,
                     help='log(prob) 고정소수점 소수부 비트 수')
    args = ap.parse_args()

    max_range_px = MAX_RANGE_M / MAP_RESOLUTION_M
    print('MAX_RANGE_PX = %d (테이블 %dx%d)' % (max_range_px, max_range_px + 1, max_range_px + 1))

    table = precompute_sensor_model_table(max_range_px)
    log_table = np.log(table)  # 0 확률 없음 (z_rand가 바닥을 깔아줌) — 확인 아래서

    finite = np.isfinite(log_table)
    assert finite.all(), 'log(0) 발생 — z_rand/z_max 바닥 확인 필요'

    lo, hi = log_table.min(), log_table.max()
    print('log(prob) 범위: [%.4f, %.4f]' % (lo, hi))

    int_bits = max(1, math.ceil(math.log2(max(abs(lo), abs(hi)) + 1))) + 1  # 부호 비트 여유
    frac_bits = args.frac_bits
    total_bits = int_bits + frac_bits
    print('제안 고정소수점: Q%d.%d (총 %d비트, 룩업테이블 한 항목당)' % (int_bits, frac_bits, total_bits))

    q = to_fixed_q(log_table, int_bits, frac_bits)
    table_width = table.shape[0]
    table_bytes = table_width * table_width * (total_bits / 8.0)
    print('테이블 전체 크기: %d x %d x %d비트 = %.1f KB (BRAM 예산 확인용)'
          % (table_width, table_width, total_bits, table_bytes / 1024.0))

    # $readmemh용 hex 파일 — 주소 = r*table_width + d (r=관측, d=기대거리)
    hex_path = '%s/sensor_model_log_q%d_%d.hex' % (args.out_dir, int_bits, frac_bits)
    hex_width_chars = (total_bits + 3) // 4
    with open(hex_path, 'w') as fh:
        flat = q.flatten()  # row-major: [r=0,d=0..W-1], [r=1,d=0..W-1], ...
        mask = (1 << total_bits) - 1
        for v in flat:
            fh.write('%0*x\n' % (hex_width_chars, int(v) & mask))
    print('작성: %s (address = r*%d + d)' % (hex_path, table_width))

    # 테스트벡터 — 실제 스펙(빔 60개)로 파티클 5개 시나리오. RTL 테스트벤치가
    # $readmemh로 그대로 읽을 hex 2개(주소 목록, 파티클별 정답)로 낸다 — 사람이
    # 읽는 텍스트(예전 testvec_particles.txt)를 Verilog 소스에 손으로 옮겨적다
    # 오타 내는 걸 원천 차단하려는 것.
    rng = np.random.default_rng(0)
    num_rays = 60
    num_particles = 5
    ADDR_W = 17   # 301*301=90601 < 2^17 (table_mem.v와 동일)
    ACC_W = 20    # 빔 60개 합, 여유 있게 (sensor_pe.v와 동일)

    addr_w_chars = (ADDR_W + 3) // 4
    acc_w_chars = (ACC_W + 3) // 4
    acc_mask = (1 << ACC_W) - 1

    addrs_path = '%s/testvec_addrs.hex' % args.out_dir
    exp_path = '%s/testvec_expected.hex' % args.out_dir
    # r/d 원본 쌍도 따로 저장 — addr_gen.v(주소생성 하드웨어)를 실제 (r,d) 입력으로
    # 검증하려면 필요함. addrs_path(미리 계산된 주소)는 addr_gen 없이 PE만 볼 때 계속 씀.
    r_path = '%s/testvec_r.hex' % args.out_dir
    d_path = '%s/testvec_d.hex' % args.out_dir
    RD_W = 9   # 0~300 담는 데 필요한 비트 수 (2^9=512)
    rd_w_chars = (RD_W + 3) // 4
    with open(addrs_path, 'w') as fa, open(exp_path, 'w') as fe, \
         open(r_path, 'w') as fr, open(d_path, 'w') as fd:
        for p in range(num_particles):
            obs = rng.integers(0, int(max_range_px) + 1, size=num_rays)
            exp_r = rng.integers(0, int(max_range_px) + 1, size=num_rays)
            # RTL은 "이미 반올림된 표 값(q)"을 정수로 더한다 — 정답지도 float을 더한 뒤
            # 한 번에 반올림하면 반올림 순서가 달라져서 RTL과 최대 ±1 LSB 어긋난다
            # (8빔 버전에서 실제로 5개 중 2개가 그랬음). 그래서 여기서도 "이미
            # 양자화된 표 값"을 정수로 더해야 RTL과 비트가 맞는다.
            log_w_q = 0
            for o, d in zip(obs, exp_r):
                addr = o * table_width + d
                fa.write('%0*x\n' % (addr_w_chars, addr))
                fr.write('%0*x\n' % (rd_w_chars, int(o)))
                fd.write('%0*x\n' % (rd_w_chars, int(d)))
                log_w_q += int(q[o, d])
            fe.write('%0*x\n' % (acc_w_chars, log_w_q & acc_mask))
    print('작성: %s, %s, %s, %s (파티클 %d개 x 빔 %d개, RTL 테스트벤치용)'
          % (addrs_path, exp_path, r_path, d_path, num_particles, num_rays))


if __name__ == '__main__':
    main()
