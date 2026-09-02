#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CORDIC(COordinate Rotation DIgital Computer) sin/cos 계산기용 상수 생성.

이 프로젝트의 곱셈기 없는 설계 철학(로그 테이블+덧셈, 시프트-덧셈 주소생성,
배럴시프터 레이마칭)과 정확히 같은 결의 알고리즘 — CORDIC은 회전을 "각도별
고정 arctan 값만큼 부호 있는 시프트-덧셈"으로 반복해서 sin/cos를 곱셈기 없이
계산한다.

수렴 범위: 순환(circular) 모드 CORDIC은 theta가 [-99.7deg, 99.7deg](arctan(2^-i)
i=0..무한 합)를 넘으면 못 돈다 — 이번 1차 구현은 [-90,90]도 안에서만 검증하고,
그 밖의 각도(파티클 theta+빔 상대각이 90도를 넘는 경우)는 사분면 접기로 축소
해야 함 — 다음 단계 과제로 명시.

고정소수점: Q1.16(부호 1 + 정수부 1 + 소수부 16, 총 18비트) — x,y,z 레지스터
전부 이 포맷. 정밀도(2^-16 ≈ 0.000015)가 최종 출력 포맷(Q9.8, 2^-8≈0.0039)
보다 훨씬 높아서 반복 중 오차 누적 여유가 넉넉함(출력 시 Q9.8로 다시 양자화).
"""

import math

N = 16          # 반복 횟수
FRAC_W = 16     # Q1.16의 소수부 비트
TOTAL_W = 18    # 부호 1 + 정수부 1 + 소수부 16
SCALE = 1 << FRAC_W


def to_fixed(v):
    """실수 -> Q1.16 18비트 2의 보수 정수(부호 포함, 마스킹된 unsigned 표현)."""
    iv = int(round(v * SCALE))
    assert -(1 << (TOTAL_W - 1)) <= iv < (1 << (TOTAL_W - 1)), 'overflow: %r' % v
    return iv & ((1 << TOTAL_W) - 1)


def compute_x0(n):
    """이 n(반복횟수)에 대한 정확한 CORDIC 게인의 역수(1/K) — 점근값 대신 유한
    반복 게인을 정확히 곱해서 구함(더 정확)."""
    k = 1.0
    for i in range(n):
        k *= math.sqrt(1.0 + 2.0 ** (-2 * i))
    return 1.0 / k


def main():
    # atan(2^-i) 테이블(라디안)
    atans = [math.atan(2.0 ** (-i)) for i in range(N)]
    with open('cordic_atan.hex', 'w') as f:
        for a in atans:
            f.write('%05x\n' % to_fixed(a))
    print('작성: cordic_atan.hex (%d개 항목)' % N)

    x0 = compute_x0(N)
    print('X0(1/K, N=%d) = %.10f -> 0x%05x' % (N, x0, to_fixed(x0)))
    with open('cordic_x0.txt', 'w') as f:
        f.write('%.10f %d\n' % (x0, to_fixed(x0)))

    # 검증용 테스트 벡터 -80~80도(수렴범위 안), Q1.16 정수값 + 파이썬 기준
    # cos/sin(같은 정밀도로 양자화한 것 — 반복 알고리즘 자체의 유한정밀도
    # 오차까지는 여기서 못 잡음, RTL 결과와 비교할 때 몇 LSB 오차는 정상)
    test_deg = [-89, -80, -60, -45, -30, -10, -1, 0, 1, 10, 30, 45, 60, 80, 89]
    with open('cordic_test_theta.hex', 'w') as ft, \
         open('cordic_test_cos.hex', 'w') as fc, \
         open('cordic_test_sin.hex', 'w') as fs, \
         open('cordic_test_deg.txt', 'w') as fd:
        for deg in test_deg:
            rad = math.radians(deg)
            ft.write('%05x\n' % to_fixed(rad))
            fc.write('%05x\n' % to_fixed(math.cos(rad)))
            fs.write('%05x\n' % to_fixed(math.sin(rad)))
            fd.write('%d %.6f %.6f %.6f\n' % (deg, rad, math.cos(rad), math.sin(rad)))
    print('작성: cordic_test_{theta,cos,sin}.hex, cordic_test_deg.txt (%d개 케이스)'
          % len(test_deg))


if __name__ == '__main__':
    main()
