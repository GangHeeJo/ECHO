#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cordic_sincos_full.v(사분면 접기 래퍼) 검증용 — 전체 범위(-179~179도) 테스트
벡터. 래퍼는 |theta|<=pi 입력을 받아 내부적으로 [0,pi/2]로 접어서 cordic_sincos
(코어, [-90,90]도만 지원)에 넣고 부호를 복원한다.

래퍼 쪽 고정소수점은 코어보다 정수부가 2비트 더 넓다(pi≈3.14가 코어 포맷
Q1.16의 표현범위 [-2,2)를 넘어서라서) — W_EXT=19, FRAC_W=16(코어와 소수부는
동일, 정수부만 1->3비트로 늘림, 표현범위 [-4,4)).
"""

import math

FRAC_W = 16
W_EXT = 19
SCALE = 1 << FRAC_W


def to_fixed_ext(v):
    iv = int(round(v * SCALE))
    assert -(1 << (W_EXT - 1)) <= iv < (1 << (W_EXT - 1)), 'overflow: %r' % v
    return iv & ((1 << W_EXT) - 1)


def main():
    print('PI_CONST      = %.10f -> 0x%06x' % (math.pi, to_fixed_ext(math.pi)))
    print('PI_HALF_CONST = %.10f -> 0x%06x' % (math.pi / 2, to_fixed_ext(math.pi / 2)))

    test_deg = [-179, -170, -135, -120, -91, -90, -89, -45, -1, 0,
                1, 45, 89, 90, 91, 120, 135, 170, 179]
    with open('cordic_full_test_theta.hex', 'w') as ft, \
         open('cordic_full_test_cos.hex', 'w') as fc, \
         open('cordic_full_test_sin.hex', 'w') as fs, \
         open('cordic_full_test_deg.txt', 'w') as fd:
        for deg in test_deg:
            rad = math.radians(deg)
            ft.write('%06x\n' % to_fixed_ext(rad))
            fc.write('%06x\n' % to_fixed_ext(math.cos(rad)))
            fs.write('%06x\n' % to_fixed_ext(math.sin(rad)))
            fd.write('%d %.6f %.6f %.6f\n' % (deg, rad, math.cos(rad), math.sin(rad)))
    print('작성: cordic_full_test_{theta,cos,sin}.hex, cordic_full_test_deg.txt (%d개)'
          % len(test_deg))


if __name__ == '__main__':
    main()
