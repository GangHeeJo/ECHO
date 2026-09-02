#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""angle_wrap.v 검증용 — 두 각도 합이 [-pi,pi]를 넘나드는 케이스 위주."""

import math

FRAC_W = 16
W_EXT = 19
SCALE = 1 << FRAC_W


def to_fixed(v, w=W_EXT):
    iv = int(round(v * SCALE))
    assert -(1 << (w - 1)) <= iv < (1 << (w - 1)), 'overflow: %r' % v
    return iv & ((1 << w) - 1)


PI_FIX     = to_fixed(math.pi)          # RTL의 PI_SUM 상수와 정확히 같은 정수
TWO_PI_FIX = to_fixed(2 * math.pi, 20)  # RTL의 TWO_PI 상수와 정확히 같은 정수


def unsign(iv_masked, w=W_EXT):
    """to_fixed()가 돌려주는 건 이미 마스킹된(2의 보수 비트패턴의) 양수
    표현이라, 산술에 쓰려면 부호 있는 정수로 되돌려야 함(RTL의 `signed`
    선언과 대응) — 이걸 빠뜨리면 음수 각도를 더할 때 조용히 틀린 값이 나옴."""
    return iv_masked - (1 << w) if iv_masked >= (1 << (w - 1)) else iv_masked


def wrap_fixed(ia, ib):
    """RTL과 완전히 같은 순서: 각도를 먼저 각각 양자화(정수)한 뒤 그 정수합을
    정수 PI_FIX/TWO_PI_FIX 상수로 wrap — 실수로 다시 계산하면(더 '정밀'해
    보여도) RTL이 실제로 보는 반올림 경계와 어긋날 수 있음(양자화 비결합성:
    round(a)+round(b) != round(a+b))."""
    s = unsign(ia) + unsign(ib)
    if s > PI_FIX:
        s -= TWO_PI_FIX
    elif s < -PI_FIX:
        s += TWO_PI_FIX
    return s & ((1 << W_EXT) - 1)


def main():
    print('TWO_PI(W_SUM=20) = %.10f -> 0x%06x' % (2 * math.pi, TWO_PI_FIX))
    print('PI(Q3.16) = 0x%06x' % PI_FIX)

    cases_deg = [
        (0, 0), (45, -30), (100, 100), (-100, -100), (170, 20), (-170, -20),
        (90, 90), (-90, -90), (179, 179), (-179, -179), (179, 1), (-179, -1),
        (135, 45), (-135, -45), (150, -150), (0, 179), (0, -179),
    ]
    with open('angle_wrap_test_a.hex', 'w') as fa, \
         open('angle_wrap_test_b.hex', 'w') as fb, \
         open('angle_wrap_test_exp.hex', 'w') as fe, \
         open('angle_wrap_test_deg.txt', 'w') as fd:
        for da, db in cases_deg:
            ra, rb = math.radians(da), math.radians(db)
            ia, ib = to_fixed(ra), to_fixed(rb)
            exp_fixed = wrap_fixed(ia, ib)
            fa.write('%05x\n' % ia)
            fb.write('%05x\n' % ib)
            fe.write('%05x\n' % exp_fixed)
            fd.write('%d %d -> raw=%d (%.2fdeg)\n' % (da, db, exp_fixed, math.degrees(ra + rb)))
    print('작성: angle_wrap_test_{a,b,exp}.hex, angle_wrap_test_deg.txt (%d개)' % len(cases_deg))


if __name__ == '__main__':
    main()
