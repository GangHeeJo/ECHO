#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ray_march_edt의 범용 배럴시프터(dx/dy <<< cur_shift)를 대체할 ROM 생성.

gen_particle_scorer_test.py가 쓰는 것과 똑같은 8개 고정 빔 각도(-60~60도, 8등분)
에 대해, 8가지 shift(0~7)별로 미리 "step_dx = dx <<< shift" 결과를 계산해
ROM(64개 항목)으로 굽는다. 주소 = {beam_id(3b), shift(3b)}.

⚠️ 스코프 한정: 이 ROM은 "빔 각도가 8개로 고정"이라는 이번 벤치마크의 전제에서만
성립한다. 실제 ZERO는 파티클 방향(theta, 연속값)에 따라 빔의 world-frame 각도가
매 파티클마다 달라지므로(beam_relative_angle + particle_theta), beam_id만으로
ROM을 인덱싱하는 이 방식은 그대로 실전에 못 들어간다 — theta까지 양자화해
주소에 포함시켜야 하는데(예: 256~4096단계), 그러면 ROM 깊이가 64 -> 16000+로
커져서 이번에 아끼려는 만큼을 다시 까먹을 수 있음. 이번 실험은 "고정 벤치마크
안에서 배럴시프터 vs ROM 크리티컬패스/자원 비교"로 범위를 좁힌 A/B다.

dx <<< shift는 18비트 신호 폭(POS_W)에서 잘림(truncate)되므로, 파이썬에서도
2의 보수 18비트로 마스킹해 정확히 같은 비트패턴을 재현한다(부동소수점으로
다시 계산하면 반올림이 하드웨어와 어긋날 수 있음 — 반드시 정수 시프트로).
"""

import math

import numpy as np

FRAC_W = 8          # gen_track_map.py의 Q9.8과 동일
POS_W = 18
NUM_RAYS = 8
NUM_SHIFTS = 8       # SHIFT_W=3 -> 0~7


def quantize_dx_dy(scale):
    """gen_particle_scorer_test.py의 quantize()/w_signed()와 동일한 양자화."""
    angles_deg = np.linspace(-60, 60, NUM_RAYS)
    out = []
    for ang in angles_deg:
        rad = math.radians(ang)
        dx = int(round(math.cos(rad) * scale))
        dy = int(round(math.sin(rad) * scale))
        out.append((dx, dy))
    return out


def shift_trunc18(v_signed, shift):
    """Verilog `dx <<< shift`가 18비트 신호에서 잘리는 것과 비트단위로 동일."""
    return (v_signed << shift) & ((1 << POS_W) - 1)


def main():
    scale = 1 << FRAC_W
    beams = quantize_dx_dy(scale)

    dx_rom = [0] * (NUM_RAYS * NUM_SHIFTS)
    dy_rom = [0] * (NUM_RAYS * NUM_SHIFTS)
    for beam_id, (dx, dy) in enumerate(beams):
        for shift in range(NUM_SHIFTS):
            addr = (beam_id << 3) | shift
            dx_rom[addr] = shift_trunc18(dx, shift)
            dy_rom[addr] = shift_trunc18(dy, shift)

    with open('beam_step_dx.hex', 'w') as f:
        for v in dx_rom:
            f.write('%05x\n' % v)
    with open('beam_step_dy.hex', 'w') as f:
        for v in dy_rom:
            f.write('%05x\n' % v)

    print('작성: beam_step_dx.hex, beam_step_dy.hex (%d개 항목, addr={beam_id,shift})'
          % (NUM_RAYS * NUM_SHIFTS))
    for beam_id, (dx, dy) in enumerate(beams):
        print('  beam %d: dx=%d dy=%d (raw Q9.8)' % (beam_id, dx, dy))


if __name__ == '__main__':
    main()
