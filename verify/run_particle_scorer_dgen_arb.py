#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""particle_scorer_dgen_arb.v cocotb 회귀 테스트 실행기.
  ~/zero/.venv/bin/python3 verify/run_particle_scorer_dgen_arb.py
"""

import os

from cocotb_tools.runner import get_runner

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_DIR = os.path.join(THIS_DIR, '..', 'rtl')

SOURCES = [
    os.path.join(RTL_DIR, f) for f in [
        'ray_march_edt.v', 'addr_gen.v', 'table_mem.v', 'sensor_pe.v',
        'cordic_sincos.v', 'cordic_sincos_full.v', 'angle_wrap.v', 'theta_wrap.v',
        'direction_gen.v', 'particle_scorer_dgen_arb.v',
    ]
] + [os.path.join(THIS_DIR, 'tb_wrapper_dgen_arb.v')]


def main():
    runner = get_runner('icarus')
    runner.build(
        sources=SOURCES,
        hdl_toplevel='tb_wrapper_dgen_arb',
        build_dir=os.path.join(THIS_DIR, 'sim_build', 'particle_scorer_dgen_arb'),
        always=True,
        timescale=('1ns', '1ps'),
    )
    runner.test(
        hdl_toplevel='tb_wrapper_dgen_arb',
        test_module='test_particle_scorer_dgen_arb',
        test_dir=THIS_DIR,
        results_xml='results_particle_scorer_dgen_arb.xml',
    )


if __name__ == '__main__':
    main()
