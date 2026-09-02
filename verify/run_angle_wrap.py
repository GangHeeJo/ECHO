#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""angle_wrap.v cocotb 테스트 실행기(빌드+시뮬레이션, Makefile 없이
cocotb 2.0의 파이썬 러너 API로 직접). 이 프로젝트 venv에서:
  ~/zero/.venv/bin/python3 verify/run_angle_wrap.py
"""

import os

from cocotb_tools.runner import get_runner

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_DIR = os.path.join(THIS_DIR, '..', 'rtl')


def main():
    runner = get_runner('icarus')
    runner.build(
        sources=[os.path.join(RTL_DIR, 'angle_wrap.v')],
        hdl_toplevel='angle_wrap',
        build_dir=os.path.join(THIS_DIR, 'sim_build', 'angle_wrap'),
        always=True,
        timescale=('1ns', '1ps'),
    )
    runner.test(
        hdl_toplevel='angle_wrap',
        test_module='test_angle_wrap',
        test_dir=THIS_DIR,
        results_xml='results_angle_wrap.xml',
    )


if __name__ == '__main__':
    main()
