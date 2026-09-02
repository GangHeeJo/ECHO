// -*- verilog -*-
// angle_wrap.v — 두 각도(파티클 theta + 빔의 고정 상대각)를 더하고, 합이
// [-pi,pi] 범위를 벗어나면(둘 다 [-pi,pi] 안이라 최대 한 번의 2pi 보정으로
// 항상 충분함 — 모듈로 연산 아니라 조건부 덧셈/뺄셈 한 번) 되돌린다.
// addr_gen.v와 같은 결(순수 조합논리, 곱셈 없음) — cordic_sincos_full.v가
// 요구하는 "|theta|<=pi" 전제조건을 만족시켜주는 전처리 단계.

module angle_wrap #(
    parameter W_EXT = 19,   // theta_a, theta_b, 결과: Q3.16 (범위 [-4,4))
    parameter W_SUM = 20    // 합 계산용(범위 [-8,8), 오버플로 방지)
) (
    input  wire signed [W_EXT-1:0] theta_a,   // |theta_a| <= pi
    input  wire signed [W_EXT-1:0] theta_b,   // |theta_b| <= pi
    output wire signed [W_EXT-1:0] theta_o    // 항상 |theta_o| <= pi
);

    localparam signed [W_SUM-1:0] TWO_PI  = 20'sd411775;   // gen_angle_wrap_table.py
    localparam signed [W_SUM-1:0] PI_SUM  = 20'sd205887;   // Q3.16 그대로, W_SUM으로 부호확장

    wire signed [W_SUM-1:0] a_ext = {{(W_SUM-W_EXT){theta_a[W_EXT-1]}}, theta_a};
    wire signed [W_SUM-1:0] b_ext = {{(W_SUM-W_EXT){theta_b[W_EXT-1]}}, theta_b};
    wire signed [W_SUM-1:0] sum   = a_ext + b_ext;

    wire signed [W_SUM-1:0] wrapped = (sum > PI_SUM)  ? (sum - TWO_PI) :
                                       (sum < -PI_SUM) ? (sum + TWO_PI) :
                                                          sum;

    assign theta_o = wrapped[W_EXT-1:0];

endmodule
