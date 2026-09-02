// -*- verilog -*-
// theta_wrap.v — 파티클 theta는 실제 ZERO(motion_model)에서 wrap 없이
// 계속 누적된다(실측: 4.57rad, 5.71rad, 7.41rad 같은 값이 그냥 나옴) —
// angle_wrap.v(두 [-pi,pi] 값의 합, 최대 1회 보정)와 달리 theta_wrap은
// "얼마나 벗어났는지 모르는" 값을 받아 범위 안에 들어올 때까지 반복
// 보정한다(최대 MAX_ITER회, 안전판). 실측으로는 1회면 항상 충분했지만
// (gen_theta_wrap_table.py 상단 주석) 여유있게 8회까지 대응.
//
// 입력 폭이 angle_wrap/cordic보다 넓음(W_RAW, 정수부 여유 더 큼) — 다 감고
// 나면 표준 Q3.16(W_EXT=19, cordic_sincos_full.v/angle_wrap.v와 동일
// 포맷)으로 잘라서 내보낸다.

module theta_wrap #(
    parameter W_RAW  = 24,   // 입력: 부호1+정수부7(약 ±64 커버)+소수부16
    parameter W_EXT  = 19,   // 출력: Q3.16(항상 |theta|<=pi)
    parameter MAX_ITER = 8
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [W_RAW-1:0] theta_raw,
    output reg                     done,
    output reg  signed [W_EXT-1:0] theta_o
);

    localparam signed [W_RAW-1:0] PI_FIX     = 24'sd205887;
    localparam signed [W_RAW-1:0] TWO_PI_FIX = 24'sd411775;
    localparam ITER_W = $clog2(MAX_ITER+1);

    localparam S_IDLE = 1'd0;
    localparam S_ITER = 1'd1;

    reg state;
    reg signed [W_RAW-1:0] acc;
    reg [ITER_W-1:0] iter;

    wire acc_hi = (acc > PI_FIX);
    wire acc_lo = (acc < -PI_FIX);
    wire in_range = !acc_hi && !acc_lo;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; theta_o <= {W_EXT{1'b0}};
            acc <= {W_RAW{1'b0}}; iter <= {ITER_W{1'b0}};
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        acc  <= theta_raw;
                        iter <= {ITER_W{1'b0}};
                        state <= S_ITER;
                    end
                end
                S_ITER: begin
                    if (in_range || iter == MAX_ITER) begin
                        theta_o <= acc[W_EXT-1:0];  // 범위 안이면 W_RAW 상위비트=부호확장분, 그냥 잘라도 안전
                        done    <= 1'b1;
                        state   <= S_IDLE;
                    end else if (acc_hi) begin
                        acc  <= acc - TWO_PI_FIX;
                        iter <= iter + 1'b1;
                    end else begin
                        acc  <= acc + TWO_PI_FIX;
                        iter <= iter + 1'b1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
