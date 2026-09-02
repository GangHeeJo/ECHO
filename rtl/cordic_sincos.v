// -*- verilog -*-
// cordic_sincos.v — CORDIC(회전 모드)으로 sin/cos를 곱셈기 없이 계산.
// direction generator(파티클 theta -> dx,dy)의 핵심 조각. 매 반복마다
// "부호에 따라 시프트한 값을 더하거나 뺀다"만 반복 — 이 프로젝트 전체의
// 설계 철학(로그테이블+덧셈 센서모델, 시프트-덧셈 주소생성, 배럴시프터
// 레이마칭)과 같은 결.
//
// ⚠️ 수렴 범위 한정: 순환 CORDIC은 |theta| <= 약 99.7도(arctan(2^-i) i=0..N-1
// 합)까지만 정확히 회전한다. 이번 1차 구현은 [-90,90]도 안에서만 검증됨 —
// 그 밖 각도(예: 파티클 뒤쪽을 향하는 빔)는 사분면 접기로 축소해야 하는데
// 아직 이 모듈엔 없음(다음 단계). tools/gen_cordic_table.py 참고.

module cordic_sincos #(
    parameter N       = 16,     // 반복 횟수
    parameter W       = 18,     // 전체 비트폭(부호 1 + 정수부 1 + 소수부 16)
    parameter ITER_W  = 4,      // $clog2(N)
    parameter ATAN_FILE = "cordic_atan.hex"
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,
    input  wire signed [W-1:0]   theta_i,   // Q1.16 라디안, [-90,90]도 안이어야 함
    output reg                   done,
    output reg  signed [W-1:0]   cos_o,
    output reg  signed [W-1:0]   sin_o
);

    // X0 = 1/K(N=16 유한반복 게인의 역수) — gen_cordic_table.py의 compute_x0(16)
    // 값과 정확히 일치해야 함(0.6072529351 -> Q1.16).
    localparam signed [W-1:0] X0 = 18'sd39797;

    localparam S_IDLE = 2'd0;
    localparam S_RUN  = 2'd1;
    localparam S_DONE = 2'd2;

    reg [1:0] state;
    reg signed [W-1:0] x_reg, y_reg, z_reg;
    reg [ITER_W-1:0] iter;

    reg signed [W-1:0] atan_rom [0:N-1];
    initial $readmemh(ATAN_FILE, atan_rom);

    wire signed [W-1:0] x_shift = x_reg >>> iter;
    wire signed [W-1:0] y_shift = y_reg >>> iter;
    wire                d_neg   = z_reg[W-1];   // z<0 ?

    wire signed [W-1:0] x_next = d_neg ? (x_reg + y_shift) : (x_reg - y_shift);
    wire signed [W-1:0] y_next = d_neg ? (y_reg - x_shift) : (y_reg + x_shift);
    wire signed [W-1:0] z_next = d_neg ? (z_reg + atan_rom[iter]) : (z_reg - atan_rom[iter]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; cos_o <= {W{1'b0}}; sin_o <= {W{1'b0}};
            x_reg <= {W{1'b0}}; y_reg <= {W{1'b0}}; z_reg <= {W{1'b0}}; iter <= {ITER_W{1'b0}};
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        x_reg <= X0; y_reg <= {W{1'b0}}; z_reg <= theta_i;
                        iter  <= {ITER_W{1'b0}};
                        state <= S_RUN;
                    end
                end
                S_RUN: begin
                    x_reg <= x_next; y_reg <= y_next; z_reg <= z_next;
                    if (iter == N-1) begin
                        state <= S_DONE;
                    end else begin
                        iter <= iter + 1'b1;
                    end
                end
                S_DONE: begin
                    cos_o <= x_reg; sin_o <= y_reg; done <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
