// -*- verilog -*-
// cordic_sincos_full.v — cordic_sincos.v(코어, [-90,90]도만 수렴)를 사분면
// 접기(quadrant folding)로 감싸서 [-180,180]도 전체를 지원하는 래퍼.
//
// 원리: cos는 짝함수, sin은 홀함수라는 대칭성만 이용 — 곱셈기 전혀 안 씀.
//   1. abs_theta = |theta|, sign_sin = (theta<0)
//   2. abs_theta가 90도(pi/2)를 넘으면 reduced = 180도(pi) - abs_theta로
//      한 번 더 접고, sign_cos = 음수로 표시(cos(180-x) = -cos(x)이므로)
//   3. reduced(항상 [0,90]도 안)를 코어 cordic_sincos에 넣음
//   4. 나온 cos/sin에 sign_cos/sign_sin 부호만 복원
//
// ⚠️ 전제: |theta_i| <= pi(180도)여야 함 — 그보다 큰 각(여러 바퀴 돈 상태)은
// 호출자가 미리 mod 2pi로 wrap해서 넣어야 함(이 모듈엔 wrap 로직 없음).
//
// 고정소수점: 코어(Q1.16, W_CORE=18)보다 정수부가 2비트 더 넓은 W_EXT=19
// (pi≈3.14가 코어 표현범위 [-2,2)를 넘어서라서) — 소수부(16비트)는 코어와
// 동일해서 reduced 값(항상 [0,pi/2] 안)은 하위 W_CORE비트만 그대로 코어에
// 넘기면 됨(재양자화 불필요).

module cordic_sincos_full #(
    parameter N       = 16,
    parameter W_CORE  = 18,
    parameter W_EXT   = 19,
    parameter ATAN_FILE = "cordic_atan.hex"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [W_EXT-1:0] theta_i,   // |theta_i| <= pi(Q3.16)
    output reg                     done,
    output reg  signed [W_EXT-1:0] cos_o,
    output reg  signed [W_EXT-1:0] sin_o
);

    localparam signed [W_EXT-1:0] PI_CONST      = 19'sd205887;  // gen_cordic_full_table.py
    localparam signed [W_EXT-1:0] PI_HALF_CONST = 19'sd102944;

    localparam S_IDLE      = 2'd0;
    localparam S_WAIT_CORE = 2'd1;

    reg [1:0] state;
    reg sign_cos, sign_sin;
    reg core_start;
    reg signed [W_CORE-1:0] reduced;

    wire core_done;
    wire signed [W_CORE-1:0] core_cos, core_sin;

    cordic_sincos #(.N(N), .W(W_CORE), .ATAN_FILE(ATAN_FILE)) u_core (
        .clk(clk), .rst_n(rst_n), .start(core_start),
        .theta_i(reduced), .done(core_done), .cos_o(core_cos), .sin_o(core_sin)
    );

    wire signed [W_EXT-1:0] abs_theta   = theta_i[W_EXT-1] ? -theta_i : theta_i;
    wire                    fold        = (abs_theta > PI_HALF_CONST);
    wire signed [W_EXT-1:0] reduced_full = fold ? (PI_CONST - abs_theta) : abs_theta;

    wire signed [W_EXT-1:0] core_cos_ext = {core_cos[W_CORE-1], core_cos};
    wire signed [W_EXT-1:0] core_sin_ext = {core_sin[W_CORE-1], core_sin};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; cos_o <= {W_EXT{1'b0}}; sin_o <= {W_EXT{1'b0}};
            sign_cos <= 1'b0; sign_sin <= 1'b0; core_start <= 1'b0; reduced <= {W_CORE{1'b0}};
        end else begin
            done <= 1'b0;
            core_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        reduced    <= reduced_full[W_CORE-1:0];
                        sign_cos   <= fold;
                        sign_sin   <= theta_i[W_EXT-1];
                        core_start <= 1'b1;
                        state      <= S_WAIT_CORE;
                    end
                end
                S_WAIT_CORE: begin
                    if (core_done) begin
                        cos_o <= sign_cos ? -core_cos_ext : core_cos_ext;
                        sin_o <= sign_sin ? -core_sin_ext : core_sin_ext;
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
