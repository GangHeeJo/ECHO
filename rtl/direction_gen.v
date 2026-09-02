// -*- verilog -*-
// direction_gen.v — 이번 세션의 핵심 결론(현재 particle_scorer 인터페이스는
// Jetson이 dx,dy를 미리 계산해줘야 해서 오히려 손해)을 푸는 조각. 파티클
// theta + beam_id만 받아서 FPGA 내부에서 dx,dy(ray_march_edt가 바로 쓸 수
// 있는 Q9.8)를 만든다 — Jetson은 더 이상 삼각함수를 계산할 필요가 없어짐.
//
// 파이프라인: angle_wrap(조합논리, theta+beam 고정각) -> cordic_sincos_full
// (사분면접기+CORDIC, ~19클럭) -> Q3.16 -> Q9.8 시프트(공짜, 곱셈 없음).
//
// beam_angle_rom의 60개 값은 고정(빔마다 라이다 장착 각도로 항상 같음) —
// gen_direction_gen_test.py가 실제 ZERO 프레임(bench/zero_frame_001.npz)의
// 진짜 각도로 생성함.

module direction_gen #(
    parameter W_EXT       = 19,   // 각도 폭(Q3.16)
    parameter IN_FRAC_W   = 16,
    parameter W_OUT        = 18,   // dx,dy 폭(Q9.8, ray_march_edt와 동일)
    parameter OUT_FRAC_W   = 8,
    parameter NUM_RAYS     = 60,
    parameter BEAM_W       = 6,    // $clog2(60)
    parameter ATAN_FILE       = "cordic_atan.hex",
    parameter BEAM_ANGLE_FILE = "beam_angles.hex"
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     start,
    input  wire signed [W_EXT-1:0]  theta_i,   // 파티클 theta, |theta|<=pi
    input  wire [BEAM_W-1:0]        beam_id,
    output reg                      done,
    output reg  signed [W_OUT-1:0]  dx_o,
    output reg  signed [W_OUT-1:0]  dy_o
);

    reg signed [W_EXT-1:0] beam_angle_rom [0:NUM_RAYS-1];
    initial $readmemh(BEAM_ANGLE_FILE, beam_angle_rom);

    wire signed [W_EXT-1:0] world_angle;
    angle_wrap #(.W_EXT(W_EXT)) u_wrap (
        .theta_a(theta_i), .theta_b(beam_angle_rom[beam_id]), .theta_o(world_angle)
    );

    reg  signed [W_EXT-1:0] wrapped_latched;
    reg                     cordic_start;
    wire                    cordic_done;
    wire signed [W_EXT-1:0] cos_c, sin_c;

    cordic_sincos_full #(.ATAN_FILE(ATAN_FILE)) u_cordic (
        .clk(clk), .rst_n(rst_n), .start(cordic_start),
        .theta_i(wrapped_latched), .done(cordic_done), .cos_o(cos_c), .sin_o(sin_c)
    );

    localparam S_IDLE = 1'd0;
    localparam S_WAIT = 1'd1;
    reg state;

    // Q3.16 -> Q9.8: 소수부 16 -> 8비트로 시프트만 하면 됨(곱셈/재양자화 불필요,
    // 값 자체가 [-1,1] 안이라 Q9.8 범위(±512)에 여유있게 들어감)
    wire signed [W_EXT-1:0] cos_shifted = cos_c >>> (IN_FRAC_W - OUT_FRAC_W);
    wire signed [W_EXT-1:0] sin_shifted = sin_c >>> (IN_FRAC_W - OUT_FRAC_W);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; dx_o <= {W_OUT{1'b0}}; dy_o <= {W_OUT{1'b0}};
            wrapped_latched <= {W_EXT{1'b0}}; cordic_start <= 1'b0;
        end else begin
            done <= 1'b0;
            cordic_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (start) begin
                        wrapped_latched <= world_angle;
                        cordic_start    <= 1'b1;
                        state           <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (cordic_done) begin
                        dx_o  <= cos_shifted[W_OUT-1:0];
                        dy_o  <= sin_shifted[W_OUT-1:0];
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
