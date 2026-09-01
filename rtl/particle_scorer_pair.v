// -*- verilog -*-
// particle_scorer_pair.v — particle_scorer_shared 2개가 table_mem_dp(듀얼포트
// BRAM) 하나를 나눠 쓰는 최상위. 목적: "테이블을 공유하면 BRAM 78%짜리 파티클
// 하나가 쓰던 예산 그대로 2개를 돌릴 수 있는가"를 실제 합성으로 확인.

module particle_scorer_pair #(
    parameter RM_POS_W = 18,
    parameter RD_W     = 9,
    parameter ADDR_W   = 17,
    parameter DATA_W   = 13,
    parameter ACC_W    = 20
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        particle_start0,
    input  wire signed [RM_POS_W-1:0]  x0_0, y0_0,
    input  wire                        beam_start0,
    input  wire signed [RM_POS_W-1:0]  dx0, dy0,
    input  wire [RD_W-1:0]             r_obs0,
    input  wire                        beam_last0,
    output wire                        beam_done0,
    output wire                        particle_done0,
    output wire signed [ACC_W-1:0]     weight_o0,

    input  wire                        particle_start1,
    input  wire signed [RM_POS_W-1:0]  x0_1, y0_1,
    input  wire                        beam_start1,
    input  wire signed [RM_POS_W-1:0]  dx1, dy1,
    input  wire [RD_W-1:0]             r_obs1,
    input  wire                        beam_last1,
    output wire                        beam_done1,
    output wire                        particle_done1,
    output wire signed [ACC_W-1:0]     weight_o1
);

    wire [ADDR_W-1:0]        table_addr0, table_addr1;
    wire signed [DATA_W-1:0] table_data0, table_data1;

    table_mem_dp #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table_shared (
        .clk(clk),
        .addr_a(table_addr0), .data_a(table_data0),
        .addr_b(table_addr1), .data_b(table_data1)
    );

    particle_scorer_shared #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                              .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer0 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start0), .x0(x0_0), .y0(y0_0),
        .beam_start(beam_start0), .dx(dx0), .dy(dy0), .r_obs(r_obs0), .beam_last(beam_last0),
        .beam_done(beam_done0), .particle_done(particle_done0), .weight_o(weight_o0),
        .table_addr(table_addr0), .table_data(table_data0)
    );

    particle_scorer_shared #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                              .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer1 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start1), .x0(x0_1), .y0(y0_1),
        .beam_start(beam_start1), .dx(dx1), .dy(dy1), .r_obs(r_obs1), .beam_last(beam_last1),
        .beam_done(beam_done1), .particle_done(particle_done1), .weight_o(weight_o1),
        .table_addr(table_addr1), .table_data(table_data1)
    );

endmodule
