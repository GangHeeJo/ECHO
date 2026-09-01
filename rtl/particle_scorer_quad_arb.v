// -*- verilog -*-
// particle_scorer_quad_arb.v — particle_scorer_arb 4개가 진짜 single-port
// table_mem 하나를 arbiter4(전국AI반도체경진대회 프로젝트 재사용, req/gnt 4비트
// round-robin)로 매 클럭 중재해서 나눠 쓴다. particle_scorer_pair_arb.v(2개판)의
// 홀드 로직(승인 후 1클럭 더 같은 주소 유지 — sensor_pe의 pend_valid 릴레이
// 때문에 필요했음)을 2비트 인덱스로 그대로 일반화한 것.

module particle_scorer_quad_arb #(
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
    output wire signed [ACC_W-1:0]     weight_o1,

    input  wire                        particle_start2,
    input  wire signed [RM_POS_W-1:0]  x0_2, y0_2,
    input  wire                        beam_start2,
    input  wire signed [RM_POS_W-1:0]  dx2, dy2,
    input  wire [RD_W-1:0]             r_obs2,
    input  wire                        beam_last2,
    output wire                        beam_done2,
    output wire                        particle_done2,
    output wire signed [ACC_W-1:0]     weight_o2,

    input  wire                        particle_start3,
    input  wire signed [RM_POS_W-1:0]  x0_3, y0_3,
    input  wire                        beam_start3,
    input  wire signed [RM_POS_W-1:0]  dx3, dy3,
    input  wire [RD_W-1:0]             r_obs3,
    input  wire                        beam_last3,
    output wire                        beam_done3,
    output wire                        particle_done3,
    output wire signed [ACC_W-1:0]     weight_o3
);

    wire [ADDR_W-1:0]        table_addr0, table_addr1, table_addr2, table_addr3;
    wire signed [DATA_W-1:0] table_data;   // 진짜 물리 테이블 출력, 4개 다 같은 걸 봄
    wire                      req0, req1, req2, req3;
    wire [3:0]                gnt;

    // particle_scorer_pair_arb.v와 동일한 이유(sensor_pe의 pend_valid 릴레이
    // 때문에 승인 후 1클럭 더 같은 주소를 유지해야 함)로 홀드 로직을 그대로
    // 2비트 인덱스로 일반화.
    reg        busy;
    reg [1:0]  held_sel;
    wire [3:0] eff_req = busy ? 4'b0000 : {req3, req2, req1, req0};

    arbiter4 u_arb (
        .clk(clk), .rst(~rst_n),
        .req(eff_req),
        .gnt(gnt)
    );

    wire [1:0] gnt_idx = gnt[0] ? 2'd0 : gnt[1] ? 2'd1 : gnt[2] ? 2'd2 : 2'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; held_sel <= 2'd0;
        end else if (|gnt) begin
            busy <= 1'b1; held_sel <= gnt_idx;
        end else if (busy) begin
            busy <= 1'b0;
        end
    end

    wire [1:0] sel_now = busy ? held_sel : gnt_idx;
    wire [ADDR_W-1:0] table_addr_shared =
        (sel_now == 2'd0) ? table_addr0 :
        (sel_now == 2'd1) ? table_addr1 :
        (sel_now == 2'd2) ? table_addr2 : table_addr3;

    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table_shared (
        .clk(clk), .addr(table_addr_shared), .data(table_data)
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer0 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start0), .x0(x0_0), .y0(y0_0),
        .beam_start(beam_start0), .dx(dx0), .dy(dy0), .r_obs(r_obs0), .beam_last(beam_last0),
        .beam_done(beam_done0), .particle_done(particle_done0), .weight_o(weight_o0),
        .table_addr(table_addr0), .table_data(table_data), .req(req0), .gnt(gnt[0])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer1 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start1), .x0(x0_1), .y0(y0_1),
        .beam_start(beam_start1), .dx(dx1), .dy(dy1), .r_obs(r_obs1), .beam_last(beam_last1),
        .beam_done(beam_done1), .particle_done(particle_done1), .weight_o(weight_o1),
        .table_addr(table_addr1), .table_data(table_data), .req(req1), .gnt(gnt[1])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer2 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start2), .x0(x0_2), .y0(y0_2),
        .beam_start(beam_start2), .dx(dx2), .dy(dy2), .r_obs(r_obs2), .beam_last(beam_last2),
        .beam_done(beam_done2), .particle_done(particle_done2), .weight_o(weight_o2),
        .table_addr(table_addr2), .table_data(table_data), .req(req2), .gnt(gnt[2])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer3 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start3), .x0(x0_3), .y0(y0_3),
        .beam_start(beam_start3), .dx(dx3), .dy(dy3), .r_obs(r_obs3), .beam_last(beam_last3),
        .beam_done(beam_done3), .particle_done(particle_done3), .weight_o(weight_o3),
        .table_addr(table_addr3), .table_data(table_data), .req(req3), .gnt(gnt[3])
    );

endmodule
