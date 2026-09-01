// -*- verilog -*-
// particle_scorer_oct_arb.v — particle_scorer_arb 8개가 진짜 single-port
// table_mem 하나를 arbiter8(전국AI반도체경진대회 프로젝트 재사용, req/gnt
// 8비트 round-robin)로 매 클럭 중재해서 나눠 쓴다. particle_scorer_quad_arb.v
// (4개판)의 홀드 로직(승인 후 1클럭 더 같은 주소 유지)을 3비트 인덱스로 그대로
// 일반화한 것 — 구조는 pair/quad와 완전히 동일, 폭만 늘어남.

module particle_scorer_oct_arb #(
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
    output wire signed [ACC_W-1:0]     weight_o3,

    input  wire                        particle_start4,
    input  wire signed [RM_POS_W-1:0]  x0_4, y0_4,
    input  wire                        beam_start4,
    input  wire signed [RM_POS_W-1:0]  dx4, dy4,
    input  wire [RD_W-1:0]             r_obs4,
    input  wire                        beam_last4,
    output wire                        beam_done4,
    output wire                        particle_done4,
    output wire signed [ACC_W-1:0]     weight_o4,

    input  wire                        particle_start5,
    input  wire signed [RM_POS_W-1:0]  x0_5, y0_5,
    input  wire                        beam_start5,
    input  wire signed [RM_POS_W-1:0]  dx5, dy5,
    input  wire [RD_W-1:0]             r_obs5,
    input  wire                        beam_last5,
    output wire                        beam_done5,
    output wire                        particle_done5,
    output wire signed [ACC_W-1:0]     weight_o5,

    input  wire                        particle_start6,
    input  wire signed [RM_POS_W-1:0]  x0_6, y0_6,
    input  wire                        beam_start6,
    input  wire signed [RM_POS_W-1:0]  dx6, dy6,
    input  wire [RD_W-1:0]             r_obs6,
    input  wire                        beam_last6,
    output wire                        beam_done6,
    output wire                        particle_done6,
    output wire signed [ACC_W-1:0]     weight_o6,

    input  wire                        particle_start7,
    input  wire signed [RM_POS_W-1:0]  x0_7, y0_7,
    input  wire                        beam_start7,
    input  wire signed [RM_POS_W-1:0]  dx7, dy7,
    input  wire [RD_W-1:0]             r_obs7,
    input  wire                        beam_last7,
    output wire                        beam_done7,
    output wire                        particle_done7,
    output wire signed [ACC_W-1:0]     weight_o7
);

    wire [ADDR_W-1:0] table_addr [0:7];
    wire signed [DATA_W-1:0] table_data;   // 진짜 물리 테이블 출력, 8개 다 같은 걸 봄
    wire [7:0] req;
    wire [7:0] gnt;

    // pair/quad와 동일한 이유(sensor_pe의 pend_valid 릴레이)로 홀드 로직 필요 —
    // 3비트 인덱스로 그대로 일반화.
    reg        busy;
    reg [2:0]  held_sel;
    wire [7:0] eff_req = busy ? 8'b0 : req;

    arbiter8 u_arb (
        .clk(clk), .rst(~rst_n),
        .req(eff_req),
        .gnt(gnt)
    );

    function [2:0] gnt_idx8;
        input [7:0] bits;
        begin
            if (bits[0]) gnt_idx8 = 3'd0;
            else if (bits[1]) gnt_idx8 = 3'd1;
            else if (bits[2]) gnt_idx8 = 3'd2;
            else if (bits[3]) gnt_idx8 = 3'd3;
            else if (bits[4]) gnt_idx8 = 3'd4;
            else if (bits[5]) gnt_idx8 = 3'd5;
            else if (bits[6]) gnt_idx8 = 3'd6;
            else gnt_idx8 = 3'd7;
        end
    endfunction

    wire [2:0] gnt_idx = gnt_idx8(gnt);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; held_sel <= 3'd0;
        end else if (|gnt) begin
            busy <= 1'b1; held_sel <= gnt_idx;
        end else if (busy) begin
            busy <= 1'b0;
        end
    end

    wire [2:0] sel_now = busy ? held_sel : gnt_idx;
    wire [ADDR_W-1:0] table_addr_shared = table_addr[sel_now];

    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table_shared (
        .clk(clk), .addr(table_addr_shared), .data(table_data)
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer0 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start0), .x0(x0_0), .y0(y0_0),
        .beam_start(beam_start0), .dx(dx0), .dy(dy0), .r_obs(r_obs0), .beam_last(beam_last0),
        .beam_done(beam_done0), .particle_done(particle_done0), .weight_o(weight_o0),
        .table_addr(table_addr[0]), .table_data(table_data), .req(req[0]), .gnt(gnt[0])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer1 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start1), .x0(x0_1), .y0(y0_1),
        .beam_start(beam_start1), .dx(dx1), .dy(dy1), .r_obs(r_obs1), .beam_last(beam_last1),
        .beam_done(beam_done1), .particle_done(particle_done1), .weight_o(weight_o1),
        .table_addr(table_addr[1]), .table_data(table_data), .req(req[1]), .gnt(gnt[1])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer2 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start2), .x0(x0_2), .y0(y0_2),
        .beam_start(beam_start2), .dx(dx2), .dy(dy2), .r_obs(r_obs2), .beam_last(beam_last2),
        .beam_done(beam_done2), .particle_done(particle_done2), .weight_o(weight_o2),
        .table_addr(table_addr[2]), .table_data(table_data), .req(req[2]), .gnt(gnt[2])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer3 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start3), .x0(x0_3), .y0(y0_3),
        .beam_start(beam_start3), .dx(dx3), .dy(dy3), .r_obs(r_obs3), .beam_last(beam_last3),
        .beam_done(beam_done3), .particle_done(particle_done3), .weight_o(weight_o3),
        .table_addr(table_addr[3]), .table_data(table_data), .req(req[3]), .gnt(gnt[3])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer4 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start4), .x0(x0_4), .y0(y0_4),
        .beam_start(beam_start4), .dx(dx4), .dy(dy4), .r_obs(r_obs4), .beam_last(beam_last4),
        .beam_done(beam_done4), .particle_done(particle_done4), .weight_o(weight_o4),
        .table_addr(table_addr[4]), .table_data(table_data), .req(req[4]), .gnt(gnt[4])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer5 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start5), .x0(x0_5), .y0(y0_5),
        .beam_start(beam_start5), .dx(dx5), .dy(dy5), .r_obs(r_obs5), .beam_last(beam_last5),
        .beam_done(beam_done5), .particle_done(particle_done5), .weight_o(weight_o5),
        .table_addr(table_addr[5]), .table_data(table_data), .req(req[5]), .gnt(gnt[5])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer6 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start6), .x0(x0_6), .y0(y0_6),
        .beam_start(beam_start6), .dx(dx6), .dy(dy6), .r_obs(r_obs6), .beam_last(beam_last6),
        .beam_done(beam_done6), .particle_done(particle_done6), .weight_o(weight_o6),
        .table_addr(table_addr[6]), .table_data(table_data), .req(req[6]), .gnt(gnt[6])
    );

    particle_scorer_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ADDR_W(ADDR_W),
                           .DATA_W(DATA_W), .ACC_W(ACC_W)) u_scorer7 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start7), .x0(x0_7), .y0(y0_7),
        .beam_start(beam_start7), .dx(dx7), .dy(dy7), .r_obs(r_obs7), .beam_last(beam_last7),
        .beam_done(beam_done7), .particle_done(particle_done7), .weight_o(weight_o7),
        .table_addr(table_addr[7]), .table_data(table_data), .req(req[7]), .gnt(gnt[7])
    );

endmodule
