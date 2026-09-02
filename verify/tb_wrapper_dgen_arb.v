// -*- verilog -*-
// tb_wrapper_dgen_arb.v — cocotb 테스트 전용 하네스. particle_scorer_dgen_arb
// 는 원래 오직 오케스트레이터(particle_scorer_dgen_oct_arb)만 이 포트들을
// 직접 다루도록 설계됐음(req/gnt/table_addr/table_data가 arbiter+공유
// table_mem을 향함) — 단독으로 테스트하려면 table_mem을 붙이고 gnt를
// 항상 1로(단일 스코어러, 진짜 중재 없음) 고정해야 한다.

module tb_wrapper_dgen_arb (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         particle_start,
    input  wire signed [17:0] x0, y0,
    input  wire signed [23:0] theta_raw,
    input  wire         beam_start,
    input  wire [8:0]   r_obs,
    output wire         beam_done,
    output wire         particle_done,
    output wire signed [19:0] weight_o
);

    wire [16:0] table_addr;
    wire signed [12:0] table_data;
    wire req;

    table_mem u_table (
        .clk(clk), .addr(table_addr), .data(table_data)
    );

    particle_scorer_dgen_arb u_dut (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start), .x0(x0), .y0(y0), .theta_raw(theta_raw),
        .beam_start(beam_start), .r_obs(r_obs),
        .beam_done(beam_done), .particle_done(particle_done), .weight_o(weight_o),
        .table_addr(table_addr), .table_data(table_data),
        .req(req), .gnt(1'b1)   // 단일 스코어러 — 항상 승인
    );

endmodule
