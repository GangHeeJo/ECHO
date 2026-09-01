// -*- verilog -*-
// particle_scorer_pair_arb.v — particle_scorer_arb 2개가 진짜 single-port
// table_mem 하나를 arbiter2(전국AI반도체경진대회 프로젝트 재사용)로 매 클럭
// 중재해서 나눠 쓴다. table_mem_dp로 "진짜 듀얼포트 공유"를 시도했다가 합성기가
// 통째로 복사해버려 BRAM이 2배(39->~78, 예산초과)로 늘었던 문제의 대안 — 이
// 버전은 물리 메모리가 처음부터 끝까지 1개뿐이라 BRAM 추가비용이 원천적으로 없음
// (대신 두 스코어러가 같은 클럭에 동시에 테이블을 요청하면 한쪽이 1클럭 대기).

module particle_scorer_pair_arb #(
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
    wire signed [DATA_W-1:0] table_data;   // 진짜 물리 테이블 출력, 둘 다 같은 걸 봄
    wire                      req0, req1;
    wire [1:0]                gnt;

    // sensor_pe는 valid_i를 준 지 1클럭 뒤 table_data를 쓰는데, 내부에 pend_valid
    // 릴레이 한 단이 더 있어서 실제로는 "승인 클럭 + 그 다음 클럭" 2클럭 내내 같은
    // 주소가 물리 테이블에 걸려있어야 한다(테이블_mem 자체 읽기지연 1클럭 + 그
    // pend_valid 릴레이 1클럭). 승인 직후 1클럭 동안 새 중재를 막고(busy) 같은
    // 승자의 주소를 그대로 붙잡아둬서(held_sel) 이걸 보장한다.
    reg busy, held_sel;
    wire [1:0] eff_req = busy ? 2'b00 : {req1, req0};

    arbiter2 u_arb (
        .clk(clk), .rst(~rst_n),
        .req(eff_req),
        .gnt(gnt)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; held_sel <= 1'b0;
        end else if (|gnt) begin
            busy <= 1'b1; held_sel <= gnt[1];
        end else if (busy) begin
            busy <= 1'b0;
        end
    end

    wire sel_now = busy ? held_sel : gnt[1];
    wire [ADDR_W-1:0] table_addr_shared = sel_now ? table_addr1 : table_addr0;

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

endmodule
