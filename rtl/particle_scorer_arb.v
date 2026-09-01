// -*- verilog -*-
// particle_scorer_arb.v — particle_scorer_shared.v와 거의 같은데, S_SCORE에서
// 곧바로 pe_valid를 쏘지 않고 공유 테이블 포트에 대한 요청(req)을 걸고 승인
// (gnt, 전국AI반도체경진대회 프로젝트의 arbiter2.v가 매 클럭 공정하게 승자를
// 정해줌)이 떨어질 때까지 그 상태에서 대기한다 — 승인되면 그때 원래처럼
// pe_valid를 1클럭 쏜다. sensor_pe.v 내부의 "1클럭 뒤 테이블 데이터가 돌아온다"
// 타이밍 계약은 전혀 안 건드림(승인된 순간부터 원래 그대로 동작).
//
// table_mem_dp로 진짜 듀얼포트를 시도했다가 BRAM이 오히려 2배로 늘어난 것과
// 달리(합성기가 90601깊이 테이블의 진짜 듀얼포트 크로스바를 못 만들고 통째로
// 복사해버림), 이 방식은 물리적으로 진짜 single-port table_mem 하나만 그대로
// 쓰고 "누가 먼저 쓸지"만 매 클럭 중재한다 — BRAM 추가비용이 원천적으로 0.

module particle_scorer_arb #(
    parameter RM_POS_W    = 18,
    parameter RM_DIST_W   = 9,
    parameter RD_W        = 9,
    parameter ADDR_W      = 17,
    parameter DATA_W      = 13,
    parameter ACC_W        = 20
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        particle_start,
    input  wire signed [RM_POS_W-1:0]  x0, y0,

    input  wire                        beam_start,
    input  wire signed [RM_POS_W-1:0]  dx, dy,
    input  wire [RD_W-1:0]             r_obs,
    input  wire                        beam_last,

    output reg                         beam_done,
    output reg                         particle_done,
    output wire signed [ACC_W-1:0]     weight_o,

    // 공유 테이블 포트 인터페이스 — 중재로 딴 순간에만 addr가 의미 있음
    output wire [ADDR_W-1:0]           table_addr,
    input  wire signed [DATA_W-1:0]    table_data,
    output wire                        req,   // "이번 클럭에 테이블 쓰고 싶다"
    input  wire                        gnt    // 중재기가 이번 클럭 승인
);

    reg                        rm_start;
    reg  signed [RM_POS_W-1:0] rm_x0, rm_y0, rm_dx, rm_dy;
    wire                       rm_done, rm_hit;
    wire [RM_DIST_W-1:0]       rm_dist;

    ray_march_edt u_ray_march (
        .clk(clk), .rst_n(rst_n), .start(rm_start),
        .x0(rm_x0), .y0(rm_y0), .dx(rm_dx), .dy(rm_dy),
        .done(rm_done), .dist_o(rm_dist), .hit_o(rm_hit)
    );

    reg  [RD_W-1:0]      ag_r, ag_d;
    wire [ADDR_W-1:0]     ag_addr;
    addr_gen #(.RD_W(RD_W), .ADDR_W(ADDR_W)) u_addr_gen (
        .r(ag_r), .d(ag_d), .addr(ag_addr)
    );

    reg                  pe_start, pe_valid, pe_last;
    reg  [ADDR_W-1:0]    pe_addr;
    wire                 pe_done;
    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
        .clk(clk), .rst_n(rst_n),
        .table_addr(), .table_data(table_data),   // table_addr는 아래서 별도로 뺌(중재 전엔 무의미)
        .start(pe_start), .valid_i(pe_valid), .last_i(pe_last), .addr_i(pe_addr),
        .done(pe_done), .weight_o(weight_o)
    );

    localparam S_IDLE       = 3'd0;
    localparam S_WAIT_BEAM  = 3'd1;
    localparam S_MARCH      = 3'd2;
    localparam S_SCORE      = 3'd3;   // 테이블 포트 요청 후 승인 대기
    localparam S_BEAM_DONE  = 3'd4;
    localparam S_WAIT_PE    = 3'd5;

    reg [2:0] state;
    reg       cur_last;

    assign req        = (state == S_SCORE);
    assign table_addr = ag_addr;   // S_SCORE 아닐 때도 그냥 흘려보내되, req=0이라 무시됨

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            beam_done <= 1'b0; particle_done <= 1'b0;
            rm_start <= 1'b0; pe_start <= 1'b0; pe_valid <= 1'b0; pe_last <= 1'b0;
            rm_x0 <= 0; rm_y0 <= 0; rm_dx <= 0; rm_dy <= 0;
            ag_r <= 0; ag_d <= 0; pe_addr <= 0; cur_last <= 1'b0;
        end else begin
            beam_done <= 1'b0; particle_done <= 1'b0;
            rm_start <= 1'b0; pe_start <= 1'b0; pe_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (particle_start) begin
                        rm_x0 <= x0; rm_y0 <= y0;
                        pe_start <= 1'b1;
                        state <= S_WAIT_BEAM;
                    end
                end

                S_WAIT_BEAM: begin
                    if (beam_start) begin
                        rm_dx <= dx; rm_dy <= dy;
                        ag_r  <= r_obs;
                        cur_last <= beam_last;
                        rm_start <= 1'b1;
                        state <= S_MARCH;
                    end
                end

                S_MARCH: begin
                    if (rm_done) begin
                        ag_d <= rm_dist;
                        state <= S_SCORE;
                    end
                end

                S_SCORE: begin
                    // req는 위에서 조합적으로 이미 걸려있음 — gnt 뜰 때까지 그냥 대기
                    if (gnt) begin
                        pe_addr  <= ag_addr;
                        pe_valid <= 1'b1;
                        pe_last  <= cur_last;
                        state    <= S_BEAM_DONE;
                    end
                end

                S_BEAM_DONE: begin
                    beam_done <= 1'b1;
                    if (cur_last)
                        state <= S_WAIT_PE;
                    else
                        state <= S_WAIT_BEAM;
                end

                S_WAIT_PE: begin
                    if (pe_done) begin
                        particle_done <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
