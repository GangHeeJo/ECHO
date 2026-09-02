// -*- verilog -*-
// particle_scorer_dgen_arb.v — particle_scorer_dgen.v(direction_gen 겹침
// 통합) + particle_scorer_arb.v(공유 table_mem 포트 req/gnt 중재)를 합침.
// 8-way로 묶을 때 쓰는 단위 셀 — direction_gen은 인스턴스마다 복제(저렴,
// 341LUT 정도라 8개 붙여도 얼마 안 됨), table_mem만 계속 공유.

module particle_scorer_dgen_arb #(
    parameter RM_POS_W    = 18,
    parameter RM_DIST_W   = 9,
    parameter RD_W        = 9,
    parameter ADDR_W      = 17,
    parameter DATA_W      = 13,
    parameter ACC_W       = 20,
    parameter NUM_RAYS    = 60,
    parameter BEAM_W      = 6,
    parameter ANGLE_W     = 19,
    parameter ATAN_FILE       = "cordic_atan.hex",
    parameter BEAM_ANGLE_FILE = "beam_angles.hex"
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        particle_start,
    input  wire signed [RM_POS_W-1:0]  x0, y0,
    input  wire signed [ANGLE_W-1:0]   theta,

    input  wire                        beam_start,
    input  wire [RD_W-1:0]             r_obs,

    output reg                         beam_done,
    output reg                         particle_done,
    output wire signed [ACC_W-1:0]     weight_o,

    // 공유 테이블 포트 인터페이스(particle_scorer_arb.v와 동일 계약)
    output wire [ADDR_W-1:0]           table_addr,
    input  wire signed [DATA_W-1:0]    table_data,
    output wire                        req,
    input  wire                        gnt
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
        .table_addr(), .table_data(table_data),
        .start(pe_start), .valid_i(pe_valid), .last_i(pe_last), .addr_i(pe_addr),
        .done(pe_done), .weight_o(weight_o)
    );

    // ── 방향 생성기(프리페치 1개짜리, particle_scorer_dgen.v와 동일) ──
    reg  signed [ANGLE_W-1:0]  theta_lat;
    reg  [BEAM_W-1:0]          dgen_beam_id;
    reg                        dgen_start;
    wire                       dgen_done;
    wire signed [RM_POS_W-1:0] dgen_dx, dgen_dy;

    direction_gen #(
        .W_EXT(ANGLE_W), .W_OUT(RM_POS_W), .NUM_RAYS(NUM_RAYS), .BEAM_W(BEAM_W),
        .ATAN_FILE(ATAN_FILE), .BEAM_ANGLE_FILE(BEAM_ANGLE_FILE)
    ) u_dgen (
        .clk(clk), .rst_n(rst_n), .start(dgen_start),
        .theta_i(theta_lat), .beam_id(dgen_beam_id),
        .done(dgen_done), .dx_o(dgen_dx), .dy_o(dgen_dy)
    );

    reg signed [RM_POS_W-1:0] pref_dx, pref_dy;
    reg                       pref_valid;
    reg  [BEAM_W-1:0]         cur_beam;
    reg                       beam_start_seen;

    localparam S_IDLE      = 3'd0;
    localparam S_WAIT_BEAM = 3'd1;
    localparam S_MARCH     = 3'd2;
    localparam S_SCORE     = 3'd3;   // 테이블 포트 요청 후 승인 대기
    localparam S_BEAM_DONE = 3'd4;
    localparam S_WAIT_PE   = 3'd5;

    reg [2:0] state;

    assign req        = (state == S_SCORE);
    assign table_addr = ag_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            beam_done <= 1'b0; particle_done <= 1'b0;
            rm_start <= 1'b0; pe_start <= 1'b0; pe_valid <= 1'b0; pe_last <= 1'b0;
            rm_x0 <= 0; rm_y0 <= 0; rm_dx <= 0; rm_dy <= 0;
            ag_r <= 0; ag_d <= 0; pe_addr <= 0;
            theta_lat <= 0; dgen_beam_id <= 0; dgen_start <= 1'b0;
            pref_dx <= 0; pref_dy <= 0; pref_valid <= 1'b0; cur_beam <= 0;
            beam_start_seen <= 1'b0;
        end else begin
            beam_done <= 1'b0; particle_done <= 1'b0;
            rm_start <= 1'b0; pe_start <= 1'b0; pe_valid <= 1'b0;
            dgen_start <= 1'b0;

            if (dgen_done) begin
                pref_dx    <= dgen_dx;
                pref_dy    <= dgen_dy;
                pref_valid <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    if (particle_start) begin
                        rm_x0 <= x0; rm_y0 <= y0;
                        theta_lat <= theta;
                        pe_start  <= 1'b1;
                        cur_beam  <= {BEAM_W{1'b0}};
                        pref_valid <= 1'b0;
                        beam_start_seen <= 1'b0;
                        dgen_beam_id <= {BEAM_W{1'b0}};
                        dgen_start   <= 1'b1;
                        state <= S_WAIT_BEAM;
                    end
                end

                S_WAIT_BEAM: begin
                    if (beam_start) begin
                        ag_r <= r_obs;
                        beam_start_seen <= 1'b1;
                    end
                    if ((beam_start_seen || beam_start) && pref_valid) begin
                        rm_dx <= pref_dx; rm_dy <= pref_dy;
                        pref_valid <= 1'b0;
                        beam_start_seen <= 1'b0;
                        rm_start <= 1'b1;
                        if (cur_beam + 1 < NUM_RAYS) begin
                            dgen_beam_id <= cur_beam + 1'b1;
                            dgen_start   <= 1'b1;
                        end
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
                    if (gnt) begin
                        pe_addr  <= ag_addr;
                        pe_valid <= 1'b1;
                        pe_last  <= (cur_beam == NUM_RAYS-1);
                        state    <= S_BEAM_DONE;
                    end
                end

                S_BEAM_DONE: begin
                    beam_done <= 1'b1;
                    cur_beam  <= cur_beam + 1'b1;
                    if (cur_beam == NUM_RAYS-1)
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
