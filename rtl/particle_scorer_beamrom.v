// -*- verilog -*-
// particle_scorer_beamrom.v — particle_scorer.v와 완전히 동일한 조율 FSM, 다만
// 레이마칭 쪽만 ray_march_edt_beamrom(배럴시프터 대신 beam ROM)을 씀. dx,dy 대신
// beam_id를 받는다 — 스코프 한정은 ray_march_edt_beamrom.v 상단 주석 참고.

module particle_scorer_beamrom #(
    parameter RM_POS_W    = 18,
    parameter RM_DIST_W   = 9,
    parameter BEAM_W      = 3,
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
    input  wire [BEAM_W-1:0]           beam_id,
    input  wire [RD_W-1:0]             r_obs,
    input  wire                        beam_last,

    output reg                         beam_done,
    output reg                         particle_done,
    output wire signed [ACC_W-1:0]     weight_o
);

    reg                        rm_start;
    reg  signed [RM_POS_W-1:0] rm_x0, rm_y0;
    reg  [BEAM_W-1:0]          rm_beam_id;
    wire                       rm_done, rm_hit;
    wire [RM_DIST_W-1:0]       rm_dist;

    ray_march_edt_beamrom u_ray_march (
        .clk(clk), .rst_n(rst_n), .start(rm_start),
        .x0(rm_x0), .y0(rm_y0), .beam_id(rm_beam_id),
        .done(rm_done), .dist_o(rm_dist), .hit_o(rm_hit)
    );

    reg  [RD_W-1:0]      ag_r, ag_d;
    wire [ADDR_W-1:0]     ag_addr;
    addr_gen #(.RD_W(RD_W), .ADDR_W(ADDR_W)) u_addr_gen (
        .r(ag_r), .d(ag_d), .addr(ag_addr)
    );

    wire [ADDR_W-1:0]        table_addr;
    wire signed [DATA_W-1:0] table_data;
    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
        .clk(clk), .addr(table_addr), .data(table_data)
    );

    reg                  pe_start, pe_valid, pe_last;
    reg  [ADDR_W-1:0]    pe_addr;
    wire                 pe_done;
    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
        .clk(clk), .rst_n(rst_n),
        .table_addr(table_addr), .table_data(table_data),
        .start(pe_start), .valid_i(pe_valid), .last_i(pe_last), .addr_i(pe_addr),
        .done(pe_done), .weight_o(weight_o)
    );

    localparam S_IDLE       = 3'd0;
    localparam S_WAIT_BEAM  = 3'd1;
    localparam S_MARCH      = 3'd2;
    localparam S_SCORE      = 3'd3;
    localparam S_BEAM_DONE  = 3'd4;
    localparam S_WAIT_PE    = 3'd5;

    reg [2:0] state;
    reg       cur_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            beam_done <= 1'b0; particle_done <= 1'b0;
            rm_start <= 1'b0; pe_start <= 1'b0; pe_valid <= 1'b0; pe_last <= 1'b0;
            rm_x0 <= 0; rm_y0 <= 0; rm_beam_id <= 0;
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
                        rm_beam_id <= beam_id;
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
                    pe_addr  <= ag_addr;
                    pe_valid <= 1'b1;
                    pe_last  <= cur_last;
                    state    <= S_BEAM_DONE;
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
