// -*- verilog -*-
// particle_scorer_shared.v — particle_scorer.v와 완전히 동일한데, 딱 하나
// 다르다: 센서모델 테이블(table_mem)을 내부에 혼자 갖지 않고 외부 포트로 뺐다
// (sensor_pe.v가 원래부터 쓰던 것과 같은 패턴). 합성해보니 BRAM 78%(39/50)가
// 전부 이 테이블 하나 때문이었음 — table_mem_dp.v(v1에서 이미 검증된 듀얼포트
// 공유)를 밖에서 인스턴스화해서 여러 particle_scorer가 물리적으로 같은 BRAM을
// 나눠 쓰게 하면(듀얼포트는 메모리를 2벌 복사하는 게 아니라 포트만 2개인 것),
// particle_scorer 하나가 쓰던 BRAM 예산 그대로 2개를 돌릴 수 있는지 확인하는 게
// 목적. particle_scorer.v 자체(단독용, 테이블 내장)는 안 건드리고 그대로 둠.

module particle_scorer_shared #(
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

    // 센서모델 테이블(BRAM) 인터페이스 — 밖에서 table_mem/table_mem_dp를 물려준다
    output wire [ADDR_W-1:0]           table_addr,
    input  wire signed [DATA_W-1:0]    table_data
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
