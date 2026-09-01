// -*- verilog -*-
// particle_scorer.v — ECHO v1(센서모델)과 v2(레이마칭)를 실제로 이어붙인 첫 조합.
// 빔 하나마다: 레이마칭으로 기대거리(d) 구하기 -> 관측값(r)과 함께 addr_gen으로
// 주소 계산 -> sensor_pe에 한 클럭 먹여서 누적. 이걸 빔 개수만큼 반복해서 파티클
// 하나의 최종 점수(log-weight)를 낸다.
//
// 이 모듈이 바로 실제 particle_filter.py의 sensor_model() 한 사이클이 하는 일
// 전체를 하드웨어로 옮긴 것 — 지금까지 따로 검증한 조각들을 처음으로 실제 순서
// 그대로 이어붙인 것.
//
// 방향벡터(dx,dy)와 관측거리(r_obs)는 여전히 외부에서 줌(레이마칭 v2.0/2.1과
// 같은 스코프 — cos/sin 생성, 진짜 라이다 관측값 처리는 각각 다른 주제).

module particle_scorer #(
    // ray_march_bram 쪽
    parameter RM_POS_W    = 18,
    parameter RM_DIST_W   = 9,     // 0~300 (15m/5cm)
    // sensor_pe/addr_gen 쪽
    parameter RD_W        = 9,     // r,d 각각 0~300
    parameter ADDR_W      = 17,
    parameter DATA_W      = 13,
    parameter ACC_W        = 20
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // 파티클 시작 — 이 위치에서 여러 빔을 순서대로 처리
    input  wire                        particle_start,
    input  wire signed [RM_POS_W-1:0]  x0, y0,

    // 빔 하나 시작 — 방향과 "관측거리"를 준다
    input  wire                        beam_start,
    input  wire signed [RM_POS_W-1:0]  dx, dy,
    input  wire [RD_W-1:0]             r_obs,
    input  wire                        beam_last,

    output reg                         beam_done,      // 이번 빔 다 처리됨, 다음 빔 줘도 됨
    output reg                         particle_done,   // 마지막 빔까지 다 끝나서 weight_o 확정
    output wire signed [ACC_W-1:0]     weight_o
);

    // ── 서브모듈들 ──────────────────────────────────────────────────
    reg                        rm_start;
    reg  signed [RM_POS_W-1:0] rm_x0, rm_y0, rm_dx, rm_dy;
    wire                       rm_done, rm_hit;
    wire [RM_DIST_W-1:0]       rm_dist;

    ray_march_bram u_ray_march (
        .clk(clk), .rst_n(rst_n), .start(rm_start),
        .x0(rm_x0), .y0(rm_y0), .dx(rm_dx), .dy(rm_dy),
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

    // ── 조율 FSM ────────────────────────────────────────────────────
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
                        pe_start <= 1'b1;           // sensor_pe 누적기 클리어
                        state <= S_WAIT_BEAM;
                    end
                end

                S_WAIT_BEAM: begin
                    if (beam_start) begin
                        rm_dx <= dx; rm_dy <= dy;
                        ag_r  <= r_obs;
                        cur_last <= beam_last;
                        rm_start <= 1'b1;           // 이 빔의 레이마칭 시작
                        state <= S_MARCH;
                    end
                end

                S_MARCH: begin
                    if (rm_done) begin
                        ag_d <= rm_dist;             // 레이마칭 결과 = 기대거리
                        state <= S_SCORE;
                    end
                end

                S_SCORE: begin
                    // addr_gen은 조합논리라 ag_r/ag_d가 이미 정해진 지금 addr도 정해짐
                    pe_addr  <= ag_addr;
                    pe_valid <= 1'b1;
                    pe_last  <= cur_last;
                    state    <= S_BEAM_DONE;
                end

                S_BEAM_DONE: begin
                    beam_done <= 1'b1;
                    if (cur_last)
                        state <= S_WAIT_PE;          // sensor_pe의 done이 한 박자 뒤에 뜸
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
