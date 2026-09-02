// -*- verilog -*-
// particle_scorer_dgen.v — particle_scorer.v + direction_gen.v를 "겹쳐서"
// 연결한 버전. 외부 dx,dy,beam_last 포트를 없애고 대신 파티클 theta 하나만
// 받아 방향을 내부에서 만든다(그 결과 particle_start 시 넘기는 게 진짜
// (x,y,theta) 전체 pose가 됨 — 실제 ZERO의 파티클 표현과 동일).
//
// ⚠️ 그냥 순서대로("방향 만들고 -> 그 다음에 레이마칭") 이어붙이면 안 됨 —
// CORDIC은 방향 하나에 고정 ~20클럭이 걸리는데, 레이마칭은 빔당 평균
// ~16클럭(EDT라 빔마다 다름, 959사이클/60빔 실측 평균)이라 순서대로면
// 오히려 빔당 시간이 거의 2배(36클럭)로 늘어난다. 대신:
//
//   지금 빔 N을 레이마칭하는 동안, 동시에 빔 N+1의 방향을 미리 계산한다
//   (1개짜리 프리페치 버퍼 — pref_dx/pref_dy/pref_valid).
//
// 레이마칭(가변, 평균 ~16클럭)이 CORDIC(고정 ~20클럭)보다 빠른 빔에서만
// S_WAIT_BEAM에서 짧게 멈추고(드묾), 그 외엔 대기 없이 바로 다음 빔으로
// 넘어간다 — 완전 순차보다 훨씬 빠를 것으로 기대(실측은 테스트벤치에서).
//
// r_obs(관측거리)는 여전히 외부에서 빔마다 받음(beam_start로 페이싱) — 이건
// 실제 라이다 스캔값이라 애초에 FPGA가 만들어낼 수 있는 게 아님, direction
// generation과는 완전히 독립적인 문제라 겹칠 필요도 없음.
//
// ⚠️ 1차 구현에 데드락 버그 있었음(발견·수정): "방향 준비 대기"와 "beam_start
// 대기"를 별도 state(S_PREP_DIR0/S_WAIT_NEXT_DIR)로 나눴더니, 그 대기가
// 가변 길이라 이 코드베이스의 관례(beam_done 1클럭 뒤 beam_start를 딱
// 1클럭만 펄스로 줌)와 안 맞아 DUT가 아직 그 대기 state에 있는 동안 펄스가
// 지나가버려 영원히 못 받는 레이스 발생(테스트벤치가 120초 타임아웃으로
// 걸림 — 아무 출력도 없이 멈춘 게 단서). S_WAIT_BEAM 하나에서
// beam_start_seen 래치로 "r_obs 준비됨"과 "방향 준비됨" 두 독립 조건을
// 각자 기다리도록 합쳐서 해결 — 어느 쪽이 늦게 와도 놓치지 않음.

module particle_scorer_dgen #(
    parameter RM_POS_W    = 18,
    parameter RM_DIST_W   = 9,
    parameter RD_W        = 9,
    parameter ADDR_W      = 17,
    parameter DATA_W      = 13,
    parameter ACC_W       = 20,
    parameter NUM_RAYS    = 60,
    parameter BEAM_W      = 6,      // $clog2(NUM_RAYS)
    parameter ANGLE_W     = 19,     // direction_gen theta 포맷(Q3.16)
    parameter ATAN_FILE       = "cordic_atan.hex",
    parameter BEAM_ANGLE_FILE = "beam_angles.hex"
) (
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire                        particle_start,
    input  wire signed [RM_POS_W-1:0]  x0, y0,
    input  wire signed [ANGLE_W-1:0]   theta,        // |theta| <= pi

    input  wire                        beam_start,   // 이번 빔 r_obs 준비됨
    input  wire [RD_W-1:0]             r_obs,

    output reg                         beam_done,
    output reg                         particle_done,
    output wire signed [ACC_W-1:0]     weight_o
);

    // ── 레이마칭/센서모델 서브모듈 (particle_scorer.v와 동일) ─────────
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

    // ── 방향 생성기(프리페치 1개짜리) ──────────────────────────────
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
    reg                       beam_start_seen;   // beam_start 펄스를 놓치지 않으려는 래치

    // ── 조율 FSM ────────────────────────────────────────────────────
    // S_WAIT_BEAM 하나가 "r_obs 준비됨(beam_start)"과 "방향 준비됨(pref_valid)"
    // 두 독립적인 조건을 각자 걸리는 시간만큼 따로 기다린다 — 어느 쪽이 늦게
    // 와도 놓치지 않음(초기엔 direction_gen이 항상 병목, 이후엔 레이마칭이
    // 병목인 빔에서만 짧게 대기).
    localparam S_IDLE      = 3'd0;
    localparam S_WAIT_BEAM = 3'd1;
    localparam S_MARCH     = 3'd2;
    localparam S_SCORE     = 3'd3;
    localparam S_BEAM_DONE = 3'd4;
    localparam S_WAIT_PE   = 3'd5;

    reg [2:0] state;

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

            // direction_gen 결과가 나오면(현재 state와 무관하게) 즉시 프리페치
            // 버퍼에 잡아둠 — 이 설계는 항상 최대 1개의 dgen 요청만 동시에
            // 진행되므로(아래 FSM이 이전 결과를 소비한 뒤에만 다음 걸 건다)
            // 겹쳐 쓸 위험 없음.
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
                        dgen_start   <= 1'b1;         // 빔 0 방향 계산 시작
                        state <= S_WAIT_BEAM;
                    end
                end

                S_WAIT_BEAM: begin
                    if (beam_start) begin
                        ag_r <= r_obs;
                        beam_start_seen <= 1'b1;      // 방향이 아직 안 됐어도 놓치지 않음
                    end
                    if ((beam_start_seen || beam_start) && pref_valid) begin
                        rm_dx <= pref_dx; rm_dy <= pref_dy;
                        pref_valid <= 1'b0;
                        beam_start_seen <= 1'b0;
                        rm_start <= 1'b1;
                        if (cur_beam + 1 < NUM_RAYS) begin
                            dgen_beam_id <= cur_beam + 1'b1;
                            dgen_start   <= 1'b1;     // 지금 빔 마칭 중에 다음 빔 방향 미리 계산
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
                    pe_addr  <= ag_addr;
                    pe_valid <= 1'b1;
                    pe_last  <= (cur_beam == NUM_RAYS-1);
                    state    <= S_BEAM_DONE;
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
