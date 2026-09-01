// -*- verilog -*-
// ray_march.v — ECHO v2 첫 모듈. 시작 위치에서 방향벡터를 따라 고정 스텝(1.0)으로
// 격자 지도를 걸으며, 벽(또는 지도 경계)에 부딪힐 때까지 걸린 스텝 수를 낸다.
//
// v2.0 스코프(의도적 단순화, 다음 단계에서 다룰 것):
//   - 스텝 크기 고정 1.0 — 진짜 range_libc의 "마칭"은 거리장 기반으로 빈 공간에서
//     성큼성큼 걷는데(더 빠름), 이 모듈은 기본 골격(격자 걷기+충돌판정+상태기계)
//     검증이 목적이라 아직 그 최적화는 안 함.
//   - 방향벡터(dx,dy)는 외부에서 이미 계산돼서 들어옴 — cos/sin을 하드웨어로
//     만드는 건(CORDIC 등) 완전히 다른 주제라 범위 밖.
//   - 지도는 작은 테스트용(20x20) 하드코딩 — 실제 트랙 크기 지도는 이후 과제.
//
// 지도 밖으로 나가는 것도 "막힘"으로 취급한다 — bagviz의 Grid.collided와 같은 관례.

module ray_march #(
    parameter GRID_W    = 20,
    parameter GRID_H    = 20,
    parameter INT_W     = 6,               // 정수부 비트 (0~63 커버, 20x20이면 충분)
    parameter FRAC_W    = 8,               // 소수부 비트
    parameter POS_W     = INT_W + FRAC_W,  // 14
    parameter MAX_STEPS = 40,
    parameter DIST_W    = 6                // ceil(log2(MAX_STEPS+1))
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [POS_W-1:0] x0, y0,  // 시작 위치 (Q6.8, 격자 칸 단위)
    input  wire signed [POS_W-1:0] dx, dy,  // 단위 방향벡터 (Q6.8, |dx,dy|=1.0)
    output reg                     done,
    output reg  [DIST_W-1:0]       dist_o,  // 부딪힐 때까지 걸린 스텝 수
    output reg                     hit_o    // 1=벽/경계에 부딪힘, 0=최대거리까지 못 찾음
);

    reg signed [POS_W-1:0] x, y;
    reg [DIST_W-1:0]        steps;
    reg                      running;

    // 현재 칸의 정수부만 추출 — 비트 슬라이스라 공짜(연산 없음)
    wire signed [INT_W-1:0] x_int = x[POS_W-1:FRAC_W];
    wire signed [INT_W-1:0] y_int = y[POS_W-1:FRAC_W];
    wire out_of_bounds = (x_int < 0) || (x_int >= GRID_W) ||
                         (y_int < 0) || (y_int >= GRID_H);

    // 지도 배열 밖 접근을 막으려고 인덱스를 안전 범위로 눌러줌(out_of_bounds가
    // 어차피 그 결과를 무시하고 강제로 "막힘" 처리하니 값 자체는 안 중요함)
    wire [INT_W-1:0] x_idx = out_of_bounds ? {INT_W{1'b0}} : x_int[INT_W-1:0];
    wire [INT_W-1:0] y_idx = out_of_bounds ? {INT_W{1'b0}} : y_int[INT_W-1:0];

    // 테스트 지도: 전부 빈 공간, x=15 열만 세로로 벽 하나
    reg [GRID_W-1:0] map_row [0:GRID_H-1];
    integer mi;
    initial begin
        for (mi = 0; mi < GRID_H; mi = mi + 1)
            map_row[mi] = (1 << 15);
    end

    wire map_bit  = map_row[y_idx][x_idx];
    wire occupied = out_of_bounds || map_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0; running <= 1'b0; dist_o <= {DIST_W{1'b0}}; hit_o <= 1'b0;
            x <= {POS_W{1'b0}}; y <= {POS_W{1'b0}}; steps <= {DIST_W{1'b0}};
        end else begin
            done <= 1'b0;   // 기본값 — 아래서 끝났을 때만 이번 클럭에 1로
            if (start) begin
                x <= x0; y <= y0; steps <= {DIST_W{1'b0}}; running <= 1'b1;
            end else if (running) begin
                if (occupied) begin
                    running <= 1'b0; done <= 1'b1;
                    dist_o <= steps; hit_o <= 1'b1;
                end else if (steps >= MAX_STEPS) begin
                    running <= 1'b0; done <= 1'b1;
                    dist_o <= steps; hit_o <= 1'b0;
                end else begin
                    x <= x + dx; y <= y + dy; steps <= steps + 1'b1;
                end
            end
        end
    end

endmodule
