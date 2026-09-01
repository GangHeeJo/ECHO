// -*- verilog -*-
// ray_march_bram.v — ray_march.v를 실제 트랙 크기(400x160칸, changwon 트랙 map.pgm)
// 지도로 확장한 버전. 가장 큰 차이: 지도를 2차원 배열로 인덱싱하던 걸(작을 때만
// 통하는, 멀티플렉서로 합성되는 방식) **평면(flat) 메모리 + 계산된 주소**로 바꿈 —
// 진짜 합성 가능한 BRAM 방식.
//
// 주소 계산: 폭을 512(2의 거듭제곱)로 패딩해뒀기 때문에(echo-ref/gen_track_map.py),
//   addr = y*512 + x = {y, 9'b0} | x
// 즉 그냥 비트를 이어붙이기만 하면 된다 — addr_gen.v(301처럼 애매한 상수)와 달리
// 이번엔 시프트-덧셈도 필요 없는, 더 쉬운 케이스.
//
// ponytail: 지도를 셀당 1비트로 그대로 저장해서 씀 — 실제로는 32비트씩 묶어
// 워드 단위로 저장하는 게 BRAM을 더 알차게 쓰는 방법인데, 이번 단계 스코프 밖.

module ray_march_bram #(
    parameter GRID_W    = 400,
    parameter GRID_H    = 160,
    parameter PADX_W    = 9,                // 512=2^9 (x 주소 비트폭, 부호 없음)
    // 정수부는 부호있는(signed) 값이라 9비트로는 양수를 255까지밖에 못 담는다
    // (부호비트 1개를 뺏기니까) — 트랙 폭 400을 담으려면 10비트 필요.
    // (2026-09-02: 실제로 9비트로 했다가 x=256에서 음수로 넘쳐서 가짜 경계이탈
    //  판정이 났던 버그를 실측으로 잡음 — center_+x 케이스가 119 대신 56에서 멈췄었음)
    parameter INT_W     = 10,               // 정수부(부호있음): -512~511 커버
    parameter FRAC_W    = 8,
    parameter POS_W     = INT_W + FRAC_W,   // 18
    parameter MAX_STEPS = 300,              // 15m / 0.05m
    parameter DIST_W    = $clog2(MAX_STEPS+1), // MAX_STEPS만 바꾸면 자동으로 맞춰짐
    parameter MAP_ADDR_W = 17,              // GRID_H(8bit,y) + PADX_W(9bit,x)
    parameter INIT_FILE = "changwon_occ.hex"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [POS_W-1:0] x0, y0,
    input  wire signed [POS_W-1:0] dx, dy,
    output reg                     done,
    output reg  [DIST_W-1:0]       dist_o,
    output reg                     hit_o
);

    reg signed [POS_W-1:0] x, y;
    reg [DIST_W-1:0]        steps;
    reg                      running;

    wire signed [INT_W-1:0] x_int = x[POS_W-1:FRAC_W];
    wire signed [INT_W-1:0] y_int = y[POS_W-1:FRAC_W];
    wire out_of_bounds = (x_int < 0) || (x_int >= GRID_W) ||
                         (y_int < 0) || (y_int >= GRID_H);

    // 안전 범위로 눌러서 주소를 만듦(out_of_bounds가 결과를 어차피 무시함)
    wire [PADX_W-1:0] x_idx = out_of_bounds ? {PADX_W{1'b0}} : x_int[PADX_W-1:0];
    wire [7:0]         y_idx = out_of_bounds ? 8'b0 : y_int[7:0];

    // 평면 주소: 곱셈도 시프트-덧셈도 없이 그냥 이어붙이기(y*512+x, 512=2^9)
    wire [MAP_ADDR_W-1:0] map_addr = {y_idx, x_idx};

    reg map_bit_arr [0:GRID_H*(1<<PADX_W)-1];
    initial $readmemb(INIT_FILE, map_bit_arr);

    wire map_bit  = map_bit_arr[map_addr];
    wire occupied = out_of_bounds || map_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0; running <= 1'b0; dist_o <= {DIST_W{1'b0}}; hit_o <= 1'b0;
            x <= {POS_W{1'b0}}; y <= {POS_W{1'b0}}; steps <= {DIST_W{1'b0}};
        end else begin
            done <= 1'b0;
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
