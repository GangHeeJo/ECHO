// -*- verilog -*-
// ray_march_edt.v — ray_march_bram.v를 "거리장(EDT) 기반 마칭"으로 최적화한 버전.
// 매 스텝 1칸씩만 가는 대신, "지금 위치에서 가장 가까운 벽까지의 안전거리"만큼
// 한 번에 전진한다 — 빈 공간에서는 성큼성큼, 벽 근처에서는 조심조심.
//
// 안전거리는 런타임에 지도에서 읽어온 값이라(컴파일타임 상수가 아님), 방향벡터에
// 그대로 곱하면 진짜 곱셈기가 필요하다. 그래서 안전거리를 **2의 거듭제곱 중 그
// 이하인 가장 큰 값**으로 반올림해서 저장해뒀고(echo-ref/gen_track_map.py의
// compute_edt_shift), 그 지수(shift)만큼 **가변 시프트(배럴 시프터)**로 곱셈을
// 대신한다 — 곱셈기 없이 큰 보폭을 얻는 절충안. ponytail: 최적의 보폭보다 약간
// 작게 걷는 손해가 있지만(반올림), 그래도 고정 1칸 걷기보다 훨씬 빠르다(실측
// 2.7~7.5배 반복 횟수 감소).
//
// ray_march_bram.v와 인터페이스는 완전히 동일 — 내부 스텝 크기만 다름.

module ray_march_edt #(
    parameter GRID_W    = 400,
    parameter GRID_H    = 160,
    parameter PADX_W    = 9,
    parameter INT_W     = 10,
    parameter FRAC_W    = 8,
    parameter POS_W     = INT_W + FRAC_W,       // 18
    parameter SHIFT_W   = 3,                    // 스텝 = 2^shift, shift는 0~7
    parameter MAX_STEPS = 300,
    parameter DIST_W    = $clog2(MAX_STEPS+1),
    parameter MAP_ADDR_W = 17,
    parameter OCC_FILE  = "changwon_occ.hex",
    parameter EDT_FILE  = "changwon_edt_shift.hex"
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
    reg [DIST_W-1:0]        dist_acc;
    reg                      running;

    wire signed [INT_W-1:0] x_int = x[POS_W-1:FRAC_W];
    wire signed [INT_W-1:0] y_int = y[POS_W-1:FRAC_W];
    wire out_of_bounds = (x_int < 0) || (x_int >= GRID_W) ||
                         (y_int < 0) || (y_int >= GRID_H);

    wire [PADX_W-1:0] x_idx = out_of_bounds ? {PADX_W{1'b0}} : x_int[PADX_W-1:0];
    wire [7:0]         y_idx = out_of_bounds ? 8'b0 : y_int[7:0];
    wire [MAP_ADDR_W-1:0] map_addr = {y_idx, x_idx};

    reg map_bit_arr [0:GRID_H*(1<<PADX_W)-1];
    initial $readmemb(OCC_FILE, map_bit_arr);
    wire map_bit  = map_bit_arr[map_addr];
    wire occupied = out_of_bounds || map_bit;

    reg [SHIFT_W-1:0] shift_arr [0:GRID_H*(1<<PADX_W)-1];
    initial $readmemb(EDT_FILE, shift_arr);
    wire [SHIFT_W-1:0] cur_shift = shift_arr[map_addr];

    // 곱셈 대신 가변 시프트(배럴 시프터) — dx,dy(방향)와 "1"(거리 누적용) 둘 다
    wire signed [POS_W-1:0] step_dx = dx <<< cur_shift;
    wire signed [POS_W-1:0] step_dy = dy <<< cur_shift;
    wire [DIST_W-1:0]        step_dist = {{(DIST_W-1){1'b0}}, 1'b1} <<< cur_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0; running <= 1'b0; dist_o <= {DIST_W{1'b0}}; hit_o <= 1'b0;
            x <= {POS_W{1'b0}}; y <= {POS_W{1'b0}}; dist_acc <= {DIST_W{1'b0}};
        end else begin
            done <= 1'b0;
            if (start) begin
                x <= x0; y <= y0; dist_acc <= {DIST_W{1'b0}}; running <= 1'b1;
            end else if (running) begin
                if (occupied) begin
                    running <= 1'b0; done <= 1'b1;
                    dist_o <= dist_acc; hit_o <= 1'b1;
                end else if (dist_acc >= MAX_STEPS) begin
                    running <= 1'b0; done <= 1'b1;
                    dist_o <= dist_acc; hit_o <= 1'b0;
                end else begin
                    x <= x + step_dx; y <= y + step_dy;
                    dist_acc <= dist_acc + step_dist;
                end
            end
        end
    end

endmodule
