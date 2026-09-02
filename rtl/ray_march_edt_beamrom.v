// -*- verilog -*-
// ray_march_edt_beamrom.v — ray_march_edt.v의 범용 배럴시프터(dx/dy <<< cur_shift)
// 를 작은 ROM 조회로 바꾼 실험판. dx,dy 입력 대신 beam_id를 받고, {beam_id,
// cur_shift} 64칸짜리 ROM에서 미리 계산해둔 step_dx/step_dy를 바로 읽는다.
//
// ⚠️ 스코프 한정(tools/gen_beam_step_rom.py 상단 주석과 동일): 이 ROM은 "빔 8개
// 각도가 고정"이라는 이번 벤치마크 전제에서만 유효하다. 실제 ZERO는 파티클
// theta(연속값)에 따라 빔의 world-frame 방향이 매번 달라져서, beam_id만으로는
// 이 ROM을 실전에 그대로 못 씀 — theta 양자화까지 주소에 넣어야 하는데 그러면
// ROM이 커져 이번에 아끼는 만큼 다시 손해볼 수 있음. 크리티컬패스/자원 A/B
// 목적의 실험판.

module ray_march_edt_beamrom #(
    parameter GRID_W    = 400,
    parameter GRID_H    = 160,
    parameter PADX_W    = 9,
    parameter INT_W     = 10,
    parameter FRAC_W    = 8,
    parameter POS_W     = INT_W + FRAC_W,       // 18
    parameter SHIFT_W   = 3,                    // 스텝 = 2^shift, shift는 0~7
    parameter NUM_RAYS  = 8,
    parameter BEAM_W    = 3,                    // $clog2(NUM_RAYS)
    parameter MAX_STEPS = 300,
    parameter DIST_W    = $clog2(MAX_STEPS+1),
    parameter MAP_ADDR_W = 17,
    parameter OCC_FILE   = "changwon_occ.hex",
    parameter EDT_FILE   = "changwon_edt_shift.hex",
    parameter STEP_DX_FILE = "beam_step_dx.hex",
    parameter STEP_DY_FILE = "beam_step_dy.hex"
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [POS_W-1:0] x0, y0,
    input  wire [BEAM_W-1:0]       beam_id,
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

    // 배럴 시프터 대신 작은 ROM 조회 — 주소 = {beam_id, cur_shift}
    localparam ROM_DEPTH = NUM_RAYS * (1 << SHIFT_W);
    reg signed [POS_W-1:0] step_dx_rom [0:ROM_DEPTH-1];
    reg signed [POS_W-1:0] step_dy_rom [0:ROM_DEPTH-1];
    initial $readmemh(STEP_DX_FILE, step_dx_rom);
    initial $readmemh(STEP_DY_FILE, step_dy_rom);

    wire [BEAM_W+SHIFT_W-1:0] rom_addr = {beam_id, cur_shift};
    wire signed [POS_W-1:0] step_dx = step_dx_rom[rom_addr];
    wire signed [POS_W-1:0] step_dy = step_dy_rom[rom_addr];
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
