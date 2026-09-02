// -*- verilog -*-
// tb_particle_scorer_60beam.v — particle_scorer.v(기존 배럴시프터, RTL 변경 없음)
// 를 실제 ZERO와 같은 빔 개수(60)로 돌려 진짜 latency를 잰다. RTL은 particle
// 개수/빔 개수에 대해 파라미터화돼있지 않고 그냥 "빔 시작을 몇 번 주느냐"로만
// 결정되므로, 재합성 없이 이 테스트벤치만으로 60빔 1파티클의 실제 사이클 수를
// 얻을 수 있다 — tools/gen_particle_scorer_60beam_test.py가 만든 정답지 사용.
//
//   iverilog -o sim/tb_particle_scorer_60beam.vvp rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer.v tb/tb_particle_scorer_60beam.v
//   cd sim && vvp tb_particle_scorer_60beam.vvp

`timescale 1ns/1ps

module tb_particle_scorer_60beam;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 60;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        particle_start;
    reg  signed [RM_POS_W-1:0] x0, y0;
    reg                        beam_start;
    reg  signed [RM_POS_W-1:0] dx, dy;
    reg  [RD_W-1:0]            r_obs;
    reg                        beam_last;
    wire                       beam_done, particle_done;
    wire signed [ACC_W-1:0]    weight_o;

    particle_scorer #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_scorer (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start), .x0(x0), .y0(y0),
        .beam_start(beam_start), .dx(dx), .dy(dy), .r_obs(r_obs), .beam_last(beam_last),
        .beam_done(beam_done), .particle_done(particle_done), .weight_o(weight_o)
    );

    reg signed [RM_POS_W-1:0] dxs [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] dys [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] x0y0 [0:1];
    reg signed [ACC_W-1:0]    expected_mem [0:0];

    integer i;
    time t_particle_start, t_particle_done;

    initial begin
        $readmemh("p60_0_dx.hex", dxs);
        $readmemh("p60_0_dy.hex", dys);
        $readmemh("p60_0_r.hex",  rs);
        $readmemh("p60_0_x0y0.hex", x0y0);
        $readmemh("p60_0_expected.hex", expected_mem);
    end

    initial begin
        particle_start = 0; beam_start = 0;
        x0 = 0; y0 = 0; dx = 0; dy = 0; r_obs = 0; beam_last = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        x0 = x0y0[0];
        y0 = x0y0[1];

        @(posedge clk); particle_start <= 1; t_particle_start = $time;
        @(posedge clk); particle_start <= 0;

        for (i = 0; i < NUM_RAYS; i = i + 1) begin
            @(posedge clk);
            dx <= dxs[i]; dy <= dys[i]; r_obs <= rs[i];
            beam_last <= (i == NUM_RAYS - 1);
            beam_start <= 1;
            @(posedge clk);
            beam_start <= 0;
            wait (beam_done);
        end

        wait (particle_done);
        t_particle_done = $time;
        @(posedge clk);

        $display("빔 개수: %0d", NUM_RAYS);
        $display("particle_start -> particle_done: %0d ns (%0d clk @ 10ns)",
                  t_particle_done - t_particle_start, (t_particle_done - t_particle_start) / 10);

        if (weight_o === expected_mem[0]) begin
            $display("PASS: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
        end else begin
            $display("FAIL: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
        end
        $finish;
    end

endmodule
