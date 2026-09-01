// -*- verilog -*-
// tb_particle_scorer.v — particle_scorer.v(레이마칭+센서모델 결합)를 파이썬
// 정답지(echo-ref/gen_particle_scorer_test.py 산출물)와 대조한다.
// 파티클 1개, changwon 트랙 자유공간 한 점에서 부채꼴 8방향 — 실제 파이프라인
// 전체("레이마칭으로 기대거리 구하기 -> 센서모델로 채점")를 처음으로 끝까지 검증.
//
//   iverilog -o sim/tb_particle_scorer.vvp rtl/ray_march_bram.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer.v tb/tb_particle_scorer.v
//   cd sim && vvp tb_particle_scorer.vvp

`timescale 1ns/1ps

module tb_particle_scorer;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

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
    reg signed [ACC_W-1:0]    expected_mem [0:0];  // $readmemh는 스칼라가 아니라 배열(메모리)만 받음

    integer i;

    initial begin
        $readmemh("particle0_dx.hex", dxs);
        $readmemh("particle0_dy.hex", dys);
        $readmemh("particle0_r.hex",  rs);
        $readmemh("particle0_x0y0.hex", x0y0);
        $readmemh("particle0_expected.hex", expected_mem);
    end

    initial begin
        $dumpfile("tb_particle_scorer.vcd");
        $dumpvars(0, tb_particle_scorer);

        particle_start = 0; beam_start = 0;
        x0 = 0; y0 = 0; dx = 0; dy = 0; r_obs = 0; beam_last = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        x0 = x0y0[0];
        y0 = x0y0[1];

        @(posedge clk); particle_start <= 1; @(posedge clk); particle_start <= 0;

        for (i = 0; i < NUM_RAYS; i = i + 1) begin
            @(posedge clk);
            dx <= dxs[i]; dy <= dys[i]; r_obs <= rs[i];
            beam_last <= (i == NUM_RAYS - 1);
            beam_start <= 1;
            @(posedge clk);
            beam_start <= 0;
            wait (beam_done);
            $display("빔 %0d 처리 완료 (t=%0t ns)", i, $time);
        end

        wait (particle_done);
        @(posedge clk);

        if (weight_o === expected_mem[0]) begin
            $display("PASS: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
            $display("ALL TESTS PASSED (레이마칭 -> 센서모델 전체 파이프라인)");
        end else begin
            $display("FAIL: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
            $display("SOME TESTS FAILED");
        end
        $finish;
    end

endmodule
