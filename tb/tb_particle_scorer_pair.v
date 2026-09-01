// -*- verilog -*-
// tb_particle_scorer_pair.v — particle_scorer_pair(테이블 공유 2개)가 여전히
// 정답(파티클0/1)을 내는지 기능 검증. tb_particle_scorer_parallel.v와 같은
// 정답지, 같은 구조(서로 다른 named task 2개) — 차이는 DUT가 테이블을 내장한
// particle_scorer 2개가 아니라 테이블 하나를 공유하는 particle_scorer_pair라는 것.
//
//   iverilog -o sim/tb_particle_scorer_pair.vvp rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem_dp.v rtl/sensor_pe.v rtl/particle_scorer_shared.v rtl/particle_scorer_pair.v tb/tb_particle_scorer_pair.v
//   cd sim && vvp tb_particle_scorer_pair.vvp

`timescale 1ns/1ps

module tb_particle_scorer_pair;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        pstart0, pstart1;
    reg  signed [RM_POS_W-1:0] px0_0, py0_0, px0_1, py0_1;
    reg                        bstart0, bstart1;
    reg  signed [RM_POS_W-1:0] dx0, dy0, dx1, dy1;
    reg  [RD_W-1:0]            r0, r1;
    reg                        blast0, blast1;
    wire                       bdone0, pdone0, bdone1, pdone1;
    wire signed [ACC_W-1:0]    weight0, weight1;

    particle_scorer_pair #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_pair (
        .clk(clk), .rst_n(rst_n),
        .particle_start0(pstart0), .x0_0(px0_0), .y0_0(py0_0),
        .beam_start0(bstart0), .dx0(dx0), .dy0(dy0), .r_obs0(r0), .beam_last0(blast0),
        .beam_done0(bdone0), .particle_done0(pdone0), .weight_o0(weight0),
        .particle_start1(pstart1), .x0_1(px0_1), .y0_1(py0_1),
        .beam_start1(bstart1), .dx1(dx1), .dy1(dy1), .r_obs1(r1), .beam_last1(blast1),
        .beam_done1(bdone1), .particle_done1(pdone1), .weight_o1(weight1)
    );

    reg signed [RM_POS_W-1:0] dxs0 [0:NUM_RAYS-1], dys0 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs0  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy0  [0:1];
    reg signed [ACC_W-1:0]    exp0 [0:0];

    reg signed [RM_POS_W-1:0] dxs1 [0:NUM_RAYS-1], dys1 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs1  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy1  [0:1];
    reg signed [ACC_W-1:0]    exp1 [0:0];

    initial begin
        $readmemh("particle0_dx.hex", dxs0); $readmemh("particle0_dy.hex", dys0);
        $readmemh("particle0_r.hex",  rs0);  $readmemh("particle0_x0y0.hex", xy0);
        $readmemh("particle0_expected.hex", exp0);
        $readmemh("particle1_dx.hex", dxs1); $readmemh("particle1_dy.hex", dys1);
        $readmemh("particle1_r.hex",  rs1);  $readmemh("particle1_x0y0.hex", xy1);
        $readmemh("particle1_expected.hex", exp1);
    end

    integer i0, i1;

    task drive_particle0;
        begin
            px0_0 = xy0[0]; py0_0 = xy0[1];
            @(posedge clk); pstart0 <= 1; @(posedge clk); pstart0 <= 0;
            for (i0 = 0; i0 < NUM_RAYS; i0 = i0 + 1) begin
                @(posedge clk);
                dx0 <= dxs0[i0]; dy0 <= dys0[i0]; r0 <= rs0[i0];
                blast0 <= (i0 == NUM_RAYS-1);
                bstart0 <= 1;
                @(posedge clk); bstart0 <= 0;
                wait (bdone0);
            end
            wait (pdone0);
        end
    endtask

    task drive_particle1;
        begin
            px0_1 = xy1[0]; py0_1 = xy1[1];
            @(posedge clk); pstart1 <= 1; @(posedge clk); pstart1 <= 0;
            for (i1 = 0; i1 < NUM_RAYS; i1 = i1 + 1) begin
                @(posedge clk);
                dx1 <= dxs1[i1]; dy1 <= dys1[i1]; r1 <= rs1[i1];
                blast1 <= (i1 == NUM_RAYS-1);
                bstart1 <= 1;
                @(posedge clk); bstart1 <= 0;
                wait (bdone1);
            end
            wait (pdone1);
        end
    endtask

    real t_start, t_end;

    initial begin
        pstart0=0; bstart0=0; dx0=0; dy0=0; r0=0; blast0=0; px0_0=0; py0_0=0;
        pstart1=0; bstart1=0; dx1=0; dy1=0; r1=0; blast1=0; px0_1=0; py0_1=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        fork
            drive_particle0();
            drive_particle1();
        join
        @(posedge clk);
        t_end = $realtime;

        $display("scorer0: weight=%0d (expected %0d) -> %s", weight0, exp0[0],
                  (weight0 === exp0[0]) ? "PASS" : "FAIL");
        $display("scorer1: weight=%0d (expected %0d) -> %s", weight1, exp1[0],
                  (weight1 === exp1[0]) ? "PASS" : "FAIL");
        $display("총 소요시간: %0.1f ns (테이블을 공유해도 정답+속도가 유지되는지 확인)",
                  t_end - t_start);

        if (weight0 === exp0[0] && weight1 === exp1[0])
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
