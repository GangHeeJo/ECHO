// -*- verilog -*-
// tb_particle_scorer_quad_arb.v — particle_scorer_quad_arb(arbiter4로 4파티클이
// table_mem 하나 시분할) 기능 검증. particle0~3 정답지(gen_particle_scorer_test.py
// 산출물, 이미 sim/에 있음) 그대로 사용.
//
//   iverilog -o sim/tb_particle_scorer_quad_arb.vvp rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter4.v rtl/particle_scorer_arb.v rtl/particle_scorer_quad_arb.v tb/tb_particle_scorer_quad_arb.v
//   cd sim && vvp tb_particle_scorer_quad_arb.vvp

`timescale 1ns/1ps

module tb_particle_scorer_quad_arb;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        pstart0, pstart1, pstart2, pstart3;
    reg  signed [RM_POS_W-1:0] px0_0, py0_0, px0_1, py0_1, px0_2, py0_2, px0_3, py0_3;
    reg                        bstart0, bstart1, bstart2, bstart3;
    reg  signed [RM_POS_W-1:0] dx0, dy0, dx1, dy1, dx2, dy2, dx3, dy3;
    reg  [RD_W-1:0]            r0, r1, r2, r3;
    reg                        blast0, blast1, blast2, blast3;
    wire                       bdone0, pdone0, bdone1, pdone1, bdone2, pdone2, bdone3, pdone3;
    wire signed [ACC_W-1:0]    weight0, weight1, weight2, weight3;

    particle_scorer_quad_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_quad (
        .clk(clk), .rst_n(rst_n),
        .particle_start0(pstart0), .x0_0(px0_0), .y0_0(py0_0),
        .beam_start0(bstart0), .dx0(dx0), .dy0(dy0), .r_obs0(r0), .beam_last0(blast0),
        .beam_done0(bdone0), .particle_done0(pdone0), .weight_o0(weight0),
        .particle_start1(pstart1), .x0_1(px0_1), .y0_1(py0_1),
        .beam_start1(bstart1), .dx1(dx1), .dy1(dy1), .r_obs1(r1), .beam_last1(blast1),
        .beam_done1(bdone1), .particle_done1(pdone1), .weight_o1(weight1),
        .particle_start2(pstart2), .x0_2(px0_2), .y0_2(py0_2),
        .beam_start2(bstart2), .dx2(dx2), .dy2(dy2), .r_obs2(r2), .beam_last2(blast2),
        .beam_done2(bdone2), .particle_done2(pdone2), .weight_o2(weight2),
        .particle_start3(pstart3), .x0_3(px0_3), .y0_3(py0_3),
        .beam_start3(bstart3), .dx3(dx3), .dy3(dy3), .r_obs3(r3), .beam_last3(blast3),
        .beam_done3(bdone3), .particle_done3(pdone3), .weight_o3(weight3)
    );

    reg signed [RM_POS_W-1:0] dxs0 [0:NUM_RAYS-1], dys0 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs0  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy0  [0:1];
    reg signed [ACC_W-1:0]    exp0 [0:0];

    reg signed [RM_POS_W-1:0] dxs1 [0:NUM_RAYS-1], dys1 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs1  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy1  [0:1];
    reg signed [ACC_W-1:0]    exp1 [0:0];

    reg signed [RM_POS_W-1:0] dxs2 [0:NUM_RAYS-1], dys2 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs2  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy2  [0:1];
    reg signed [ACC_W-1:0]    exp2 [0:0];

    reg signed [RM_POS_W-1:0] dxs3 [0:NUM_RAYS-1], dys3 [0:NUM_RAYS-1];
    reg [RD_W-1:0]            rs3  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] xy3  [0:1];
    reg signed [ACC_W-1:0]    exp3 [0:0];

    initial begin
        $readmemh("particle0_dx.hex", dxs0); $readmemh("particle0_dy.hex", dys0);
        $readmemh("particle0_r.hex",  rs0);  $readmemh("particle0_x0y0.hex", xy0);
        $readmemh("particle0_expected.hex", exp0);
        $readmemh("particle1_dx.hex", dxs1); $readmemh("particle1_dy.hex", dys1);
        $readmemh("particle1_r.hex",  rs1);  $readmemh("particle1_x0y0.hex", xy1);
        $readmemh("particle1_expected.hex", exp1);
        $readmemh("particle2_dx.hex", dxs2); $readmemh("particle2_dy.hex", dys2);
        $readmemh("particle2_r.hex",  rs2);  $readmemh("particle2_x0y0.hex", xy2);
        $readmemh("particle2_expected.hex", exp2);
        $readmemh("particle3_dx.hex", dxs3); $readmemh("particle3_dy.hex", dys3);
        $readmemh("particle3_r.hex",  rs3);  $readmemh("particle3_x0y0.hex", xy3);
        $readmemh("particle3_expected.hex", exp3);
    end

    integer i0, i1, i2, i3;

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

    task drive_particle2;
        begin
            px0_2 = xy2[0]; py0_2 = xy2[1];
            @(posedge clk); pstart2 <= 1; @(posedge clk); pstart2 <= 0;
            for (i2 = 0; i2 < NUM_RAYS; i2 = i2 + 1) begin
                @(posedge clk);
                dx2 <= dxs2[i2]; dy2 <= dys2[i2]; r2 <= rs2[i2];
                blast2 <= (i2 == NUM_RAYS-1);
                bstart2 <= 1;
                @(posedge clk); bstart2 <= 0;
                wait (bdone2);
            end
            wait (pdone2);
        end
    endtask

    task drive_particle3;
        begin
            px0_3 = xy3[0]; py0_3 = xy3[1];
            @(posedge clk); pstart3 <= 1; @(posedge clk); pstart3 <= 0;
            for (i3 = 0; i3 < NUM_RAYS; i3 = i3 + 1) begin
                @(posedge clk);
                dx3 <= dxs3[i3]; dy3 <= dys3[i3]; r3 <= rs3[i3];
                blast3 <= (i3 == NUM_RAYS-1);
                bstart3 <= 1;
                @(posedge clk); bstart3 <= 0;
                wait (bdone3);
            end
            wait (pdone3);
        end
    endtask

    real t_start, t_end;
    integer fails;

    initial begin
        pstart0=0; bstart0=0; dx0=0; dy0=0; r0=0; blast0=0; px0_0=0; py0_0=0;
        pstart1=0; bstart1=0; dx1=0; dy1=0; r1=0; blast1=0; px0_1=0; py0_1=0;
        pstart2=0; bstart2=0; dx2=0; dy2=0; r2=0; blast2=0; px0_2=0; py0_2=0;
        pstart3=0; bstart3=0; dx3=0; dy3=0; r3=0; blast3=0; px0_3=0; py0_3=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        fork
            drive_particle0();
            drive_particle1();
            drive_particle2();
            drive_particle3();
        join
        @(posedge clk);
        t_end = $realtime;

        fails = 0;
        if (weight0 === exp0[0]) $display("scorer0: weight=%0d (expected %0d) -> PASS", weight0, exp0[0]);
        else begin $display("scorer0: weight=%0d (expected %0d) -> FAIL", weight0, exp0[0]); fails = fails + 1; end
        if (weight1 === exp1[0]) $display("scorer1: weight=%0d (expected %0d) -> PASS", weight1, exp1[0]);
        else begin $display("scorer1: weight=%0d (expected %0d) -> FAIL", weight1, exp1[0]); fails = fails + 1; end
        if (weight2 === exp2[0]) $display("scorer2: weight=%0d (expected %0d) -> PASS", weight2, exp2[0]);
        else begin $display("scorer2: weight=%0d (expected %0d) -> FAIL", weight2, exp2[0]); fails = fails + 1; end
        if (weight3 === exp3[0]) $display("scorer3: weight=%0d (expected %0d) -> PASS", weight3, exp3[0]);
        else begin $display("scorer3: weight=%0d (expected %0d) -> FAIL", weight3, exp3[0]); fails = fails + 1; end

        $display("총 소요시간: %0.1f ns (테이블 포트 4-way 시분할 - 얼마나 느려지는지 확인)",
                  t_end - t_start);

        if (fails == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
