// -*- verilog -*-
// tb_particle_scorer_oct_arb.v — particle_scorer_oct_arb(arbiter8로 8파티클이
// table_mem 하나 시분할) 기능 검증. particle0~7 정답지 그대로 사용.
// (2D 메모리 슬라이스로 $readmemh를 돌리려다 Icarus가 "needs 2 indices"로
// 거부해서, pair/quad와 같은 완전 명시 스타일로 되돌림 — 장황하지만 확실히 됨.)
//
//   iverilog -o sim/tb_particle_scorer_oct_arb.vvp rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/arbiter8.v rtl/particle_scorer_arb.v rtl/particle_scorer_oct_arb.v tb/tb_particle_scorer_oct_arb.v
//   cd sim && vvp tb_particle_scorer_oct_arb.vvp

`timescale 1ns/1ps

module tb_particle_scorer_oct_arb;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        pstart0, pstart1, pstart2, pstart3, pstart4, pstart5, pstart6, pstart7;
    reg  signed [RM_POS_W-1:0] px0_0, py0_0, px0_1, py0_1, px0_2, py0_2, px0_3, py0_3;
    reg  signed [RM_POS_W-1:0] px0_4, py0_4, px0_5, py0_5, px0_6, py0_6, px0_7, py0_7;
    reg                        bstart0, bstart1, bstart2, bstart3, bstart4, bstart5, bstart6, bstart7;
    reg  signed [RM_POS_W-1:0] dx0, dy0, dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4, dx5, dy5, dx6, dy6, dx7, dy7;
    reg  [RD_W-1:0]            r0, r1, r2, r3, r4, r5, r6, r7;
    reg                        blast0, blast1, blast2, blast3, blast4, blast5, blast6, blast7;
    wire                       bdone0, pdone0, bdone1, pdone1, bdone2, pdone2, bdone3, pdone3;
    wire                       bdone4, pdone4, bdone5, pdone5, bdone6, pdone6, bdone7, pdone7;
    wire signed [ACC_W-1:0]    weight0, weight1, weight2, weight3, weight4, weight5, weight6, weight7;

    particle_scorer_oct_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_oct (
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
        .beam_done3(bdone3), .particle_done3(pdone3), .weight_o3(weight3),
        .particle_start4(pstart4), .x0_4(px0_4), .y0_4(py0_4),
        .beam_start4(bstart4), .dx4(dx4), .dy4(dy4), .r_obs4(r4), .beam_last4(blast4),
        .beam_done4(bdone4), .particle_done4(pdone4), .weight_o4(weight4),
        .particle_start5(pstart5), .x0_5(px0_5), .y0_5(py0_5),
        .beam_start5(bstart5), .dx5(dx5), .dy5(dy5), .r_obs5(r5), .beam_last5(blast5),
        .beam_done5(bdone5), .particle_done5(pdone5), .weight_o5(weight5),
        .particle_start6(pstart6), .x0_6(px0_6), .y0_6(py0_6),
        .beam_start6(bstart6), .dx6(dx6), .dy6(dy6), .r_obs6(r6), .beam_last6(blast6),
        .beam_done6(bdone6), .particle_done6(pdone6), .weight_o6(weight6),
        .particle_start7(pstart7), .x0_7(px0_7), .y0_7(py0_7),
        .beam_start7(bstart7), .dx7(dx7), .dy7(dy7), .r_obs7(r7), .beam_last7(blast7),
        .beam_done7(bdone7), .particle_done7(pdone7), .weight_o7(weight7)
    );

    reg signed [RM_POS_W-1:0] dxs0 [0:NUM_RAYS-1], dys0 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs0 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy0 [0:1]; reg signed [ACC_W-1:0] exp0 [0:0];
    reg signed [RM_POS_W-1:0] dxs1 [0:NUM_RAYS-1], dys1 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs1 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy1 [0:1]; reg signed [ACC_W-1:0] exp1 [0:0];
    reg signed [RM_POS_W-1:0] dxs2 [0:NUM_RAYS-1], dys2 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs2 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy2 [0:1]; reg signed [ACC_W-1:0] exp2 [0:0];
    reg signed [RM_POS_W-1:0] dxs3 [0:NUM_RAYS-1], dys3 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs3 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy3 [0:1]; reg signed [ACC_W-1:0] exp3 [0:0];
    reg signed [RM_POS_W-1:0] dxs4 [0:NUM_RAYS-1], dys4 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs4 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy4 [0:1]; reg signed [ACC_W-1:0] exp4 [0:0];
    reg signed [RM_POS_W-1:0] dxs5 [0:NUM_RAYS-1], dys5 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs5 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy5 [0:1]; reg signed [ACC_W-1:0] exp5 [0:0];
    reg signed [RM_POS_W-1:0] dxs6 [0:NUM_RAYS-1], dys6 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs6 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy6 [0:1]; reg signed [ACC_W-1:0] exp6 [0:0];
    reg signed [RM_POS_W-1:0] dxs7 [0:NUM_RAYS-1], dys7 [0:NUM_RAYS-1]; reg [RD_W-1:0] rs7 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy7 [0:1]; reg signed [ACC_W-1:0] exp7 [0:0];

    initial begin
        $readmemh("particle0_dx.hex", dxs0); $readmemh("particle0_dy.hex", dys0); $readmemh("particle0_r.hex", rs0); $readmemh("particle0_x0y0.hex", xy0); $readmemh("particle0_expected.hex", exp0);
        $readmemh("particle1_dx.hex", dxs1); $readmemh("particle1_dy.hex", dys1); $readmemh("particle1_r.hex", rs1); $readmemh("particle1_x0y0.hex", xy1); $readmemh("particle1_expected.hex", exp1);
        $readmemh("particle2_dx.hex", dxs2); $readmemh("particle2_dy.hex", dys2); $readmemh("particle2_r.hex", rs2); $readmemh("particle2_x0y0.hex", xy2); $readmemh("particle2_expected.hex", exp2);
        $readmemh("particle3_dx.hex", dxs3); $readmemh("particle3_dy.hex", dys3); $readmemh("particle3_r.hex", rs3); $readmemh("particle3_x0y0.hex", xy3); $readmemh("particle3_expected.hex", exp3);
        $readmemh("particle4_dx.hex", dxs4); $readmemh("particle4_dy.hex", dys4); $readmemh("particle4_r.hex", rs4); $readmemh("particle4_x0y0.hex", xy4); $readmemh("particle4_expected.hex", exp4);
        $readmemh("particle5_dx.hex", dxs5); $readmemh("particle5_dy.hex", dys5); $readmemh("particle5_r.hex", rs5); $readmemh("particle5_x0y0.hex", xy5); $readmemh("particle5_expected.hex", exp5);
        $readmemh("particle6_dx.hex", dxs6); $readmemh("particle6_dy.hex", dys6); $readmemh("particle6_r.hex", rs6); $readmemh("particle6_x0y0.hex", xy6); $readmemh("particle6_expected.hex", exp6);
        $readmemh("particle7_dx.hex", dxs7); $readmemh("particle7_dy.hex", dys7); $readmemh("particle7_r.hex", rs7); $readmemh("particle7_x0y0.hex", xy7); $readmemh("particle7_expected.hex", exp7);
    end

    integer i0, i1, i2, i3, i4, i5, i6, i7;

    task drive_particle0; begin px0_0=xy0[0]; py0_0=xy0[1]; @(posedge clk); pstart0<=1; @(posedge clk); pstart0<=0;
        for (i0=0;i0<NUM_RAYS;i0=i0+1) begin @(posedge clk); dx0<=dxs0[i0]; dy0<=dys0[i0]; r0<=rs0[i0]; blast0<=(i0==NUM_RAYS-1); bstart0<=1; @(posedge clk); bstart0<=0; wait(bdone0); end
        wait(pdone0); end endtask
    task drive_particle1; begin px0_1=xy1[0]; py0_1=xy1[1]; @(posedge clk); pstart1<=1; @(posedge clk); pstart1<=0;
        for (i1=0;i1<NUM_RAYS;i1=i1+1) begin @(posedge clk); dx1<=dxs1[i1]; dy1<=dys1[i1]; r1<=rs1[i1]; blast1<=(i1==NUM_RAYS-1); bstart1<=1; @(posedge clk); bstart1<=0; wait(bdone1); end
        wait(pdone1); end endtask
    task drive_particle2; begin px0_2=xy2[0]; py0_2=xy2[1]; @(posedge clk); pstart2<=1; @(posedge clk); pstart2<=0;
        for (i2=0;i2<NUM_RAYS;i2=i2+1) begin @(posedge clk); dx2<=dxs2[i2]; dy2<=dys2[i2]; r2<=rs2[i2]; blast2<=(i2==NUM_RAYS-1); bstart2<=1; @(posedge clk); bstart2<=0; wait(bdone2); end
        wait(pdone2); end endtask
    task drive_particle3; begin px0_3=xy3[0]; py0_3=xy3[1]; @(posedge clk); pstart3<=1; @(posedge clk); pstart3<=0;
        for (i3=0;i3<NUM_RAYS;i3=i3+1) begin @(posedge clk); dx3<=dxs3[i3]; dy3<=dys3[i3]; r3<=rs3[i3]; blast3<=(i3==NUM_RAYS-1); bstart3<=1; @(posedge clk); bstart3<=0; wait(bdone3); end
        wait(pdone3); end endtask
    task drive_particle4; begin px0_4=xy4[0]; py0_4=xy4[1]; @(posedge clk); pstart4<=1; @(posedge clk); pstart4<=0;
        for (i4=0;i4<NUM_RAYS;i4=i4+1) begin @(posedge clk); dx4<=dxs4[i4]; dy4<=dys4[i4]; r4<=rs4[i4]; blast4<=(i4==NUM_RAYS-1); bstart4<=1; @(posedge clk); bstart4<=0; wait(bdone4); end
        wait(pdone4); end endtask
    task drive_particle5; begin px0_5=xy5[0]; py0_5=xy5[1]; @(posedge clk); pstart5<=1; @(posedge clk); pstart5<=0;
        for (i5=0;i5<NUM_RAYS;i5=i5+1) begin @(posedge clk); dx5<=dxs5[i5]; dy5<=dys5[i5]; r5<=rs5[i5]; blast5<=(i5==NUM_RAYS-1); bstart5<=1; @(posedge clk); bstart5<=0; wait(bdone5); end
        wait(pdone5); end endtask
    task drive_particle6; begin px0_6=xy6[0]; py0_6=xy6[1]; @(posedge clk); pstart6<=1; @(posedge clk); pstart6<=0;
        for (i6=0;i6<NUM_RAYS;i6=i6+1) begin @(posedge clk); dx6<=dxs6[i6]; dy6<=dys6[i6]; r6<=rs6[i6]; blast6<=(i6==NUM_RAYS-1); bstart6<=1; @(posedge clk); bstart6<=0; wait(bdone6); end
        wait(pdone6); end endtask
    task drive_particle7; begin px0_7=xy7[0]; py0_7=xy7[1]; @(posedge clk); pstart7<=1; @(posedge clk); pstart7<=0;
        for (i7=0;i7<NUM_RAYS;i7=i7+1) begin @(posedge clk); dx7<=dxs7[i7]; dy7<=dys7[i7]; r7<=rs7[i7]; blast7<=(i7==NUM_RAYS-1); bstart7<=1; @(posedge clk); bstart7<=0; wait(bdone7); end
        wait(pdone7); end endtask

    real t_start, t_end;
    integer fails;

    initial begin
        pstart0=0;bstart0=0;dx0=0;dy0=0;r0=0;blast0=0;px0_0=0;py0_0=0;
        pstart1=0;bstart1=0;dx1=0;dy1=0;r1=0;blast1=0;px0_1=0;py0_1=0;
        pstart2=0;bstart2=0;dx2=0;dy2=0;r2=0;blast2=0;px0_2=0;py0_2=0;
        pstart3=0;bstart3=0;dx3=0;dy3=0;r3=0;blast3=0;px0_3=0;py0_3=0;
        pstart4=0;bstart4=0;dx4=0;dy4=0;r4=0;blast4=0;px0_4=0;py0_4=0;
        pstart5=0;bstart5=0;dx5=0;dy5=0;r5=0;blast5=0;px0_5=0;py0_5=0;
        pstart6=0;bstart6=0;dx6=0;dy6=0;r6=0;blast6=0;px0_6=0;py0_6=0;
        pstart7=0;bstart7=0;dx7=0;dy7=0;r7=0;blast7=0;px0_7=0;py0_7=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        fork
            drive_particle0(); drive_particle1(); drive_particle2(); drive_particle3();
            drive_particle4(); drive_particle5(); drive_particle6(); drive_particle7();
        join
        @(posedge clk);
        t_end = $realtime;

        fails = 0;
        if (weight0 === exp0[0]) $display("scorer0: weight=%0d (expected %0d) -> PASS", weight0, exp0[0]); else begin $display("scorer0: weight=%0d (expected %0d) -> FAIL", weight0, exp0[0]); fails=fails+1; end
        if (weight1 === exp1[0]) $display("scorer1: weight=%0d (expected %0d) -> PASS", weight1, exp1[0]); else begin $display("scorer1: weight=%0d (expected %0d) -> FAIL", weight1, exp1[0]); fails=fails+1; end
        if (weight2 === exp2[0]) $display("scorer2: weight=%0d (expected %0d) -> PASS", weight2, exp2[0]); else begin $display("scorer2: weight=%0d (expected %0d) -> FAIL", weight2, exp2[0]); fails=fails+1; end
        if (weight3 === exp3[0]) $display("scorer3: weight=%0d (expected %0d) -> PASS", weight3, exp3[0]); else begin $display("scorer3: weight=%0d (expected %0d) -> FAIL", weight3, exp3[0]); fails=fails+1; end
        if (weight4 === exp4[0]) $display("scorer4: weight=%0d (expected %0d) -> PASS", weight4, exp4[0]); else begin $display("scorer4: weight=%0d (expected %0d) -> FAIL", weight4, exp4[0]); fails=fails+1; end
        if (weight5 === exp5[0]) $display("scorer5: weight=%0d (expected %0d) -> PASS", weight5, exp5[0]); else begin $display("scorer5: weight=%0d (expected %0d) -> FAIL", weight5, exp5[0]); fails=fails+1; end
        if (weight6 === exp6[0]) $display("scorer6: weight=%0d (expected %0d) -> PASS", weight6, exp6[0]); else begin $display("scorer6: weight=%0d (expected %0d) -> FAIL", weight6, exp6[0]); fails=fails+1; end
        if (weight7 === exp7[0]) $display("scorer7: weight=%0d (expected %0d) -> PASS", weight7, exp7[0]); else begin $display("scorer7: weight=%0d (expected %0d) -> FAIL", weight7, exp7[0]); fails=fails+1; end

        $display("총 소요시간: %0.1f ns (테이블 포트 8-way 시분할 - 얼마나 느려지는지 확인)", t_end - t_start);

        if (fails == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
