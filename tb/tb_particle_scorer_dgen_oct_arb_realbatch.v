// -*- verilog -*-
// tb_particle_scorer_dgen_oct_arb_realbatch.v — particle_scorer_dgen_oct_arb
// (direction_gen 내장 8-way)를 63배치(rf_b0..62_p0..7, 실제 ZERO 파티클
// 500개 재사용)에 대해 순차로 돌려 500파티클 전체의 "진짜" 총 사이클을
// 잰다. tb_particle_scorer_oct_arb_realbatch.v(외부 dx,dy 버전)와 구조
// 동일 — dx,dy,r 대신 theta,r만 배치마다 로드.
//
//   iverilog -o sim/tb_particle_scorer_dgen_oct_arb_realbatch.vvp rtl/ray_march_edt.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/cordic_sincos.v rtl/cordic_sincos_full.v rtl/angle_wrap.v rtl/direction_gen.v rtl/arbiter8.v rtl/particle_scorer_dgen_arb.v rtl/particle_scorer_dgen_oct_arb.v tb/tb_particle_scorer_dgen_oct_arb_realbatch.v
//   cd sim && vvp tb_particle_scorer_dgen_oct_arb_realbatch.vvp

`timescale 1ns/1ps

module tb_particle_scorer_dgen_oct_arb_realbatch;

    localparam RM_POS_W = 18;
    localparam RD_W     = 9;
    localparam ACC_W    = 20;
    localparam ANGLE_W  = 19;
    localparam THETA_RAW_W = 24;
    localparam NUM_RAYS = 60;
    localparam NUM_BATCHES = 63;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        pstart0, pstart1, pstart2, pstart3, pstart4, pstart5, pstart6, pstart7;
    reg  signed [RM_POS_W-1:0] px0_0, py0_0, px0_1, py0_1, px0_2, py0_2, px0_3, py0_3;
    reg  signed [RM_POS_W-1:0] px0_4, py0_4, px0_5, py0_5, px0_6, py0_6, px0_7, py0_7;
    reg  signed [THETA_RAW_W-1:0] th0, th1, th2, th3, th4, th5, th6, th7;
    reg                        bstart0, bstart1, bstart2, bstart3, bstart4, bstart5, bstart6, bstart7;
    reg  [RD_W-1:0]            r0, r1, r2, r3, r4, r5, r6, r7;
    wire                       bdone0, pdone0, bdone1, pdone1, bdone2, pdone2, bdone3, pdone3;
    wire                       bdone4, pdone4, bdone5, pdone5, bdone6, pdone6, bdone7, pdone7;
    wire signed [ACC_W-1:0]    weight0, weight1, weight2, weight3, weight4, weight5, weight6, weight7;

    particle_scorer_dgen_oct_arb #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W), .ANGLE_W(ANGLE_W)) u_oct (
        .clk(clk), .rst_n(rst_n),
        .particle_start0(pstart0), .x0_0(px0_0), .y0_0(py0_0), .theta_raw0(th0),
        .beam_start0(bstart0), .r_obs0(r0),
        .beam_done0(bdone0), .particle_done0(pdone0), .weight_o0(weight0),
        .particle_start1(pstart1), .x0_1(px0_1), .y0_1(py0_1), .theta_raw1(th1),
        .beam_start1(bstart1), .r_obs1(r1),
        .beam_done1(bdone1), .particle_done1(pdone1), .weight_o1(weight1),
        .particle_start2(pstart2), .x0_2(px0_2), .y0_2(py0_2), .theta_raw2(th2),
        .beam_start2(bstart2), .r_obs2(r2),
        .beam_done2(bdone2), .particle_done2(pdone2), .weight_o2(weight2),
        .particle_start3(pstart3), .x0_3(px0_3), .y0_3(py0_3), .theta_raw3(th3),
        .beam_start3(bstart3), .r_obs3(r3),
        .beam_done3(bdone3), .particle_done3(pdone3), .weight_o3(weight3),
        .particle_start4(pstart4), .x0_4(px0_4), .y0_4(py0_4), .theta_raw4(th4),
        .beam_start4(bstart4), .r_obs4(r4),
        .beam_done4(bdone4), .particle_done4(pdone4), .weight_o4(weight4),
        .particle_start5(pstart5), .x0_5(px0_5), .y0_5(py0_5), .theta_raw5(th5),
        .beam_start5(bstart5), .r_obs5(r5),
        .beam_done5(bdone5), .particle_done5(pdone5), .weight_o5(weight5),
        .particle_start6(pstart6), .x0_6(px0_6), .y0_6(py0_6), .theta_raw6(th6),
        .beam_start6(bstart6), .r_obs6(r6),
        .beam_done6(bdone6), .particle_done6(pdone6), .weight_o6(weight6),
        .particle_start7(pstart7), .x0_7(px0_7), .y0_7(py0_7), .theta_raw7(th7),
        .beam_start7(bstart7), .r_obs7(r7),
        .beam_done7(bdone7), .particle_done7(pdone7), .weight_o7(weight7)
    );

    reg [RD_W-1:0] rs0 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy0 [0:1]; reg signed [THETA_RAW_W-1:0] thv0 [0:0];
    reg [RD_W-1:0] rs1 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy1 [0:1]; reg signed [THETA_RAW_W-1:0] thv1 [0:0];
    reg [RD_W-1:0] rs2 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy2 [0:1]; reg signed [THETA_RAW_W-1:0] thv2 [0:0];
    reg [RD_W-1:0] rs3 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy3 [0:1]; reg signed [THETA_RAW_W-1:0] thv3 [0:0];
    reg [RD_W-1:0] rs4 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy4 [0:1]; reg signed [THETA_RAW_W-1:0] thv4 [0:0];
    reg [RD_W-1:0] rs5 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy5 [0:1]; reg signed [THETA_RAW_W-1:0] thv5 [0:0];
    reg [RD_W-1:0] rs6 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy6 [0:1]; reg signed [THETA_RAW_W-1:0] thv6 [0:0];
    reg [RD_W-1:0] rs7 [0:NUM_RAYS-1]; reg signed [RM_POS_W-1:0] xy7 [0:1]; reg signed [THETA_RAW_W-1:0] thv7 [0:0];

    reg [256*8-1:0] fname;
    integer batch;

    task load_batch(input integer b); begin
        $sformat(fname, "rf_b%0d_p0_r.hex",  b); $readmemh(fname, rs0);
        $sformat(fname, "rf_b%0d_p0_x0y0.hex", b); $readmemh(fname, xy0);
        $sformat(fname, "rf_b%0d_p0_theta_raw.hex", b); $readmemh(fname, thv0);
        $sformat(fname, "rf_b%0d_p1_r.hex",  b); $readmemh(fname, rs1);
        $sformat(fname, "rf_b%0d_p1_x0y0.hex", b); $readmemh(fname, xy1);
        $sformat(fname, "rf_b%0d_p1_theta_raw.hex", b); $readmemh(fname, thv1);
        $sformat(fname, "rf_b%0d_p2_r.hex",  b); $readmemh(fname, rs2);
        $sformat(fname, "rf_b%0d_p2_x0y0.hex", b); $readmemh(fname, xy2);
        $sformat(fname, "rf_b%0d_p2_theta_raw.hex", b); $readmemh(fname, thv2);
        $sformat(fname, "rf_b%0d_p3_r.hex",  b); $readmemh(fname, rs3);
        $sformat(fname, "rf_b%0d_p3_x0y0.hex", b); $readmemh(fname, xy3);
        $sformat(fname, "rf_b%0d_p3_theta_raw.hex", b); $readmemh(fname, thv3);
        $sformat(fname, "rf_b%0d_p4_r.hex",  b); $readmemh(fname, rs4);
        $sformat(fname, "rf_b%0d_p4_x0y0.hex", b); $readmemh(fname, xy4);
        $sformat(fname, "rf_b%0d_p4_theta_raw.hex", b); $readmemh(fname, thv4);
        $sformat(fname, "rf_b%0d_p5_r.hex",  b); $readmemh(fname, rs5);
        $sformat(fname, "rf_b%0d_p5_x0y0.hex", b); $readmemh(fname, xy5);
        $sformat(fname, "rf_b%0d_p5_theta_raw.hex", b); $readmemh(fname, thv5);
        $sformat(fname, "rf_b%0d_p6_r.hex",  b); $readmemh(fname, rs6);
        $sformat(fname, "rf_b%0d_p6_x0y0.hex", b); $readmemh(fname, xy6);
        $sformat(fname, "rf_b%0d_p6_theta_raw.hex", b); $readmemh(fname, thv6);
        $sformat(fname, "rf_b%0d_p7_r.hex",  b); $readmemh(fname, rs7);
        $sformat(fname, "rf_b%0d_p7_x0y0.hex", b); $readmemh(fname, xy7);
        $sformat(fname, "rf_b%0d_p7_theta_raw.hex", b); $readmemh(fname, thv7);
    end endtask

    integer i0, i1, i2, i3, i4, i5, i6, i7;

    task drive_particle0; begin px0_0=xy0[0]; py0_0=xy0[1]; th0=thv0[0]; @(posedge clk); pstart0<=1; @(posedge clk); pstart0<=0;
        for (i0=0;i0<NUM_RAYS;i0=i0+1) begin @(posedge clk); r0<=rs0[i0]; bstart0<=1; @(posedge clk); bstart0<=0; wait(bdone0); end
        wait(pdone0); end endtask
    task drive_particle1; begin px0_1=xy1[0]; py0_1=xy1[1]; th1=thv1[0]; @(posedge clk); pstart1<=1; @(posedge clk); pstart1<=0;
        for (i1=0;i1<NUM_RAYS;i1=i1+1) begin @(posedge clk); r1<=rs1[i1]; bstart1<=1; @(posedge clk); bstart1<=0; wait(bdone1); end
        wait(pdone1); end endtask
    task drive_particle2; begin px0_2=xy2[0]; py0_2=xy2[1]; th2=thv2[0]; @(posedge clk); pstart2<=1; @(posedge clk); pstart2<=0;
        for (i2=0;i2<NUM_RAYS;i2=i2+1) begin @(posedge clk); r2<=rs2[i2]; bstart2<=1; @(posedge clk); bstart2<=0; wait(bdone2); end
        wait(pdone2); end endtask
    task drive_particle3; begin px0_3=xy3[0]; py0_3=xy3[1]; th3=thv3[0]; @(posedge clk); pstart3<=1; @(posedge clk); pstart3<=0;
        for (i3=0;i3<NUM_RAYS;i3=i3+1) begin @(posedge clk); r3<=rs3[i3]; bstart3<=1; @(posedge clk); bstart3<=0; wait(bdone3); end
        wait(pdone3); end endtask
    task drive_particle4; begin px0_4=xy4[0]; py0_4=xy4[1]; th4=thv4[0]; @(posedge clk); pstart4<=1; @(posedge clk); pstart4<=0;
        for (i4=0;i4<NUM_RAYS;i4=i4+1) begin @(posedge clk); r4<=rs4[i4]; bstart4<=1; @(posedge clk); bstart4<=0; wait(bdone4); end
        wait(pdone4); end endtask
    task drive_particle5; begin px0_5=xy5[0]; py0_5=xy5[1]; th5=thv5[0]; @(posedge clk); pstart5<=1; @(posedge clk); pstart5<=0;
        for (i5=0;i5<NUM_RAYS;i5=i5+1) begin @(posedge clk); r5<=rs5[i5]; bstart5<=1; @(posedge clk); bstart5<=0; wait(bdone5); end
        wait(pdone5); end endtask
    task drive_particle6; begin px0_6=xy6[0]; py0_6=xy6[1]; th6=thv6[0]; @(posedge clk); pstart6<=1; @(posedge clk); pstart6<=0;
        for (i6=0;i6<NUM_RAYS;i6=i6+1) begin @(posedge clk); r6<=rs6[i6]; bstart6<=1; @(posedge clk); bstart6<=0; wait(bdone6); end
        wait(pdone6); end endtask
    task drive_particle7; begin px0_7=xy7[0]; py0_7=xy7[1]; th7=thv7[0]; @(posedge clk); pstart7<=1; @(posedge clk); pstart7<=0;
        for (i7=0;i7<NUM_RAYS;i7=i7+1) begin @(posedge clk); r7<=rs7[i7]; bstart7<=1; @(posedge clk); bstart7<=0; wait(bdone7); end
        wait(pdone7); end endtask

    real t_start, t_end;
    integer x_count;
    integer fails_sanity;

    initial begin
        pstart0=0;bstart0=0;r0=0;px0_0=0;py0_0=0;th0=0;
        pstart1=0;bstart1=0;r1=0;px0_1=0;py0_1=0;th1=0;
        pstart2=0;bstart2=0;r2=0;px0_2=0;py0_2=0;th2=0;
        pstart3=0;bstart3=0;r3=0;px0_3=0;py0_3=0;th3=0;
        pstart4=0;bstart4=0;r4=0;px0_4=0;py0_4=0;th4=0;
        pstart5=0;bstart5=0;r5=0;px0_5=0;py0_5=0;th5=0;
        pstart6=0;bstart6=0;r6=0;px0_6=0;py0_6=0;th6=0;
        pstart7=0;bstart7=0;r7=0;px0_7=0;py0_7=0;th7=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        x_count = 0;
        fails_sanity = 0;
        t_start = $realtime;
        for (batch = 0; batch < NUM_BATCHES; batch = batch + 1) begin
            load_batch(batch);
            fork
                drive_particle0(); drive_particle1(); drive_particle2(); drive_particle3();
                drive_particle4(); drive_particle5(); drive_particle6(); drive_particle7();
            join
            if (^weight0 === 1'bx) x_count = x_count + 1;
            if (batch == 0) begin
                // bench/cordic_recompute_001.npz의 파티클0~7 기대값(gen_dgen_accuracy_test.py,
                // RTL의 CORDIC을 비트단위로 재현한 파이썬 모델) — theta_wrap 통합 후에도
                // 정확성이 유지되는지 8개 다(감기 필요없는 것/필요한 것 섞어서) 확인.
                if (weight0 !== -46170) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p0: %0d (exp -46170)", weight0); end
                if (weight1 !== -50615) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p1: %0d (exp -50615)", weight1); end
                if (weight2 !== -50388) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p2: %0d (exp -50388)", weight2); end
                if (weight3 !== -46390) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p3: %0d (exp -46390)", weight3); end
                if (weight4 !== -50044) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p4: %0d (exp -50044)", weight4); end
                if (weight5 !== -49927) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p5: %0d (exp -49927)", weight5); end
                if (weight6 !== -48818) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p6: %0d (exp -48818)", weight6); end
                if (weight7 !== -50209) begin fails_sanity=fails_sanity+1; $display("SANITY FAIL p7: %0d (exp -50209)", weight7); end
                if (fails_sanity == 0) $display("SANITY: 배치0 파티클0~7 전부 CORDIC 파이썬 재현값과 정확히 일치(theta_wrap 포함)");
            end
        end
        @(posedge clk);
        t_end = $realtime;

        $display("배치 수: %0d (파티클 %0d개, direction_gen 내장 8-way 겹침)", NUM_BATCHES, NUM_BATCHES*8);
        $display("총 소요시간: %0.1f ns = %0d 사이클 @ 10ns", t_end - t_start, (t_end - t_start) / 10);
        $display("미확정(x) weight 배치 수: %0d", x_count);
        if (x_count == 0) $display("ALL BATCHES CLEAN (no X)");
        else $display("WARNING: X detected in %0d batches", x_count);
        $finish;
    end

endmodule
