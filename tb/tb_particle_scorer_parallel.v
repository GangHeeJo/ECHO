// -*- verilog -*-
// tb_particle_scorer_parallel.v — particle_scorer 2개를 동시에 돌려서 파티클 2개를
// "레이마칭+센서모델 전체 파이프라인"째로 병렬 처리하는지 검증한다.
// particle_scorer.v가 이미 내부에 필요한 걸 다 갖춘 독립 블록이라, 추가 배선
// 거의 없이 2개 나란히 놓기만 하면 됨 — v1(sensor_pe)때의 병렬화 패턴 그대로.
//
// task 재진입성 교훈(tb_sensor_pe_x4.v에서 겪은 무한루프) 반영: 같은 task를
// fork로 동시에 두 번 부르지 않고, 처음부터 서로 다른 task로 나눔(재진입 문제
// 자체가 생길 여지가 없음).
//
//   iverilog -o sim/tb_particle_scorer_parallel.vvp rtl/ray_march_bram.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer.v tb/tb_particle_scorer_parallel.v
//   cd sim && vvp tb_particle_scorer_parallel.vvp

`timescale 1ns/1ps

module tb_particle_scorer_parallel;

    localparam RM_POS_W  = 18;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // particle_scorer 0
    reg                        pstart0;
    reg  signed [RM_POS_W-1:0] px0_0, py0_0;
    reg                        bstart0;
    reg  signed [RM_POS_W-1:0] dx0, dy0;
    reg  [RD_W-1:0]            r0;
    reg                        blast0;
    wire                       bdone0, pdone0;
    wire signed [ACC_W-1:0]    weight0;

    particle_scorer #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_scorer0 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(pstart0), .x0(px0_0), .y0(py0_0),
        .beam_start(bstart0), .dx(dx0), .dy(dy0), .r_obs(r0), .beam_last(blast0),
        .beam_done(bdone0), .particle_done(pdone0), .weight_o(weight0)
    );

    // particle_scorer 1
    reg                        pstart1;
    reg  signed [RM_POS_W-1:0] px0_1, py0_1;
    reg                        bstart1;
    reg  signed [RM_POS_W-1:0] dx1, dy1;
    reg  [RD_W-1:0]            r1;
    reg                        blast1;
    wire                       bdone1, pdone1;
    wire signed [ACC_W-1:0]    weight1;

    particle_scorer #(.RM_POS_W(RM_POS_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_scorer1 (
        .clk(clk), .rst_n(rst_n),
        .particle_start(pstart1), .x0(px0_1), .y0(py0_1),
        .beam_start(bstart1), .dx(dx1), .dy(dy1), .r_obs(r1), .beam_last(blast1),
        .beam_done(bdone1), .particle_done(pdone1), .weight_o(weight1)
    );

    // echo-ref/gen_particle_scorer_test.py 산출물
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

    initial begin
        $dumpfile("tb_particle_scorer_parallel.vcd");
        $dumpvars(0, tb_particle_scorer_parallel);
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
        $display("총 소요시간: %0.1f ns (혼자 처리할 때(단일 particle_scorer)와 비슷해야 '진짜 병렬'임)",
                  t_end - t_start);

        if (weight0 === exp0[0] && weight1 === exp1[0])
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
