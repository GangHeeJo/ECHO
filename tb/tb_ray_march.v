// -*- verilog -*-
// tb_ray_march.v — ray_march.v를 손으로 계산한 4가지 시나리오와 대조한다.
// (테스트 지도: 20x20, x=15 열에 세로 벽 하나. Q6.8 고정소수점, 1.0=256)
//
//   1) 정면(+x) 방향으로 쏴서 벽(x=15)에 맞는지 — 축 정렬, 계산이 가장 단순
//   2) 옆(+y) 방향으로 쏴서 지도 경계(y=20)에 맞는지 — "지도 밖도 막힘" 확인
//   3) 대각선(45도) 방향으로 쏴서 벽에 맞는지 — 임의 각도에서도 되는지 확인
//   4) 벽도 경계도 못 만나게 MAX_STEPS를 작게 잡아서 "못 찾음"(hit_o=0) 확인
//
//   iverilog -o sim/tb_ray_march.vvp rtl/ray_march.v tb/tb_ray_march.v
//   vvp sim/tb_ray_march.vvp

`timescale 1ns/1ps

module tb_ray_march;

    localparam POS_W = 14;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // DUT1: MAX_STEPS=40 — 벽/경계까지 도달하는 케이스들
    reg  start1;
    reg  signed [POS_W-1:0] x0_1, y0_1, dx1, dy1;
    wire done1, hit1;
    wire [5:0] dist1;

    ray_march #(.MAX_STEPS(40), .DIST_W(6)) u_rm1 (
        .clk(clk), .rst_n(rst_n), .start(start1),
        .x0(x0_1), .y0(y0_1), .dx(dx1), .dy(dy1),
        .done(done1), .dist_o(dist1), .hit_o(hit1)
    );

    // DUT2: MAX_STEPS=5 — "아무것도 못 찾음" 케이스 전용(벽/경계 다 5스텝보다 멀리 둠)
    reg  start2;
    reg  signed [POS_W-1:0] x0_2, y0_2, dx2, dy2;
    wire done2, hit2;
    wire [5:0] dist2;

    ray_march #(.MAX_STEPS(5), .DIST_W(6)) u_rm2 (
        .clk(clk), .rst_n(rst_n), .start(start2),
        .x0(x0_2), .y0(y0_2), .dx(dx2), .dy(dy2),
        .done(done2), .dist_o(dist2), .hit_o(hit2)
    );

    integer pass_count, fail_count;

    task check(input [159:0] name, input [5:0] got_dist, input got_hit,
               input [5:0] exp_dist, input exp_hit);
        begin
            if (got_dist === exp_dist && got_hit === exp_hit) begin
                $display("PASS %0s: dist=%0d hit=%0d (expected dist=%0d hit=%0d)",
                          name, got_dist, got_hit, exp_dist, exp_hit);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL %0s: dist=%0d hit=%0d (expected dist=%0d hit=%0d)",
                          name, got_dist, got_hit, exp_dist, exp_hit);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_ray_march.vcd");
        $dumpvars(0, tb_ray_march);

        pass_count = 0; fail_count = 0;
        start1 = 0; start2 = 0;
        x0_1=0; y0_1=0; dx1=0; dy1=0;
        x0_2=0; y0_2=0; dx2=0; dy2=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // 케이스1: (2.0,10.0)에서 +x 방향 -> 벽(x=15)까지 13스텝
        x0_1 = 14'sd512;  y0_1 = 14'sd2560;   // 2.0, 10.0  (Q6.8: *256)
        dx1  = 14'sd256;  dy1  = 14'sd0;      // (1.0, 0.0)
        @(posedge clk); start1 <= 1; @(posedge clk); start1 <= 0;
        wait (done1); @(posedge clk);
        check("case1_+x_wall", dist1, hit1, 13, 1);

        // 케이스2: (10.0,2.0)에서 +y 방향 -> 벽 없음, 지도경계(y=20)까지 18스텝
        x0_1 = 14'sd2560; y0_1 = 14'sd512;
        dx1  = 14'sd0;    dy1  = 14'sd256;
        @(posedge clk); start1 <= 1; @(posedge clk); start1 <= 0;
        wait (done1); @(posedge clk);
        check("case2_+y_boundary", dist1, hit1, 18, 1);

        // 케이스3: (2.0,2.0)에서 45도 대각선(0.707,0.707) -> 벽(x=15)까지 19스텝
        x0_1 = 14'sd512;  y0_1 = 14'sd512;
        dx1  = 14'sd181;  dy1  = 14'sd181;    // 181/256 ~= 0.707
        @(posedge clk); start1 <= 1; @(posedge clk); start1 <= 0;
        wait (done1); @(posedge clk);
        check("case3_diagonal_wall", dist1, hit1, 19, 1);

        // 케이스4: (9.0,10.0)에서 +x, 근데 MAX_STEPS=5라 벽(6칸 거리)에 못 닿음
        x0_2 = 14'sd2304; y0_2 = 14'sd2560;
        dx2  = 14'sd256;  dy2  = 14'sd0;
        @(posedge clk); start2 <= 1; @(posedge clk); start2 <= 0;
        wait (done2); @(posedge clk);
        check("case4_max_steps_no_hit", dist2, hit2, 5, 0);

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
