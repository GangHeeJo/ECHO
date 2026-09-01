// -*- verilog -*-
// tb_ray_march_bram.v — ray_march_bram.v(실제 changwon 트랙 지도, 400x160칸)를
// echo-ref/gen_track_map.py가 같은 알고리즘으로 미리 걸어본 정답(track_scenarios.txt
// 값을 그대로 옮김)과 대조한다. 지도가 실제 트랙이라 손으로 계산이 불가능해서,
// 소프트웨어가 먼저 계산한 값을 정답으로 쓴다.
//
//   iverilog -o sim/tb_ray_march_bram.vvp rtl/ray_march_bram.v tb/tb_ray_march_bram.v
//   cd sim && vvp tb_ray_march_bram.vvp

`timescale 1ns/1ps

module tb_ray_march_bram;

    localparam POS_W = 18;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg  start;
    reg  signed [POS_W-1:0] x0, y0, dx, dy;
    wire done, hit;
    wire [8:0] dist;

    ray_march_bram u_rm (
        .clk(clk), .rst_n(rst_n), .start(start),
        .x0(x0), .y0(y0), .dx(dx), .dy(dy),
        .done(done), .dist_o(dist), .hit_o(hit)
    );

    integer pass_count, fail_count;

    task check(input [159:0] name, input [8:0] got_dist, input got_hit,
               input [8:0] exp_dist, input exp_hit);
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

    task run_case(input [159:0] name,
                  input signed [POS_W-1:0] px0, py0, pdx, pdy,
                  input [8:0] exp_dist, input exp_hit);
        begin
            x0 = px0; y0 = py0; dx = pdx; dy = pdy;
            @(posedge clk); start <= 1; @(posedge clk); start <= 0;
            wait (done); @(posedge clk);
            check(name, dist, hit, exp_dist, exp_hit);
        end
    endtask

    initial begin
        $dumpfile("tb_ray_march_bram.vcd");
        $dumpvars(0, tb_ray_march_bram);

        pass_count = 0; fail_count = 0;
        start = 0; x0=0; y0=0; dx=0; dy=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // echo-ref/gen_track_map.py의 track_scenarios.txt 값 그대로 (Q9.8 고정소수점)
        run_case("center_+x",   18'sd51200, 18'sd20480, 18'sd256,  18'sd0,     119, 1);
        run_case("center_-x",   18'sd51200, 18'sd20480, -18'sd256, 18'sd0,      18, 1);
        run_case("center_+y",   18'sd51200, 18'sd20480, 18'sd0,    18'sd256,    16, 1);
        run_case("center_-y",   18'sd51200, 18'sd20480, 18'sd0,    -18'sd256,   28, 1);
        run_case("diag",        18'sd51200, 18'sd20480, 18'sd181,  18'sd181,    33, 1);
        run_case("near_wall",   18'sd2560,  18'sd20480, -18'sd256, 18'sd0,       0, 1);

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED (실제 changwon 트랙 지도)");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
