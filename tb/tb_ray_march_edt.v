// -*- verilog -*-
// tb_ray_march_edt.v — ray_march_edt.v(거리장 기반)를 ray_march_bram.v(고정
// 1칸 스텝)과 같은 6가지 시나리오로 검증하고, 걸린 클럭 수를 직접 대조한다.
// echo-ref/gen_track_map.py의 track_scenarios_edt.txt 값 그대로.
//
//   iverilog -o sim/tb_ray_march_edt.vvp rtl/ray_march_edt.v tb/tb_ray_march_edt.v
//   cd sim && vvp tb_ray_march_edt.vvp

`timescale 1ns/1ps

module tb_ray_march_edt;

    localparam POS_W = 18;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg  start;
    reg  signed [POS_W-1:0] x0, y0, dx, dy;
    wire done, hit;
    wire [8:0] dist;

    ray_march_edt u_rm (
        .clk(clk), .rst_n(rst_n), .start(start),
        .x0(x0), .y0(y0), .dx(dx), .dy(dy),
        .done(done), .dist_o(dist), .hit_o(hit)
    );

    integer pass_count, fail_count;
    real t_start, t_end;

    task check(input [159:0] name, input [8:0] got_dist, input got_hit,
               input [8:0] exp_dist, input exp_hit, input real cycles);
        begin
            if (got_dist === exp_dist && got_hit === exp_hit) begin
                $display("PASS %0s: dist=%0d hit=%0d (expected dist=%0d hit=%0d), %0.0f클럭",
                          name, got_dist, got_hit, exp_dist, exp_hit, cycles);
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
                  input [8:0] exp_dist, input exp_hit, input [8:0] old_cycles);
        begin
            x0 = px0; y0 = py0; dx = pdx; dy = pdy;
            t_start = $realtime;
            @(posedge clk); start <= 1; @(posedge clk); start <= 0;
            wait (done); @(posedge clk);
            t_end = $realtime;
            check(name, dist, hit, exp_dist, exp_hit, (t_end - t_start) / 10.0);
            $display("  (참고: ray_march_bram.v는 이 케이스에 %0d클럭 걸렸음)", old_cycles);
        end
    endtask

    initial begin
        $dumpfile("tb_ray_march_edt.vcd");
        $dumpvars(0, tb_ray_march_edt);

        pass_count = 0; fail_count = 0;
        start = 0; x0=0; y0=0; dx=0; dy=0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // echo-ref/gen_track_map.py의 track_scenarios_edt.txt 값 (dist는 고정스텝과
        // 동일 — 같은 실제 거리를 재는 거니까 당연함, 다른 건 "몇 클럭 걸렸나"임)
        // old_cycles는 tb_ray_march_bram.v 실행 결과에서 옮김(dist+1클럭 정도)
        run_case("center_+x",   18'sd51200, 18'sd20480, 18'sd256,  18'sd0,     119, 1, 120);
        run_case("center_-x",   18'sd51200, 18'sd20480, -18'sd256, 18'sd0,      18, 1,  19);
        run_case("center_+y",   18'sd51200, 18'sd20480, 18'sd0,    18'sd256,    16, 1,  17);
        run_case("center_-y",   18'sd51200, 18'sd20480, 18'sd0,    -18'sd256,   28, 1,  29);
        run_case("diag",        18'sd51200, 18'sd20480, 18'sd181,  18'sd181,    33, 1,  34);
        run_case("near_wall",   18'sd2560,  18'sd20480, -18'sd256, 18'sd0,       0, 1,   1);

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED (거리장 기반 마칭)");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
