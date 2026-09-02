// -*- verilog -*-
// tb_direction_gen.v — direction_gen.v를 실제 ZERO 프레임(bench/zero_frame_001.npz)
// 의 진짜 파티클 5개 x 실제 60빔 = 300케이스로 검증. gen_direction_gen_test.py.
//
//   iverilog -o sim/tb_direction_gen.vvp rtl/cordic_sincos.v rtl/cordic_sincos_full.v rtl/angle_wrap.v rtl/direction_gen.v tb/tb_direction_gen.v
//   cd sim && vvp tb_direction_gen.vvp

`timescale 1ns/1ps

module tb_direction_gen;

    localparam W_EXT = 19;
    localparam W_OUT = 18;
    localparam BEAM_W = 6;
    localparam NUM_RAYS = 60;
    localparam NUM_PARTICLES = 5;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                     start;
    reg  signed [W_EXT-1:0] theta_i;
    reg  [BEAM_W-1:0]       beam_id;
    wire                    done;
    wire signed [W_OUT-1:0] dx_o, dy_o;

    direction_gen u_dgen (
        .clk(clk), .rst_n(rst_n), .start(start),
        .theta_i(theta_i), .beam_id(beam_id),
        .done(done), .dx_o(dx_o), .dy_o(dy_o)
    );

    reg signed [W_EXT-1:0] thetas [0:NUM_PARTICLES-1];
    reg signed [W_OUT-1:0] exp_dx [0:NUM_PARTICLES*NUM_RAYS-1];
    reg signed [W_OUT-1:0] exp_dy [0:NUM_PARTICLES*NUM_RAYS-1];

    integer p, b, idx;
    integer dx_err, dy_err, max_abs_err, fails, total;

    initial begin
        $readmemh("direction_gen_test_theta.hex", thetas);
        $readmemh("direction_gen_test_dx.hex", exp_dx);
        $readmemh("direction_gen_test_dy.hex", exp_dy);
    end

    initial begin
        start = 0; theta_i = 0; beam_id = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        fails = 0; max_abs_err = 0; total = 0;

        for (p = 0; p < NUM_PARTICLES; p = p + 1) begin
            for (b = 0; b < NUM_RAYS; b = b + 1) begin
                idx = p * NUM_RAYS + b;
                theta_i <= thetas[p];
                beam_id <= b[BEAM_W-1:0];
                @(posedge clk); start <= 1;
                @(posedge clk); start <= 0;
                wait (done);

                dx_err = dx_o - exp_dx[idx];
                dy_err = dy_o - exp_dy[idx];
                if (dx_err < 0) dx_err = -dx_err;
                if (dy_err < 0) dy_err = -dy_err;
                if (dx_err > max_abs_err) max_abs_err = dx_err;
                if (dy_err > max_abs_err) max_abs_err = dy_err;
                total = total + 1;

                if (dx_err > 3 || dy_err > 3) begin
                    fails = fails + 1;
                    $display("FAIL p=%0d b=%0d: dx=0x%05x(exp 0x%05x,err=%0d) dy=0x%05x(exp 0x%05x,err=%0d)",
                              p, b, dx_o, exp_dx[idx], dx_err, dy_o, exp_dy[idx], dy_err);
                end

                @(posedge clk);
            end
            $display("파티클 %0d(theta=0x%05x) 60빔 완료", p, thetas[p]);
        end

        $display("총 %0d케이스, 최대 절대오차 %0d LSB(Q9.8, 1LSB=%.5f)", total, max_abs_err, 1.0/256.0);
        if (fails == 0) $display("ALL TESTS PASSED (%0d/%0d, 실제 ZERO 프레임 데이터)", total, total);
        else $display("SOME TESTS FAILED (%0d/%0d)", fails, total);
        $finish;
    end

endmodule
