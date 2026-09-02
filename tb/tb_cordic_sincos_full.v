// -*- verilog -*-
// tb_cordic_sincos_full.v — cordic_sincos_full.v(사분면 접기 래퍼)를
// tools/gen_cordic_full_table.py가 만든 19개 각도(-179~179도, 전 범위·경계
// 케이스 포함)와 대조.
//
//   iverilog -o sim/tb_cordic_sincos_full.vvp rtl/cordic_sincos.v rtl/cordic_sincos_full.v tb/tb_cordic_sincos_full.v
//   cd sim && vvp tb_cordic_sincos_full.vvp

`timescale 1ns/1ps

module tb_cordic_sincos_full;

    localparam W_EXT = 19;
    localparam NUM_CASES = 19;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                     start;
    reg  signed [W_EXT-1:0] theta_i;
    wire                    done;
    wire signed [W_EXT-1:0] cos_o, sin_o;

    cordic_sincos_full u_full (
        .clk(clk), .rst_n(rst_n), .start(start),
        .theta_i(theta_i), .done(done), .cos_o(cos_o), .sin_o(sin_o)
    );

    reg signed [W_EXT-1:0] thetas [0:NUM_CASES-1];
    reg signed [W_EXT-1:0] exp_cos [0:NUM_CASES-1];
    reg signed [W_EXT-1:0] exp_sin [0:NUM_CASES-1];

    integer i;
    integer cos_err, sin_err, max_abs_err, fails;

    initial begin
        $readmemh("cordic_full_test_theta.hex", thetas);
        $readmemh("cordic_full_test_cos.hex",   exp_cos);
        $readmemh("cordic_full_test_sin.hex",   exp_sin);
    end

    initial begin
        start = 0; theta_i = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        fails = 0;
        max_abs_err = 0;

        for (i = 0; i < NUM_CASES; i = i + 1) begin
            theta_i <= thetas[i];
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            wait (done);

            cos_err = cos_o - exp_cos[i];
            sin_err = sin_o - exp_sin[i];
            if (cos_err < 0) cos_err = -cos_err;
            if (sin_err < 0) sin_err = -sin_err;
            if (cos_err > max_abs_err) max_abs_err = cos_err;
            if (sin_err > max_abs_err) max_abs_err = sin_err;

            $display("case %2d: theta=0x%05x cos=0x%05x(exp 0x%05x, err=%0d) sin=0x%05x(exp 0x%05x, err=%0d)",
                      i, thetas[i], cos_o, exp_cos[i], cos_err, sin_o, exp_sin[i], sin_err);

            if (cos_err > 32 || sin_err > 32) begin
                fails = fails + 1;
                $display("  -> FAIL");
            end

            @(posedge clk);
        end

        $display("최대 절대오차: %0d LSB (Q3.16, 1LSB=%.6f)", max_abs_err, 1.0/65536.0);
        if (fails == 0) $display("ALL TESTS PASSED (%0d/%0d cases, 전 범위 사분면 포함)", NUM_CASES, NUM_CASES);
        else $display("SOME TESTS FAILED (%0d/%0d)", fails, NUM_CASES);
        $finish;
    end

endmodule
