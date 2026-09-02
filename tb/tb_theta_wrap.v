// -*- verilog -*-
// tb_theta_wrap.v — theta_wrap.v 검증. gen_theta_wrap_table.py의 14개
// 케이스(실측 범위 + 여유있는 다중바퀴 케이스).
//
//   iverilog -o sim/tb_theta_wrap.vvp rtl/theta_wrap.v tb/tb_theta_wrap.v
//   cd sim && vvp tb_theta_wrap.vvp

`timescale 1ns/1ps

module tb_theta_wrap;

    localparam W_RAW = 24;
    localparam W_EXT = 19;
    localparam NUM_CASES = 14;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                     start;
    reg  signed [W_RAW-1:0] theta_raw;
    wire                    done;
    wire signed [W_EXT-1:0] theta_o;

    theta_wrap u_wrap (
        .clk(clk), .rst_n(rst_n), .start(start),
        .theta_raw(theta_raw), .done(done), .theta_o(theta_o)
    );

    reg signed [W_RAW-1:0] raws [0:NUM_CASES-1];
    reg signed [W_EXT-1:0] exps [0:NUM_CASES-1];

    integer i, err, fails;

    initial begin
        $readmemh("theta_wrap_test_raw.hex", raws);
        $readmemh("theta_wrap_test_exp.hex", exps);
    end

    initial begin
        start = 0; theta_raw = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        fails = 0;
        for (i = 0; i < NUM_CASES; i = i + 1) begin
            theta_raw <= raws[i];
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            wait (done);

            err = theta_o - exps[i];
            if (err < 0) err = -err;

            $display("case %2d: raw=0x%06x -> o=0x%05x (exp 0x%05x, err=%0d)",
                      i, raws[i], theta_o, exps[i], err);
            if (err > 0) begin
                fails = fails + 1;
                $display("  -> FAIL");
            end
            @(posedge clk);
        end

        if (fails == 0) $display("ALL TESTS PASSED (%0d/%0d cases, exact match - 순수 정수 덧뺄셈)", NUM_CASES, NUM_CASES);
        else $display("SOME TESTS FAILED (%0d/%0d)", fails, NUM_CASES);
        $finish;
    end

endmodule
