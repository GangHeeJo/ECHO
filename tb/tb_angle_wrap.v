// -*- verilog -*-
// tb_angle_wrap.v — angle_wrap.v(순수 조합논리) 검증. 클럭 없이 그냥 값
// 넣고 바로 비교(조합논리라 지연 0).
//
//   iverilog -o sim/tb_angle_wrap.vvp rtl/angle_wrap.v tb/tb_angle_wrap.v
//   cd sim && vvp tb_angle_wrap.vvp

`timescale 1ns/1ps

module tb_angle_wrap;

    localparam W_EXT = 19;
    localparam NUM_CASES = 17;

    reg  signed [W_EXT-1:0] theta_a, theta_b;
    wire signed [W_EXT-1:0] theta_o;

    angle_wrap u_wrap (.theta_a(theta_a), .theta_b(theta_b), .theta_o(theta_o));

    reg signed [W_EXT-1:0] as [0:NUM_CASES-1];
    reg signed [W_EXT-1:0] bs [0:NUM_CASES-1];
    reg signed [W_EXT-1:0] exps [0:NUM_CASES-1];

    integer i, err, max_abs_err, fails;

    initial begin
        $readmemh("angle_wrap_test_a.hex", as);
        $readmemh("angle_wrap_test_b.hex", bs);
        $readmemh("angle_wrap_test_exp.hex", exps);

        fails = 0; max_abs_err = 0;
        for (i = 0; i < NUM_CASES; i = i + 1) begin
            theta_a = as[i];
            theta_b = bs[i];
            #1;  // 조합논리 안정화

            err = theta_o - exps[i];
            if (err < 0) err = -err;
            if (err > max_abs_err) max_abs_err = err;

            $display("case %2d: a=0x%05x b=0x%05x -> o=0x%05x (exp 0x%05x, err=%0d)",
                      i, as[i], bs[i], theta_o, exps[i], err);
            if (err > 1) begin
                fails = fails + 1;
                $display("  -> FAIL");
            end
        end

        $display("최대 절대오차: %0d LSB", max_abs_err);
        if (fails == 0) $display("ALL TESTS PASSED (%0d/%0d cases)", NUM_CASES, NUM_CASES);
        else $display("SOME TESTS FAILED (%0d/%0d)", fails, NUM_CASES);
        $finish;
    end

endmodule
