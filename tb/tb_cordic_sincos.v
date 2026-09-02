// -*- verilog -*-
// tb_cordic_sincos.v — cordic_sincos.v를 tools/gen_cordic_table.py가 만든
// 15개 각도(-89~89도) 테스트 벡터와 대조. CORDIC은 유한 반복 근사라 다른
// 모듈들처럼 비트 단위 완전일치는 기대 안 함 — 오차를 LSB 단위로 출력해서
// 눈으로 확인.
//
//   iverilog -o sim/tb_cordic_sincos.vvp rtl/cordic_sincos.v tb/tb_cordic_sincos.v
//   cd sim && vvp tb_cordic_sincos.vvp

`timescale 1ns/1ps

module tb_cordic_sincos;

    localparam W = 18;
    localparam N = 16;
    localparam NUM_CASES = 15;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                  start;
    reg  signed [W-1:0]  theta_i;
    wire                 done;
    wire signed [W-1:0]  cos_o, sin_o;

    cordic_sincos #(.N(N), .W(W)) u_cordic (
        .clk(clk), .rst_n(rst_n), .start(start),
        .theta_i(theta_i), .done(done), .cos_o(cos_o), .sin_o(sin_o)
    );

    reg signed [W-1:0] thetas [0:NUM_CASES-1];
    reg signed [W-1:0] exp_cos [0:NUM_CASES-1];
    reg signed [W-1:0] exp_sin [0:NUM_CASES-1];

    integer i;
    integer cos_err, sin_err;
    integer max_abs_err;
    integer fails;

    initial begin
        $readmemh("cordic_test_theta.hex", thetas);
        $readmemh("cordic_test_cos.hex",   exp_cos);
        $readmemh("cordic_test_sin.hex",   exp_sin);
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

            // 16회 반복 근사 오차 허용치 — 여유있게 32 LSB(Q1.16 기준 4.9e-4)
            if (cos_err > 32 || sin_err > 32) begin
                fails = fails + 1;
                $display("  -> FAIL (오차 허용치 초과)");
            end

            @(posedge clk);
        end

        $display("최대 절대오차: %0d LSB (Q1.16, 1LSB=1/65536=%.6f)", max_abs_err, 1.0/65536.0);
        if (fails == 0) $display("ALL TESTS PASSED (%0d/%0d cases)", NUM_CASES, NUM_CASES);
        else $display("SOME TESTS FAILED (%0d/%0d)", fails, NUM_CASES);
        $finish;
    end

endmodule
