// -*- verilog -*-
// tb_addr_gen.v — addr_gen.v(시프트+덧셈 곱셈)를 손으로 계산한 정답과 대조한다.
// 파일 입출력 없이 그 자리에서 바로 검증 가능한 작은 자가검증.
//
//   iverilog -o sim/tb_addr_gen.vvp rtl/addr_gen.v tb/tb_addr_gen.v
//   vvp sim/tb_addr_gen.vvp

`timescale 1ns/1ps

module tb_addr_gen;

    reg  [8:0]  r, d;
    wire [16:0] addr;

    addr_gen #(.RD_W(9), .ADDR_W(17)) u_gen (.r(r), .d(d), .addr(addr));

    integer i, pass_count, fail_count;
    reg [8:0]  test_r [0:5];
    reg [8:0]  test_d [0:5];
    reg [16:0] test_expected [0:5];

    initial begin
        // r*301+d, 손으로 계산한 값
        test_r[0]=0;   test_d[0]=0;   test_expected[0]=0;          // 0*301+0
        test_r[1]=1;   test_d[1]=0;   test_expected[1]=301;        // 1*301+0
        test_r[2]=0;   test_d[2]=1;   test_expected[2]=1;          // 0*301+1
        test_r[3]=10;  test_d[3]=5;   test_expected[3]=3015;       // 10*301+5
        test_r[4]=52;  test_d[4]=244; test_expected[4]=15896;      // 52*301+244
        test_r[5]=300; test_d[5]=300; test_expected[5]=90600;      // 300*301+300 (표 맨 끝)

        pass_count = 0;
        fail_count = 0;
        for (i = 0; i < 6; i = i + 1) begin
            r = test_r[i];
            d = test_d[i];
            #1;  // 조합논리 안정될 시간
            if (addr === test_expected[i]) begin
                $display("PASS r=%0d d=%0d -> addr=%0d (expected %0d)",
                          r, d, addr, test_expected[i]);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL r=%0d d=%0d -> addr=%0d (expected %0d)",
                          r, d, addr, test_expected[i]);
                fail_count = fail_count + 1;
            end
        end
        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
