// -*- verilog -*-
// tb_sensor_pe_addrgen.v — addr_gen.v를 sensor_pe.v 앞단에 실제로 끼워서, 이제는
// 테스트벤치가 미리 계산한 주소를 주는 게 아니라 **원본 (r,d) 값만 주고 하드웨어가
// 스스로 주소를 계산**해서 룩업+누적까지 끝내는 전체 파이프라인을 검증한다.
//
//   r,d (테스트벤치가 줌) -> addr_gen(시프트+덧셈) -> addr -> table_mem -> sensor_pe -> weight
//
//   iverilog -o sim/tb_sensor_pe_addrgen.vvp rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v tb/tb_sensor_pe_addrgen.v
//   cd sim && vvp tb_sensor_pe_addrgen.vvp

`timescale 1ns/1ps

module tb_sensor_pe_addrgen;

    localparam RD_W   = 9;
    localparam ADDR_W = 17;
    localparam DATA_W = 13;
    localparam ACC_W  = 20;
    localparam NUM_RAYS = 60;
    localparam NUM_PARTICLES = 5;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg  [RD_W-1:0]    r_i, d_i;     // PE가 아니라 addr_gen한테 주는 "원본" 입력
    wire [ADDR_W-1:0]  addr_i;       // addr_gen이 계산해낸 주소 — 하드웨어가 만든 것!

    reg                start, valid_i, last_i;
    wire               done;
    wire signed [ACC_W-1:0] weight_o;
    wire [ADDR_W-1:0]        table_addr;
    wire signed [DATA_W-1:0] table_data;

    addr_gen #(.RD_W(RD_W), .ADDR_W(ADDR_W)) u_addrgen (
        .r(r_i), .d(d_i), .addr(addr_i)
    );

    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
        .clk(clk), .addr(table_addr), .data(table_data)
    );

    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
        .clk(clk), .rst_n(rst_n),
        .table_addr(table_addr), .table_data(table_data),
        .start(start), .valid_i(valid_i), .last_i(last_i),
        .addr_i(addr_i),          // <- addr_gen 출력을 그대로 PE 입력에 연결
        .done(done), .weight_o(weight_o)
    );

    reg [RD_W-1:0]         rs [0:NUM_PARTICLES*NUM_RAYS-1];
    reg [RD_W-1:0]         ds [0:NUM_PARTICLES*NUM_RAYS-1];
    reg signed [ACC_W-1:0] expected [0:NUM_PARTICLES-1];

    integer p, r, pass_count, fail_count;

    initial begin
        $readmemh("testvec_r.hex", rs);
        $readmemh("testvec_d.hex", ds);
        $readmemh("testvec_expected.hex", expected);
    end

    task run_particle(input integer idx);
        begin
            @(posedge clk);
            start   <= 1'b1; valid_i <= 1'b0; last_i <= 1'b0;
            @(posedge clk);
            start <= 1'b0;
            for (r = 0; r < NUM_RAYS; r = r + 1) begin
                r_i     <= rs[idx*NUM_RAYS + r];   // 이제 주소가 아니라 원본 r,d를 줌
                d_i     <= ds[idx*NUM_RAYS + r];
                valid_i <= 1'b1;
                last_i  <= (r == NUM_RAYS-1);
                @(posedge clk);
            end
            valid_i <= 1'b0; last_i <= 1'b0;
            wait (done == 1'b1);
            @(posedge clk);

            if (weight_o === expected[idx]) begin
                $display("PASS particle %0d: weight=%0d (expected %0d)", idx, weight_o, expected[idx]);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL particle %0d: weight=%0d (expected %0d)", idx, weight_o, expected[idx]);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        pass_count = 0; fail_count = 0;
        start = 0; valid_i = 0; last_i = 0; r_i = 0; d_i = 0; rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        for (p = 0; p < NUM_PARTICLES; p = p + 1)
            run_particle(p);

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED (주소 전부 하드웨어가 직접 계산함)");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
