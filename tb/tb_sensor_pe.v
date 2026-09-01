// -*- verilog -*-
// tb_sensor_pe.v — sensor_pe.v를 echo-ref/gen_sensor_model.py가 만든 파이썬
// 정답지(testvec_addrs.hex, testvec_expected.hex)와 대조하는 자가검증 테스트벤치.
// 파티클 5개 x 빔 60개(실제 ZERO 스펙과 동일한 빔 수) — 데이터는 $readmemh로 읽어들여서
// Verilog 소스에는 숫자를 손으로 옮겨적지 않는다(옮겨적다 생긴 오타를 RTL 버그와
// 헷갈리는 걸 원천 차단).
//
//   iverilog -o sim/tb_sensor_pe.vvp rtl/table_mem.v rtl/sensor_pe.v tb/tb_sensor_pe.v
//   cd sim && vvp tb_sensor_pe.vvp
//   gtkwave sim/tb_sensor_pe.vcd   (파형으로 직접 확인하고 싶을 때)

`timescale 1ns/1ps

module tb_sensor_pe;

    localparam ADDR_W = 17;
    localparam DATA_W = 13;
    localparam ACC_W  = 20;
    localparam NUM_PARTICLES = 5;
    localparam NUM_RAYS      = 60;   // 실제 ZERO 스펙(1081빔/18 다운샘플 ≈ 60)

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;   // 100MHz 가정 (10ns 주기)

    reg                start;
    reg                valid_i;
    reg                last_i;
    reg  [ADDR_W-1:0]  addr_i;
    wire               done;
    wire signed [ACC_W-1:0] weight_o;

    wire [ADDR_W-1:0]       table_addr;
    wire signed [DATA_W-1:0] table_data;

    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
        .clk  (clk),
        .addr (table_addr),
        .data (table_data)
    );

    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
        .clk        (clk),
        .rst_n      (rst_n),
        .table_addr (table_addr),
        .table_data (table_data),
        .start      (start),
        .valid_i    (valid_i),
        .last_i     (last_i),
        .addr_i     (addr_i),
        .done       (done),
        .weight_o   (weight_o)
    );

    // echo-ref/gen_sensor_model.py의 산출물. addrs는 [파티클0의 빔60개][파티클1의
    // 빔60개]...로 이어붙인 1차원 배열 — 인덱스는 idx*NUM_RAYS+r로 계산해서 쓴다.
    reg [ADDR_W-1:0]        addrs    [0:NUM_PARTICLES*NUM_RAYS-1];
    reg signed [ACC_W-1:0]  expected [0:NUM_PARTICLES-1];

    integer p, r, pass_count, fail_count;

    initial begin
        $readmemh("testvec_addrs.hex", addrs);
        $readmemh("testvec_expected.hex", expected);
    end

    initial begin
        $dumpfile("tb_sensor_pe.vcd");
        $dumpvars(0, tb_sensor_pe);
    end

    task run_particle(input integer idx);
        begin
            // start 펄스 — 이 클럭엔 valid_i를 같이 안 준다(파이프라인 레이스 방지)
            @(posedge clk);
            start   <= 1'b1;
            valid_i <= 1'b0;
            last_i  <= 1'b0;
            @(posedge clk);
            start <= 1'b0;

            for (r = 0; r < NUM_RAYS; r = r + 1) begin
                addr_i  <= addrs[idx*NUM_RAYS + r];
                valid_i <= 1'b1;
                last_i  <= (r == NUM_RAYS-1);
                @(posedge clk);
            end
            valid_i <= 1'b0;
            last_i  <= 1'b0;

            // BRAM 1클럭 지연 때문에 마지막 빔의 done은 한 클럭 더 뒤에 뜬다
            wait (done == 1'b1);
            @(posedge clk); // done이 선 그 엣지에서 weight_o도 같이 확정됨, 한 클럭 더 관찰

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
        pass_count = 0;
        fail_count = 0;
        start   = 0;
        valid_i = 0;
        last_i  = 0;
        addr_i  = 0;
        rst_n   = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        for (p = 0; p < NUM_PARTICLES; p = p + 1)
            run_particle(p);

        $display("---");
        $display("%0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
