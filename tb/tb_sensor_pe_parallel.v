// -*- verilog -*-
// tb_sensor_pe_parallel.v — PE 2개가 듀얼포트 메모리 하나를 공유하며 파티클
// 2개를 "진짜 동시에" 처리하는지 검증한다.
//
// tb_sensor_pe.v와 같은 정답지(testvec_addrs.hex/testvec_expected.hex)를 쓰되,
// 이번엔 파티클 0/1을 PE0/PE1에 각각 동시에 흘려보낸다. 검증 포인트 두 가지:
//   1. 각 PE의 최종 weight가 정답과 일치하는가 (기능이 맞는가)
//   2. 두 PE의 done이 거의 같은 클럭에 뜨는가, 그리고 전체 소요 시간이
//      "파티클 1개 처리 시간"과 비슷한가 (진짜 병렬로 처리됐는가 — 순서대로
//      했으면 시간이 거의 2배 걸렸을 것)
//
//   iverilog -o sim/tb_sensor_pe_parallel.vvp rtl/table_mem_dp.v rtl/sensor_pe.v tb/tb_sensor_pe_parallel.v
//   cd sim && vvp tb_sensor_pe_parallel.vvp

`timescale 1ns/1ps

module tb_sensor_pe_parallel;

    localparam ADDR_W = 17;
    localparam DATA_W = 13;
    localparam ACC_W  = 20;
    localparam NUM_RAYS = 60;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // PE0 인터페이스
    reg                     start0, valid0, last0;
    reg  [ADDR_W-1:0]       addr0;
    wire                    done0;
    wire signed [ACC_W-1:0] weight0;

    // PE1 인터페이스
    reg                     start1, valid1, last1;
    reg  [ADDR_W-1:0]       addr1;
    wire                    done1;
    wire signed [ACC_W-1:0] weight1;

    wire [ADDR_W-1:0]        table_addr_a, table_addr_b;
    wire signed [DATA_W-1:0] table_data_a, table_data_b;

    table_mem_dp #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
        .clk    (clk),
        .addr_a (table_addr_a), .data_a (table_data_a),
        .addr_b (table_addr_b), .data_b (table_data_b)
    );

    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe0 (
        .clk(clk), .rst_n(rst_n),
        .table_addr(table_addr_a), .table_data(table_data_a),
        .start(start0), .valid_i(valid0), .last_i(last0), .addr_i(addr0),
        .done(done0), .weight_o(weight0)
    );

    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe1 (
        .clk(clk), .rst_n(rst_n),
        .table_addr(table_addr_b), .table_data(table_data_b),
        .start(start1), .valid_i(valid1), .last_i(last1), .addr_i(addr1),
        .done(done1), .weight_o(weight1)
    );

    reg [ADDR_W-1:0]       addrs    [0:5*NUM_RAYS-1];  // 파일엔 파티클 5개치 다 있음, 0/1번만 씀
    reg signed [ACC_W-1:0] expected [0:4];

    integer r;
    real t_start, t_end;

    initial begin
        $readmemh("testvec_addrs.hex", addrs);
        $readmemh("testvec_expected.hex", expected);
    end

    initial begin
        $dumpfile("tb_sensor_pe_parallel.vcd");
        $dumpvars(0, tb_sensor_pe_parallel);
    end

    // PE0에 파티클 idx를 흘려보내는 task (tb_sensor_pe.v의 run_particle과 같은 패턴)
    task drive_pe0(input integer idx);
        integer rr;
        begin
            @(posedge clk);
            start0 <= 1'b1; valid0 <= 1'b0; last0 <= 1'b0;
            @(posedge clk);
            start0 <= 1'b0;
            for (rr = 0; rr < NUM_RAYS; rr = rr + 1) begin
                addr0  <= addrs[idx*NUM_RAYS + rr];
                valid0 <= 1'b1;
                last0  <= (rr == NUM_RAYS-1);
                @(posedge clk);
            end
            valid0 <= 1'b0; last0 <= 1'b0;
        end
    endtask

    task drive_pe1(input integer idx);
        integer rr;
        begin
            @(posedge clk);
            start1 <= 1'b1; valid1 <= 1'b0; last1 <= 1'b0;
            @(posedge clk);
            start1 <= 1'b0;
            for (rr = 0; rr < NUM_RAYS; rr = rr + 1) begin
                addr1  <= addrs[idx*NUM_RAYS + rr];
                valid1 <= 1'b1;
                last1  <= (rr == NUM_RAYS-1);
                @(posedge clk);
            end
            valid1 <= 1'b0; last1 <= 1'b0;
        end
    endtask

    initial begin
        start0 = 0; valid0 = 0; last0 = 0; addr0 = 0;
        start1 = 0; valid1 = 0; last1 = 0; addr1 = 0;
        rst_n  = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        // 핵심: fork/join — 두 task가 "동시에" 시작해서 같은 클럭들을 같이 씀
        fork
            drive_pe0(0);   // PE0 = 파티클 0
            drive_pe1(1);   // PE1 = 파티클 1, 정확히 같은 타이밍에 병렬로
        join

        // 두 PE 다 done 뜰 때까지 기다림 (거의 동시에 뜰 것으로 기대)
        wait (done0 == 1'b1);
        wait (done1 == 1'b1);
        @(posedge clk);
        t_end = $realtime;

        $display("PE0 (particle 0): weight=%0d (expected %0d) -> %s",
                  weight0, expected[0], (weight0 === expected[0]) ? "PASS" : "FAIL");
        $display("PE1 (particle 1): weight=%0d (expected %0d) -> %s",
                  weight1, expected[1], (weight1 === expected[1]) ? "PASS" : "FAIL");
        $display("총 소요시간: %0.1f ns (파티클 1개 단독 처리 시간과 비슷해야 '진짜 병렬'임)",
                  t_end - t_start);

        if (weight0 === expected[0] && weight1 === expected[1])
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
