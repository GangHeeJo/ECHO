// -*- verilog -*-
// tb_sensor_pe_x4.v — PE 4개, 듀얼포트 메모리(테이블 복사본) 2개로 파티클 4개를
// 동시에 처리. sensor_pe.v/table_mem_dp.v는 전혀 안 고치고 그대로 재사용한다
// (테이블 복사본 하나당 포트 2개 = PE 2개, 그래서 PE 4개엔 테이블 2벌 필요).
//
//   iverilog -o sim/tb_sensor_pe_x4.vvp rtl/table_mem_dp.v rtl/sensor_pe.v tb/tb_sensor_pe_x4.v
//   cd sim && vvp tb_sensor_pe_x4.vvp

`timescale 1ns/1ps

module tb_sensor_pe_x4;

    localparam ADDR_W = 17;
    localparam DATA_W = 13;
    localparam ACC_W  = 20;
    localparam NUM_RAYS = 60;
    localparam NUM_PE   = 4;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // PE 4개분 신호 — 배열로 선언(generate로 인스턴스화)
    reg                     start [0:NUM_PE-1];
    reg                     valid_i [0:NUM_PE-1];
    reg                     last_i [0:NUM_PE-1];
    reg  [ADDR_W-1:0]       addr_i [0:NUM_PE-1];
    wire                    done [0:NUM_PE-1];
    wire signed [ACC_W-1:0] weight_o [0:NUM_PE-1];

    // 테이블 복사본 2개, 각각 포트 A/B로 PE 2개씩 담당
    // copy0: PE0(포트A), PE1(포트B) / copy1: PE2(포트A), PE3(포트B)
    wire [ADDR_W-1:0]        t_addr_a [0:1];
    wire [ADDR_W-1:0]        t_addr_b [0:1];
    wire signed [DATA_W-1:0] t_data_a [0:1];
    wire signed [DATA_W-1:0] t_data_b [0:1];

    genvar gi;
    generate
        for (gi = 0; gi < 2; gi = gi + 1) begin : g_table
            table_mem_dp #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
                .clk    (clk),
                .addr_a (t_addr_a[gi]), .data_a (t_data_a[gi]),
                .addr_b (t_addr_b[gi]), .data_b (t_data_b[gi])
            );
        end
        for (gi = 0; gi < NUM_PE; gi = gi + 1) begin : g_pe
            // gi=0,2 -> 각 테이블 복사본의 포트A / gi=1,3 -> 포트B
            // (genvar 조건이라 컴파일 시점에 둘 중 하나로 고정됨 — 런타임 mux 아님)
            if (gi % 2 == 0) begin : g_porta
                sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
                    .clk(clk), .rst_n(rst_n),
                    .table_addr(t_addr_a[gi/2]), .table_data(t_data_a[gi/2]),
                    .start(start[gi]), .valid_i(valid_i[gi]), .last_i(last_i[gi]),
                    .addr_i(addr_i[gi]), .done(done[gi]), .weight_o(weight_o[gi])
                );
            end else begin : g_portb
                sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
                    .clk(clk), .rst_n(rst_n),
                    .table_addr(t_addr_b[gi/2]), .table_data(t_data_b[gi/2]),
                    .start(start[gi]), .valid_i(valid_i[gi]), .last_i(last_i[gi]),
                    .addr_i(addr_i[gi]), .done(done[gi]), .weight_o(weight_o[gi])
                );
            end
        end
    endgenerate

    reg [ADDR_W-1:0]       addrs    [0:5*NUM_RAYS-1];  // 파티클 5개치, 0~3번만 씀
    reg signed [ACC_W-1:0] expected [0:4];

    initial begin
        $readmemh("testvec_addrs.hex", addrs);
        $readmemh("testvec_expected.hex", expected);
    end

    initial begin
        $dumpfile("tb_sensor_pe_x4.vcd");
        $dumpvars(0, tb_sensor_pe_x4);
    end

    // automatic 필수 — fork로 4번 동시에 부르는데, 기본(static) task는 호출마다
    // 변수(rr, pe, idx까지)를 공유해서 4개가 서로 덮어쓰며 꼬인다(무한루프 원인이었음).
    task automatic drive_pe(input integer pe, input integer idx);
        integer rr;
        begin
            @(posedge clk);
            start[pe] <= 1'b1; valid_i[pe] <= 1'b0; last_i[pe] <= 1'b0;
            @(posedge clk);
            start[pe] <= 1'b0;
            for (rr = 0; rr < NUM_RAYS; rr = rr + 1) begin
                addr_i[pe]  <= addrs[idx*NUM_RAYS + rr];
                valid_i[pe] <= 1'b1;
                last_i[pe]  <= (rr == NUM_RAYS-1);
                @(posedge clk);
            end
            valid_i[pe] <= 1'b0; last_i[pe] <= 1'b0;
        end
    endtask

    integer k, fails;
    real t_start, t_end;

    initial begin
        for (k = 0; k < NUM_PE; k = k + 1) begin
            start[k] = 0; valid_i[k] = 0; last_i[k] = 0; addr_i[k] = 0;
        end
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        // 파티클 0,1,2,3을 PE 0,1,2,3에 각각 동시에 흘려보냄
        fork
            drive_pe(0, 0);
            drive_pe(1, 1);
            drive_pe(2, 2);
            drive_pe(3, 3);
        join

        for (k = 0; k < NUM_PE; k = k + 1)
            wait (done[k] == 1'b1);
        @(posedge clk);
        t_end = $realtime;

        fails = 0;
        for (k = 0; k < NUM_PE; k = k + 1) begin
            if (weight_o[k] === expected[k]) begin
                $display("PE%0d (particle %0d): weight=%0d (expected %0d) -> PASS",
                          k, k, weight_o[k], expected[k]);
            end else begin
                $display("PE%0d (particle %0d): weight=%0d (expected %0d) -> FAIL",
                          k, k, weight_o[k], expected[k]);
                fails = fails + 1;
            end
        end
        $display("총 소요시간: %0.1f ns (PE 2개 버전과 비슷해야 '진짜 4배 병렬'임)",
                  t_end - t_start);

        if (fails == 0) $display("ALL TESTS PASSED");
        else $display("SOME TESTS FAILED");
        $finish;
    end

endmodule
