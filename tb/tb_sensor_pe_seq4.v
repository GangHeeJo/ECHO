// -*- verilog -*-
// tb_sensor_pe_seq4.v — PE 1개로 파티클 4개를 "순서대로" 처리했을 때 걸리는 시간을
// 실측한다. tb_sensor_pe_x4.v(PE 4개, 640ns)와 직접 비교하려고 만든 대조군.

`timescale 1ns/1ps

module tb_sensor_pe_seq4;

    localparam ADDR_W = 17;
    localparam DATA_W = 13;
    localparam ACC_W  = 20;
    localparam NUM_RAYS = 60;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                start, valid_i, last_i;
    reg  [ADDR_W-1:0]  addr_i;
    wire               done;
    wire signed [ACC_W-1:0] weight_o;
    wire [ADDR_W-1:0]       table_addr;
    wire signed [DATA_W-1:0] table_data;

    table_mem #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_table (
        .clk(clk), .addr(table_addr), .data(table_data)
    );
    sensor_pe #(.ADDR_W(ADDR_W), .DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
        .clk(clk), .rst_n(rst_n),
        .table_addr(table_addr), .table_data(table_data),
        .start(start), .valid_i(valid_i), .last_i(last_i), .addr_i(addr_i),
        .done(done), .weight_o(weight_o)
    );

    reg [ADDR_W-1:0]       addrs    [0:5*NUM_RAYS-1];
    reg signed [ACC_W-1:0] expected [0:4];
    integer p, r, fails;
    real t_start, t_end;

    initial begin
        $readmemh("testvec_addrs.hex", addrs);
        $readmemh("testvec_expected.hex", expected);
    end

    task run_particle(input integer idx);
        begin
            @(posedge clk);
            start <= 1'b1; valid_i <= 1'b0; last_i <= 1'b0;
            @(posedge clk);
            start <= 1'b0;
            for (r = 0; r < NUM_RAYS; r = r + 1) begin
                addr_i  <= addrs[idx*NUM_RAYS + r];
                valid_i <= 1'b1;
                last_i  <= (r == NUM_RAYS-1);
                @(posedge clk);
            end
            valid_i <= 1'b0; last_i <= 1'b0;
            wait (done == 1'b1);
            @(posedge clk);
        end
    endtask

    initial begin
        start = 0; valid_i = 0; last_i = 0; addr_i = 0; rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        t_start = $realtime;
        fails = 0;
        for (p = 0; p < 4; p = p + 1) begin
            run_particle(p);
            if (weight_o !== expected[p]) fails = fails + 1;
        end
        t_end = $realtime;

        $display("PE 1개로 파티클 4개 순서대로 처리: %0.1f ns (fails=%0d)", t_end - t_start, fails);
        $finish;
    end

endmodule
