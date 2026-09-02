// -*- verilog -*-
// tb_particle_scorer_beamrom.v — particle_scorer_beamrom.v를 기존 particle0의
// 정답지(expected.hex)로 검증. dx/dy.hex는 이제 안 씀(beam_id=0..7 순서가 이미
// gen_particle_scorer_test.py와 gen_beam_step_rom.py 양쪽 다 -60~60도 8등분과
// 같은 순서라 그대로 대응됨) — r.hex/x0y0.hex/expected.hex만 재사용.
//
//   iverilog -o sim/tb_particle_scorer_beamrom.vvp rtl/ray_march_edt_beamrom.v rtl/addr_gen.v rtl/table_mem.v rtl/sensor_pe.v rtl/particle_scorer_beamrom.v tb/tb_particle_scorer_beamrom.v
//   cd sim && vvp tb_particle_scorer_beamrom.vvp

`timescale 1ns/1ps

module tb_particle_scorer_beamrom;

    localparam RM_POS_W  = 18;
    localparam BEAM_W    = 3;
    localparam RD_W      = 9;
    localparam ACC_W     = 20;
    localparam NUM_RAYS  = 8;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg                        particle_start;
    reg  signed [RM_POS_W-1:0] x0, y0;
    reg                        beam_start;
    reg  [BEAM_W-1:0]          beam_id;
    reg  [RD_W-1:0]            r_obs;
    reg                        beam_last;
    wire                       beam_done, particle_done;
    wire signed [ACC_W-1:0]    weight_o;

    particle_scorer_beamrom #(.RM_POS_W(RM_POS_W), .BEAM_W(BEAM_W), .RD_W(RD_W), .ACC_W(ACC_W)) u_scorer (
        .clk(clk), .rst_n(rst_n),
        .particle_start(particle_start), .x0(x0), .y0(y0),
        .beam_start(beam_start), .beam_id(beam_id), .r_obs(r_obs), .beam_last(beam_last),
        .beam_done(beam_done), .particle_done(particle_done), .weight_o(weight_o)
    );

    reg [RD_W-1:0]            rs  [0:NUM_RAYS-1];
    reg signed [RM_POS_W-1:0] x0y0 [0:1];
    reg signed [ACC_W-1:0]    expected_mem [0:0];

    integer i;

    initial begin
        $readmemh("particle0_r.hex",  rs);
        $readmemh("particle0_x0y0.hex", x0y0);
        $readmemh("particle0_expected.hex", expected_mem);
    end

    initial begin
        $dumpfile("tb_particle_scorer_beamrom.vcd");
        $dumpvars(0, tb_particle_scorer_beamrom);

        particle_start = 0; beam_start = 0;
        x0 = 0; y0 = 0; beam_id = 0; r_obs = 0; beam_last = 0;
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        x0 = x0y0[0];
        y0 = x0y0[1];

        @(posedge clk); particle_start <= 1; @(posedge clk); particle_start <= 0;

        for (i = 0; i < NUM_RAYS; i = i + 1) begin
            @(posedge clk);
            beam_id <= i[BEAM_W-1:0]; r_obs <= rs[i];
            beam_last <= (i == NUM_RAYS - 1);
            beam_start <= 1;
            @(posedge clk);
            beam_start <= 0;
            wait (beam_done);
            $display("빔 %0d 처리 완료 (t=%0t ns)", i, $time);
        end

        wait (particle_done);
        @(posedge clk);

        if (weight_o === expected_mem[0]) begin
            $display("PASS: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
            $display("ALL TESTS PASSED (beam ROM 버전)");
        end else begin
            $display("FAIL: weight_o=%0d (expected %0d)", weight_o, expected_mem[0]);
            $display("SOME TESTS FAILED");
        end
        $finish;
    end

endmodule
