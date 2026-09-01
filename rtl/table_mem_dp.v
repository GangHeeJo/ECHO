// -*- verilog -*-
// table_mem_dp.v — 센서모델 log(확률) 테이블의 듀얼포트(읽기 2개) 버전.
//
// FPGA의 BRAM은 원래 포트 2개(듀얼포트)를 표준으로 지원한다. 같은 mem 배열을
// 두 개의 독립된 always 블록에서 각자 읽게 하면, 합성 툴이 이 패턴을 "진짜
// 듀얼포트 BRAM 하나"로 인식한다(테이블을 2벌 복사하는 게 아니다) — 그래서
// PE 2개가 매 클럭 서로 다른 주소를 동시에 읽을 수 있다.
//
// table_mem.v(싱글포트)와 내용은 완전히 같고 포트만 2개로 늘린 것.

module table_mem_dp #(
    parameter ADDR_W   = 17,
    parameter DATA_W   = 13,
    parameter DEPTH    = 90601,   // 301*301
    parameter INIT_FILE = "sensor_model_log_q5_8.hex"
) (
    input  wire                     clk,
    input  wire [ADDR_W-1:0]        addr_a,
    output reg  signed [DATA_W-1:0] data_a,
    input  wire [ADDR_W-1:0]        addr_b,
    output reg  signed [DATA_W-1:0] data_b
);

    reg signed [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) data_a <= mem[addr_a];   // 포트 A — PE0 전용
    always @(posedge clk) data_b <= mem[addr_b];   // 포트 B — PE1 전용, 서로 독립적

endmodule
