// -*- verilog -*-
// table_mem.v — 센서모델 log(확률) 테이블을 담는 동기식(등록된) 읽기 BRAM.
//
// echo-ref/gen_sensor_model.py가 만든 .hex 파일을 $readmemh로 그대로 읽어들인다.
// 주소 = r*301+d (r=관측거리, d=기대거리, 둘 다 0~300 — gen_sensor_model.py와 동일).
// 읽기 지연이 1클럭인 게 핵심 — sensor_pe.v가 이 타이밍을 그대로 가정하고 설계됨.

module table_mem #(
    parameter ADDR_W   = 17,
    parameter DATA_W   = 13,
    parameter DEPTH    = 90601,   // 301*301
    parameter INIT_FILE = "sensor_model_log_q5_8.hex"
) (
    input  wire                     clk,
    input  wire [ADDR_W-1:0]        addr,
    output reg  signed [DATA_W-1:0] data
);

    reg signed [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        data <= mem[addr];
    end

endmodule
