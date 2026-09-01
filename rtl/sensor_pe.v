// -*- verilog -*-
// sensor_pe.v — ECHO v1: 파티클필터 센서모델 평가를 위한 최소 PE(연산기) 1개.
//
// 매 빔(라이다 방향)마다, 미리 만들어둔 log(확률) 테이블(BRAM, table_mem.v)에서
// 값을 찾아 누적 레지스터에 더하기만 한다. 곱셈기가 없다 — 원본 소프트웨어가
// 확률을 곱하는 걸(log(a*b)=log(a)+log(b)), 테이블 자체를 log(확률)로 저장해서
// 하드웨어에서는 덧셈으로 바꿔둔 설계다(echo-ref/gen_sensor_model.py 참고).
//
// 인터페이스는 BRAM의 1클럭 동기 읽기 지연을 그대로 반영한 파이프라인이다:
//   클럭 N   : addr_i/valid_i/last_i 를 준다
//   클럭 N+1 : table_data 로 그 주소의 값이 돌아온다 -> 누적
//
// ponytail: addr_i(=r*table_width+d)는 상위(테스트벤치/컨트롤러)가 미리 계산해서
// 준다. table_width=301이 2의 거듭제곱이 아니라 곱셈기 없이 주소를 만드는 문제는
// v1 스코프 밖 — 여러 PE를 두는 다음 단계에서 공유 주소생성 유닛으로 풀 것.

module sensor_pe #(
    parameter ADDR_W = 17,   // 테이블 주소 폭: 301*301=90601 < 2^17
    parameter DATA_W = 13,   // 테이블 항목 폭: Q5.8 고정소수점 log(prob)
    parameter ACC_W  = 20    // 누적기 폭: 빔 최대 60개 합, 여유 있게
) (
    input  wire                      clk,
    input  wire                      rst_n,      // active-low 비동기 리셋

    // 테이블(BRAM) 인터페이스
    output wire [ADDR_W-1:0]         table_addr,
    input  wire signed [DATA_W-1:0]  table_data,

    // 제어 인터페이스
    input  wire                      start,      // 1클럭 펄스: 누적기 클리어(새 파티클 시작)
    input  wire                      valid_i,    // 이번 클럭 addr_i가 유효한 룩업 요청
    input  wire                      last_i,     // 이번 요청이 이 파티클의 마지막 빔
    input  wire [ADDR_W-1:0]         addr_i,

    output reg                       done,       // 1클럭 펄스: weight_o 확정
    output reg  signed [ACC_W-1:0]   weight_o
);

    assign table_addr = addr_i;

    // BRAM 읽기 지연(1클럭)에 맞춰 valid/last를 그대로 한 클럭 밀어서 들고 다닌다.
    reg pend_valid, pend_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_o   <= {ACC_W{1'b0}};
            done       <= 1'b0;
            pend_valid <= 1'b0;
            pend_last  <= 1'b0;
        end else begin
            done <= 1'b0;   // 기본값 — 아래서 마지막 빔이면 이번 클럭만 1로

            if (start) begin
                weight_o <= {ACC_W{1'b0}};
            end else if (pend_valid) begin
                weight_o <= weight_o + $signed(table_data);
                if (pend_last)
                    done <= 1'b1;
            end

            pend_valid <= valid_i;
            pend_last  <= last_i;
        end
    end

endmodule
