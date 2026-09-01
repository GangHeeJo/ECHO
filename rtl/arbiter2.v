// 2-input round-robin arbiter — arbiter4(가변량 barrel shifter)보다 훨씬 싼 공정
// 중재의 최소 단위. last_gnt 1비트 하나만으로 "둘 다 요청하면 지난번 안 이긴 쪽"을
// 그대로 골라주면 되므로 시프터가 필요 없음.
module arbiter2(
  input        clk,
  input        rst,
  input  [1:0] req,
  output [1:0] gnt
);
  reg last_gnt; // 0 또는 1: 마지막으로 그랜트된 쪽의 인덱스

  wire prefer1 = last_gnt == 1'b0; // 지난번 0이 이겼으면 이번엔 1을 우대
  assign gnt[1] = req[1] & (prefer1 | ~req[0]);
  assign gnt[0] = req[0] & ~gnt[1];

  always @(posedge clk) begin
    if (rst)
      last_gnt <= 1'b1; // rst 후 첫 경합은 0을 우대(=last_gnt=1로 시작)
    else if (|req)
      last_gnt <= gnt[1];
  end
endmodule
