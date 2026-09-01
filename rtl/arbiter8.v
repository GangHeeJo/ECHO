// 8-input round-robin arbiter (synchronous) — arbiter4.v를 8비트로 그대로 일반화.
// E번(8x8=64개로 규모 확장) 검증용.
module arbiter8(
  input        clk,
  input        rst,
  input  [7:0] req,
  output [7:0] gnt
);
  reg [2:0] last_gnt;

  wire [7:0] req_rot = {req, req} >> (last_gnt + 1);
  wire [7:0] gnt_rot = req_rot & (~req_rot + 1'b1);
  assign gnt = (gnt_rot << (last_gnt + 1)) | (gnt_rot >> (8 - (last_gnt + 1)));

  function [2:0] idx8;
    input [7:0] bits;
    begin
      if (bits[0]) idx8 = 3'd0;
      else if (bits[1]) idx8 = 3'd1;
      else if (bits[2]) idx8 = 3'd2;
      else if (bits[3]) idx8 = 3'd3;
      else if (bits[4]) idx8 = 3'd4;
      else if (bits[5]) idx8 = 3'd5;
      else if (bits[6]) idx8 = 3'd6;
      else idx8 = 3'd7;
    end
  endfunction

  always @(posedge clk) begin
    if (rst)
      last_gnt <= 3'd7;
    else if (|req)
      last_gnt <= idx8(gnt);
  end
endmodule
