// 4-input round-robin arbiter (synchronous)
// req[i]=1 : requester i wants the shared bus
// gnt[i]=1 : requester i is granted this cycle (at most one bit set)
// priority rotates to the requester after the last-granted one, so
// no single requester can starve the others under sustained contention.
module arbiter4(
  input        clk,
  input        rst,
  input  [3:0] req,
  output [3:0] gnt
);
  reg [1:0] last_gnt;  // index of requester granted last time

  // rotate the fixed priority order so it starts right after last_gnt
  wire [3:0] req_rot  = {req, req} >> (last_gnt + 1);
  wire [3:0] gnt_rot  = req_rot & (~req_rot + 1'b1); // isolate lowest set bit
  assign gnt = (gnt_rot << (last_gnt + 1)) | (gnt_rot >> (4 - (last_gnt + 1)));

  wire [1:0] gnt_idx = gnt[0] ? 2'd0 : gnt[1] ? 2'd1 : gnt[2] ? 2'd2 : 2'd3;

  always @(posedge clk) begin
    if (rst)
      last_gnt <= 2'd3;
    else if (|req)
      last_gnt <= gnt_idx;
  end
endmodule
