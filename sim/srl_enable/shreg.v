// An 8-bit wide, 8-deep shift register that shifts only while `en` is high,
// read at both ends so the middle six stages can become SRLs.
module shreg(input clk, input en, input [7:0] d, output [7:0] head, output [7:0] tail);
reg [63:0] sr = 64'd0;
always @(posedge clk)
    if (en)
        sr <= {d, sr[63:8]};
assign head = sr[7:0];
assign tail = sr[63:56];
endmodule
