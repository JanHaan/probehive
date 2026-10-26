`default_nettype none
//
// slot_ram
// ========
//
// One of a slot's buffers: a block RAM of 32-bit words with byte enables, a port for each side,
// each in its own clock.  Yosys maps such a RAM onto a RAMB36 only if neither port reads in a
// cycle it writes (NO_CHANGE); otherwise it builds it from LUTs.  LiteX's Verilog writer drops
// Migen's port modes, so the buffers are written here.

module slot_ram #(parameter AW = 10) (
                input wire             clk_a,
                input wire  [AW-1:0]   adr_a,
                input wire  [3:0]      we_a,
                input wire  [31:0]     dat_w_a,
                output reg [31:0] dat_r_a,
                input wire             clk_b,
                input wire  [AW-1:0]   adr_b,
                input wire  [3:0]      we_b,
                input wire  [31:0]     dat_w_b,
                output reg [31:0] dat_r_b
             );
   reg [31:0] mem [0:(1 << AW) - 1];
   integer i, j;

   always @(posedge clk_a) begin
      for (i = 0; i < 4; i = i + 1)
        if (we_a[i]) mem[adr_a][i * 8 +: 8] <= dat_w_a[i * 8 +: 8];
      if (we_a == 0) dat_r_a <= mem[adr_a];
   end

   always @(posedge clk_b) begin
      for (j = 0; j < 4; j = j + 1)
        if (we_b[j]) mem[adr_b][j * 8 +: 8] <= dat_w_b[j * 8 +: 8];
      if (we_b == 0) dat_r_b <= mem[adr_b];
   end
endmodule
